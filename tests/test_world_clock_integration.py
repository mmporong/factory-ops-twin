"""Clock route and update order fixtures; no actual SDK/DDS delivery claim."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from factory_ops.world_clock import WorldClockPublisher
from scripts.isaac.g2_scene import _setup_worker_clock, _update_physics_app


class WorldClockIntegrationTests(unittest.TestCase):
    def test_physx_chooses_one_factory_and_legacy_routes_keep_graph(self):
        stage, app, manager = object(), object(), object()
        received=[]
        owner=object()
        selected=(object(),object(),received,owner)
        with patch('factory_ops.world_clock.create_ros_world_clock',return_value=selected) as factory, \
                patch('scripts.isaac.g2_scene._clock_graph') as graph:
            result=_setup_worker_clock(stage,app,manager,local_sdk45=True,lidar_backend='physx_planar')
            self.assertIs(selected,result)
            factory.assert_called_once_with(app,manager)
            graph.assert_not_called()
        for local in (False,True):
            old=(object(),object(),received)
            with patch('factory_ops.world_clock.create_ros_world_clock') as factory, \
                    patch('scripts.isaac.g2_scene._clock_graph',return_value=old) as graph:
                self.assertEqual((*old,None),_setup_worker_clock(stage,app,manager,local_sdk45=local,lidar_backend='rtx_flat_scan'))
                factory.assert_not_called()
                graph.assert_called_once_with(stage,app)

    def test_wire_clock_samples_after_physics_update_and_paused_frame_does_not_advance(self):
        world=SimpleNamespace(time=.1,frame=6)
        sent=[]
        owner=WorldClockPublisher(lambda sec,ns:sent.append((sec,ns)),lambda:world.time,lambda:world.frame)
        def advance():
            world.time=.2
            world.frame=12
        app=SimpleNamespace(update=Mock(side_effect=advance))
        _update_physics_app(app,owner)
        self.assertEqual([(0,200000000)],sent)
        self.assertEqual(12,owner.report()['last_published_physics_frame'])
        app.update.side_effect=None
        _update_physics_app(app,owner)
        self.assertEqual(1,len(sent))
        self.assertEqual(1,owner.report()['duplicate_count'])
        legacy=SimpleNamespace(update=Mock())
        _update_physics_app(legacy,None)
        legacy.update.assert_called_once_with()


if __name__=='__main__':
    unittest.main()
