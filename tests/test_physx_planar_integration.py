"""ROS wire and journal boundaries only; no SDK or navigation success claim."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from factory_ops.fleet_transport import prepare_bundle
from factory_ops.physx_planar import SCHEMA_NAME, SENSOR_BACKEND, TIME_SOURCE
from scripts.isaac.g2_scene import _setup_native_lidar_publication
from scripts.isaac.local45_fleet4_transport_scene import Local45FleetLifecycle
from tests.test_native_scan_publication import fake_ros_modules


class PhysxIntegrationTests(unittest.TestCase):
    def test_eight_ros_publishers_use_world_snapshot_and_instantaneous_timing(self):
        snapshot = dict(schema_name=SCHEMA_NAME, sensor_backend=SENSOR_BACKEND, time_source=TIME_SOURCE,
            native_timestamp=None, acquired_world_time_s=12.345, acquired_physics_frame=738,
            num_cols_ticked=720, linear_depth_data=[[0.1 if i == 0 else 4.] for i in range(720)],
            intensity_data=[[0 if i == 5 else 255] for i in range(720)],
            azimuth_rad=[-math.pi + i * math.pi / 360 for i in range(720)], zenith_rad=[0.])
        sensors, metadata, robots = [], [], {}
        for robot in ('carter1', 'carter2', 'carter3', 'carter4'):
            robots[robot] = {'node': SimpleNamespace(create_publisher=Mock(side_effect=lambda *a: Mock())),
                             'ros': {'resources': []}}
            for side in ('front', 'rear'):
                sensors.append(SimpleNamespace(read_planar_snapshot=Mock(return_value=snapshot),
                    read_native_scan=Mock(side_effect=AssertionError('must not read RTX'))))
                metadata.append({'robot_id':robot,'name':side,'frame_id':side+'_scan'})
        with patch.dict(sys.modules, fake_ros_modules()):
            publish = _setup_native_lidar_publication(sensors, metadata, robots, backend='physx_planar',
                admission={'simulation_time':Mock(side_effect=AssertionError('must not use RTX guard'))})
            publish()
            publish()
            newer = {**snapshot,'acquired_world_time_s':12.445,'acquired_physics_frame':744}
            for sensor in sensors:
                sensor.read_planar_snapshot.return_value = newer
            publish()
        for runtime in robots.values():
            self.assertEqual(['scan_front_raw','scan_rear_raw'],
                [c.args[1] for c in runtime['node'].create_publisher.call_args_list])
            for publisher in runtime['ros']['resources']:
                self.assertEqual(2,publisher.publish.call_count)
                first, second = [c.args[0] for c in publisher.publish.call_args_list]
                self.assertEqual((12,345000000),(first.header.stamp.sec,first.header.stamp.nanosec))
                self.assertEqual(0.,first.time_increment)
                self.assertAlmostEqual(.1,second.scan_time)
                self.assertEqual(720,len(first.ranges))
                self.assertEqual(.1,first.ranges[0])
                self.assertEqual(math.inf,first.ranges[5])

    def test_physx_observations_use_separate_journal_and_reject_wrong_backend(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);bundle=root/'bundle';prepare_bundle(bundle)
            lifecycle=Local45FleetLifecycle(bundle,'a'*32,lidar_backend='physx_planar')
            metadata=[dict(robot_id=d['robot_id'],name=side,
                prim_path=d['prim_path']+'/chassis_link/lidar_'+side,
                topic='/'+d['namespace']+'/scan_'+side+'_raw',frame_id=side+'_scan',
                sensor_backend=SENSOR_BACKEND,profile_sha256='b'*64,near_range_m=.1,far_range_m=30.)
                for d in lifecycle.registry.definitions for side in ('front','rear')]
            sensors=[SimpleNamespace(open_native_observer=Mock(return_value=SimpleNamespace(
                read=Mock(return_value={'schema_name':'local45-physx-planar-observation','time_source':TIME_SOURCE}),close=Mock()))) for _ in metadata]
            bad=deepcopy(metadata);bad[0]['sensor_backend']='rtx_flat_scan'
            with self.assertRaisesRegex(ValueError,'selected observation backend'):
                lifecycle.register_lidar_sensors(sensors,bad,SimpleNamespace(is_playing=lambda:True))
            # Failed partial registration closes opened handles; start a clean owner.
            lifecycle=Local45FleetLifecycle(bundle,'a'*32,lidar_backend='physx_planar')
            lifecycle.register_lidar_sensors(sensors,metadata,SimpleNamespace(is_playing=lambda:True))
            lifecycle.output=root
            lifecycle._record_lidar_acquisition('periodic_1hz',12.345,738)
            rows=[json.loads(s) for s in (root/'physx-planar-observations.jsonl').read_text().splitlines()]
            self.assertEqual(8,len(rows))
            self.assertTrue(all(r['time_source']==TIME_SOURCE for r in rows))
            self.assertFalse((root/'local45-lidar-observations.jsonl').exists())
            lifecycle.cleanup_render_products()


if __name__ == '__main__':
    unittest.main()
