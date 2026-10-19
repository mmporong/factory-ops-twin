"""CPU contracts only: SDK APIs are substituted, no native application runs."""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace
from typing import Any
import tempfile
import unittest
from unittest.mock import Mock, patch

from factory_ops import isaac45_runtime as facade


class Prim:
    def __init__(self, path, *, scene=False, valid=True):
        self.path, self.scene, self.valid = path, scene, valid

    def IsA(self, schema):
        return self.scene

    def IsValid(self):
        return self.valid

    def GetPath(self):
        return self.path


class Stage:
    def __init__(self):
        self.prims = {p.path:p for p in [Prim('/World/Physics', scene=True), Prim('/World/Carter/chassis_link')]}

    def Traverse(self):
        return list(self.prims.values())

    def GetPrimAtPath(self, path):
        return self.prims.get(path, Prim(path, valid=False))


class NativeRobot:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.dof_names = ['left', 'right', 'caster']
        self.positions = [.12, .34, .56]
        self.pose = ([1., 2., .1], [1., 0., 0., 0.])  # Native wxyz, unchanged.
        self.actions = []

    def get_joint_positions(self):
        return self.positions

    def get_world_pose(self):
        return self.pose

    def apply_wheel_actions(self, action):
        self.actions.append(action)


class World:
    existing: Any = None

    @classmethod
    def instance(cls):
        return cls.existing

    def __init__(self, **kwargs):
        World.existing = self
        self.kwargs = kwargs
        self.added = []
        self.scene = SimpleNamespace(add=self.add)
        self.physics = SimpleNamespace(enable_gpu_dynamics=Mock())
        self.current_time = 0.
        self.current_time_step_index = 0
        self.resets = 0

    def add(self, robot):
        self.added.append(robot)
        return robot

    def get_physics_context(self):
        return self.physics

    def reset(self):
        self.resets += 1


class RuntimeTests(unittest.TestCase):
    def test_tf_message_publisher_keeps_namespaces_and_static_durability(self):
        qos = SimpleNamespace(QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
            DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="latched"),
            HistoryPolicy=SimpleNamespace(KEEP_LAST="last"))
        message = SimpleNamespace(TFMessage=lambda **kwargs: SimpleNamespace(**kwargs))
        def modules(name):
            return {"tf2_msgs.msg": message, "rclpy.qos": qos}[name]
        dynamic_pub, static_pub = Mock(), Mock()
        node = SimpleNamespace(create_publisher=Mock(side_effect=[dynamic_pub, static_pub]))
        with patch.object(facade.importlib, "import_module", side_effect=modules):
            dynamic = facade._TfMessagePublisher(node)
            static = facade._TfMessagePublisher(node, static=True)
        calls = node.create_publisher.call_args_list
        self.assertEqual("tf", calls[0].args[1])
        self.assertEqual(100, calls[0].args[2].depth)
        self.assertEqual("tf_static", calls[1].args[1])
        self.assertEqual("latched", calls[1].args[2].durability)
        first, second = SimpleNamespace(child_frame_id="front"), SimpleNamespace(child_frame_id="rear")
        dynamic.sendTransform(first)
        self.assertEqual([first], dynamic_pub.publish.call_args.args[0].transforms)
        static.sendTransform(first)
        static.sendTransform([first, second])
        self.assertEqual([first, second], static_pub.publish.call_args.args[0].transforms)

    def setUp(self):
        World.existing = None
        self.stage = Stage()
        self.stage_api = SimpleNamespace(open_stage=Mock(return_value=True),
            add_reference_to_stage=Mock(), is_stage_loading=Mock(return_value=False))
        self.bindings = SimpleNamespace(stage=self.stage_api,
            usd=SimpleNamespace(get_context=lambda:SimpleNamespace(get_stage=lambda:self.stage)),
            physics=SimpleNamespace(Scene=object()), World=World, Robot=NativeRobot,
            Action=lambda **kwargs:SimpleNamespace(**kwargs), assets=lambda:'https://installed-assets/4.5/')
        self.app = SimpleNamespace(update=Mock())
        with patch.object(facade, '_load_bindings', return_value=self.bindings):
            self.runtime = facade.create_runtime(self.app)
        self.robot = self.runtime.WheeledRobot('/World/Carter', ['left', 'right'])
        self.addCleanup(lambda:setattr(World, 'existing', None))

    def test_import_is_sdk_and_numpy_free_and_python310_compatible(self):
        path = Path(facade.__file__)
        ast.parse(path.read_text(encoding='utf-8'), feature_version=(3,10))
        import builtins
        original = builtins.__import__
        def guarded(name, *args, **kwargs):
            if name.split('.')[0] in ('omni', 'isaacsim', 'numpy', 'pxr'):
                raise AssertionError('eager SDK import: ' + name)
            return original(name, *args, **kwargs)
        spec = importlib.util.spec_from_file_location('cold_runtime025', path)
        assert spec is not None and spec.loader is not None
        with patch('builtins.__import__', side_effect=guarded):
            spec.loader.exec_module(importlib.util.module_from_spec(spec))

    def test_stage_translation_and_actual_stage_failure(self):
        # A clean runtime, before any robot registration.
        runtime = facade.Runtime45(self.app, self.bindings)
        self.assertEqual(runtime.stage_utils.open_stage('factory.usda'), (True, self.stage))
        runtime.stage_utils.add_reference_to_stage('asset.usd', '/World/Carter')
        self.stage_api.add_reference_to_stage.assert_called_once_with(usd_path='asset.usd', prim_path='/World/Carter')
        self.stage_api.open_stage.return_value = False
        self.assertEqual(runtime.stage_utils.open_stage('bad.usda'), (False, None))
        self.assertFalse(runtime.stage_utils.is_stage_loading())
        with self.assertRaises(RuntimeError):
            self.runtime.stage_utils.open_stage('replacement.usda')

    def test_one_scene_reused_and_robots_registered_only_at_setup(self):
        self.assertIsNone(self.robot._native)
        with self.assertRaises(RuntimeError):self.robot.get_dof_positions()
        world = self.runtime.SimulationManager.setup_simulation(.02)
        self.assertEqual(world.kwargs['physics_prim_path'], '/World/Physics')
        self.assertEqual(world.kwargs['device'], 'cpu')
        self.assertEqual(world.kwargs['rendering_dt'], .02)
        self.assertEqual(len(world.added), 1)
        self.assertFalse(world.added[0].kwargs['create_robot'])
        world.physics.enable_gpu_dynamics.assert_called_once_with(False)
        self.assertIs(self.runtime.SimulationManager.setup_simulation(.02), world)
        self.assertEqual(world.resets, 1)

    def test_multiple_or_missing_scenes_and_foreign_world_are_rejected(self):
        self.stage.prims['/Other'] = Prim('/Other', scene=True)
        with self.assertRaises(ValueError):self.runtime.SimulationManager.setup_simulation(.02)
        self.assertIsNone(World.existing)
        del self.stage.prims['/Other']
        del self.stage.prims['/World/Physics']
        with self.assertRaises(ValueError):self.runtime.SimulationManager.setup_simulation(.02)
        self.stage.prims['/World/Physics'] = Prim('/World/Physics', scene=True)
        World.existing = object()
        with self.assertRaises(RuntimeError):self.runtime.SimulationManager.setup_simulation(.02)

    def test_dt_and_late_registration_do_not_reset_physics(self):
        for dt in (True, 0, -1, float('nan'), float('inf'), None):
            with self.subTest(dt=dt), self.assertRaises(ValueError):self.runtime.SimulationManager.setup_simulation(dt)
        with self.assertRaises(ValueError):self.runtime.SimulationManager.setup_simulation(.02, device='cuda')
        world = self.runtime.SimulationManager.setup_simulation(.02)
        with self.assertRaises(RuntimeError):self.runtime.SimulationManager.setup_simulation(.01)
        with self.assertRaises(RuntimeError):self.runtime.WheeledRobot('/New', ['left'])
        self.assertEqual(world.resets, 1)

    def test_actual_counter_and_encoder_values_are_not_synthesized(self):
        manager = self.runtime.SimulationManager
        self.assertEqual((manager.get_simulation_time(), manager.get_num_physics_steps()), (0., 0))
        world = manager.setup_simulation(.02)
        world.current_time, world.current_time_step_index = .34, 17
        for _ in range(3):
            self.assertEqual((manager.get_simulation_time(), manager.get_num_physics_steps()), (.34, 17))
        self.assertIs(self.robot.get_dof_positions(), self.robot._native.positions)
        positions, orientations = self.robot.get_world_poses()
        self.assertEqual(positions, [[1., 2., .1]])
        self.assertEqual(orientations, [[1., 0., 0., 0.]])

    def test_wheel_vector_is_wrapped_as_action_without_pose_changes(self):
        self.runtime.SimulationManager.setup_simulation(.02)
        vector = [1., -2.]
        self.robot.apply_wheel_actions(vector)
        self.assertIs(self.robot._native.actions[0].joint_velocities, vector)
        self.assertEqual(self.robot.dof_names, ['left', 'right', 'caster'])
        for invalid in ([1.], [float('nan'), 0.], [True, 0.], [[1., 2.]]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):self.robot.apply_wheel_actions(invalid)
        self.assertEqual(self.robot._native.pose, ([1., 2., .1], [1., 0., 0., 0.]))

    def test_asset_path_and_duplicate_registration(self):
        self.assertEqual(self.runtime.asset_url, 'https://installed-assets/4.5' + facade.ASSET_PATH)
        with self.assertRaises(ValueError):self.runtime.WheeledRobot('/World/Carter', ['left', 'right'])
        with self.assertRaises(ValueError):self.runtime.WheeledRobot('/New', 'left')

    def lidar_fixture(self, directory):
        sdk = Path(directory)
        profile = sdk/'exts/isaacsim.sensors.rtx/data/lidar_configs/SLAMTEC/RPLIDAR_S2E.json'
        profile.parent.mkdir(parents=True)
        profile.write_text(json.dumps({'profile':{'scanType':'rotary', 'scanRateBaseHz':10.,
            'reportRateBaseHz':32000., 'nearRangeM':.05, 'farRangeM':30.}}), encoding='utf-8')
        config = {'physics_dt_s':.02, 'lidar_profile':'Example_Rotary_2D', 'lidars':[
            {'name':'front', 'frame_id':'front_scan', 'translation_xyz_m':[.2,0.,.32], 'yaw_rad':0.},
            {'name':'rear', 'frame_id':'rear_scan', 'translation_xyz_m':[-.65,0.,.32], 'yaw_rad':3.14159}]}
        robots = {'carter1':{'definition':{'robot_id':'carter1','namespace':'carter1','prim_path':'/World/Carter'}, 'robot':self.robot}}
        products, writers = [], []
        def create(command, **kwargs):
            self.assertEqual(command, 'IsaacSensorCreateRtxLidar')
            self.assertEqual(kwargs['config'], 'RPLIDAR_S2E')
            prim = Prim(kwargs['path']); self.stage.prims[prim.path] = prim
            return True, prim
        def product(*args, **kwargs):
            handle = SimpleNamespace(destroy=Mock())
            products.append(handle)
            return handle
        def writer(name):
            self.assertEqual(name, facade.LIDAR_WRITER)
            handle = SimpleNamespace(initialize=Mock(), attach=Mock(), detach=Mock())
            writers.append(handle)
            return handle
        modules = {'omni.kit.commands':SimpleNamespace(execute=create),
            'omni.replicator.core':SimpleNamespace(create=SimpleNamespace(render_product=product), writers=SimpleNamespace(get=writer))}
        return config, robots, modules, products, writers, profile

    def setup_lidars(self, config, robots, verify=None):
        author = Mock()
        verifier = verify or Mock(return_value={'parent_frame':'base_link'})
        sensors, metadata = self.runtime.setup_lidars(config, robots, self.stage, object(), object(),
            SimpleNamespace(Quatd=lambda *args:args), self.runtime.SimulationManager, self.app, author, verifier)
        return sensors, metadata, author, verifier

    def test_lidar_mount_topics_profile_proof_and_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, robots, modules, products, writers, profile = self.lidar_fixture(directory)
            with patch.dict(os.environ, ISAAC_PATH=directory), patch.object(facade.importlib, 'import_module', side_effect=modules.__getitem__):
                sensors, metadata, author, verifier = self.setup_lidars(cfg, robots)
            self.assertEqual(author.call_count, 2)
            self.assertEqual(verifier.call_count, 2)
            self.assertEqual(metadata[1]['topic'], '/carter1/scan_rear_raw')
            self.assertEqual(metadata[1]['frame_id'], 'rear_scan')
            self.assertEqual(metadata[0]['profile_sha256'], hashlib.sha256(profile.read_bytes()).hexdigest())
            self.assertEqual(metadata[0]['requested_profile'], 'Example_Rotary_2D')
            self.assertEqual(metadata[0]['horizontal_resolution_deg'], .1125)
            self.assertNotIn('accumulate_outputs', metadata[0])
            writers[0].initialize.assert_called_once_with(topicName='/carter1/scan_front_raw', frameId='front_scan')
            for sensor in sensors:
                sensor.detach_writer(facade.LIDAR_WRITER)
                sensor.detach_writer(facade.LIDAR_WRITER)
            for handle in products:handle.destroy.assert_called_once_with()
            for handle in writers:handle.detach.assert_called_once_with()

    def test_world_reset_precedes_rtx_graph_creation_and_is_not_repeated(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, robots, modules, _, _, _ = self.lidar_fixture(directory)
            events = []
            original_reset = World.reset
            def reset(world):
                original_reset(world)
                events.append('World.reset')
            def after_world(label, function):
                def invoke(*args, **kwargs):
                    self.assertTrue(self.runtime.SimulationManager.ready)
                    self.assertEqual(World.existing.resets, 1)
                    events.append(label)
                    return function(*args, **kwargs)
                return invoke
            commands, rep = modules['omni.kit.commands'], modules['omni.replicator.core']
            commands.execute = after_world('sensor', commands.execute)
            rep.create.render_product = after_world('render_product', rep.create.render_product)
            rep.writers.get = after_world('writer', rep.writers.get)
            with patch.dict(os.environ, ISAAC_PATH=directory), patch.object(World, 'reset', reset), \
                    patch.object(facade.importlib, 'import_module', side_effect=modules.__getitem__):
                sensors, _, _, _ = self.setup_lidars(cfg, robots)
            self.assertEqual(events, ['World.reset', 'sensor', 'render_product', 'writer',
                                      'sensor', 'render_product', 'writer'])
            self.assertEqual(World.existing.resets, 1)
            self.app.update.assert_called_once_with()
            for sensor in sensors: sensor.detach_writer(facade.LIDAR_WRITER)

    def test_lidar_mount_failure_detaches_all_created_writers(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, robots, modules, products, writers, _ = self.lidar_fixture(directory)
            with patch.dict(os.environ, ISAAC_PATH=directory), patch.object(facade.importlib, 'import_module', side_effect=modules.__getitem__):
                with self.assertRaisesRegex(ValueError, 'mount differs'):
                    self.setup_lidars(cfg, robots, Mock(side_effect=ValueError('mount differs')))
            for handle in products:handle.destroy.assert_called_once_with()
            for handle in writers:handle.detach.assert_called_once_with()

    def test_lidar_duplicate_and_nonfinite_mount_fail_before_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, robots, modules, products, _, _ = self.lidar_fixture(directory)
            cfg['lidars'].append(deepcopy(cfg['lidars'][0]))
            with patch.dict(os.environ, ISAAC_PATH=directory):
                with self.assertRaises(ValueError):self.setup_lidars(cfg, robots)
                cfg['lidars'].pop()
                cfg['lidars'][0]['translation_xyz_m'][0] = float('nan')
                with self.assertRaises(ValueError):self.setup_lidars(cfg, robots)
            self.assertEqual(products, [])
            self.assertIsNone(World.existing)

    def test_detach_failure_still_destroys_product_and_reports_failure(self):
        product = SimpleNamespace(destroy=Mock())
        sensor = facade._LidarSensor(product)
        sensor._writer = SimpleNamespace(detach=Mock(side_effect=RuntimeError('detach failed')))
        with self.assertRaisesRegex(RuntimeError, 'detach failed'):sensor.detach_writer(facade.LIDAR_WRITER)
        product.destroy.assert_called_once_with()

    def test_ros_environment_validates_before_mutation_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            library = Path(directory)/'exts/isaacsim.ros2.bridge/humble/lib'
            library.mkdir(parents=True)
            with patch.dict(os.environ, {'ISAAC_PATH':directory,'PATH':'original'}, clear=True):
                with self.assertRaises(RuntimeError):facade.configure_ros_environment(72)
                self.assertEqual(os.environ['PATH'], 'original')
                (library/'rmw_fastrtps_cpp.dll').write_bytes(b'fixture')
                with patch.object(facade.os, 'add_dll_directory', create=True) as add_dll, patch.dict(facade._DLL_HANDLES, clear=True):
                    for _ in range(2):facade.configure_ros_environment(72)
                    self.assertEqual(add_dll.call_count, 1)
                self.assertEqual(os.environ['PATH'].split(os.pathsep).count(str(library)), 1)
                self.assertEqual(os.environ['ROS_DOMAIN_ID'], '72')
                self.assertEqual(os.environ['ROS_DISTRO'], 'humble')
                with self.assertRaises(ValueError):facade.configure_ros_environment(True)


if __name__ == '__main__':
    unittest.main()
