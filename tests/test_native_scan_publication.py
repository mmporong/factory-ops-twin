"""CPU boundary check for native timestamps reaching the actual ROS message adapter."""
from array import array
from copy import deepcopy
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from scripts.isaac.g2_scene import _setup_native_lidar_publication


def fake_ros_modules():
    qos = ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **kw: SimpleNamespace(**kw)
    qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT='best_effort')
    messages = ModuleType('sensor_msgs.msg')
    messages.LaserScan = lambda: SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()))
    return {'rclpy.qos': qos, 'sensor_msgs.msg': messages}


class NativeScanPublicationTests(unittest.TestCase):
    def test_topics_frames_native_time_and_post_capture_new_scan_are_preserved(self):
        payload = {'timeStamp': 2.125, 'numRows': 1, 'numCols': 3,
            'azimuthRange': [-1., 1.], 'horizontalResolution': 1.,
            'rotationRate': 10., 'depthRange': [.05, 30.], 'linearDepthData': [1., 2., 3.]}
        sensors, metadata, robots = [], [], {}
        for robot in ('carter1', 'carter2'):
            node = SimpleNamespace(create_publisher=Mock(side_effect=lambda *args: Mock()))
            robots[robot] = {'node': node, 'ros': {'resources': []}}
            for side in ('front', 'rear'):
                sensors.append(SimpleNamespace(read_native_scan=Mock(return_value=deepcopy(payload))))
                metadata.append({'robot_id': robot, 'name': side, 'frame_id': side+'_scan'})
        with patch.dict(sys.modules, fake_ros_modules()):
            publish = _setup_native_lidar_publication(sensors, metadata, robots)
            publish()
            publish()  # Camera work can leave an unchanged native buffer; do not republish it.
            sensors[0].read_native_scan.return_value['timeStamp'] = 2.225
            publish()
            sensors[0].read_native_scan.return_value['timeStamp'] = 2.1
            with self.assertRaisesRegex(ValueError, 'native scan carter1/front.*regressed'):
                publish()  # No opt-in: strict failure is retained.
        for robot, runtime in robots.items():
            self.assertEqual(['scan_front_raw', 'scan_rear_raw'],
                [call.args[1] for call in runtime['node'].create_publisher.call_args_list])
            for side, publisher in zip(('front', 'rear'), runtime['ros']['resources']):
                expected = 2 if robot == 'carter1' and side == 'front' else 1
                self.assertEqual(expected, publisher.publish.call_count)
                message = publisher.publish.call_args_list[0].args[0]
                self.assertEqual((2, 125000000), (message.header.stamp.sec, message.header.stamp.nanosec))
                self.assertEqual(side+'_scan', message.header.frame_id)
                self.assertEqual([1., 2., 3.], message.ranges)
                self.assertEqual(expected, runtime['ros']['native_scan_publication'][side]['published_count'])

    def test_opt_in_world_clock_guard_preserves_rejection_and_native_ros_headers(self):
        native = {'timeStamp': 200.350010449, 'numRows': 1, 'numCols': 3,
            'azimuthRange': [-1., 1.], 'horizontalResolution': 1.,
            'rotationRate': 10., 'depthRange': [.05, 30.], 'linearDepthData': array('f', [1., -1., 3.])}
        old = {**native, 'timeStamp': 200.3333437821436}
        new = {**native, 'timeStamp': 200.450010449}
        sensor = SimpleNamespace(read_native_scan=Mock(side_effect=[native, old, new]),
                                 _product=SimpleNamespace(path='/Render/Carter2Rear'))
        metadata = [{'robot_id': 'carter2', 'name': 'rear', 'frame_id': 'rear_scan',
                     'rotation_frequency_hz': 10., 'prim_path': '/World/carter2/chassis/lidar_rear'}]
        publisher = Mock()
        robots = {'carter2': {'node': SimpleNamespace(create_publisher=Mock(return_value=publisher)),
                             'ros': {'resources': []}}}
        simulation_time = Mock(side_effect=[200.43334378674626, 200.43334378674626, 200.5])
        physics_frame = Mock(side_effect=[12026, 12026, 12030])
        with tempfile.TemporaryDirectory() as folder, patch.dict(sys.modules, fake_ros_modules()):
            output = Path(folder)
            publish = _setup_native_lidar_publication([sensor], metadata, robots,
                admission={'run_id': 'a' * 32, 'output': output,
                           'physics_dt_s': 1. / 60,
                           'simulation_time': simulation_time, 'physics_frame': physics_frame})
            publish()
            publish()
            state = robots['carter2']['ros']['native_scan_publication']['rear']
            self.assertEqual(('rejected', 1, 1, 0, 0), tuple(state[k] for k in
                ('status', 'rejected_count', 'published_count', 'duplicate_count', 'warmup_count')))
            self.assertEqual(1, publisher.publish.call_count)
            journal = json.loads((output / 'native-scan-anomalies/events.jsonl').read_text())
            self.assertEqual('/Render/Carter2Rear', journal['identity']['render_product_path'])
            self.assertEqual(metadata[0]['prim_path'], journal['identity']['prim_path'])
            self.assertEqual(('a' * 32, 'carter2', 'rear', 12026),
                (journal['identity']['run_id'], journal['identity']['robot_id'], journal['identity']['side'], journal['physics_frame']))
            self.assertFalse(journal['atomic_sdk_frame_consistency_verified'])
            snapshot_path = output / journal['snapshot']['path']
            snapshot = json.loads(snapshot_path.read_text())
            depth = snapshot['payload']['linearDepthData']
            self.assertEqual(old['linearDepthData'].tobytes(), (snapshot_path.parent / depth['file']).read_bytes())
            publish()
            self.assertEqual(2, publisher.publish.call_count)
            self.assertEqual([(200, 350010449), (200, 450010449)],
                [(call.args[0].header.stamp.sec, call.args[0].header.stamp.nanosec)
                 for call in publisher.publish.call_args_list])
            self.assertEqual(3, simulation_time.call_count)
            self.assertEqual(3, physics_frame.call_count)


if __name__ == '__main__':
    unittest.main()
