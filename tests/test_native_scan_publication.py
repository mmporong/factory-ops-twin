"""CPU boundary check for native timestamps reaching the actual ROS message adapter."""
from copy import deepcopy
from types import ModuleType, SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

from scripts.isaac.g2_scene import _setup_native_lidar_publication


class NativeScanPublicationTests(unittest.TestCase):
    def test_topics_frames_native_time_and_post_capture_new_scan_are_preserved(self):
        qos = ModuleType('rclpy.qos')
        qos.QoSProfile = lambda **kw: SimpleNamespace(**kw)
        qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT='best_effort')
        messages = ModuleType('sensor_msgs.msg')
        messages.LaserScan = lambda: SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()))
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
        with patch.dict(sys.modules, {'rclpy.qos': qos, 'sensor_msgs.msg': messages}):
            publish = _setup_native_lidar_publication(sensors, metadata, robots)
            publish()
            publish()  # Camera work can leave an unchanged native buffer; do not republish it.
            sensors[0].read_native_scan.return_value['timeStamp'] = 2.225
            publish()
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


if __name__ == '__main__':
    unittest.main()
