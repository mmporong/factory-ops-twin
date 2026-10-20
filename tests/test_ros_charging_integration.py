"""ROS clock/odometry ordering at the optional virtual charging boundary."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from scripts.ros.g3_mission import observe_charging_odometry


class ChargingIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.charging = SimpleNamespace(energy=SimpleNamespace(charging_limits={"max_observation_gap_s": .3}),
                                       observe=Mock(return_value={"energy": {}}))
        self.sample = {"stamp_s": 10., "linear_m_s": -.1, "angular_rad_s": .2, "loaded": False}
        self.runtime = {"charging_odom_pending": deque([self.sample]), "pose": None}

    def test_future_packet_waits_for_clock_without_timestamp_rewrite_or_double_consume(self):
        self.assertIsNone(observe_charging_odometry(self.charging, "run", "carter4", self.runtime, None, 9.9))
        self.charging.observe.assert_not_called()
        self.assertEqual(1, len(self.runtime["charging_odom_pending"]))
        observe_charging_odometry(self.charging, "run", "carter4", self.runtime, None, 10.)
        self.charging.observe.assert_called_once_with("run", "carter4", 10., -.1, .2, loaded=False,
                                                     observed_at_s=10., dock_id=None, pose=None)
        observe_charging_odometry(self.charging, "run", "carter4", self.runtime, None, 10.1)
        self.assertEqual(1, self.charging.observe.call_count)

    def test_newer_or_stale_pose_is_not_used_as_docking_geometry(self):
        for pose_time in (10.01, 9.5):
            with self.subTest(pose_time=pose_time):
                self.runtime["charging_odom_pending"] = deque([self.sample])
                self.runtime["pose"] = {"x_m":1., "y_m":2., "yaw_rad":0., "stamp_s":pose_time}
                observe_charging_odometry(self.charging, "run", "carter4", self.runtime, {"dock_id":"CH1"}, 10.1)
                self.assertIsNone(self.charging.observe.call_args.kwargs["pose"])

    def test_fresh_estimate_preserves_identity_and_original_stamp(self):
        self.runtime["pose"] = {"x_m":1., "y_m":2., "yaw_rad":0., "stamp_s":9.95}
        observe_charging_odometry(self.charging, "run", "carter4", self.runtime, {"dock_id":"CH1"}, 10.1)
        pose = self.charging.observe.call_args.kwargs["pose"]
        self.assertEqual(("run", "carter4", "map", 9.95),
                         (pose["run_id"], pose["robot_id"], pose["frame_id"], pose["stamp_s"]))


if __name__ == "__main__":
    unittest.main()
