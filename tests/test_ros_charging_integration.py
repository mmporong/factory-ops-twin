"""ROS clock/odometry ordering at the optional virtual charging boundary."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import math
from pathlib import Path

from factory_ops.fleet_execution import make_chunk
from factory_ops.fleet_routing import RouteBook
from factory_ops.navigation import GoalPose
from scripts.ros.g3_mission import navigation_leg_goal, observe_charging_odometry


class ChargingIntegrationTests(unittest.TestCase):
    def test_charging_staging_uses_route_endpoint_and_tangent_without_a_station_goal(self):
        configs = Path(__file__).resolve().parents[1]/'configs/factory-fleet4'
        book = RouteBook.from_files(configs/'layout.json', configs/'g3-routing.json')
        plan = book.plan('P1','M_LOW',closed=False)
        goal = navigation_leg_goal(plan, {}, allow_transit=True)
        self.assertEqual(('map',12.,5.,math.pi/2), (goal.frame_id,goal.x,goal.y,goal.yaw))
        chunk = make_chunk(book,'P1','M_LOW',closed=False,final_goal=goal)
        self.assertTrue(chunk.final)
        self.assertEqual('M_LOW',chunk.goal)
        with self.assertRaisesRegex(ValueError, 'no authored station goal'):
            navigation_leg_goal(plan, {})
        authored = GoalPose('map',12.,5.,0.)
        self.assertIs(authored, navigation_leg_goal(plan, {'M_LOW':authored}, allow_transit=True))

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
