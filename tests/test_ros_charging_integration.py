"""ROS clock/odometry ordering at the optional virtual charging boundary."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import math
from pathlib import Path
import json
import tempfile

from factory_ops.fleet_execution import make_chunk
from factory_ops.fleet_routing import RouteBook
from factory_ops.navigation import GoalPose
from scripts.ros.g3_mission import (navigation_leg_goal, observe_charging_odometry,
    publish_charging_state, queue_charging_odometry)


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

    def test_prior_causal_pose_is_retained_when_latest_pose_is_ahead_of_queued_odometry(self):
        self.runtime["pose"] = {"x_m":1., "y_m":2., "yaw_rad":0., "stamp_s":9.95}
        observe_charging_odometry(self.charging, "run", "carter4", self.runtime, {"dock_id":"CH1"}, 9.96)
        self.runtime["pose"] = {"x_m":1., "y_m":2., "yaw_rad":0., "stamp_s":10.05}
        observe_charging_odometry(self.charging, "run", "carter4", self.runtime, {"dock_id":"CH1"}, 10.1)
        pose = self.charging.observe.call_args.kwargs["pose"]
        self.assertIsNotNone(pose)
        self.assertEqual(pose["stamp_s"], 9.95)

    def test_delayed_encoder_batch_can_accumulate_observed_geometric_dwell(self):
        from factory_ops.energy_runtime import EnergyRuntime, GEOMETRIC_CONTACT_SOURCE
        from factory_ops.ros_charging import RosChargingRuntime
        profile = json.loads((Path(__file__).resolve().parents[1]/
            "configs/factory-fleet4/energy-profile.json").read_text())
        energy = EnergyRuntime("run", profile, {"carter4":.5}, virtual_charging_source=GEOMETRIC_CONTACT_SOURCE)
        charging = RosChargingRuntime(energy, {"CH1":{"anchor":"P1", "x_m":1., "y_m":2., "yaw_rad":0.}},
            contact_dwell_s=1., position_tolerance_m=.1, yaw_tolerance_rad=.1, resume_soc=.8)
        charging.observe("run", "carter4", .8, 0., 0., loaded=False, observed_at_s=.8)
        reservation = charging.reserve("run", "carter4", carrying_payload=False, observed_at_s=.9)
        charging.arrived("run", "carter4", dock_id="CH1", anchor="P1", goal_id="1"*32,
            navigation_succeeded=True, stamp_s=1., observed_at_s=1.)
        runtime = {"pose":{"x_m":1.,"y_m":2.,"yaw_rad":0.,"stamp_s":.9}, "charging_odom_pending":deque()}
        observe_charging_odometry(charging, "run", "carter4", runtime, reservation, .9)
        for index in range(11, 41):
            now, encoder_stamp = index/10, (index-1)/10
            runtime["pose"] = {"x_m":1.,"y_m":2.,"yaw_rad":0.,"stamp_s":now}
            runtime["charging_odom_pending"].append({"stamp_s":encoder_stamp,
                "linear_m_s":0.,"angular_rad_s":0.,"loaded":False})
            observe_charging_odometry(charging, "run", "carter4", runtime, reservation, now)
        self.assertEqual(charging.snapshot("run", observed_at_s=4.)["robots"]["carter4"]["charging"]["phase"], "charging")
        self.assertGreater(energy.snapshot()["robots"]["carter4"]["charged_wh"], 0.)

    def test_callback_batch_larger_than_drain_rounds_is_consumed_in_stamp_order(self):
        self.runtime["charging_odom_pending"].clear()
        for index in range(96):
            stamp = index/10
            queue_charging_odometry(self.charging, "run", "carter4", self.runtime, None,
                {"stamp_s":stamp, "linear_m_s":.1, "angular_rad_s":0., "loaded":False}, stamp)
        self.assertEqual([], list(self.runtime["charging_odom_pending"]))
        self.assertEqual(list(range(96)),
            [round(call.args[2]*10) for call in self.charging.observe.call_args_list])

    def test_uncausal_callback_batch_keeps_original_samples_and_overflows_at_bound(self):
        self.runtime["charging_odom_pending"].clear()
        for index in range(64):
            queue_charging_odometry(self.charging, "run", "carter4", self.runtime, None,
                {"stamp_s":10.+index/10, "linear_m_s":.1, "angular_rad_s":0., "loaded":False}, 9.)
        with self.assertRaisesRegex(RuntimeError, "virtual charging odometry queue overflow"):
            queue_charging_odometry(self.charging, "run", "carter4", self.runtime, None,
                {"stamp_s":16.4, "linear_m_s":.1, "angular_rad_s":0., "loaded":False}, 9.)
        self.charging.observe.assert_not_called()
        self.assertEqual([10.+index/10 for index in range(64)],
            [sample["stamp_s"] for sample in self.runtime["charging_odom_pending"]])

    def test_projection_refreshes_soc_without_repeating_lifecycle_events(self):
        state = {"owners":{}, "robots":{"carter4":{"charging":None,
            "energy":{"battery_pct":50.,"observed_at_s":1.}}}}
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            revision, published, changed = publish_charging_state(output, state, 1., None, -math.inf)
            self.assertTrue(changed)
            state["robots"]["carter4"]["energy"].update(battery_pct=51., observed_at_s=1.5)
            revision, published, changed = publish_charging_state(output, state, 1.5, revision, published)
            self.assertFalse(changed)
            self.assertEqual(json.loads((output/"charging-state.json").read_text())["robots"]["carter4"]["energy"]["battery_pct"], 50.)
            state["robots"]["carter4"]["energy"]["observed_at_s"] = 2.
            revision, published, changed = publish_charging_state(output, state, 2., revision, published)
            self.assertFalse(changed)
            fresh = json.loads((output/"charging-state.json").read_text())["robots"]["carter4"]["energy"]
            self.assertEqual((fresh["battery_pct"], fresh["observed_at_s"]), (51., 2.))
            state["robots"]["carter4"]["charging"] = {"dock_id":"CH1", "phase":"approach"}
            revision, published, changed = publish_charging_state(output, state, 2.1, revision, published)
            self.assertTrue(changed)
            self.assertEqual(published, 2.1)


if __name__ == "__main__":
    unittest.main()
