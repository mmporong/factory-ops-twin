from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

from factory_ops.layout import Layout
from factory_ops.traffic import TrafficController
from factory_ops.traffic_observation import project_observation
from scripts.ros.g2_mission import (
    ActionEvidence,
    CANCELED,
    MovingChannelFault,
    PoseFreshnessGate,
    ScenarioMachine,
    authored_route_length,
    build_namespaced_params,
    evidence_subscribers_ready,
    filtered_scan_is_recordable,
    make_authorization,
    mask_scan_ranges,
    named_subscriber_topics_ready,
    publish_scan_fanout,
    scan_recording_topics,
)


ROOT = Path(__file__).resolve().parents[1]
TRAFFIC = ROOT / "configs/factory-v3/traffic.json"
G2 = ROOT / "configs/factory-v3/g2.json"
SCENARIOS = ROOT / "configs/factory-v3/g2-scenarios.json"
RADIUS = 1.137109


def observe(traffic: TrafficController, robot: str, xy: tuple[float, float], now: float) -> None:
    traffic.update_observation(
        robot, pose_xy=xy, pose_time=now, heartbeat_time=now,
        **project_observation(traffic, xy, RADIUS), now=now,
    )


class G2MissionTests(unittest.TestCase):
    def test_evidence_readiness_requires_every_publisher_subscription(self) -> None:
        class Publisher:
            def __init__(self, count: int) -> None:
                self.count = count

            def get_subscription_count(self) -> int:
                return self.count

        first = Publisher(1)
        second = Publisher(0)
        self.assertFalse(evidence_subscribers_ready(()))
        self.assertFalse(evidence_subscribers_ready((first, second)))
        second.count = 1
        self.assertTrue(evidence_subscribers_ready((first, second)))

    def test_raw_topic_barrier_and_scan_epoch_reject_pre_recorder_queue(self) -> None:
        class Info:
            def __init__(self, name: str) -> None:
                self.node_name = name

        class Node:
            def __init__(self) -> None:
                self.subscriptions = {
                    "/carter1/scan_front_raw": [Info("g2_adapter_carter1"), Info("rosbag2_recorder")],
                    "/carter1/scan_rear_raw": [Info("g2_adapter_carter1")],
                }

            def get_subscriptions_info_by_topic(self, topic: str) -> list[Info]:
                return self.subscriptions.get(topic, [])

        node = Node()
        topics = ("/carter1/scan_front_raw", "/carter1/scan_rear_raw")
        self.assertFalse(named_subscriber_topics_ready(node, topics, "rosbag2_recorder"))
        node.subscriptions[topics[1]].append(Info("rosbag2_recorder"))
        self.assertTrue(named_subscriber_topics_ready(node, topics, "/rosbag2_recorder"))
        self.assertFalse(filtered_scan_is_recordable(False, 2_400_000_000, 2_500_000_000))
        self.assertFalse(filtered_scan_is_recordable(True, 2_400_000_000, 2_300_000_000))
        self.assertFalse(filtered_scan_is_recordable(True, 2_400_000_000, 2_400_000_000))
        self.assertTrue(filtered_scan_is_recordable(True, 2_400_000_000, 2_500_000_000))

    def test_filtered_scan_fanout_preserves_one_message_for_costmap_and_amcl(self) -> None:
        class Publisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message: object) -> None:
                self.messages.append(message)

        message = object()
        costmap = Publisher()
        localization = Publisher()
        publish_scan_fanout(message, (costmap, localization))
        self.assertIs(costmap.messages[0], message)
        self.assertIs(localization.messages[0], message)
        with self.assertRaisesRegex(ValueError, "costmap and localization"):
            publish_scan_fanout(message, (costmap,))
        self.assertEqual(scan_recording_topics(("carter1",)), (
            "/carter1/scan_front_raw", "/carter1/scan_rear_raw",
            "/carter1/scan_front", "/carter1/scan_rear", "/carter1/scan_localization",
        ))
        self.assertFalse(filtered_scan_is_recordable(True, None, 2_500_000_000))
        for sec, nano in ((1, 1), (123, 999_999_999), (26, 772_713_542), (374, 663_036_237)):
            total_ns = sec * 1_000_000_000 + nano
            self.assertFalse(filtered_scan_is_recordable(True, total_ns, sec * 1_000_000_000 + nano))
        self.assertNotEqual(26_772_713_542 / 1e9, 26 + 772_713_542 / 1e9)
        self.assertGreater(374 + 663_036_237 / 1e9, 374_663_036_237 / 1e9)

    def test_scan_mask_keeps_only_sensor_frame_forward_half_plane(self) -> None:
        ranges = list(range(1, 10))
        masked = mask_scan_ranges(-math.pi, math.pi, math.pi / 4, ranges)
        self.assertTrue(all(math.isinf(masked[index]) for index in (0, 1, 7, 8)))
        self.assertEqual(masked[2:7], [3.0, 4.0, 5.0, 6.0, 7.0])
        nan_ranges = [1.0] * 9
        nan_ranges[4] = math.nan
        nan_ranges[0] = math.nan
        nan_masked = mask_scan_ranges(-math.pi, math.pi, math.pi / 4, nan_ranges)
        self.assertTrue(math.isnan(nan_masked[4]))
        self.assertTrue(math.isinf(nan_masked[0]))
        with self.assertRaisesRegex(ValueError, "length"):
            mask_scan_ranges(-math.pi, math.pi, math.pi / 4, ranges[:-2])
        with self.assertRaisesRegex(ValueError, "metadata"):
            mask_scan_ranges(-math.pi, math.pi, 0.0, ranges)

    def test_scan_mask_accepts_rtx_upper_exclusive_float32_topology(self) -> None:
        angle_min = -3.1415927410125732
        angle_max = 3.1415927410125732
        angle_increment = 0.001745329238474369
        ranges = [-1.0] * 3600
        masked = mask_scan_ranges(angle_min, angle_max, angle_increment, ranges)
        self.assertEqual(len(masked), 3600)
        self.assertEqual(masked[1800], -1.0)
        self.assertTrue(math.isinf(masked[0]))
        self.assertEqual(mask_scan_ranges(-1.0, 1.0, 1.0, [1.0, 2.0, 3.0]), [1.0, 2.0, 3.0])
        for wrong_length in (3500, 3602):
            with self.assertRaisesRegex(ValueError, "inclusive/exclusive"):
                mask_scan_ranges(angle_min, angle_max, angle_increment, [-1.0] * wrong_length)

    def test_nav2_template_is_relative_namespaced_and_declares_jazzy_through_poses(self) -> None:
        text = (ROOT / "configs/factory-v3/nav2-g2.yaml").read_text(encoding="utf-8")
        self.assertIn("scan_topic: scan_localization", text)
        self.assertIn("odom_topic: odom", text)
        self.assertIn("navigators: [navigate_to_pose, navigate_through_poses]", text)
        self.assertIn("plugin: nav2_bt_navigator::NavigateThroughPosesNavigator", text)
        self.assertIn("max_angular_accel: 2.0", text)
        self.assertIn("movement_time_allowance: 30.0", text)
        self.assertNotIn("velocity_smoother", text)
        self.assertIn("observation_sources: front rear", text)
        self.assertIn("topic: scan_front", text)
        self.assertIn("topic: scan_rear", text)
        self.assertEqual(text.count("inf_is_valid: false"), 4)
        self.assertNotIn("inf_is_valid: true", text)

    def test_prepared_params_duplicate_two_namespaces_and_qualified_radius(self) -> None:
        scenario = json.loads(SCENARIOS.read_text(encoding="utf-8"))["scenarios"]["F1"]
        node = lambda: {"ros__parameters": {}}
        def costmap(name: str) -> dict:
            return {name: {"ros__parameters": {
                "inflation_layer": {}, "static_layer": {},
                "obstacle_layer": {"front": {}, "rear": {}},
            }}}
        template = {"/**": node(), "map_server": node(), "navigation_map_server": node(),
                    "amcl": node(), "local_costmap": costmap("local_costmap"),
                    "global_costmap": costmap("global_costmap")}
        radii = {"carter1": 1.137109, "carter2": 1.137109}
        result = build_namespaced_params(template, scenario, radii,
                                         physical_map="physical.yaml", navigation_map="navigation.yaml")
        self.assertEqual(set(result), {"/**", "carter1", "carter2"})
        self.assertAlmostEqual(radii["carter1"], 1.137109)
        self.assertEqual(result["carter1"]["amcl"]["ros__parameters"]["initial_pose"]["x"], 22.0)
        self.assertEqual(result["carter2"]["amcl"]["ros__parameters"]["initial_pose"]["x"], 32.0)
        for robot in ("carter1", "carter2"):
            for costmap_name in ("local_costmap", "global_costmap"):
                params = result[robot][costmap_name][costmap_name]["ros__parameters"]
                self.assertEqual(params["static_layer"]["map_topic"], f"/{robot}/navigation_map")
                self.assertEqual(params["obstacle_layer"]["front"]["topic"], f"/{robot}/scan_front")
                self.assertEqual(params["obstacle_layer"]["rear"]["topic"], f"/{robot}/scan_rear")
        self.assertNotEqual(result["carter1"], result["carter2"])

    def test_action_evidence_requires_acceptance_uuid_and_terminal_cancel_status(self) -> None:
        goal = ActionEvidence("carter2", "blocked", "navigate_to_pose")
        uuid = "5db4143f-fbcc-4ea0-9c89-12dbd9683920"
        goal.accepted(uuid, 10.0)
        goal.request_cancel(15.0)
        with self.assertRaisesRegex(RuntimeError, "GoalStatus"):
            goal.terminal(4, 0, 15.1)
        goal.terminal(CANCELED, 0, 15.2)
        self.assertEqual(goal.state, "canceled")
        self.assertEqual(goal.goal_uuid, uuid)

    def test_navigate_lease_requires_actual_accepted_goal_and_reservation(self) -> None:
        uuid = "5db4143f-fbcc-4ea0-9c89-12dbd9683920"
        lease = make_authorization(
            run_id="run", scenario_id="F1", robot_id="carter1", sequence=1,
            now_s=2.0, ttl_s=0.5, mode="navigate", decision_event_seq=4,
            route_id="carter1_cross", goal_uuid=uuid, reservation_id=3,
            resources=["Z_UPPER"], exit_id="Z_UPPER:east",
        )
        self.assertEqual(lease["valid_until_sim_time_s"], 2.5)
        self.assertEqual(lease["scope"]["goal_uuid"], uuid)
        with self.assertRaisesRegex(ValueError, "accepted goal"):
            make_authorization(run_id="run", scenario_id="F1", robot_id="carter1", sequence=2,
                               now_s=2.1, ttl_s=0.5, mode="navigate", decision_event_seq=5)

    def test_scenario_machine_rejects_skipped_operational_evidence(self) -> None:
        machine = ScenarioMachine("F3")
        machine.transition("c1_reserved")
        with self.assertRaisesRegex(RuntimeError, "invalid"):
            machine.transition("cycle_observed")
        machine.transition("blocked_goal_active")
        machine.transition("cycle_observed")
        self.assertEqual(machine.history[-1], "cycle_observed")

    def test_moving_channel_fault_exposes_link_edges_without_dispatcher_end_time(self) -> None:
        fault = MovingChannelFault("carter2", 30.0)
        self.assertTrue(fault.link_up(10.0))
        fault.trigger(10.0)
        self.assertEqual(fault.edge(10.0), "link_down")
        self.assertFalse(fault.link_up(39.9))
        self.assertIsNone(fault.edge(39.9))
        self.assertTrue(fault.link_up(40.0))
        self.assertEqual(fault.edge(40.0), "link_up")
        self.assertEqual(fault.evidence(40.0)["actual_end_sim_time_s"], 40.0)

    def test_pose_freshness_gate_does_not_refresh_from_cached_or_old_tf(self) -> None:
        gate = PoseFreshnessGate(("carter1", "carter2"), 1.0)
        self.assertTrue(gate.accept("carter1", pose_stamp_s=4.9, now_s=5.0))
        self.assertTrue(gate.accept("carter2", pose_stamp_s=4.9, now_s=5.0))
        self.assertEqual(gate.stale(6.0), ())
        self.assertFalse(gate.accept("carter1", pose_stamp_s=4.9, now_s=6.01))
        self.assertEqual(gate.stale(6.01), ("carter1", "carter2"))

    def test_f3_yield_cost_is_authored_route_length_from_observed_start(self) -> None:
        poses = [(32, 4, math.pi / 2), (32, 20, math.pi),
                 (16, 20, math.pi / 2), (16, 22.5, math.pi / 2)]
        self.assertAlmostEqual(authored_route_length((30, 4), poses), 36.5)

    def test_f1_real_controller_waits_then_auto_grants_after_observed_release(self) -> None:
        traffic = TrafficController.from_file(TRAFFIC, run_id="F1-test")
        observe(traffic, "carter1", (22, 4), 0.0)
        observe(traffic, "carter2", (32, 8), 0.0)
        first = traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0.1)
        waiting = traffic.request("carter2", ["Z_UPPER"], "Z_UPPER:west", now=0.1)
        self.assertTrue(first.granted)
        self.assertFalse(waiting.granted)
        observe(traffic, "carter1", (36, 4), 1.0)
        observe(traffic, "carter2", (32, 8), 1.0)
        released = traffic.release("carter1", ["Z_UPPER"], "Z_UPPER:east", now=1.1)
        self.assertTrue(released.granted)
        snapshot = traffic.snapshot(now=1.1)
        self.assertEqual(snapshot["active_reservations"]["carter2"]["exit_id"], "Z_UPPER:west")

    def test_f2_stale_owner_is_retained_and_denies_waiter(self) -> None:
        traffic = TrafficController.from_file(TRAFFIC, run_id="F2-test")
        observe(traffic, "carter1", (22, 4), 0.0)
        observe(traffic, "carter2", (26, 4), 0.0)
        claim = traffic.claim_observed_occupancy("carter2", "Z_UPPER", "Z_UPPER:east", now=0.1)
        self.assertTrue(claim.granted)
        self.assertFalse(traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0.2).granted)
        observe(traffic, "carter1", (22, 4), 4.0)
        stale = traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=4.1)
        self.assertFalse(stale.granted)
        self.assertEqual(stale.reason, "stale_occupancy")
        self.assertEqual(traffic.snapshot(now=4.1)["resource_owner"]["Z_UPPER"], "carter2")

    def test_f3_cycle_uses_strict_observed_exit_wait_and_selects_carter2_yield(self) -> None:
        traffic = TrafficController.from_file(TRAFFIC, run_id="F3-test")
        observe(traffic, "carter1", (26.8, 4), 0.0)
        observe(traffic, "carter2", (30, 4), 0.0)
        self.assertTrue(traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0.1).granted)
        self.assertFalse(traffic.request("carter2", ["Z_UPPER"], "Z_UPPER:west", now=0.1).granted)
        self.assertTrue(traffic.observe_exit_wait("carter1", "carter2", now=0.2, planning_radius_m=RADIUS))
        for step in range(1, 12):
            observed_at = 0.2 + step * 0.5
            observe(traffic, "carter1", (26.8, 4), observed_at)
            observe(traffic, "carter2", (30, 4), observed_at)
            self.assertTrue(traffic.observe_exit_wait(
                "carter1", "carter2", now=observed_at, planning_radius_m=RADIUS,
            ))
        recovery = traffic.recovery_decision(
            now=5.8, pocket_route_costs={"carter2": {"P1": None, "P2": 36.5}},
        )
        self.assertIsNotNone(recovery)
        assert recovery is not None
        self.assertEqual((recovery.robot_id, recovery.command, recovery.target),
                         ("carter2", "cancel_and_yield", "P2"))


if __name__ == "__main__":
    unittest.main()
