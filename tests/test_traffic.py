import json
import math
from pathlib import Path
import tempfile
import unittest

from factory_ops.traffic import Decision, TrafficController, TrafficError


CONFIG = Path(__file__).parents[1] / "configs" / "factory-v3" / "traffic.json"


def _require_decision(value: Decision | None) -> Decision:
    if value is None:
        raise AssertionError("expected a traffic decision")
    return value


def _reservation_id(value: Decision) -> int:
    if value.reservation_id is None:
        raise AssertionError("expected a reservation token")
    return value.reservation_id


def _recovery_id(value: Decision) -> int:
    if value.recovery_id is None:
        raise AssertionError("expected a recovery token")
    return value.recovery_id


class TrafficControllerTests(unittest.TestCase):
    def controller(self) -> TrafficController:
        return TrafficController.from_file(CONFIG)

    def observe(self, ctl, robot, now, occupied=(), clear=(), status="available"):
        ctl.update_observation(
            robot,
            pose_xy=(22, 4),
            pose_time=now,
            heartbeat_time=now,
            occupied_resources=occupied,
            clear_exits=clear,
            status=status,
            now=now,
        )

    def test_config_loads_three_resources_and_marks_backend_limit(self):
        ctl = self.controller()
        self.assertEqual(set(ctl.resources), {"Z_UPPER", "Z_LOWER", "J_SHIP"})
        self.assertIn("simulator/Nav2 execution not verified", ctl.snapshot(now=0)["backend"])

    def test_atomic_contention_exit_lock_and_fair_pending_order(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 0)
        self.observe(ctl, "carter2", 0)
        self.observe(ctl, "carter2", 0)
        self.observe(ctl, "carter3", 0)
        first = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        self.assertTrue(first.granted)

        blocked = ctl.request("carter3", ["Z_UPPER", "J_SHIP"], "J_SHIP:east", now=0.1)
        earlier = ctl.request("carter2", ["Z_UPPER"], "Z_UPPER:west", now=0.1)
        self.assertFalse(blocked.granted)
        self.assertFalse(earlier.granted)
        self.assertEqual(ctl.snapshot(now=0.1)["resource_owner"], {"Z_UPPER": "carter1"})
        self.assertNotIn("J_SHIP", ctl.snapshot(now=0.1)["resource_owner"])

        self.observe(ctl, "carter1", 0.3, clear=("Z_UPPER:east",))
        self.assertTrue(ctl.release("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0.3).granted)
        snap = ctl.snapshot(now=0.3)
        self.assertEqual(snap["resource_owner"], {"Z_UPPER": "carter2"})
        self.assertNotIn("J_SHIP", snap["resource_owner"])

    def test_atomic_bundle_cannot_be_partially_released(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 0)
        ctl.request("carter1", ["Z_UPPER", "J_SHIP"], "J_SHIP:east", now=0)
        self.observe(
            ctl, "carter1", 1,
            clear=("Z_UPPER:east", "J_SHIP:east"),
        )
        denied = ctl.release("carter1", ["J_SHIP"], "J_SHIP:east", now=1)
        self.assertEqual(denied.reason, "reservation_bundle_mismatch")
        self.assertEqual(
            ctl.snapshot(now=1)["resource_owner"],
            {"J_SHIP": "carter1", "Z_UPPER": "carter1"},
        )

    def test_stale_owner_keeps_occupancy_and_blocks_new_entry(self):
        ctl = self.controller()
        self.observe(ctl, "carter2", 0, occupied=("Z_UPPER",))
        claimed = ctl.claim_observed_occupancy("carter2", "Z_UPPER", "Z_UPPER:east", now=0)
        self.assertTrue(claimed.granted)
        self.observe(ctl, "carter1", 4)
        denied = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:west", now=4)
        self.assertFalse(denied.granted)
        self.assertEqual(denied.reason, "stale_occupancy")
        self.assertEqual(ctl.snapshot(now=4)["resource_owner"]["Z_UPPER"], "carter2")
        stops = ctl.safety_decisions(now=4)
        self.assertIn(("carter2", "request_stop"), {(item.robot_id, item.command) for item in stops})

    def test_release_requires_fresh_physical_exit_and_clear_port_then_acks(self):
        ctl = self.controller()
        self.observe(ctl, "carter2", 0, occupied=("Z_UPPER",))
        ctl.claim_observed_occupancy("carter2", "Z_UPPER", "Z_UPPER:east", now=0)
        self.observe(ctl, "carter1", 0.1)
        ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:west", now=0.1)

        self.observe(ctl, "carter2", 30, occupied=("Z_UPPER",), clear=("Z_UPPER:east",))
        still_inside = ctl.release("carter2", ["Z_UPPER"], "Z_UPPER:east", now=30)
        self.assertEqual(still_inside.reason, "footprint_not_clear")
        self.observe(ctl, "carter2", 31)
        port_not_clear = ctl.release("carter2", ["Z_UPPER"], "Z_UPPER:east", now=31)
        self.assertEqual(port_not_clear.reason, "destination_or_port_not_clear")
        self.observe(ctl, "carter2", 32, clear=("Z_UPPER:east",))
        self.observe(ctl, "carter1", 32)
        released = ctl.release("carter2", ["Z_UPPER"], "Z_UPPER:east", now=32)
        self.assertTrue(released.granted)
        self.assertEqual(released.reason, "release_ack")
        self.assertEqual(ctl.snapshot(now=32)["resource_owner"]["Z_UPPER"], "carter1")

    def test_cycle_after_five_seconds_yields_high_id_and_p1_tie(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 0)
        self.observe(ctl, "carter2", 0)
        held = ctl.request("carter2", ["Z_LOWER"], "Z_LOWER:east", now=0)
        ctl.record_wait("carter1", ["carter2"], now=0)
        ctl.record_wait("carter2", ["carter1"], now=0)
        self.assertIsNone(ctl.detect_cycle(now=4.999))
        self.observe(ctl, "carter1", 5)
        self.observe(ctl, "carter2", 5)
        decision = _require_decision(ctl.recovery_decision(
            now=5,
            pocket_route_costs={"carter2": {"P1": 12.0, "P2": 12.0}},
        ))
        self.assertTrue(decision.granted)
        self.assertEqual((decision.robot_id, decision.command, decision.target),
                         ("carter2", "cancel_and_yield", "P1"))
        self.assertGreater(_recovery_id(decision), 0)
        self.assertEqual(decision.reservation_id, held.reservation_id)

    def test_cycle_without_feasible_pocket_is_preserved_as_fault(self):
        ctl = self.controller()
        ctl.record_wait("carter1", ["carter2"], now=0)
        ctl.record_wait("carter2", ["carter1"], now=0)
        self.observe(ctl, "carter1", 5)
        self.observe(ctl, "carter2", 5)
        decision = _require_decision(
            ctl.recovery_decision(now=5, pocket_route_costs={"carter2": {"P1": None}})
        )
        self.assertFalse(decision.granted)
        self.assertEqual(decision.reason, "no_feasible_pocket")
        self.assertEqual(decision.command, "fault")

    def test_recovery_requires_fresh_motion_allowed_cycle_observations(self):
        ctl = self.controller()
        ctl.record_wait("carter1", ["carter2"], now=0)
        ctl.record_wait("carter2", ["carter1"], now=0)
        missing = _require_decision(ctl.recovery_decision(
            now=5, pocket_route_costs={"carter2": {"P1": 1.0}}
        ))
        self.assertEqual((missing.reason, missing.command),
                         ("unknown_robot_observation", "request_stop"))
        self.observe(ctl, "carter1", 6)
        ctl.update_observation(
            "carter2", pose_xy=(30, 4), pose_time=6, heartbeat_time=6,
            status="fault", motion_allowed=False, now=6,
        )
        blocked = _require_decision(ctl.recovery_decision(
            now=6, pocket_route_costs={"carter2": {"P1": 1.0}}
        ))
        self.assertEqual((blocked.reason, blocked.command),
                         ("motion_not_allowed", "request_stop"))
        self.assertGreater(_recovery_id(blocked), _recovery_id(missing))

    def test_stall_is_separate_from_normal_waits(self):
        ctl = self.controller()
        ctl.update_progress("carter1", now=0, kind="goal_started")
        ctl.update_progress("carter1", now=30, kind="reservation_wait")
        self.assertFalse(ctl.is_stalled("carter1", now=59.999))
        self.assertTrue(ctl.is_stalled("carter1", now=60))
        ctl.update_progress("carter1", now=61, kind="pose_progress")
        ctl.update_progress("carter1", now=100, kind="process_wait")
        self.assertFalse(ctl.is_stalled("carter1", now=120))

    def test_resume_waits_for_release_ack(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 0)
        self.observe(ctl, "carter2", 0)
        grant = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        held = ctl.resume_after_release_ack(
            "carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(grant),
            goal_xy=(20, 4), now=0
        )
        self.assertEqual((held.granted, held.command), (False, "hold"))
        self.observe(ctl, "carter1", 1, clear=("Z_UPPER:east",))
        self.observe(ctl, "carter2", 1)
        ctl.release("carter1", ["Z_UPPER"], "Z_UPPER:east", now=1)
        resume = ctl.resume_after_release_ack(
            "carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(grant),
            goal_xy=(20, 4), now=1
        )
        self.assertEqual((resume.granted, resume.command), (True, "replan_resume"))
        reused = ctl.resume_after_release_ack(
            "carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(grant),
            goal_xy=(20, 4), now=1,
        )
        self.assertEqual(reused.reason, "release_ack_consumed")

    def test_reservation_generation_invalidates_old_ack_and_binds_waiter(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 0)
        self.observe(ctl, "carter2", 0)
        self.observe(ctl, "carter3", 0)
        first = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        held = ctl.resume_after_release_ack(
            "carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(first), goal_xy=(20, 4), now=0,
        )
        self.assertEqual(held.reason, "release_ack_required")
        wrong_waiter = ctl.resume_after_release_ack(
            "carter3", owner_robot_id="carter1", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(first), goal_xy=(20, 4), now=0,
        )
        self.assertEqual(wrong_waiter.reason, "release_ack_waiter_mismatch")
        self.observe(ctl, "carter1", 1, clear=("Z_UPPER:east",))
        self.observe(ctl, "carter2", 1)
        self.observe(ctl, "carter3", 1)
        ctl.release("carter1", ["Z_UPPER"], "Z_UPPER:east", now=1)
        self.observe(ctl, "carter1", 1)
        second = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=1)
        self.assertGreater(_reservation_id(second), _reservation_id(first))
        old = ctl.resume_after_release_ack(
            "carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(first), goal_xy=(20, 4), now=1,
        )
        self.assertEqual(old.reason, "release_ack_generation_mismatch")

    def test_stale_or_motion_blocked_waiter_cannot_consume_release_ack(self):
        ctl = self.controller()
        self.observe(ctl, "owner", 0)
        self.observe(ctl, "waiter", 0)
        grant = ctl.request("owner", ["Z_UPPER"], "Z_UPPER:east", now=0)
        self.observe(ctl, "owner", 4, clear=("Z_UPPER:east",))
        ctl.release("owner", ["Z_UPPER"], "Z_UPPER:east", now=4)
        stale = ctl.resume_after_release_ack(
            "waiter", owner_robot_id="owner", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(grant), goal_xy=(20, 4), now=4,
        )
        self.assertEqual((stale.reason, stale.command), ("stale_pose", "request_stop"))
        ctl.update_observation(
            "waiter", pose_xy=(16, 22.5), pose_time=5, heartbeat_time=5,
            status="fault", motion_allowed=False, now=5,
        )
        blocked = ctl.resume_after_release_ack(
            "waiter", owner_robot_id="owner", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(grant), goal_xy=(20, 4), now=5,
        )
        self.assertEqual(blocked.reason, "motion_not_allowed")
        self.observe(ctl, "waiter", 6)
        resumed = ctl.resume_after_release_ack(
            "waiter", owner_robot_id="owner", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(grant), goal_xy=(20, 4), now=6,
        )
        self.assertTrue(resumed.granted)

    def test_pending_robot_cannot_claim_and_preserves_all_mappings(self):
        ctl = self.controller()
        self.observe(ctl, "owner", 0)
        self.observe(ctl, "waiter", 0, occupied=("Z_LOWER",))
        ctl.request("owner", ["Z_UPPER"], "Z_UPPER:east", now=0)
        ctl.request("waiter", ["Z_UPPER"], "Z_UPPER:west", now=0)
        before = ctl.snapshot(now=0)
        denied = ctl.claim_observed_occupancy(
            "waiter", "Z_LOWER", "Z_LOWER:east", now=0
        )
        after = ctl.snapshot(now=0)
        self.assertEqual(denied.reason, "pending_request_conflict")
        for key in ("resource_owner", "exit_owner", "active_reservations", "pending"):
            self.assertEqual(before[key], after[key])

    def test_active_claim_conflict_cannot_overwrite_mapping_and_same_claim_is_idempotent(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 0, occupied=("Z_UPPER", "Z_LOWER"))
        claim = ctl.claim_observed_occupancy(
            "carter1", "Z_UPPER", "Z_UPPER:east", now=0
        )
        before = ctl.snapshot(now=0)
        repeated = ctl.claim_observed_occupancy(
            "carter1", "Z_UPPER", "Z_UPPER:east", now=0
        )
        conflict = ctl.claim_observed_occupancy(
            "carter1", "Z_LOWER", "Z_LOWER:east", now=0
        )
        after = ctl.snapshot(now=0)
        self.assertEqual(repeated.reason, "already_claimed")
        self.assertEqual(repeated.reservation_id, claim.reservation_id)
        self.assertEqual(conflict.reason, "active_reservation_conflict")
        self.assertEqual(before["resource_owner"], after["resource_owner"])
        self.assertEqual(before["exit_owner"], after["exit_owner"])
        self.assertEqual(before["active_reservations"], after["active_reservations"])

    def test_stale_pending_is_removed_and_does_not_block_fresh_waiter(self):
        ctl = self.controller()
        for robot in ("owner", "stale", "fresh"):
            self.observe(ctl, robot, 0)
        ctl.request("owner", ["Z_UPPER"], "Z_UPPER:east", now=0)
        ctl.request("stale", ["Z_UPPER"], "Z_UPPER:west", now=0.1)
        ctl.request("fresh", ["Z_UPPER"], "Z_UPPER:west", now=0.2)
        self.observe(ctl, "owner", 4, clear=("Z_UPPER:east",))
        self.observe(ctl, "fresh", 4)
        ctl.release("owner", ["Z_UPPER"], "Z_UPPER:east", now=4)
        snap = ctl.snapshot(now=4)
        self.assertEqual(snap["resource_owner"]["Z_UPPER"], "fresh")
        self.assertNotIn("stale", {item["robot_id"] for item in snap["pending"]})

    def test_fault_status_is_diagnostic_but_motion_allowed_blocks_grant_and_keeps_hold(self):
        ctl = self.controller()
        ctl.update_observation(
            "carter1", pose_xy=(22, 4), pose_time=0, heartbeat_time=0,
            status="fault", motion_allowed=False, now=0,
        )
        denied = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        self.assertEqual((denied.reason, denied.command), ("motion_not_allowed", "hold_at_entry"))
        ctl.update_observation(
            "carter2", pose_xy=(26, 4), pose_time=0, heartbeat_time=0,
            occupied_resources=("Z_UPPER",), status="fault",
            motion_allowed=False, now=0,
        )
        claimed = ctl.claim_observed_occupancy(
            "carter2", "Z_UPPER", "Z_UPPER:east", now=0
        )
        self.assertTrue(claimed.granted)
        self.assertEqual(claimed.command, "request_stop")
        repeated_claim = ctl.claim_observed_occupancy(
            "carter2", "Z_UPPER", "Z_UPPER:east", now=0
        )
        self.assertEqual(
            (repeated_claim.granted, repeated_claim.command),
            (True, "request_stop"),
        )
        self.observe(ctl, "carter3", 0)
        other = ctl.request("carter3", ["Z_UPPER"], "Z_UPPER:west", now=0)
        self.assertEqual(other.reason, "occupied")
        stops = ctl.safety_decisions(now=0)
        self.assertIn(("carter2", "request_stop"),
                      {(item.robot_id, item.command) for item in stops})
        ctl.update_observation(
            "carter2", pose_xy=(26, 4), pose_time=1, heartbeat_time=1,
            occupied_resources=("Z_UPPER",), status="fault",
            motion_allowed=False, now=1,
        )
        repeat = ctl.request("carter2", ["Z_UPPER"], "Z_UPPER:east", now=1)
        self.assertEqual(repeat.reason, "motion_not_allowed")
        self.assertEqual(ctl.snapshot(now=1)["resource_owner"]["Z_UPPER"], "carter2")
        self.assertIsNotNone(claimed.reservation_id)

    def test_bound_waiter_auto_grant_preserves_predecessor_ack_for_one_resume(self):
        ctl = self.controller()
        self.observe(ctl, "owner", 0)
        self.observe(ctl, "waiter", 0)
        predecessor = ctl.request("owner", ["Z_UPPER"], "Z_UPPER:east", now=0)
        ctl.request("waiter", ["Z_UPPER"], "Z_UPPER:west", now=0)
        hold = ctl.resume_after_release_ack(
            "waiter", owner_robot_id="owner", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(predecessor),
            goal_xy=(20, 4), now=0,
        )
        self.assertEqual(hold.reason, "release_ack_required")
        self.observe(ctl, "owner", 1, clear=("Z_UPPER:east",))
        self.observe(ctl, "waiter", 1)
        ctl.release("owner", ["Z_UPPER"], "Z_UPPER:east", now=1)
        snap = ctl.snapshot(now=1)
        successor = snap["active_reservations"]["waiter"]["reservation_id"]
        self.assertGreater(successor, _reservation_id(predecessor))
        ack = next(
            item for item in snap["release_acks"]
            if item["reservation_id"] == _reservation_id(predecessor)
        )
        self.assertEqual(ack["superseded_by"], successor)
        resumed = ctl.resume_after_release_ack(
            "waiter", owner_robot_id="owner", exit_id="Z_UPPER:east",
            expected_reservation_id=_reservation_id(predecessor),
            goal_xy=(20, 4), now=1,
        )
        self.assertTrue(resumed.granted)
        linked = [
            event for event in ctl.events
            if event["event_type"] == "release_ack_linked_to_successor"
        ]
        self.assertEqual(len(linked), 1)
        self.assertEqual(linked[0].get("reservation_id"), successor)

    def test_event_cursor_has_run_id_sequence_and_reservation_token(self):
        ctl = TrafficController.from_file(CONFIG, run_id="cursor-test")
        self.observe(ctl, "carter1", 0)
        grant = ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        first = ctl.events_since(0)
        self.assertEqual(first["run_id"], "cursor-test")
        self.assertEqual(
            [event["event_seq"] for event in first["events"]],
            list(range(1, first["next_cursor"] + 1)),
        )
        self.assertEqual(
            first["events"][-1].get("reservation_id"),
            _reservation_id(grant),
        )
        empty = ctl.events_since(first["next_cursor"])
        self.assertEqual(empty["events"], ())
        with self.assertRaises(TrafficError):
            ctl.events_since(first["next_cursor"] + 1)

    def test_invalid_release_unknown_claim_nan_time_reverse_and_units_rejected(self):
        ctl = self.controller()
        self.observe(ctl, "carter1", 1)
        self.assertEqual(
            ctl.claim_observed_occupancy("unknown", "Z_UPPER", "Z_UPPER:east", now=1).reason,
            "unknown_robot_observation",
        )
        ctl.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=1)
        self.observe(ctl, "carter2", 2, clear=("Z_UPPER:east",))
        self.assertEqual(
            ctl.release("carter2", ["Z_UPPER"], "Z_UPPER:east", now=2).reason,
            "not_owner",
        )
        with self.assertRaises(TrafficError):
            ctl.snapshot(now=math.nan)
        with self.assertRaises(TrafficError):
            ctl.snapshot(now=1)
        with self.assertRaises(TrafficError):
            ctl.update_observation(
                "carter3", pose_xy=(0, 0), pose_time=3, heartbeat_time=3,
                now=3, units="mm"
            )

        data = json.loads(CONFIG.read_text(encoding="utf-8"))
        data["units"] = "mm"
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "traffic.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaises(TrafficError):
                TrafficController.from_file(path)


if __name__ == "__main__":
    unittest.main()
