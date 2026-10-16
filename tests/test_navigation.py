from __future__ import annotations

from dataclasses import replace
import json
import math
import unittest
from typing import Any
from uuid import uuid4

from factory_ops.navigation import (
    ABORTED,
    CANCELED,
    SUCCEEDED,
    GoalPose,
    LocalizationPose,
    NavigationError,
    NavigationIntent,
    NavigationResult,
    SingleOrderMission,
)
from factory_ops.operations import OrderState, Transition


class SingleOrderMissionTest(unittest.TestCase):
    def test_external_runtime_failure_is_fail_stopped_without_inventing_action_result(self) -> None:
        mission = SingleOrderMission("external-failure")
        intent = mission.start()
        before = mission.ledger.snapshot()
        mission.fail("DDS transport disconnected", sim_time=1.0)
        self.assertTrue(mission.failed)
        self.assertIsNone(mission.active_intent)
        self.assertEqual(mission.ledger.snapshot(), before)
        self.assertIsNotNone(mission.failure)
        assert mission.failure is not None
        self.assertIsNone(mission.failure["result"])
        self.assertEqual(mission.failure["intent"]["goal_id"], intent.goal_id)
        self.assertTrue(mission.ledger.replay_matches())
        with self.assertRaisesRegex(NavigationError, "fail-stopped"):
            mission.submit_result(self.result(intent, 1.1))

    def test_external_failure_rejects_invalid_reason_and_past_time(self) -> None:
        mission = SingleOrderMission("invalid-external-failure")
        mission.start()
        mission.advance(1.0)
        for reason, sim_time in [("", 1.0), ("error", 0.5)]:
            with self.assertRaises(NavigationError):
                mission.fail(reason, sim_time=sim_time)
            self.assertFalse(mission.failed)
            self.assertIsNotNone(mission.active_intent)

    def result(
        self,
        intent: NavigationIntent,
        sim_time: float,
        *,
        result_id: str | None = None,
        goal_id: str | None = None,
        task_id: str | None = None,
        station: str | None = None,
        attempt: int | None = None,
        status: int = SUCCEEDED,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        stamp: float | None = None,
    ) -> NavigationResult:
        goal = intent.goal
        return NavigationResult(
            result_id or str(uuid4()),
            goal_id or intent.goal_id,
            task_id or intent.task_id,
            station or intent.station,
            intent.attempt if attempt is None else attempt,
            status,
            sim_time,
            LocalizationPose(
                "map",
                goal.x if x is None else x,
                goal.y if y is None else y,
                goal.yaw if yaw is None else yaw,
                sim_time if stamp is None else stamp,
            ),
        )

    def test_one_order_flow_exports_replayable_serializable_evidence(self) -> None:
        mission = SingleOrderMission("g1-flow")

        pickup_s1 = mission.start()
        self.assertEqual(("S1", Transition.PICKUP), (pickup_s1.station, pickup_s1.transition))
        self.assertEqual("map", pickup_s1.goal.frame_id)
        drop_ci = self._accept_and_get_active(mission, pickup_s1, 1.0)
        self.assertEqual(("CI", Transition.DROP), (drop_ci.station, drop_ci.transition))
        mission.submit_result(self.result(drop_ci, 2.0))

        self.assertIsNone(mission.active_intent)
        self.assertEqual(OrderState.PROCESSING, mission.ledger.orders["O1"].state)
        self.assertIsNone(mission.advance(21.999))
        self.assertEqual(OrderState.PROCESSING, mission.ledger.orders["O1"].state)
        pickup_co = mission.advance(22.0)
        assert pickup_co is not None
        self.assertEqual(("CO", Transition.PICKUP), (pickup_co.station, pickup_co.transition))
        drop_out = self._accept_and_get_active(mission, pickup_co, 23.0)
        self.assertEqual(("OUT", Transition.DROP), (drop_out.station, drop_out.transition))
        mission.submit_result(self.result(drop_out, 24.0))

        evidence = mission.export()
        self.assertTrue(mission.completed)
        self.assertEqual(OrderState.COMPLETE, mission.ledger.orders["O1"].state)
        self.assertTrue(evidence["replay_matches"])
        self.assertIn("no ROS or physical navigation claim", evidence["scope"])
        json.dumps(evidence)

    def test_wrong_goal_and_binding_are_rejected_then_real_result_can_retry(self) -> None:
        mission = SingleOrderMission("g1-retry")
        intent = mission.start()
        ledger_before = mission.ledger.snapshot()

        with self.assertRaisesRegex(NavigationError, "goal_id"):
            mission.submit_result(self.result(intent, 1.0, goal_id=str(uuid4())))
        self.assertEqual(ledger_before, mission.ledger.snapshot())
        self.assertEqual(intent, mission.active_intent)

        with self.assertRaisesRegex(NavigationError, "binding"):
            mission.submit_result(self.result(intent, 1.0, station="CI"))
        self.assertEqual(ledger_before, mission.ledger.snapshot())
        receipt = mission.submit_result(self.result(intent, 1.0))
        self.assertTrue(receipt.accepted)
        drop_intent = mission.active_intent
        assert drop_intent is not None
        ledger_after_pickup = mission.ledger.snapshot()
        with self.assertRaisesRegex(NavigationError, "goal_id"):
            mission.submit_result(self.result(intent, 1.1))
        self.assertEqual(ledger_after_pickup, mission.ledger.snapshot())
        self.assertEqual(drop_intent, mission.active_intent)

    def test_duplicate_receipt_is_idempotent_and_changed_reuse_is_rejected(self) -> None:
        mission = SingleOrderMission("g1-dedupe")
        intent = mission.start()
        result = self.result(intent, 1.0)
        first = mission.submit_result(result)
        ledger_after = mission.ledger.snapshot()
        journal_after = mission.navigation_journal

        duplicate = mission.submit_result(result)
        self.assertTrue(duplicate.duplicate)
        self.assertEqual(first.ledger_seq, duplicate.ledger_seq)
        self.assertEqual(ledger_after, mission.ledger.snapshot())
        self.assertEqual(journal_after, mission.navigation_journal)

        changed = replace(result, station="CI")
        with self.assertRaisesRegex(NavigationError, "reused"):
            mission.submit_result(changed)
        self.assertEqual(ledger_after, mission.ledger.snapshot())

    def test_cancel_abort_and_nonterminal_status_fail_stop_without_pickup(self) -> None:
        for index, status in enumerate((CANCELED, ABORTED, 1)):
            with self.subTest(status=status):
                mission = SingleOrderMission(f"g1-status-{index}")
                intent = mission.start()
                before = mission.ledger.snapshot()
                with self.assertRaises(NavigationError):
                    mission.submit_result(self.result(intent, 1.0, status=status))
                self.assertEqual(before, mission.ledger.snapshot())
                self.assertTrue(mission.failed)
                self.assertIsNone(mission.active_intent)
                self.assertEqual("mission_failed", mission.navigation_journal[-1]["kind"])

    def test_invalid_success_pose_is_fail_stop_and_guarded_from_mutation(self) -> None:
        cases: tuple[tuple[str, dict[str, Any], str], ...] = (
            ("stale", {"stamp": 0.4}, "stale"),
            ("future", {"stamp": 1.1}, "future"),
            ("off-goal", {"x": 4.251}, "xy tolerance"),
            ("bad-yaw", {"yaw": 0.251}, "yaw tolerance"),
        )
        for name, kwargs, message in cases:
            with self.subTest(name=name):
                mission = SingleOrderMission(f"g1-pose-{name}")
                intent = mission.start()
                before = mission.ledger.snapshot()
                with self.assertRaisesRegex(NavigationError, message):
                    mission.submit_result(self.result(intent, 1.0, **kwargs))
                self.assertEqual(before, mission.ledger.snapshot())
                self.assertTrue(mission.failed)
                with self.assertRaisesRegex(NavigationError, "fail-stopped"):
                    mission.advance(2.0)
                self.assertEqual(before, mission.ledger.snapshot())

    def test_yaw_tolerance_wraps_at_pi(self) -> None:
        goals = {
            "S1": GoalPose("map", 4.0, 4.0, math.pi - 0.1),
            "CI": GoalPose("map", 28.0, 8.0, 0.0),
            "CO": GoalPose("map", 28.0, 16.0, 0.0),
            "OUT": GoalPose("map", 36.0, 12.0, 0.0),
        }
        mission = SingleOrderMission("g1-yaw-wrap", station_goals=goals)
        intent = mission.start()
        receipt = mission.submit_result(self.result(intent, 1.0, yaw=-math.pi + 0.1))
        self.assertTrue(receipt.accepted)

    def test_logical_clock_may_advance_before_result_but_cannot_go_back(self) -> None:
        mission = SingleOrderMission("g1-clock")
        intent = mission.start()
        self.assertIsNone(mission.advance(5.0))
        self.assertEqual(intent, mission.active_intent)
        mission.submit_result(self.result(intent, 5.0))
        with self.assertRaisesRegex(NavigationError, "monotonic"):
            mission.advance(4.0)

    def test_timeout_records_failure_without_transport_commit(self) -> None:
        mission = SingleOrderMission("g1-timeout", goal_timeout_s=3.0)
        mission.start()
        before = mission.ledger.snapshot()
        with self.assertRaisesRegex(NavigationError, "timed out"):
            mission.advance(3.001)
        self.assertEqual(before, mission.ledger.snapshot())
        self.assertTrue(mission.failed)
        assert mission.failure is not None
        self.assertEqual("navigation timeout", mission.failure["reason"])

    def test_result_requires_valid_uuid_and_nonzero_monotonic_time(self) -> None:
        mission = SingleOrderMission("g1-result-shape")
        intent = mission.start()
        with self.assertRaisesRegex(NavigationError, "UUID"):
            self.result(intent, 1.0, result_id="not-a-uuid")
        with self.assertRaisesRegex(NavigationError, "positive"):
            self.result(intent, 0.0)

    def _accept_and_get_active(
        self, mission: SingleOrderMission, intent: NavigationIntent, sim_time: float
    ) -> NavigationIntent:
        mission.submit_result(self.result(intent, sim_time))
        active = mission.active_intent
        assert active is not None
        return active


if __name__ == "__main__":
    unittest.main()
