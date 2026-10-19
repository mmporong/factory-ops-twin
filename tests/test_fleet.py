"""Synthetic adapter-contract tests; these are not physical navigation evidence."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
import json
from typing import Any, cast
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.navigation import ABORTED, SUCCEEDED, GoalPose, LocalizationPose, NavigationError, NavigationIntent, NavigationResult
from factory_ops.operations import LedgerConfig, OrderState, Transition


class FleetMissionTest(unittest.TestCase):
    ROBOTS = {"carter1": "P1", "carter2": "P2"}

    @staticmethod
    def order(number: int, *, ready_time: float = 0.0, source: str | None = None) -> FleetOrder:
        return FleetOrder(f"O{number}", f"PAY{number}", source or f"S{number}", ready_time)

    @staticmethod
    def costs(robot, task) -> float | None:
        del robot, task
        return 1.0

    @staticmethod
    def result(
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
        stamp: float | None = None,
    ) -> NavigationResult:
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
                intent.goal.x if x is None else x,
                intent.goal.y,
                intent.goal.yaw,
                sim_time if stamp is None else stamp,
            ),
        )

    def mission(
        self,
        orders: list[FleetOrder] | None = None,
        **kwargs,
    ) -> FleetMission:
        return FleetMission(
            f"fleet-{uuid4()}",
            orders or [self.order(1), self.order(2)],
            robot_stations=self.ROBOTS,
            **kwargs,
        )

    def accept(self, mission: FleetMission, intent: NavigationIntent, at: float) -> None:
        mission.submit_result(intent.robot_id, self.result(intent, at), error_code=0)

    def test_dispatch_issues_two_independent_goals_and_rejects_foreign_robot(self) -> None:
        mission = self.mission()
        intents = mission.dispatch(0.0, self.costs)
        self.assertEqual(2, len(intents))
        self.assertEqual({"carter1", "carter2"}, {intent.robot_id for intent in intents})
        self.assertEqual(2, len({intent.goal_id for intent in intents}))
        self.assertTrue(all(intent.transition is Transition.PICKUP for intent in intents))

        before = mission.ledger.snapshot()
        with self.assertRaisesRegex(NavigationError, "foreign robot"):
            mission.submit_result("intruder", self.result(intents[0], 1.0), error_code=0)
        self.assertEqual(before, mission.ledger.snapshot())
        self.assertEqual(intents[0], mission.active_intents[intents[0].robot_id])

    def test_one_two_and_four_robots_use_same_dispatch_and_observed_handover_contract(self) -> None:
        for count in (1, 2, 4):
            with self.subTest(count=count):
                starts = dict(zip((f"carter{i}" for i in range(1, count + 1)), ("S1", "S2", "S3", "OUT")))
                mission = FleetMission(f"fleet-{count}",
                    [self.order(i, source="S1") for i in range(1, count + 1)],
                    robot_stations=starts, ledger_config=LedgerConfig(input_capacity=count))
                intents = mission.dispatch(0, self.costs)
                self.assertEqual(count, len(intents))
                self.assertEqual(count, len({i.goal_id for i in intents}))
                for intent in intents:
                    self.accept(mission, intent, 1)
                for intent in list(mission.active_intents.values()):
                    self.assertEqual(Transition.DROP, intent.transition)
                    self.accept(mission, intent, 2)
                self.assertFalse(mission.active_intents)
                self.assertTrue(mission.ledger.replay_matches())

    def test_start_slots_must_be_distinct_and_custom_slots_need_a_valid_goal(self) -> None:
        for starts in ({}, {"carter1": "P1", "carter2": "P1"}, {" ": "P1"}, {"carter1": "P3"}):
            with self.subTest(starts=starts), self.assertRaises(NavigationError):
                FleetMission("invalid-starts", [self.order(1)], robot_stations=starts)
        base = self.mission()
        goals = {**base.station_goals, "P3": GoalPose("map", 18, 1.5, 0)}
        mission = FleetMission("custom-start", [self.order(1)], robot_stations={"carter3": "P3"}, station_goals=goals)
        self.assertEqual("P3", mission.ledger.robots["carter3"].location)
        # Declaring a coordinate is an adapter contract, not footprint/traffic qualification.
        self.assertEqual(1, len(mission.dispatch(0, self.costs)))

    def test_future_order_is_not_exposed_but_remains_expected_and_censored(self) -> None:
        mission = self.mission(
            [self.order(1), self.order(2, ready_time=10.0)],
            horizon=5.0,
        )
        self.assertEqual(1, mission.released_order_count)
        self.assertEqual(2, mission.expected_order_count)
        self.assertNotIn("O2", mission.ledger.orders)
        mission.finish_incomplete()
        exported = mission.export()
        self.assertTrue(exported["censored"])
        self.assertFalse(exported["completed"])
        self.assertEqual((1, 2), (exported["released_order_count"], exported["expected_order_count"]))
        self.assertTrue(exported["replay_matches"])

    def test_malformed_dispatch_at_future_time_preserves_all_mission_state(self) -> None:
        invalid_requests = [
            {"eligible_robot_ids": ["intruder"]},
            {"eligible_robot_ids": "carter1"},
            {"eligible_robot_ids": b"carter1"},
            {"eligible_robot_ids": {"carter1"}},
            {"eligible_robot_ids": [""]},
            {"eligible_robot_ids": [None]},
            {"travel_cost": 3.0},
            {"feasible": False},
        ]
        for invalid in invalid_requests:
            with self.subTest(request=invalid):
                mission = self.mission([self.order(1), self.order(2, ready_time=10.0)])
                before = mission.export()
                request: dict[str, Any] = {"travel_cost": self.costs, **invalid}
                with self.assertRaises(NavigationError):
                    mission.dispatch(10.0, **cast(Any, request))
                self.assertEqual(before, mission.export())

    def test_input_buffer_backpressure_uses_shared_can_assign_gate(self) -> None:
        mission = self.mission(
            [self.order(1), self.order(2)],
            ledger_config=LedgerConfig(input_capacity=1),
        )
        intents = mission.dispatch(0.0, self.costs)
        self.assertEqual(1, len(intents))
        self.assertEqual(1, mission.ledger.input_slots_used)
        remaining = [task for task in mission.ledger.ready_tasks if task.task_id != intents[0].task_id]
        self.assertEqual(1, len(remaining))
        self.assertFalse(mission.ledger.can_assign(remaining[0]))

    def test_pickup_then_abort_or_transport_error_preserves_payload_and_fail_stops(self) -> None:
        for label, status, error_code in (("abort", ABORTED, 0), ("transport", SUCCEEDED, 17)):
            with self.subTest(label=label):
                mission = self.mission([self.order(1)])
                pickup = mission.dispatch(0.0, self.costs)[0]
                self.accept(mission, pickup, 1.0)
                drop = mission.active_intents[pickup.robot_id]
                before = mission.ledger.snapshot()
                self.assertEqual("PAY1", before["robots"][pickup.robot_id]["payload_id"])
                with self.assertRaises(NavigationError):
                    mission.submit_result(
                        pickup.robot_id,
                        self.result(drop, 2.0, status=status),
                        error_code=error_code,
                    )
                self.assertTrue(mission.failed)
                self.assertEqual(before, mission.ledger.snapshot())
                self.assertEqual("PAY1", mission.ledger.robots[pickup.robot_id].payload_id)
                self.assertFalse(mission.completed)

    def test_timeout_preserves_assigned_ledger_state(self) -> None:
        mission = self.mission([self.order(1)], goal_timeout_s=3.0)
        mission.dispatch(0.0, self.costs)
        before = mission.ledger.snapshot()
        with self.assertRaisesRegex(NavigationError, "timed out"):
            mission.advance(3.001)
        self.assertTrue(mission.failed)
        self.assertEqual(before, mission.ledger.snapshot())

    def test_success_result_advances_clock_releases_due_orders_and_avoids_regression(self) -> None:
        mission = self.mission([self.order(1), self.order(2, ready_time=0.5)])
        first = mission.dispatch(0.0, self.costs)[0]
        mission.submit_result(first.robot_id, self.result(first, 1.0), error_code=0)
        self.assertEqual(2, mission.released_order_count)
        self.assertIn("O2", mission.ledger.orders)
        self.assertEqual(1.0, mission.ledger.sim_time)
        issued = mission.dispatch(1.0, self.costs)
        self.assertEqual(["O2"], [intent.order_id for intent in issued])

    def test_valid_result_cannot_commit_past_another_robot_deadline(self) -> None:
        mission = self.mission(goal_timeout_s=3.0)
        pickups = mission.dispatch(0.0, self.costs)
        first = pickups[0]
        mission.submit_result(first.robot_id, self.result(first, 1.0), error_code=0)
        drop = mission.active_intents[first.robot_id]
        before = mission.ledger.snapshot()
        with self.assertRaisesRegex(NavigationError, "timed out"):
            mission.submit_result(first.robot_id, self.result(drop, 3.5), error_code=0)
        self.assertTrue(mission.failed)
        self.assertEqual(before, mission.ledger.snapshot())

    def test_valid_result_after_horizon_cannot_commit_handoff(self) -> None:
        mission = self.mission([self.order(1)], horizon=2.0)
        pickup = mission.dispatch(0.0, self.costs)[0]
        mission.submit_result(pickup.robot_id, self.result(pickup, 1.0), error_code=0)
        drop = mission.active_intents[pickup.robot_id]
        before = mission.ledger.snapshot()
        with self.assertRaisesRegex(NavigationError, "horizon"):
            mission.submit_result(drop.robot_id, self.result(drop, 2.001), error_code=0)
        self.assertEqual(before, mission.ledger.snapshot())
        self.assertEqual(drop, mission.active_intents[drop.robot_id])
        self.assertFalse(mission.failed)

    def test_sources_and_robot_starting_anchors_are_restricted(self) -> None:
        for station in ("CI", "CO", "OUT", "P1", "unknown"):
            with self.subTest(source=station), self.assertRaisesRegex(NavigationError, "source station"):
                FleetOrder("O", "P", station, 0.0)
        with self.assertRaisesRegex(NavigationError, "known station"):
            FleetMission(
                "bad-anchor",
                [self.order(1)],
                robot_stations={"carter1": "P1", "carter2": "unknown"},
            )
        valid = FleetMission(
            "known-anchors",
            [self.order(1)],
            robot_stations={"carter1": "CI", "carter2": "P2"},
        )
        self.assertEqual("CI", valid.ledger.robots["carter1"].location)

    def test_duplicate_receipt_is_idempotent_and_contaminated_reuse_is_rejected(self) -> None:
        mission = self.mission([self.order(1)])
        intent = mission.dispatch(0.0, self.costs)[0]
        result = self.result(intent, 1.0)
        first = mission.submit_result(intent.robot_id, result, error_code=0)
        after = mission.ledger.snapshot()
        journal = mission.navigation_journal
        duplicate = mission.submit_result(intent.robot_id, result, error_code=0)
        self.assertTrue(duplicate.duplicate)
        self.assertEqual(first.ledger_seq, duplicate.ledger_seq)
        self.assertEqual(after, mission.ledger.snapshot())
        self.assertEqual(journal, mission.navigation_journal)

        with self.assertRaisesRegex(NavigationError, "reused"):
            mission.submit_result(
                intent.robot_id,
                replace(result, station="CI"),
                error_code=0,
            )
        self.assertEqual(after, mission.ledger.snapshot())

    def test_mismatched_task_attempt_and_goal_leave_ledger_unchanged(self) -> None:
        variants: tuple[Callable[[NavigationIntent], NavigationResult], ...] = (
            lambda intent: self.result(intent, 1.0, task_id="O999:in"),
            lambda intent: self.result(intent, 1.0, attempt=1),
            lambda intent: self.result(intent, 1.0, goal_id=str(uuid4())),
        )
        for index, make_result in enumerate(variants):
            with self.subTest(index=index):
                mission = self.mission([self.order(1)])
                intent = mission.dispatch(0.0, self.costs)[0]
                before = mission.ledger.snapshot()
                with self.assertRaises(NavigationError):
                    mission.submit_result(
                        intent.robot_id,
                        make_result(intent),
                        error_code=0,
                    )
                self.assertEqual(before, mission.ledger.snapshot(), index)
                self.assertEqual(intent, mission.active_intents[intent.robot_id])

    def test_all_orders_run_through_process_and_shipping_before_completion(self) -> None:
        mission = self.mission()
        inbound = mission.dispatch(0.0, self.costs)
        by_order = {intent.order_id: intent for intent in inbound}
        self.accept(mission, by_order["O1"], 1.0)
        self.accept(mission, by_order["O2"], 1.1)
        self.accept(mission, mission.active_intents[by_order["O1"].robot_id], 2.0)
        self.accept(mission, mission.active_intents[by_order["O2"].robot_id], 2.1)
        self.assertFalse(mission.completed)

        outbound_one = mission.dispatch(22.0, self.costs)
        self.assertEqual(1, len(outbound_one))
        self.assertEqual("O1", outbound_one[0].order_id)
        self.accept(mission, outbound_one[0], 23.0)
        self.accept(mission, mission.active_intents[outbound_one[0].robot_id], 24.0)
        self.assertFalse(mission.completed)

        outbound_two = mission.dispatch(42.0, self.costs)
        self.assertEqual(1, len(outbound_two))
        self.assertEqual("O2", outbound_two[0].order_id)
        self.accept(mission, outbound_two[0], 43.0)
        self.accept(mission, mission.active_intents[outbound_two[0].robot_id], 44.0)

        exported = mission.export()
        self.assertTrue(mission.completed)
        self.assertEqual(2, mission.completed_order_count)
        self.assertEqual(
            {OrderState.COMPLETE},
            {order.state for order in mission.ledger.orders.values()},
        )
        self.assertTrue(exported["replay_matches"])
        self.assertIn("no ROS, Isaac Sim, or physical result claim", exported["scope"])
        json.dumps(exported)

    def test_fifo_eta_ties_unreachable_and_external_eligibility_share_conditions(self) -> None:
        costs = {
            ("carter1", "S1"): 10.0,
            ("carter2", "S1"): 11.0,
            ("carter1", "S2"): 1.0,
            ("carter2", "S2"): 2.0,
        }
        fifo = self.mission(policy="fifo")
        eta = self.mission(policy="eta")
        self.assertEqual("O1", fifo.dispatch(0.0, costs)[0].order_id)
        self.assertEqual("O2", eta.dispatch(0.0, costs)[0].order_id)

        tied = self.mission([self.order(1)])
        tied_costs = {("carter1", "S1"): 1.0, ("carter2", "S1"): 1.0}
        self.assertEqual("carter1", tied.dispatch(0.0, tied_costs)[0].robot_id)

        unreachable = self.mission()
        later_only = {("carter2", "S2"): 3.0}
        issued = unreachable.dispatch(
            0.0,
            later_only,
            eligible_robot_ids=["carter2"],
        )
        self.assertEqual([("O2", "carter2")], [(item.order_id, item.robot_id) for item in issued])

    def test_first_dispatch_cost_exception_fail_stops_without_partial_assignment(self) -> None:
        mission = self.mission()
        before = mission.ledger.snapshot()

        def broken_cost(robot, task):
            del robot, task
            raise RuntimeError("cost backend unavailable")

        with self.assertRaisesRegex(NavigationError, "selection callback"):
            mission.dispatch(0.0, broken_cost)
        self.assertTrue(mission.failed)
        self.assertEqual({}, mission.active_intents)
        self.assertEqual(before, mission.ledger.snapshot())
        assert mission.failure is not None
        self.assertEqual({}, mission.failure["active_intents"])
        self.assertIn("RuntimeError: cost backend unavailable", mission.failure["reason"])
        self.assertEqual("mission_failed", mission.navigation_journal[-1]["kind"])

    def test_second_dispatch_cost_exception_records_partial_assignment_then_fail_stops(self) -> None:
        mission = self.mission()
        calls = 0

        def third_cost_fails(robot, task):
            nonlocal calls
            del robot, task
            calls += 1
            if calls == 3:
                raise RuntimeError("second selection failed")
            return 1.0

        with self.assertRaisesRegex(NavigationError, "selection callback"):
            mission.dispatch(0.0, third_cost_fails)
        self.assertEqual(3, calls)
        self.assertTrue(mission.failed)
        self.assertEqual({}, mission.active_intents)
        assert mission.failure is not None
        self.assertEqual(1, len(mission.failure["active_intents"]))
        self.assertEqual(1, sum(robot.task_id is not None for robot in mission.ledger.robots.values()))
        self.assertEqual(1, sum(order.state is OrderState.INBOUND_ASSIGNED for order in mission.ledger.orders.values()))
        self.assertTrue(mission.ledger.replay_matches())
        with self.assertRaisesRegex(NavigationError, "fail-stopped"):
            mission.dispatch(0.0, self.costs)

    def test_feasible_callback_exception_never_leaves_an_operable_partial_intent(self) -> None:
        mission = self.mission()
        before = mission.ledger.snapshot()

        def broken_feasible(task):
            del task
            raise LookupError("eligibility source failed")

        with self.assertRaisesRegex(NavigationError, "selection callback"):
            mission.dispatch(0.0, self.costs, feasible=broken_feasible)
        self.assertTrue(mission.failed)
        self.assertEqual({}, mission.active_intents)
        self.assertEqual(before, mission.ledger.snapshot())
        assert mission.failure is not None
        self.assertIn("LookupError: eligibility source failed", mission.failure["reason"])


if __name__ == "__main__":
    unittest.main()
