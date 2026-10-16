from __future__ import annotations

import json
import unittest

from factory_ops.operations import (
    Command,
    EventId,
    Ledger,
    LedgerConfig,
    LedgerError,
    OrderState,
    Stage,
    Task,
    Transition,
    choose_eta,
    choose_fifo,
)


class OperationsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ledger = Ledger("run-1")
        self.ledger.add_robot("carter1", "P1")
        self.ledger.add_robot("carter2", "P2")
        self.ledger.add_order("O1", "P1", "S1", ready_time=0)
        self.seq = 0

    def command(
        self,
        stage: Stage,
        transition: Transition,
        *,
        time: float | None = None,
        order_id: str = "O1",
        payload_id: str = "P1",
        task_id: str | None = None,
        robot_id: str | None = None,
        attempt: int = 0,
        version: int | None = None,
        producer: str = "driver",
        station_id: str | None = "__auto__",
    ) -> Command:
        self.seq += 1
        order = self.ledger.orders[order_id]
        resolved_station = station_id
        if station_id == "__auto__":
            resolved_station = None
            task = self.ledger.tasks.get(task_id or "")
            if task is not None and transition in (Transition.PICKUP, Transition.DROP):
                resolved_station = (
                    task.pickup_station if transition is Transition.PICKUP else task.drop_station
                )
        return Command(
            EventId("run-1", producer, self.seq),
            self.ledger.sim_time if time is None else time,
            order_id,
            payload_id,
            stage,
            transition,
            order.state_version if version is None else version,
            task_id=task_id,
            station_id=resolved_station,
            robot_id=robot_id,
            attempt=attempt,
        )

    def inbound_to_ci(self, order_id="O1", payload_id="P1", robot_id="carter1") -> None:
        task = f"{order_id}:in"
        for transition in (Transition.ASSIGN, Transition.PICKUP, Transition.DROP):
            self.ledger.apply(
                self.command(
                    Stage.INBOUND,
                    transition,
                    order_id=order_id,
                    payload_id=payload_id,
                    task_id=task,
                    robot_id=robot_id,
                )
            )

    def output_to_complete(self, order_id="O1", payload_id="P1", robot_id="carter1") -> None:
        task = f"{order_id}:out"
        for transition in (Transition.ASSIGN, Transition.PICKUP, Transition.DROP):
            self.ledger.apply(
                self.command(
                    Stage.OUTBOUND,
                    transition,
                    order_id=order_id,
                    payload_id=payload_id,
                    task_id=task,
                    robot_id=robot_id,
                )
            )
        self.ledger.apply(
            self.command(Stage.OUTBOUND, Transition.ORDER_COMPLETE, order_id=order_id, payload_id=payload_id)
        )

    def test_complete_lot_flow_and_ci_drop_frees_robot(self) -> None:
        self.inbound_to_ci()
        robot = self.ledger.robots["carter1"]
        self.assertTrue(robot.available)
        self.assertIsNone(robot.payload_id)
        self.assertIsNone(robot.task_id)
        self.assertEqual(OrderState.INPUT_WAITING, self.ledger.orders["O1"].state)

        self.ledger.advance(0)
        self.assertEqual(OrderState.PROCESSING, self.ledger.orders["O1"].state)
        self.ledger.advance(20)
        self.assertEqual(OrderState.OUTPUT_WAITING, self.ledger.orders["O1"].state)
        self.output_to_complete(robot_id="carter2")
        self.assertEqual(OrderState.COMPLETE, self.ledger.orders["O1"].state)
        self.assertEqual(1, sum(order.state is OrderState.COMPLETE for order in self.ledger.orders.values()))
        self.assertTrue(self.ledger.replay_matches())
        json.dumps(self.ledger.events)
        json.dumps(self.ledger.snapshot())

    def test_output_capacity_blocks_then_releases_processing(self) -> None:
        ledger = Ledger("run-1", LedgerConfig(output_capacity=1))
        ledger.add_robot("carter1", "P1")
        ledger.add_robot("carter2", "P2")
        self.ledger = ledger
        self.seq = 0
        ledger.add_order("O1", "P1", "S1", ready_time=0)
        self.inbound_to_ci()
        ledger.advance(0)
        ledger.advance(20)

        ledger.add_order("O2", "P2", "S2", ready_time=20)
        self.inbound_to_ci("O2", "P2", "carter1")
        ledger.advance(20)
        self.assertEqual(OrderState.INPUT_WAITING, ledger.orders["O2"].state)

        self.output_to_complete("O1", "P1", "carter2")
        ledger.advance(20)
        self.assertEqual(OrderState.PROCESSING, ledger.orders["O2"].state)

    def test_ci_capacity_rejection_preserves_loaded_payload(self) -> None:
        ledger = Ledger("run-1", LedgerConfig(input_capacity=1))
        ledger.add_robot("carter1", "P1")
        ledger.add_robot("carter2", "P2")
        self.ledger = ledger
        self.seq = 0
        ledger.add_order("O1", "P1", "S1", ready_time=0)
        ledger.add_order("O2", "P2", "S2", ready_time=0)
        ledger.apply(
            self.command(
                Stage.INBOUND,
                Transition.ASSIGN,
                order_id="O1",
                payload_id="P1",
                task_id="O1:in",
                robot_id="carter1",
            )
        )
        self.assertEqual(1, ledger.input_slots_used)
        self.assertFalse(ledger.can_assign(ledger.tasks["O2:in"]))
        before = ledger.snapshot()
        with self.assertRaises(LedgerError):
            ledger.apply(
                self.command(
                    Stage.INBOUND,
                    Transition.ASSIGN,
                    order_id="O2",
                    payload_id="P2",
                    task_id="O2:in",
                    robot_id="carter2",
                )
            )
        self.assertEqual(before, ledger.snapshot())
        self.assertTrue(ledger.robots["carter2"].available)

        for transition in (Transition.PICKUP, Transition.DROP):
            ledger.apply(
                self.command(
                    Stage.INBOUND,
                    transition,
                    order_id="O1",
                    payload_id="P1",
                    task_id="O1:in",
                    robot_id="carter1",
                )
            )
        self.assertEqual(1, ledger.input_slots_used)
        ledger.advance(0)
        self.assertEqual(0, ledger.input_slots_used)
        self.assertTrue(ledger.can_assign(ledger.tasks["O2:in"]))

    def test_duplicate_event_and_business_key_different_attempt_are_noops(self) -> None:
        command = self.command(
            Stage.INBOUND, Transition.ASSIGN, task_id="O1:in", robot_id="carter1"
        )
        first = self.ledger.apply(command)
        snapshot = self.ledger.snapshot()
        duplicate_event = self.ledger.apply(command)
        self.assertTrue(duplicate_event.duplicate)

        self.seq += 1
        business_duplicate = Command(
            EventId("run-1", "retry", self.seq),
            0,
            "O1",
            "P1",
            Stage.INBOUND,
            Transition.ASSIGN,
            999,
            task_id="O1:in",
            robot_id="carter2",
            attempt=7,
        )
        duplicate_key = self.ledger.apply(business_duplicate)
        self.assertTrue(duplicate_key.duplicate)
        self.assertEqual(first.ledger_seq, duplicate_key.ledger_seq)
        self.assertEqual(snapshot, self.ledger.snapshot())

    def test_version_out_of_order_invalid_and_clock_fail_transactionally(self) -> None:
        before = self.ledger.snapshot()
        before_events = self.ledger.events
        bad_commands = [
            self.command(
                Stage.INBOUND,
                Transition.PICKUP,
                task_id="O1:in",
                robot_id="carter1",
            ),
            self.command(
                Stage.INBOUND,
                Transition.ASSIGN,
                task_id="O1:in",
                robot_id="carter1",
                version=3,
            ),
        ]
        for bad in bad_commands:
            with self.assertRaises(LedgerError):
                self.ledger.apply(bad)
            self.assertEqual(before, self.ledger.snapshot())
            self.assertEqual(before_events, self.ledger.events)

        with self.assertRaises(ValueError):
            self.command(Stage.INBOUND, Transition.ASSIGN, time=float("nan"))
        self.ledger.advance(2)
        state_at_two = self.ledger.snapshot()
        with self.assertRaises(LedgerError):
            self.ledger.advance(1)
        self.assertEqual(state_at_two, self.ledger.snapshot())

    def test_tick_order_is_ledger_sequence_and_replayable(self) -> None:
        self.inbound_to_ci()
        start_results = self.ledger.advance(5)
        complete_results = self.ledger.advance(25)
        self.assertEqual([Transition.PROCESS_START.value], [r.event["transition"] for r in start_results])
        self.assertEqual([Transition.PROCESS_COMPLETE.value], [r.event["transition"] for r in complete_results])
        committed = [event for event in self.ledger.events if "state_version" in event]
        self.assertEqual(sorted(event["ledger_seq"] for event in committed), [event["ledger_seq"] for event in committed])
        replayed = Ledger.replay(self.ledger.events)
        self.assertEqual(self.ledger.snapshot(), replayed.snapshot())

    def test_fifo_and_eta_can_make_different_decisions_and_ties_are_stable(self) -> None:
        tasks = [
            Task("old", "O1", "P1", Stage.INBOUND, "S1", "CI", 0),
            Task("new", "O2", "P2", Stage.INBOUND, "S2", "CI", 1),
        ]
        robots = list(self.ledger.robots.values())
        costs = {
            ("carter1", "S1"): 10,
            ("carter2", "S1"): 10,
            ("carter1", "S2"): 1,
            ("carter2", "S2"): 2,
        }
        fifo = choose_fifo(tasks, robots, costs)
        eta = choose_eta(tasks, robots, costs, speed=1, dwell=5)
        assert fifo is not None
        assert eta is not None
        self.assertEqual(("old", "carter1"), (fifo.task_id, fifo.robot_id))
        self.assertEqual(("new", "carter1"), (eta.task_id, eta.robot_id))

        tie = {key: 1 for key in costs}
        stable = choose_eta(tasks, robots, tie, speed=1, dwell=5)
        assert stable is not None
        self.assertEqual(("old", "carter1"), (stable.task_id, stable.robot_id))

    def test_infeasible_old_task_does_not_block_later_dispatch(self) -> None:
        tasks = [
            Task("blocked-in", "O1", "P1", Stage.INBOUND, "S1", "CI", 0),
            Task("ready-out", "O2", "P2", Stage.OUTBOUND, "CO", "OUT", 1),
        ]
        robots = list(self.ledger.robots.values())
        costs = {
            ("carter1", "S1"): float("inf"),
            ("carter2", "S1"): float("inf"),
            ("carter1", "CO"): 3,
            ("carter2", "CO"): 4,
        }
        allowed = lambda task: task.task_id != "blocked-in"
        fifo = choose_fifo(tasks, robots, costs, feasible=allowed)
        eta = choose_eta(tasks, robots, costs, speed=1, dwell=5, feasible=allowed)
        assert fifo is not None
        assert eta is not None
        self.assertEqual("ready-out", fifo.task_id)
        self.assertEqual("ready-out", eta.task_id)

        # Path infeasibility alone uses the same skip behavior even without a filter.
        path_only = choose_fifo(tasks, robots, costs)
        assert path_only is not None
        self.assertEqual("ready-out", path_only.task_id)

    def test_full_ci_skips_older_inbound_and_dispatches_outbound(self) -> None:
        ledger = Ledger("run-1", LedgerConfig(input_capacity=1))
        ledger.add_robot("carter1", "P1")
        ledger.add_robot("carter2", "P2")
        self.ledger = ledger
        self.seq = 0
        for order_id, payload_id, source in (("O1", "P1", "S1"), ("O2", "P2", "S2"), ("O3", "P3", "S3")):
            ledger.add_order(order_id, payload_id, source, ready_time=0)
        self.inbound_to_ci("O1", "P1", "carter1")
        ledger.advance(0)
        ledger.advance(20)
        self.inbound_to_ci("O2", "P2", "carter1")

        self.assertFalse(ledger.can_assign(ledger.tasks["O3:in"]))
        self.assertTrue(ledger.can_assign(ledger.tasks["O1:out"]))
        costs = {
            ("carter1", "S3"): 1,
            ("carter2", "S3"): 1,
            ("carter1", "CO"): 4,
            ("carter2", "CO"): 3,
        }
        assignment = choose_fifo(
            ledger.ready_tasks,
            list(ledger.robots.values()),
            costs,
            feasible=ledger.can_assign,
        )
        assert assignment is not None
        self.assertEqual(("O1:out", "carter2"), (assignment.task_id, assignment.robot_id))

    def test_loaded_payload_rejects_duplicate_dispatch(self) -> None:
        self.ledger.apply(
            self.command(Stage.INBOUND, Transition.ASSIGN, task_id="O1:in", robot_id="carter1")
        )
        self.ledger.apply(
            self.command(Stage.INBOUND, Transition.PICKUP, task_id="O1:in", robot_id="carter1")
        )
        snapshot = self.ledger.snapshot()
        self.ledger.add_order("O2", "P2", "S2", ready_time=0)
        snapshot = self.ledger.snapshot()
        self.seq += 1
        retry = Command(
            EventId("run-1", "retry", self.seq),
            0,
            "O2",
            "P2",
            Stage.INBOUND,
            Transition.ASSIGN,
            self.ledger.orders["O2"].state_version,
            task_id="O2:in",
            robot_id="carter1",
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(retry)
        self.assertEqual(snapshot, self.ledger.snapshot())
        self.assertEqual("carter1", self.ledger.orders["O1"].owner)

    def test_unstarted_retry_preserves_task_and_blocks_retry_after_pickup(self) -> None:
        self.ledger.apply(
            self.command(Stage.INBOUND, Transition.ASSIGN, task_id="O1:in", robot_id="carter1")
        )
        version = self.ledger.orders["O1"].state_version
        result = self.ledger.reassign_unstarted(
            EventId("run-1", "dispatcher", 1),
            sim_time=0,
            task_id="O1:in",
            from_robot_id="carter1",
            to_robot_id="carter2",
            expected_state_version=version,
            reason="goal rejected before pickup",
        )
        self.assertTrue(result.applied)
        self.assertEqual(1, self.ledger.tasks["O1:in"].attempt)
        self.assertEqual("carter2", self.ledger.tasks["O1:in"].robot_id)
        self.assertTrue(self.ledger.robots["carter1"].available)
        self.ledger.apply(
            self.command(
                Stage.INBOUND,
                Transition.PICKUP,
                task_id="O1:in",
                robot_id="carter2",
                attempt=1,
            )
        )
        before = self.ledger.snapshot()
        with self.assertRaises(LedgerError):
            self.ledger.reassign_unstarted(
                EventId("run-1", "dispatcher", 2),
                sim_time=0,
                task_id="O1:in",
                from_robot_id="carter2",
                to_robot_id="carter1",
                expected_state_version=self.ledger.orders["O1"].state_version,
                reason="unsafe late retry",
            )
        self.assertEqual(before, self.ledger.snapshot())
        self.assertTrue(self.ledger.replay_matches())

    def test_cross_order_pickup_and_station_mismatch_are_atomic_rejections(self) -> None:
        self.ledger.add_order("O2", "P2", "S2", ready_time=0)
        self.ledger.apply(
            self.command(Stage.INBOUND, Transition.ASSIGN, task_id="O1:in", robot_id="carter1")
        )
        self.ledger.apply(
            self.command(
                Stage.INBOUND,
                Transition.ASSIGN,
                order_id="O2",
                payload_id="P2",
                task_id="O2:in",
                robot_id="carter2",
            )
        )
        before = self.ledger.snapshot()
        cross_order = self.command(
            Stage.INBOUND,
            Transition.PICKUP,
            order_id="O1",
            payload_id="P1",
            task_id="O2:in",
            robot_id="carter2",
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(cross_order)
        self.assertEqual(before, self.ledger.snapshot())

        wrong_station = self.command(
            Stage.INBOUND,
            Transition.PICKUP,
            task_id="O1:in",
            robot_id="carter1",
        )
        wrong_station = Command(
            wrong_station.event_id,
            wrong_station.sim_time,
            wrong_station.order_id,
            wrong_station.payload_id,
            wrong_station.stage,
            wrong_station.transition,
            wrong_station.expected_state_version,
            task_id=wrong_station.task_id,
            station_id="S2",
            robot_id=wrong_station.robot_id,
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(wrong_station)
        self.assertEqual(before, self.ledger.snapshot())

    def test_pickup_and_drop_require_station_without_state_change(self) -> None:
        self.ledger.apply(
            self.command(Stage.INBOUND, Transition.ASSIGN, task_id="O1:in", robot_id="carter1")
        )
        before_pickup = self.ledger.snapshot()
        missing_pickup_station = self.command(
            Stage.INBOUND,
            Transition.PICKUP,
            task_id="O1:in",
            robot_id="carter1",
            station_id=None,
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(missing_pickup_station)
        self.assertEqual(before_pickup, self.ledger.snapshot())

        self.ledger.apply(
            self.command(Stage.INBOUND, Transition.PICKUP, task_id="O1:in", robot_id="carter1")
        )
        before_drop = self.ledger.snapshot()
        missing_drop_station = self.command(
            Stage.INBOUND,
            Transition.DROP,
            task_id="O1:in",
            robot_id="carter1",
            station_id=None,
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(missing_drop_station)
        self.assertEqual(before_drop, self.ledger.snapshot())

    def test_event_id_body_collision_is_rejected_across_operation_types(self) -> None:
        event_id = EventId("run-1", "driver", 1)
        assign = Command(
            event_id,
            0,
            "O1",
            "P1",
            Stage.INBOUND,
            Transition.ASSIGN,
            0,
            task_id="O1:in",
            robot_id="carter1",
        )
        self.ledger.apply(assign)
        before = self.ledger.snapshot()
        pickup_same_id = Command(
            event_id,
            0,
            "O1",
            "P1",
            Stage.INBOUND,
            Transition.PICKUP,
            1,
            task_id="O1:in",
            robot_id="carter1",
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(pickup_same_id)
        with self.assertRaises(LedgerError):
            self.ledger.reassign_unstarted(
                event_id,
                sim_time=0,
                task_id="O1:in",
                from_robot_id="carter1",
                to_robot_id="carter2",
                expected_state_version=1,
                reason="collision",
            )
        self.assertEqual(before, self.ledger.snapshot())

    def test_business_noop_event_id_receipt_rejects_later_reuse(self) -> None:
        original = self.command(
            Stage.INBOUND, Transition.ASSIGN, task_id="O1:in", robot_id="carter1"
        )
        self.ledger.apply(original)
        noop_id = EventId("run-1", "retry", 1)
        business_noop = Command(
            noop_id,
            0,
            "O1",
            "P1",
            Stage.INBOUND,
            Transition.ASSIGN,
            999,
            task_id="O1:in",
            robot_id="carter2",
            attempt=7,
        )
        before = self.ledger.snapshot()
        before_events = self.ledger.events
        result = self.ledger.apply(business_noop)
        self.assertTrue(result.duplicate)
        self.assertEqual(before, self.ledger.snapshot())
        self.assertEqual(before_events, self.ledger.events)

        reused = Command(
            noop_id,
            0,
            "O1",
            "P1",
            Stage.INBOUND,
            Transition.PICKUP,
            self.ledger.orders["O1"].state_version,
            task_id="O1:in",
            robot_id="carter1",
        )
        with self.assertRaises(LedgerError):
            self.ledger.apply(reused)
        with self.assertRaises(LedgerError):
            self.ledger.reassign_unstarted(
                noop_id,
                sim_time=0,
                task_id="O1:in",
                from_robot_id="carter1",
                to_robot_id="carter2",
                expected_state_version=self.ledger.orders["O1"].state_version,
                reason="reused no-op receipt",
            )
        self.assertEqual(before, self.ledger.snapshot())
        self.assertEqual(before_events, self.ledger.events)

    def test_replay_rejects_metadata_and_body_tampering_but_json_roundtrip_passes(self) -> None:
        self.inbound_to_ci()
        serialized = json.loads(json.dumps(self.ledger.events))
        replayed = Ledger.replay(serialized)
        self.assertEqual(self.ledger.snapshot(), replayed.snapshot())

        ledger_seq_tampered = json.loads(json.dumps(serialized))
        ledger_seq_tampered[-1]["ledger_seq"] += 10
        with self.assertRaises(LedgerError):
            Ledger.replay(ledger_seq_tampered)

        version_tampered = json.loads(json.dumps(serialized))
        version_tampered[-1]["state_version"] += 1
        with self.assertRaises(LedgerError):
            Ledger.replay(version_tampered)

        body_tampered = json.loads(json.dumps(serialized))
        body_tampered[-1]["reason"] = "modified after commit"
        with self.assertRaises(LedgerError):
            Ledger.replay(body_tampered)

    def test_integer_configuration_fields_reject_bool_and_float(self) -> None:
        for raw_json in (
            '{"input_capacity": true}',
            '{"input_capacity": 2.0}',
            '{"output_capacity": false}',
            '{"workers": 1.0}',
            '{"service_time": true}',
        ):
            with self.subTest(raw_json=raw_json), self.assertRaises(ValueError):
                LedgerConfig(**json.loads(raw_json))
        with self.assertRaises(ValueError):
            EventId("run-1", "producer", True)


if __name__ == "__main__":
    unittest.main()
