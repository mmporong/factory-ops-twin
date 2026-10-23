from __future__ import annotations

import unittest

from factory_ops.operations import Command, EventId, Ledger, LedgerConfig, LedgerError, OrderState, Stage, Transition


class EquipmentAvailabilityTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ledger = Ledger("run-equipment", LedgerConfig(output_capacity=1, service_time=20))
        self.ledger.add_robot("carter1", "P1")
        self.ledger.add_order("O1", "P1", "S1", ready_time=0)
        self.driver_seq = 0
        self.equipment_seq = 0

    def command(
        self,
        transition: Transition,
        *,
        stage: Stage = Stage.INBOUND,
        order_id: str = "O1",
        payload_id: str = "P1",
        task_id: str = "O1:in",
        robot_id: str = "carter1",
    ) -> Command:
        self.driver_seq += 1
        task = self.ledger.tasks[task_id]
        station_id = task.drop_station if transition is Transition.DROP else task.pickup_station
        return Command(
            EventId(self.ledger.run_id, "driver", self.driver_seq),
            self.ledger.sim_time,
            order_id,
            payload_id,
            stage,
            transition,
            self.ledger.orders[order_id].state_version,
            task_id=task_id,
            station_id=station_id,
            robot_id=robot_id,
        )

    def inbound_to_ci(self, order_id: str = "O1", payload_id: str = "P1") -> None:
        for transition in (Transition.ASSIGN, Transition.PICKUP, Transition.DROP):
            self.ledger.apply(
                self.command(
                    transition,
                    order_id=order_id,
                    payload_id=payload_id,
                    task_id=f"{order_id}:in",
                )
            )

    def availability(self, *, sim_time: float, available: bool, reason: str | None = None):
        self.equipment_seq += 1
        return self.ledger.set_equipment_availability(
            EventId(self.ledger.run_id, "equipment", self.equipment_seq),
            sim_time=sim_time,
            available=available,
            reason=reason,
        )

    def test_failure_pauses_active_service_and_repair_resumes_remaining_time(self) -> None:
        self.inbound_to_ci()
        self.ledger.advance(0)

        self.availability(sim_time=8, available=False, reason="tester alarm")
        equipment = self.ledger.snapshot()["equipment"]
        self.assertEqual("down", equipment["status"])
        self.assertEqual("tester alarm", equipment["reason"])
        self.assertEqual("O1", equipment["paused_order_id"])
        self.assertEqual(12, equipment["remaining_service"])
        self.assertIsNone(self.ledger.orders["O1"].service_end)

        self.ledger.advance(100)
        self.assertEqual(OrderState.PROCESSING, self.ledger.orders["O1"].state)
        self.availability(sim_time=100, available=True, reason="alarm cleared")
        self.assertEqual(112, self.ledger.orders["O1"].service_end)
        self.ledger.advance(111.9)
        self.assertEqual(OrderState.PROCESSING, self.ledger.orders["O1"].state)
        self.ledger.advance(112)
        self.assertEqual(OrderState.OUTPUT_WAITING, self.ledger.orders["O1"].state)
        self.assertEqual(self.ledger.snapshot(), Ledger.replay(self.ledger.events).snapshot())

    def test_down_cell_blocks_new_work_and_keeps_output_backpressure(self) -> None:
        self.availability(sim_time=0, available=False, reason="maintenance")
        self.inbound_to_ci()
        with self.assertRaisesRegex(LedgerError, "available equipment"):
            self.ledger.apply(
                Command(
                    EventId(self.ledger.run_id, "driver", self.driver_seq + 1),
                    self.ledger.sim_time,
                    "O1",
                    "P1",
                    Stage.PROCESS,
                    Transition.PROCESS_START,
                    self.ledger.orders["O1"].state_version,
                    station_id="CELL1",
                )
            )
        self.ledger.advance(50)
        self.assertEqual(OrderState.INPUT_WAITING, self.ledger.orders["O1"].state)

        self.availability(sim_time=50, available=True)
        self.ledger.advance(50)
        self.assertEqual(OrderState.PROCESSING, self.ledger.orders["O1"].state)
        self.ledger.advance(70)
        self.assertEqual(OrderState.OUTPUT_WAITING, self.ledger.orders["O1"].state)

        self.ledger.add_order("O2", "P2", "S2", ready_time=70)
        self.inbound_to_ci("O2", "P2")
        self.ledger.advance(90)
        self.assertEqual(OrderState.INPUT_WAITING, self.ledger.orders["O2"].state)

    def test_repeated_state_is_noop_and_event_id_remains_collision_safe(self) -> None:
        first = self.availability(sim_time=3, available=True)
        self.assertFalse(first.applied)
        self.assertTrue(first.duplicate)
        events_after_noop = self.ledger.events

        duplicate = self.ledger.set_equipment_availability(
            EventId(self.ledger.run_id, "equipment", 1),
            sim_time=3,
            available=True,
        )
        self.assertTrue(duplicate.duplicate)
        self.assertEqual(events_after_noop, self.ledger.events)
        with self.assertRaises(LedgerError):
            self.ledger.set_equipment_availability(
                EventId(self.ledger.run_id, "equipment", 1),
                sim_time=3,
                available=False,
                reason="event id collision",
            )
        with self.assertRaises(LedgerError):
            self.availability(sim_time=2, available=False)
        with self.assertRaisesRegex(LedgerError, "reserved"):
            self.ledger.set_equipment_availability(
                EventId(self.ledger.run_id, "cell", 1),
                sim_time=3,
                available=False,
            )


if __name__ == "__main__":
    unittest.main()
