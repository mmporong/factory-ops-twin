import json
from pathlib import Path
import unittest

from factory_ops.process_line import ProcessLine


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "semiconductor-process-line.json"


class ProcessLineTests(unittest.TestCase):
    def setUp(self):
        self.line = ProcessLine("run-1", json.loads(CONFIG.read_text(encoding="utf-8")))
        self.clock = 0.0
        self.observation = 0

    def move(self, task_id: str, robot: str = "amr-1") -> None:
        self.line.assign(task_id, robot)
        self.observation += 1
        self.line.pickup("run-1", task_id, robot, f"obs-{self.observation}", self.clock)
        self.observation += 1
        self.line.drop("run-1", task_id, robot, f"obs-{self.observation}", self.clock)

    def advance(self, seconds: float) -> None:
        self.clock += seconds
        self.line.advance(self.clock)

    def finish_lot(self, task_id: str) -> None:
        self.move(task_id)
        for cell_id in ("TEST", "AOI", "PACK"):
            self.advance(self.line.cells[cell_id].spec.processing_time_s)
            self.move(self.line.ready_transports[0].task_id)
        self.move(self.line.ready_transports[0].task_id)

    def test_lot_crosses_three_cells_and_counts_return_only_after_empty_carrier_drop(self):
        task = self.line.release_lot("LOT-1", "CARRIER-1", "S1")
        with self.assertRaisesRegex(ValueError, "unique"):
            self.line.release_lot("LOT-1", "CARRIER-2", "S2")
        self.move(task)
        for cell_id in ("TEST", "AOI", "PACK"):
            self.advance(self.line.cells[cell_id].spec.processing_time_s)
            ready = self.line.ready_transports
            self.assertEqual(1, len(ready))
            self.assertEqual("CARRIER-1", ready[0].carrier_id)
            self.move(ready[0].task_id)
        state = self.line.snapshot()
        self.assertEqual(1, state["completed_lots"])
        self.assertEqual(0, state["returned"])
        return_task = self.line.ready_transports[0]
        self.assertEqual(("OUT", "S1", "empty_return"),
                         (return_task.origin, return_task.destination, return_task.kind))
        self.move(return_task.task_id)
        self.assertEqual(1, self.line.snapshot()["returned"])
        drops = [event for event in self.line.snapshot()["events"] if event["kind"] == "transport_dropped"]
        self.assertEqual(["TEST_IN", "AOI_IN", "PACK_IN", "OUT", "S1"],
                         [event["destination"] for event in drops])

    def test_completed_lot_count_stays_monotonic_during_empty_return_transport(self):
        task = self.line.release_lot("LOT-1", "CARRIER-1", "S1")
        self.move(task)
        for cell_id in ("TEST", "AOI", "PACK"):
            self.advance(self.line.cells[cell_id].spec.processing_time_s)
            self.move(self.line.ready_transports[0].task_id)

        return_task = self.line.ready_transports[0]
        self.assertEqual((1, 0),
                         (self.line.snapshot()["completed_lots"], self.line.snapshot()["returned"]))
        self.line.assign(return_task.task_id, "amr-1")
        self.observation += 1
        self.line.pickup(
            "run-1", return_task.task_id, "amr-1", f"obs-{self.observation}", self.clock,
        )
        in_return = self.line.snapshot()
        self.assertEqual("in_transport", in_return["lots"][0]["state"])
        self.assertEqual((1, 0), (in_return["completed_lots"], in_return["returned"]))

        self.observation += 1
        self.line.drop(
            "run-1", return_task.task_id, "amr-1", f"obs-{self.observation}", self.clock,
        )
        returned = self.line.snapshot()
        self.assertEqual((1, 1), (returned["completed_lots"], returned["returned"]))

    def test_input_slot_assignment_is_atomic_and_wrong_robot_cannot_commit(self):
        first = self.line.release_lot("LOT-1", "CARRIER-1", "S1")
        second = self.line.release_lot("LOT-2", "CARRIER-2", "S2")
        self.line.assign(first, "amr-1")
        with self.assertRaisesRegex(ValueError, "capacity"):
            self.line.assign(second, "amr-2")
        before = self.line.snapshot()
        with self.assertRaisesRegex(ValueError, "bound"):
            self.line.pickup("run-1", first, "amr-2", "wrong-robot", self.clock)
        self.assertEqual(before, self.line.snapshot())
        self.line.pickup("run-1", first, "amr-1", "pickup-first", self.clock)
        self.line.drop("run-1", first, "amr-1", "drop-first", self.clock)
        self.line.assign(second, "amr-2")
        self.assertEqual(1, self.line.cells["TEST"].reserved_inputs)

    def test_failure_preserves_remaining_time_and_full_output_blocks_next_completion(self):
        first = self.line.release_lot("LOT-1", "CARRIER-1", "S1")
        self.move(first)
        self.advance(10)
        self.line.set_available("TEST", False)
        remaining = self.line.cells["TEST"].remaining_s
        self.advance(100)
        self.assertEqual(remaining, self.line.cells["TEST"].remaining_s)
        self.line.set_available("TEST", True)
        self.advance(remaining)
        first_move = self.line.ready_transports[0]
        second = self.line.release_lot("LOT-2", "CARRIER-2", "S2")
        self.move(second)
        self.advance(30)
        test = self.line.cells["TEST"]
        self.assertEqual("LOT-2", test.active_lot)
        self.assertEqual(0, test.remaining_s)
        self.assertEqual(["LOT-1"], test.output_lots)
        self.move(first_move.task_id)
        self.assertEqual(["LOT-2"], self.line.cells["TEST"].output_lots)

    def test_observation_retry_is_idempotent_and_future_or_second_drop_cannot_mutate(self):
        task = self.line.release_lot("LOT-1", "CARRIER-1", "S1")
        self.line.assign(task, "amr-1")
        pickup = self.line.pickup("run-1", task, "amr-1", "pickup", self.clock)
        before_retry = self.line.snapshot()
        duplicate = self.line.pickup("run-1", task, "amr-1", "pickup", self.clock)
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(before_retry, self.line.snapshot())
        with self.assertRaisesRegex(ValueError, "later"):
            self.line.drop("run-1", task, "amr-1", "future", self.clock + 1)
        self.line.drop("run-1", task, "amr-1", "drop", self.clock)
        after = self.line.snapshot()
        with self.assertRaisesRegex(ValueError, "bound"):
            self.line.drop("run-1", task, "amr-1", "second-drop", self.clock)
        self.assertEqual(after, self.line.snapshot())
        self.assertFalse(pickup["duplicate"])

    def test_returned_carrier_can_be_reused_only_from_its_returned_source(self):
        first = self.line.release_lot("LOT-1", "CARRIER-1", "S1")
        with self.assertRaisesRegex(ValueError, "active"):
            self.line.release_lot("LOT-EARLY", "CARRIER-1", "S1")
        self.finish_lot(first)
        first_state = next(lot for lot in self.line.snapshot()["lots"] if lot["lot_id"] == "LOT-1")
        with self.assertRaisesRegex(ValueError, "current source"):
            self.line.release_lot("LOT-WRONG-1", "CARRIER-1", "S2")
        second = self.line.release_lot("LOT-2", "CARRIER-1", "S1")
        state = self.line.snapshot()
        self.assertEqual(1, state["active_carrier_count"])
        self.assertEqual({"CARRIER-1": "LOT-2"}, state["active_carriers"])
        self.assertEqual(first_state, next(lot for lot in state["lots"] if lot["lot_id"] == "LOT-1"))
        self.finish_lot(second)
        with self.assertRaisesRegex(ValueError, "current source"):
            self.line.release_lot("LOT-WRONG-2", "CARRIER-1", "S3")
        final = self.line.snapshot()
        self.assertEqual(0, final["active_carrier_count"])
        self.assertEqual({"CARRIER-1": "S1"}, final["available_carriers"])
        self.assertEqual("returned", next(lot for lot in final["lots"] if lot["lot_id"] == "LOT-1")["state"])


if __name__ == "__main__":
    unittest.main()
