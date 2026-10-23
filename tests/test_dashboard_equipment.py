"""CELL1 control is an operational-model contract, not a physics claim."""
from pathlib import Path
import tempfile
import unittest

from factory_ops.dashboard import DashboardService, atomic_json, validate_snapshot
from factory_ops.dashboard_model import DashboardModel


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"


class DashboardEquipmentTest(unittest.TestCase):
    def setUp(self) -> None:
        self.service = DashboardService(CONFIG)

    def command(self, key: str, available: bool, reason: str | None = None) -> dict:
        state = self.service.snapshot()
        parameters = {"available": available}
        if reason is not None:
            parameters["reason"] = reason
        return self.service.command({
            "command_id": key,
            "expected_revision": state["revision"],
            "action": "set_equipment_availability",
            "parameters": parameters,
        })

    def advance_until_processing(self) -> dict:
        for _ in range(300):
            self.service.tick(1)
            state = self.service.snapshot()
            if state["equipment"]["active_order_id"] is not None:
                return state
        self.fail("CELL1 did not start processing")

    def test_model_creation_boundary_supports_deterministic_empty_scenarios(self) -> None:
        model = DashboardModel(CONFIG, initial_orders=0, run_id="scenario-baseline")
        state = model.snapshot()
        self.assertEqual("scenario-baseline", state["run_id"])
        self.assertEqual([], state["tasks"])
        self.assertEqual(0, state["kpis"]["pending_tasks"])
        with self.assertRaises(ValueError):
            DashboardModel(CONFIG, initial_orders=-1)
        with self.assertRaises(ValueError):
            DashboardModel(CONFIG, initial_orders=True)
        with self.assertRaises(ValueError):
            DashboardModel(CONFIG, run_id=" ")

    def test_failure_preserves_remaining_service_and_recovery_resumes_it(self) -> None:
        before = self.advance_until_processing()
        order_id = before["equipment"]["active_order_id"]
        receipt = self.command("cell-down", False, "검사기 온도 상한")
        self.assertEqual("applied", receipt["status"])

        down = validate_snapshot(self.service.snapshot())
        self.assertFalse(down["equipment"]["available"])
        self.assertEqual(order_id, down["equipment"]["active_order_id"])
        remaining = down["equipment"]["remaining_service_s"]
        self.assertIsNotNone(remaining)
        assert remaining is not None
        for _ in range(5):
            self.service.tick(1)
        held = self.service.snapshot()
        self.assertEqual(remaining, held["equipment"]["remaining_service_s"])
        self.assertTrue(any("CELL1 중단" in (robot["waiting_reason"] or "") for robot in held["robots"]))
        self.assertTrue(any(event["type"] == "equipment_down" for event in held["events"]))

        recovered = self.command("cell-up", True)
        self.assertEqual("applied", recovered["status"])
        restored = self.service.snapshot()
        self.assertTrue(restored["equipment"]["available"])
        self.assertEqual(remaining, restored["equipment"]["remaining_service_s"])
        self.assertEqual("equipment_recovered", restored["events"][-1]["type"])
        self.assertFalse(any("CELL1 중단" in (robot["waiting_reason"] or "") for robot in restored["robots"]))

        for _ in range(int(remaining) + 2):
            self.service.tick(1)
        self.assertNotEqual(order_id, self.service.snapshot()["equipment"]["active_order_id"])
        assert self.service.model is not None
        self.assertTrue(self.service.model.ledger.replay_matches())

    def test_invalid_or_unsupported_commands_do_not_mutate_equipment(self) -> None:
        before = self.service.snapshot()
        bad_values = [
            {},
            {"available": 0},
            {"available": False, "reason": ""},
            {"available": False, "reason": "x", "station_id": "CELL2"},
        ]
        for index, parameters in enumerate(bad_values):
            receipt = self.service.command({
                "command_id": f"invalid-{index}",
                "expected_revision": before["revision"],
                "action": "set_equipment_availability",
                "parameters": parameters,
            })
            self.assertEqual("rejected", receipt["status"])
            self.assertEqual(before["equipment"], self.service.snapshot()["equipment"])

        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "state.json"
            outbox = Path(folder) / "commands"
            before.update(mode="live", capabilities=[])
            atomic_json(source, before)
            live = DashboardService(CONFIG, source=source, outbox=outbox)
            receipt = live.command({
                "command_id": "unsupported-live-equipment",
                "expected_revision": before["revision"],
                "action": "set_equipment_availability",
                "parameters": {"available": False, "reason": "should-not-queue"},
            })
            self.assertEqual("rejected", receipt["status"])
            self.assertFalse(outbox.exists())


if __name__ == "__main__":
    unittest.main()
