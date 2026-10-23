from pathlib import Path
import tempfile
import unittest

from factory_ops.dashboard import DashboardService, validate_snapshot


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-semiconductor-dual-lane"


class OperationsDashboardTest(unittest.TestCase):
    def command(self, service, key, closed):
        state = service.snapshot()
        return service.command({"command_id": key, "expected_revision": state["revision"],
                                "expected_run_id": state["run_id"], "action": "set_aisle_closed",
                                "parameters": {"closed": closed}})

    def equipment_command(self, service, key, available, reason=None):
        state = service.snapshot()
        parameters = {"available": available}
        if reason is not None:
            parameters["reason"] = reason
        return service.command({"command_id": key, "expected_revision": state["revision"],
                                "expected_run_id": state["run_id"],
                                "action": "set_equipment_availability", "parameters": parameters})

    def test_equipment_fault_receipt_and_state_are_preserved_in_read_only_replay(self):
        with tempfile.TemporaryDirectory() as folder:
            service = DashboardService(CONFIG, execution_mode="operations", scenario_dir=Path(folder))
            service.tick(1)
            run_id = service.snapshot()["run_id"]
            receipt = self.equipment_command(service, "cell-down", False, "검사기 온도 상한")
            self.assertEqual((receipt["run_id"], receipt["status"]), (run_id, "applied"))
            down = validate_snapshot(service.snapshot())
            self.assertFalse(down["equipment"]["available"])
            self.assertEqual(down["equipment"]["reason"], "검사기 온도 상한")
            self.assertEqual(down["commands"][-1], receipt)
            saved = service.save_scenario({"name": "CELL1 equipment fault"})
            replay = service.replay_frame(saved["scenario_id"], saved["frame_count"] - 1)
            self.assertEqual(replay["run_id"], run_id)
            self.assertEqual(replay["equipment"], down["equipment"])
            self.assertEqual(replay["commands"], down["commands"])
            self.assertEqual(replay["capabilities"], [])

    def test_closure_receipt_waits_for_drain_and_replay_preserves_scope(self):
        with tempfile.TemporaryDirectory() as folder:
            service = DashboardService(CONFIG, execution_mode="operations", scenario_dir=Path(folder))
            service.tick(1)
            receipt = self.command(service, "close", True)
            self.assertEqual(receipt["status"], "queued")
            queued = validate_snapshot(service.snapshot())
            self.assertTrue(queued["traffic"]["closure_pending"])
            for _ in range(1000):
                service.tick(1)
                state = service.snapshot()
                if state["traffic"]["closed"]:
                    break
            self.assertTrue(state["traffic"]["closed"])
            self.assertEqual(state["commands"][-1]["status"], "applied")
            self.assertEqual(state["execution_mode"], "operations")
            saved = service.save_scenario({"name": "planned aisle closure"})
            replay = service.replay_frame(saved["scenario_id"], saved["frame_count"] - 1)
            self.assertEqual(replay["execution_mode"], "operations")
            self.assertEqual(replay["dispatch_policy"], "fifo")
            self.assertEqual(replay["traffic"], state["traffic"])
            self.assertEqual(replay["equipment"], state["equipment"])
            self.assertEqual(replay["capabilities"], [])

    def test_superseded_closure_is_rejected_and_preview_cannot_request_it(self):
        service = DashboardService(CONFIG, execution_mode="operations", dispatch_policy="eta")
        service.tick(1)
        self.assertEqual(self.command(service, "close", True)["status"], "queued")
        self.assertEqual(self.command(service, "reopen", False)["status"], "applied")
        receipts = {r["command_id"]: r for r in validate_snapshot(service.snapshot())["commands"]}
        self.assertEqual(receipts["close"]["status"], "rejected")
        preview = DashboardService(CONFIG)
        self.assertEqual(self.command(preview, "not-supported", True)["status"], "rejected")
        with self.assertRaises(ValueError):
            DashboardService(CONFIG, execution_mode="operations", source=Path("unused.json"))


if __name__ == "__main__":
    unittest.main()
