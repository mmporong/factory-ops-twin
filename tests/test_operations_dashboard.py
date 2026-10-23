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
