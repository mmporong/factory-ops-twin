"""Read-only dashboard binding for the campus ProcessNetwork model."""
from __future__ import annotations

from pathlib import Path
import unittest

from factory_ops.dashboard import DashboardService, validate_snapshot


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-semiconductor-campus16"


class ProcessLineDashboardTest(unittest.TestCase):
    def test_campus_process_network_exposes_real_fleet_lines_and_lots(self) -> None:
        service = DashboardService(CONFIG, execution_mode="process-line", lots=48)
        initial = validate_snapshot(service.snapshot())
        self.assertEqual("operations", initial["execution_mode"])
        self.assertEqual(16, len(initial["robots"]))
        self.assertEqual(48, len(initial["production_line"]["lots"]))
        self.assertEqual(12, len(initial["production_line"]["cells"]))
        self.assertEqual(["L1", "L2", "L3", "L4"], initial["production_line"]["line_ids"])
        self.assertEqual([], initial["capabilities"])
        self.assertNotIn("integrations", initial)
        service.tick(.25)
        advanced = validate_snapshot(service.snapshot())
        self.assertEqual(.25, advanced["sim_time_s"])
        self.assertEqual(initial["run_id"], advanced["run_id"])

    def test_process_line_is_read_only_and_rejects_command_outbox(self) -> None:
        service = DashboardService(CONFIG, execution_mode="process-line", lots=1)
        state = service.snapshot()
        receipt = service.command({"command_id": "pause", "expected_revision": state["revision"],
                                   "action": "pause"})
        self.assertEqual("rejected", receipt["status"])
        self.assertIn("지원하지 않는", receipt["reason"])
        with self.assertRaisesRegex(ValueError, "read-only"):
            DashboardService(CONFIG, execution_mode="process-line", outbox=Path("commands"))

    def test_process_line_lot_bounds_and_mode_are_fail_closed(self) -> None:
        for lots in (0, 65, True, 1.5):
            with self.subTest(lots=lots), self.assertRaisesRegex(ValueError, "1 to 64"):
                DashboardService(CONFIG, execution_mode="process-line", lots=lots)  # type: ignore[arg-type]
        with self.assertRaisesRegex(ValueError, "preview, operations or process-line"):
            DashboardService(CONFIG, execution_mode="unknown")


if __name__ == "__main__":
    unittest.main()
