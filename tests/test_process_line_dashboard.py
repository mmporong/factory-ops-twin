"""Campus observation and opt-in equipment command binding."""
from __future__ import annotations

from pathlib import Path
from copy import deepcopy
import json
import tempfile
import threading
import unittest
from urllib.request import Request, urlopen

from factory_ops.dashboard import DashboardService, make_server, validate_snapshot


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-semiconductor-campus16"


class ProcessLineDashboardTest(unittest.TestCase):
    def equipment_request(self, state, key, cell_id, available):
        return {"command_id": key, "expected_run_id": state["run_id"],
                "expected_revision": state["revision"], "action": "set_equipment_availability",
                "parameters": {"cell_id": cell_id, "available": available,
                               "reason": "operator scenario" if not available else None}}

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
        self.assertIn("factory_ops/operations_bridge.py",
                      initial["api_producer"]["selected_modules_sha256"])
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
        with self.assertRaisesRegex(ValueError, "local process-line"):
            DashboardService(CONFIG, process_line_controls=True)

    def test_named_cell_fault_recovery_and_replay_preserve_same_run(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            service = DashboardService(CONFIG, execution_mode="process-line", lots=1,
                                       process_line_controls=True, scenario_dir=Path(folder))
            state = validate_snapshot(service.snapshot())
            self.assertEqual(["set_equipment_availability"], state["capabilities"])
            cell_id = state["equipment_cells"][-1]["station_id"]
            request = self.equipment_request(state, "last-cell-down", cell_id, False)
            down_receipt = service.command(request)
            self.assertEqual("applied", down_receipt["status"])
            self.assertEqual(cell_id, down_receipt["effect"]["cell_id"])
            self.assertEqual(down_receipt, service.command(request))
            down = validate_snapshot(service.snapshot())
            self.assertEqual(state["run_id"], down["run_id"])
            self.assertTrue(down["equipment"]["available"])
            self.assertFalse(down["equipment_cells"][-1]["available"])
            self.assertEqual(state["inventory"], down["inventory"])
            self.assertEqual(state["tasks"], down["tasks"])
            self.assertEqual("line_cell_down", down["events"][-1]["type"])
            service.tick(.1)
            saved = service.save_scenario({"name": "last process cell fault"})
            replay = service.replay_frame(saved["scenario_id"], saved["frame_count"] - 1)
            self.assertEqual(down["equipment_cells"], replay["equipment_cells"])
            self.assertEqual([], replay["capabilities"])
            self.assertEqual(down_receipt, replay["commands"][-1])
            recovered = service.command(self.equipment_request(service.snapshot(), "recover", cell_id, True))
            self.assertEqual("applied", recovered["status"])
            self.assertTrue(service.snapshot()["equipment_cells"][-1]["available"])

    def test_bad_cell_run_and_revision_cannot_mutate_equipment(self) -> None:
        service = DashboardService(CONFIG, execution_mode="process-line", lots=1, process_line_controls=True)
        initial = service.snapshot()
        cell_id = initial["equipment_cells"][-1]["station_id"]
        for index, patch in enumerate(({"expected_run_id": "different"}, {"expected_revision": -1},
                                       {"parameters": {"available": False}},
                                       {"parameters": {"cell_id": [], "available": False}},
                                       {"parameters": {"cell_id": "unknown", "available": False}})):
            request = self.equipment_request(initial, f"bad-{index}", cell_id, False)
            request.update(patch)
            self.assertEqual("rejected", service.command(request)["status"])
            self.assertEqual(initial["equipment_cells"], service.snapshot()["equipment_cells"])
        malformed = deepcopy(initial)
        malformed["equipment_cells"].append(deepcopy(malformed["equipment_cells"][-1]))
        with self.assertRaisesRegex(ValueError, "unique"):
            validate_snapshot(malformed)

    def test_http_commands_reach_named_process_cell_and_server_closes(self) -> None:
        service = DashboardService(CONFIG, execution_mode="process-line", lots=1, process_line_controls=True)
        server = make_server(service, 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(base + "/api/snapshot", timeout=5) as response:
                state = json.load(response)
            cell_id = state["equipment_cells"][-1]["station_id"]
            body = self.equipment_request(state, "http-down", cell_id, False)
            request = Request(base + "/api/commands", json.dumps(body).encode(),
                              {"Content-Type": "application/json"})
            with urlopen(request, timeout=5) as response:
                receipt = json.load(response)
            with urlopen(base + "/api/snapshot", timeout=5) as response:
                after = validate_snapshot(json.load(response))
            self.assertEqual(("applied", state["run_id"]), (receipt["status"], after["run_id"]))
            self.assertFalse(after["equipment_cells"][-1]["available"])
            self.assertTrue(after["equipment"]["available"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
