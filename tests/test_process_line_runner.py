import copy
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from scripts.demo import run_process_line as runner


class ProcessLineRunnerTests(unittest.TestCase):
    def cycle_state(self):
        return {
            "traffic": {"bridge_admission": {"wait_cycle": ["a", "b"]},
                        "deliveries": {"a": 1, "b": 2, "c": 1}},
            "robots": [
                {"id": robot_id, "pose": {"x_m": i, "y_m": 0, "yaw_rad": 0},
                 "task_id": "task-" + robot_id, "payload_id": None, "charged_wh": 0}
                for i, robot_id in enumerate(("a", "b", "c"))
            ],
        }

    def test_external_motion_does_not_reset_stationary_cycle(self):
        state = self.cycle_state()
        original = runner.stationary_cycle_signature(state)
        state["robots"][2]["pose"]["x_m"] += 10
        state["traffic"]["deliveries"]["c"] += 1
        self.assertEqual(original, runner.stationary_cycle_signature(state))

    def test_cycle_member_motion_delivery_and_charging_are_progress(self):
        initial = self.cycle_state()
        for change in ("pose", "delivery", "charge", "task"):
            with self.subTest(change=change):
                state = copy.deepcopy(initial)
                if change == "pose":
                    state["robots"][0]["pose"]["x_m"] += 1
                elif change == "delivery":
                    state["traffic"]["deliveries"]["a"] += 1
                elif change == "charge":
                    state["robots"][0]["charged_wh"] += 1
                else:
                    state["robots"][0]["task_id"] = "next-task"
                self.assertNotEqual(runner.stationary_cycle_signature(initial),
                                    runner.stationary_cycle_signature(state))

    def test_cycle_disappearance_and_order_are_handled(self):
        state = self.cycle_state()
        original = runner.stationary_cycle_signature(state)
        state["traffic"]["bridge_admission"]["wait_cycle"].reverse()
        self.assertEqual(original, runner.stationary_cycle_signature(state))
        state["traffic"]["bridge_admission"]["wait_cycle"] = None
        self.assertIsNone(runner.stationary_cycle_signature(state))

    def test_interrupt_finalizes_real_model_snapshot_and_recording(self):
        original_advance = runner.LineOperationsModel.advance
        advances = 0

        def interrupt_after_second_step(model, dt):
            nonlocal advances
            original_advance(model, dt)
            advances += 1
            if advances == 2:
                raise KeyboardInterrupt

        config = runner.ROOT / "configs/factory-semiconductor-campus16"
        with tempfile.TemporaryDirectory(prefix="factoryops-interrupt-") as folder:
            output = Path(folder) / "run"
            argv = ["run_process_line", "--config", str(config), "--output-dir", str(output),
                    "--lots", "1", "--horizon", "5", "--max-wall-seconds", "30",
                    "--record-start", "0", "--record-duration", "5"]
            with patch.object(sys, "argv", argv), patch.object(
                    runner.LineOperationsModel, "advance", interrupt_after_second_step):
                exit_code = runner.main()
            result_bytes = (output / "result.json").read_bytes()
            result = json.loads(result_bytes)
            manifest = json.loads((output / "recording-manifest.json").read_text())
            self.assertEqual(exit_code, 2)
            self.assertEqual(result["status"], "incomplete")
            self.assertEqual(result["stop_reason"], "diagnostic_interrupt")
            self.assertTrue(result["source_stable_during_run"])
            self.assertAlmostEqual(result["final_snapshot"]["sim_time_s"], 2.0)
            self.assertEqual(manifest["case_status"], "incomplete")
            self.assertEqual(manifest["case_report"]["sha256"], hashlib.sha256(result_bytes).hexdigest())
            with gzip.open(output / "snapshots.jsonl.gz", "rt", encoding="utf-8") as stream:
                rows = [json.loads(row) for row in stream]
            self.assertEqual(len(rows), manifest["frame_count"])
            self.assertEqual(len(rows), 1)

    def test_cycle_timeout_finalizes_incomplete_even_with_external_motion(self):
        # Inject an unchanged component witness while the real model keeps moving.
        witness = (("amr-15", (0, 0, 0), "blocked-a", None, 0, 0),
                   ("amr-16", (1, 0, 0), "blocked-b", None, 0, 0))

        def witness_after_external_motion(state):
            moved = any(distance > 0 for robot_id, distance in state["traffic"]["travel_m"].items()
                        if robot_id not in {"amr-15", "amr-16"})
            return witness if moved else None

        config = runner.ROOT / "configs/factory-semiconductor-campus16"
        with tempfile.TemporaryDirectory(prefix="factoryops-cycle-timeout-") as folder:
            output = Path(folder) / "run"
            argv = ["run_process_line", "--config", str(config), "--output-dir", str(output),
                    "--lots", "1", "--horizon", "60", "--max-wall-seconds", "30",
                    "--cycle-stall-timeout", "2"]
            with patch.object(sys, "argv", argv), patch.object(
                    runner, "stationary_cycle_signature", side_effect=witness_after_external_motion):
                exit_code = runner.main()
            result = json.loads((output / "result.json").read_text(encoding="utf-8"))
            self.assertEqual(exit_code, 2)
            self.assertEqual(result["stop_reason"], "stationary_wait_cycle")
            self.assertEqual(result["stationary_cycle"]["members"], ["amr-15", "amr-16"])
            self.assertGreaterEqual(result["stationary_cycle"]["elapsed_sim_s"], 2)
            self.assertGreater(sum(result["final_snapshot"]["traffic"]["travel_m"].values()), 0)
            self.assertLess(result["final_snapshot"]["sim_time_s"], 60)


if __name__ == "__main__":
    unittest.main()
