"""Prelaunch resource-range admission and one real, tiny CPU Windows Job."""
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

from scripts.isaac import run_local45_fleet_pair as pair
from scripts.isaac import windows_job_runner as runner


RUNNER = Path(runner.__file__).resolve()


@unittest.skipUnless(os.name == "nt", "Windows-specific prelaunch/Job boundary")
class RuntimeBudgetTests(unittest.TestCase):
    def test_pair_existing_and_future_bounds_reach_unused_input_guard_without_mutation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            # The absent launch.json stops accepted inputs before any owner/process creation.
            for bound in (60, 450, 7200, 9000, 10800):
                with self.subTest(bound=bound), patch.object(pair, "WslProcessSupervisor") as owner:
                    with self.assertRaises(FileNotFoundError):
                        pair.run(root, root, bound)
                    owner.assert_not_called()
                    self.assertEqual(list(root.iterdir()), [])

    def test_invalid_pair_bounds_are_rejected_before_even_resolving_paths(self):
        for bound in (59, 10801, math.nan, math.inf, -math.inf):
            with self.subTest(bound=bound), patch.object(Path, "resolve") as resolve:
                with self.assertRaisesRegex(ValueError, "60..10800s bound"):
                    pair.run(Path("unused"), Path("unused"), bound)
                resolve.assert_not_called()

    def test_invalid_operation_story_mode_is_rejected_before_resolving_paths(self):
        with patch.object(Path, "resolve") as resolve:
            with self.assertRaisesRegex(ValueError, "operation story mode"):
                pair.run(Path("unused"), Path("unused"), 60, operation_story_mode="invalid")
            resolve.assert_not_called()

    def test_operation_story_receipt_never_claims_disabled_or_unvalidated_media_complete(self):
        off = pair._operation_story_receipt({"scene_extension": {"operation_story": {
            "requested_mode": "off", "enabled": False}}}, "off")
        self.assertEqual(off, {"requested_mode": "off", "actual_enabled": False,
            "final_media_status": "not_collected"})
        on = pair._operation_story_receipt({"scene_extension": {"operation_story": {
            "requested_mode": "on", "enabled": True}}}, "on")
        self.assertEqual("collection_enabled_not_validated", on["final_media_status"])
        with self.assertRaisesRegex(ValueError, "differs from the pair request"):
            pair._operation_story_receipt({"scene_extension": {"operation_story": {
                "requested_mode": "off", "enabled": True}}}, "off")
        with self.assertRaisesRegex(ValueError, "differs from the pair request"):
            pair._operation_story_receipt({"scene_extension": {"operation_story": {
                "requested_mode": "on", "enabled": False}}}, "off")

    def test_job_invalid_bounds_and_existing_receipts_never_create_owned_artifacts(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec = root/"spec.json"
            spec.write_text(json.dumps({"run_id": uuid4().hex}), encoding="utf-8")
            receipt = root/"job.json"
            for bound in (0, 10800.001, 10801, math.nan, math.inf, -math.inf):
                with self.subTest(bound=bound), patch.object(runner, "Job") as job:
                    with self.assertRaises(ValueError):
                        runner.run(spec, receipt, bound)
                    job.assert_not_called()
                    self.assertEqual(list(root.iterdir()), [spec])
            for path in (receipt, root/"job.json.gate", root/"job.json.preflight.json"):
                path.write_text("preserve-existing-evidence", encoding="ascii")
                with self.subTest(existing=path.name), patch.object(runner, "Job") as job:
                    with self.assertRaises(ValueError):
                        runner.run(spec, receipt, 9000)
                    job.assert_not_called()
                    self.assertEqual(path.read_text(encoding="ascii"), "preserve-existing-evidence")
                path.unlink()

    def test_real_9000_budget_job_exits_promptly_and_closes_its_descendant(self):
        supplied = os.environ.get("FACTORY_OPS_RUNTIME_BUDGET_EVIDENCE")
        if supplied:
            root = Path(supplied)
            self.assertTrue(root.is_absolute())
            self.assertFalse(root.exists(), "preserve prior fixture evidence")
            root.mkdir(parents=True)
        else:
            root = Path(tempfile.mkdtemp(prefix="factory-ops-runtime-budget-081-"))
        pid_path, spec, receipt = root/"descendant.pid", root/"spec.json", root/"job.json"
        code = ("import subprocess,sys; from pathlib import Path; "
            "child=subprocess.Popen([sys.executable,'-B','-c','import time; time.sleep(60)'],"
            "stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); "
            "Path(sys.argv[1]).write_text(str(child.pid)); print('cpu parent exits; Job owns descendant')")
        spec.write_text(json.dumps({"run_id": uuid4().hex, "cwd": str(root),
            "command": [sys.executable, "-B", "-c", code, str(pid_path)],
            "expected_inputs": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (RUNNER, Path(__file__).resolve())}}), encoding="utf-8")
        started = time.monotonic()
        actual = subprocess.run([sys.executable, "-B", str(RUNNER), "--spec", str(spec),
            "--receipt", str(receipt), "--timeout", "9000"], capture_output=True, timeout=20)
        elapsed = time.monotonic()-started
        (root/"runner.stdout.log").write_bytes(actual.stdout)
        (root/"runner.stderr.log").write_bytes(actual.stderr)
        result = json.loads(receipt.read_text(encoding="utf-8"))
        pid = int(pid_path.read_text(encoding="ascii"))
        api = ctypes.WinDLL("kernel32", use_last_error=True)
        api.OpenProcess.argtypes = (ctypes.c_uint, ctypes.c_bool, ctypes.c_uint)
        api.OpenProcess.restype = ctypes.c_void_p
        api.WaitForSingleObject.argtypes = (ctypes.c_void_p, ctypes.c_uint)
        api.WaitForSingleObject.restype = ctypes.c_uint
        api.CloseHandle.argtypes = (ctypes.c_void_p,)
        api.CloseHandle.restype = ctypes.c_bool
        handle = api.OpenProcess(0x100000, False, pid)
        if handle:
            try:
                wait_result = int(api.WaitForSingleObject(handle, 0))
                gone = wait_result == 0
                pid_check = {"pid": pid, "handle_opened": True, "wait_result": wait_result, "gone": gone}
            finally:
                api.CloseHandle(handle)
        else:
            error = ctypes.get_last_error()
            gone = error == 87  # ERROR_INVALID_PARAMETER: no current process with this PID.
            pid_check = {"pid": pid, "handle_opened": False, "winerror": error, "gone": gone}
        summary = {"run_id": result["run_id"], "requested_timeout_s": 9000, "actual_exit_code": actual.returncode,
            "observed_wall_s": elapsed, "descendant_check": pid_check, "job_receipt": str(receipt),
            "job_status": result["status"], "closed": result["closed"],
            "remaining_owned_pids": result["remaining_owned_pids"], "scope": "CPU Job only; no SDK/ROS/WSL"}
        (root/"fixture.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
        print(json.dumps({"runtime_budget_fixture": str(root), **summary}))
        self.assertEqual(actual.returncode, 0, actual.stderr)
        self.assertEqual(result["status"], "passed", result)
        self.assertTrue(result["assignment_confirmed"] and result["gate_opened"] and result["prelaunch_inputs_verified"], result)
        self.assertTrue(result["kill_on_job_close"] and result["closed"], result)
        self.assertFalse(result["timed_out"] or result["abort_requested"], result)
        self.assertEqual(result["remaining_owned_pids"], [])
        self.assertIn(pid, result["owned_pids_before_cleanup"])
        self.assertTrue(gone, pid_check)
        self.assertLess(elapsed, 20)


if __name__ == "__main__":
    unittest.main()
