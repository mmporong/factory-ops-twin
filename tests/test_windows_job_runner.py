"""Exercise actual descendant containment without starting Isaac Sim."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import uuid

from scripts.isaac import windows_job_runner as runner

RUNNER = Path(__file__).resolve().parents[1] / "scripts/isaac/windows_job_runner.py"


@unittest.skipUnless(os.name == "nt", "Windows kernel containment test")
class WindowsJobRunnerTests(unittest.TestCase):
    def invoke(self, child_code, timeout=3):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            pid_file = directory / "descendant.pid"
            spec = directory / "spec.json"
            spec.write_text(json.dumps({"run_id": uuid.uuid4().hex, "cwd": str(directory),
                "command": [sys.executable, "-c", child_code, str(pid_file)],
                "expected_inputs": {str(RUNNER): hashlib.sha256(RUNNER.read_bytes()).hexdigest()}}), encoding="utf-8")
            receipt = directory / "job.json"
            actual = subprocess.run([sys.executable, str(RUNNER), "--spec", str(spec),
                "--receipt", str(receipt), "--timeout", str(timeout)],
                capture_output=True, text=True, timeout=20)
            result = json.loads(receipt.read_text(encoding="utf-8"))
            pid = int(pid_file.read_text()) if pid_file.exists() else None
            if pid is not None:
                api = ctypes.WinDLL("kernel32", use_last_error=True)
                api.OpenProcess.argtypes = (ctypes.c_uint, ctypes.c_bool, ctypes.c_uint)
                api.OpenProcess.restype = ctypes.c_void_p
                api.WaitForSingleObject.argtypes = (ctypes.c_void_p, ctypes.c_uint)
                api.CloseHandle.argtypes = (ctypes.c_void_p,)
                handle = api.OpenProcess(0x100000, False, pid)
                if handle:
                    self.assertEqual(api.WaitForSingleObject(handle, 0), 0, result)
                    api.CloseHandle(handle)
            self.assertTrue(result["assignment_confirmed"], actual.stderr)
            self.assertTrue(result["gate_opened"])
            self.assertTrue(result["prelaunch_inputs_verified"], result)
            self.assertTrue(result["closed"], result)
            self.assertEqual(result["remaining_owned_pids"], [])
            return actual, result, pid

    def test_normal_exit(self):
        actual, result, _ = self.invoke("print('bounded child complete')")
        self.assertEqual(actual.returncode, 0, actual.stderr)
        self.assertEqual(result["status"], "passed")
        self.assertIn("bounded child complete", actual.stdout)

    def test_exited_parent_does_not_leave_unnamed_grandchild(self):
        code = ("import subprocess,sys; from pathlib import Path; "
            "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
            "Path(sys.argv[1]).write_text(str(child.pid))")
        actual, result, pid = self.invoke(code)
        self.assertEqual(actual.returncode, 0, actual.stderr)
        self.assertIn(pid, result["owned_pids_before_cleanup"])

    def test_timeout_kills_entire_owned_tree(self):
        code = ("import subprocess,sys,time; from pathlib import Path; "
            "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); "
            "Path(sys.argv[1]).write_text(str(child.pid)); time.sleep(60)")
        actual, result, pid = self.invoke(code, timeout=1)
        self.assertEqual(actual.returncode, 1, actual.stderr)
        self.assertTrue(result["timed_out"])
        self.assertIn(pid, result["owned_pids_before_cleanup"])
        self.assertGreaterEqual(len(result["owned_pids_before_cleanup"]), 2)


class PrelaunchInputTests(unittest.TestCase):
    def test_changed_source_config_or_dependency_never_launches_command(self):
        for kind in ("source", "config", "dependency"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source, spec, gate, proof = (root/name for name in (kind, "spec.json", "gate", "proof.json"))
                source.write_text("approved", encoding="utf-8")
                value = {"run_id": "bounded", "command": ["never-launch"], "cwd": str(root),
                    "expected_inputs": {str(source): hashlib.sha256(source.read_bytes()).hexdigest()}}
                spec.write_text(json.dumps(value), encoding="utf-8")
                captured = hashlib.sha256(spec.read_bytes()).hexdigest()
                gate.write_text("bounded", encoding="ascii")
                source.write_text("changed", encoding="utf-8")
                with patch.object(runner.subprocess, "call") as launch:
                    with self.assertRaisesRegex(ValueError, "prelaunch input differs"):
                        runner.gated_worker(spec, gate, captured, proof)
                    launch.assert_not_called()
                self.assertEqual(json.loads(proof.read_text(encoding="utf-8"))["status"], "failed")

    def test_changed_spec_never_launches_command(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            spec, gate, proof = (root/name for name in ("spec.json", "gate", "proof.json"))
            spec.write_text('{"run_id":"approved"}', encoding="utf-8")
            captured = hashlib.sha256(spec.read_bytes()).hexdigest()
            spec.write_text('{"run_id":"changed"}', encoding="utf-8")
            with patch.object(runner.subprocess, "call") as launch:
                with self.assertRaisesRegex(ValueError, "spec differs"):
                    runner.gated_worker(spec, gate, captured, proof)
                launch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
