"""One OS cleanup failure must not skip the other OS or final evidence."""
import json
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock

from scripts.isaac.run_local45_fleet_pair import close_pair


class PairCleanupTests(unittest.TestCase):
    def test_windows_and_log_failures_still_stop_linux_and_write_failed_receipt(self):
        with tempfile.TemporaryDirectory() as temporary:
            stage = Path(temporary)
            worker = Mock()
            worker.poll.return_value = None
            worker.terminate.side_effect = OSError("terminate denied")
            worker.wait.side_effect = subprocess.TimeoutExpired("owned worker", 10)
            stream = Mock()
            stream.close.side_effect = OSError("log flush failed")
            owner = Mock()
            owner.stop.return_value = {"closure_verified": True}
            report = {"run_id": "a" * 32, "status": "running", "windows_closed": False, "linux_closed": False}
            close_pair(worker, [stream], owner, stage, report, time.monotonic())
            owner.stop.assert_called_once()
            worker.wait.assert_called_once()
            saved = json.loads((stage / "pair-host.json").read_text(encoding="utf-8"))
            self.assertEqual("failed", saved["status"])
            self.assertTrue(saved["linux_closed"])
            self.assertFalse(saved["windows_closed"])
            self.assertEqual(4, len(saved["cleanup_errors"]))

    def test_linux_failure_does_not_skip_windows_receipt_and_final_save(self):
        with tempfile.TemporaryDirectory() as temporary:
            stage = Path(temporary)
            owner = Mock()
            owner.stop.side_effect = RuntimeError("user manager unavailable")
            (stage / "job.json").write_text(json.dumps({"run_id": "a" * 32, "closed": True,
                "remaining_owned_pids": [], "status": "passed"}), encoding="utf-8")
            report = {"run_id": "a" * 32, "status": "passed", "windows_closed": False, "linux_closed": False}
            close_pair(None, [], owner, stage, report, time.monotonic())
            saved = json.loads((stage / "pair-host.json").read_text(encoding="utf-8"))
            self.assertEqual("failed", saved["status"])
            self.assertTrue(saved["windows_closed"])
            self.assertFalse(saved["linux_closed"])


if __name__ == "__main__":
    unittest.main()
