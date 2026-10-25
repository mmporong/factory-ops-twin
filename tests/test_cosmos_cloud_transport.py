"""CPU-only regressions for the SSH transport boundary."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch


BREV_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "brev"
sys.path.insert(0, str(BREV_SCRIPTS))
try:
    spec = importlib.util.spec_from_file_location(
        "cosmos_transport_under_test", BREV_SCRIPTS / "run_cosmos_smoke.py"
    )
    supervisor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(supervisor)
finally:
    sys.path.remove(str(BREV_SCRIPTS))


class CosmosSshTransportTests(unittest.TestCase):
    def test_success_returns_stdout_without_known_host_stderr(self) -> None:
        reply = subprocess.CompletedProcess(
            args=["ssh"], returncode=0, stdout=b"29360128\n",
            stderr=b"Warning: Permanently added host to known hosts.\n",
        )
        with patch.object(supervisor.subprocess, "run", return_value=reply):
            returncode, stdout = supervisor._ssh("du -sb output")
        self.assertEqual(returncode, 0)
        self.assertEqual(stdout, "29360128\n")

    def test_failure_preserves_stderr_diagnostic(self) -> None:
        reply = subprocess.CompletedProcess(
            args=["ssh"], returncode=255, stdout=b"", stderr=b"connection timed out\n"
        )
        with patch.object(supervisor.subprocess, "run", return_value=reply):
            with self.assertRaisesRegex(RuntimeError, "connection timed out"):
                supervisor._ssh("nvidia-smi")


if __name__ == "__main__":
    unittest.main()
