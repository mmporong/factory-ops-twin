"""CPU-only regressions for the paid VM's readiness admission boundary."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch


BREV_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "brev"
sys.path.insert(0, str(BREV_SCRIPTS))
try:
    spec = importlib.util.spec_from_file_location("cosmos_readiness_under_test", BREV_SCRIPTS / "run_cosmos_smoke.py")
    supervisor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(supervisor)
finally:
    sys.path.remove(str(BREV_SCRIPTS))


def observation(status: str, shell: str, health: str) -> dict:
    return {"status": status, "build_status": "COMPLETED", "shell_status": shell, "health_status": health}


class Clock:
    now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class CosmosReadinessTests(unittest.TestCase):
    def test_only_batch_388_gets_extended_ready_timeout(self) -> None:
        self.assertEqual(supervisor._ready_timeout_seconds(387), 600)
        self.assertEqual(supervisor._ready_timeout_seconds(388), 1200)

    def test_booting_health_does_not_admit_gpu_or_abort_before_full_readiness(self) -> None:
        states = [observation("STOPPED", "NOT READY", "UNAVAILABLE"),
                  observation("STARTING", "NOT READY", "UNHEALTHY"),
                  observation("RUNNING", "NOT READY", "UNHEALTHY"),
                  observation("RUNNING", "READY", "HEALTHY")]
        clock, record = Clock(), {}
        with patch.object(supervisor, "_load_listing", side_effect=[({}, state) for state in states]), \
             patch.object(supervisor.time, "monotonic", clock.monotonic), \
             patch.object(supervisor.time, "sleep", clock.sleep):
            supervisor._poll_ready(1000.0, record, lambda: None)
        self.assertEqual(len(record["readiness_polls"]), 4)
        self.assertEqual(clock.now, 3 * supervisor.POLL_SECONDS)
        self.assertEqual(record["readiness_polls"][-1]["health_status"], "HEALTHY")

    def test_not_ready_remains_blocked_and_reaches_deadline(self) -> None:
        clock, record = Clock(), {}
        with patch.object(supervisor, "_load_listing", return_value=({}, observation("RUNNING", "NOT READY", "UNHEALTHY"))), \
             patch.object(supervisor.time, "monotonic", clock.monotonic), \
             patch.object(supervisor.time, "sleep", clock.sleep):
            with self.assertRaisesRegex(TimeoutError, "did not become"):
                supervisor._poll_ready(supervisor.STOP_RESERVE_SECONDS + 2 * supervisor.POLL_SECONDS, record, lambda: None)
        self.assertEqual(clock.now, 2 * supervisor.POLL_SECONDS)

    def test_running_ready_health_telemetry_does_not_replace_hardware_probe(self) -> None:
        record = {}
        with patch.object(supervisor, "_load_listing", return_value=({}, observation("RUNNING", "READY", "UNHEALTHY"))):
            supervisor._poll_ready(10**12, record, lambda: None)
        self.assertEqual(record["readiness_polls"][-1]["health_status"], "UNHEALTHY")

    def test_terminal_instance_status_still_fails_immediately(self) -> None:
        with patch.object(supervisor, "_load_listing", return_value=({}, observation("FAILED", "READY", "UNHEALTHY"))), \
             patch.object(supervisor.time, "sleep") as sleep:
            with self.assertRaises(RuntimeError):
                supervisor._poll_ready(10**12, {}, lambda: None)
        sleep.assert_not_called()

    def test_displayed_unhealthy_ready_requires_hardware_probe_next(self) -> None:
        record = {}
        displayed = observation("UNHEALTHY", "READY", "UNHEALTHY")
        with patch.object(supervisor, "_load_listing", return_value=({}, displayed)), \
             patch.object(supervisor.time, "sleep") as sleep:
            supervisor._poll_ready(10**12, record, lambda: None)
        self.assertEqual(record["readiness_polls"][-1]["status"], "UNHEALTHY")
        sleep.assert_not_called()

    def test_live_keepalive_prevents_owned_processes_gone_receipt(self) -> None:
        keepalive = Mock()
        keepalive.poll.return_value = None
        self.assertFalse(supervisor._owned_processes_gone({}, keepalive))

    def test_stopped_keepalive_and_remote_ssh_allow_gone_receipt(self) -> None:
        keepalive = Mock()
        keepalive.poll.return_value = 0
        record = {"remote_ssh_pid": 42, "remote_ssh_gone": True}
        self.assertTrue(supervisor._owned_processes_gone(record, keepalive))


if __name__ == "__main__":
    unittest.main()
