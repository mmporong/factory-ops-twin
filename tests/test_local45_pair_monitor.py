"""CPU-only foreground-client monitoring; no WSL/SDK process is launched."""
from pathlib import Path
import json
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from factory_ops.wsl_process_supervisor import WslProcessSupervisor
from scripts.isaac.run_local45_fleet_pair import monitor_pair, wait_for_bootstrap_marker


class PairMonitorTests(unittest.TestCase):
    def setUp(self):
        self.owner = WslProcessSupervisor("a" * 32)
        self.client = Mock(pid=12345)
        self.client.poll.return_value = None
        self.owner._client = self.client
        # Any accidental repeated unit/cgroup query reproduces the host fault.
        for method in ("inspect", "_call"):
            probe = patch.object(self.owner, method,
                side_effect=subprocess.TimeoutExpired("unexpected WSL status query", 20))
            mocked = probe.start()
            self.addCleanup(probe.stop)
            self.addCleanup(mocked.assert_not_called)

    def test_public_state_observes_only_its_client_even_after_exit(self):
        state = self.owner.client_state()
        self.assertEqual(12345, state["pid"])
        self.assertTrue(state["running"])
        self.assertIsNone(state["returncode"])
        self.assertFalse(state["closed"])
        self.client.wait.assert_not_called()
        self.client.poll.return_value = 7
        self.client.wait.return_value = 7
        exited = self.owner.client_state()
        self.assertFalse(exited["running"])
        self.assertEqual(7, exited["returncode"])
        self.assertTrue(exited["closed"])
        self.assertNotIn("closure_verified", exited)
        self.assertNotIn("unit_state", exited)
        self.client.terminate.assert_not_called()
        self.client.kill.assert_not_called()

    def test_healthy_monitor_never_inspects_unit_and_does_not_guess_closure(self):
        worker = Mock()
        worker.poll.side_effect = [None, None, 0]
        with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", side_effect=[1, 3]), \
                patch("scripts.isaac.run_local45_fleet_pair.time.sleep") as sleep:
            self.assertIsNone(monitor_pair(worker, self.owner, started=0, bound=60))
        self.assertEqual(2, self.client.poll.call_count)
        self.assertEqual(2, sleep.call_count)
        sleep.assert_any_call(2)

    def test_client_exit_allows_sdk_cleanup_without_claiming_linux_closure(self):
        self.client.poll.return_value = 0
        worker = Mock()
        worker.poll.side_effect = [None, None, 0]
        with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", side_effect=[5, 15]), \
                patch("scripts.isaac.run_local45_fleet_pair.time.sleep"):
            observed = monitor_pair(worker, self.owner, started=0, bound=60)
        self.assertIsNotNone(observed)
        assert observed is not None
        self.assertEqual(5, observed["observed_elapsed_s"])
        self.assertEqual(0, observed["client"]["returncode"])
        self.assertIn("Linux closure not inferred", observed["scope"])
        self.assertNotIn("closure_verified", observed)

    def test_client_exit_starts_one_bounded_cleanup_grace(self):
        self.client.poll.return_value = 1
        worker = Mock()
        worker.poll.return_value = None
        with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", side_effect=[0, 89, 91]), \
                patch("scripts.isaac.run_local45_fleet_pair.time.sleep") as sleep, \
                self.assertRaisesRegex(RuntimeError, "within90s"):
            monitor_pair(worker, self.owner, started=0, bound=1000)
        self.assertEqual(2, sleep.call_count)

    def test_pair_deadline_still_preempts_running_client(self):
        worker = Mock()
        worker.poll.return_value = None
        with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", return_value=96), \
                patch("scripts.isaac.run_local45_fleet_pair.time.sleep") as sleep, \
                self.assertRaisesRegex(TimeoutError, "pair owner deadline"):
            monitor_pair(worker, self.owner, started=0, bound=60)
        sleep.assert_not_called()
        self.client.poll.assert_not_called()

    def test_maximum_budget_reserves_cleanup_before_relay_job_can_expire(self):
        worker = Mock()
        worker.poll.return_value = None
        relay = Mock(spec=["assert_healthy"])
        relay.assert_healthy.side_effect = RuntimeError("relay hard timeout must not mask the owner budget")
        with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", return_value=10770), \
                self.assertRaisesRegex(TimeoutError, "pair owner deadline"):
            monitor_pair(worker, self.owner, started=0, bound=10800, relay=relay)
        relay.assert_healthy.assert_not_called()
        self.client.poll.assert_not_called()
        worker.terminate.assert_not_called()

    def test_sdk_primary_failure_preempts_a_live_frozen_ros_client(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder) / "sdk-failure.json"
            marker.write_text(json.dumps({"run_id": "a" * 32, "status": "failed",
                "stage": "mission", "error": {"type": "ValueError",
                "message": "G3 mission-start time is invalid"}}), encoding="utf-8")
            worker = Mock()
            worker.poll.return_value = None
            with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", return_value=5), \
                    patch("scripts.isaac.run_local45_fleet_pair.time.sleep") as sleep, \
                    self.assertRaisesRegex(RuntimeError, "SDK failure before cleanup: mission: ValueError: G3 mission-start"):
                monitor_pair(worker, self.owner, started=0, bound=60,
                             failure_path=marker, run_id="a" * 32)
            sleep.assert_not_called()
            self.client.poll.assert_not_called()
            worker.terminate.assert_not_called()  # Caller finally owns both OS cleanup paths.

    def test_sdk_failure_checkpoint_from_another_run_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder) / "sdk-failure.json"
            marker.write_text(json.dumps({"run_id": "b" * 32, "status": "failed"}), encoding="utf-8")
            worker = Mock()
            worker.poll.return_value = None
            with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", return_value=5), \
                    self.assertRaisesRegex(RuntimeError, "identity/status mismatch"):
                monitor_pair(worker, self.owner, started=0, bound=60,
                             failure_path=marker, run_id="a" * 32)

    def test_bootstrap_marker_is_read_without_unit_queries(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder) / "verified.json"
            with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", side_effect=[0, 0, .2]), \
                    patch("scripts.isaac.run_local45_fleet_pair.time.sleep",
                          side_effect=lambda _: marker.write_text("{}", encoding="utf-8")) as sleep:
                wait_for_bootstrap_marker(marker, self.owner, started=0, bound=60)
            sleep.assert_called_once_with(.2)
            self.assertEqual(2, self.client.poll.call_count)

    def test_bootstrap_rejects_exit_and_preserves_both_deadlines(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder) / "verified.json"
            for now, error, message in ((26, RuntimeError, "bootstrap marker"),
                                        (96, TimeoutError, "pair owner deadline")):
                with self.subTest(now=now), \
                        patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", side_effect=[0, now]), \
                        self.assertRaisesRegex(error, message):
                    wait_for_bootstrap_marker(marker, self.owner, started=0, bound=60)
            marker.write_text("{}", encoding="utf-8")
            self.client.poll.return_value = 1
            with patch("scripts.isaac.run_local45_fleet_pair.time.monotonic", side_effect=[0, 1]), \
                    self.assertRaisesRegex(RuntimeError, "living foreground client"):
                wait_for_bootstrap_marker(marker, self.owner, started=0, bound=60)


if __name__ == "__main__":
    unittest.main()
