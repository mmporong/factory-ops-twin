"""The scale VM reuse launcher must fail closed around identity and Stop."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from scripts.brev import reuse_scale as reuse


def row(state: str) -> str:
    readiness = "NOT READY" if state == "STOPPED" else "READY"
    return (f"{reuse.NAME} {state} COMPLETED {readiness} {reuse.INSTANCE_ID} "
            f"{reuse.INSTANCE_TYPE} L40S")


class ScaleReuseTest(unittest.TestCase):
    def test_arm_uses_timer_without_wait_and_verified_wsl_keepalive(self):
        calls = []
        keepalive = Mock(pid=4321)
        keepalive.poll.return_value = None

        def wsl(arguments, **kwargs):
            calls.append(arguments)
            if arguments[:3] == ["systemctl", "--user", "show-environment"]:
                return 0, ""
            if arguments[0] == "systemd-run":
                return 0, "Running timer as owned.timer."
            if arguments[3].endswith(".timer"):
                return 0, "LoadState=loaded\nActiveState=active\nControlPID=0\n"
            return 0, ("LoadState=loaded\nActiveState=inactive\n"
                       f"ExecStart={{ path={reuse.BREV[-1]} ; "
                       f"argv[]={reuse.BREV[-1]} stop {reuse.NAME} ; }}\n")

        with patch.object(reuse, "wsl_call", side_effect=wsl), \
                patch.object(reuse.subprocess, "Popen", return_value=keepalive) as popen:
            receipt, process = reuse.arm_watchdog("owned", "deadline")

        arm = next(arguments for arguments in calls if arguments[0] == "systemd-run")
        self.assertNotIn("--wait", arm)
        self.assertIn("--on-active=7200s", arm)
        self.assertIn("--property=Environment=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin", arm)
        self.assertEqual(arm[-3:], [reuse.BREV[-1], "stop", reuse.NAME])
        popen.assert_called_once_with(
            reuse.WSL + ["/usr/bin/sleep", "7500"], stdin=reuse.subprocess.DEVNULL,
            stdout=reuse.subprocess.DEVNULL, stderr=reuse.subprocess.DEVNULL)
        self.assertIs(process, keepalive)
        self.assertEqual(receipt["keepalive_pid"], 4321)
        self.assertIn("Running timer", receipt["arm_output"])

    def test_identity_mismatch_is_untouched(self):
        mismatch = row("STOPPED").replace(reuse.INSTANCE_ID, "different-id")
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(reuse, "target_row", return_value=mismatch), \
                patch.object(reuse, "brev_call") as brev, \
                patch.object(reuse, "wsl_call") as wsl, \
                patch("sys.argv", ["reuse", "--output", str(Path(folder) / "run"), "--execute"]):
            with self.assertRaises(ValueError):
                reuse.main()
            brev.assert_not_called()
            wsl.assert_not_called()
            self.assertFalse((Path(folder) / "run").exists())

    def test_watchdog_verification_failure_prevents_start(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(reuse, "target_row", return_value=row("STOPPED")), \
                patch.object(reuse, "arm_watchdog", side_effect=RuntimeError("timer inactive")), \
                patch.object(reuse, "cleanup_watchdog", return_value={"inactive_confirmed": True}), \
                patch.object(reuse, "brev_call") as brev, \
                patch("sys.argv", ["reuse", "--output", str(Path(folder) / "run"), "--execute"]):
            self.assertEqual(reuse.main(), 1)
            brev.assert_not_called()
            receipt = json.loads((Path(folder) / "run/host.json").read_text(encoding="utf-8"))
            self.assertFalse(receipt["start_attempted"])
            self.assertTrue(receipt["actual_stopped"])
            self.assertTrue(receipt["watchdog_inactive_confirmed"])

    def test_failed_start_still_stops_after_transient_lookup_and_cleans_timer(self):
        brev_calls = []

        def command(arguments, **kwargs):
            brev_calls.append(arguments)
            if arguments[0] == "start":
                raise RuntimeError("provider start failed after request")
            return "Stop accepted"

        rows = [row("STOPPED"), RuntimeError("temporary status failure"), row("STOPPED")]
        cleanup = {"inactive_confirmed": True, "unit_states": {}}
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(reuse, "target_row", side_effect=rows), \
                patch.object(reuse, "arm_watchdog", return_value=({"timer": "owned.timer"}, object())), \
                patch.object(reuse, "cleanup_watchdog", return_value=cleanup) as cleanup_call, \
                patch.object(reuse, "brev_call", side_effect=command), \
                patch.object(reuse.time, "sleep") as sleep, \
                patch("sys.argv", ["reuse", "--output", str(Path(folder) / "run"), "--execute"]):
            self.assertEqual(reuse.main(), 1)
            self.assertEqual(brev_calls, [["start", reuse.NAME], ["stop", reuse.NAME]])
            cleanup_call.assert_called_once()
            receipt = json.loads((Path(folder) / "run/host.json").read_text(encoding="utf-8"))
            self.assertTrue(receipt["start_attempted"])
            self.assertTrue(receipt["actual_stopped"])
            self.assertTrue(receipt["watchdog_inactive_confirmed"])
            self.assertEqual(len(receipt["stop_lookup_errors"]), 1)
            sleep.assert_called_once_with(30)

    def test_starting_waits_for_ready_then_bootstraps_and_confirms_stop(self):
        brev_calls = []
        markers = ("FACTORYOPS_L40S_PAIR_CONFIRMED\n"
                   "FACTORYOPS_SCALE_SDK_INITIALIZED\n"
                   "FACTORYOPS_SCALE_BOOTSTRAP_CLOSED\n")

        def command(arguments, **kwargs):
            brev_calls.append(arguments)
            return markers if arguments[0] == "exec" else "accepted"

        starting = row("RUNNING").replace("RUNNING COMPLETED READY", "STARTING BUILDING NOT READY")
        rows = [row("STOPPED"), starting, row("RUNNING"), row("STOPPED")]
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(reuse, "target_row", side_effect=rows), \
                patch.object(reuse, "arm_watchdog", return_value=({"timer": "owned.timer"}, object())), \
                patch.object(reuse, "cleanup_watchdog", return_value={"inactive_confirmed": True}), \
                patch.object(reuse, "brev_call", side_effect=command), \
                patch.object(reuse.time, "sleep") as sleep, \
                patch("sys.argv", ["reuse", "--output", str(Path(folder) / "run"), "--execute"]):
            self.assertEqual(reuse.main(), 0)
            self.assertEqual([call[0] for call in brev_calls], ["start", "refresh", "exec", "stop"])
            sleep.assert_called_once_with(20)
            receipt = json.loads((Path(folder) / "run/host.json").read_text(encoding="utf-8"))
            self.assertEqual(receipt["status"], "passed")
            self.assertEqual(receipt["readiness_row"], row("RUNNING"))
            self.assertTrue(receipt["actual_stopped"])
            self.assertTrue(receipt["watchdog_inactive_confirmed"])


if __name__ == "__main__":
    unittest.main()
