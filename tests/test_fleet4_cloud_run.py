"""Reject stale identity/budget/storage inputs without spending cloud credits."""
from datetime import datetime, timedelta, timezone
from contextlib import ExitStack
import json
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.brev import fleet4_cloud_run as caller


class CloudPreflightTests(unittest.TestCase):
    def observation(self):
        return {"observed_at_utc": datetime.now(timezone.utc).isoformat(), "instance_id": caller.INSTANCE_ID,
                "state": "STOPPED", "batch_budget_usd": 15, "rate_ceiling_estimate_usd_per_hour": 4.2,
                "displayed_credit_usd": 336.43}

    def check(self, observation):
        with patch.object(caller.shutil, "disk_usage", return_value=SimpleNamespace(free=30 * caller.GIB)), \
             patch.object(caller.subprocess, "check_output", return_value=b""):
            caller.preflight("fleet4-transport-normal-001", observation)

    def test_fresh_existing_instance_has_a_read_only_preflight(self):
        with patch.object(caller, "command") as cloud:
            self.check(self.observation())
            cloud.assert_not_called()

    def test_wrong_instance_budget_low_credit_and_nonfinite_credit_are_rejected(self):
        for change in ({"instance_id": "another-instance"}, {"batch_budget_usd": 50},
                       {"displayed_credit_usd": 34}, {"displayed_credit_usd": float("nan")},
                       {"state": "RUNNING"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.check({**self.observation(), **change})

    def test_old_future_and_timezone_free_observations_are_rejected(self):
        current = datetime.now(timezone.utc)
        for stamp in (current - timedelta(hours=2), current + timedelta(hours=2), current.replace(tzinfo=None)):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                self.check({**self.observation(), "observed_at_utc": stamp.isoformat()})

    def test_old_archives_are_preserved_when_backup_space_is_low(self):
        def disk(path):
            return SimpleNamespace(free=(25 if str(path) == "E:/" else 30) * caller.GIB)
        with patch.object(caller.shutil, "disk_usage", side_effect=disk), \
             patch.object(caller.subprocess, "check_output", return_value=b""), \
             self.assertRaisesRegex(ValueError, "preserve existing archives"):
            caller.preflight("fleet4-transport-normal-001", self.observation())

    def test_dirty_source_or_unowned_label_cannot_start_a_producer(self):
        with patch.object(caller.shutil, "disk_usage", return_value=SimpleNamespace(free=30 * caller.GIB)), \
             patch.object(caller.subprocess, "check_output", return_value=b" M unrelated.txt"), \
             self.assertRaisesRegex(ValueError, "reviewed producer"):
            caller.preflight("fleet4-transport-normal-001", self.observation())
        with self.assertRaisesRegex(ValueError, "normal development run only"):
            self.check_label()

    def check_label(self):
        caller.preflight("another-project-run", self.observation())


class CloudFailureCleanupTests(unittest.TestCase):
    def exercise(self, mode):
        """Exercise real main/finally; every external cloud/shell call is mocked."""
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            root = Path(temporary)
            observation = root / "observation.json"
            observation.write_text(json.dumps(CloudPreflightTests().observation()), encoding="utf-8")
            flags = {"started": False, "stopped": False, "tar": 0, "state_calls": 0, "writes": 0}
            clock = [0.0]
            run = "fleet4-transport-normal-001"
            run_id = (b"a" * 16).hex()
            original_write = caller.write

            def command(args, **kwargs):
                if args[:2] == ["git", "archive"]:
                    Path(args[args.index("--output") + 1]).write_bytes(b"test source archive")
                if args == caller.BREV + ["start", "--detached", caller.INSTANCE]:
                    flags["started"] = True
                return subprocess.CompletedProcess(args, 0, b"raw commit fixture", b"")

            def state(**kwargs):
                flags["state_calls"] += 1
                if mode == "state_failure" and flags["state_calls"] == 3:
                    raise RuntimeError("transient ls failure")
                return "STOPPED" if not flags["started"] or flags["stopped"] else "RUNNING READY "

            def request_stop(args, **kwargs):
                self.assertEqual(caller.BREV + ["stop", caller.INSTANCE], args)
                flags["stopped"] = True
                return subprocess.CompletedProcess(args, 0, b"request accepted", b"")

            def remote(script, timeout=45):
                if "<<'PY'\n" in script:
                    program = script.split("<<'PY'\n", 1)[1].rsplit("\nPY\n", 1)[0]
                    compile(program, "generated-remote-program", "exec")
                if "nohup bash" in script:
                    if mode == "lost_reply":
                        raise TimeoutError("launch reply lost")
                    return "12345\n"
                if "remaining_owned_containers" in script:
                    if mode in ("lost_reply", "running_exit"):
                        return json.dumps({"passed": False, "run_id": run_id, "run_label": run,
                                           "remaining_owned_containers": ["active-nav"]})
                    return json.dumps({"passed": True, "run_id": run_id, "run_label": run,
                                       "remaining_owned_containers": []})
                if "tar -C" in script:
                    flags["tar"] += 1
                    raise TimeoutError("injected slow preservation")
                if "actions=p/" in script:
                    if mode == "near_deadline":
                        clock[0] = caller.RUN_WINDOW_S - caller.STOP_RESERVE_S + 1
                    return json.dumps({"finished": True, "exit": {"runner_exit_code": 0,
                        "cleanup_passed": True, "nav": {"Running": mode == "running_exit", "ExitCode": 0},
                        "sim": {"Running": False, "ExitCode": 0}}})
                return ""

            def write(path, data):
                flags["writes"] += 1
                if mode == "record_failure" and flags["writes"] > 1:
                    raise OSError("injected record disk failure")
                original_write(path, data)

            stack.enter_context(patch.object(caller, "ART", root / "art"))
            stack.enter_context(patch.object(caller.os, "chdir"))
            stack.enter_context(patch.object(caller.os, "urandom", return_value=b"a" * 16))
            stack.enter_context(patch.object(caller, "preflight"))
            stack.enter_context(patch.object(caller, "command", side_effect=command))
            stack.enter_context(patch.object(caller, "remote", side_effect=remote))
            stack.enter_context(patch.object(caller, "state", side_effect=state))
            stack.enter_context(patch.object(caller, "write", side_effect=write))
            stack.enter_context(patch.object(caller.subprocess, "check_output", return_value="b" * 40))
            stop = stack.enter_context(patch.object(caller.subprocess, "run", side_effect=request_stop))
            stack.enter_context(patch.object(caller.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(caller.time, "sleep"))
            spy = stack.enter_context(patch.object(caller, "stop_existing", wraps=caller.stop_existing))
            stack.enter_context(patch("sys.argv", ["runner", "--run", run, "--observation", str(observation), "--execute"]))
            result = caller.main()
            assert spy.call_args is not None
            return result, dict(flags), dict(spy.call_args.args[0]), stop.call_count

    def test_launch_reply_loss_and_active_exit_record_never_archive(self):
        for mode in ("lost_reply", "running_exit"):
            with self.subTest(mode=mode):
                code, flags, result, stops = self.exercise(mode)
                self.assertEqual(1, code)
                self.assertEqual(0, flags["tar"])
                self.assertIn("closure", result["preservation_error"])
                self.assertGreaterEqual(stops, 1)
                self.assertIn("actual_stopped_at_utc", result)

    def test_transient_ls_and_record_failure_cannot_skip_stop(self):
        for mode in ("state_failure", "record_failure"):
            with self.subTest(mode=mode):
                _, flags, result, stops = self.exercise(mode)
                self.assertGreaterEqual(stops, 1)
                self.assertTrue(flags["stopped"])
                self.assertIn("actual_stopped_at_utc", result)
                if mode == "record_failure":
                    self.assertTrue(result["host_record_errors"])

    def test_slow_preservation_still_runs_stop(self):
        code, flags, result, stops = self.exercise("slow_preservation")
        self.assertEqual(1, code)
        self.assertEqual(1, flags["tar"])
        self.assertFalse(result["preserved"])
        self.assertGreaterEqual(stops, 1)

    def test_near_deadline_preserves_the_stop_window_instead_of_archiving(self):
        _, flags, result, stops = self.exercise("near_deadline")
        self.assertEqual(0, flags["tar"])
        self.assertGreaterEqual(stops, 1)
        self.assertFalse(result["preserved"])

    def test_each_preservation_operation_is_clamped_to_the_remaining_budget(self):
        with patch.object(caller.time, "monotonic", return_value=11500):
            self.assertEqual(200, caller.remaining_timeout(12000, 600))
        with patch.object(caller.time, "monotonic", return_value=11700):
            with self.assertRaises(TimeoutError):
                caller.remaining_timeout(12000, 600)


if __name__ == "__main__":
    unittest.main()
