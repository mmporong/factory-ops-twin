"""Mock contracts by default; --actual runs one harmless existing-WSL admission."""
import base64
import hashlib
import json
import os
from pathlib import Path
import sys
import subprocess
import time
from typing import Any
import unittest
from unittest.mock import patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from factory_ops.wsl_process_supervisor import WslProcessSupervisor


RUN = "a" * 32


class _Client:
    pid = 12345

    def __init__(self):
        self.returncode = None
        self.waits = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.waits += 1
        self.returncode = 0
        return 0


def owned(supervisor):
    return {"Id": supervisor.unit_name, "Transient": "yes", "Description": supervisor._description,
            "InvocationID": "b" * 32, "ControlGroup": "/user.slice/app.slice/" + supervisor.unit_name,
            "LoadState": "loaded", "ActiveState": "active", "SubState": "running"}


class SupervisorTests(unittest.TestCase):
    def test_malformed_inputs_never_invoke_wsl(self):
        for run in ("", "../unit", "A" * 32, "0" * 31):
            with self.subTest(run=run), self.assertRaises(ValueError):
                WslProcessSupervisor(run)
        invalid_options: list[dict[str, Any]] = [
            {"distribution": "--exec"}, {"distribution": "Ubuntu\n"},
            {"command_timeout_s": float("inf")}, {"stop_timeout_s": True},
            {"command_timeout_s": 1, "stop_timeout_s": 2}]
        for options in invalid_options:
            with self.subTest(options=options), self.assertRaises(ValueError):
                WslProcessSupervisor(RUN, **options)
        supervisor = WslProcessSupervisor(RUN)
        with patch.object(supervisor, "_call") as call:
            for command in ("echo hi", [], ["relative"], ["/bin/echo", "x\0y"],
                            ["/bin/echo", "x" * 4097], ["/bin/echo"] * 129,
                            ["/bin/echo", "한" * 4000]):
                with self.subTest(command=command), self.assertRaises(ValueError):
                    supervisor.start(command, working_directory="/tmp", log_path="/tmp/test.log", runtime_bound_s=10)
            for directory, log, duration in (("relative", "/tmp/log", 1), ("/tmp", "/tmp/%u", 1),
                                           ("/tmp/../", "/tmp/log", 1), ("/tmp", "/tmp/log\n", 1),
                                           ("/tmp", "/tmp/log", float("nan"))):
                with self.assertRaises(ValueError):
                    supervisor.start(["/bin/true"], working_directory=directory, log_path=log, runtime_bound_s=duration)
            call.assert_not_called()

    def test_preexisting_unit_never_grants_stop_authority(self):
        supervisor = WslProcessSupervisor(RUN)
        with patch.object(supervisor, "_show", return_value=owned(supervisor)), \
                patch.object(supervisor, "_checked") as checked:
            with self.assertRaisesRegex(RuntimeError, "existed"):
                supervisor.start(["/bin/true"], working_directory="/tmp", log_path="/tmp/new", runtime_bound_s=10)
            with self.assertRaisesRegex(RuntimeError, "denied"):
                supervisor.stop()
            checked.assert_not_called()

    def test_unique_unit_and_command_are_owned_and_opaque(self):
        supervisor = WslProcessSupervisor(RUN)
        self.assertNotEqual(supervisor.unit_name, WslProcessSupervisor(RUN).unit_name)
        self.assertTrue(supervisor.unit_name.startswith("factory-ops-" + RUN + "-"))
        command = ["/bin/echo", "literal $HOME %u $(touch /tmp/unwanted); quote'\"", ""]
        client = _Client()
        self.addCleanup(supervisor._join_client)
        with patch.object(supervisor, "_show", side_effect=[{"LoadState": "not-found"}, owned(supervisor)]), \
                patch.object(supervisor, "_checked", return_value=""), \
                patch("factory_ops.wsl_process_supervisor.subprocess.Popen", return_value=client) as popen, \
                patch.object(supervisor, "inspect", return_value={"ownership_match": True}):
            supervisor.start(command, working_directory="/tmp", log_path="/tmp/new", runtime_bound_s=45)
            launch = popen.call_args.args[0]
            self.assertEqual(json.loads(base64.b64decode(launch[-1])), command)
            self.assertIn("--wait", launch)
            self.assertEqual(client.waits, 0)
            self.assertIn("--expand-environment=no", launch)
            self.assertIn("--property=KillMode=control-group", launch)
            self.assertIn("--property=RuntimeMaxSec=45.0", launch)
            self.assertNotIn("sh", launch)
            with self.assertRaisesRegex(RuntimeError, "single-use"):
                supervisor.start(command, working_directory="/tmp", log_path="/tmp/new2", runtime_bound_s=45)

    def test_launch_race_does_not_stop_a_foreign_unit(self):
        supervisor = WslProcessSupervisor(RUN)
        foreign = {**owned(supervisor), "Description": "someone else"}
        with patch.object(supervisor, "_show", side_effect=[{"LoadState": "not-found"}, foreign]), \
                patch.object(supervisor, "_checked", return_value=""), \
                patch("factory_ops.wsl_process_supervisor.subprocess.Popen", return_value=_Client()), \
                patch.object(supervisor, "stop") as stop:
            with self.assertRaises(RuntimeError):
                supervisor.start(["/bin/true"], working_directory="/tmp", log_path="/tmp/new", runtime_bound_s=10)
            stop.assert_not_called()

    def test_replaced_invocation_refuses_stop(self):
        supervisor = WslProcessSupervisor(RUN)
        supervisor._capture_identity(owned(supervisor))
        replaced = {**owned(supervisor), "InvocationID": "c" * 32}
        with patch.object(supervisor, "_show", return_value=replaced), \
                patch.object(supervisor, "_checked") as checked:
            self.assertFalse(supervisor.inspect()["ownership_match"])
            with self.assertRaisesRegex(RuntimeError, "changed"):
                supervisor.stop()
            checked.assert_not_called()

    def test_empty_cgroup_requires_known_pid_disappearance(self):
        supervisor = WslProcessSupervisor(RUN)
        supervisor._capture_identity(owned(supervisor))
        supervisor._known_processes = {"12": 100}
        base = {"cgroup_exists": False, "cgroup_populated": False, "cgroup_pids": [], "processes": []}
        missing = {"LoadState": "not-found", "ActiveState": "inactive"}
        with patch.object(supervisor, "_show", return_value=missing), \
                patch.object(supervisor, "_checked", side_effect=[
                    json.dumps({**base, "live_known_processes": [{"pid": 12, "start_ticks": 100}]}),
                    json.dumps({**base, "live_known_processes": []})]):
            self.assertFalse(supervisor.inspect()["closure_verified"])
            self.assertTrue(supervisor.inspect()["closure_verified"])

    def test_stop_only_uses_the_exact_unit_and_is_idempotent(self):
        supervisor = WslProcessSupervisor(RUN)
        live = {"ownership_match": True, "closure_verified": False}
        closed = {"ownership_match": True, "closure_verified": True}
        with patch.object(supervisor, "inspect", side_effect=[live, closed, closed]), \
                patch.object(supervisor, "_checked", return_value="") as checked:
            self.assertTrue(supervisor.stop()["closure_verified"])
            self.assertTrue(supervisor.stop()["closure_verified"])
            checked.assert_called_once_with(["systemctl", "--user", "--no-ask-password", "stop", supervisor.unit_name])

    def test_admission_polls_and_returns_with_living_foreground_client(self):
        supervisor = WslProcessSupervisor(RUN)
        client = _Client()
        self.addCleanup(supervisor._join_client)
        with patch.object(supervisor, "_show", side_effect=[{"LoadState": "not-found"},
                {"LoadState": "not-found"}, owned(supervisor)]), \
                patch.object(supervisor, "_checked", return_value=""), \
                patch.object(supervisor, "inspect", return_value={"ownership_match": True}), \
                patch("factory_ops.wsl_process_supervisor.subprocess.Popen", return_value=client), \
                patch("factory_ops.wsl_process_supervisor.time.sleep"):
            supervisor.start(["/bin/sleep", "70"], working_directory="/tmp", log_path="/tmp/new", runtime_bound_s=85)
            self.assertTrue(supervisor._client_state()["running"])
            self.assertEqual(0, client.waits)
        supervisor._join_client()
        self.assertTrue(supervisor._client_state()["closed"])
        self.assertEqual([], supervisor._client_output)

    def test_query_failure_has_returncode_and_both_outputs(self):
        supervisor = WslProcessSupervisor(RUN)
        with patch.object(supervisor, "_call", return_value=subprocess.CompletedProcess([], 1, "partial", "")):
            with self.assertRaisesRegex(RuntimeError, "returncode=1; stdout='partial'; stderr=''"):
                supervisor._show()

    def test_failed_stop_query_still_joins_only_its_foreground_client(self):
        supervisor = WslProcessSupervisor(RUN)
        with patch("factory_ops.wsl_process_supervisor.subprocess.Popen", return_value=_Client()):
            supervisor._launch(["systemd-run", "--wait", "--unit=" + supervisor.unit_name])
        with patch.object(supervisor, "inspect", side_effect=RuntimeError("query failed")):
            with self.assertRaisesRegex(RuntimeError, "query failed"):
                supervisor.stop()
        self.assertTrue(supervisor._client_state()["closed"])
        self.assertFalse(supervisor._client_state()["forced_termination"])

    def test_live_exact_unit_waiter_prevents_closure(self):
        supervisor = WslProcessSupervisor(RUN)
        supervisor._capture_identity(owned(supervisor))
        record = {"cgroup_exists": False, "cgroup_populated": False, "cgroup_pids": [],
                  "processes": [], "live_known_processes": [],
                  "foreground_waiters": [{"pid": 44, "start_ticks": 100}]}
        with patch.object(supervisor, "_show", return_value={"LoadState": "not-found"}), \
                patch.object(supervisor, "_checked", return_value=json.dumps(record)):
            self.assertFalse(supervisor.inspect()["closure_verified"])
            self.assertEqual(100, supervisor._known_processes["44"])

    def test_active_cgroup_change_refuses_stop(self):
        supervisor = WslProcessSupervisor(RUN)
        supervisor._capture_identity(owned(supervisor))
        changed = {**owned(supervisor), "ControlGroup": "/other/" + supervisor.unit_name}
        with patch.object(supervisor, "_show", return_value=changed), patch.object(supervisor, "_checked") as checked:
            self.assertFalse(supervisor.inspect()["ownership_match"])
            with self.assertRaises(RuntimeError):
                supervisor.stop()
            checked.assert_not_called()


_PARENT = """import json,os,signal,subprocess,sys,time
child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(180)'],start_new_session=True)
def stop(signum,frame):
    child.wait(timeout=12)
    raise SystemExit(0)
signal.signal(signal.SIGTERM,stop)
print(json.dumps({'event':'ready','parent_pid':os.getpid(),'child_pid':child.pid,
    'parent_session':os.getsid(0),'child_session':os.getsid(child.pid),'argv_literal':sys.argv[1:]}),flush=True)
while True:
    time.sleep(.1)
"""


def actual_admission() -> dict[str, Any]:
    """Explicit opt-in admission, logs outside the repository, never ROS."""
    if os.name != "nt":
        raise RuntimeError("actual admission is launched from Windows only")
    directory = Path.home() / "artifacts/local45/wsl-030-001"
    directory.mkdir(parents=True, exist_ok=False)
    linux_directory = "/mnt/" + directory.drive[0].lower() + directory.as_posix()[2:]
    run_id = uuid.uuid4().hex
    supervisor = WslProcessSupervisor(run_id)
    literal = "$HOME %u $(echo not-a-shell); 'quoted'"
    command = ["/usr/bin/python3", "-u", "-c", _PARENT, literal]
    record: dict[str, Any] = {"run_id": run_id, "unit_name": supervisor.unit_name,
        "windows_test_directory": str(directory), "linux_test_directory": linux_directory,
        "command": command, "runtime_bound_s": 180, "status": "starting",
        "source_sha256": {str(p.relative_to(Path(__file__).resolve().parents[1])).replace("\\", "/"):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path(__file__).resolve().parents[1] / "factory_ops/wsl_process_supervisor.py", Path(__file__).resolve()]}}
    proof_path = directory / "admission.json"
    def save():
        proof_path.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    try:
        record["started"] = supervisor.start(command, working_directory=linux_directory,
            log_path=linux_directory + "/process.log", runtime_bound_s=180)
        save()
        deadline = time.monotonic() + 20
        log = directory / "process.log"
        while time.monotonic() < deadline:
            lines = log.read_text(encoding="utf-8").splitlines()
            if lines:
                ready = json.loads(lines[0])
                if ready.get("event") == "ready":
                    record["ready"] = ready
                    break
            time.sleep(.1)
        else:
            raise RuntimeError("parent/setsid child did not report readiness")
        record["before_stop"] = supervisor.inspect()
        ready = record["ready"]
        if ready["parent_session"] == ready["child_session"] or ready["argv_literal"] != [literal]:
            raise AssertionError("setsid/literal argv boundary failed")
        observed = {p["pid"] for p in record["before_stop"]["processes"]}
        if not {ready["parent_pid"], ready["child_pid"]} <= observed:
            raise AssertionError("parent and child must both be observed inside the owned cgroup")
        record["stopped"] = supervisor.stop()
        if not record["stopped"]["closure_verified"]:
            raise AssertionError("owned cgroup/observed PIDs did not close")
        record["status"] = "passed"
    except BaseException as error:
        record["status"] = "failed"
        record["error"] = type(error).__name__ + ": " + str(error)
        raise
    finally:
        try:
            if supervisor._identity is not None:
                record["finally_closure"] = supervisor.stop()
                if not record["finally_closure"]["closure_verified"]:
                    record["status"] = "failed"
        except Exception as error:
            record["status"] = "failed"
            record["cleanup_error"] = type(error).__name__ + ": " + str(error)
        finally:
            save()
    if record["status"] != "passed":
        raise RuntimeError("final closure proof failed; see admission.json")
    return record


def actual_lifetime_admission() -> dict[str, Any]:
    """One opt-in 70-second fixture; no WSL queries during the quiet interval.

    Launch this entry point under a 100-second subprocess timeout. RuntimeMaxSec
    additionally bounds the owned service even if the test caller disappears.
    """
    if os.name != "nt":
        raise RuntimeError("actual lifetime admission is launched from Windows only")
    root = Path(__file__).resolve().parents[1]
    directory = root / "artifacts/local45/wsl-lifetime-037"
    directory.mkdir(parents=True, exist_ok=False)
    linux_directory = "/mnt/" + directory.drive[0].lower() + directory.as_posix()[2:]
    supervisor = WslProcessSupervisor(uuid.uuid4().hex, command_timeout_s=6, stop_timeout_s=2)
    began = time.monotonic()
    deadline = began + 100
    proof = directory / "admission.json"
    log = directory / "process.log"
    literal = "$HOME %u $(echo not-a-shell); 'quoted'"
    command = ["/usr/bin/python3", "-u", "-c", _PARENT, literal]
    record: dict[str, Any] = {"status": "starting", "run_id": supervisor.run_id,
        "unit_name": supervisor.unit_name, "windows_test_directory": str(directory),
        "linux_test_directory": linux_directory, "command": command, "runtime_bound_s": 85,
        "quiet_interval_s": 70, "fixture_hard_bound_s": 100, "client_checks": [],
        "source_sha256": {}, "scope": "WSL client lifetime and owned cgroup closure only; no ROS/SDK/GPU"}
    for name in ("factory_ops/wsl_process_supervisor.py", "tests/test_wsl_process_supervisor.py"):
        data = (root / name).read_bytes()
        (directory / Path(name).name).write_bytes(data)
        record["source_sha256"][name] = hashlib.sha256(data).hexdigest()

    def save():
        temporary = proof.with_suffix(".tmp")
        temporary.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(proof)

    def bound():
        if time.monotonic() >= deadline:
            raise TimeoutError("100-second fixture limit reached")

    save()
    try:
        record["boot_id_before"] = supervisor._checked(["/usr/bin/cat", "/proc/sys/kernel/random/boot_id"]).strip()
        record["started"] = supervisor.start(command, working_directory=linux_directory,
            log_path=linux_directory + "/process.log", runtime_bound_s=85)
        ready_deadline = time.monotonic() + 5
        while time.monotonic() < ready_deadline:
            lines = log.read_text(encoding="utf-8").splitlines()
            if lines and (ready := json.loads(lines[0])).get("event") == "ready":
                record["ready"] = ready
                break
            time.sleep(.1)
        else:
            raise RuntimeError("parent/setsid child did not report readiness")
        ready = record["ready"]
        before = supervisor.inspect()
        record["before_quiet"] = before
        if (ready["parent_session"] == ready["child_session"] or ready["argv_literal"] != [literal]
                or not {ready["parent_pid"], ready["child_pid"]} <= {p["pid"] for p in before["processes"]}
                or not before["foreground_waiters"] or not before["client"]["running"]):
            raise AssertionError("owned parent/setsid child/foreground waiter/client admission failed")
        save()
        quiet_started = time.monotonic()
        quiet_deadline = quiet_started + 70
        # Only Windows Popen.poll here: periodic wsl.exe probes must not be an
        # alternative lifetime anchor during the interval being measured.
        while True:
            bound()
            client = supervisor._client_state()
            record["client_checks"].append({"quiet_elapsed_s": time.monotonic() - quiet_started, **client})
            if not client["running"] or client["pid"] != before["client"]["pid"]:
                raise AssertionError("foreground client did not stay alive for the full interval")
            remaining = quiet_deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(1., remaining))
        record["actual_quiet_interval_s"] = time.monotonic() - quiet_started
        record["boot_id_after"] = supervisor._checked(["/usr/bin/cat", "/proc/sys/kernel/random/boot_id"]).strip()
        record["after_quiet"] = supervisor.inspect()
        if (not record["boot_id_before"] or record["boot_id_before"] != record["boot_id_after"]
                or not record["after_quiet"]["client"]["running"]
                or not record["after_quiet"]["foreground_waiters"]):
            raise AssertionError("boot identity or owned foreground lifetime changed")
        record["stopped"] = supervisor.stop()
        if not record["stopped"]["closure_verified"]:
            raise AssertionError("owned unit/cgroup/observed processes/client closure failed")
        bound()
        record["status"] = "passed"
    except BaseException as error:
        record.update(status="failed", error=type(error).__name__ + ": " + str(error))
    finally:
        try:
            if supervisor._identity is not None:
                record["finally_closure"] = supervisor.stop()
                if not record["finally_closure"]["closure_verified"]:
                    record["status"] = "failed"
            else:
                supervisor._join_client()
        except BaseException as error:
            record.update(status="failed", cleanup_error=type(error).__name__ + ": " + str(error))
        record["elapsed_wall_s"] = time.monotonic() - began
        if record["elapsed_wall_s"] > 100:
            record.update(status="failed", hard_bound_exceeded=True)
        record["final_client"] = supervisor._client_state()
        if log.exists():
            record["log_sha256"] = hashlib.sha256(log.read_bytes()).hexdigest()
        save()
    if record["status"] != "passed":
        raise RuntimeError("lifetime fixture failed; see " + str(proof))
    return record


if __name__ == "__main__":
    if sys.argv[1:] == ["--actual-lifetime"]:
        result = actual_lifetime_admission()
        print(json.dumps({key: result[key] for key in
            ("status", "unit_name", "windows_test_directory", "actual_quiet_interval_s", "final_client")}, indent=2))
    elif sys.argv[1:] == ["--actual"]:
        result = actual_admission()
        print(json.dumps({key: result[key] for key in
            ("status", "unit_name", "windows_test_directory", "ready", "stopped")}, indent=2))
    else:
        unittest.main()
