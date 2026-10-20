"""Mock contracts by default; --actual runs one harmless existing-WSL admission."""
import base64
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from typing import Any
import unittest
from unittest.mock import patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from factory_ops.wsl_process_supervisor import WslProcessSupervisor


RUN = "a" * 32


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
        with patch.object(supervisor, "_show", side_effect=[{"LoadState": "not-found"}, owned(supervisor)]), \
                patch.object(supervisor, "_checked", return_value="") as checked, \
                patch.object(supervisor, "inspect", return_value={"ownership_match": True}):
            supervisor.start(command, working_directory="/tmp", log_path="/tmp/new", runtime_bound_s=45)
            launch = checked.call_args_list[-1].args[0]
            self.assertEqual(json.loads(base64.b64decode(launch[-1])), command)
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
                patch.object(supervisor, "_checked", side_effect=["", RuntimeError("already exists")]), \
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


if __name__ == "__main__":
    if sys.argv[1:] == ["--actual"]:
        result = actual_admission()
        print(json.dumps({key: result[key] for key in
            ("status", "unit_name", "windows_test_directory", "ready", "stopped")}, indent=2))
    else:
        unittest.main()
