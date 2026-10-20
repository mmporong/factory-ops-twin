"""Own one WSL user-service cgroup; Windows Job Objects do not close Linux tasks.

Requires an existing Ubuntu systemd user manager and /usr/bin/python3. Never
uses a shell, sudo, installation, PID-name killing, or a caller-selected unit.
"""
from __future__ import annotations

import base64
import json
import math
from pathlib import PurePosixPath
import re
import subprocess
import time
from typing import Any, Sequence
import uuid


_PROPERTIES = "Id,LoadState,ActiveState,SubState,InvocationID,ControlGroup,Description,Transient,MainPID,Result,ExecMainStatus,KillMode,SendSIGKILL,RuntimeMaxUSec,TimeoutStopUSec,WorkingDirectory,StandardOutput,StandardError"
_EXEC = "import base64,json,os,sys; a=json.loads(base64.b64decode(sys.argv[1])); os.execv(a[0],a)"
_PREPARE_LOG = """import os,sys
if not os.path.isdir(sys.argv[1]):
    raise ValueError('working directory must already exist')
fd=os.open(sys.argv[2],os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
os.close(fd)
"""
_CGROUP_PROBE = """import json,os,sys
group=sys.argv[1]
known=json.loads(sys.argv[2])
root='/sys/fs/cgroup'+group
def identity(pid):
    try:
        with open('/proc/'+str(pid)+'/stat') as f:
            data=f.read()
        tail=data[data.rfind(')')+2:].split()
        return {'pid':int(pid),'start_ticks':int(tail[19]),'state':tail[0],'session_id':int(tail[3])}
    except FileNotFoundError:
        return None
present=os.path.isdir(root)
pids=set()
populated=False
if present:
    with open(root+'/cgroup.events') as f:
        events=dict(line.split() for line in f)
    if events.get('populated') not in ('0','1'):
        raise RuntimeError('cgroup populated state unavailable')
    populated=events['populated']=='1'
    for directory,_,_ in os.walk(root):
        try:
            with open(directory+'/cgroup.procs') as f:
                pids.update(int(p) for p in f.read().split())
        except FileNotFoundError:
            pass
processes=[p for pid in sorted(pids) if (p:=identity(pid)) is not None]
live_known=[]
for pid,ticks in known.items():
    current=identity(pid)
    if current is not None and current['start_ticks']==ticks:
        live_known.append(current)
print(json.dumps({'cgroup_exists':present,'cgroup_populated':populated,
    'cgroup_pids':sorted(pids),'processes':processes,'live_known_processes':live_known}))
"""


def _path(value: str) -> str:
    if (not isinstance(value, str) or not value.startswith("/") or len(value) > 2048
            or any(ord(c) < 32 for c in value) or "%" in value
            or any(part in (".", "..") for part in value.split("/"))):
        raise ValueError("paths must be absolute Linux paths without controls, specifiers or traversal")
    return str(PurePosixPath(value))


def _seconds(value: float, maximum: int) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 1 <= value <= maximum:
        raise ValueError("time bound is outside the permitted finite range")
    return float(value)


class WslProcessSupervisor:
    """Single-use supervisor. Call stop in finally and retain its closure record.

    Unit creation refuses an existing name. Inspect/stop require the captured
    InvocationID and description; replacement units are never stopped. A random
    per-instance suffix prevents normal concurrent launches from sharing names.
    """

    def __init__(self, run_id: str, *, distribution: str = "Ubuntu",
                 command_timeout_s: float = 20, stop_timeout_s: float = 10) -> None:
        if not isinstance(run_id, str) or re.fullmatch(r"[0-9a-f]{32}", run_id) is None:
            raise ValueError("run_id must be 32 lowercase hex characters")
        if not isinstance(distribution, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", distribution) is None:
            raise ValueError("invalid existing WSL distribution name")
        self.run_id = run_id
        self.distribution = distribution
        self._timeout = _seconds(command_timeout_s, 60)
        self._stop_timeout = _seconds(stop_timeout_s, 30)
        if self._timeout <= self._stop_timeout:
            raise ValueError("command timeout must exceed the systemd stop timeout")
        token = uuid.uuid4().hex
        self._unit = "factory-ops-" + run_id + "-" + token + ".service"
        self._description = "factory-ops run=" + run_id + " owner=" + token
        self._attempted = False
        self._identity: dict[str, str] | None = None
        self._starting_state: dict[str, str] | None = None
        self._known_processes: dict[str, int] = {}

    @property
    def unit_name(self) -> str:
        return self._unit

    def _call(self, args: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(["wsl.exe", "--distribution", self.distribution, "--exec", *args],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=self._timeout, check=False, shell=False)

    def _checked(self, args: Sequence[str]) -> str:
        result = self._call(args)
        if result.returncode:
            raise RuntimeError("WSL user-manager command failed: " + result.stderr[-1500:])
        return result.stdout

    def _show(self) -> dict[str, str]:
        result = self._call(["systemctl", "--user", "--no-ask-password", "show", self._unit,
                             "--property=" + _PROPERTIES])
        state = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        if result.returncode not in (0, 4) or "LoadState" not in state:
            raise RuntimeError("cannot observe the existing user manager: " + result.stderr[-1500:])
        return state

    def _capture_identity(self, state: dict[str, str]) -> None:
        group = state.get("ControlGroup", "")
        if (state.get("Id") != self._unit or state.get("Transient") != "yes"
                or state.get("Description") != self._description
                or re.fullmatch(r"[0-9a-f]{32}", state.get("InvocationID", "")) is None
                or not group.endswith("/" + self._unit)):
            raise RuntimeError("created unit identity/cgroup was not established; no stop authority captured")
        _path(group)
        self._identity = {key: state[key] for key in ("Id", "InvocationID", "Description", "ControlGroup")}

    def start(self, command: Sequence[str], *, working_directory: str,
              log_path: str, runtime_bound_s: float) -> dict[str, Any]:
        if self._attempted:
            raise RuntimeError("supervisor instances are single-use")
        if (isinstance(command, (str, bytes)) or not isinstance(command, (list, tuple))
                or not 1 <= len(command) <= 128
                or any(not isinstance(arg, str) or "\0" in arg or len(arg) > 4096 for arg in command)
                or sum(len(arg) + 1 for arg in command) > 8192):
            raise ValueError("command must be a bounded argv list/tuple without NUL")
        _path(command[0])
        working_directory, log_path = _path(working_directory), _path(log_path)
        runtime = _seconds(runtime_bound_s, 86400)
        encoded = base64.b64encode(json.dumps(list(command), ensure_ascii=True).encode()).decode("ascii")
        if len(encoded) > 16384:
            raise ValueError("encoded command exceeds the bounded Windows transport size")
        self._attempted = True
        self._starting_state = self._show()
        if self._starting_state["LoadState"] != "not-found":
            raise RuntimeError("unit existed before this launch; refusing creation or stop authority")
        self._checked(["/usr/bin/python3", "-c", _PREPARE_LOG, working_directory, log_path])
        # Base64 JSON keeps user argv opaque to systemd specifier/environment
        # expansion as well as to Windows/WSL quoting. The wrapper execs argv.
        launch = ["systemd-run", "--user", "--no-ask-password", "--quiet", "--service-type=exec",
            "--unit=" + self._unit, "--description=" + self._description,
            "--expand-environment=no", "--working-directory=" + working_directory,
            "--property=RuntimeMaxSec=" + str(runtime),
            "--property=TimeoutStopSec=" + str(self._stop_timeout),
            "--property=KillMode=control-group", "--property=SendSIGKILL=yes",
            "--property=StandardOutput=append:" + log_path,
            "--property=StandardError=append:" + log_path,
            "--", "/usr/bin/python3", "-c", _EXEC, encoded]
        try:
            self._checked(launch)
            self._capture_identity(self._show())
            return self.inspect()
        except Exception:
            # A transport timeout can occur after service creation. Only the
            # exact generated description/invocation may authorize cleanup.
            if self._identity is None:
                try:
                    self._capture_identity(self._show())
                except Exception:
                    pass
            if self._identity is not None:
                self.stop()
            raise

    def inspect(self) -> dict[str, Any]:
        if self._identity is None:
            raise RuntimeError("no created unit identity; inspect/stop denied")
        state = self._show()
        missing = state["LoadState"] == "not-found"
        matched = missing or all(state.get(key) == self._identity[key]
            for key in ("Id", "InvocationID", "Description")) and state.get("Transient") == "yes"
        if not matched:
            return {"unit_name": self._unit, "run_id": self.run_id, "unit_state": state,
                    "ownership_match": False, "closure_verified": False,
                    "starting_state": self._starting_state, "starting_identity": dict(self._identity)}
        observed = json.loads(self._checked(["/usr/bin/python3", "-c", _CGROUP_PROBE,
            self._identity["ControlGroup"], json.dumps(self._known_processes)]))
        for process in observed["processes"]:
            self._known_processes[str(process["pid"])] = process["start_ticks"]
        inactive = missing or state.get("ActiveState") in ("inactive", "failed")
        closed = inactive and not observed["cgroup_populated"] and not observed["cgroup_pids"] and not observed["live_known_processes"]
        return {"unit_name": self._unit, "run_id": self.run_id, "unit_state": state,
                "ownership_match": True, "starting_state": self._starting_state,
                "starting_identity": dict(self._identity), "control_group": self._identity["ControlGroup"],
                **observed, "known_processes": dict(self._known_processes),
                "closure_verified": closed, "closure_scope": "owned Linux systemd user-service cgroup and observed PID/start-time identities"}

    def stop(self) -> dict[str, Any]:
        before = self.inspect()
        if not before["ownership_match"]:
            raise RuntimeError("unit invocation changed; refusing to stop another owner's unit")
        if before["closure_verified"]:
            return before
        self._checked(["systemctl", "--user", "--no-ask-password", "stop", self._unit])
        deadline = time.monotonic() + self._stop_timeout
        while True:
            result = self.inspect()
            if result["closure_verified"] or not result["ownership_match"] or time.monotonic() >= deadline:
                return result
            time.sleep(.1)
