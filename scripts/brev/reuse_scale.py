"""Reuse the stopped FactoryOps scale VM behind an independent hard-stop timer.

The default mode is read-only.  ``--execute`` is required to arm the timer,
start the existing VM, and run the bounded Isaac Sim SDK probe.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import uuid


BREV = ["wsl", "-d", "Ubuntu", "--", "/home/llroad/.local/bin/brev"]
WSL = ["wsl", "-d", "Ubuntu", "--"]
NAME = "factory-ops-scale-234"
INSTANCE_ID = "fy0i3jtz5"
INSTANCE_TYPE = "gpu-l40s-d.2gpu-64vcpu-384gb"
IMAGE = "nvcr.io/nvidia/isaac-sim:6.2.0"
HARD_STOP_SECONDS = 7200
STOP_RESERVE_SECONDS = 300
HOURLY_ESTIMATE_USD = 5.48
ESTIMATED_ENVELOPE_USD = 11.0


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def call_command(
    argv: list[str], *, timeout: float = 60, log_path: Path | None = None,
    check: bool = True,
) -> tuple[int, str]:
    """Run one argv-only command and decode all output as UTF-8."""
    reply = subprocess.run(argv, capture_output=True, timeout=timeout)
    output = (reply.stdout + reply.stderr).decode("utf-8", errors="replace")
    if log_path is not None:
        log_path.write_text(output, encoding="utf-8")
    if check and reply.returncode:
        program = Path(argv[0]).name
        raise RuntimeError(f"{program} exited {reply.returncode}: {output[-3000:]}")
    return reply.returncode, output


def brev_call(arguments: list[str], **kwargs: object) -> str:
    return call_command(BREV + arguments, **kwargs)[1]


def wsl_call(arguments: list[str], **kwargs: object) -> tuple[int, str]:
    return call_command(WSL + arguments, **kwargs)


def target_row() -> str | None:
    rows = [row.strip() for row in brev_call(["ls"]).splitlines()
            if row.strip().split()[:1] == [NAME]]
    if len(rows) > 1:
        raise RuntimeError("target VM identity is ambiguous")
    return rows[0] if rows else None


def validate_identity(row: str | None) -> list[str]:
    if row is None:
        raise ValueError("the reviewed FactoryOps VM is not listed")
    fields = row.split()
    if len(fields) < 6 or fields[0] != NAME:
        raise ValueError("target VM name does not match the reviewed VM")
    if INSTANCE_ID not in fields or INSTANCE_TYPE not in fields:
        raise ValueError("target VM id or instance type does not match the reviewed VM")
    return fields


def validate_target(row: str | None, required_state: str) -> None:
    fields = validate_identity(row)
    if fields[1] != required_state:
        raise ValueError(f"target must be uniquely {required_state}")


def _show_properties(unit: str) -> str:
    return wsl_call([
        "systemctl", "--user", "show", unit,
        "--property=LoadState,ActiveState,Unit,ExecStart,ControlPID",
        "--no-pager",
    ], timeout=30)[1]


def arm_watchdog(
    unit_base: str, deadline_utc: str, log_path: Path | None = None,
) -> tuple[dict[str, object], subprocess.Popen[bytes]]:
    """Arm and verify a user-systemd timer before any VM start is allowed."""
    wsl_call(["systemctl", "--user", "show-environment"], timeout=30)
    service = unit_base + ".service"
    timer = unit_base + ".timer"
    command = BREV[-1]
    keepalive_argv = WSL + ["/usr/bin/sleep", "7500"]
    keepalive = subprocess.Popen(
        keepalive_argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    arm_arguments = [
        "systemd-run", "--user", "--unit=" + unit_base,
        "--on-active=" + str(HARD_STOP_SECONDS) + "s",
        "--timer-property=AccuracySec=1s", "--collect",
        "--property=Environment=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        command, "stop", NAME,
    ]
    timer_state = ""
    service_state = ""
    try:
        if keepalive.poll() is not None:
            raise RuntimeError("WSL keepalive exited before hard-stop timer registration")
        _, arm_output = wsl_call(arm_arguments, timeout=30, log_path=log_path)
        for _ in range(30):
            if keepalive.poll() is not None:
                raise RuntimeError("WSL keepalive exited after hard-stop timer registration")
            try:
                timer_state = _show_properties(timer)
                service_state = _show_properties(service)
                if ("LoadState=loaded" in timer_state
                        and "ActiveState=active" in timer_state):
                    break
            except RuntimeError:
                pass
            time.sleep(0.1)
        else:
            raise RuntimeError("hard-stop timer is not loaded and active")
        normalized = service_state.replace('\\"', '"')
        if ("LoadState=loaded" not in service_state or command not in service_state
                or "stop" not in normalized or NAME not in normalized):
            raise RuntimeError("hard-stop service command could not be verified")
    except Exception as error:
        try:
            cleanup_watchdog(unit_base, keepalive)
        except Exception as cleanup_error:
            raise RuntimeError(
                f"{error}; watchdog cleanup also failed: {cleanup_error}") from error
        raise
    if "LoadState=loaded" not in timer_state or "ActiveState=active" not in timer_state:
        raise RuntimeError("hard-stop timer is not loaded and active")
    return {
        "unit_base": unit_base,
        "timer": timer,
        "service": service,
        "deadline_utc": deadline_utc,
        "arm_output": arm_output,
        "keepalive_pid": keepalive.pid,
        "keepalive_argv": keepalive_argv,
        "timer_properties": timer_state,
        "service_properties": service_state,
    }, keepalive


def cleanup_watchdog(
    unit_base: str, keepalive: subprocess.Popen[bytes] | None = None,
) -> dict[str, object]:
    """Stop only this launcher's units and prove they are no longer active."""
    units = [unit_base + ".timer", unit_base + ".service"]
    stop_rc, stop_output = wsl_call(
        ["systemctl", "--user", "stop", *units], timeout=30, check=False)
    states: dict[str, str] = {}
    for unit in units:
        _, state = wsl_call(
            ["systemctl", "--user", "show", unit,
             "--property=LoadState,ActiveState", "--no-pager"],
            timeout=30, check=False)
        states[unit] = state
        if "ActiveState=active" in state or "ActiveState=activating" in state:
            raise RuntimeError(f"owned watchdog unit remains active: {unit}")
    keepalive_exit_code = None
    if keepalive is not None:
        if keepalive.poll() is None:
            keepalive.terminate()
        try:
            keepalive_exit_code = keepalive.wait(timeout=10)
        except subprocess.TimeoutExpired:
            keepalive.kill()
            keepalive_exit_code = keepalive.wait(timeout=10)
        if keepalive.poll() is None:
            raise RuntimeError("owned WSL keepalive remains running")
    return {"stop_returncode": stop_rc, "stop_output": stop_output,
            "unit_states": states, "keepalive_exit_code": keepalive_exit_code,
            "keepalive_gone": keepalive is None or keepalive.poll() is not None,
            "inactive_confirmed": True}


def _bootstrap_script() -> str:
    return """#!/usr/bin/env bash
set -euo pipefail
work="$HOME/workspace/factoryops-scale-bootstrap-234"
mkdir -p "$work"/cache/{kit,ov,pip,glcache,computecache} "$work"/{logs,data,documents}
command -v docker
mapfile -t gpu_rows < <(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits)
printf '%s\\n' "${gpu_rows[@]}"
test "${#gpu_rows[@]}" -eq 2
for row in "${gpu_rows[@]}"; do
  [[ "$row" == *L40S* ]]
  memory=$(printf '%s' "$row" | cut -d, -f2 | tr -d ' ')
  test "$memory" -ge 40000
done
echo FACTORYOPS_L40S_PAIR_CONFIRMED
nproc
free -b
df -h "$HOME/workspace"
docker info --format '{{.ServerVersion}}'
timeout --signal=TERM --kill-after=30s 1800s docker pull nvcr.io/nvidia/isaac-sim:6.2.0
docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{json .RepoDigests}}'
image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
[[ "$image" == *@sha256:* ]]
cat > "$work/probe.py" <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 480, "renderer": "RayTracedLighting"})
for _ in range(8):
    app.update()
print("FACTORYOPS_SCALE_SDK_INITIALIZED", flush=True)
app.close()
PY
container=factoryops-scale-reuse-234
trap 'docker rm -f "$container" >/dev/null 2>&1 || true' EXIT
timeout --signal=TERM --kill-after=30s 900s docker run --rm --name "$container" --gpus all --network host --ipc host \\
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 \\
  -v "$work/cache/kit:/isaac-sim/kit/cache:rw" -v "$work/cache/ov:/root/.cache/ov:rw" \\
  -v "$work/cache/pip:/root/.cache/pip:rw" -v "$work/cache/glcache:/root/.cache/nvidia/GLCache:rw" \\
  -v "$work/cache/computecache:/root/.nv/ComputeCache:rw" \\
  -v "$work/logs:/root/.nvidia-omniverse/logs:rw" -v "$work/data:/root/.local/share/ov/data:rw" \\
  -v "$work/documents:/root/Documents:rw" \\
  -v "$work:/workspace/bootstrap:rw" --entrypoint /bin/bash "$image" \\
  -lc 'cd /isaac-sim && ./python.sh /workspace/bootstrap/probe.py --allow-root'
echo FACTORYOPS_SCALE_BOOTSTRAP_CLOSED
"""


def _ascii_summary(record: dict[str, object]) -> str:
    keys = ("status", "actual_stopped", "watchdog_inactive_confirmed", "error", "stop_error")
    return json.dumps({key: record.get(key) for key in keys}, ensure_ascii=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise ValueError("--output must name a new folder")
    initial_row = target_row()
    validate_target(initial_row, "STOPPED")
    if HOURLY_ESTIMATE_USD * HARD_STOP_SECONDS / 3600 > ESTIMATED_ENVELOPE_USD:
        raise ValueError("two-hour estimate exceeds the reviewed cost envelope")
    if not args.execute:
        print(json.dumps({
            "name": NAME, "instance_id": INSTANCE_ID, "instance_type": INSTANCE_TYPE,
            "initial_row": initial_row, "hard_stop_seconds": HARD_STOP_SECONDS,
            "hourly_search_estimate_usd": HOURLY_ESTIMATE_USD,
            "estimated_envelope_usd": ESTIMATED_ENVELOPE_USD,
            "settled_billing_guaranteed": False, "execute": False,
        }, ensure_ascii=True))
        return 0

    output.mkdir(parents=True, exist_ok=False)
    started_at = datetime.now(timezone.utc)
    deadline_utc = (started_at + timedelta(seconds=HARD_STOP_SECONDS)).isoformat()
    unit_base = "factoryops-scale-234-stop-" + uuid.uuid4().hex[:12]
    record: dict[str, object] = {
        "schema_version": 1, "name": NAME, "instance_id": INSTANCE_ID,
        "instance_type": INSTANCE_TYPE, "image": IMAGE,
        "started_at": started_at.isoformat(), "deadline_utc": deadline_utc,
        "hard_stop_seconds": HARD_STOP_SECONDS, "stop_reserve_seconds": STOP_RESERVE_SECONDS,
        "hourly_search_estimate_usd": HOURLY_ESTIMATE_USD,
        "estimated_envelope_usd": ESTIMATED_ENVELOPE_USD,
        "settled_billing_guaranteed": False, "launcher_pid": os.getpid(),
        "initial_row": initial_row, "status": "arming_watchdog",
        "start_attempted": False, "actual_stopped": False,
        "watchdog_inactive_confirmed": False,
        "watchdog_limitations": [
            "host power loss or premature WSL keepalive termination can prevent the user-systemd timer from firing"
        ],
        "scope": "hardware and Isaac Sim 6.2 SDK initialization only; no fleet physics qualification",
    }
    script = output / "bootstrap.sh"
    script.write_text(_bootstrap_script(), encoding="utf-8", newline="\n")
    record["bootstrap_sha256"] = hashlib.sha256(script.read_bytes()).hexdigest()

    def save() -> None:
        (output / "host.json").write_text(
            json.dumps(record, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    save()
    watchdog_armed = False
    watchdog_keepalive: subprocess.Popen[bytes] | None = None
    deadline = time.monotonic() + HARD_STOP_SECONDS
    try:
        watchdog_receipt, watchdog_keepalive = arm_watchdog(
            unit_base, deadline_utc, output / "watchdog-arm.log")
        record["watchdog"] = watchdog_receipt
        watchdog_armed = True
        record["status"] = "starting"
        record["start_attempted"] = True
        save()
        brev_call(["start", NAME], timeout=180, log_path=output / "start.log")
        ready_end = min(deadline - STOP_RESERVE_SECONDS, time.monotonic() + 900)
        record["status"] = "waiting_ready"
        while time.monotonic() < ready_end:
            try:
                row = target_row()
                record["readiness_row"] = row
                fields = validate_identity(row)
                if fields[1:4] == ["RUNNING", "COMPLETED", "READY"]:
                    break
                if fields[1] not in {"STARTING", "RUNNING", "STOPPED"}:
                    raise ValueError(f"unexpected target lifecycle state: {fields[1]}")
            except RuntimeError as error:
                record["readiness_lookup_error"] = f"{type(error).__name__}: {error}"
            save()
            time.sleep(20)
        else:
            raise TimeoutError("reviewed VM did not become RUNNING/COMPLETED/READY")
        remaining = min(3300.0, deadline - time.monotonic() - STOP_RESERVE_SECONDS)
        if remaining <= 0:
            raise TimeoutError("Stop reserve reached before SDK bootstrap")
        record["status"] = "bootstrapping"
        save()
        brev_call(["refresh"], timeout=60)
        linux_path = "/mnt/" + script.drive[0].lower() + script.as_posix()[2:]
        log = brev_call(["exec", NAME, "@" + linux_path], timeout=remaining,
                        log_path=output / "bootstrap.log")
        required = ("FACTORYOPS_L40S_PAIR_CONFIRMED", "FACTORYOPS_SCALE_SDK_INITIALIZED",
                    "FACTORYOPS_SCALE_BOOTSTRAP_CLOSED")
        if any(marker not in log for marker in required):
            raise RuntimeError("hardware or SDK initialization/closure sentinels absent")
        record["status"] = "passed"
    except Exception as error:
        record["status"] = "failed"
        record["error"] = f"{type(error).__name__}: {error}"
        print(record["error"].encode("ascii", errors="backslashreplace").decode("ascii"), flush=True)
    finally:
        if record["start_attempted"]:
            try:
                record["stop_response"] = brev_call(["stop", NAME], timeout=120)
            except Exception as error:
                record["stop_request_error"] = f"{type(error).__name__}: {error}"
            stop_end = time.monotonic() + STOP_RESERVE_SECONDS
            lookup_errors: list[str] = []
            while time.monotonic() < stop_end:
                try:
                    row = target_row()
                    record["final_row"] = row
                    validate_target(row, "STOPPED")
                    record["actual_stopped"] = True
                    break
                except Exception as error:
                    lookup_errors.append(f"{type(error).__name__}: {error}")
                    # Status polling at 10s hit Brev's rate limit in batch239.
                    time.sleep(30)
            if lookup_errors:
                record["stop_lookup_errors"] = lookup_errors
            if not record["actual_stopped"]:
                record["stop_error"] = "actual STOPPED was not confirmed; watchdog retained"
        else:
            # Start was never issued.  The exact initial row proves the owned VM stayed stopped.
            record["actual_stopped"] = True

        if record["actual_stopped"]:
            try:
                record["watchdog_cleanup"] = cleanup_watchdog(unit_base, watchdog_keepalive)
                record["watchdog_inactive_confirmed"] = True
            except Exception as error:
                record["watchdog_cleanup_error"] = f"{type(error).__name__}: {error}"
        else:
            record["recovery"] = {
                "watchdog_retained": watchdog_armed,
                "stop_argv": BREV + ["stop", NAME],
                "status_argv": BREV + ["ls"],
                "timer_status_argv": WSL + ["systemctl", "--user", "status", unit_base + ".timer"],
            }
        record["finished_at"] = timestamp()
        save()
    print(_ascii_summary(record))
    return 0 if (record["status"] == "passed" and record["actual_stopped"]
                 and record["watchdog_inactive_confirmed"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
