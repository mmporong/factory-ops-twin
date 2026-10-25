"""Run the approved one-clip Cosmos smoke job on the reviewed Brev VM.

The default mode is read-only. ``--execute --key-stdin`` is required before
the supervisor arms a hard-stop watchdog, starts the VM, or opens SSH.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "brev"))
import reuse_scale as watchdog_runtime


BREV = watchdog_runtime.BREV
HARD_STOP_SECONDS = watchdog_runtime.HARD_STOP_SECONDS
WSL = ["wsl", "-d", "Ubuntu", "--exec"]
arm_watchdog = watchdog_runtime.arm_watchdog
brev_call = watchdog_runtime.brev_call
call_command = watchdog_runtime.call_command
cleanup_watchdog = watchdog_runtime.cleanup_watchdog
timestamp = watchdog_runtime.timestamp
def wsl_call(arguments: list[str], **kwargs: object) -> tuple[int, str]:
    """Preserve remote shell text until SSH delivers it to the target host."""
    return call_command(WSL + arguments, **kwargs)


NAME = "factory-ops-scale-234"
INSTANCE_ID = "fy0i3jtz5"
INSTANCE_TYPE = "gpu-l40s-d.2gpu-64vcpu-384gb"
HOST_ALIAS = NAME + "-host"
TARGET_PROFILE = "scale234"
BATCH_ID = 379
REMOTE_DIR = "workspace/factoryops-cosmos-379"
OUTPUT_BASENAME = "cosmos-brev-smoke-379"
REMOTE_ROOT = "$HOME/workspace/factoryops-cosmos-379"
REMOTE_KEY = "/dev/shm/factoryops-cosmos-379/ngc-key"
REQUIRED_INPUTS = (
    "inference-request.json",
    "input-reference.mp4",
    "bundle.json",
    "deployment-contract.json",
)
EXPECTED_INPUT_ROOT = Path("E:/FactoryOpsData/cosmos-static-smoke-346")
READY_TIMEOUT_SECONDS = 600
STOP_TIMEOUT_SECONDS = 600
POLL_SECONDS = 30
STOP_RESERVE_SECONDS = 600
MAX_REMOTE_OUTPUT_BYTES = 250 * 1024 * 1024
HOURLY_ESTIMATE_USD = 5.48256
ESTIMATED_ENVELOPE_USD = 2 * HOURLY_ESTIMATE_USD
TARGETS = {
    "scale234": {
        "name": "factory-ops-scale-234",
        "id": "fy0i3jtz5",
        "instance_type": "gpu-l40s-d.2gpu-64vcpu-384gb",
        "host_alias_suffix": "",
        "hourly_estimate_usd": 5.48256,
    },
    "g0": {
        "name": "factory-ops-twin-g0",
        "id": "evica0x9l",
        "instance_type": "g6e.2xlarge",
        "host_alias_suffix": "",
        "hourly_estimate_usd": 2.690496,
    },
}
SSH_OPTIONS = ("-o", "BatchMode=yes", "-o", "ConnectTimeout=30",
               "-o", "RequestTTY=no", "-o", "ServerAliveInterval=10",
               "-o", "ServerAliveCountMax=3", "-o", "ControlMaster=no",
               "-o", "ControlPath=none")
NVAPI_PATTERN = re.compile(r"(?i)nvapi(?:[-_:]|%2[dDfF])?[A-Za-z0-9._~-]{8,}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _redact(text: str, secret: str | None) -> str:
    if secret:
        text = text.replace(secret, "[REDACTED_NGC_KEY]")
    return NVAPI_PATTERN.sub("[REDACTED_NVAPI_TOKEN]", text)


def _load_listing() -> tuple[dict[str, object], dict[str, object]]:
    _, raw = call_command(BREV + ["ls", "--json"], timeout=60)
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as error:
        raise RuntimeError("brev ls --json did not return valid JSON") from error
    rows = payload.get("workspaces")
    if not isinstance(rows, list):
        raise ValueError("brev ls --json is missing the workspaces array")
    matches = [row for row in rows if isinstance(row, dict) and row.get("name") == NAME]
    if len(matches) != 1:
        raise ValueError("reviewed Brev workspace must be listed exactly once")
    target = matches[0]
    if target.get("id") != INSTANCE_ID or target.get("instance_type") != INSTANCE_TYPE:
        raise ValueError("reviewed Brev workspace identity or instance type changed")
    return payload, target


def _require_initial_state(target: dict[str, object]) -> None:
    required = {
        "status": "STOPPED",
        "build_status": "COMPLETED",
    }
    mismatches = [f"{key}={target.get(key)!r}" for key, value in required.items()
                  if target.get(key) != value]
    if mismatches:
        raise ValueError("reviewed workspace is not STOPPED/COMPLETED: "
                         + ", ".join(mismatches))


def _validate_inputs(args: argparse.Namespace) -> tuple[Path, Path, dict[str, Path]]:
    output = args.output.resolve()
    if output.name != OUTPUT_BASENAME:
        raise ValueError(f"--output basename must be {OUTPUT_BASENAME!r}")
    if output.exists():
        raise ValueError("--output must name a new directory")
    remote_script = args.remote_script.resolve()
    if not remote_script.is_file():
        raise ValueError("--remote-script must name an existing file")
    inputs = args.inputs_directory.resolve()
    if inputs != EXPECTED_INPUT_ROOT.resolve():
        raise ValueError(f"--inputs-directory must be {EXPECTED_INPUT_ROOT}")
    if not inputs.is_dir():
        raise ValueError("--inputs-directory does not exist")
    files = {name: inputs / name for name in REQUIRED_INPUTS}
    missing = [name for name, path in files.items() if not path.is_file()]
    if missing:
        raise ValueError("required approved inputs are missing: " + ", ".join(missing))
    if args.execute != args.key_stdin:
        raise ValueError("--execute and --key-stdin must be supplied together")
    return output, remote_script, files


def _read_key() -> str:
    raw = sys.stdin.buffer.read()
    lines = raw.splitlines()
    if len(lines) != 1 or not lines[0] or b"\x00" in lines[0]:
        raise ValueError("--key-stdin requires exactly one non-empty UTF-8 line")
    try:
        return lines[0].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("--key-stdin requires exactly one non-empty UTF-8 line") from error


def _linux_path(path: Path) -> str:
    return wsl_call(["wslpath", "-a", str(path)], timeout=30)[1].strip()


def _ssh(command: str, *, timeout: float = 60, check: bool = True) -> tuple[int, str]:
    reply = subprocess.run(
        WSL + ["ssh", *SSH_OPTIONS, HOST_ALIAS, command],
        capture_output=True,
        timeout=timeout,
    )
    stdout = _redact(reply.stdout.decode("utf-8", errors="replace"), None)
    stderr = _redact(reply.stderr.decode("utf-8", errors="replace"), None)
    if check and reply.returncode:
        raise RuntimeError(f"ssh exited {reply.returncode}: {(stdout + stderr)[-3000:]}")
    return reply.returncode, stdout


def _scp(local_path: Path, remote_name: str) -> None:
    local = _linux_path(local_path)
    wsl_call(
        ["scp", *SSH_OPTIONS, local, f"{HOST_ALIAS}:{REMOTE_DIR}/{remote_name}"],
        timeout=180,
    )


def _poll_ready(deadline: float, record: dict[str, object], save: object) -> None:
    end = min(deadline - STOP_RESERVE_SECONDS, time.monotonic() + READY_TIMEOUT_SECONDS)
    polls: list[dict[str, object]] = []
    record["readiness_polls"] = polls
    while True:
        _, target = _load_listing()
        row = {
            "at": timestamp(),
            "status": target.get("status"),
            "build_status": target.get("build_status"),
            "shell_status": target.get("shell_status"),
            "health_status": target.get("health_status"),
        }
        polls.append(row)
        save()
        if target.get("status") in {"FAILED", "ERROR", "DELETED", "DELETING"}:
            raise RuntimeError("workspace entered a failed lifecycle state before remote execution")
        # CLI v0.6.335 maps raw RUNNING + unhealthy telemetry to display UNHEALTHY.
        # READY is only a boot candidate; the real SSH/GPU probe is still required.
        if all((
            target.get("status") in {"RUNNING", "UNHEALTHY"},
            target.get("build_status") == "COMPLETED",
            target.get("shell_status") == "READY",
        )):
            return
        if time.monotonic() >= end:
            raise TimeoutError("workspace did not become RUNNING/COMPLETED/READY")
        time.sleep(POLL_SECONDS)


def _send_secret(secret: str) -> None:
    command = f'umask 077; mkdir -p {REMOTE_KEY.rsplit("/", 1)[0]}; cat > {REMOTE_KEY}'
    completed = subprocess.run(
        WSL + ["ssh", *SSH_OPTIONS, HOST_ALIAS, command],
        input=secret.encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=60,
    )
    if completed.returncode:
        safe = _redact(
            (completed.stdout + completed.stderr).decode("utf-8", errors="replace"), secret
        )
        raise RuntimeError(f"secret transfer failed with exit {completed.returncode}: {safe[-1000:]}")


def _run_remote(
    output: Path,
    record: dict[str, object],
    save: object,
    secret: str,
    timeout: float,
) -> int:
    remote_timeout = max(1, int(timeout))
    argv = WSL + [
        "ssh", *SSH_OPTIONS, HOST_ALIAS,
        f'timeout --signal=TERM --kill-after=30s {remote_timeout}s '
        f'/bin/bash "{REMOTE_ROOT}/run.sh" {BATCH_ID} {TARGET_PROFILE}',
    ]
    process = subprocess.Popen(
        argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    record["remote_ssh_pid"] = process.pid
    record["remote_ssh_argv"] = [*WSL, "ssh", *SSH_OPTIONS, HOST_ALIAS, "<approved-run-command>"]
    save()
    messages: queue.Queue[tuple[str, str | None]] = queue.Queue()

    def drain(label: str, stream: object) -> None:
        assert stream is not None
        for raw in iter(stream.readline, b""):
            messages.put((label, _redact(raw.decode("utf-8", errors="replace"), secret)))
        messages.put((label, None))

    threads = [
        threading.Thread(target=drain, args=("stdout", process.stdout), daemon=True),
        threading.Thread(target=drain, args=("stderr", process.stderr), daemon=True),
    ]
    for thread in threads:
        thread.start()
    open_streams = 2
    started = time.monotonic()
    last_heartbeat = 0.0
    log_path = output / "run.log"
    try:
        with log_path.open("w", encoding="utf-8", newline="\n") as log:
            while open_streams or process.poll() is None:
                if time.monotonic() - started > timeout + 45:
                    raise TimeoutError("remote Cosmos command exceeded the bounded execution window")
                try:
                    label, message = messages.get(timeout=1)
                    if message is None:
                        open_streams -= 1
                    else:
                        log.write(f"[{label}] {message}")
                        log.flush()
                        record["last_remote_output_at"] = timestamp()
                except queue.Empty:
                    pass
                if time.monotonic() - last_heartbeat >= 30:
                    record["remote_heartbeat_at"] = timestamp()
                    record["remote_elapsed_seconds"] = round(time.monotonic() - started, 1)
                    save()
                    last_heartbeat = time.monotonic()
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=15)
        for thread in threads:
            thread.join(timeout=10)
        record["reader_threads_gone"] = all(not thread.is_alive() for thread in threads)
        record["remote_ssh_exit_code"] = process.returncode
        record["remote_ssh_gone"] = process.poll() is not None
        save()
    return int(record["remote_ssh_exit_code"])


def _receive_output(output: Path) -> dict[str, object]:
    _, size_text = _ssh(
        f'test -d "{REMOTE_ROOT}/output" && '
        f'du -sb "{REMOTE_ROOT}/output" | cut -f1',
        timeout=60,
    )
    try:
        byte_count = int(size_text.strip().splitlines()[-1])
    except (IndexError, ValueError) as error:
        raise RuntimeError("remote output size was not a single byte count") from error
    if byte_count > MAX_REMOTE_OUTPUT_BYTES:
        raise ValueError("remote output exceeds the approved 250 MiB limit")
    _, forbidden = _ssh(
        f'find "{REMOTE_ROOT}/output" '
        r'\( -name .cache -o -name keys -o -name ngc-key \) -print -quit',
        timeout=60,
    )
    if forbidden.strip():
        raise ValueError("remote output contains a forbidden cache or key path")
    destination = output / "remote-output"
    destination.mkdir(exist_ok=False)
    wsl_call(
        ["scp", *SSH_OPTIONS, "-r",
         f"{HOST_ALIAS}:{REMOTE_DIR}/output/.",
         _linux_path(destination) + "/"],
        timeout=600,
    )
    files = sorted(path for path in destination.rglob("*") if path.is_file())
    local_bytes = sum(path.stat().st_size for path in files)
    if local_bytes > MAX_REMOTE_OUTPUT_BYTES:
        raise ValueError("received output exceeds the approved 250 MiB limit")
    return {
        "remote_reported_bytes": byte_count,
        "local_bytes": local_bytes,
        "local_files": [
            {"path": path.relative_to(output).as_posix(), "bytes": path.stat().st_size,
             "sha256": _sha256(path)} for path in files
        ],
    }


def _stop_and_confirm(record: dict[str, object], save: object) -> bool:
    try:
        response = brev_call(["stop", NAME], timeout=180)
        record["stop_response"] = _redact(response, None)
    except Exception as error:
        record["stop_request_error"] = f"{type(error).__name__}: {error}"
    end = time.monotonic() + STOP_TIMEOUT_SECONDS
    polls: list[dict[str, object]] = []
    record["stop_polls"] = polls
    while True:
        try:
            _, target = _load_listing()
            polls.append({"at": timestamp(), "status": target.get("status"),
                          "health_status": target.get("health_status")})
            save()
            if target.get("status") == "STOPPED":
                return True
        except Exception as error:
            polls.append({"at": timestamp(), "lookup_error": f"{type(error).__name__}: {error}"})
            save()
        if time.monotonic() >= end:
            return False
        time.sleep(POLL_SECONDS)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-id", type=int, choices=(379, 380, 381, 382, 383, 384), default=379)
    parser.add_argument("--target-profile", choices=tuple(TARGETS), default="scale234")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--remote-script", type=Path, required=True)
    parser.add_argument("--inputs-directory", type=Path, default=EXPECTED_INPUT_ROOT)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--key-stdin", action="store_true")
    return parser


def main() -> int:
    global NAME, INSTANCE_ID, INSTANCE_TYPE, HOST_ALIAS, TARGET_PROFILE
    global BATCH_ID, REMOTE_DIR, REMOTE_ROOT, REMOTE_KEY, OUTPUT_BASENAME
    global HOURLY_ESTIMATE_USD, ESTIMATED_ENVELOPE_USD
    args = _parser().parse_args()
    target_profile = TARGETS[args.target_profile]
    TARGET_PROFILE = args.target_profile
    NAME = str(target_profile["name"])
    INSTANCE_ID = str(target_profile["id"])
    INSTANCE_TYPE = str(target_profile["instance_type"])
    HOST_ALIAS = NAME + str(target_profile["host_alias_suffix"])
    HOURLY_ESTIMATE_USD = float(target_profile["hourly_estimate_usd"])
    ESTIMATED_ENVELOPE_USD = 2 * HOURLY_ESTIMATE_USD
    watchdog_runtime.NAME = NAME
    BATCH_ID = args.batch_id
    REMOTE_DIR = f"workspace/factoryops-cosmos-{BATCH_ID}"
    REMOTE_ROOT = "$HOME/" + REMOTE_DIR
    REMOTE_KEY = f"/dev/shm/factoryops-cosmos-{BATCH_ID}/ngc-key"
    OUTPUT_BASENAME = f"cosmos-brev-smoke-{BATCH_ID}"
    output, remote_script, input_files = _validate_inputs(args)
    preview = {
        "name": NAME,
        "instance_id": INSTANCE_ID,
        "instance_type": INSTANCE_TYPE,
        "target_profile": TARGET_PROFILE,
        "ssh_host_alias": HOST_ALIAS,
        "batch_id": BATCH_ID,
        "execute": args.execute,
        "hard_stop_seconds": HARD_STOP_SECONDS,
        "estimated_envelope_usd": ESTIMATED_ENVELOPE_USD,
        "remote_script_sha256": _sha256(remote_script),
        "inputs": {name: {"sha256": _sha256(path), "bytes": path.stat().st_size}
                   for name, path in input_files.items()},
    }
    secret = _read_key() if args.execute else ""
    listing, target = _load_listing()
    _require_initial_state(target)
    if not args.execute:
        print(json.dumps(preview, ensure_ascii=True))
        return 0

    output.mkdir(parents=True, exist_ok=False)
    started = datetime.now(timezone.utc)
    deadline = time.monotonic() + HARD_STOP_SECONDS
    deadline_utc = (started + timedelta(seconds=HARD_STOP_SECONDS)).isoformat()
    unit_base = f"factoryops-cosmos-{BATCH_ID}-stop-" + uuid.uuid4().hex[:12]
    record: dict[str, object] = {
        "schema_version": 1,
        **preview,
        "status": "initializing",
        "started_at": started.isoformat(),
        "deadline_utc": deadline_utc,
        "stop_reserve_seconds": STOP_RESERVE_SECONDS,
        "launcher_pid": os.getpid(),
        "source": {"path": Path(__file__).resolve().as_posix(),
                   "sha256": _sha256(Path(__file__).resolve())},
        "host_id": INSTANCE_ID,
        "initial_target": target,
        "start_attempted": False,
        "actual_stopped": False,
        "watchdog_inactive_confirmed": False,
        "owned_processes_gone": False,
        "reader_threads_gone": False,
        "secret_transfer_attempted": False,
    }

    def save() -> None:
        _atomic_json(output / "status.json", record)

    _atomic_json(output / "initial-ls.json", listing)
    save()
    keepalive: subprocess.Popen[bytes] | None = None
    watchdog_armed = False
    try:
        record["status"] = "arming_watchdog"
        save()
        watchdog, keepalive = arm_watchdog(unit_base, deadline_utc, output / "watchdog-arm.log")
        record["watchdog"] = watchdog
        watchdog_armed = True
        record["status"] = "starting"
        record["start_attempted"] = True
        save()
        start_output = brev_call(["start", NAME], timeout=180)
        (output / "start.log").write_text(_redact(start_output, secret), encoding="utf-8")
        brev_call(["refresh"], timeout=60)
        record["status"] = "waiting_ready"
        save()
        _poll_ready(deadline, record, save)
        brev_call(["refresh"], timeout=60)

        record["status"] = "hardware_preflight"
        save()
        _, hardware = _ssh(
            "set -eu; nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader; "
            "command -v docker; docker info --format '{{.ServerVersion}}'; free -b; "
            "df -B1 \"$HOME\"",
            timeout=120,
        )
        (output / "hardware.log").write_text(_redact(hardware, secret), encoding="utf-8")
        _ssh(f'mkdir -p "{REMOTE_ROOT}"', timeout=60)
        _scp(remote_script, "run.sh")
        for name, path in input_files.items():
            _scp(path, name)
        record["secret_transfer_attempted"] = True
        save()
        _send_secret(secret)

        remaining = deadline - time.monotonic() - STOP_RESERVE_SECONDS
        if remaining <= 0:
            raise TimeoutError("stop reserve reached before remote Cosmos execution")
        record["status"] = "running_remote"
        save()
        exit_code = _run_remote(output, record, save, secret, remaining)
        secret = ""
        record["status"] = "receiving_output"
        save()
        try:
            record["received_output"] = _receive_output(output)
        except Exception as error:
            record["output_receive_error"] = f"{type(error).__name__}: {error}"
            if not exit_code:
                raise
        if exit_code:
            raise RuntimeError(f"remote Cosmos command exited {exit_code}")
        record["status"] = "passed"
    except Exception as error:
        record["status"] = "failed"
        record["error"] = _redact(f"{type(error).__name__}: {error}", secret)
    finally:
        if record["secret_transfer_attempted"]:
            try:
                _ssh(f"rm -f {REMOTE_KEY}", timeout=30, check=False)
                record["remote_key_cleanup_attempted"] = True
            except Exception as error:
                record["remote_key_cleanup_error"] = _redact(
                    f"{type(error).__name__}: {error}", secret
                )
        if record["start_attempted"]:
            record["actual_stopped"] = _stop_and_confirm(record, save)
        else:
            record["actual_stopped"] = True
        if record["actual_stopped"]:
            try:
                record["watchdog_cleanup"] = cleanup_watchdog(unit_base, keepalive)
                record["watchdog_inactive_confirmed"] = True
            except Exception as error:
                record["watchdog_cleanup_error"] = f"{type(error).__name__}: {error}"
        else:
            record["stop_error"] = "actual STOPPED was not confirmed; watchdog retained"
            record["recovery"] = {
                "watchdog_retained": watchdog_armed,
                "stop_argv": BREV + ["stop", NAME],
                "timer_status_argv": WSL + [
                    "systemctl", "--user", "status", unit_base + ".timer"
                ],
                "keepalive_pid": keepalive.pid if keepalive else None,
            }
        remote_pid = record.get("remote_ssh_pid")
        record["owned_processes_gone"] = (
            remote_pid is None or bool(record.get("remote_ssh_gone"))
        ) and (not record["actual_stopped"] or keepalive is None or keepalive.poll() is not None)
        record["finished_at"] = timestamp()
        save()
        secret = ""
    print(json.dumps({
        "status": record["status"],
        "actual_stopped": record["actual_stopped"],
        "watchdog_inactive_confirmed": record["watchdog_inactive_confirmed"],
        "owned_processes_gone": record["owned_processes_gone"],
        "error": record.get("error"),
    }, ensure_ascii=True))
    return 0 if (record["status"] == "passed" and record["actual_stopped"]
                 and record["watchdog_inactive_confirmed"]
                 and record["owned_processes_gone"]
                 and record["reader_threads_gone"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
