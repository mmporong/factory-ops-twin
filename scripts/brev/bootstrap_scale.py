"""Bound one new scale-development VM bootstrap, then confirm Brev Stop.

This does not run the fleet or modify the preserved four-robot VM.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time


BREV = ["wsl", "-d", "Ubuntu", "--", "/home/llroad/.local/bin/brev"]
NAME = "factory-ops-scale-233"
INSTANCE_TYPE = "g7e.8xlarge"
IMAGE = "nvcr.io/nvidia/isaac-sim:6.2.0"
MAX_SECONDS = 21600
HOURLY_CEILING_USD = 6.60
BUDGET_USD = 40.0


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def call(arguments: list[str], timeout: float = 60, log_path: Path | None = None) -> str:
    reply = subprocess.run(BREV + arguments, capture_output=True, timeout=timeout)
    output = (reply.stdout + reply.stderr).decode("utf-8", errors="replace")
    if log_path is not None:
        log_path.write_text(output, encoding="utf-8")
    if reply.returncode:
        raise RuntimeError(f"brev {arguments[0]} exited {reply.returncode}: {output[-3000:]}")
    return output


def target_row() -> str | None:
    rows = [row.strip() for row in call(["ls"]).splitlines()
            if row.strip().split()[:1] == [NAME]]
    if len(rows) > 1:
        raise RuntimeError("target VM identity is ambiguous")
    return rows[0] if rows else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if BUDGET_USD < MAX_SECONDS / 3600 * HOURLY_CEILING_USD:
        raise ValueError("wall limit exceeds the quoted budget envelope")
    if target_row() is not None:
        raise ValueError("preserve existing target; use a separately reviewed new batch")
    if not args.execute:
        print(json.dumps({"name": NAME, "type": INSTANCE_TYPE, "image": IMAGE,
                          "wall_cap_s": MAX_SECONDS, "budget_usd": BUDGET_USD,
                          "execute": False}))
        return 0
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    record = {"schema_version": 1, "name": NAME, "instance_type": INSTANCE_TYPE,
              "requested_disk_gb": 512, "image": IMAGE, "started_at": timestamp(),
              "scope": "new cloud hardware and SDK initialization only; no fleet or physical qualification",
              "wall_cap_s": MAX_SECONDS, "budget_usd": BUDGET_USD,
              "hourly_ceiling_estimate_usd": HOURLY_CEILING_USD,
              "local_gpu_used": False, "status": "starting", "actual_stopped": False}
    deadline = time.monotonic() + MAX_SECONDS

    def save() -> None:
        (output / "host.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")

    script = output / "bootstrap.sh"
    script.write_text("""#!/usr/bin/env bash
set -euo pipefail
work="$HOME/workspace/factoryops-scale-bootstrap-233"
mkdir -p "$work"
command -v docker
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
nproc
free -b
df -h "$HOME/workspace"
docker info --format '{{.ServerVersion}}'
timeout --signal=TERM --kill-after=30s 1800s docker pull nvcr.io/nvidia/isaac-sim:6.2.0
docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{json .RepoDigests}}'
image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
cat > "$work/probe.py" <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 640, "height": 480, "renderer": "RayTracedLighting"})
for _ in range(8):
    app.update()
print("FACTORYOPS_SCALE_SDK_INITIALIZED", flush=True)
app.close()
PY
container=factoryops-scale-bootstrap-233
trap 'docker rm -f "$container" >/dev/null 2>&1 || true' EXIT
timeout --signal=TERM --kill-after=30s 900s docker run --rm --name "$container" --gpus all --network host --ipc host \\
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 \\
  -v "$work:/workspace/bootstrap:rw" --entrypoint /bin/bash "$image" \\
  -lc 'cd /isaac-sim && ./python.sh /workspace/bootstrap/probe.py --allow-root'
echo FACTORYOPS_SCALE_BOOTSTRAP_CLOSED
""", encoding="utf-8", newline="\n")
    record["bootstrap_sha256"] = hashlib.sha256(script.read_bytes()).hexdigest()
    save()
    try:
        print("Creating one stoppable RTX PRO 6000 VM; preserved L40S unchanged", flush=True)
        call(["create", NAME, "--type", INSTANCE_TYPE, "--provider", "aws", "--stoppable",
              "--min-disk", "512", "--timeout", "600"], timeout=660, log_path=output / "create.log")
        record["created_row"] = target_row()
        if record["created_row"] is None:
            raise RuntimeError("created VM was not listed")
        record["status"] = "bootstrapping"
        save()
        call(["refresh"])
        linux_path = "/mnt/" + script.drive[0].lower() + script.as_posix()[2:]
        print("Hardware/driver/Docker check and pinned Isaac Sim initialization", flush=True)
        remaining = min(3300.0, deadline - time.monotonic() - 300)
        if remaining <= 0:
            raise TimeoutError("Stop reserve reached")
        log = call(["exec", NAME, "@" + linux_path], timeout=remaining, log_path=output / "bootstrap.log")
        if "FACTORYOPS_SCALE_BOOTSTRAP_CLOSED" not in log or "FACTORYOPS_SCALE_SDK_INITIALIZED" not in log:
            raise RuntimeError("SDK initialization/closure sentinels absent")
        record["status"] = "passed"
    except Exception as error:
        record["status"] = "failed"
        record["error"] = f"{type(error).__name__}: {error}"
        print(record["error"], flush=True)
    finally:
        try:
            if target_row() is not None:
                record["stop_response"] = call(["stop", NAME], timeout=120)
                stop_end = time.monotonic() + 240
                while time.monotonic() < stop_end:
                    row = target_row()
                    record["final_row"] = row
                    if row and row.split()[1] == "STOPPED":
                        record["actual_stopped"] = True
                        break
                    time.sleep(10)
                if not record["actual_stopped"]:
                    raise RuntimeError("Stop accepted but actual STOPPED unconfirmed")
            else:
                record["no_target_vm_listed_after_attempt"] = True
        except Exception as error:
            record["stop_error"] = f"{type(error).__name__}: {error}"
            print("STOP REQUIRES ATTENTION: " + record["stop_error"], flush=True)
        record["finished_at"] = timestamp()
        save()
    print(json.dumps({key: record.get(key) for key in ("status", "actual_stopped", "error", "stop_error")}))
    return 0 if record["status"] == "passed" and record["actual_stopped"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
