"""Bound one approved existing-instance development run, preserve it, then stop."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from typing import Any


PROJECT = Path(__file__).resolve().parents[2]
ART = PROJECT / "artifacts/fleet4"
INSTANCE = "factory-ops-twin-g0"
INSTANCE_ID = "evica0x9l"
REMOTE_ROOT = "/home/ubuntu/workspace/factory-ops-g0"
WSL = ["wsl", "-d", "Ubuntu", "--"]
BREV = WSL + ["/home/llroad/.local/bin/brev"]
SSH = WSL + ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=20",
             "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=3",
             "-o", "ControlMaster=no", "-o", "ControlPath=none", INSTANCE]
NAV_IMAGE = "sha256:47bc5bc84a911b757883c115d2eb0f9a1e616dc8ba2236d45f3968773387056c"
BUILD = REMOTE_ROOT + "/build-attestations/g3-q-normal-fifo-028/build-attestation.json"
BUILD_SHA = "609e2be20d613d830ffddb3b75ebe96bedc8fd2938c209c41a5011689a498da7"
GIB = 1024 ** 3
RUN_WINDOW_S = 10800
PRESERVATION_RESERVE_S = 2400
STOP_RESERVE_S = 300


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def command(argv: list[str], *, timeout: float = 60, stdin: bytes | None = None) -> subprocess.CompletedProcess[bytes]:
    response = subprocess.run(argv, input=stdin, capture_output=True, timeout=timeout)
    if response.returncode:
        raise RuntimeError((response.stdout + response.stderr).decode(errors="replace")[-2500:])
    return response


def remote(script: str, timeout: float = 45) -> str:
    return command(SSH + ["bash", "-s"], timeout=timeout, stdin=script.encode()).stdout.decode()


def state(*, timeout: float = 45) -> str:
    reply = command(BREV + ["ls"], timeout=timeout).stdout.decode()
    rows = [row for row in reply.splitlines() if INSTANCE in row]
    if len(rows) != 1 or INSTANCE_ID not in rows[0]:
        raise RuntimeError("the approved existing Brev instance identity differs")
    return rows[0]


def linux(path: Path) -> str:
    value = path.resolve()
    if not re.fullmatch(r"[A-Za-z]:", value.drive):
        raise ValueError("WSL copies require an absolute Windows drive path")
    return "/mnt/" + value.drive[0].lower() + value.as_posix()[2:]


def remaining_timeout(deadline: float, cap_s: float, *, reserve_s: float = STOP_RESERVE_S) -> float:
    remaining = deadline - time.monotonic() - reserve_s
    if remaining <= 0:
        raise TimeoutError("operation cannot use the reserved Stop window")
    return min(cap_s, remaining)


def digest(path: Path, *, deadline: float | None = None) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            if deadline is not None:
                remaining_timeout(deadline, 1)
            value.update(chunk)
    return value.hexdigest()


def best_effort_save(path: Path, result: dict[str, Any]) -> None:
    try:
        write(path, result)
    except BaseException as error:
        result.setdefault("host_record_errors", []).append(f"{type(error).__name__}: {error}")


def stop_existing(result: dict[str, Any], deadline: float) -> None:
    """A failed query or log write must not skip the approved Stop request."""
    for attempt in range(1, 4):
        left = deadline - time.monotonic()
        try:
            if "STOPPED" in state(timeout=max(1, min(15, left))):
                result["actual_stopped_at_utc"] = now()
                return
        except BaseException as error:
            result.setdefault("stop_query_errors", []).append(str(error))
        try:
            reply = subprocess.run(BREV + ["stop", INSTANCE], capture_output=True,
                                   timeout=max(1, min(45, deadline - time.monotonic())))
            result.setdefault("stop_attempts", []).append({"attempt": attempt, "returncode": reply.returncode,
                "output": (reply.stdout + reply.stderr).decode(errors="replace")[-2000:]})
        except BaseException as error:
            result.setdefault("stop_attempts", []).append({"attempt": attempt, "error": str(error)})
        for _ in range(20):
            if time.monotonic() >= deadline:
                break
            try:
                if "STOPPED" in state(timeout=max(1, min(15, deadline - time.monotonic()))):
                    result["actual_stopped_at_utc"] = now()
                    return
            except BaseException as error:
                result.setdefault("stop_query_errors", []).append(str(error))
            time.sleep(min(2, max(0, deadline - time.monotonic())))
        if time.monotonic() >= deadline:
            break
    result["stop_error"] = "Stop requested; actual STOPPED was not confirmed within the observation window"


def confirm_closed_pair(stage: str, run: str, run_id: str, deadline: float) -> dict[str, Any]:
    """Re-query owned resources even if the launch reply or exit record was lost."""
    response = remote(f"""python3 - {stage} {run} {run_id} <<'PY'
import datetime,json,os,pathlib,re,signal,subprocess,sys,time
p=pathlib.Path(sys.argv[1]); run=sys.argv[2]; run_id=sys.argv[3]
assert p.parent==pathlib.Path('{REMOTE_ROOT}') and p.name==run
assert re.fullmatch(r'fleet4-transport-normal-[0-9]{{3}}',run)
assert re.fullmatch(r'[0-9a-f]{{32}}',run_id)
(p/'abort-requested').touch(exist_ok=True)
pidfile=p/'runner.pid'; pid=None
if (p/'launch-requested').exists() and not pidfile.is_file():
 for _ in range(5):
  if pidfile.is_file():break
  time.sleep(1)
 if not pidfile.is_file():raise RuntimeError('launch PID unknown; archive blocked')
if pidfile.is_file():
 raw=pidfile.read_text().strip(); assert raw.isdecimal() and int(raw)>1; pid=int(raw)
 def owned():
  f=pathlib.Path('/proc')/str(pid)/'cmdline'
  if not f.is_file():return False
  args=f.read_bytes().decode().split('\\0')
  return str(p/'source/scripts/brev/run-fleet4.sh') in args and str(p) in args
 if owned():
  os.kill(pid,signal.SIGTERM)
  for _ in range(60):
   if not owned():break
   time.sleep(1)
  if owned():os.kill(pid,signal.SIGKILL)
def cli(args):return subprocess.run(['docker',*args],capture_output=True,text=True,timeout=30)
def ids():
 r=cli(['ps','-aq','--no-trunc','--filter','label=factory_ops.run_id='+run_id]);r.check_returncode();return r.stdout.split()
def inspect(cid):
 r=cli(['inspect',cid])
 if not r.returncode:return json.loads(r.stdout)[0]
 check=cli(['ps','-aq','--no-trunc','--filter','id='+cid]);check.check_returncode()
 if check.stdout.strip():raise RuntimeError('owned container inspection failed')
 return None
before=ids();assert len(before)<=2;observed=[]
for cid in before:
 obj=inspect(cid)
 if obj is None:continue
 labels=obj['Config']['Labels'] or {{}}
 assert labels.get('factory_ops.run_id')==run_id and labels.get('factory_ops.run_label')==run
 assert obj['Name'] in ('/factory-ops-'+run+'-nav','/factory-ops-'+run+'-sim')
 if obj['State']['Running']:cli(['stop','-t','20',cid])
 obj=inspect(cid)
 if obj is not None:
  if obj['State']['Running']:raise RuntimeError('owned worker still running; archive blocked')
  role=obj['Name'].rsplit('-',1)[-1]
  with (p/(role+'-recovery.log')).open('xb') as f:
   subprocess.run(['docker','logs',cid],stdout=f,stderr=subprocess.STDOUT,timeout=30)
  observed.append({{'id':cid,'name':obj['Name'],'state':obj['State']}})
  cli(['rm',cid])
  if inspect(cid) is not None:raise RuntimeError('owned container removal not confirmed')
if ids():raise RuntimeError('owned containers remain; archive blocked')
proof={{'passed':True,'run_id':run_id,'run_label':run,'runner_pid':pid,
 'observed_containers':observed,'remaining_owned_containers':[],
 'observed_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}}
with (p/'closure-confirmation.json').open('x') as f:json.dump(proof,f,indent=2)
print(json.dumps(proof))
PY
""", timeout=remaining_timeout(deadline, 150))
    proof = json.loads(response)
    if (proof.get("passed") is not True or proof.get("run_id") != run_id or proof.get("run_label") != run
            or proof.get("remaining_owned_containers") != []):
        raise RuntimeError("fresh owned pair closure was not confirmed; archive blocked")
    return proof


def preflight(run: str, observation: dict[str, Any]) -> None:
    if re.fullmatch(r"fleet4-transport-normal-[0-9]{3}", run) is None:
        raise ValueError("this caller admits the normal development run only")
    observed = datetime.fromisoformat(observation["observed_at_utc"])
    if observed.tzinfo is None:
        raise ValueError("billing observation requires a timezone")
    age = (datetime.now(timezone.utc) - observed).total_seconds()
    if (observed.tzinfo is None or not 0 <= age <= 3600 or observation.get("instance_id") != INSTANCE_ID
            or observation.get("state") != "STOPPED" or observation.get("batch_budget_usd") != 15
            or observation.get("rate_ceiling_estimate_usd_per_hour") != 4.2
            or type(observation.get("displayed_credit_usd")) not in (int, float)
            or not math.isfinite(observation["displayed_credit_usd"])
            or observation["displayed_credit_usd"] < 35):
        raise ValueError("fresh billing, existing-instance identity and bounded allocation are required")
    for drive in ("D:/", "E:/"):
        if shutil.disk_usage(drive).free < 26 * GIB:
            raise ValueError(f"preserve existing archives; insufficient bounded-run space on {drive}")
    if shutil.disk_usage(PROJECT).free < GIB:
        raise ValueError("insufficient local source/telemetry space")
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=PROJECT):
        raise ValueError("commit the reviewed producer before spending compute")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--observation", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    os.chdir(PROJECT)
    observation = json.loads(args.observation.read_text(encoding="utf-8-sig"))
    preflight(args.run, observation)
    if not args.execute:
        print("preflight passed; no Brev mutation")
        return 0
    ART.mkdir(exist_ok=True)
    record = ART / f"{args.run}-host.json"
    if record.exists():
        raise ValueError("preserve prior attempts; use a new run label")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    if re.fullmatch(r"[0-9a-f]{40}", commit) is None:
        raise ValueError("producer commit identity is invalid")
    source = ART / f"{args.run}-source.tar"
    raw_commit = ART / f"{args.run}-source.commit"
    if source.exists() or raw_commit.exists():
        raise ValueError("source export must use new paths")
    command(["git", "archive", "--format=tar", "--output", str(source), commit])
    raw_commit.write_bytes(command(["git", "cat-file", "commit", commit]).stdout)
    stage = REMOTE_ROOT + "/" + args.run
    run_id = os.urandom(16).hex()
    result: dict[str, Any] = {"run_label": args.run, "run_id": run_id, "instance_id": INSTANCE_ID,
        "producer_commit": commit, "source_tar_sha256": digest(source), "started_at_utc": now(),
        "host_pid": os.getpid(), "workers_passed": False, "closed_validation": "pending",
        "preserved": False, "scope": "four-AMR development; not primary qualification or physical cargo proof"}
    write(record, result)
    compute_requested = stage_created = False
    remote_pid: int | None = None
    started_clock = time.monotonic()
    deadline = started_clock + RUN_WINDOW_S
    try:
        if "STOPPED" not in state():
            raise RuntimeError("start requires the existing environment to be stopped")
        compute_requested = True
        start_reply = command(BREV + ["start", INSTANCE], timeout=remaining_timeout(deadline, 90))
        result["start_request"] = {"returncode": start_reply.returncode,
            "output": (start_reply.stdout + start_reply.stderr).decode(errors="replace")[-2000:]}
        write(record, result)
        print("EXISTING_L40S_START_REQUESTED", flush=True)
        previous_boot_state = None
        boot_query_failures = 0
        for _ in range(100):
            try:
                row = state(timeout=remaining_timeout(deadline, 45, reserve_s=PRESERVATION_RESERVE_S))
                boot_query_failures = 0
            except RuntimeError as error:
                result.setdefault("boot_query_errors", []).append(str(error))
                boot_query_failures += 1
                if boot_query_failures >= 3:
                    raise
                time.sleep(2)
                continue
            if row != previous_boot_state:
                print("BOOT_STATE", row.strip(), flush=True)
                previous_boot_state = row
            if "RUNNING" in row and " READY " in row and "NOT READY" not in row:
                break
            time.sleep(5)
        else:
            raise RuntimeError("existing instance did not become ready within the boot bound")
        remote(f"set -euo pipefail\ntest ! -e {stage}\npython3 -c \"import shutil; assert shutil.disk_usage('{REMOTE_ROOT}').free > 12000000000\"\nmkdir {stage}\n", timeout=remaining_timeout(deadline, 45, reserve_s=PRESERVATION_RESERVE_S))
        stage_created = True
        for local, name in ((source, "source.tar"), (raw_commit, "source.commit")):
            command(WSL + ["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=20", linux(local),
                           INSTANCE + ":" + stage + "/" + name], timeout=remaining_timeout(deadline, 180, reserve_s=PRESERVATION_RESERVE_S))
        script = f"""set -euo pipefail
test "$(sha256sum {stage}/source.tar | cut -d ' ' -f 1)" = {result['source_tar_sha256']}
mkdir {stage}/source
tar --no-same-owner -xf {stage}/source.tar -C {stage}/source
docker run --rm --network none --user "$(id -u):$(id -g)" --entrypoint python3 \
 -e PYTHONPATH=/workspace/project -e PYTHONDONTWRITEBYTECODE=1 \
 -v {stage}/source:/workspace/project:ro -v {stage}:/workspace/fleet4-stage:rw \
 {NAV_IMAGE} /workspace/project/scripts/brev/fleet4_prepare.py \
 --stage /workspace/fleet4-stage --source-root /workspace/project \
 --run-id {run_id} --source-commit {commit} > {stage}/preflight.json
mkdir {stage}/build
cp {BUILD} {stage}/build/build-attestation.json
cp $(dirname {BUILD})/build-receipt.json {stage}/build/build-receipt.json
docker image inspect {NAV_IMAGE} > {stage}/nav-image-inspect.json
docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 > {stage}/isaac-image-inspect.json
test ! -e {stage}/abort-requested
touch {stage}/launch-requested
test ! -e {stage}/abort-requested
nohup bash {stage}/source/scripts/brev/run-fleet4.sh {stage} {run_id} {commit} {NAV_IMAGE} {BUILD} {BUILD_SHA} > {stage}/runner.log 2>&1 < /dev/null &
runner_pid=$!
printf '%s\\n' "$runner_pid" > {stage}/runner.pid
echo "$runner_pid"
"""
        reply = remote(script, timeout=remaining_timeout(deadline, 180, reserve_s=PRESERVATION_RESERVE_S))
        remote_pid = int(reply.strip().splitlines()[-1])
        result.update(remote_runner_pid=remote_pid, remote_stage=stage, compute_ready_at_utc=now())
        write(record, result)
        print("FOUR_AMR_DEVELOPMENT_STARTED", run_id, flush=True)
        last_progress = ""
        while time.monotonic() < deadline - PRESERVATION_RESERVE_S:
            body = remote(f"""python3 - {stage} <<'PY'
import json,pathlib,sys
p=pathlib.Path(sys.argv[1]); result={{'finished':(p/'runner-exit.json').is_file()}}
pid=p/'runner.pid'
if pid.is_file():
 raw=pid.read_text().strip()
 result['runner_alive']=raw.isdecimal() and (pathlib.Path('/proc')/raw/'cmdline').is_file()
for key,path in [('dashboard',p/'results/run/dashboard-state.json'),('exit',p/'runner-exit.json')]:
 if path.is_file():
  try:result[key]=json.loads(path.read_text())
  except (OSError,ValueError):pass
actions=p/'results/run/actions.jsonl'
if actions.is_file():
 with actions.open('rb') as f:
  f.seek(max(0,actions.stat().st_size-8192))
  for line in reversed(f.read().splitlines()):
   try:result['progress']=json.loads(line);break
   except ValueError:pass
print(json.dumps(result))
PY
""", timeout=remaining_timeout(deadline, 30, reserve_s=PRESERVATION_RESERVE_S))
            sample = json.loads(body)
            if "dashboard" in sample:
                write(ART / f"{args.run}-dashboard-state.json", sample["dashboard"])
            write(ART / f"{args.run}-observation.json", {"observed_at_utc": now(), **sample})
            progress = sample.get("progress", {})
            fingerprint = str(progress.get("kind")) + str(int(progress.get("sim_time_s", 0)) // 15)
            if fingerprint != last_progress:
                print("PROGRESS", progress.get("kind"), progress.get("sim_time_s"), flush=True)
                last_progress = fingerprint
            if sample["finished"]:
                result["runner_exit"] = sample.get("exit")
                result["workers_passed"] = (sample.get("exit", {}).get("runner_exit_code") == 0
                    and sample.get("exit", {}).get("cleanup_passed") is True
                    and all(sample.get("exit", {}).get(label, {}).get("ExitCode") == 0
                            and sample.get("exit", {}).get(label, {}).get("Running") is False for label in ("nav", "sim")))
                break
            if sample.get("runner_alive") is False:
                raise RuntimeError("remote runner exited without a sealed exit record")
            time.sleep(2)
        else:
            raise RuntimeError("bounded compute window reached; preserve and stop")
    except BaseException as error:
        result["error"] = {"type": type(error).__name__, "message": str(error)}
        print("DEVELOPMENT_ATTEMPT_ERROR", type(error).__name__, str(error), flush=True)
    finally:
        try:
            if stage_created:
                try:
                    result["closure_confirmation"] = confirm_closed_pair(stage, args.run, run_id, deadline)
                    archive_path = stage + ".tar.gz"
                    packed = remote(f"set -euo pipefail\ntest ! -e {archive_path}\ntar -C {REMOTE_ROOT} -czf {archive_path} {args.run}\nsha256sum {archive_path}\n",
                                    timeout=remaining_timeout(deadline, 600)).strip().split()[0]
                    destinations = []
                    for drive in ("D:/", "E:/"):
                        folder = Path(drive) / "FactoryOpsArtifacts/fleet4-transport"
                        folder.mkdir(parents=True, exist_ok=True)
                        target = folder / (args.run + ".tar.gz")
                        if target.exists():
                            raise ValueError("preserve existing raw archives")
                        command(WSL + ["scp", "-o", "BatchMode=yes", INSTANCE + ":" + archive_path,
                                       linux(target)], timeout=remaining_timeout(deadline, 600))
                        if digest(target, deadline=deadline) != packed:
                            raise RuntimeError("preserved archive hash differs")
                        destinations.append({"path": str(target), "bytes": target.stat().st_size, "sha256": packed})
                    result.update(preserved=True, archive_sha256=packed, backups=destinations)
                    print("RAW_PRESERVED_D_E", packed, flush=True)
                except BaseException as error:
                    result["preservation_error"] = str(error)
                best_effort_save(record, result)
        finally:
            if compute_requested:
                try:
                    stop_existing(result, deadline)
                except BaseException as error:
                    result["stop_error"] = str(error)
            result["finished_at_utc"] = now()
            result["elapsed_wall_s"] = time.monotonic() - started_clock
            result["time_window_exceeded"] = time.monotonic() > deadline
            best_effort_save(record, result)
    # A worker exit is deliberately distinct from closed-result validation.
    return 0 if (result["workers_passed"] and result["preserved"] and result.get("actual_stopped_at_utc")
                 and not result.get("host_record_errors") and not result["time_window_exceeded"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
