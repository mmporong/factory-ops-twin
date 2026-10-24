"""Run a bounded CPU process-line pilot with a small, source-bound record."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from factory_ops.dashboard import validate_snapshot
from factory_ops.compat import replace_shared_file, write_shared_text
from factory_ops.line_operations_model import LineOperationsModel
from factory_ops.operations_experiment import PRODUCER_PIN_NAMES
from factory_ops.operations_recording import SnapshotRecorder


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--lots", type=int, default=12)
    parser.add_argument("--policy", choices=("fifo", "eta"), default="fifo")
    parser.add_argument("--horizon", type=float, default=3600)
    parser.add_argument("--max-wall-seconds", type=float, default=300)
    parser.add_argument("--stall-timeout", type=float, default=600,
                        help="stop incomplete after this many simulation seconds without motion or process progress")
    parser.add_argument("--record-start", type=float, default=600)
    parser.add_argument("--record-duration", type=float, default=0,
                        help="optional observed 1Hz media window, up to 300 simulation seconds")
    args = parser.parse_args()
    if args.lots not in range(1, 65) or any(
        not math.isfinite(value) or value <= 0 for value in (args.horizon, args.max_wall_seconds, args.stall_timeout)
    ):
        parser.error("lots must be 1..64; horizon and wall limit must be finite and positive")
    if (not math.isfinite(args.record_start) or args.record_start < 0
            or not math.isfinite(args.record_duration) or not 0 <= args.record_duration <= 300):
        parser.error("record start must be nonnegative; duration must be 0..300 seconds")
    config, output = args.config.resolve(), args.output_dir.resolve()
    if not config.is_dir():
        parser.error("config must exist")
    producers = set(PRODUCER_PIN_NAMES) | {
        "__init__.py", "dashboard.py", "dashboard_charging.py", "dashboard_events.py",
        "mock_mes.py", "live_control.py", "scenario_replay.py", "layout.py", "footprint.py",
        "operations/__init__.py", "operations/dispatch.py", "process_line.py", "line_operations_model.py", "process_network.py",
        "operations_recording.py", "operations_bridge.py", "compat.py",
    }
    selected = [(path, Path("config") / path.name) for path in sorted(config.glob("*.json"))]
    selected += [(ROOT / "factory_ops" / name, Path("factory_ops") / name) for name in sorted(producers)]
    selected.append((Path(__file__), Path("scripts/demo/run_process_line.py")))
    # Read every required input before creating the new output directory.
    sources = [(path, relative, path.read_bytes()) for path, relative in selected]
    output.mkdir(parents=True, exist_ok=False)
    bindings = []
    for path, relative, data in sources:
        frozen = output / "frozen-source" / relative
        frozen.parent.mkdir(parents=True, exist_ok=True)
        frozen.write_bytes(data)
        bindings.append({"path": relative.as_posix(), "sha256": digest(data), "bytes": len(data)})
    frozen_manifest_path = output / "frozen-source/manifest.json"
    frozen_manifest_path.write_text(json.dumps(bindings, indent=2) + "\n", encoding="utf-8")
    source_pins = {"config": {}, "producer": {}}
    for binding in bindings:
        parts = Path(binding["path"]).parts
        if len(parts) == 2 and parts[0] == "config":
            source_pins["config"][parts[1]] = binding["sha256"]
        else:
            source_pins["producer"][binding["path"]] = binding["sha256"]
    source_binding_sha256 = digest(json.dumps(
        source_pins, ensure_ascii=False, sort_keys=True, separators=(",", ":"),
    ).encode("utf-8"))
    run_id = f"process-line-{uuid4()}"
    snapshots, error = [], None
    next_capture = 0.0
    started = time.perf_counter()
    model = None
    state = None
    status = "incomplete"
    stop_reason = "horizon_or_wall_limit"
    last_progress_s = 0.0
    last_signature = None
    recorder = (SnapshotRecorder(output / "snapshots.jsonl.gz", start_s=args.record_start,
                                duration_s=args.record_duration, sample_hz=1)
                if args.record_duration else None)
    try:
        model = LineOperationsModel(config, policy=args.policy, initial_lots=args.lots, run_id=run_id)
        while model.sim_time < args.horizon and time.perf_counter() - started < args.max_wall_seconds:
            model.advance(min(1.0, args.horizon - model.sim_time))
            state = model.snapshot()
            signature = (
                tuple(round(value, 8) for value in state["traffic"]["travel_m"].values()),
                state["production_line"]["revision"],
                tuple(cell["remaining_s"] for cell in state["production_line"]["cells"].values()),
                tuple((robot["id"], robot.get("battery_pct")) for robot in state["robots"]
                      if robot.get("charging_phase") == "charging"),
            )
            if signature != last_signature:
                last_progress_s, last_signature = model.sim_time, signature
            if recorder is not None:
                recorder(state)
            if model.sim_time >= next_capture and len(snapshots) < 80:
                snapshots.append(state)
                next_capture += max(1.0, args.horizon / 79)
            if state["production_line"]["returned"] == args.lots and state["traffic"]["quiescent"]:
                status = "complete"
                stop_reason = "all_lots_returned_and_quiescent"
                break
            if int(model.sim_time + 1e-5) % 30 == 0:
                progress = {"run_id": run_id, "sim_time_s": model.sim_time,
                            "elapsed_wall_s": time.perf_counter() - started,
                            "completed_lots": state["production_line"]["completed_lots"],
                            "returned": state["production_line"]["returned"],
                            "traffic": state["traffic"]}
                temporary = output / ".progress.tmp"
                write_shared_text(temporary, json.dumps(progress, ensure_ascii=True))
                replace_shared_file(temporary, output / "progress.json")
            if model.sim_time - last_progress_s >= args.stall_timeout:
                stop_reason = "no_motion_or_process_progress"
                break
        state = validate_snapshot(model.snapshot())
    except Exception as failure:
        error = {"type": type(failure).__name__, "message": str(failure)}
        status = "failed"
        stop_reason = "runtime_error"
    finally:
        if recorder is not None:
            recorder.close()
    stable = all(path.is_file() and digest(path.read_bytes()) == digest(data) for path, _, data in sources)
    if not stable:
        status = "invalid"
    result = {
        "schema_version": 1, "run_id": run_id, "status": status, "error": error, "stop_reason": stop_reason,
        "scope": "authored CPU process line and kinematic reservations; no physics, sensors or measured factory times",
        "workload": {"lots": args.lots, "release": "all at simulation start", "policy": args.policy},
        "limits": {"horizon_s": args.horizon, "wall_s": args.max_wall_seconds, "snapshot_count": 80,
                   "stall_timeout_s": args.stall_timeout},
        "elapsed_wall_s": time.perf_counter() - started,
        "source_stable_during_run": stable, "representative_snapshots": snapshots, "final_snapshot": state,
    }
    (output / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if recorder is not None:
        recording = recorder.evidence()
        manifest = {"schema_version": 1, "run_id": run_id, "case_status": status,
                    "frame_count": recording["frames"], "snapshots": recording,
                    "case_report": {"path": "result.json", "sha256": digest((output / "result.json").read_bytes())},
                    "sources": {
                        "config_and_producer_sha256": source_pins,
                        "source_stable_during_run": stable,
                        "frozen_source": {
                            "relative_path": "frozen-source",
                            "manifest_relative_path": "frozen-source/manifest.json",
                            "manifest_sha256": digest(frozen_manifest_path.read_bytes()),
                            "source_binding_sha256": source_binding_sha256,
                            "file_count": len(bindings),
                        },
                    },
                    "scope": "observed CPU operations snapshots; no interpolation or physics claim"}
        (output / "recording-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_id": run_id, "status": status, "output": str(output), "error": error}, ensure_ascii=True))
    return 0 if status == "complete" else 2 if status == "incomplete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
