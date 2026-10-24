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
from factory_ops.line_operations_model import LineOperationsModel
from factory_ops.operations_experiment import PRODUCER_PIN_NAMES


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
    args = parser.parse_args()
    if args.lots not in range(1, 17) or any(
        not math.isfinite(value) or value <= 0 for value in (args.horizon, args.max_wall_seconds)
    ):
        parser.error("lots must be 1..16; horizon and wall limit must be finite and positive")
    config, output = args.config.resolve(), args.output_dir.resolve()
    if not config.is_dir():
        parser.error("config must exist")
    producers = set(PRODUCER_PIN_NAMES) | {
        "__init__.py", "dashboard.py", "dashboard_charging.py", "dashboard_events.py",
        "mock_mes.py", "live_control.py", "scenario_replay.py", "layout.py", "footprint.py",
        "operations/__init__.py", "operations/dispatch.py", "process_line.py", "line_operations_model.py",
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
    (output / "frozen-source/manifest.json").write_text(
        json.dumps(bindings, indent=2) + "\n", encoding="utf-8",
    )
    run_id = f"process-line-{uuid4()}"
    snapshots, error = [], None
    next_capture = 0.0
    started = time.perf_counter()
    model = None
    state = None
    status = "incomplete"
    try:
        model = LineOperationsModel(config, policy=args.policy, initial_lots=args.lots, run_id=run_id)
        while model.sim_time < args.horizon and time.perf_counter() - started < args.max_wall_seconds:
            model.advance(min(1.0, args.horizon - model.sim_time))
            state = model.snapshot()
            if model.sim_time >= next_capture and len(snapshots) < 80:
                snapshots.append(state)
                next_capture += max(1.0, args.horizon / 79)
            if state["production_line"]["returned"] == args.lots and state["traffic"]["quiescent"]:
                status = "complete"
                break
        state = validate_snapshot(model.snapshot())
    except Exception as failure:
        error = {"type": type(failure).__name__, "message": str(failure)}
        status = "failed"
    stable = all(path.is_file() and digest(path.read_bytes()) == digest(data) for path, _, data in sources)
    if not stable:
        status = "invalid"
    result = {
        "schema_version": 1, "run_id": run_id, "status": status, "error": error,
        "scope": "authored CPU process line and kinematic reservations; no physics, sensors or measured factory times",
        "workload": {"lots": args.lots, "release": "all at simulation start", "policy": args.policy},
        "limits": {"horizon_s": args.horizon, "wall_s": args.max_wall_seconds, "snapshot_count": 80},
        "elapsed_wall_s": time.perf_counter() - started,
        "source_stable_during_run": stable, "representative_snapshots": snapshots, "final_snapshot": state,
    }
    (output / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_id": run_id, "status": status, "output": str(output), "error": error}, ensure_ascii=True))
    return 0 if status == "complete" else 2 if status == "incomplete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
