"""Bounded deterministic comparisons for the CPU operations model.

The experiment runner records representative state and aggregate evidence.  It
does not emit every simulation step and makes no physics or real-factory claim.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Mapping, Sequence
from uuid import uuid4

from .operations_model import OperationsModel


SCENARIOS = ("normal", "tester_down", "aisle_closed")
POLICIES = ("fifo", "eta")
SOURCES = ("S1", "S3", "S2", "S1", "S2", "S3", "S1", "S3", "S2", "S1", "S2", "S3")
ORDER_COUNT = len(SOURCES)
RELEASE_INTERVAL_S = 30.0
AUTHORED_DUE_OFFSET_S = 240.0
TESTER_DOWN_START_S = 120.0
TESTER_DOWN_END_S = 300.0
MAX_REPRESENTATIVE_SNAPSHOTS = 120
MAX_EVENT_EXAMPLES = 24
CONFIG_PIN_NAMES = (
    "layout.json", "g2.json", "g3-routing.json", "transport-profile.json",
    "operations-charging.json",
)
PRODUCER_PIN_NAMES = (
    "charging.py", "dashboard_model.py", "drive_profile.py", "energy.py",
    "fleet_routing.py", "motion.py", "operations_charging.py",
    "operations_experiment.py", "operations_model.py", "operations_motion.py",
    "operations_corridor.py", "operations_replan.py", "route_reservation.py", "operations/inventory.py",
    "operations/ledger.py", "operations/model.py",
)


def _finite_number(value: Any, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    result = float(value)
    if positive and result <= 0:
        raise ValueError(f"{label} must be positive")
    return result


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _config_pins(config: Path) -> dict[str, str]:
    return {name: _file_sha256(config / name) for name in CONFIG_PIN_NAMES if (config / name).is_file()}


def _producer_pins() -> dict[str, str]:
    package = Path(__file__).resolve().parent
    return {f"factory_ops/{name}": _file_sha256(package / name)
            for name in PRODUCER_PIN_NAMES if (package / name).is_file()}


def _freeze_sources(config: Path, output: Path) -> dict[str, Any]:
    """Copy only the explicit experiment inputs before a CLI run begins."""
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f"experiment output already exists: {output}")
    frozen = output.parent / "frozen-source"
    if frozen.exists():
        raise FileExistsError(f"frozen source already exists: {frozen}")
    package = Path(__file__).resolve().parent
    selected: list[tuple[str, Path, Path]] = []
    for name in CONFIG_PIN_NAMES:
        source = config / name
        if source.is_file():
            selected.append(("config", source, Path("config") / name))
    for name in PRODUCER_PIN_NAMES:
        source = package / name
        if source.is_file():
            selected.append(("producer", source, Path("factory_ops") / name))
    if not selected:
        raise ValueError("no explicit experiment source files are available to freeze")

    frozen.mkdir(parents=True)
    records = []
    binding: dict[str, dict[str, str]] = {"config": {}, "producer": {}}
    for kind, source, relative in selected:
        data = source.read_bytes()
        digest = _sha_bytes(data)
        target = frozen / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        if _file_sha256(target) != digest:
            raise OSError(f"frozen source verification failed: {relative.as_posix()}")
        pin_key = source.name if kind == "config" else relative.as_posix()
        binding[kind][pin_key] = digest
        records.append({
            "kind": kind,
            "source_key": pin_key,
            "frozen_path": relative.as_posix(),
            "sha256": digest,
            "bytes": len(data),
        })
    manifest = {
        "schema_version": 1,
        "scope": "explicit operations experiment production modules and selected config; no directory-wide copy",
        "source_binding_sha256": _canonical_sha256(binding),
        "files": records,
    }
    manifest_path = frozen / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {
        "relative_path": "frozen-source",
        "manifest_relative_path": "frozen-source/manifest.json",
        "manifest_sha256": _file_sha256(manifest_path),
        "source_binding_sha256": manifest["source_binding_sha256"],
        "file_count": len(records),
        "binding": binding,
    }


def _sha_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _attach_frozen_source(payload: dict[str, Any], frozen: Mapping[str, Any]) -> None:
    cases = payload.get("cases")
    rows = cases if isinstance(cases, list) else [payload]
    reference = frozen["binding"]
    for row in rows:
        if not isinstance(row, dict):
            continue
        provenance = row.get("provenance")
        if not isinstance(provenance, dict):
            provenance = {}
            row["provenance"] = provenance
        matches = provenance.get("sources_before_sha256") == reference
        provenance["frozen_source"] = {
            "relative_path": frozen["relative_path"],
            "manifest_relative_path": frozen["manifest_relative_path"],
            "manifest_sha256": frozen["manifest_sha256"],
            "source_binding_sha256": frozen["source_binding_sha256"],
            "binding_matches_case_start": matches,
        }
        if not matches:
            row["status"] = "invalid"
            errors = row.setdefault("validation_errors", [])
            if isinstance(errors, list):
                errors.append("case source binding differs from the pre-run frozen source")
    payload["frozen_source"] = {
        key: frozen[key] for key in (
            "relative_path", "manifest_relative_path", "manifest_sha256",
            "source_binding_sha256", "file_count",
        )
    }


def _fleet_quiescence(snapshot: Mapping[str, Any]) -> tuple[bool, str]:
    traffic = snapshot.get("traffic")
    if isinstance(traffic, Mapping) and type(traffic.get("quiescent")) is bool:
        return bool(traffic["quiescent"]), "traffic.quiescent"
    return False, "unavailable_assumed_false"


def _task_rows(snapshot: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    tasks = snapshot.get("tasks", [])
    return [row for row in tasks if isinstance(row, Mapping)] if isinstance(tasks, list) else []


def _completed_orders(snapshot: Mapping[str, Any]) -> int:
    kpis = snapshot.get("kpis")
    if isinstance(kpis, Mapping):
        value = kpis.get("completed_orders")
        if type(value) is int and value >= 0:
            return value
    completed = {
        row.get("order_id") for row in _task_rows(snapshot)
        if row.get("type") == "outbound" and row.get("state") == "complete" and row.get("order_id")
    }
    return len(completed)


def _wip(snapshot: Mapping[str, Any]) -> int:
    equipment = snapshot.get("equipment")
    if isinstance(equipment, Mapping) and type(equipment.get("wip_orders")) is int:
        return max(0, int(equipment["wip_orders"]))
    kpis = snapshot.get("kpis")
    if isinstance(kpis, Mapping) and type(kpis.get("wip_orders")) is int:
        return max(0, int(kpis["wip_orders"]))
    states: dict[str, bool] = {}
    for row in _task_rows(snapshot):
        order_id = row.get("order_id")
        if isinstance(order_id, str) and order_id:
            states.setdefault(order_id, False)
            if row.get("type") == "outbound" and row.get("state") == "complete":
                states[order_id] = True
    return sum(not complete for complete in states.values())


def _traffic_wait(snapshot: Mapping[str, Any]) -> float:
    traffic = snapshot.get("traffic")
    if isinstance(traffic, Mapping):
        wait_by_robot = traffic.get("wait_s")
        if isinstance(wait_by_robot, Mapping):
            values = list(wait_by_robot.values())
            if all(isinstance(value, (int, float)) and not isinstance(value, bool)
                   and math.isfinite(value) and value >= 0 for value in values):
                return sum(float(value) for value in values)
        for key in ("cumulative_wait_s", "total_wait_s", "wait_time_s"):
            value = traffic.get(key)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
                return max(0.0, float(value))
    kpis = snapshot.get("kpis")
    if isinstance(kpis, Mapping):
        value = kpis.get("traffic_wait_s")
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            return max(0.0, float(value))
    return 0.0


def _compact_snapshot(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """Keep bounded evidence needed to explain a comparison result."""
    robots = snapshot.get("robots", [])
    robot_rows = []
    if isinstance(robots, list):
        for row in robots:
            if not isinstance(row, Mapping):
                continue
            robot_rows.append({key: row.get(key) for key in (
                "id", "status", "task_id", "task_type", "destination", "waiting_reason", "pose",
                "blockers", "navigation_phase", "speed_mps"
            )})
    equipment = snapshot.get("equipment")
    traffic = snapshot.get("traffic")
    return {
        "sim_time_s": snapshot.get("sim_time_s"),
        "robots": robot_rows,
        "equipment": dict(equipment) if isinstance(equipment, Mapping) else {},
        "traffic": dict(traffic) if isinstance(traffic, Mapping) else {},
        "kpis": dict(snapshot.get("kpis", {})) if isinstance(snapshot.get("kpis"), Mapping) else {},
    }


def _event_summary(event_by_id: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    ordered = sorted(
        event_by_id.values(),
        key=lambda event: (float(event.get("sim_time_s", 0.0)), str(event.get("id", ""))),
    )
    kinds = Counter(str(event.get("type", "unknown")) for event in ordered)
    examples = ordered[-MAX_EVENT_EXAMPLES:]
    return {"total_unique": len(ordered), "counts_by_type": dict(sorted(kinds.items())), "examples": examples}


def _completion_times(snapshot: Mapping[str, Any], known: dict[str, float], now: float) -> None:
    for row in _task_rows(snapshot):
        order_id = row.get("order_id")
        if (isinstance(order_id, str) and order_id and row.get("type") == "outbound"
                and row.get("state") == "complete"):
            known.setdefault(order_id, now)


def _final_metrics(
    final: Mapping[str, Any],
    releases: Mapping[str, float],
    completion_times: Mapping[str, float],
    *,
    evaluation_end_s: float,
    wip_time_integral: float,
    peak_wip: int,
    participants: set[str],
    traffic_wait_s: float,
    elapsed_wall_s: float,
) -> dict[str, Any]:
    completed = _completed_orders(final)
    due_times = {order: released + AUTHORED_DUE_OFFSET_S for order, released in releases.items()}
    completed_tardiness = {
        order: max(0.0, completion_times[order] - due_times[order])
        for order in completion_times if order in due_times
    }
    incomplete = sorted(set(releases) - set(completion_times))
    incomplete_lower_bound = sum(max(0.0, evaluation_end_s - due_times[order]) for order in incomplete)
    tasks = _task_rows(final)
    returns_completed = sum(row.get("type") == "empty_return" and row.get("state") == "complete" for row in tasks)
    returns_pending = sum(row.get("type") == "empty_return" and row.get("state") != "complete" for row in tasks)
    delivered = sum(row.get("type") == "inbound" and row.get("state") == "complete" for row in tasks)
    outbound_delivered = sum(row.get("type") == "outbound" and row.get("state") == "complete" for row in tasks)
    missed_due = sum(value > 0 for value in completed_tardiness.values())
    missed_due += sum(evaluation_end_s > due_times[order] for order in incomplete)
    total_tardiness_lower_bound = sum(completed_tardiness.values()) + incomplete_lower_bound
    unreleased = max(0, ORDER_COUNT - len(releases))
    order_demand_finished = len(releases) == ORDER_COUNT and completed == ORDER_COUNT
    material_cycles_completed = min(completed, returns_completed)
    material_cycle_finished = (
        order_demand_finished and returns_completed == ORDER_COUNT and returns_pending == 0
    )
    fleet_quiescent, quiescence_source = _fleet_quiescence(final)
    return {
        "released_orders": len(releases),
        "demanded_orders": ORDER_COUNT,
        "unreleased_demand": unreleased,
        "completed_cycles": completed,
        "incomplete_orders": max(0, ORDER_COUNT - completed),
        "released_incomplete_order_ids": incomplete,
        "delivered_to_cell": delivered,
        "completed_outbound_deliveries": outbound_delivered,
        "empty_returns_completed": returns_completed,
        "empty_returns_pending": returns_pending,
        "material_cycles_completed": material_cycles_completed,
        "due_offset_s": AUTHORED_DUE_OFFSET_S,
        "missed_due_orders": missed_due,
        "completed_tardiness_s": round(sum(completed_tardiness.values()), 6),
        "incomplete_tardiness_lower_bound_s": round(incomplete_lower_bound, 6),
        "total_tardiness_lower_bound_s": round(total_tardiness_lower_bound, 6),
        "mean_wip": round(wip_time_integral / evaluation_end_s if evaluation_end_s else 0.0, 6),
        "peak_wip": peak_wip,
        "robot_participation_count": len(participants),
        "robot_participants": sorted(participants),
        "traffic_wait_s": round(traffic_wait_s, 6),
        "elapsed_wall_s": round(elapsed_wall_s, 6),
        "finished_all_demand": order_demand_finished,
        "finished_all_material_cycles": material_cycle_finished,
        "business_finished": material_cycle_finished,
        "fleet_quiescent": fleet_quiescent,
        "fleet_quiescence_source": quiescence_source,
        "experiment_complete": material_cycle_finished and fleet_quiescent,
    }


def run_case(
    config: Path,
    *,
    scenario: str,
    policy: str,
    seed: int = 17,
    horizon_s: float = 3600.0,
    dt_s: float = 0.1,
    model_factory: Callable[..., Any] = OperationsModel,
    snapshot_observer: Callable[[Mapping[str, Any]], None] | None = None,
) -> dict[str, Any]:
    """Run one authored workload and return bounded comparison evidence."""
    if scenario not in SCENARIOS:
        raise ValueError(f"scenario must be one of {', '.join(SCENARIOS)}")
    if policy not in POLICIES:
        raise ValueError(f"policy must be one of {', '.join(POLICIES)}")
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    horizon_s = _finite_number(horizon_s, "horizon_s", positive=True)
    dt_s = _finite_number(dt_s, "dt_s", positive=True)
    if dt_s > 1:
        raise ValueError("dt_s must be at most one second")
    if snapshot_observer is not None and not callable(snapshot_observer):
        raise ValueError("snapshot_observer must be callable or null")
    config = Path(config).resolve()
    if not config.is_dir():
        raise ValueError("config must be an existing directory")

    source_pins_before = {"config": _config_pins(config), "producer": _producer_pins()}
    case_spec = {
        "scenario": scenario, "dispatch_policy": policy, "seed": seed,
        "horizon_s": horizon_s, "dt_s": dt_s, "sources": list(SOURCES),
        "release_interval_s": RELEASE_INTERVAL_S,
        "source_binding_sha256": _canonical_sha256(source_pins_before),
    }
    case_id = _canonical_sha256(case_spec)[:16]
    run_id = f"operations-{scenario}-{policy}-{case_id}-{uuid4().hex[:12]}"
    started = time.perf_counter()
    release_index = 0
    releases: dict[str, float] = {}
    completion_times: dict[str, float] = {}
    representative: list[dict[str, Any]] = []
    events: dict[str, Mapping[str, Any]] = {}
    participants: set[str] = set()
    peak_wip = 0
    wip_time_integral = 0.0
    traffic_wait_s = 0.0
    capture_interval = max(dt_s, horizon_s / (MAX_REPRESENTATIVE_SNAPSHOTS - 1))
    next_capture = 0.0
    down_applied = False
    recovery_applied = False
    observer_calls = 0

    snapshot: Mapping[str, Any] = {}
    failure: dict[str, Any] | None = None
    try:
        model = model_factory(config, policy=policy, initial_orders=0, run_id=run_id)
        if scenario == "aisle_closed":
            model.set_aisle_closed({"closed": True})
        snapshot = model.snapshot()
        while float(snapshot.get("sim_time_s", 0.0)) < horizon_s - 1e-9:
            now = float(snapshot.get("sim_time_s", 0.0))
            while release_index < ORDER_COUNT and release_index * RELEASE_INTERVAL_S <= now + 1e-9:
                source = SOURCES[release_index]
                order_id = model.create_order({"source": source, "quantity": 1})
                releases[str(order_id)] = release_index * RELEASE_INTERVAL_S
                release_index += 1
            if scenario == "tester_down" and not down_applied and now >= TESTER_DOWN_START_S - 1e-9:
                model.set_equipment_availability({"available": False, "reason": "authored comparison outage"})
                down_applied = True
            if scenario == "tester_down" and not recovery_applied and now >= TESTER_DOWN_END_S - 1e-9:
                model.set_equipment_availability({"available": True, "reason": "authored comparison recovery"})
                recovery_applied = True

            snapshot = model.snapshot()
            step = min(dt_s, horizon_s - now)
            current_wip = _wip(snapshot)
            wip_time_integral += current_wip * step
            peak_wip = max(peak_wip, current_wip)
            model.advance(step)
            snapshot = model.snapshot()
            if snapshot_observer is not None:
                snapshot_observer(deepcopy(snapshot))
                observer_calls += 1
            observed_time = float(snapshot.get("sim_time_s", now + step))
            _completion_times(snapshot, completion_times, observed_time)
            traffic_wait_s = max(traffic_wait_s, _traffic_wait(snapshot))
            for row in _task_rows(snapshot):
                robot_id = row.get("robot_id")
                if isinstance(robot_id, str) and robot_id:
                    participants.add(robot_id)
            raw_events = snapshot.get("events", [])
            if isinstance(raw_events, list):
                for index, event in enumerate(raw_events):
                    if isinstance(event, Mapping):
                        event_id = str(event.get("id", f"{observed_time:.6f}:{index}"))
                        events[event_id] = dict(event)
            if observed_time + 1e-9 >= next_capture or observed_time >= horizon_s - 1e-9:
                representative.append(_compact_snapshot(snapshot))
                next_capture += capture_interval
    except Exception as exc:
        failure = {"type": type(exc).__name__, "message": str(exc),
                   "sim_time_s": snapshot.get("sim_time_s", 0.0), "release_index": release_index}

    elapsed = time.perf_counter() - started
    metrics = _final_metrics(
        snapshot, releases, completion_times,
        evaluation_end_s=min(horizon_s, float(snapshot.get("sim_time_s", 0.0))),
        wip_time_integral=wip_time_integral, peak_wip=peak_wip,
        participants=participants, traffic_wait_s=traffic_wait_s, elapsed_wall_s=elapsed,
    )
    ledger_projection = {
        "tasks": snapshot.get("tasks", []),
        "inventory": snapshot.get("inventory", {}),
        "equipment": snapshot.get("equipment", {}),
        "kpis": snapshot.get("kpis", {}),
    }
    source_check_error = None
    try:
        source_pins_after = {"config": _config_pins(config), "producer": _producer_pins()}
        source_stable = source_pins_before == source_pins_after
    except Exception as exc:
        source_pins_after = {}
        source_stable = False
        source_check_error = {"type": type(exc).__name__, "message": str(exc)}
    if not source_stable:
        status = "invalid"
    elif failure is not None:
        status = "failed"
    elif metrics["experiment_complete"]:
        status = "complete"
    else:
        status = "incomplete"
    return {
        "schema_version": 1,
        "run_id": run_id,
        "case_identity": {"id": case_id, "spec": case_spec},
        "status": status,
        "error": failure,
        "execution_mode": "operations",
        "scenario": scenario,
        "dispatch_policy": policy,
        "seed": seed,
        "seed_role": "identity_only_no_stochastic_inputs",
        "workload": {
            "order_count": ORDER_COUNT,
            "release_interval_s": RELEASE_INTERVAL_S,
            "sources": list(SOURCES),
            "due_offset_s": AUTHORED_DUE_OFFSET_S,
            "tester_down_window_s": [TESTER_DOWN_START_S, TESTER_DOWN_END_S]
            if scenario == "tester_down" else None,
            "aisle_closed_at_s": 0.0 if scenario == "aisle_closed" else None,
        },
        "simulation": {"horizon_s": horizon_s, "dt_s": dt_s},
        "recording_budget": {
            "raw_timestep_records": False,
            "maximum_representative_snapshots": MAX_REPRESENTATIVE_SNAPSHOTS,
            "maximum_event_examples": MAX_EVENT_EXAMPLES,
            "event_counts_cover_all_unique_events": True,
        },
        "snapshot_observer": {
            "enabled": snapshot_observer is not None,
            "calls_completed": observer_calls,
            "copy_mode": "deepcopy" if snapshot_observer is not None else "none",
            "scope": (
                "optional read-only same-run capture hook; media/capture performance is separate from "
                "operations metrics and its wall cost is not a policy performance claim"
            ),
        },
        "metrics": metrics,
        "events": _event_summary(events),
        "representative_snapshots": representative[:MAX_REPRESENTATIVE_SNAPSHOTS],
        "final_snapshot": _compact_snapshot(snapshot),
        "provenance": {
            "scope": "authored deterministic CPU operations comparison; no physics or calibrated production claim",
            "config_path": str(config),
            "source_stable_during_run": source_stable,
            "source_check_error": source_check_error,
            "sources_before_sha256": source_pins_before,
            "sources_after_sha256": source_pins_after,
            "ledger_projection_sha256": _canonical_sha256(ledger_projection),
        },
    }


def run_matrix(config: Path, *, seed: int, horizon_s: float, dt_s: float) -> dict[str, Any]:
    cases = [
        run_case(config, scenario=scenario, policy=policy, seed=seed, horizon_s=horizon_s, dt_s=dt_s)
        for scenario in SCENARIOS for policy in POLICIES
    ]
    return {
        "schema_version": 1,
        "execution_mode": "operations",
        "matrix": {"scenarios": list(SCENARIOS), "policies": list(POLICIES), "case_count": len(cases)},
        "cases": cases,
    }


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path = path.resolve()
    if path.exists():
        raise FileExistsError(f"experiment output already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a bounded CPU operations comparison")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--horizon", type=float, default=3600.0)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--scenario", choices=SCENARIOS, default="normal")
    parser.add_argument("--policy", choices=POLICIES, default="fifo")
    parser.add_argument("--all", action="store_true", help="run the explicit three-scenario/two-policy matrix")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = args.config.resolve()
    output = args.output.resolve()
    if not config.is_dir():
        raise ValueError("config must be an existing directory")
    _finite_number(args.horizon, "horizon_s", positive=True)
    dt_s = _finite_number(args.dt, "dt_s", positive=True)
    if dt_s > 1:
        raise ValueError("dt_s must be at most one second")
    frozen = _freeze_sources(config, output)
    payload = (run_matrix(config, seed=args.seed, horizon_s=args.horizon, dt_s=args.dt)
               if args.all else run_case(config, scenario=args.scenario, policy=args.policy,
                                         seed=args.seed, horizon_s=args.horizon, dt_s=args.dt))
    _attach_frozen_source(payload, frozen)
    _write_json(output, payload)
    print(json.dumps({"output": str(output), "cases": len(payload.get("cases", [payload])),
                      "frozen_source_manifest_sha256": frozen["manifest_sha256"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
