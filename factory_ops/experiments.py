from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
import hashlib
import json
from math import isfinite
from pathlib import Path
import random
import re
import sys
from typing import Any


CONDITIONS = ("normal", "closure", "burst")
POLICIES = ("fifo", "eta")
PILOT_SEED = 100
PHYSICAL_BACKEND = "isaac_sim_ros2_nav2"
FIXTURE_BACKEND = "explicit_non_evidence"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")


def _number(value: object, name: str, *, positive: bool = False, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    if positive and result <= 0:
        raise ValueError(f"{name} must be positive")
    if nonnegative and result < 0:
        raise ValueError(f"{name} must be nonnegative")
    return result


def _integer(value: object, name: str, *, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{name} must be an object with string keys")
    return value


def _sequence(value: object, name: str) -> Sequence[Any]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{name} must be an array")
    return value


def _require_keys(value: Mapping[str, Any], expected: set[str], name: str) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ValueError(f"{name} fields mismatch; missing={missing}, extra={extra}")


def _validate_json(value: object, path: str = "$") -> None:
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not isfinite(value):
            raise ValueError(f"{path} contains a non-finite number")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{path} contains a non-string object key")
            _validate_json(child, f"{path}.{key}")
        return
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for index, child in enumerate(value):
            _validate_json(child, f"{path}[{index}]")
        return
    raise ValueError(f"{path} contains a non-JSON value")


def canonical_hash(value: object) -> str:
    """Return the SHA-256 of strict canonical JSON (sorted keys, no NaN)."""
    _validate_json(value)
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_experiment_config(source: str | Path | Mapping[str, Any]) -> dict[str, Any]:
    if isinstance(source, Mapping):
        settings = deepcopy(dict(source))
    else:
        settings = json.loads(Path(source).read_text(encoding="utf-8"))
    validate_settings(settings)
    return settings


def validate_settings(settings: Mapping[str, Any]) -> None:
    _require_keys(
        settings,
        {
            "schema_version", "revision", "status", "scope", "matrix", "load",
            "workloads", "conditions", "calibration", "source_bindings", "evidence_contract",
        },
        "experiment config",
    )
    if settings.get("schema_version") != 1:
        raise ValueError("experiment config schema_version must be 1")
    _text(settings.get("revision"), "revision")
    if settings.get("status") != "authored_pilot_only_not_executed":
        raise ValueError("config status must preserve the authored pilot-only limitation")
    if settings.get("scope") != "g3a_pending_inputs_not_physical_evidence":
        raise ValueError("config scope must identify G3a pending inputs as non-physical evidence")

    matrix = _mapping(settings.get("matrix"), "matrix")
    _require_keys(matrix, {"conditions", "policies", "seeds", "expected_trials"}, "matrix")
    if tuple(_sequence(matrix.get("conditions"), "matrix.conditions")) != CONDITIONS:
        raise ValueError("matrix.conditions must be normal, closure, burst")
    if tuple(_sequence(matrix.get("policies"), "matrix.policies")) != POLICIES:
        raise ValueError("matrix.policies must be fifo, eta")
    if list(_sequence(matrix.get("seeds"), "matrix.seeds")) != list(range(10)):
        raise ValueError("matrix.seeds must be exactly 0..9")
    if matrix.get("expected_trials") != 60:
        raise ValueError("matrix.expected_trials must be 60")

    load = _mapping(settings.get("load"), "load")
    _require_keys(
        load,
        {
            "initial_robots", "service_time_s", "workers", "ci_capacity", "co_capacity",
            "nominal_speed_m_s", "dwell_s",
        },
        "load",
    )
    robots = _mapping(load.get("initial_robots"), "load.initial_robots")
    if robots != {"carter1": "P1", "carter2": "P2"}:
        raise ValueError("initial robots must bind carter1/P1 and carter2/P2")
    if _integer(load.get("workers"), "load.workers", minimum=1) != 1:
        raise ValueError("G3 contract supports one worker")
    for name in ("ci_capacity", "co_capacity"):
        if _integer(load.get(name), f"load.{name}", minimum=1) != 2:
            raise ValueError(f"load.{name} must be 2")
    for name in ("service_time_s", "nominal_speed_m_s", "dwell_s"):
        _number(load.get(name), f"load.{name}", positive=True)

    workloads = _mapping(settings.get("workloads"), "workloads")
    _require_keys(workloads, {"qualification", "pilot"}, "workloads")
    qualification = _mapping(workloads.get("qualification"), "workloads.qualification")
    _require_keys(qualification, {"order_count", "sources", "horizon_s"}, "qualification")
    if _integer(qualification.get("order_count"), "qualification.order_count", minimum=1) != 2:
        raise ValueError("qualification must contain two orders")
    if list(_sequence(qualification.get("sources"), "qualification.sources")) != ["S1", "S3"]:
        raise ValueError("qualification sources must be S1 and S3")
    _number(qualification.get("horizon_s"), "qualification.horizon_s", positive=True)

    pilot = _mapping(workloads.get("pilot"), "workloads.pilot")
    _require_keys(
        pilot,
        {
            "order_count_min", "order_count_max", "sources", "horizon_s",
            "normal_interval_s", "release_jitter_s", "burst_window_s", "first_release_s",
        },
        "pilot",
    )
    minimum = _integer(pilot.get("order_count_min"), "pilot.order_count_min", minimum=1)
    maximum = _integer(pilot.get("order_count_max"), "pilot.order_count_max", minimum=minimum)
    if (minimum, maximum) != (4, 6):
        raise ValueError("pilot order count range must remain 4..6")
    sources = list(_sequence(pilot.get("sources"), "pilot.sources"))
    if sources != ["S1", "S2", "S3"]:
        raise ValueError("pilot sources must be S1, S2, S3")
    for name in (
        "horizon_s", "normal_interval_s", "release_jitter_s", "burst_window_s", "first_release_s"
    ):
        _number(pilot.get(name), f"pilot.{name}", positive=True)
    if float(pilot["burst_window_s"]) >= float(pilot["normal_interval_s"]):
        raise ValueError("burst window must be shorter than the normal release interval")

    conditions = _mapping(settings.get("conditions"), "conditions")
    if set(conditions) != set(CONDITIONS):
        raise ValueError("conditions must contain exactly normal, closure and burst")
    closure = _mapping(conditions.get("closure"), "conditions.closure")
    _require_keys(
        closure,
        {
            "model", "status", "route_edge_id", "route_graph_effect", "planner_visibility",
            "scheduled_end_exposed", "barrier_provenance",
        },
        "conditions.closure",
    )
    if (closure.get("model") != "physical_barrier_plus_route_disable"
            or closure.get("status") != "planned_not_executed"
            or closure.get("planner_visibility") != "current_state_only"
            or closure.get("scheduled_end_exposed") is not False
            or closure.get("route_graph_effect") != "disable_C_LOW-E_LOW"):
        raise ValueError("closure must be a planned current-state-only barrier and route disable")
    _text(closure.get("route_edge_id"), "conditions.closure.route_edge_id")
    barrier = _mapping(closure.get("barrier_provenance"), "conditions.closure.barrier_provenance")
    _require_keys(
        barrier,
        {"source", "layout_object_id", "rect_xyxy_m", "height_m", "scene_active_interval_s"},
        "conditions.closure.barrier_provenance",
    )
    if (barrier.get("source") != "configs/factory-v3/layout.json"
            or barrier.get("layout_object_id") != "blocked_aisle"
            or barrier.get("rect_xyxy_m") != [25.5, 2.5, 26.5, 5.5]
            or _number(barrier.get("height_m"), "barrier.height_m", positive=True) != 1.5
            or barrier.get("scene_active_interval_s") != [0.0, 600.0]):
        raise ValueError("closure barrier provenance must match the authored blocked_aisle object")
    for name in ("normal", "burst"):
        item = _mapping(conditions.get(name), f"conditions.{name}")
        _require_keys(item, {"model", "status"}, f"conditions.{name}")
        _text(item.get("model"), f"conditions.{name}.model")
        if item.get("status") != "planned_not_executed":
            raise ValueError(f"conditions.{name} must remain planned_not_executed")

    calibration = _mapping(settings.get("calibration"), "calibration")
    _require_keys(
        calibration,
        {"status", "required_before_full_matrix", "required_backend", "artifact"},
        "calibration",
    )
    if (calibration.get("status") != "required_not_supplied"
            or calibration.get("required_before_full_matrix") is not True
            or calibration.get("required_backend") != PHYSICAL_BACKEND
            or calibration.get("artifact") is not None):
        raise ValueError("current config must identify the missing physical calibration artifact")

    bindings = _sequence(settings.get("source_bindings"), "source_bindings")
    if not bindings:
        raise ValueError("source_bindings cannot be empty")
    seen_paths: set[str] = set()
    for index, raw in enumerate(bindings):
        binding = _mapping(raw, f"source_bindings[{index}]")
        _require_keys(binding, {"path", "sha256"}, f"source_bindings[{index}]")
        path = _text(binding.get("path"), f"source_bindings[{index}].path")
        digest = _text(binding.get("sha256"), f"source_bindings[{index}].sha256")
        if path in seen_paths or not _SHA256.fullmatch(digest):
            raise ValueError("source bindings require unique paths and lowercase SHA-256 values")
        seen_paths.add(path)

    evidence = _mapping(settings.get("evidence_contract"), "evidence_contract")
    _require_keys(
        evidence,
        {"backend", "requires_trial_input_binding", "requires_source_binding", "cpu_smoke_is_evidence"},
        "evidence_contract",
    )
    if (evidence.get("backend") != PHYSICAL_BACKEND
            or evidence.get("requires_trial_input_binding") is not True
            or evidence.get("requires_source_binding") is not True
            or evidence.get("cpu_smoke_is_evidence") is not False):
        raise ValueError("physical evidence contract cannot be weakened")
    _validate_json(settings)


def assert_execution_ready(settings: Mapping[str, Any]) -> None:
    """Reject the authored pilot plan until a physical calibration artifact is bound."""
    validate_settings(settings)
    raise ValueError(
        "full matrix execution is gated: bind an actual Isaac Sim/ROS 2/Nav2 calibration "
        "artifact with source, config and load hashes before creating an execution config"
    )


def _rng(settings: Mapping[str, Any], seed: int) -> random.Random:
    material = f"{settings['revision']}|{seed}".encode("utf-8")
    return random.Random(int.from_bytes(hashlib.sha256(material).digest()[:8], "big"))


def _condition_data(settings: Mapping[str, Any], condition: str) -> dict[str, Any]:
    _mapping(settings["conditions"][condition], f"conditions.{condition}")
    # Barrier geometry and scheduling belong to the scene/exogenous producer. Routing and
    # dispatch receive only the current state, so an ETA policy cannot use its authored end.
    return {"current_closed": condition == "closure"}


def build_workload(
    seed: int,
    condition: str,
    settings: Mapping[str, Any],
    *,
    profile: str = "pilot",
) -> dict[str, Any]:
    validate_settings(settings)
    if condition not in CONDITIONS:
        raise ValueError("unknown condition")
    if profile not in {"qualification", "pilot", "primary"}:
        raise ValueError("profile must be qualification, pilot or primary")
    if profile == "pilot":
        if seed != PILOT_SEED:
            raise ValueError("pilot workload seed must be 100")
    elif seed not in settings["matrix"]["seeds"]:
        raise ValueError("qualification/primary seed is outside the authored 0..9 matrix")

    rng = _rng(settings, seed)
    specification = settings["workloads"]["pilot" if profile == "primary" else profile]
    first_release = float(settings["workloads"]["pilot"]["first_release_s"])
    if profile == "qualification":
        count = 2
        sources = list(specification["sources"])
        ready_times = [first_release, first_release + float(settings["load"]["dwell_s"])]
    else:
        count = rng.randint(specification["order_count_min"], specification["order_count_max"])
        available_sources = list(specification["sources"])
        sources = [rng.choice(available_sources) for _ in range(count)]
        # Make the source composition seed-dependent while ensuring the authored three-station
        # pilot exercises more than one source in every trial.
        if len(set(sources)) == 1:
            sources[-1] = available_sources[(available_sources.index(sources[0]) + 1) % len(available_sources)]
        jitter = float(specification["release_jitter_s"])
        interval = float(specification["normal_interval_s"])
        if condition == "burst":
            window = float(specification["burst_window_s"])
            ready_times = sorted(first_release + rng.uniform(0.0, window) for _ in range(count))
        else:
            ready_times = [first_release]
            for _ in range(1, count):
                ready_times.append(ready_times[-1] + interval + rng.uniform(-jitter, jitter))

    horizon = float(specification["horizon_s"])
    if any(stamp < 0 or stamp >= horizon for stamp in ready_times):
        raise ValueError("authored release schedule must remain inside the horizon")
    prefix = {"qualification": "Q", "pilot": "C", "primary": "P"}[profile]
    orders = [
        {
            "order_id": f"G3-{prefix}-{seed:02d}-{index + 1:02d}",
            "payload_id": f"PAY-{prefix}-{seed:02d}-{index + 1:02d}",
            "source_station": source,
            "ready_time": round(ready_times[index], 9),
        }
        for index, source in enumerate(sources)
    ]
    workload = {
        "schema_version": 1,
        "profile": profile,
        "seed": seed,
        "condition": condition,
        "orders": orders,
        "initial_robots": deepcopy(settings["load"]["initial_robots"]),
        "horizon_s": horizon,
        "load": {
            "service_time_s": float(settings["load"]["service_time_s"]),
            "workers": settings["load"]["workers"],
            "ci_capacity": settings["load"]["ci_capacity"],
            "co_capacity": settings["load"]["co_capacity"],
            "nominal_speed_m_s": float(settings["load"]["nominal_speed_m_s"]),
            "dwell_s": float(settings["load"]["dwell_s"]),
        },
        "condition_data": _condition_data(settings, condition),
        "dispatch_visibility": {
            "rule": "release_feed_current_time_only",
            "future_ready_times_exposed": False,
            "ground_truth_exposed": False,
        },
        "provenance": {
            "kind": {
                "qualification": "authored_qualification_input",
                "pilot": "authored_calibration_pilot_input",
                "primary": "authored_primary_input",
            }[profile],
            "revision": settings["revision"],
            "source_bindings": deepcopy(settings["source_bindings"]),
            "execution_status": "planned_not_executed",
        },
    }
    validate_workload(workload, settings)
    return workload


def released_orders(workload: Mapping[str, Any], sim_time_s: float) -> list[dict[str, Any]]:
    now = _number(sim_time_s, "sim_time_s", nonnegative=True)
    orders = _sequence(workload.get("orders"), "workload.orders")
    return [deepcopy(dict(order)) for order in orders if float(order["ready_time"]) <= now]


def validate_workload(workload: Mapping[str, Any], settings: Mapping[str, Any]) -> None:
    validate_settings(settings)
    _require_keys(
        workload,
        {
            "schema_version", "profile", "seed", "condition", "orders", "initial_robots",
            "horizon_s", "load", "condition_data", "dispatch_visibility", "provenance",
        },
        "workload",
    )
    if workload.get("schema_version") != 1:
        raise ValueError("workload schema_version must be 1")
    profile = workload.get("profile")
    if profile not in {"qualification", "pilot", "primary"}:
        raise ValueError("invalid workload profile")
    seed = _integer(workload.get("seed"), "workload.seed")
    if profile == "pilot":
        if seed != PILOT_SEED:
            raise ValueError("pilot workload seed must be 100")
    elif seed not in settings["matrix"]["seeds"]:
        raise ValueError("qualification/primary workload seed is foreign to the matrix")
    condition = workload.get("condition")
    if condition not in CONDITIONS:
        raise ValueError("workload condition is foreign to the matrix")
    if "policy" in workload:
        raise ValueError("policy cannot be embedded in a workload")
    horizon = _number(workload.get("horizon_s"), "workload.horizon_s", positive=True)
    if workload.get("initial_robots") != settings["load"]["initial_robots"]:
        raise ValueError("initial robot poses do not match the experiment config")
    if workload.get("load") != {
        "service_time_s": float(settings["load"]["service_time_s"]),
        "workers": settings["load"]["workers"],
        "ci_capacity": settings["load"]["ci_capacity"],
        "co_capacity": settings["load"]["co_capacity"],
        "nominal_speed_m_s": float(settings["load"]["nominal_speed_m_s"]),
        "dwell_s": float(settings["load"]["dwell_s"]),
    }:
        raise ValueError("workload load settings do not match the experiment config")
    if workload.get("condition_data") != _condition_data(settings, str(condition)):
        raise ValueError("condition data does not match the experiment config")
    visibility = _mapping(workload.get("dispatch_visibility"), "dispatch_visibility")
    if visibility != {
        "rule": "release_feed_current_time_only",
        "future_ready_times_exposed": False,
        "ground_truth_exposed": False,
    }:
        raise ValueError("dispatch visibility cannot expose future inputs or ground truth")
    provenance = _mapping(workload.get("provenance"), "workload.provenance")
    if provenance != {
        "kind": {
            "qualification": "authored_qualification_input",
            "pilot": "authored_calibration_pilot_input",
            "primary": "authored_primary_input",
        }[profile],
        "revision": settings["revision"],
        "source_bindings": settings["source_bindings"],
        "execution_status": "planned_not_executed",
    }:
        raise ValueError("workload provenance does not bind the authored sources")

    orders = _sequence(workload.get("orders"), "workload.orders")
    specification_profile = "pilot" if profile == "primary" else profile
    expected = 2 if profile == "qualification" else None
    if expected is not None and len(orders) != expected:
        raise ValueError("qualification workload must contain exactly two orders")
    if profile in {"pilot", "primary"} and not 4 <= len(orders) <= 6:
        raise ValueError("pilot/primary workload must contain four to six orders")
    order_ids: set[str] = set()
    payload_ids: set[str] = set()
    allowed_sources = set(settings["workloads"][specification_profile]["sources"])
    for index, raw in enumerate(orders):
        order = _mapping(raw, f"orders[{index}]")
        _require_keys(order, {"order_id", "payload_id", "source_station", "ready_time"}, f"orders[{index}]")
        order_id = _text(order.get("order_id"), f"orders[{index}].order_id")
        payload_id = _text(order.get("payload_id"), f"orders[{index}].payload_id")
        source = _text(order.get("source_station"), f"orders[{index}].source_station")
        ready = _number(order.get("ready_time"), f"orders[{index}].ready_time", nonnegative=True)
        if order_id in order_ids or payload_id in payload_ids:
            raise ValueError("order_id and payload_id must be unique")
        if source not in allowed_sources or ready >= horizon:
            raise ValueError("order source or ready time is outside the workload contract")
        order_ids.add(order_id)
        payload_ids.add(payload_id)
    _validate_json(workload)


def _trial_without_hash(trial: Mapping[str, Any]) -> dict[str, Any]:
    return {key: deepcopy(value) for key, value in trial.items() if key != "trial_input_sha256"}


def build_matrix(settings: Mapping[str, Any]) -> list[dict[str, Any]]:
    validate_settings(settings)
    config_hash = canonical_hash(settings)
    matrix: list[dict[str, Any]] = []
    for condition in CONDITIONS:
        for seed in settings["matrix"]["seeds"]:
            workload = build_workload(seed, condition, settings, profile="primary")
            workload_hash = canonical_hash(workload)
            # Alternate which policy runs first within each paired seed to reduce a systematic
            # warm-cache/order effect without changing either member's workload.
            paired_order = ("eta", "fifo") if seed % 2 == 0 else ("fifo", "eta")
            for policy in paired_order:
                trial = {
                    "schema_version": 1,
                    "trial_id": f"g3-{condition}-{policy}-{seed:02d}",
                    "condition": condition,
                    "policy": policy,
                    "seed": seed,
                    "workload": deepcopy(workload),
                    "workload_sha256": workload_hash,
                    "source_config_sha256": config_hash,
                    "execution_gate": "calibration_required_not_supplied",
                    "status": "pending_not_executed",
                }
                trial["trial_input_sha256"] = canonical_hash(_trial_without_hash(trial))
                matrix.append(trial)
    if len(matrix) != settings["matrix"]["expected_trials"]:
        raise AssertionError("matrix cardinality contract is inconsistent")
    return matrix


def plan_manifest(settings: Mapping[str, Any]) -> dict[str, Any]:
    trials = build_matrix(settings)
    return {
        "schema_version": 1,
        "classification": "g3_authored_pending_manifest_not_physical_evidence",
        "status": "pending_not_executed",
        "source_config_sha256": canonical_hash(settings),
        "trial_count": len(trials),
        "calibration_gate": "required_not_supplied",
        "trials": trials,
    }


def verify_plan_manifest(manifest: Mapping[str, Any], settings: Mapping[str, Any]) -> None:
    expected = plan_manifest(settings)
    if manifest != expected:
        raise ValueError("pending manifest does not exactly match the canonical authored matrix")


def _result_key(result: Mapping[str, Any]) -> tuple[str, str, int]:
    condition = result.get("condition")
    policy = result.get("policy")
    seed = result.get("seed")
    if condition not in CONDITIONS or policy not in POLICIES or type(seed) is not int:
        raise ValueError("result has a foreign condition, policy or seed")
    return str(condition), str(policy), seed


def validate_trial_result(
    result: Mapping[str, Any],
    trial: Mapping[str, Any],
    settings: Mapping[str, Any],
    *,
    allow_explicit_non_evidence: bool = False,
) -> None:
    validate_settings(settings)
    _require_keys(
        result,
        {
            "schema_version", "trial_id", "condition", "policy", "seed", "trial_outcome",
            "failure_kind", "validation_status", "metrics_state", "backend", "source", "input",
            "evidence", "metrics", "failure_reason",
        },
        "trial result",
    )
    if result.get("schema_version") != 1:
        raise ValueError("trial result schema_version must be 1")
    if _result_key(result) != (trial["condition"], trial["policy"], trial["seed"]):
        raise ValueError("result identity does not match its matrix trial")
    if result.get("trial_id") != trial["trial_id"]:
        raise ValueError("result trial_id does not match")
    backend = result.get("backend")
    evidence = _mapping(result.get("evidence"), "result.evidence")
    if allow_explicit_non_evidence:
        if backend != FIXTURE_BACKEND or evidence.get("classification") != "explicit_non_evidence_test_fixture":
            raise ValueError("test aggregation accepts only explicit non-evidence fixtures")
    else:
        if backend != PHYSICAL_BACKEND or evidence.get("classification") != "g3_physical_trial":
            raise ValueError("only Isaac Sim/ROS 2/Nav2 G3 physical evidence is accepted")
    _require_keys(
        evidence,
        {"classification", "g3_evidence_id", "manifest_sha256", "artifact_sha256"},
        "result.evidence",
    )
    _text(evidence.get("g3_evidence_id"), "evidence.g3_evidence_id")
    for name in ("manifest_sha256", "artifact_sha256"):
        if not _SHA256.fullmatch(str(evidence.get(name, ""))):
            raise ValueError(f"evidence.{name} must be a lowercase SHA-256")

    source = _mapping(result.get("source"), "result.source")
    _require_keys(source, {"commit", "experiment_config_sha256"}, "result.source")
    if not _COMMIT.fullmatch(str(source.get("commit", ""))):
        raise ValueError("source.commit must be a full lowercase Git commit")
    if source.get("experiment_config_sha256") != trial["source_config_sha256"]:
        raise ValueError("result source config hash does not match the trial")
    inputs = _mapping(result.get("input"), "result.input")
    _require_keys(inputs, {"trial_input_sha256", "workload_sha256"}, "result.input")
    if (inputs.get("trial_input_sha256") != trial["trial_input_sha256"]
            or inputs.get("workload_sha256") != trial["workload_sha256"]):
        raise ValueError("result input hashes do not match the trial")

    outcome = result.get("trial_outcome")
    failure_kind = result.get("failure_kind")
    validation_status = result.get("validation_status")
    metrics_state = result.get("metrics_state")
    if outcome not in {"completed", "incomplete", "failed"}:
        raise ValueError("trial_outcome must be completed, incomplete or failed")
    if failure_kind not in {None, "operational", "collection"}:
        raise ValueError("failure_kind must be null, operational or collection")
    if validation_status not in {"validated", "invalid"}:
        raise ValueError("validation_status must be validated or invalid")
    if metrics_state not in {"known", "unknown"}:
        raise ValueError("metrics_state must be known or unknown")
    metrics = _mapping(result.get("metrics"), "result.metrics")
    _require_keys(
        metrics,
        {"orders_total", "orders_completed", "orders_completed_bounds", "horizon_s", "mean_completed_cycle_time_s", "makespan_s"},
        "result.metrics",
    )
    total = _integer(metrics.get("orders_total"), "metrics.orders_total", minimum=1)
    expected_total = len(trial["workload"]["orders"])
    bounds = _sequence(metrics.get("orders_completed_bounds"), "metrics.orders_completed_bounds")
    if len(bounds) != 2:
        raise ValueError("orders_completed_bounds must contain lower and upper bounds")
    lower = _integer(bounds[0], "metrics.orders_completed_bounds[0]")
    upper = _integer(bounds[1], "metrics.orders_completed_bounds[1]")
    if total != expected_total or lower > upper or upper > total:
        raise ValueError("result order counts do not match the trial workload")
    horizon = _number(metrics.get("horizon_s"), "metrics.horizon_s", positive=True)
    if horizon != float(trial["workload"]["horizon_s"]):
        raise ValueError("result horizon does not match the trial workload")
    completed_value = metrics.get("orders_completed")
    if metrics_state == "unknown":
        if not (outcome == "failed" and failure_kind == "collection"
                and validation_status == "invalid" and completed_value is None
                and [lower, upper] == [0, total]):
            raise ValueError("unknown metrics are reserved for invalid collection failures")
        if metrics.get("mean_completed_cycle_time_s") is not None or metrics.get("makespan_s") is not None:
            raise ValueError("unknown collection metrics cannot contain derived measurements")
        _text(result.get("failure_reason"), "failure_reason")
        _validate_json(result)
        return
    completed = _integer(completed_value, "metrics.orders_completed")
    if [lower, upper] != [completed, completed]:
        raise ValueError("known metrics bounds must equal the observed completed count")
    if validation_status != "validated" or failure_kind == "collection":
        raise ValueError("known metrics require validated non-collection evidence")
    if outcome in {"completed", "incomplete"} and failure_kind is not None:
        raise ValueError("completed/incomplete outcomes cannot declare a failure kind")
    if outcome == "failed" and failure_kind != "operational":
        raise ValueError("known failed outcomes must be operational failures")
    mean_cycle = metrics.get("mean_completed_cycle_time_s")
    if completed == 0:
        if mean_cycle is not None:
            raise ValueError("mean completed cycle time must be null with no completed orders")
    else:
        _number(mean_cycle, "metrics.mean_completed_cycle_time_s", nonnegative=True)
    makespan = metrics.get("makespan_s")
    if outcome == "completed":
        if completed != total:
            raise ValueError("completed result must complete every order")
        if _number(makespan, "metrics.makespan_s", positive=True) > horizon:
            raise ValueError("makespan cannot exceed the horizon")
        if result.get("failure_reason") is not None:
            raise ValueError("completed result cannot have a failure reason")
    else:
        if makespan is not None:
            raise ValueError("makespan is valid only when the entire workload completes")
        _text(result.get("failure_reason"), "failure_reason")
    _validate_json(result)


def summarize(
    results: Sequence[Mapping[str, Any]],
    settings: Mapping[str, Any],
    *,
    allow_explicit_non_evidence: bool = False,
    calibration_path: Path | None = None,
    pilot_slots: Mapping[str, Mapping[str, Any]] | None = None,
    resource_input: Mapping[str, Any] | None = None,
    expected_source_commit: str | None = None,
    experiment_config_path: Path | None = None,
    primary_result_slots: Mapping[str, Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    physical_inputs = (calibration_path, pilot_slots, resource_input,
                       expected_source_commit, experiment_config_path, primary_result_slots)
    if allow_explicit_non_evidence:
        if any(value is not None for value in physical_inputs):
            raise ValueError("explicit non-evidence aggregation cannot accept physical gate inputs")
    else:
        if any(value is None for value in physical_inputs):
            raise ValueError("physical aggregation requires raw-revalidated calibration artifact inputs")
        assert calibration_path is not None and pilot_slots is not None
        assert resource_input is not None and expected_source_commit is not None
        assert experiment_config_path is not None and primary_result_slots is not None
        if calibration_path.is_symlink() or not calibration_path.is_file():
            raise ValueError("physical calibration gate must be a regular file")
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
        if not isinstance(calibration, Mapping):
            raise ValueError("physical calibration gate must be an object")
        from .g3_calibration import validate_calibration
        validate_calibration(
            calibration, pilot_slots, resource_input,
            expected_source_commit=expected_source_commit,
            expected_config_sha256=canonical_hash(settings),
            experiment_config_path=experiment_config_path,
        )
        if (calibration.get("classification") != "g3_actual_six_pilot_calibration"
                or calibration.get("status") != "primary_execution_eligible"
                or calibration.get("execution_eligible") is not True
                or not _SHA256.fullmatch(str(calibration.get("calibration_sha256", "")))):
            raise ValueError("physical aggregation requires an eligible actual calibration")
    matrix = build_matrix(settings)
    trials = {(trial["condition"], trial["policy"], trial["seed"]): trial for trial in matrix}
    if len(results) != len(trials):
        raise ValueError("all 60 matrix results are required; failures and incomplete trials cannot be deleted")
    if not allow_explicit_non_evidence:
        assert primary_result_slots is not None and experiment_config_path is not None
        assert expected_source_commit is not None
        expected_trial_ids = {str(trial["trial_id"]) for trial in matrix}
        if set(primary_result_slots) != expected_trial_ids:
            raise ValueError("physical result slots must cover the exact canonical 60 trial IDs")
        supplied: dict[str, Mapping[str, Any]] = {}
        for result in results:
            trial_id = result.get("trial_id")
            if not isinstance(trial_id, str) or trial_id in supplied:
                raise ValueError("physical results require unique canonical trial IDs")
            supplied[trial_id] = result
        if set(supplied) != expected_trial_ids:
            raise ValueError("physical results do not cover the exact canonical 60 trial IDs")
        from .g3_supervisor import build_collection_failure_result
        from .g3_trial_result import build_trial_result
        for trial in matrix:
            trial_id = str(trial["trial_id"])
            slot = _mapping(primary_result_slots[trial_id], f"primary_result_slots.{trial_id}")
            kind = slot.get("kind")
            if kind == "validated_trial":
                _require_keys(
                    slot,
                    {"kind", "report_path", "mission_path", "artifact_root", "session_root",
                     "primary_inputs_path", "primary_inputs_sha256"},
                    f"primary_result_slots.{trial_id}",
                )
                rebuilt = build_trial_result(
                    trial, Path(str(slot["report_path"])), Path(str(slot["mission_path"])),
                    Path(str(slot["artifact_root"])), session_root=Path(str(slot["session_root"])),
                    source_commit=expected_source_commit,
                    experiment_config_path=experiment_config_path,
                    primary_inputs_path=Path(str(slot["primary_inputs_path"])),
                    primary_inputs_sha256=str(slot["primary_inputs_sha256"]),
                )
            elif kind == "collection_failure":
                _require_keys(
                    slot, {"kind", "receipt_path", "evidence_root"},
                    f"primary_result_slots.{trial_id}",
                )
                rebuilt = build_collection_failure_result(
                    trial, Path(str(slot["receipt_path"])), Path(str(slot["evidence_root"])),
                    source_commit=expected_source_commit,
                    experiment_config_path=experiment_config_path,
                )
            else:
                raise ValueError(f"primary result slot kind is invalid: {trial_id}")
            if supplied[trial_id] != rebuilt:
                raise ValueError(f"physical result differs from raw artifact replay: {trial_id}")
    indexed: dict[tuple[str, str, int], Mapping[str, Any]] = {}
    evidence_ids: set[str] = set()
    source_commits: set[str] = set()
    for result in results:
        key = _result_key(result)
        if key not in trials:
            raise ValueError("result is foreign to the configured matrix")
        if key in indexed:
            raise ValueError("duplicate matrix result")
        validate_trial_result(
            result, trials[key], settings, allow_explicit_non_evidence=allow_explicit_non_evidence
        )
        evidence_id = str(result["evidence"]["g3_evidence_id"])
        if evidence_id in evidence_ids:
            raise ValueError("G3 evidence IDs must be unique")
        evidence_ids.add(evidence_id)
        source_commits.add(str(result["source"]["commit"]))
        indexed[key] = result
    if set(indexed) != set(trials):
        raise ValueError("matrix result keys are incomplete or foreign")
    if len(source_commits) != 1:
        raise ValueError("all matrix results must bind one measurement source commit")
    measurement_source_commit = next(iter(source_commits))
    if not allow_explicit_non_evidence and measurement_source_commit != expected_source_commit:
        raise ValueError("matrix result source commit differs from the calibration source commit")

    groups: dict[str, Any] = {}
    for condition in CONDITIONS:
        groups[condition] = {}
        for policy in POLICIES:
            rows = [indexed[(condition, policy, seed)] for seed in range(10)]
            completed_trials = [row for row in rows if row["trial_outcome"] == "completed"]
            failed = sum(row["trial_outcome"] == "failed" for row in rows)
            incomplete = sum(row["trial_outcome"] == "incomplete" for row in rows)
            orders_total = sum(int(row["metrics"]["orders_total"]) for row in rows)
            known_rows = [row for row in rows if row["metrics_state"] == "known"]
            orders_completed = sum(int(row["metrics"]["orders_completed"]) for row in known_rows)
            lower_completed = sum(int(row["metrics"]["orders_completed_bounds"][0]) for row in rows)
            upper_completed = sum(int(row["metrics"]["orders_completed_bounds"][1]) for row in rows)
            cycles = [
                float(row["metrics"]["mean_completed_cycle_time_s"])
                for row in rows if row["metrics"]["mean_completed_cycle_time_s"] is not None
            ]
            makespans = [float(row["metrics"]["makespan_s"]) for row in completed_trials]
            groups[condition][policy] = {
                "trial_counts": {
                    "total": 10,
                    "full_workload_completed": len(completed_trials),
                    "incomplete_at_horizon": incomplete,
                    "failed": failed,
                },
                "horizon_order_counts": {
                    "total": orders_total,
                    "known_completed": orders_completed,
                    "completed_bounds": [lower_completed, upper_completed],
                    "unknown_trial_count": len(rows) - len(known_rows),
                },
                "completed_orders_only_mean_cycle_time_s": (
                    sum(cycles) / len(cycles) if cycles else None
                ),
                "completed_orders_only_label": "censoring_biased_not_a_full_workload_lead",
                "full_workload_makespan_mean_s": (
                    sum(makespans) / len(makespans) if makespans else None
                ),
                "makespan_scope": "full_workload_completed_trials_only",
            }

    paired: dict[str, Any] = {}
    for condition in CONDITIONS:
        pairs: list[dict[str, Any]] = []
        for seed in range(10):
            fifo = indexed[(condition, "fifo", seed)]
            eta = indexed[(condition, "eta", seed)]
            fifo_total = int(fifo["metrics"]["orders_total"])
            eta_total = int(eta["metrics"]["orders_total"])
            if fifo["input"]["workload_sha256"] != eta["input"]["workload_sha256"]:
                raise ValueError("paired policies do not share an identical workload hash")
            if fifo_total != eta_total:
                raise ValueError("paired policies do not share an identical workload size")
            pair = {
                "seed": seed,
                "workload_sha256": fifo["input"]["workload_sha256"],
                "primary_denominator_included": True,
                "fifo_outcome": fifo["trial_outcome"],
                "eta_outcome": eta["trial_outcome"],
                "failure_present": fifo["trial_outcome"] == "failed" or eta["trial_outcome"] == "failed",
                "incomplete_present": (
                    fifo["trial_outcome"] == "incomplete" or eta["trial_outcome"] == "incomplete"
                ),
                "eta_minus_fifo_completed_orders_by_horizon": None,
                "eta_minus_fifo_completed_orders_bounds": [
                    int(eta["metrics"]["orders_completed_bounds"][0]) - int(fifo["metrics"]["orders_completed_bounds"][1]),
                    int(eta["metrics"]["orders_completed_bounds"][1]) - int(fifo["metrics"]["orders_completed_bounds"][0]),
                ],
                "both_full_workloads_completed": (
                    fifo["trial_outcome"] == "completed" and eta["trial_outcome"] == "completed"
                ),
                "eta_minus_fifo_makespan_s": None,
                "eta_minus_fifo_completed_only_mean_cycle_time_s": None,
            }
            if fifo["metrics_state"] == "known" and eta["metrics_state"] == "known":
                pair["eta_minus_fifo_completed_orders_by_horizon"] = (
                    int(eta["metrics"]["orders_completed"]) - int(fifo["metrics"]["orders_completed"])
                )
            if pair["both_full_workloads_completed"]:
                pair["eta_minus_fifo_makespan_s"] = (
                    float(eta["metrics"]["makespan_s"])
                    - float(fifo["metrics"]["makespan_s"])
                )
                pair["eta_minus_fifo_completed_only_mean_cycle_time_s"] = (
                    float(eta["metrics"]["mean_completed_cycle_time_s"])
                    - float(fifo["metrics"]["mean_completed_cycle_time_s"])
                )
            pairs.append(pair)
        observed_deltas = [int(pair["eta_minus_fifo_completed_orders_by_horizon"])
                           for pair in pairs if pair["eta_minus_fifo_completed_orders_by_horizon"] is not None]
        mean_bounds = [sum(int(pair["eta_minus_fifo_completed_orders_bounds"][index]) for pair in pairs) / 10
                       for index in (0, 1)]
        makespan_deltas = [
            float(pair["eta_minus_fifo_makespan_s"])
            for pair in pairs if pair["eta_minus_fifo_makespan_s"] is not None
        ]
        paired[condition] = {
            "primary_metric": "eta_minus_fifo_completed_orders_by_horizon",
            "primary_pair_count": len(pairs),
            "primary_excluded_pair_count": 0,
            "primary_pairs_with_failure": sum(bool(pair["failure_present"]) for pair in pairs),
            "primary_pairs_with_incomplete": sum(bool(pair["incomplete_present"]) for pair in pairs),
            "primary_mean_delta": (sum(observed_deltas) / 10 if len(observed_deltas) == 10 else None),
            "primary_mean_delta_bounds": mean_bounds,
            "observed_only_pair_count": len(observed_deltas),
            "observed_only_mean_delta": (sum(observed_deltas) / len(observed_deltas) if observed_deltas else None),
            "observed_only_label": "known_pairs_only_selection_biased_when_below_ten",
            "restricted_full_completion_pair_count": len(makespan_deltas),
            "restricted_mean_eta_minus_fifo_makespan_s": (
                sum(makespan_deltas) / len(makespan_deltas) if makespan_deltas else None
            ),
            "restricted_scope": "same-seed pairs where both policies completed every order",
            "pairs": pairs,
        }

    return {
        "schema_version": 1,
        "classification": (
            "explicit_non_evidence_test_fixture"
            if allow_explicit_non_evidence else "g3_physical_experiment_summary"
        ),
        "source_config_sha256": canonical_hash(settings),
        "measurement_source_commit": measurement_source_commit,
        "trial_count": len(indexed),
        "denominator_policy": "all_planned_trials_including_failed_and_incomplete",
        "groups": groups,
        "paired_comparison": paired,
        "claim_limits": [
            "no real-robot safety claim",
            "no measured improvement claim without physical evidence",
            "no repeatability claim beyond this authored seed matrix",
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Materialize or verify the authored G3 pending matrix")
    parser.add_argument("config", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", type=Path)
    args = parser.parse_args(list(argv) if argv is not None else None)
    settings = load_experiment_config(args.config)
    if args.verify is not None:
        candidate = json.loads(args.verify.read_text(encoding="utf-8"))
        verify_plan_manifest(_mapping(candidate, "manifest"), settings)
        print(json.dumps({"passed": True, "trial_count": 60}, sort_keys=True))
        return 0
    manifest = plan_manifest(settings)
    rendered = json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    if args.output is None:
        sys.stdout.write(rendered)
    else:
        args.output.write_text(rendered, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
