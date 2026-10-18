"""Build immutable inputs for G3b qualification/pilots; never a primary sweep."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import tempfile
from typing import Any

from .experiments import build_matrix, build_workload, canonical_hash, load_experiment_config
from .g3_prepare import _bindings
from .fleet_routing import RouteBook
from .usda import export_usda

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v3"
SCOPE = "qualification_and_pilot_only_not_primary_comparison"
PRIMARY_SCOPE = "primary_comparison_fixed_inputs"
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def source_hashes(config_dir: Path) -> dict[str, str]:
    paths = ("layout.json", "traffic.json", "g2.json", "g3-routing.json", "g3-experiment.json", "g3-runtime.json", "nav2-g2.yaml")
    return {name: sha(config_dir / name) for name in paths}


def _validate_runtime(runtime: dict[str, Any]) -> None:
    if runtime.get("qualification") != "g3_two_robot_orders" or runtime.get("scope") != SCOPE:
        raise ValueError("G3 runtime scope differs")
    for name in (
        "max_wall_time_s", "startup_sim_allowance_s", "chunk_timeout_s", "pose_ttl_s",
        "path_ttl_s", "path_clock_wait_wall_s", "terminal_recording_wait_wall_s",
        "reservation_tracking_margin_m", "station_xy_tolerance_m", "station_yaw_tolerance_rad",
        "stopped_window_s", "stopped_linear_m_s", "stopped_angular_rad_s", "odom_ttl_s",
    ):
        value = runtime[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"G3 runtime limit must be finite and positive: {name}")
    callback_drain = runtime.get("callback_drain")
    if not isinstance(callback_drain, dict) or set(callback_drain) != {"max_rounds", "wall_budget_s"}:
        raise ValueError("G3 callback drain fields differ")
    rounds = callback_drain["max_rounds"]
    if type(rounds) is not int or not 1 <= rounds <= 256:
        raise ValueError("G3 callback drain max_rounds must be an integer from 1 through 256")
    wall_budget = callback_drain["wall_budget_s"]
    if (isinstance(wall_budget, bool) or not isinstance(wall_budget, (int, float))
            or not math.isfinite(wall_budget) or not 0 < wall_budget <= .1):
        raise ValueError("G3 callback drain wall_budget_s must be finite and in (0, 0.1]")
    nav2 = runtime.get("g3_nav2")
    if (not isinstance(nav2, dict) or set(nav2) != {"follow_path_stateful"}
            or nav2["follow_path_stateful"] is not False):
        raise ValueError("G3 Nav2 override must be exactly FollowPath stateful false")
    if (runtime["station_xy_tolerance_m"] > .25
            or runtime["station_yaw_tolerance_rad"] > .25 or runtime["pose_ttl_s"] > .5):
        raise ValueError("G3 runtime cannot relax FleetMission pose limits")
    if type(runtime["camera_capture_every_steps"]) is not int or runtime["camera_capture_every_steps"] < 1:
        raise ValueError("G3 capture cadence must be a positive step count")
    media = runtime.get("real_time_media")
    media_fields = {
        "profiles", "window_s", "sample_every_physics_steps",
        "minimum_abs_linear_m_s", "minimum_abs_angular_rad_s",
    }
    if not isinstance(media, dict) or set(media) != media_fields:
        raise ValueError("G3 real-time media profile fields differ")
    if media["profiles"] != ["qualification"]:
        raise ValueError("G3 real-time media is qualification-only")
    window = media["window_s"]
    if type(window) not in (int, float) or not math.isfinite(window) or not 0 < window <= 30:
        raise ValueError("G3 real-time media window must be finite and at most 30 seconds")
    period = media["sample_every_physics_steps"]
    if type(period) is not int or not 1 <= period <= 60:
        raise ValueError("G3 real-time media cadence must be between 1 and 60 physics steps")
    for name in ("minimum_abs_linear_m_s", "minimum_abs_angular_rad_s"):
        value = media[name]
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 5:
            raise ValueError(f"G3 real-time media threshold is invalid: {name}")
    anchors = runtime["anchor_yaw_rad"]
    if (set(anchors) != {"S1", "S2", "S3", "CI", "CO", "OUT", "P1", "P2"}
            or any(type(value) not in (int, float) or not math.isfinite(value)
                   for value in anchors.values())):
        raise ValueError("G3 anchor yaw must cover every station and parking pocket")


def _write_session(output: Path, *, config_dir: Path = CONFIG, profile: str = "qualification",
                    policy: str = "fifo", condition: str = "normal", seed: int = 0) -> dict[str, Any]:
    if profile not in {"qualification", "pilot"} or policy not in {"fifo", "eta"}:
        raise ValueError("only qualification/pilot and fifo/eta are supported; primary sweep is gated")
    if output.exists() or output.is_symlink():
        raise ValueError("session output must be a new dedicated directory")
    config_dir = config_dir.resolve(strict=True)
    settings = load_experiment_config(config_dir / "g3-experiment.json")
    _bindings(settings, config_dir)
    workload = build_workload(seed, condition, settings, profile=profile)
    runtime = json.loads((config_dir / "g3-runtime.json").read_text(encoding="utf-8"))
    _validate_runtime(runtime)
    book = RouteBook.from_files(config_dir / "layout.json", config_dir / "g3-routing.json")
    preflight = book.preflight()
    if not preflight["passed"]:
        raise ValueError("route preflight failed")
    scenario_id = f"{profile.upper()}_{condition.upper()}"
    config = deepcopy(json.loads((config_dir / "g2.json").read_text(encoding="utf-8")))
    config.update({"qualification": "g3_two_robot_orders", "g3_runtime": runtime,
                   "max_sim_time_s": workload["horizon_s"] + runtime["startup_sim_allowance_s"],
                   "max_wall_time_s": runtime["max_wall_time_s"],
                   "camera_capture_every_steps": runtime["camera_capture_every_steps"]})
    config["scope"] = {**config["scope"], "qualification": SCOPE, "fault": "no injected G2 channel fault"}
    scenario = {"spawns": {robot: [*book.nodes[node], runtime["anchor_yaw_rad"][node]] for robot, node in workload["initial_robots"].items()},
                "max_sim_time_s": config["max_sim_time_s"], "condition": condition}
    output.mkdir(parents=True)
    save(output / "layout.json", book.layout.data)
    save(output / "config.json", config)
    save(output / "scenarios.json", {"scenarios": {scenario_id: scenario}})
    save(output / "route-preflight.json", preflight)
    for state in ("open", "closed"):
        save(output / "maps" / state / "manifest.json", book.layout.export_maps(output / "maps" / state, include_dynamic_barrier=state == "closed"))
    scene = export_usda(book.layout, output / "scene/factory-v3.usda")
    scene["path"] = "scene/factory-v3.usda"
    save(output / "scene/manifest.json", scene)
    files = {path.relative_to(output).as_posix(): sha(path) for path in sorted(output.rglob("*")) if path.is_file()}
    session = {"schema_version": 1, "scope": SCOPE, "primary_evaluation": False,
               "profile": profile, "policy": policy, "condition": condition, "seed": seed,
               "scenario_id": scenario_id, "workload": workload, "workload_sha256": canonical_hash(workload),
               "source_config": source_hashes(config_dir), "files": files,
               "scene_closure": deepcopy(settings["conditions"]["closure"]["barrier_provenance"])
                   if condition == "closure" else None}
    save(output / "session.json", session)
    return session


def prepare_session(output: Path, *, config_dir: Path = CONFIG, profile: str = "qualification",
                    policy: str = "fifo", condition: str = "normal", seed: int = 0) -> dict[str, Any]:
    _write_session(output, config_dir=config_dir, profile=profile, policy=policy, condition=condition, seed=seed)
    return verify_session(output, config_dir=config_dir)


def _load_regular_json(path: Path, name: str) -> tuple[bytes, dict[str, Any]]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} must be a regular file")
    raw = path.read_bytes()
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError(f"{name} must contain a JSON object")
    return raw, value


def _load_primary_input_manifest(path: Path, expected_sha256: str) -> dict[str, Any]:
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError("primary input manifest must be an absolute regular file")
    if not _SHA256.fullmatch(expected_sha256) or sha(path) != expected_sha256:
        raise ValueError("primary input manifest SHA differs from the supervisor pin")
    value = json.loads(path.read_text(encoding="utf-8"))
    expected = {"trial_path", "calibration_path", "pilot_slots", "resource_input",
                "expected_source_commit"}
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError("primary input manifest fields differ")
    for name in ("trial_path", "calibration_path"):
        raw_path = value[name]
        if not isinstance(raw_path, str) or not Path(raw_path).is_absolute():
            raise ValueError(f"primary input manifest {name} must be an absolute path")
    if not isinstance(value["pilot_slots"], dict) or not isinstance(value["resource_input"], dict):
        raise ValueError("primary input manifest evidence inputs must be objects")
    if not isinstance(value["expected_source_commit"], str):
        raise ValueError("primary input manifest source commit is invalid")
    return {
        "trial_path": Path(value["trial_path"]),
        "calibration_path": Path(value["calibration_path"]),
        "pilot_slots": value["pilot_slots"],
        "resource_input": value["resource_input"],
        "expected_source_commit": value["expected_source_commit"],
    }


def _sequential_input_bytes(resource_input: dict[str, Any]) -> dict[str, bytes]:
    v1 = {"evidence_root", "supervisor_path", "supervisor_sha256", "requested_primary_runs", "reserve_fraction"}
    if "schema_version" not in resource_input:
        return {}
    if set(resource_input) != v1 | {"schema_version", "sequential_plan_path", "sequential_plan_sha256"}:
        raise ValueError("sequential resource input fields differ")
    root = Path(str(resource_input["evidence_root"])).resolve(strict=True)

    def read(reference: dict[str, Any], name: str) -> bytes:
        if not isinstance(reference, dict) or set(reference) != {"path", "sha256"}:
            raise ValueError(f"{name} reference differs")
        relative = reference["path"]
        if not isinstance(relative, str) or "\\" in relative:
            raise ValueError(f"{name} path is unsafe")
        parsed = PurePosixPath(relative)
        if parsed.is_absolute() or any(part in {"", ".", ".."} for part in parsed.parts):
            raise ValueError(f"{name} path is unsafe")
        path = root.joinpath(*parsed.parts)
        try:
            path.resolve(strict=True).relative_to(root)
        except (OSError, ValueError) as exc:
            raise ValueError(f"{name} path escapes the evidence root") from exc
        if path.is_symlink() or not path.is_file() or not _SHA256.fullmatch(str(reference["sha256"])):
            raise ValueError(f"{name} must be a hash-bound regular file")
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != reference["sha256"]:
            raise ValueError(f"{name} bytes differ")
        return raw

    plan_ref = {"path": resource_input["sequential_plan_path"],
                "sha256": resource_input["sequential_plan_sha256"]}
    plan_raw = read(plan_ref, "sequential plan")
    plan = json.loads(plan_raw)
    output = {"sequential-capacity/plan.json": plan_raw}
    for index, receipt_ref in enumerate(plan.get("coordinator_receipts", [])):
        receipt_raw = read(receipt_ref, f"coordinator receipt {index}")
        output[f"sequential-capacity/coordinator-{index}.json"] = receipt_raw
        receipt = json.loads(receipt_raw)
        for run_index, run in enumerate(receipt.get("runs", [])):
            run_documents: dict[str, bytes] = {}
            for name in ("package_record", "preservation_receipt", "full_restore_receipt",
                         "semantic_replay_report", "source_support_closure", "raw_envelope"):
                raw = read(run[name], f"coordinator receipt {index} run {run_index} {name}")
                run_documents[name] = raw
                output[f"sequential-capacity/coordinator-{index}-run-{run_index}-{name}.json"] = raw
            package_record = json.loads(run_documents["package_record"])
            output[f"sequential-capacity/coordinator-{index}-run-{run_index}-package-manifest.json"] = read(
                package_record["package_manifest"],
                f"coordinator receipt {index} run {run_index} package manifest",
            )
    return output


def _primary_inputs_from_environment() -> dict[str, Any]:
    raw_path = os.environ.get("FACTORY_OPS_G3_PRIMARY_INPUTS_PATH")
    digest = os.environ.get("FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256")
    if raw_path is None or digest is None:
        raise ValueError("primary verification requires supervisor-pinned primary input manifest environment")
    return _load_primary_input_manifest(Path(raw_path), digest)


def _validate_primary_inputs(
    *, trial_path: Path, calibration_path: Path, pilot_slots: dict[str, dict[str, Any]],
    resource_input: dict[str, Any], expected_source_commit: str, config_dir: Path,
) -> tuple[bytes, dict[str, Any], bytes, dict[str, Any], bytes, dict[str, Any]]:
    if not _COMMIT.fullmatch(expected_source_commit):
        raise ValueError("fixed source commit must be a full lowercase Git commit")
    config_dir = config_dir.resolve(strict=True)
    experiment_path = config_dir / "g3-experiment.json"
    config_raw, settings = _load_regular_json(experiment_path, "base experiment config")
    settings = load_experiment_config(settings)
    _bindings(settings, config_dir)
    trial_raw, trial = _load_regular_json(trial_path, "canonical trial input")
    candidates = {row["trial_input_sha256"]: row for row in build_matrix(settings)}
    if trial.get("trial_input_sha256") not in candidates or trial != candidates[trial["trial_input_sha256"]]:
        raise ValueError("primary trial does not exactly match the canonical 60-run matrix")
    calibration_raw, calibration = _load_regular_json(
        calibration_path, "calibration gate"
    )
    from .g3_calibration import validate_calibration
    validate_calibration(
        calibration, pilot_slots, resource_input,
        expected_source_commit=expected_source_commit,
        expected_config_sha256=canonical_hash(settings),
        experiment_config_path=experiment_path,
    )
    if (calibration.get("classification") != "g3_actual_six_pilot_calibration"
            or calibration.get("execution_eligible") is not True
            or calibration.get("status") != "primary_execution_eligible"
            or not _SHA256.fullmatch(str(calibration.get("calibration_sha256", "")))):
        raise ValueError("only a revalidated eligible actual calibration can open primary execution")
    return trial_raw, trial, calibration_raw, calibration, config_raw, settings


def _write_primary_session(
    output: Path, *, trial_path: Path, calibration_path: Path,
    pilot_slots: dict[str, dict[str, Any]], resource_input: dict[str, Any],
    expected_source_commit: str, config_dir: Path = CONFIG,
) -> dict[str, Any]:
    if output.exists() or output.is_symlink():
        raise ValueError("session output must be a new dedicated directory")
    config_dir = config_dir.resolve(strict=True)
    trial_raw, trial, calibration_raw, calibration, config_raw, settings = _validate_primary_inputs(
        trial_path=trial_path, calibration_path=calibration_path, pilot_slots=pilot_slots,
        resource_input=resource_input, expected_source_commit=expected_source_commit,
        config_dir=config_dir,
    )
    workload = deepcopy(trial["workload"])
    condition, policy, seed = trial["condition"], trial["policy"], trial["seed"]
    runtime = json.loads((config_dir / "g3-runtime.json").read_text(encoding="utf-8"))
    _validate_runtime(runtime)
    book = RouteBook.from_files(config_dir / "layout.json", config_dir / "g3-routing.json")
    preflight = book.preflight()
    if not preflight["passed"]:
        raise ValueError("route preflight failed")
    scenario_id = f"PRIMARY_{condition.upper()}"
    config = deepcopy(json.loads((config_dir / "g2.json").read_text(encoding="utf-8")))
    config.update({
        "qualification": "g3_primary_comparison", "g3_runtime": runtime,
        "max_sim_time_s": workload["horizon_s"] + runtime["startup_sim_allowance_s"],
        "max_wall_time_s": runtime["max_wall_time_s"],
        "camera_capture_every_steps": runtime["camera_capture_every_steps"],
    })
    config["scope"] = {**config["scope"], "qualification": PRIMARY_SCOPE,
                       "fault": "no injected G2 channel fault"}
    scenario = {
        "spawns": {robot: [*book.nodes[node], runtime["anchor_yaw_rad"][node]]
                   for robot, node in workload["initial_robots"].items()},
        "max_sim_time_s": config["max_sim_time_s"], "condition": condition,
    }
    output.mkdir(parents=True)
    save(output / "layout.json", book.layout.data)
    save(output / "config.json", config)
    save(output / "scenarios.json", {"scenarios": {scenario_id: scenario}})
    save(output / "route-preflight.json", preflight)
    for state in ("open", "closed"):
        save(output / "maps" / state / "manifest.json", book.layout.export_maps(
            output / "maps" / state, include_dynamic_barrier=state == "closed"
        ))
    scene = export_usda(book.layout, output / "scene/factory-v3.usda")
    scene["path"] = "scene/factory-v3.usda"
    save(output / "scene/manifest.json", scene)
    (output / "trial-input.json").write_bytes(trial_raw)
    (output / "calibration-gate.json").write_bytes(calibration_raw)
    (output / "base-experiment-config.json").write_bytes(config_raw)
    (output / "fixed-source-commit.txt").write_bytes((expected_source_commit + "\n").encode("ascii"))
    sequential_inputs = _sequential_input_bytes(resource_input)
    for relative, raw in sequential_inputs.items():
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(raw)
    files = {path.relative_to(output).as_posix(): sha(path)
             for path in sorted(output.rglob("*")) if path.is_file()}
    session = {
        "schema_version": 1, "scope": PRIMARY_SCOPE, "primary_evaluation": True,
        "profile": "primary", "policy": policy, "condition": condition, "seed": seed,
        "scenario_id": scenario_id, "workload": workload,
        "workload_sha256": trial["workload_sha256"],
        "trial_input_sha256": trial["trial_input_sha256"],
        "fixed_source_commit": expected_source_commit,
        "experiment_config_sha256": canonical_hash(settings),
        "calibration_sha256": calibration["calibration_sha256"],
        "source_config": source_hashes(config_dir), "files": files,
        "scene_closure": deepcopy(settings["conditions"]["closure"]["barrier_provenance"])
            if condition == "closure" else None,
    }
    if sequential_inputs:
        session["sequential_capacity_sha256"] = resource_input["sequential_plan_sha256"]
    save(output / "session.json", session)
    return session


def prepare_primary_session(
    output: Path, *, trial_path: Path, calibration_path: Path,
    pilot_slots: dict[str, dict[str, Any]], resource_input: dict[str, Any],
    expected_source_commit: str, config_dir: Path = CONFIG,
) -> dict[str, Any]:
    _write_primary_session(
        output, trial_path=trial_path, calibration_path=calibration_path,
        pilot_slots=pilot_slots, resource_input=resource_input,
        expected_source_commit=expected_source_commit, config_dir=config_dir,
    )
    return verify_session(
        output, config_dir=config_dir, trial_path=trial_path, calibration_path=calibration_path,
        pilot_slots=pilot_slots, resource_input=resource_input,
        expected_source_commit=expected_source_commit,
    )


def verify_session(output: Path, *, config_dir: Path = CONFIG, expected_context: dict[str, Any] | None = None,
                   expected_sha256: str | None = None, trial_path: Path | None = None,
                   calibration_path: Path | None = None,
                   pilot_slots: dict[str, dict[str, Any]] | None = None,
                   resource_input: dict[str, Any] | None = None,
                   expected_source_commit: str | None = None) -> dict[str, Any]:
    if output.is_symlink() or any(path.is_symlink() for path in output.rglob("*")):
        raise ValueError("session must contain regular files")
    if expected_sha256 is not None and sha(output / "session.json") != expected_sha256:
        raise ValueError("session SHA differs from supervisor-bound input")
    session = json.loads((output / "session.json").read_text(encoding="utf-8"))
    if session.get("primary_evaluation") is True or session.get("scope") == PRIMARY_SCOPE:
        primary_values = (trial_path, calibration_path, pilot_slots, resource_input,
                          expected_source_commit)
        if all(value is None for value in primary_values):
            pinned = _primary_inputs_from_environment()
            trial_path, calibration_path = pinned["trial_path"], pinned["calibration_path"]
            pilot_slots, resource_input = pinned["pilot_slots"], pinned["resource_input"]
            expected_source_commit = pinned["expected_source_commit"]
        elif any(value is None for value in primary_values):
            raise ValueError("primary verification requires one complete external input set")
        assert trial_path is not None and calibration_path is not None
        assert pilot_slots is not None and resource_input is not None
        assert expected_source_commit is not None
        trial_raw, trial, calibration_raw, calibration, config_raw, settings = _validate_primary_inputs(
            trial_path=trial_path, calibration_path=calibration_path, pilot_slots=pilot_slots,
            resource_input=resource_input, expected_source_commit=expected_source_commit,
            config_dir=config_dir,
        )
        expected_identity = {"profile": "primary", "policy": trial["policy"],
                             "condition": trial["condition"], "seed": trial["seed"]}
        if expected_context is not None and expected_context != expected_identity:
            raise ValueError("expected run context differs from the canonical primary trial")
        if any(session.get(key) != value for key, value in expected_identity.items()):
            raise ValueError("primary session identity differs from canonical trial")
        if (session.get("schema_version") != 1 or session.get("scope") != PRIMARY_SCOPE
                or session.get("workload") != trial["workload"]
                or session.get("workload_sha256") != trial["workload_sha256"]
                or session.get("trial_input_sha256") != trial["trial_input_sha256"]
                or session.get("fixed_source_commit") != expected_source_commit
                or session.get("experiment_config_sha256") != canonical_hash(settings)
                or session.get("calibration_sha256") != calibration["calibration_sha256"]
                or session.get("sequential_capacity_sha256") != resource_input.get("sequential_plan_sha256")
                or session.get("source_config") != source_hashes(config_dir)):
            raise ValueError("primary session metadata differs from bound inputs")
        raw_inputs = {
            "trial-input.json": trial_raw,
            "calibration-gate.json": calibration_raw,
            "base-experiment-config.json": config_raw,
            "fixed-source-commit.txt": (expected_source_commit + "\n").encode("ascii"),
        }
        raw_inputs.update(_sequential_input_bytes(resource_input))
        if any((output / name).read_bytes() != raw for name, raw in raw_inputs.items()):
            raise ValueError("primary session did not preserve exact input bytes")
        expected_files = set(session.get("files", {}))
        actual_files = {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()}
        if actual_files != expected_files | {"session.json"}:
            raise ValueError("primary session exact file set differs")
        for name, digest in session["files"].items():
            if sha(output / name) != digest:
                raise ValueError(f"primary session artifact differs: {name}")
        with tempfile.TemporaryDirectory() as temporary:
            derived = Path(temporary) / "session"
            current = _write_primary_session(
                derived, trial_path=trial_path, calibration_path=calibration_path,
                pilot_slots=pilot_slots, resource_input=resource_input,
                expected_source_commit=expected_source_commit, config_dir=config_dir,
            )
            if session != current:
                raise ValueError("primary session metadata differs from current source derivation")
            for name in expected_files:
                if (output / name).read_bytes() != (derived / name).read_bytes():
                    raise ValueError(f"primary source-derived artifact differs: {name}")
        return session
    if expected_context is not None:
        if set(expected_context) != {"profile", "policy", "condition", "seed"}:
            raise ValueError("expected session context must bind profile, policy, condition and seed")
        if any(type(session.get(key)) is not type(value) or session.get(key) != value for key, value in expected_context.items()):
            raise ValueError("session identity differs from expected run context")
    if (type(session.get("schema_version")) is not int or session["schema_version"] != 1 or session.get("scope") != SCOPE
            or session.get("primary_evaluation") is not False
            or session.get("profile") not in {"qualification", "pilot"} or session.get("policy") not in {"fifo", "eta"}):
        raise ValueError("session qualification scope differs")
    settings = load_experiment_config(config_dir / "g3-experiment.json")
    _bindings(settings, config_dir)
    expected = build_workload(session["seed"], session["condition"], settings, profile=session["profile"])
    if session["workload"] != expected or session["workload_sha256"] != canonical_hash(expected):
        raise ValueError("session workload differs from authored sources")
    if session["source_config"] != source_hashes(config_dir):
        raise ValueError("session source config differs")
    expected_files = {"config.json", "scenarios.json", "layout.json", "route-preflight.json", "scene/factory-v3.usda", "scene/manifest.json",
                      *(f"maps/{state}/{name}" for state in ("open", "closed") for name in ("map.pgm", "keepout.pgm", "map.yaml", "keepout.yaml", "manifest.json"))}
    if set(session["files"]) != expected_files or {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()} != expected_files | {"session.json"}:
        raise ValueError("session exact file set differs")
    for name, expected_hash in session["files"].items():
        if sha(output / name) != expected_hash:
            raise ValueError(f"session artifact differs: {name}")
    with tempfile.TemporaryDirectory() as temporary:
        derived = Path(temporary) / "session"
        current = _write_session(derived, config_dir=config_dir, profile=session["profile"],
                                 policy=session["policy"], condition=session["condition"], seed=session["seed"])
        if session != current:
            raise ValueError("session metadata differs from current source derivation")
        for name in expected_files:
            if (output / name).read_bytes() != (derived / name).read_bytes():
                raise ValueError(f"session source-derived artifact differs: {name}")
    return session


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", choices=("qualification", "pilot", "primary"))
    parser.add_argument("--condition", choices=("normal", "closure", "burst"))
    parser.add_argument("--policy", choices=("fifo", "eta"))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--primary-inputs", type=Path)
    parser.add_argument("--primary-inputs-sha256")
    parser.add_argument("--config-dir", type=Path, default=CONFIG)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    context = {"profile": args.profile, "condition": args.condition, "policy": args.policy, "seed": args.seed}
    primary_values = (args.primary_inputs, args.primary_inputs_sha256)
    if args.profile == "primary" or any(value is not None for value in primary_values):
        if args.profile != "primary" or any(value is None for value in primary_values):
            parser.error("primary requires --profile primary, --primary-inputs and --primary-inputs-sha256")
        primary_args = _load_primary_input_manifest(args.primary_inputs, args.primary_inputs_sha256)
        primary_args["config_dir"] = args.config_dir
        session = (verify_session(args.output, **primary_args) if args.verify
                   else prepare_primary_session(args.output, **primary_args))
    elif args.verify:
        if any(value is not None for value in context.values()) and any(value is None for value in context.values()):
            parser.error("verification context requires profile, condition, policy and seed together")
        session = verify_session(args.output, config_dir=args.config_dir,
                                 expected_context=context if args.profile is not None else None)
    else:
        profile = args.profile or "qualification"
        default_seed = 100 if profile == "pilot" else 0
        session = prepare_session(args.output, config_dir=args.config_dir, profile=profile,
                                  condition=args.condition or "normal", policy=args.policy or "fifo",
                                  seed=args.seed if args.seed is not None else default_seed)
    print(json.dumps({"verified": True, "scenario_id": session["scenario_id"], "scope": session["scope"], "orders": len(session["workload"]["orders"]), "workload_sha256": session["workload_sha256"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
