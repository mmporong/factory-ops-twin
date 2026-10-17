"""Build immutable inputs for G3b qualification/pilots; never a primary sweep."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from typing import Any

from .experiments import build_workload, canonical_hash, load_experiment_config
from .g3_prepare import _bindings
from .fleet_routing import RouteBook
from .usda import export_usda

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v3"
SCOPE = "qualification_and_pilot_only_not_primary_comparison"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def source_hashes(config_dir: Path) -> dict[str, str]:
    paths = ("layout.json", "traffic.json", "g2.json", "g3-routing.json", "g3-experiment.json", "g3-runtime.json", "nav2-g2.yaml")
    return {name: sha(config_dir / name) for name in paths}


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
    if runtime.get("qualification") != "g3_two_robot_orders" or runtime.get("scope") != SCOPE:
        raise ValueError("G3 runtime scope differs")
    for name in ("max_wall_time_s", "startup_sim_allowance_s", "chunk_timeout_s", "pose_ttl_s", "path_ttl_s", "path_clock_wait_wall_s", "terminal_recording_wait_wall_s",
                 "reservation_tracking_margin_m", "station_xy_tolerance_m", "station_yaw_tolerance_rad",
                 "stopped_window_s", "stopped_linear_m_s", "stopped_angular_rad_s", "odom_ttl_s"):
        value = runtime[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"G3 runtime limit must be finite and positive: {name}")
    if runtime["station_xy_tolerance_m"] > .25 or runtime["station_yaw_tolerance_rad"] > .25 or runtime["pose_ttl_s"] > .5:
        raise ValueError("G3 runtime cannot relax FleetMission pose limits")
    if type(runtime["camera_capture_every_steps"]) is not int or runtime["camera_capture_every_steps"] < 1:
        raise ValueError("G3 capture cadence must be a positive step count")
    if set(runtime["anchor_yaw_rad"]) != {"S1", "S2", "S3", "CI", "CO", "OUT", "P1", "P2"} or any(type(v) not in (int, float) or not math.isfinite(v) for v in runtime["anchor_yaw_rad"].values()):
        raise ValueError("G3 anchor yaw must cover every station and parking pocket")
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


def verify_session(output: Path, *, config_dir: Path = CONFIG, expected_context: dict[str, Any] | None = None,
                   expected_sha256: str | None = None) -> dict[str, Any]:
    if output.is_symlink() or any(path.is_symlink() for path in output.rglob("*")):
        raise ValueError("session must contain regular files")
    if expected_sha256 is not None and sha(output / "session.json") != expected_sha256:
        raise ValueError("session SHA differs from supervisor-bound input")
    session = json.loads((output / "session.json").read_text(encoding="utf-8"))
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
    parser.add_argument("--profile", choices=("qualification", "pilot"))
    parser.add_argument("--condition", choices=("normal", "closure", "burst"))
    parser.add_argument("--policy", choices=("fifo", "eta"))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    context = {"profile": args.profile, "condition": args.condition, "policy": args.policy, "seed": args.seed}
    if args.verify:
        if any(value is not None for value in context.values()) and any(value is None for value in context.values()):
            parser.error("verification context requires profile, condition, policy and seed together")
        session = verify_session(args.output, expected_context=context if args.profile is not None else None)
    else:
        session = prepare_session(args.output, profile=args.profile or "qualification", condition=args.condition or "normal", policy=args.policy or "fifo", seed=args.seed if args.seed is not None else 0)
    print(json.dumps({"verified": True, "scenario_id": session["scenario_id"], "scope": session["scope"], "orders": len(session["workload"]["orders"]), "workload_sha256": session["workload_sha256"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
