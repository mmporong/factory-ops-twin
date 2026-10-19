"""Prepare and verify a bounded four-robot stationary sensor smoke input."""
from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import shutil
from typing import Any

from .fleet_registry import FleetRegistry
from .layout import Layout
from .usda import export_usda

QUALIFICATION = "fleet4_stationary_sensor_smoke"
SCENARIO = "four_robot_sensor_smoke"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-v4"
BUNDLE_FILES = frozenset({"config.json", "scenarios.json", "layout.json", "profile.json",
                          "registry.json", "render-profile.json", "visual-assets.json", "scene/factory-v3.usda"})


def _sha(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def compile_inputs(config_dir: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], Layout]:
    profile = json.loads((config_dir / "fleet4-development.json").read_text(encoding="utf-8"))
    config = json.loads((config_dir / "g2.json").read_text(encoding="utf-8"))
    layout = Layout.from_file(config_dir / "layout.json")
    for key in ("observation_window_s", "maximum_stationary_linear_m_s", "maximum_stationary_angular_rad_s",
                "max_sim_time_s", "max_wall_time_s"):
        if type(profile.get(key)) not in (int, float) or not math.isfinite(profile[key]) or profile[key] <= 0:
            raise ValueError(f"invalid stationary smoke limit: {key}")
    for key in ("minimum_control_samples_per_robot", "minimum_ros_sensor_messages"):
        if type(profile.get(key)) is not int or profile[key] <= 0:
            raise ValueError(f"invalid stationary smoke count: {key}")
    if profile.get("schema_version") != 1 or not isinstance(profile.get("robots"), list) or len(profile["robots"]) != 4:
        raise ValueError("development profile must declare four robots")
    if profile["observation_window_s"] >= profile["max_sim_time_s"]:
        raise ValueError("observation window must fit within the stationary simulation bound")
    anchors = {**layout.stations, **layout.pockets}
    spawns = {}
    definitions = []
    for robot in profile["robots"]:
        anchor = robot["spawn_anchor"]
        if anchor not in anchors or not layout.is_planning_free(anchors[anchor]):
            raise ValueError(f"stationary start is not a free authored anchor: {anchor}")
        spawns[robot["robot_id"]] = [*anchors[anchor], robot["yaw_rad"]]
        definitions.append({key: robot[key] for key in ("robot_id", "namespace", "prim_path")})
    config.update(qualification=QUALIFICATION, robots=definitions,
                  max_sim_time_s=profile["max_sim_time_s"], max_wall_time_s=profile["max_wall_time_s"],
                  camera_capture_every_steps=1_000_000)
    scenario = {"spawns": spawns, "max_sim_time_s": profile["max_sim_time_s"],
                "scope": profile["scope"]}
    registry = FleetRegistry(config, scenario)
    if registry.robot_ids[:2] != ("carter1", "carter2"):
        raise ValueError("shared evidence camera roles require carter1 and carter2 first")
    return config, scenario, profile, layout


def prepare_bundle(output: Path, config_dir: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    config_dir = config_dir.resolve(strict=True)
    config, scenario, profile, layout = compile_inputs(config_dir)
    output = output.resolve()
    if output.exists() or output.is_symlink():
        raise ValueError("stationary smoke input must use a new bundle directory")
    output.mkdir(parents=True)
    for name in ("layout.json", "render-profile.json", "visual-assets.json"):
        shutil.copyfile(config_dir / name, output / name)
    _write(output / "config.json", config)
    _write(output / "scenarios.json", {"scenarios": {SCENARIO: scenario}})
    _write(output / "profile.json", profile)
    _write(output / "registry.json", FleetRegistry(config, scenario).manifest())
    # Preserve the shared visual worker's historical scene filename; layout remains v4.
    export_usda(layout, output / "scene/factory-v3.usda")
    manifest = {"schema_version": 1, "qualification": QUALIFICATION,
                "scope": "prepared inputs only; no Isaac execution or navigation qualification",
                "source_files": {name: _sha(config_dir / name) for name in
                                 ("fleet4-development.json", "g2.json", "layout.json", "render-profile.json", "visual-assets.json")},
                "files": {name: _sha(output / name) for name in sorted(BUNDLE_FILES)}}
    _write(output / "bundle.json", manifest)
    return verify_bundle(output)


def verify_bundle(root: Path) -> dict[str, Any]:
    root = root.resolve(strict=True)
    manifest = json.loads((root / "bundle.json").read_text(encoding="utf-8"))
    if manifest.get("qualification") != QUALIFICATION or set(manifest.get("files", {})) != BUNDLE_FILES:
        raise ValueError("stationary smoke bundle manifest differs")
    for name, digest in manifest["files"].items():
        path = root / name
        if path.is_symlink() or not path.is_file() or _sha(path) != digest:
            raise ValueError(f"stationary smoke input changed: {name}")
    config = json.loads((root / "config.json").read_text(encoding="utf-8"))
    scenario = json.loads((root / "scenarios.json").read_text(encoding="utf-8"))["scenarios"][SCENARIO]
    registry = FleetRegistry(config, scenario)
    if len(registry.robot_ids) != 4 or len(registry.sensor_keys) != 8 or config.get("qualification") != QUALIFICATION:
        raise ValueError("stationary smoke requires four robots and eight lidars")
    declared = json.loads((root / "registry.json").read_text(encoding="utf-8"))
    if declared != registry.manifest():
        raise ValueError("stationary smoke registry differs from worker inputs")
    return {"passed": True, "scope": manifest["scope"], "robot_ids": list(registry.robot_ids),
            "lidar_count": len(registry.sensor_keys), "bundle_sha256": _sha(root / "bundle.json")}


def validate_result(output: Path, root: Path) -> dict[str, Any]:
    verify_bundle(root)
    config = json.loads((root / "config.json").read_text(encoding="utf-8"))
    scenario = json.loads((root / "scenarios.json").read_text(encoding="utf-8"))["scenarios"][SCENARIO]
    profile = json.loads((root / "profile.json").read_text(encoding="utf-8"))
    registry = FleetRegistry(config, scenario)
    result = json.loads((output / "fleet4-scene.json").read_text(encoding="utf-8"))
    errors = []
    run_id = result.get("run_id")
    if not isinstance(run_id, str) or re.fullmatch(r"[0-9a-f]{32}", run_id) is None or result.get("scenario_id") != SCENARIO:
        errors.append("observed run/scenario identity is invalid")
    mission = result.get("mission_control", {})
    if (mission.get("run_id") != run_id or mission.get("scenario_id") != SCENARIO or mission.get("status") != "passed"
            or set(mission.get("control_samples", {})) != set(registry.robot_ids)
            or any(type(n) is not int or n < profile["minimum_control_samples_per_robot"]
                   for n in mission.get("control_samples", {}).values())):
        errors.append("stationary completion is not bound to all four robots in this run")
    visual_inputs = result.get("scene_extension", {}).get("industrial_visuals", {}).get("configuration_inputs", {})
    for key, name in (("visual_assets", "visual-assets.json"), ("render_profile", "render-profile.json")):
        if visual_inputs.get(key, {}).get("sha256") != _sha(root / name):
            errors.append(f"executed visual configuration differs: {key}")
    for input_key, name in (("config", "config.json"), ("scenarios", "scenarios.json"),
                            ("layout", "layout.json"), ("scene", "scene/factory-v3.usda"),
                            ("fleet_bundle", "bundle.json"), ("fleet_profile", "profile.json")):
        if result.get("inputs", {}).get(input_key, {}).get("sha256") != _sha(root / name):
            errors.append(f"executed input digest differs: {input_key}")
    if result.get("status") != "passed" or result.get("qualification") != QUALIFICATION:
        errors.append("Isaac stationary worker did not pass")
    if set(result.get("robots", {})) != set(registry.robot_ids):
        errors.append("observed robot identities differ")
    if (len(result.get("lidars", [])) != len(registry.sensor_keys)
            or {(r.get("robot_id"), r.get("name")) for r in result.get("lidars", [])} != registry.sensor_keys):
        errors.append("observed lidar identities differ")
    if result.get("clock", {}).get("advanced") is not True:
        errors.append("simulation clock did not advance")
    for definition in registry.definitions:
        robot_id = definition["robot_id"]
        robot = result.get("robots", {}).get(robot_id, {})
        if (robot.get("namespace") != definition["namespace"] or robot.get("prim_path") != definition["prim_path"]
                or robot.get("footprint", {}).get("passed") is not True):
            errors.append(f"asset/namespace qualification missing: {robot_id}")
        ros = robot.get("ros", {})
        if any(type(ros.get(key)) is not int or ros[key] < profile["minimum_ros_sensor_messages"]
               for key in ("odom", "tf", "scan_front_raw", "scan_rear_raw")):
            errors.append(f"ROS sensor/odometry messages missing: {robot_id}")
    journal = output / "lidar-acquisition-events.jsonl"
    rows = [json.loads(line) for line in journal.read_text(encoding="utf-8").splitlines()] if journal.is_file() else []
    expected_prim = {(r["robot_id"], r["name"]): r["prim_path"] for r in registry.manifest()["lidars"]}
    if any(r.get("run_id") != run_id or r.get("scenario_id") != SCENARIO
           or r.get("prim_path") != expected_prim.get((r.get("robot_id"), r.get("side"))) for r in rows):
        errors.append("lidar journal contains a foreign run/scenario/prim")
    for robot_id, side in sorted(registry.sensor_keys):
        headers = [r["gmo"] for r in rows if r.get("robot_id") == robot_id and r.get("side") == side
                   and r.get("run_id") == run_id and r.get("scenario_id") == SCENARIO
                   and r.get("prim_path") == expected_prim[(robot_id, side)]
                   and r.get("status") == "gmo_header" and r.get("gmo", {}).get("num_elements", 0) > 0]
        if (len(headers) < 2 or headers[-1].get("timestamp_ns", -1) <= headers[0].get("timestamp_ns", -1)
                or headers[-1].get("frame_id", -1) <= headers[0].get("frame_id", -1)):
            errors.append(f"RTX sensor did not show advancing data: {robot_id}/{side}")
    return {"passed": not errors, "errors": errors, "robot_count": len(registry.robot_ids),
            "lidar_count": len(registry.sensor_keys),
            "scope": "stationary asset/sensor bootstrap only; not four-robot navigation, cargo, parking or charging"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    print(json.dumps(prepare_bundle(args.output, args.config), ensure_ascii=False))


if __name__ == "__main__":
    main()
