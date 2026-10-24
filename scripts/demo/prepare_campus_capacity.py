"""Prepare fail-closed 1/4/8/16 native SDK capacity-probe inputs without starting Kit."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
from scripts.isaac.local_fleet_runtime_probe import ROBOT_COUNTS, RUNTIME_MODES, validate_config


MOTION_DISTANCE_M = .6
ROBOT_CLEARANCE_M = .7


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def point_has_clearance(layout: Mapping[str, Any], x: float, y: float) -> bool:
    inside = any(
        rect[0] + ROBOT_CLEARANCE_M <= x <= rect[2] - ROBOT_CLEARANCE_M
        and rect[1] + ROBOT_CLEARANCE_M <= y <= rect[3] - ROBOT_CLEARANCE_M
        for rect in (region["rect"] for region in layout["amr_allowed_regions"])
    )
    blocked = any(
        rect[0] - ROBOT_CLEARANCE_M <= x <= rect[2] + ROBOT_CLEARANCE_M
        and rect[1] - ROBOT_CLEARANCE_M <= y <= rect[3] + ROBOT_CLEARANCE_M
        for rect in (obstacle["rect"] for obstacle in layout["physical_obstacles"])
    )
    return inside and not blocked


def build_robot_definitions(
    g2: Mapping[str, Any], fleet: Mapping[str, Any], routing: Mapping[str, Any],
    layout: Mapping[str, Any], count: int,
) -> list[dict[str, Any]]:
    source_robots = g2.get("robots")
    stations = fleet.get("robot_stations")
    parking = routing.get("parking")
    nodes = routing.get("nodes")
    if not isinstance(source_robots, list) or len(source_robots) != 16:
        raise ValueError("frozen g2 robot identity list must contain 16 robots")
    if not isinstance(stations, list) or len(stations) != 16 or len(set(stations)) != 16:
        raise ValueError("frozen fleet profile must contain 16 unique homes")
    if not isinstance(parking, Mapping) or not isinstance(nodes, Mapping):
        raise ValueError("frozen routing parking or nodes are absent")
    result = []
    for index, identity in enumerate(source_robots[:count], start=1):
        station = stations[index - 1]
        if parking.get(f"amr-{index}") != station or station not in nodes:
            raise ValueError(f"frozen home mapping differs for robot {index}")
        x, y = (float(value) for value in nodes[station])
        yaw = math.pi / 2 if station.endswith(("P1", "P3")) else -math.pi / 2
        end = [x + MOTION_DISTANCE_M * math.cos(yaw), y + MOTION_DISTANCE_M * math.sin(yaw)]
        for step in range(13):
            ratio = step / 12
            px, py = x + (end[0] - x) * ratio, y + (end[1] - y) * ratio
            if not point_has_clearance(layout, px, py):
                raise ValueError(f"bounded wheel smoke leaves authored clearance at {station}")
        result.append({
            "robot_id": identity["robot_id"], "namespace": identity["namespace"],
            "prim_path": identity["prim_path"], "home_station": station,
            "spawn_xy_yaw": [x, y, yaw], "expected_motion_endpoint_xy": end,
        })
    return result


def verify_frozen_inputs(frozen_root: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    manifest_path = frozen_root / "manifest.json"
    manifest = load_json(manifest_path)
    rows = {row["path"]: row for row in manifest}
    required = ("config/fleet-profile.json", "config/g2.json", "config/g3-routing.json", "config/layout.json")
    loaded = {}
    for relative in required:
        path = frozen_root / relative
        row = rows.get(relative)
        if (not path.is_file() or not isinstance(row, Mapping) or path.stat().st_size != row.get("bytes")
                or sha256(path) != row.get("sha256")):
            raise ValueError(f"frozen input differs from manifest: {relative}")
        loaded[relative] = load_json(path)
    return loaded, rows


def prepare(frozen_root: Path, scene: Path, scene_sha256: str, output: Path, repo_root: Path) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError("output must be a new directory")
    if not scene.is_file() or sha256(scene) != scene_sha256:
        raise ValueError("scene bytes differ from the pinned SHA-256")
    scene_dependencies = {}
    for name in ("factory-v3.usda", "factory-v3.visual.usda"):
        dependency = scene.parent / name
        if not dependency.is_file():
            raise ValueError(f"scene dependency is absent: {name}")
        scene_dependencies[name] = sha256(dependency)
    try:
        scene_relative = scene.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError as error:
        raise ValueError("scene must be inside the repository") from error
    loaded, _ = verify_frozen_inputs(frozen_root)
    fleet = loaded["config/fleet-profile.json"]
    g2 = loaded["config/g2.json"]
    routing = loaded["config/g3-routing.json"]
    layout = loaded["config/layout.json"]
    if fleet.get("fleet_count") != 16 or layout.get("revision") != "amr-semiconductor-campus16-v2":
        raise ValueError("frozen campus fleet/layout contract differs")
    output.mkdir(parents=True)
    config_rows = []
    for mode in RUNTIME_MODES:
        for count in ROBOT_COUNTS:
            config = {
                "schema_version": 2, "runtime_mode": mode, "robot_count": count,
                "domain_id": g2["ros_domain_id"], "maximum_wall_time_s": 540,
                "physics_dt_s": g2["physics_dt_s"], "scene": scene_relative,
                "scene_sha256": scene_sha256, "scene_dependencies": scene_dependencies,
                "robots": build_robot_definitions(g2, fleet, routing, layout, count),
                "motion_profile": {"linear_m_s": .2, "start_sim_s": 1., "stop_sim_s": 4., "step_count": 420,
                    "nominal_displacement_m": MOTION_DISTANCE_M},
                "lidars": g2["lidars"], "lidar_profile": g2["lidar_profile"],
                "static_reference_lidar": False,
                "source_pins": {
                    "frozen_manifest_path": str(frozen_root / "manifest.json"),
                    "frozen_manifest_sha256": sha256(frozen_root / "manifest.json"),
                    "fleet_profile_sha256": sha256(frozen_root / "config/fleet-profile.json"),
                    "g2_sha256": sha256(frozen_root / "config/g2.json"),
                    "routing_sha256": sha256(frozen_root / "config/g3-routing.json"),
                    "layout_sha256": sha256(frozen_root / "config/layout.json"),
                },
                "scope": ("actual SDK wheel/joint/World time capacity probe"
                    + (" with ROS/RTX sensor topic receipt and stamps" if mode == "sensors" else " without ROS/RTX sensors")
                    + "; no snapshot replay, navigation, transport, cargo or charging qualification"),
            }
            validate_config(config)
            destination = output / f"{mode}-fleet-{count:02d}.json"
            destination.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
            config_rows.append({"mode": mode, "robot_count": count, "path": str(destination),
                "sha256": sha256(destination), "robot_ids": [row["robot_id"] for row in config["robots"]],
                "home_stations": [row["home_station"] for row in config["robots"]]})
    result = {
        "schema_version": 1, "status": "prepared_not_executed",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "scene": {"path": str(scene), "sha256": scene_sha256, "dependencies_sha256": scene_dependencies},
        "frozen_inputs": {"root": str(frozen_root), "manifest_sha256": sha256(frozen_root / "manifest.json"),
            "layout_revision": layout["revision"]},
        "configs": config_rows,
        "validation": {"config_count": len(config_rows), "robot_counts": list(ROBOT_COUNTS),
            "runtime_modes": list(RUNTIME_MODES), "unique_robot_id_namespace_prim_path": True,
            "unique_home_count": 16, "bounded_motion_distance_m": MOTION_DISTANCE_M,
            "clearance_radius_m": ROBOT_CLEARANCE_M, "all_motion_segments_inside_authored_clearance": True,
            "Kit_started": False, "CUDA_started": False, "GPU_started": False, "ROS_started": False},
        "remaining_conditions": [
            "Each config uses a 540 second internal deadline and requires a new output directory inside the existing Windows Job runner with a 600 second host bound.",
            "For future outputs, acquire the shared GPU lease after any other GPU owner finishes and freeze every input source before launch.",
            "A pass requires actual wheel displacement, joint movement, increasing World time, clean shutdown, and in sensors mode every expected topic with increasing nonzero stamps.",
            "The 1/4/8/16 ladder measures admission capacity; it does not prove navigation, transport, cargo, charging, or 16-AMR operational throughput.",
        ],
    }
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen-root", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.frozen_root.resolve(), args.scene.resolve(), args.scene_sha256,
        args.output.resolve(), REPO_ROOT)
    print(json.dumps({"status": result["status"], "configs": len(result["configs"]), "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
