"""Freeze four-AMR development inputs without claiming a physical mission ran."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
from typing import Any, Mapping

from .fleet import FleetOrder
from .fleet_registry import FleetRegistry
from .fleet_routing import RouteBook, swept_tiles
from .layout import FLEET4_REVISION, Layout
from .usda import export_usda
from .visual_state import canonical_sha256


QUALIFICATION = "fleet4_transport_development"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"
SOURCE_FILES = frozenset({"g2.json", "g3-runtime.json", "layout.json", "g3-routing.json",
                          "nav2-g2.yaml", "render-profile.json", "visual-assets.json", "transport-profile.json",
                          "energy-profile.json"})
BUNDLE_FILES = SOURCE_FILES | {"config.json", "session.json", "scenarios.json", "registry.json",
                               "scene/factory-v3.usda", "route-preflight.json"}


def transport_verification_record(*, run_id: str, session: Mapping[str, Any], registry: FleetRegistry,
                                  bundle_sha256: str) -> dict[str, Any]:
    """Share an SDK/ROS marker without importing the ROS adapter into Isaac."""
    source = os.environ.get("FACTORY_OPS_G3_SOURCE_COMMIT")
    if (re.fullmatch(r"[0-9a-f]{32}", run_id) is None
            or re.fullmatch(r"[0-9a-f]{64}", bundle_sha256) is None
            or not isinstance(source, str) or re.fullmatch(r"[0-9a-f]{40}", source) is None
            or session.get("profile") != "development" or session.get("primary_evaluation") is not False
            or session.get("scope") != QUALIFICATION):
        raise ValueError("fleet transport development identity differs")
    ros_dir = Path(__file__).resolve().parents[1] / "scripts/ros"
    return {"schema_version": 1, "status": "verified", "run_id": run_id,
            "scenario_id": session["scenario_id"], "bundle_sha256": bundle_sha256,
            "source_commit": source, "profile": "development", "primary_evaluation": False,
            "scope": session["scope"], "robot_ids": list(registry.robot_ids),
            "robots": {r["robot_id"]: {"namespace": r["namespace"], "prim_path": r["prim_path"]}
                       for r in registry.definitions},
            "source_files": {name: _sha(ros_dir / name)
                             for name in ("g3_mission.py", "fleet4_mission.py", "fleet4.launch.py")}}


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read(root: Path, name: str) -> Any:
    return json.loads((root / name).read_text(encoding="utf-8"))


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _positive(value: Any, field: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{field} must be finite and positive")
    return float(value)


def expand_lot_stream(profile: Mapping[str, Any]) -> dict[str, Any]:
    """Compile a finite shift's repeated lot arrivals into auditable identities."""
    result = deepcopy(dict(profile))
    stream = result.get("lot_stream")
    if stream is None:
        return result
    required = {"lot_count", "release_interval_s", "source_stations", "sku", "quantity", "scope"}
    if not isinstance(stream, dict) or set(stream) != required or "orders" in result:
        raise ValueError("lot_stream must declare a bounded shift without separate orders")
    count, quantity = stream["lot_count"], stream["quantity"]
    if type(count) is not int or not 4 <= count <= 120 or type(quantity) is not int or quantity <= 0:
        raise ValueError("lot stream requires 4..120 lots and positive integer tray quantity")
    interval = _positive(stream["release_interval_s"], "lot release interval")
    sources = stream["source_stations"]
    if (not isinstance(sources, list) or not sources or len(set(sources)) != len(sources)
            or any(s not in {"S1", "S2", "S3"} for s in sources)
            or not isinstance(stream["sku"], str) or not stream["sku"]
            or not isinstance(stream["scope"], str) or not stream["scope"]):
        raise ValueError("lot stream needs unique source stations, SKU and scope")
    material = result.get("material_flow")
    if not isinstance(material, dict) or "materials" in material:
        raise ValueError("lot stream generates material declarations from finite initial stock")
    result["orders"], material["materials"] = [], []
    for i in range(count):
        identity, source = f"SC-LOT-{i+1:03d}", sources[i % len(sources)]
        result["orders"].append({"order_id": identity, "payload_id": f"SC-PAY-{i+1:03d}",
            "source_station": source, "ready_time": i*interval})
        material["materials"].append({"order_id": identity, "source": source,
            "sku": stream["sku"], "quantity": quantity, "tote_id": f"SC-CARRIER-{i+1:03d}"})
    return result


def parking_preflight(book: RouteBook, runtime: Mapping[str, Any],
                      parking: Mapping[str, str]) -> dict[str, Any]:
    """Reject overlapping retained holds and parked robots blocking shared lanes."""
    if (len(parking) != 4 or len(set(parking.values())) != 4
            or set(parking.values()) != set(book.layout.pockets)
            or dict(parking) != book.config["parking"]):
        raise ValueError("transport parking must bind four distinct declared pockets")
    radius = book.planning_radius_m + _positive(runtime["reservation_tracking_margin_m"], "tracking margin")
    bounds = book.layout.bounds
    rectangle = (bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax)
    held = {robot: swept_tiles([book.nodes[anchor]], radius,
                               tile_size_m=book.tile_size_m, bounds_xy=rectangle)
            for robot, anchor in parking.items()}
    errors: list[dict[str, Any]] = []
    for robot, anchor in parking.items():
        x, y = book.nodes[anchor]
        if not (bounds.xmin + radius <= x <= bounds.xmax - radius
                and bounds.ymin + radius <= y <= bounds.ymax - radius):
            errors.append({"robot_id": robot, "error": "retained hold exceeds bounded floor"})
        for peer in held:
            if robot < peer and held[robot] & held[peer]:
                errors.append({"robot_id": robot, "peer_id": peer, "error": "parking holds overlap"})
        for left, right in sorted(book.edges):
            if left in book.layout.pockets or right in book.layout.pockets:
                continue
            lane = swept_tiles([book.nodes[left], book.nodes[right]], radius,
                               tile_size_m=book.tile_size_m, bounds_xy=rectangle)
            if held[robot] & lane:
                errors.append({"robot_id": robot, "edge": [left, right], "error": "parked hold blocks shared lane"})
    if errors:
        raise ValueError(f"four-AMR parking preflight failed: {errors}")
    return {"passed": True, "reservation_radius_m": radius,
            "parking_by_robot": dict(parking),
            "hold_tiles": {robot: [list(tile) for tile in sorted(tiles)] for robot, tiles in held.items()},
            "scope": "authored geometry and retained tiles only; no Nav2 or physical docking proof"}


def _compile(root: Path) -> dict[str, Any]:
    profile = expand_lot_stream(_read(root, "transport-profile.json"))
    config = _read(root, "g2.json")
    runtime = _read(root, "g3-runtime.json")
    book = RouteBook.from_files(root / "layout.json", root / "g3-routing.json")
    if book.base_layout.data.get("revision") != FLEET4_REVISION:
        raise ValueError("four-AMR transport requires the isolated fleet4 layout revision")
    if (profile.get("schema_version") != 1 or profile.get("scope") != QUALIFICATION
            or profile.get("primary_evaluation") is not False or config.get("qualification") != QUALIFICATION
            or profile.get("policy") not in ("fifo", "eta") or profile.get("condition") not in ("normal", "closure")):
        raise ValueError("transport development profile identity differs")
    horizon = _positive(profile.get("horizon_s"), "horizon_s")
    if horizon >= _positive(config.get("max_sim_time_s"), "max_sim_time_s"):
        raise ValueError("transport horizon must fit within the simulation bound")
    _positive(config.get("max_wall_time_s"), "max_wall_time_s")
    initial = profile["initial_robots"]
    parking = parking_preflight(book, runtime, initial)
    definitions = config["robots"]
    robot_ids = {row["robot_id"] for row in definitions}
    if robot_ids != set(initial) or len(definitions) != 4:
        raise ValueError("transport starts must exactly match four configured robot IDs")
    yaws = runtime["anchor_yaw_rad"]
    anchors = {**book.layout.stations, **book.layout.pockets}
    if any(type(yaws.get(anchor)) not in (int, float) or not math.isfinite(yaws[anchor]) for anchor in anchors):
        raise ValueError("transport requires a finite yaw for every operation/parking anchor")
    orders = [FleetOrder(**row) for row in profile["orders"]]
    if (not orders or len({row.order_id for row in orders}) != len(orders)
            or len({row.payload_id for row in orders}) != len(orders)
            or any(row.ready_time >= horizon for row in orders)):
        raise ValueError("transport orders must have distinct identities and fit the horizon")
    load = profile["load"]
    for key in ("ci_capacity", "co_capacity", "workers"):
        if type(load.get(key)) is not int or load[key] <= 0:
            raise ValueError(f"transport load {key} must be a positive integer")
    for key in ("service_time_s", "nominal_speed_m_s", "dwell_s"):
        _positive(load.get(key), key)
    scene_closure = profile.get("scene_closure")
    if profile["condition"] == "normal":
        if scene_closure is not None:
            raise ValueError("normal transport cannot declare a hidden scene closure")
    else:
        if not isinstance(scene_closure, dict):
            raise ValueError("closure transport requires an authored scene closure")
        interval = scene_closure.get("scene_active_interval_s")
        if (not isinstance(interval, list) or len(interval) != 2
                or any(type(t) not in (int, float) or not math.isfinite(t) for t in interval)
                or not 0 <= interval[0] < interval[1] <= horizon):
            raise ValueError("scene closure interval must fit the mission horizon")
        scene_closure = {"scene_active_interval_s": list(interval),
                         "rect_xyxy_m": list(book.layout.data["blocked_aisle"]["rect"]),
                         "height_m": book.layout.data["blocked_aisle"]["height_m"],
                         "scope": "evaluator-owned schedule; operational routing observes current state only"}
    config = deepcopy(config)
    config["g3_runtime"] = runtime
    scenario_id = f"fleet4_transport_{profile['condition']}"
    scenario = {"spawns": {robot: [*anchors[anchor], yaws[anchor]] for robot, anchor in initial.items()},
                "max_sim_time_s": config["max_sim_time_s"], "scope": QUALIFICATION}
    registry = FleetRegistry(config, scenario)
    workload = {"initial_robots": dict(initial), "load": dict(load),
                "orders": [row.to_dict() for row in orders], "horizon_s": horizon}
    if "lot_stream" in profile:
        workload["lot_stream"] = deepcopy(profile["lot_stream"])
    chaining = runtime.get("post_drop_dispatch")
    if chaining is not None:
        if (not isinstance(chaining, dict) or set(chaining) != {"mode", "idle_grace_s"}
                or chaining["mode"] != "chain_then_park"
                or type(chaining["idle_grace_s"]) not in (int, float)
                or not math.isfinite(chaining["idle_grace_s"]) or not 0 < chaining["idle_grace_s"] <= 5):
            raise ValueError("post-drop dispatch requires chain_then_park with 0..5s idle grace")
    if "material_flow" in profile:
        from .fleet import FleetMission
        from .material_flow import MaterialFlow
        from .navigation import GoalPose
        MaterialFlow(FleetMission("input-material-preflight", orders, robot_stations=initial,
            station_goals={name: GoalPose("map", *xy, yaws[name]) for name, xy in anchors.items()}), profile["material_flow"])
        workload["material_flow"] = deepcopy(profile["material_flow"])
    if "energy" in profile:
        from .energy import EnergyFleet
        energy = {"profile": _read(root, "energy-profile.json"), "initial_soc": deepcopy(profile["energy"]["initial_soc"])}
        if set(energy["initial_soc"]) != robot_ids:
            raise ValueError("energy initial SOC must cover exactly the declared fleet")
        EnergyFleet(energy["profile"], energy["initial_soc"])
        if "dispatch" in profile["energy"]:
            settings = deepcopy(profile["energy"]["dispatch"])
            if (set(settings) != {"charger_anchors", "expected_wait_s", "angular_speed_rad_s"}
                    or not isinstance(settings["charger_anchors"], list) or not settings["charger_anchors"]
                    or any(not isinstance(anchor, str) or anchor not in anchors for anchor in settings["charger_anchors"])
                    or len(set(settings["charger_anchors"])) != len(settings["charger_anchors"])):
                raise ValueError("energy dispatch requires explicit distinct known charger-return anchors")
            _positive(settings["expected_wait_s"], "energy expected wait")
            _positive(settings["angular_speed_rad_s"], "energy planning angular speed")
            energy["dispatch"] = settings
        workload["energy"] = energy
    session = {"schema_version": 1, "scenario_id": scenario_id, "profile": "development",
               "primary_evaluation": False, "scope": QUALIFICATION, "condition": profile["condition"],
               "policy": profile["policy"], "scene_closure": scene_closure,
               "workload": workload, "workload_sha256": canonical_sha256(workload)}
    routes = book.preflight()
    if not routes["passed"]:
        raise ValueError(f"four-AMR route preflight failed: {routes['errors']}")
    preflight = {"passed": True, "route_count": routes["route_count"],
                 "edge_count": len(routes["edge_checks"]), "parking": parking,
                 "layout_sha256": book.base_layout.source_sha256,
                 "routing_sha256": _sha(root / "g3-routing.json"),
                 "scope": "authored preflight; not four-robot runtime qualification"}
    return {"config": config, "session": session, "scenario": scenario,
            "registry": registry, "book": book, "preflight": preflight}


def prepare_bundle(output: Path, config_dir: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    source = config_dir.resolve(strict=True)
    compiled = _compile(source)
    if output.exists() or output.is_symlink():
        raise ValueError("transport bundle must use a new directory")
    output = output.absolute()
    output.mkdir(parents=True)
    for name in sorted(SOURCE_FILES):
        shutil.copyfile(source / name, output / name)
    _write(output / "config.json", compiled["config"])
    _write(output / "session.json", compiled["session"])
    _write(output / "scenarios.json", {"scenarios": {compiled["session"]["scenario_id"]: compiled["scenario"]}})
    _write(output / "registry.json", compiled["registry"].manifest())
    _write(output / "route-preflight.json", compiled["preflight"])
    export_usda(compiled["book"].base_layout, output / "scene/factory-v3.usda")
    _write(output / "bundle.json", {"schema_version": 1, "qualification": QUALIFICATION,
            "scope": "development inputs only; no primary evaluation or physical transport claim",
            "source_files": {name: _sha(source / name) for name in sorted(SOURCE_FILES)},
            "files": {name: _sha(output / name) for name in sorted(BUNDLE_FILES)}})
    return verify_bundle(output)


def load_bundle(root: Path) -> dict[str, Any]:
    if root.is_symlink():
        raise ValueError("transport bundle must be a regular directory")
    root = root.resolve(strict=True)
    manifest = _read(root, "bundle.json")
    if (manifest.get("schema_version") != 1 or manifest.get("qualification") != QUALIFICATION
            or set(manifest.get("files", {})) != BUNDLE_FILES):
        raise ValueError("transport bundle manifest identity/files differ")
    for name, digest in manifest["files"].items():
        path = root / name
        if (not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest)
                or path.is_symlink() or not path.is_file() or _sha(path) != digest):
            raise ValueError(f"transport input changed: {name}")
    if manifest.get("source_files") != {name: _sha(root / name) for name in SOURCE_FILES}:
        raise ValueError("transport source file provenance differs")
    compiled = _compile(root)
    expected = {"config.json": compiled["config"], "session.json": compiled["session"],
                "scenarios.json": {"scenarios": {compiled["session"]["scenario_id"]: compiled["scenario"]}},
                "registry.json": compiled["registry"].manifest(), "route-preflight.json": compiled["preflight"]}
    for name, value in expected.items():
        if _read(root, name) != value:
            raise ValueError(f"transport derived input differs: {name}")
    with tempfile.TemporaryDirectory(prefix="factory-fleet4-scene-") as temp:
        regenerated = export_usda(compiled["book"].base_layout, Path(temp) / "factory.usda")
    if regenerated["sha256"] != _sha(root / "scene/factory-v3.usda"):
        raise ValueError("transport scene differs from the authored layout")
    compiled["bundle_sha256"] = _sha(root / "bundle.json")
    return compiled


def verify_bundle(root: Path) -> dict[str, Any]:
    compiled = load_bundle(root)
    return {"passed": True, "scope": QUALIFICATION, "primary_evaluation": False,
            "bundle_sha256": compiled["bundle_sha256"],
            "robot_ids": list(compiled["registry"].robot_ids),
            "lidar_count": len(compiled["registry"].sensor_keys), "preflight": compiled["preflight"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    report = verify_bundle(args.bundle) if args.verify else prepare_bundle(args.bundle, args.config_dir)
    print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
