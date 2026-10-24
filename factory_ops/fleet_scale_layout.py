"""Deterministic authored parking-annex configurations for larger AMR fleets."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any, Mapping

from .layout import FLEET_SCALE12_REVISION, FLEET_SCALE16_REVISION


SCALE_SPECS = {
    12: {"revision": FLEET_SCALE12_REVISION, "bounds_x": 51.0, "south_x": (36.0, 40.0, 44.0, 48.0)},
    16: {"revision": FLEET_SCALE16_REVISION, "bounds_x": 59.0, "south_x": (36.0, 40.0, 44.0, 48.0, 52.0, 56.0)},
}


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _pocket_coordinates(fleet_count: int) -> dict[str, list[float]]:
    spec = SCALE_SPECS[fleet_count]
    pockets: dict[str, list[float]] = {
        "P1": [12.0, 1.5], "P2": [12.0, 26.0],
        "P3": [17.0, 1.5], "P4": [17.0, 26.0],
    }
    index = 5
    for x in spec["south_x"]:
        pockets[f"P{index}"] = [x, 1.5]
        pockets[f"P{index + 1}"] = [x, 26.0]
        index += 2
    return pockets


def build_layout(base: Mapping[str, Any], fleet_count: int) -> dict[str, Any]:
    """Extend only the east transport/parking envelope of the dual-lane bay."""
    spec = SCALE_SPECS[fleet_count]
    result = deepcopy(dict(base))
    result["revision"] = spec["revision"]
    result["status"] = f"Authored {fleet_count}-AMR parking-annex design; runtime validation pending"
    result["bounds_xy"] = [0, 0, int(spec["bounds_x"]), 28]
    result["pockets"] = _pocket_coordinates(fleet_count)
    regions = {region["id"]: region for region in result["amr_allowed_regions"]}
    regions["MAIN_LOW"]["rect"][2] = spec["bounds_x"]
    regions["MAIN_HIGH"]["rect"][2] = spec["bounds_x"]
    for pocket_id, (x, y) in result["pockets"].items():
        if int(pocket_id[1:]) <= 4:
            continue
        result["amr_allowed_regions"].append({
            "id": f"{pocket_id}_POCKET",
            "rect": [x - 1.5, 0.0 if y < 10 else 22.5, x + 1.5, 4.5 if y < 10 else 28.0],
            "purpose": "off-main fleet parking and charger-queue waiting",
        })
    for lane in result.get("traffic_lanes", []):
        if lane["id"] in {"LOW_E", "HIGH_E"}:
            lane["to_xy"][0] = spec["south_x"][-1]
        elif lane["id"] in {"LOW_W", "HIGH_W"}:
            lane["from_xy"][0] = spec["south_x"][-1]
    result["unchanged_contracts"] = [
        *result.get("unchanged_contracts", []),
        "original 36x28m production, shipping, obstacle and blocked-aisle geometry retained",
        "east annex is service-access parking and virtual charger queue space, not added process capacity",
    ]
    result["unperformed_validation"] = [
        f"{fleet_count}-AMR runtime transport and throughput scaling",
        "parking-annex sensor/map alignment",
        "physical charging, cargo and safety certification",
    ]
    result["layout_provenance"] = {
        "kind": "authored-not-surveyed",
        "base": "configs/factory-semiconductor-dual-lane/layout.json",
        "preserved_production_bounds_xy": [0, 0, 36, 28],
        "annex_bounds_xy": [36, 0, int(spec["bounds_x"]), 28],
        "annex_role": "off-main fleet parking, charger queue and maintenance staging",
        "geometry_rule": "paired north/south 3m-wide bays on 4m centres, connected to the existing separated east/west mains",
        "reason": "scale fleet count without placing idle AMRs in process, shipping or main-traffic space",
        "pending": "asset footprints, multi-robot runtime, sensor/map alignment and operational capacity evidence",
    }
    result["production_domain"]["scope"] = (
        f"Existing 36x28m semiconductor final-test/pack bay plus an authored east parking annex; "
        f"{fleet_count}-AMR design only, not a surveyed factory"
    )
    return result


def build_routing(base: Mapping[str, Any], fleet_count: int) -> dict[str, Any]:
    """Add reachable parking branches while preserving every baseline route edge."""
    spec = SCALE_SPECS[fleet_count]
    result = deepcopy(dict(base))
    result["scope"] = f"{fleet_count}-AMR authored route and parking-annex preflight only; no runtime qualification"
    result["base_layout"] = f"configs/factory-semiconductor-scale{fleet_count}/layout.json"
    result["policy_region_overrides"]["MAIN_LOW"] = [2, 3, spec["bounds_x"], 9]
    result["policy_region_overrides"]["MAIN_HIGH"] = [2, 18, spec["bounds_x"], 24]

    pockets = _pocket_coordinates(fleet_count)
    south = [(name, xy[0]) for name, xy in pockets.items() if int(name[1:]) > 4 and xy[1] < 10]
    north = [(name, xy[0]) for name, xy in pockets.items() if int(name[1:]) > 4 and xy[1] > 10]

    def add_annex(items: list[tuple[str, float]], prefix_e: str, prefix_w: str,
                  entry_e: str, entry_w: str, y_e: float, y_w: float) -> None:
        previous_e = entry_e
        for pocket, x in items:
            node_e, node_w = f"{prefix_e}_{pocket}", f"{prefix_w}_{pocket}"
            result["nodes"][pocket] = [x, 1.5 if y_e < 10 else 26.0]
            result["nodes"][node_e] = [x, y_e]
            result["nodes"][node_w] = [x, y_w]
            result["edges"].extend([[pocket, node_e], [pocket, node_w]])
            result["directed_edges"].append([previous_e, node_e])
            previous_e = node_e
        far_pocket = items[-1][0]
        result["directed_edges"].append([f"{prefix_e}_{far_pocket}", f"{prefix_w}_{far_pocket}"])
        previous_w = f"{prefix_w}_{far_pocket}"
        for pocket, _x in reversed(items[:-1]):
            node_w = f"{prefix_w}_{pocket}"
            result["directed_edges"].append([previous_w, node_w])
            previous_w = node_w
        result["directed_edges"].append([previous_w, entry_w])

    add_annex(south, "LOW_E", "LOW_W", "LOW_E_EN", "LOW_W_EN", 4.5, 7.5)
    add_annex(north, "HIGH_E", "HIGH_W", "HIGH_E_EN", "HIGH_W_EN", 19.5, 22.5)
    result["parking"] = {f"amr-{index}": f"P{index}" for index in range(1, fleet_count + 1)}
    result["parking_provenance"] = {
        "kind": "authored_geometry",
        "minimum_home_center_spacing_m": 4.0,
        "service_meaning": "idle staging and two virtual-charger queues outside the production envelope",
        "runtime_validated": False,
    }
    return result


def build_fleet_profile(fleet_count: int) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "fleet_count": fleet_count,
        "robot_stations": [f"P{index}" for index in range(1, fleet_count + 1)],
        "scope": "Authored operations fleet start contract; no physical multi-robot execution or throughput claim",
        "provenance": {
            "kind": "authored_scale_candidate",
            "base": "factory-semiconductor-dual-lane four-AMR layout",
            "station_assignment": "one unique off-main home per robot",
            "pending": "operations integration, congestion experiment and Isaac Sim runtime evidence",
        },
    }


def build_charging(base: Mapping[str, Any], fleet_count: int) -> dict[str, Any]:
    result = deepcopy(dict(base))
    result["scope"] = (
        f"authored {fleet_count}-AMR operations-mode virtual battery, two docks and separate parking queue; "
        "no physics, electrical contact, hardware calibration or runtime evidence"
    )
    result["initial_soc"] = {
        f"amr-{index}": round(0.85 - ((index - 1) % 4) * 0.05, 2)
        for index in range(1, fleet_count + 1)
    }
    result["queue_stations"] = [f"P{index}" for index in range(3, fleet_count + 1)]
    result["queue_scope"] = "Authored waiting locations only; queue scheduling and charging throughput remain unvalidated"
    return result


def build_g2(base: Mapping[str, Any], fleet_count: int) -> dict[str, Any]:
    """Expand declared identities while retaining the authored drive/sensor limits."""
    result = deepcopy(dict(base))
    result["qualification"] = f"fleet{fleet_count}_configuration_declaration"
    result["robots"] = [
        {
            "robot_id": f"carter{index}",
            "namespace": f"carter{index}",
            "prim_path": f"/World/Carter{index}",
        }
        for index in range(1, fleet_count + 1)
    ]
    result["scope"] = deepcopy(result["scope"])
    result["scope"]["fleet_scale"] = (
        f"{fleet_count} declared robot identities with front/rear sensor IDs; "
        "configuration validation only, no USD spawn, sensor or physical runtime claim"
    )
    return result


def build_scale_bundle(base_dir: str | Path, fleet_count: int) -> dict[str, dict[str, Any]]:
    if fleet_count not in SCALE_SPECS:
        raise ValueError("fleet_count must be 12 or 16")
    root = Path(base_dir)
    render = _read(root / "render-profile.json")
    overview = next(camera for camera in render["cameras"] if camera["id"] == "portfolio_overview")
    midpoint = SCALE_SPECS[fleet_count]["bounds_x"] / 2
    overview.update(eye_xyz_m=[midpoint, -22.0, 40.0], target_xyz_m=[midpoint, 14.0, 0.7],
                    purpose="production bay and separate fleet parking annex; authored camera, capture pending")
    return {
        "layout.json": build_layout(_read(root / "layout.json"), fleet_count),
        "g3-routing.json": build_routing(_read(root / "g3-routing.json"), fleet_count),
        "fleet-profile.json": build_fleet_profile(fleet_count),
        "operations-charging.json": build_charging(_read(root / "operations-charging.json"), fleet_count),
        "visual-assets.json": _read(root / "visual-assets.json"),
        "render-profile.json": render,
        "g2.json": build_g2(_read(root / "g2.json"), fleet_count),
    }


def minimum_home_spacing(profile: Mapping[str, Any], layout: Mapping[str, Any]) -> float:
    stations = profile["robot_stations"]
    points = [layout["pockets"][name] for name in stations]
    return min(math.dist(left, right) for index, left in enumerate(points) for right in points[index + 1:])


__all__ = [
    "SCALE_SPECS", "build_charging", "build_fleet_profile", "build_g2", "build_layout", "build_routing",
    "build_scale_bundle", "minimum_home_spacing",
]
