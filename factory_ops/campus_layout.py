"""Author a four-bay semiconductor final-test campus for a 16-AMR fleet.

The generated geometry is a planning hypothesis assembled from the existing
dual-lane bay.  It is not surveyed CAD and carries no runtime qualification.
"""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Mapping

from .process_line_layout import add_process_line_routing


CAMPUS_REVISION = "amr-semiconductor-campus16-v1"
BAY_OFFSETS = {"A": (0.0, 0.0), "B": (42.0, 0.0), "C": (0.0, 32.0), "D": (42.0, 32.0)}
CAMPUS_BOUNDS = [0, 0, 78, 60]


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _shift_point(point: list[float], offset: tuple[float, float]) -> list[float]:
    return [point[0] + offset[0], point[1] + offset[1]]


def _shift_rect(rect: list[float], offset: tuple[float, float]) -> list[float]:
    return [rect[0] + offset[0], rect[1] + offset[1], rect[2] + offset[0], rect[3] + offset[1]]


def _prefix(name: str, bay: str) -> str:
    return f"{bay}_{name}"


def build_campus_layout(base: Mapping[str, Any]) -> dict[str, Any]:
    source = deepcopy(dict(base))
    result = deepcopy(source)
    result.update(
        revision=CAMPUS_REVISION,
        status="Authored four-bay 16-AMR campus; RouteBook, renderer and physical runtime qualification pending",
        bounds_xy=deepcopy(CAMPUS_BOUNDS),
    )
    result["amr_allowed_regions"] = []
    result["stations"] = []
    result["pockets"] = {}
    result["physical_obstacles"] = []
    result["traffic_lanes"] = []
    blocked_aisles = []

    for bay, offset in BAY_OFFSETS.items():
        for region in source["amr_allowed_regions"]:
            item = deepcopy(region)
            item["id"] = _prefix(item["id"], bay)
            item["rect"] = _shift_rect(item["rect"], offset)
            result["amr_allowed_regions"].append(item)
        for station in source["stations"]:
            item = deepcopy(station)
            item["id"] = _prefix(item["id"], bay)
            item["xy"] = _shift_point(item["xy"], offset)
            result["stations"].append(item)
        for pocket, point in source["pockets"].items():
            result["pockets"][_prefix(pocket, bay)] = _shift_point(point, offset)
        for obstacle in source["physical_obstacles"]:
            item = deepcopy(obstacle)
            item["id"] = _prefix(item["id"], bay)
            item["rect"] = _shift_rect(item["rect"], offset)
            result["physical_obstacles"].append(item)
        blocked = deepcopy(source["blocked_aisle"])
        blocked["id"] = f"{bay}_BLOCKED_AISLE"
        blocked["rect"] = _shift_rect(blocked["rect"], offset)
        blocked_aisles.append(blocked)
        for lane in source.get("traffic_lanes", []):
            item = deepcopy(lane)
            item["id"] = _prefix(item["id"], bay)
            item["from_xy"] = _shift_point(item["from_xy"], offset)
            item["to_xy"] = _shift_point(item["to_xy"], offset)
            result["traffic_lanes"].append(item)

    bridges = [
        ("H_AB", [28.5, 3.0, 51.0, 9.0], "east/west paired lanes between south bays"),
        ("H_CD", [28.5, 35.0, 51.0, 41.0], "east/west paired lanes between north bays"),
        ("V_AC", [31.4, 22.5, 36.0, 36.5], "single-reservation north/south link on the clear east perimeter"),
        ("V_BD", [73.4, 22.5, 78.0, 36.5], "single-reservation north/south link on the clear east perimeter"),
    ]
    result["amr_allowed_regions"].extend(
        {"id": f"CAMPUS_{name}", "rect": rect, "purpose": purpose}
        for name, rect, purpose in bridges
    )
    result["amr_allowed_regions"].extend([
        {"id": "CAMPUS_V_AC_S_APPROACH", "rect": [28.5, 18.0, 36.0, 24.4], "purpose": "clear south-bay east-spine bridge approach"},
        {"id": "CAMPUS_V_AC_N_APPROACH", "rect": [28.5, 35.0, 36.0, 41.0], "purpose": "clear north-bay east-spine bridge approach"},
        {"id": "CAMPUS_V_BD_S_APPROACH", "rect": [70.5, 18.0, 78.0, 24.4], "purpose": "clear south-bay east-spine bridge approach"},
        {"id": "CAMPUS_V_BD_N_APPROACH", "rect": [70.5, 35.0, 78.0, 41.0], "purpose": "clear north-bay east-spine bridge approach"},
    ])
    result["amr_allowed_regions"].extend([
        {"id": "CAMPUS_H_AB_RESERVATION", "rect": [32.0, 5.0, 47.0, 8.8], "purpose": "atomic shared bridge reservation overlay"},
        {"id": "CAMPUS_H_CD_RESERVATION", "rect": [32.0, 37.0, 47.0, 40.8], "purpose": "atomic shared bridge reservation overlay"},
        {"id": "CAMPUS_V_AC_RESERVATION", "rect": [33.0, 24.0, 35.8, 36.0], "purpose": "atomic shared bridge reservation overlay"},
        {"id": "CAMPUS_V_BD_RESERVATION", "rect": [75.0, 24.0, 77.8, 36.0], "purpose": "atomic shared bridge reservation overlay"},
    ])
    result["blocked_aisles"] = blocked_aisles
    result["blocked_aisle"] = deepcopy(blocked_aisles[0])
    result["blocked_aisle"]["compatibility_scope"] = "A-bay compatibility view; blocked_aisles is authoritative for the campus"
    result["unchanged_contracts"] = [
        "each bay preserves all 19 source equipment footprints and the original AMR lane geometry",
        "collision, evaluation, localization and reservation radii are unchanged",
        "inter-bay links use gated main/east-spine continuations and do not cross a raw-material station centre",
    ]
    result["unperformed_validation"] = [
        "16-AMR RouteBook and reservation runtime",
        "four-bay USD scene transformation, asset placement and sensor/map alignment",
        "physical cargo, charging, process timing and safety certification",
    ]
    result["layout_provenance"] = {
        "kind": "authored-not-surveyed",
        "base": "configs/factory-semiconductor-dual-lane/layout.json",
        "bay_offsets_xy_m": {bay: list(offset) for bay, offset in BAY_OFFSETS.items()},
        "campus_bounds_xy_m": deepcopy(CAMPUS_BOUNDS),
        "geometry_rule": "four immutable 36x28m bays on 42m x 32m centres with clear directional bridge corridors",
        "pending": "RouteBook preflight, USD scene transformation, renderer coverage and multi-robot physical evidence",
    }
    result["production_domain"] = deepcopy(source["production_domain"])
    result["production_domain"]["scope"] = (
        "Four authored semiconductor final-test/inspection/pack bays in a 78x60m campus; "
        "planning geometry only, not surveyed CAD or process certification"
    )
    result["production_domain"]["zone_labels"] = [
        {**deepcopy(label), "label": f"{bay} {label['label']}", "origin_xy": _shift_point(label["origin_xy"], offset)}
        for bay, offset in BAY_OFFSETS.items()
        for label in source["production_domain"].get("zone_labels", [])
    ]
    return result


def _prefix_routing(base: Mapping[str, Any], bay: str, offset: tuple[float, float]) -> dict[str, Any]:
    source = add_process_line_routing(base)
    result = deepcopy(source)
    result["nodes"] = {_prefix(name, bay): _shift_point(point, offset) for name, point in source["nodes"].items()}
    result["edges"] = [[_prefix(left, bay), _prefix(right, bay)] for left, right in source["edges"]]
    result["directed_edges"] = [
        [_prefix(left, bay), _prefix(right, bay)] for left, right in source["directed_edges"]
    ]
    result["policy_region_overrides"] = {
        _prefix(name, bay): _shift_rect(rect, offset)
        for name, rect in source["policy_region_overrides"].items()
    }
    result["operations_bottlenecks"] = [
        {
            **deepcopy(item),
            "id": _prefix(item["id"], bay),
            "region_id": _prefix(item["region_id"], bay),
        }
        for item in source.get("operations_bottlenecks", [])
    ]
    result["closure"] = deepcopy(source["closure"])
    result["closure"]["edge"] = [_prefix(name, bay) for name in source["closure"]["edge"]]
    result["closure"]["barrier_source"] = f"configs/factory-semiconductor-campus16/layout.json#/blocked_aisles/{bay}"
    line = deepcopy(source["process_line_routing"])
    for value in line.values():
        if isinstance(value, dict) and "anchor" in value:
            value["anchor"] = _prefix(value["anchor"], bay)
    result["process_line_routing"] = line
    return result


def build_campus_routing(base: Mapping[str, Any]) -> dict[str, Any]:
    bays = {bay: _prefix_routing(base, bay, offset) for bay, offset in BAY_OFFSETS.items()}
    first = bays["A"]
    result = {
        key: deepcopy(value)
        for key, value in first.items()
        if key not in {"nodes", "edges", "directed_edges", "policy_region_overrides", "operations_bottlenecks", "closure", "closures", "parking", "process_line_routing"}
    }
    result["scope"] = "Four-bay 16-AMR authored route graph; RouteBook, reservation and physical runtime qualification pending"
    result["base_layout"] = "configs/factory-semiconductor-campus16/layout.json"
    result["nodes"] = {}
    result["edges"] = []
    result["directed_edges"] = []
    result["policy_region_overrides"] = {}
    result["operations_bottlenecks"] = []
    result["closures"] = []
    result["process_line_routing"] = {}
    for bay, routing in bays.items():
        result["nodes"].update(routing["nodes"])
        result["edges"].extend(routing["edges"])
        result["directed_edges"].extend(routing["directed_edges"])
        result["policy_region_overrides"].update(routing["policy_region_overrides"])
        result["operations_bottlenecks"].extend(routing["operations_bottlenecks"])
        result["closures"].append(routing["closure"])
        result["process_line_routing"][bay] = routing["process_line_routing"]

    bridge_regions = {
        "CAMPUS_H_AB_RESERVATION": [32.0, 5.0, 47.0, 8.8],
        "CAMPUS_H_CD_RESERVATION": [32.0, 37.0, 47.0, 40.8],
        "CAMPUS_V_AC_RESERVATION": [33.0, 24.0, 35.8, 36.0],
        "CAMPUS_V_BD_RESERVATION": [75.0, 24.0, 77.8, 36.0],
    }
    result["policy_region_overrides"].update(bridge_regions)
    for region_id in bridge_regions:
        result["operations_bottlenecks"].append({
            "id": region_id.lower().replace("_", "-"),
            "region_id": region_id,
            "release_policy": "first_route_point_outside_region",
        })

    connector_nodes = {
        "AB_E_W_GATE": [31.0, 6.0], "AB_E_W_IN": [33.0, 6.0],
        "AB_E_E_IN": [46.0, 6.0], "AB_E_E_GATE": [48.0, 6.0],
        "AB_W_E_GATE": [48.0, 7.5], "AB_W_E_IN": [46.0, 7.5],
        "AB_W_W_IN": [33.0, 7.5], "AB_W_W_GATE": [31.0, 7.5],
        "CD_E_W_GATE": [31.0, 38.0], "CD_E_W_IN": [33.0, 38.0],
        "CD_E_E_IN": [46.0, 38.0], "CD_E_E_GATE": [48.0, 38.0],
        "CD_W_E_GATE": [48.0, 39.5], "CD_W_E_IN": [46.0, 39.5],
        "CD_W_W_IN": [33.0, 39.5], "CD_W_W_GATE": [31.0, 39.5],
        "AC_S_GATE": [33.0, 23.5], "AC_S_IN": [34.5, 25.0],
        "AC_N_IN": [34.5, 35.0], "AC_N_GATE": [33.0, 36.5],
        "BD_S_GATE": [75.0, 23.5], "BD_S_IN": [76.5, 25.0],
        "BD_N_IN": [76.5, 35.0], "BD_N_GATE": [75.0, 36.5],
    }
    result["nodes"].update(connector_nodes)
    connector_paths = {
        "AB_E": ["A_LOW_E_EN", "AB_E_W_GATE", "AB_E_W_IN", "AB_E_E_IN", "AB_E_E_GATE", "B_LOW_E_W"],
        "AB_W": ["B_LOW_W_W", "AB_W_E_GATE", "AB_W_E_IN", "AB_W_W_IN", "AB_W_W_GATE", "A_LOW_W_EN"],
        "CD_E": ["C_LOW_E_EN", "CD_E_W_GATE", "CD_E_W_IN", "CD_E_E_IN", "CD_E_E_GATE", "D_LOW_E_W"],
        "CD_W": ["D_LOW_W_W", "CD_W_E_GATE", "CD_W_E_IN", "CD_W_W_IN", "CD_W_W_GATE", "C_LOW_W_EN"],
        "AC_N": ["A_HIGH_E_EN", "AC_S_GATE", "AC_S_IN", "AC_N_IN", "AC_N_GATE", "C_LOW_E_EN"],
        "AC_S": ["C_LOW_W_EN", "AC_N_GATE", "AC_N_IN", "AC_S_IN", "AC_S_GATE", "A_HIGH_W_EN"],
        "BD_N": ["B_HIGH_E_EN", "BD_S_GATE", "BD_S_IN", "BD_N_IN", "BD_N_GATE", "D_LOW_E_EN"],
        "BD_S": ["D_LOW_W_EN", "BD_N_GATE", "BD_N_IN", "BD_S_IN", "BD_S_GATE", "B_HIGH_W_EN"],
    }
    for path in connector_paths.values():
        result["directed_edges"].extend([list(edge) for edge in zip(path, path[1:])])
    result["bridge_admission"] = {
        "policy": "atomic_route_leg_resources",
        "resources": {link: sorted(node for node in connector_nodes if node.startswith(link + "_"))
                      for link in ("AB", "CD", "AC", "BD")},
    }
    result["operations_bottleneck_policy"] = "merge_adjacent_route_spans"
    result["inter_bay_connections"] = {
        "scope": "Directional gated links; every shared bridge has an authored outside gate and no connector segment crosses a raw-material station centre",
        "links": [
            {"id": "AB", "eastbound": [*connector_paths["AB_E"], "B_LOW_E_P1"], "westbound": connector_paths["AB_W"]},
            {"id": "CD", "eastbound": [*connector_paths["CD_E"], "D_LOW_E_P1"], "westbound": connector_paths["CD_W"]},
            {"id": "AC", "northbound": [*connector_paths["AC_N"], "C_LOW_W_EN"], "southbound": [*connector_paths["AC_S"], "A_HIGH_W_ES"]},
            {"id": "BD", "northbound": [*connector_paths["BD_N"], "D_LOW_W_EN"], "southbound": [*connector_paths["BD_S"], "B_HIGH_W_ES"]},
        ],
    }
    result["closure"] = deepcopy(result["closures"][0])
    result["closure"]["compatibility_scope"] = "A-bay compatibility view; closures is authoritative for the campus"
    result["parking"] = {
        f"amr-{index}": f"{bay}_P{slot}"
        for index, (bay, slot) in enumerate(
            ((bay, slot) for bay in BAY_OFFSETS for slot in range(1, 5)), start=1
        )
    }
    result["parking_provenance"] = {
        "kind": "authored_geometry",
        "assignment": "four unique off-main homes per bay",
        "runtime_validated": False,
    }
    return result


def build_fleet_profile() -> dict[str, Any]:
    homes = [f"{bay}_P{slot}" for bay in BAY_OFFSETS for slot in range(1, 5)]
    return {
        "schema_version": 1,
        "fleet_count": 16,
        "robot_stations": homes,
        "scope": "Authored four-bay fleet start contract; no 16-AMR physical runtime claim",
        "provenance": {"kind": "authored_campus_candidate", "pending": "RouteBook, reservation, sensor and physics evidence"},
    }


def build_charging(base: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    result["scope"] = "Authored 16-AMR virtual energy model with two geometric docks per bay; no electrical or physical claim"
    result["initial_soc"] = {
        f"amr-{index}": round(0.85 - ((index - 1) % 4) * 0.05, 2) for index in range(1, 17)
    }
    result["docks"] = {}
    for bay in BAY_OFFSETS:
        result["docks"][f"{bay}_CH1"] = {
            "anchor": f"{bay}_P1", "exit_anchor": f"{bay}_LOW_E_P1", "yaw_rad": -1.5707963267948966,
        }
        result["docks"][f"{bay}_CH2"] = {
            "anchor": f"{bay}_P2", "exit_anchor": f"{bay}_HIGH_W_P2", "yaw_rad": 1.5707963267948966,
        }
    result["queue_stations"] = [f"{bay}_P{slot}" for bay in BAY_OFFSETS for slot in (3, 4)]
    result["queue_scope"] = "Authored per-bay charger waiting homes; queue scheduling and throughput remain unvalidated"
    return result


def build_g2(base: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    result["qualification"] = "campus16_configuration_declaration"
    result["robots"] = [
        {"robot_id": f"carter{index}", "namespace": f"carter{index}", "prim_path": f"/World/Carter{index}"}
        for index in range(1, 17)
    ]
    result["scope"] = deepcopy(result["scope"])
    result["scope"]["fleet_scale"] = (
        "16 declared robot identities with inherited front/rear sensor definitions; "
        "configuration only, no USD spawn, sensor or physical runtime claim"
    )
    return result


def build_process_network() -> dict[str, Any]:
    specifications = (
        ("L1", "A", "B", "D", "D"),
        ("L2", "B", "D", "C", "C"),
        ("L3", "C", "A", "B", "B"),
        ("L4", "D", "C", "A", "A"),
    )
    lines = []
    for line_id, test_bay, aoi_bay, pack_bay, ship_bay in specifications:
        lines.append({
            "id": line_id,
            "source_locations": [f"{test_bay}_S{index}" for index in range(1, 4)],
            "config": {
                "schema_version": 1,
                "shipping_station": f"{ship_bay}_OUT",
                "cells": [
                    {"id": "TEST", "input_station": f"{test_bay}_CI", "output_station": f"{test_bay}_CO", "processing_time_s": 30, "input_capacity": 2, "output_capacity": 2},
                    {"id": "AOI", "input_station": f"{aoi_bay}_AOI_IN", "output_station": f"{aoi_bay}_AOI_OUT", "processing_time_s": 20, "input_capacity": 2, "output_capacity": 2},
                    {"id": "PACK", "input_station": f"{pack_bay}_PACK_IN", "output_station": f"{pack_bay}_PACK_OUT", "processing_time_s": 15, "input_capacity": 2, "output_capacity": 2},
                ],
            },
        })
    return {
        "schema_version": 1,
        "scope": "Authored four-line campus process network; timings and capacities are simulation inputs, not measured factory performance",
        "dispatch_mode": "downstream_wip_drain_least_assigned",
        "inventory": {"stock_per_source": 20, "carrier_count": 64},
        "lines": lines,
        "provenance": {"kind": "authored_candidate", "pending": "global dispatch, RouteBook, physics and throughput evidence"},
    }


def build_visual_assets(base: Mapping[str, Any]) -> dict[str, Any]:
    source = deepcopy(dict(base))
    result = deepcopy(source)
    result["source"] = source["source"] + " Four-bay placement is an authored translation pending an actual campus render."
    result["obstacle_assets"] = {
        _prefix(name, bay): {
            **deepcopy(value),
            **({"source_obstacle_id": _prefix(value["source_obstacle_id"], bay)} if "source_obstacle_id" in value else {}),
        }
        for bay in BAY_OFFSETS
        for name, value in source["obstacle_assets"].items()
    }
    environment = deepcopy(source["environment"])
    environment["scope"] = (
        "Four translated authored service-area sets; actual USD campus transformation, renderer coverage and collision review pending"
    )
    environment["fixtures"] = []
    excluded_local = {
        "A": {"EAST_SERVICE_TOOLS_LOW", "EAST_SERVICE_CART_LOW", "EAST_SERVICE_TOOLS_HIGH"},
        "B": {"WEST_SERVICE_TOOLS_LOW", "WEST_SERVICE_CART_LOW", "EAST_SERVICE_TOOLS_HIGH"},
        "C": {"EAST_SERVICE_TOOLS_LOW", "EAST_SERVICE_CART_LOW"},
        "D": {"WEST_SERVICE_TOOLS_LOW", "WEST_SERVICE_CART_LOW", "EAST_SERVICE_TOOLS_LOW", "EAST_SERVICE_CART_LOW"},
    }
    excluded = []
    for bay, offset in BAY_OFFSETS.items():
        for fixture in source["environment"]["fixtures"]:
            if fixture["id"] in excluded_local[bay]:
                excluded.append({
                    "id": _prefix(fixture["id"], bay),
                    "reason": "translated native fixture bounds plus 0.1m clearance intersect an authored campus bridge",
                })
                continue
            item = deepcopy(fixture)
            item["id"] = _prefix(item["id"], bay)
            item["center_xy"] = _shift_point(item["center_xy"], offset)
            if "source_obstacle_id" in item:
                item["source_obstacle_id"] = _prefix(item["source_obstacle_id"], bay)
            environment["fixtures"].append(item)
    environment["excluded_fixtures"] = excluded
    environment["fixture_count"] = len(environment["fixtures"])
    result["environment"] = environment
    result["campus_transform"] = {
        "bay_offsets_xy_m": {bay: list(offset) for bay, offset in BAY_OFFSETS.items()},
        "scope": "Authored JSON placement only; scene generation and successful render are pending",
    }
    return result


def build_render_profile(base: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    overview = next(camera for camera in result["cameras"] if camera["id"] == "portfolio_overview")
    overview.update(
        eye_xyz_m=[39.0, -35.0, 72.0],
        target_xyz_m=[39.0, 30.0, 0.7],
        focal_length_mm=22,
        purpose="four-bay semiconductor campus and 16-AMR material flow; authored camera, capture pending",
    )
    result["campus_scope"] = "Camera authoring only; four-bay USD transform and successful GPU render pending"
    return result


def build_campus_bundle(base_dir: str | Path) -> dict[str, dict[str, Any]]:
    root = Path(base_dir)
    return {
        "layout.json": build_campus_layout(_read(root / "layout.json")),
        "g3-routing.json": build_campus_routing(_read(root / "g3-routing.json")),
        "fleet-profile.json": build_fleet_profile(),
        "operations-charging.json": build_charging(_read(root / "operations-charging.json")),
        "g2.json": build_g2(_read(root / "g2.json")),
        "process-network.json": build_process_network(),
        "visual-assets.json": build_visual_assets(_read(root / "visual-assets.json")),
        "render-profile.json": build_render_profile(_read(root / "render-profile.json")),
    }


__all__ = [
    "BAY_OFFSETS", "CAMPUS_BOUNDS", "CAMPUS_REVISION", "build_campus_bundle", "build_campus_layout",
    "build_campus_routing", "build_charging", "build_fleet_profile", "build_g2", "build_process_network",
    "build_render_profile", "build_visual_assets",
]
