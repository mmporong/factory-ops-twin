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


CAMPUS_REVISION = "amr-semiconductor-campus16-v2"
CAMPUS_Y_OFFSET = 8.0
BAY_OFFSETS = {"A": (0.0, 8.0), "B": (42.0, 8.0), "C": (0.0, 40.0), "D": (42.0, 40.0)}
CAMPUS_BOUNDS = [0, 0, 82, 76]
PACK_EGRESS_LOCAL = [27.0, 4.5]
PACK_WAIT_POINTS = {
    bay: [27.0 + offset[0], (1.5 if bay in {"A", "B"} else -1.5) + offset[1]]
    for bay, offset in BAY_OFFSETS.items()
}
PACK_STAGING_SPUR_LOCAL = [25.4, 0.1, 28.6, 9.0]
PACK_STAGING_SPUR_NORTH_LOCAL = [25.4, -3.2, 28.6, 9.0]
LOCAL_STAGING_POINTS = {
    "SOURCE_WAIT": [3.5, 19.5],
    "CELL_WAIT": [24.5, 10.5],
}
SHARED_JUNCTION_REGIONS = {
    "CAMPUS_C_LOWER_JUNCTION": [28.4, 44.4, 33.1, 47.6],
    "CAMPUS_D_LOW_W_TRUNK": [56.9, 47.4, 67.6, 47.6],
    "CAMPUS_D_EAST_CI_TURN": [67.4, 47.4, 67.6, 50.6],
    "CAMPUS_D_EAST_SPINE_TURN": [70.4, 44.4, 70.6, 47.6],
}
BRIDGE_QUEUES = {
    "AB_E_QUEUE": [32.0, 9.5],
    "AB_W_QUEUE": [60.0, 15.5],
    "CD_E_QUEUE": [30.5, 41.5],
    "CD_W_QUEUE": [60.0, 45.5],
    "AC_S_QUEUE": [33.5, 27.5],
    "AC_N_QUEUE": [31.5, 38.5],
    "BD_S_QUEUE": [75.5, 27.5],
    "BD_N_QUEUE": [73.5, 38.5],
}
BRIDGE_QUEUE_ACCESS_REGIONS = {
    "CAMPUS_AB_E_QUEUE_ACCESS": [30.45, 8.0, 33.5, 12.5],
    "CAMPUS_AC_N_QUEUE_ACCESS": [28.3, 36.8, 37.0, 45.0],
    "CAMPUS_BD_N_QUEUE_ACCESS": [70.3, 36.8, 75.2, 43.2],
}
BRIDGE_SAFE_EXITS = {
    "A_AB_W_SAFE_EXIT": [15.0, 3.0],
    "A_AC_N_SAFE_EXIT": [27.0, 3.0],
    "B_AB_E_SAFE_EXIT": [80.0, 12.5],
    "B_BD_N_SAFE_EXIT": [80.0, 30.5],
    "C_CD_W_SAFE_EXIT": [31.0, 73.0],
    "C_AC_S_SAFE_EXIT": [41.0, 73.0],
    "D_CD_E_SAFE_EXIT": [74.0, 73.0],
    "D_BD_S_SAFE_EXIT": [80.0, 73.0],
}
BRIDGE_SAFE_EXIT_REGIONS = {
    "CAMPUS_A_SAFE_ENTRY": [19.5, 14.0, 30.0, 17.0],
    "CAMPUS_A_SAFE_VERTICAL": [19.5, 4.0, 22.5, 17.0],
    "CAMPUS_A_SAFE_BRANCHES": [13.0, 1.0, 29.0, 7.0],
    "CAMPUS_B_AB_SAFE_SPUR": [70.4, 11.0, 82.0, 14.0],
    "CAMPUS_B_BD_SAFE_SPUR": [70.4, 29.0, 82.0, 32.0],
    "CAMPUS_C_SAFE_ENTRY": [28.4, 46.0, 38.0, 49.0],
    "CAMPUS_C_SAFE_VERTICAL": [35.0, 47.0, 38.0, 71.5],
    "CAMPUS_C_SAFE_BRANCHES": [29.0, 68.5, 42.0, 76.0],
    "CAMPUS_D_CD_SAFE_VERTICAL": [49.5, 43.0, 52.5, 61.5],
    "CAMPUS_D_SAFE_CROSS": [49.5, 58.5, 80.0, 61.5],
    "CAMPUS_D_BD_SAFE_ENTRY": [69.0, 43.0, 80.0, 46.0],
    "CAMPUS_D_SAFE_VERTICAL": [77.1, 43.0, 80.0, 71.5],
    "CAMPUS_D_SAFE_BRANCHES": [72.0, 68.5, 82.0, 76.0],
}


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
        status="Authored four-bay 16-AMR v2 campus; RouteBook preflight passed, renderer and physical runtime qualification pending",
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
        result["amr_allowed_regions"].append({
            "id": f"{bay}_PACK_STAGING_SPUR",
            "rect": _shift_rect(
                PACK_STAGING_SPUR_LOCAL if bay in {"A", "B"} else PACK_STAGING_SPUR_NORTH_LOCAL,
                offset,
            ),
            "purpose": "post-drop PACK_IN egress and wait spur outside the main eastbound intersection",
        })
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
        ("H_AB", [28.5, 11.0, 51.0, 17.0], "east/west paired lanes between south bays"),
        ("H_CD", [28.5, 43.0, 51.0, 49.0], "east/west paired lanes between north bays"),
        ("V_AC", [31.4, 30.5, 36.0, 44.5], "single-reservation north/south link on the clear east perimeter"),
        ("V_BD", [73.4, 30.5, 78.0, 44.5], "single-reservation north/south link on the clear east perimeter"),
    ]
    result["amr_allowed_regions"].extend(
        {"id": f"CAMPUS_{name}", "rect": rect, "purpose": purpose}
        for name, rect, purpose in bridges
    )
    result["amr_allowed_regions"].extend([
        {"id": "CAMPUS_V_AC_S_APPROACH", "rect": [28.5, 26.0, 36.0, 32.4], "purpose": "clear south-bay east-spine bridge approach"},
        {"id": "CAMPUS_V_AC_N_APPROACH", "rect": [28.5, 43.0, 36.0, 49.0], "purpose": "clear north-bay east-spine bridge approach"},
        {"id": "CAMPUS_V_BD_S_APPROACH", "rect": [70.5, 26.0, 78.0, 32.4], "purpose": "clear south-bay east-spine bridge approach"},
        {"id": "CAMPUS_V_BD_N_APPROACH", "rect": [70.5, 43.0, 78.0, 49.0], "purpose": "clear north-bay east-spine bridge approach"},
    ])
    result["amr_allowed_regions"].extend([
        {"id": "CAMPUS_H_AB_RESERVATION", "rect": [32.0, 13.0, 47.0, 16.8], "purpose": "atomic shared bridge reservation overlay"},
        {"id": "CAMPUS_H_CD_RESERVATION", "rect": [32.0, 45.0, 47.0, 48.8], "purpose": "atomic shared bridge reservation overlay"},
        {"id": "CAMPUS_V_AC_RESERVATION", "rect": [33.0, 31.4, 35.8, 44.6], "purpose": "atomic shared bridge reservation overlay including both gate approaches"},
        {"id": "CAMPUS_V_BD_RESERVATION", "rect": [75.0, 31.4, 77.8, 44.6], "purpose": "atomic shared bridge reservation overlay including both gate approaches"},
    ])
    result["amr_allowed_regions"].extend(
        {"id": name, "rect": deepcopy(rect), "purpose": "atomic shared-junction policy overlay"}
        for name, rect in SHARED_JUNCTION_REGIONS.items()
    )
    result["amr_allowed_regions"].extend(
        {"id": name, "rect": deepcopy(rect), "purpose": "off-route vertical bridge queue access envelope"}
        for name, rect in BRIDGE_QUEUE_ACCESS_REGIONS.items()
    )
    result["amr_allowed_regions"].extend(
        {"id": name, "rect": deepcopy(rect), "purpose": "dedicated post-bridge safe-exit access envelope"}
        for name, rect in BRIDGE_SAFE_EXIT_REGIONS.items()
    )
    result["blocked_aisles"] = blocked_aisles
    result["blocked_aisle"] = deepcopy(blocked_aisles[0])
    result["blocked_aisle"]["compatibility_scope"] = "A-bay compatibility view; blocked_aisles is authoritative for the campus"
    result["unchanged_contracts"] = [
        "each bay preserves all 19 source equipment footprints and the original AMR lane geometry",
        "collision, evaluation, localization and reservation radii are unchanged",
        "inter-bay links use gated main/east-spine continuations and do not cross a raw-material station centre",
    ]
    result["operational_staging"] = [
        {
            "id": f"{bay}_PACK_WAIT",
            "xy": deepcopy(PACK_WAIT_POINTS[bay]),
            "purpose": "post-drop egress wait outside PACK_IN; not a fleet home or process-capacity station",
        }
        for bay, offset in BAY_OFFSETS.items()
    ]
    result["operational_staging"].extend(
        {
            "id": f"{bay}_{name}",
            "xy": _shift_point(point, offset),
            "purpose": "shared local post-drop standby; degree-one route endpoint, not a fleet home",
        }
        for bay, offset in BAY_OFFSETS.items()
        for name, point in LOCAL_STAGING_POINTS.items()
    )
    result["bridge_queues"] = [
        {
            "id": queue_id,
            "xy": deepcopy(point),
            "purpose": "mandatory off-route stop before inter-bay bridge admission; floor marker only",
        }
        for queue_id, point in BRIDGE_QUEUES.items()
    ]
    result["bridge_safe_exits"] = [
        {
            "id": exit_id,
            "xy": deepcopy(point),
            "purpose": "dedicated post-bridge release buffer; excluded from process staging, fleet homes and task destinations",
        }
        for exit_id, point in BRIDGE_SAFE_EXITS.items()
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
        "pending": "USD scene transformation, renderer coverage and multi-robot physical evidence",
    }
    result["production_domain"] = deepcopy(source["production_domain"])
    result["production_domain"]["scope"] = (
        "Four authored semiconductor final-test/inspection/pack bays in an 82x76m v2 campus; "
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

    for bay, offset in BAY_OFFSETS.items():
        east_start = f"{bay}_LOW_E_ES"
        east_end = f"{bay}_LOW_E_EN"
        pack_in = f"{bay}_PACK_IN"
        egress = f"{bay}_PACK_EGRESS"
        wait = f"{bay}_PACK_WAIT"
        result["nodes"][egress] = _shift_point(PACK_EGRESS_LOCAL, offset)
        result["nodes"][wait] = deepcopy(PACK_WAIT_POINTS[bay])
        result["directed_edges"].remove([east_start, east_end])
        result["directed_edges"].extend([
            [east_start, egress],
            [egress, east_end],
            [pack_in, egress],
        ])
        result["edges"].append([egress, wait])

        source_wait = f"{bay}_SOURCE_WAIT"
        cell_wait = f"{bay}_CELL_WAIT"
        result["nodes"][source_wait] = _shift_point(LOCAL_STAGING_POINTS["SOURCE_WAIT"], offset)
        result["nodes"][cell_wait] = _shift_point(LOCAL_STAGING_POINTS["CELL_WAIT"], offset)
        result["edges"].extend([
            [f"{bay}_HIGH_E_W", source_wait],
            [f"{bay}_CI", cell_wait],
        ])

    result["staging_after_drop"] = {}
    for bay in BAY_OFFSETS:
        result["staging_after_drop"].update({
            **{f"{bay}_S{slot}": f"{bay}_SOURCE_WAIT" for slot in range(1, 4)},
            f"{bay}_CI": f"{bay}_CELL_WAIT",
            f"{bay}_AOI_IN": f"{bay}_CELL_WAIT",
            f"{bay}_OUT": f"{bay}_CELL_WAIT",
            f"{bay}_PACK_IN": f"{bay}_PACK_WAIT",
        })

    bridge_regions = {
        "CAMPUS_H_AB_RESERVATION": [32.0, 13.0, 47.0, 16.8],
        "CAMPUS_H_CD_RESERVATION": [32.0, 45.0, 47.0, 48.8],
        "CAMPUS_V_AC_RESERVATION": [33.0, 31.4, 35.8, 44.6],
        "CAMPUS_V_BD_RESERVATION": [75.0, 31.4, 77.8, 44.6],
        **deepcopy(SHARED_JUNCTION_REGIONS),
    }
    result["policy_region_overrides"].update(bridge_regions)
    for region_id in bridge_regions:
        result["operations_bottlenecks"].append({
            "id": region_id.lower().replace("_", "-"),
            "region_id": region_id,
            "release_policy": "first_route_point_outside_region",
        })

    connector_nodes = {
        "AB_E_ACCESS": [32.0, 12.5],
        "AB_E_QUEUE": deepcopy(BRIDGE_QUEUES["AB_E_QUEUE"]),
        "AB_E_W_GATE": [31.0, 14.0], "AB_E_W_IN": [33.0, 14.0],
        "AB_E_E_IN": [46.0, 14.0], "AB_E_E_GATE": [48.0, 14.0],
        "AB_W_E_GATE": [48.0, 15.5], "AB_W_E_IN": [46.0, 15.5],
        "AB_W_W_IN": [33.0, 15.5], "AB_W_W_GATE": [31.0, 15.5],
        "AB_W_QUEUE": deepcopy(BRIDGE_QUEUES["AB_W_QUEUE"]),
        "AB_W_RETURN": [57.0, 15.5], "AB_W_P1": [54.0, 15.5], "AB_W_ENTRY": [51.0, 15.5],
        "CD_E_QUEUE": deepcopy(BRIDGE_QUEUES["CD_E_QUEUE"]),
        "CD_E_ACCESS": [27.0, 41.5],
        "CD_E_ENTRY": [28.5, 44.5],
        "CD_E_W_GATE": [31.0, 46.0], "CD_E_W_IN": [33.0, 46.0],
        "CD_E_E_IN": [46.0, 46.0], "CD_E_E_GATE": [48.0, 46.0],
        "CD_W_E_GATE": [48.0, 47.5], "CD_W_E_IN": [46.0, 47.5],
        "CD_W_W_IN": [33.0, 47.5], "CD_W_W_GATE": [31.0, 47.5],
        "CD_W_QUEUE": deepcopy(BRIDGE_QUEUES["CD_W_QUEUE"]),
        "CD_W_RETURN": [57.0, 47.5], "CD_W_P1": [54.0, 47.5], "CD_W_ENTRY": [51.0, 47.5],
        "AC_S_GATE": [33.0, 31.5], "AC_S_IN": [34.5, 33.0],
        "AC_N_IN": [34.5, 43.0], "AC_N_GATE": [33.0, 44.5],
        "AC_N_ACCESS": [35.5, 38.5],
        "BD_S_GATE": [75.0, 31.5], "BD_S_IN": [76.5, 33.0],
        "BD_N_IN": [76.5, 43.0], "BD_N_GATE": [75.0, 44.5],
        **{name: deepcopy(point) for name, point in BRIDGE_QUEUES.items() if name.startswith(("AC_", "BD_"))},
    }
    result["nodes"].update(connector_nodes)
    connector_paths = {
        "AB_E": ["A_LOW_E_EN", "AB_E_ACCESS", "AB_E_QUEUE", "AB_E_W_GATE", "AB_E_W_IN", "AB_E_E_IN", "AB_E_E_GATE", "B_LOW_E_W"],
        "AB_W": ["B_LOW_W_C", "AB_W_QUEUE", "AB_W_RETURN", "AB_W_P1", "AB_W_ENTRY", "AB_W_E_GATE", "AB_W_E_IN", "AB_W_W_IN", "AB_W_W_GATE", "A_LOW_W_EN"],
        "CD_E": ["C_PACK_EGRESS", "CD_E_ACCESS", "CD_E_QUEUE", "CD_E_ENTRY", "CD_E_W_GATE", "CD_E_W_IN", "CD_E_E_IN", "CD_E_E_GATE", "D_LOW_E_W"],
        "CD_W": ["D_LOW_W_C", "CD_W_QUEUE", "CD_W_RETURN", "CD_W_P1", "CD_W_ENTRY", "CD_W_E_GATE", "CD_W_E_IN", "CD_W_W_IN", "CD_W_W_GATE", "C_LOW_W_EN"],
        "AC_N": ["A_HIGH_E_EN", "AC_S_QUEUE", "AC_S_GATE", "AC_S_IN", "AC_N_IN", "AC_N_GATE", "C_LOW_E_EN"],
        "AC_S": ["C_LOW_W_EN", "AC_N_QUEUE", "AC_N_ACCESS", "AC_N_GATE", "AC_N_IN", "AC_S_IN", "AC_S_GATE", "A_HIGH_W_EN"],
        "BD_N": ["B_HIGH_E_EN", "BD_S_QUEUE", "BD_S_GATE", "BD_S_IN", "BD_N_IN", "BD_N_GATE", "D_LOW_E_EN"],
        "BD_S": ["D_LOW_W_EN", "BD_N_QUEUE", "BD_N_GATE", "BD_N_IN", "BD_S_IN", "BD_S_GATE", "B_HIGH_W_EN"],
    }
    for path in connector_paths.values():
        result["directed_edges"].extend([list(edge) for edge in zip(path, path[1:])])
    result["directed_edges"].append(["AC_N_GATE", "CD_E_ACCESS"])

    safe_exit_access_nodes = {
        "A_SAFE_ENTRY": [21.0, 15.5],
        "A_SAFE_HUB": [21.0, 5.5],
        "C_SAFE_ENTRY": [36.5, 47.5],
        "C_SAFE_HUB": [36.5, 70.0],
        "D_CD_E_SAFE_ENTRY": [51.0, 60.0],
        "D_CD_E_SAFE_TOP": [75.0, 60.0],
        "D_CD_E_SAFE_BRANCH": [75.0, 70.0],
        "D_BD_S_SAFE_ENTRY": [80.0, 44.5],
        "D_BD_S_SAFE_BRANCH": [80.0, 70.0],
    }
    result["nodes"].update(safe_exit_access_nodes)
    result["nodes"].update(deepcopy(BRIDGE_SAFE_EXITS))
    result["edges"].extend([
        ["A_HIGH_W_EN", "A_LOW_W_EN"],
        ["A_LOW_W_EN", "A_SAFE_ENTRY"],
        ["A_SAFE_ENTRY", "A_SAFE_HUB"],
        ["A_SAFE_HUB", "A_AB_W_SAFE_EXIT"],
        ["A_SAFE_HUB", "A_AC_N_SAFE_EXIT"],
        ["B_LOW_E_EN", "B_AB_E_SAFE_EXIT"],
        ["B_HIGH_W_EN", "B_BD_N_SAFE_EXIT"],
        ["C_LOW_W_EN", "C_SAFE_ENTRY"],
        ["C_SAFE_ENTRY", "C_SAFE_HUB"],
        ["C_SAFE_HUB", "C_CD_W_SAFE_EXIT"],
        ["C_SAFE_HUB", "C_AC_S_SAFE_EXIT"],
        ["D_LOW_E_W", "D_CD_E_SAFE_ENTRY"],
        ["D_CD_E_SAFE_ENTRY", "D_CD_E_SAFE_TOP"],
        ["D_CD_E_SAFE_TOP", "D_CD_E_SAFE_BRANCH"],
        ["D_CD_E_SAFE_BRANCH", "D_CD_E_SAFE_EXIT"],
        ["D_LOW_E_EN", "D_BD_S_SAFE_ENTRY"],
        ["D_BD_S_SAFE_ENTRY", "D_BD_S_SAFE_BRANCH"],
        ["D_BD_S_SAFE_BRANCH", "D_BD_S_SAFE_EXIT"],
    ])
    holdable_nodes = [
        *(f"{bay}_P{slot}" for bay in BAY_OFFSETS for slot in range(1, 5)),
        *(f"{bay}_PACK_WAIT" for bay in BAY_OFFSETS),
        *(f"{bay}_{name}" for bay in BAY_OFFSETS for name in LOCAL_STAGING_POINTS),
        *BRIDGE_QUEUES,
        *BRIDGE_SAFE_EXITS,
    ]
    resource_exclusions = {*BRIDGE_QUEUES, "AB_E_ACCESS", "CD_E_ACCESS"}
    result["bridge_admission"] = {
        "policy": "atomic_route_leg_resources",
        "resources": {link: sorted(node for node in connector_nodes
                                    if node.startswith(link + "_") and node not in resource_exclusions)
                      for link in ("AB", "CD", "AC", "BD")},
        "queue_nodes": list(BRIDGE_QUEUES),
        "holdable_nodes": holdable_nodes,
        "safe_exit_nodes": {
            "AB_E_QUEUE": "B_AB_E_SAFE_EXIT",
            "AB_W_QUEUE": "A_AB_W_SAFE_EXIT",
            "CD_E_QUEUE": "D_CD_E_SAFE_EXIT",
            "CD_W_QUEUE": "C_CD_W_SAFE_EXIT",
            "AC_S_QUEUE": "C_AC_S_SAFE_EXIT",
            "AC_N_QUEUE": "A_AC_N_SAFE_EXIT",
            "BD_S_QUEUE": "D_BD_S_SAFE_EXIT",
            "BD_N_QUEUE": "B_BD_N_SAFE_EXIT",
        },
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


def build_wide_bridge_layout(base: Mapping[str, Any]) -> dict[str, Any]:
    """Use the full authored AB/CD road envelopes for separated opposing lanes."""
    result = build_campus_layout(base)
    result.update(
        revision=CAMPUS_REVISION,
        status=(
            "Authored four-bay 16-AMR campus with 6m AB/CD paired-road envelopes; "
            "RouteBook preflight passed, renderer and physical runtime qualification pending"
        ),
    )
    for region in result["amr_allowed_regions"]:
        if region["id"] == "CAMPUS_AB_E_QUEUE_ACCESS":
            region["rect"] = [30.0, 8.0, 33.5, 12.5]
            region["purpose"] = "widened queue-to-eastbound-lane approach envelope"
    result["traffic_lanes"].extend([
        {"id": "CAMPUS_AB_E", "from_xy": [28.5, 12.5], "to_xy": [51.0, 12.5]},
        {"id": "CAMPUS_AB_W", "from_xy": [51.0, 15.5], "to_xy": [28.5, 15.5]},
        {"id": "CAMPUS_CD_E", "from_xy": [28.5, 44.5], "to_xy": [51.0, 44.5]},
        {"id": "CAMPUS_CD_W", "from_xy": [51.0, 47.5], "to_xy": [28.5, 47.5]},
    ])
    result["wide_bridge_geometry"] = {
        "road_envelope_width_m": 6.0,
        "lane_center_separation_m": 3.0,
        "endpoint_connectors": {
            "AB_E": [[28.5, 12.5], [51.0, 12.5]],
            "AB_W": [[51.0, 15.5], [28.5, 15.5]],
            "CD_E": [[28.5, 44.5], [51.0, 44.5]],
            "CD_W": [[51.0, 47.5], [28.5, 47.5]],
        },
        "approach_overrides": {"AB_E_QUEUE": [30.0, 8.0, 33.5, 12.5]},
        "safe_exit_overrides": {"CD_E_QUEUE": {"node": "D_LOW_E_P1", "xy": [54.0, 44.5]}},
        "scope": "authored selectable geometry variant; no throughput or physical-runtime claim",
    }
    result["layout_provenance"]["geometry_rule"] = (
        "four immutable bays with AB/CD opposing centerlines separated by 3m inside existing 6m road envelopes"
    )
    return result


def build_wide_bridge_routing(base: Mapping[str, Any]) -> dict[str, Any]:
    """Split AB/CD resources only after giving each direction a separate swept corridor."""
    result = build_campus_routing(base)
    result["scope"] = (
        "Four-bay 16-AMR authored route graph with separated AB/CD opposing lanes; "
        "reservation and physical runtime qualification pending"
    )
    result["base_layout"] = "configs/factory-semiconductor-campus16-wide-bridges/layout.json"

    for node in ("AB_E_W_GATE", "AB_E_W_IN", "AB_E_E_IN", "AB_E_E_GATE"):
        result["nodes"][node][1] = 12.5
    for node in ("CD_E_W_GATE", "CD_E_W_IN", "CD_E_E_IN", "CD_E_E_GATE"):
        result["nodes"][node][1] = 44.5

    admission = result["bridge_admission"]
    admission["safe_exit_nodes"]["CD_E_QUEUE"] = "D_LOW_E_P1"
    admission["holdable_nodes"].append("D_LOW_E_P1")
    shared_resources = {
        link: admission["resources"][link] for link in ("AC", "BD")
    }
    resources = {}
    excluded = set(admission["queue_nodes"]) | {"AB_E_ACCESS", "CD_E_ACCESS"}
    for link in ("AB_E", "AB_W", "CD_E", "CD_W"):
        resources[link] = sorted(
            node for node in result["nodes"]
            if node.startswith(link + "_") and node not in excluded
        )
    resources.update(shared_resources)
    admission["resources"] = resources
    radius = result["authored_allowances"]["reservation_radius_including_tracking_m"]
    tile_size = result["authored_allowances"]["tile_size_m"]
    result["wide_bridge_geometry"] = {
        "road_envelope_width_m": 6.0,
        "lane_center_separation_m": 3.0,
        "minimum_center_separation_m": 2 * radius,
        "reservation_radius_m": radius,
        "tile_size_m": tile_size,
        "directional_resources": ["AB_E", "AB_W", "CD_E", "CD_W"],
        "shared_resources_unchanged": ["AC", "BD"],
        "safe_exit_overrides": {"CD_E_QUEUE": {"node": "D_LOW_E_P1", "xy": [54.0, 44.5]}},
        "scope": "selection candidate; safety guards and vehicle envelope are unchanged",
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
    pack_spur_conflicts = {
        "MAINT_TOOL_STORAGE_04", "MAINT_ROLLING_BENCH_04",
        "MAINT_TOOL_STORAGE_05", "MAINT_ROLLING_BENCH_05",
        "MAINT_PARTS_CART_04", "MAINT_PARTS_CART_05",
    }
    queue_access_conflicts = {
        "C": {"MAINT_TOOL_STORAGE_06", "MAINT_ROLLING_BENCH_06"},
        "D": {"MAINT_TOOL_STORAGE_06", "MAINT_ROLLING_BENCH_06"},
    }
    safe_exit_conflicts = {
        "A": {
            "MAINT_TOOL_STORAGE_01", "MAINT_ROLLING_BENCH_01",
            "MAINT_TOOL_STORAGE_02", "MAINT_ROLLING_BENCH_02",
            "MAINT_PARTS_CART_01", "MAINT_PARTS_CART_02",
        },
        "B": {"EAST_SERVICE_TOOLS_LOW", "EAST_SERVICE_CART_LOW"},
        "C": {"EAST_SERVICE_TOOLS_MID", "EAST_SERVICE_TOOLS_HIGH"},
        "D": {"EAST_SERVICE_TOOLS_MID", "EAST_SERVICE_TOOLS_HIGH"},
    }
    excluded = []
    for bay, offset in BAY_OFFSETS.items():
        for fixture in source["environment"]["fixtures"]:
            if (fixture["id"] in excluded_local[bay] or fixture["id"] in pack_spur_conflicts
                    or fixture["id"] in queue_access_conflicts.get(bay, set())
                    or fixture["id"] in safe_exit_conflicts.get(bay, set())):
                excluded.append({
                    "id": _prefix(fixture["id"], bay),
                    "reason": (
                        "translated native fixture bounds plus 0.1m clearance intersect an authored campus bridge"
                        if fixture["id"] in excluded_local[bay]
                        else (
                            "translated native fixture bounds plus 0.1m clearance intersect the PACK staging spur"
                            if fixture["id"] in pack_spur_conflicts
                            else (
                                "translated native fixture bounds plus 0.1m clearance intersect vertical bridge queue access"
                                if fixture["id"] in queue_access_conflicts.get(bay, set())
                                else "translated native fixture bounds plus 0.1m clearance intersect dedicated bridge safe-exit access"
                            )
                        )
                    ),
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
    for camera in result["cameras"]:
        if camera["id"] in {"west_material_flow", "cell_process", "east_shipping", "packing_conveyor"}:
            for key in ("eye_xyz_m", "target_xyz_m"):
                camera[key][0] += BAY_OFFSETS["A"][0]
                camera[key][1] += BAY_OFFSETS["A"][1]
    overview = next(camera for camera in result["cameras"] if camera["id"] == "portfolio_overview")
    overview.update(
        eye_xyz_m=[41.0, -37.0, 82.0],
        target_xyz_m=[41.0, 38.0, 0.7],
        focal_length_mm=22,
        purpose="four-bay semiconductor campus and 16-AMR material flow; authored camera, capture pending",
    )
    result["campus_scope"] = (
        "Camera authoring for the 82x76m v2 campus and 16-AMR flow; successful GPU capture remains pending"
    )
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


def build_wide_bridge_campus_bundle(base_dir: str | Path) -> dict[str, dict[str, Any]]:
    root = Path(base_dir)
    result = build_campus_bundle(root)
    result["layout.json"] = build_wide_bridge_layout(_read(root / "layout.json"))
    result["g3-routing.json"] = build_wide_bridge_routing(_read(root / "g3-routing.json"))
    fixtures = {
        fixture["id"]: fixture
        for fixture in result["visual-assets.json"]["environment"]["fixtures"]
    }
    fixtures["A_MAINT_TOOL_STORAGE_06"]["center_xy"] = [29.6, 7.5]
    fixtures["A_MAINT_ROLLING_BENCH_06"]["center_xy"] = [29.3, 10.45]
    return result


__all__ = [
    "BAY_OFFSETS", "CAMPUS_BOUNDS", "CAMPUS_REVISION",
    "build_campus_bundle", "build_campus_layout", "build_wide_bridge_campus_bundle",
    "build_wide_bridge_layout", "build_wide_bridge_routing",
    "build_campus_routing", "build_charging", "build_fleet_profile", "build_g2", "build_process_network",
    "build_render_profile", "build_visual_assets",
]
