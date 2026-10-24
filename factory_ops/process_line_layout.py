"""Build scale-fleet configs with authored TEST/AOI/PACK transport anchors."""

from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
from typing import Any, Mapping

from .fleet_scale_layout import build_scale_bundle


LINE_POINTS = {
    "AOI_IN": [4.0, 13.5],
    "AOI_OUT": [7.0, 13.5],
    "PACK_IN": [27.0, 7.5],
    "PACK_OUT": [27.0, 19.5],
}


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def _split_arc(edges: list[list[str]], left: str, right: str, middle: str) -> None:
    try:
        index = edges.index([left, right])
    except ValueError as error:
        raise ValueError(f"missing directed lane arc {left}->{right}") from error
    edges[index:index + 1] = [[left, middle], [middle, right]]


def add_process_line_routing(routing: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(routing))
    result["nodes"].update(deepcopy(LINE_POINTS))
    result["edges"].extend([
        ["S2", "AOI_IN"], ["S2", "AOI_OUT"],
    ])
    _split_arc(result["directed_edges"], "LOW_W_EN", "LOW_W_ES", "PACK_IN")
    _split_arc(result["directed_edges"], "HIGH_E_ES", "HIGH_E_EN", "PACK_OUT")
    result["process_line_routing"] = {
        "scope": "Authored functional TEST/AOI/PACK handoff anchors; no equipment certification or robot runtime claim",
        "TEST_IN": {"anchor": "CI", "equipment_context": "PROCESS_1 test handler input buffer"},
        "TEST_OUT": {"anchor": "CO", "equipment_context": "PROCESS_1/PROCESS_2 logical test output"},
        "AOI_IN": {"anchor": "AOI_IN", "equipment_context": "S2 side handoff bay feeding the authored BENCH_A2/BENCH_B2 logical AOI stage"},
        "AOI_OUT": {"anchor": "AOI_OUT", "equipment_context": "S2 side handoff bay releasing the authored logical AOI/marking stage"},
        "PACK_IN": {"anchor": "PACK_IN", "equipment_context": "FG_LOW packaging approach from the east-spine lower westbound lane"},
        "PACK_OUT": {"anchor": "PACK_OUT", "equipment_context": "FG_HIGH finished tray release to the east-spine upper eastbound lane"},
        "geometry_reason": "AOI stops use the S2 side bay outside the west shared-corridor boundary; PACK anchors split existing east-spine arcs and avoid the occupied parking annex without changing collision or reservation radii",
    }
    result["scope"] = result["scope"].replace("route and parking-annex", "route, parking-annex and process-line")
    return result


def line_process_config(base: Mapping[str, Any]) -> dict[str, Any]:
    result = deepcopy(dict(base))
    aliases = {
        "TEST": ("CI", "CO"),
        "AOI": ("AOI_IN", "AOI_OUT"),
        "PACK": ("PACK_IN", "PACK_OUT"),
    }
    for cell in result["cells"]:
        cell["input_station"], cell["output_station"] = aliases[cell["id"]]
    result["scope"] = (
        "Authored CPU TEST/AOI/PACK ledger bound to authored route anchors; "
        "no equipment certification, robot physics, sensor or measured process-time claim"
    )
    result["provenance"]["equipment_context"] = {
        "TEST": "PROCESS_1 handler with CI/CO logical handoffs",
        "AOI": "BENCH_A2 and BENCH_B2 inspection/marking context",
        "PACK": "FG_LOW and FG_HIGH packaging/staging context",
    }
    return result


def build_line_bundle(config_root: str | Path, fleet_count: int) -> dict[str, dict[str, Any]]:
    root = Path(config_root)
    bundle = build_scale_bundle(root / "factory-semiconductor-dual-lane", fleet_count)
    bundle["g3-routing.json"] = add_process_line_routing(bundle["g3-routing.json"])
    bundle["process-line.json"] = line_process_config(_read(root / "semiconductor-process-line.json"))
    return bundle


__all__ = ["LINE_POINTS", "add_process_line_routing", "build_line_bundle", "line_process_config"]
