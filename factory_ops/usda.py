"""Minimal text USDA authoring for the validated graybox layout."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re

from .layout import Layout, LayoutError, Rect


def export_usda(layout: Layout, output_path: str | Path) -> dict[str, object]:
    """Write a dependency-free OpenUSD text layer for later Isaac validation."""
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    authored_ids = ["Floor", "FloorVisual"] + [item["id"] for item in layout.data["physical_obstacles"]] + list(layout.stations) + list(layout.pockets)
    prim_names = [_prim_name(identifier) for identifier in authored_ids]
    if len(prim_names) != len(set(prim_names)):
        raise LayoutError("authored IDs collide after conversion to safe USD prim names")
    revision = _quote(str(layout.data.get("revision", "unknown")))
    lines = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "World"',
        '    upAxis = "Z"',
        "    metersPerUnit = 1",
        ")",
        "",
        'def Xform "World"',
        "{",
        f"    custom string sourceRevision = {revision}",
        '    custom string validationStatus = "authored text only; pxr and Isaac Sim validation not performed"',
        "",
    ]
    lines.extend(_cube("FloorVisual", Rect(0.0, 0.0, 40.0, 24.0), 0.1, -0.05, "floor_visual", collision=False, indent=4))
    lines.append("")
    lines.extend(_collision_plane("Floor", indent=4))
    lines.append("")
    lines.extend(_scope_open("Obstacles", 4))
    for obstacle in layout.data["physical_obstacles"]:
        rect = Rect.from_value(obstacle["rect"], f"obstacle {obstacle['id']}")
        height = float(obstacle["height_m"])
        if not math.isfinite(height) or height <= 0:
            raise LayoutError(f"obstacle {obstacle['id']} has invalid height")
        lines.extend(_cube(_prim_name(obstacle["id"]), rect, height, height / 2, obstacle["id"], collision=True, indent=8))
    lines.extend(_scope_close(4))
    lines.append("")
    lines.extend(_scope_open("Markers", 4))
    for marker_id, xy in {**layout.stations, **layout.pockets}.items():
        lines.extend(_marker(_prim_name(marker_id), marker_id, xy, indent=8))
    lines.extend(_scope_close(4))
    lines.append("}")
    lines.append("")
    text = "\n".join(lines)
    output.write_text(text, encoding="utf-8", newline="\n")
    return {
        "path": str(output),
        "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "default_prim": "World",
        "up_axis": "Z",
        "meters_per_unit": 1,
        "static_collider_count": 20,
        "obstacle_collider_count": 19,
        "floor_collision_geometry": "invisible static Z-axis Plane at z=0; infinite physics surface",
        "floor_visual_geometry": "non-colliding 40x24x0.1 m Cube slab ending at z=0",
        "bounded_operations_scope": "40x24 m authored occupancy map; the physics plane is intentionally infinite",
        "non_colliding_marker_count": len(layout.stations) + len(layout.pockets),
        "robot_assets": 0,
        "validation": "OpenUSD text authored; pxr/Isaac Sim parsing and collision behavior not validated",
    }


def _cube(name: str, rect: Rect, height: float, z: float, marker_id: str, *, collision: bool, indent: int) -> list[str]:
    width = rect.xmax - rect.xmin
    depth = rect.ymax - rect.ymin
    x = (rect.xmin + rect.xmax) / 2
    y = (rect.ymin + rect.ymax) / 2
    pad = " " * indent
    header = [f'{pad}def Cube "{name}"']
    if collision:
        header = [f'{pad}def Cube "{name}" (',
                  f'{pad}    prepend apiSchemas = ["PhysicsCollisionAPI"]', f"{pad})"]
    return [
        *header,
        f"{pad}{{",
        f"{pad}    custom string markerId = {_quote(marker_id)}",
        f"{pad}    bool physics:collisionEnabled = {'true' if collision else 'false'}",
        f"{pad}    double size = 2",
        f"{pad}    double3 xformOp:scale = ({_num(width / 2)}, {_num(depth / 2)}, {_num(height / 2)})",
        f"{pad}    double3 xformOp:translate = ({_num(x)}, {_num(y)}, {_num(z)})",
        f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        f"{pad}}}",
    ]


def _collision_plane(name: str, *, indent: int) -> list[str]:
    pad = " " * indent
    return [
        f'{pad}def Plane "{name}" (',
        f'{pad}    prepend apiSchemas = ["PhysicsCollisionAPI"]',
        f"{pad})",
        f"{pad}{{",
        f'{pad}    uniform token axis = "Z"',
        f"{pad}    float3[] extent = [(-1, -1, 0), (1, 1, 0)]",
        f'{pad}    custom string markerId = "floor"',
        f'{pad}    custom string physicsScope = "infinite plane; bounded operation is enforced by the 40x24 m occupancy map"',
        f"{pad}    bool physics:collisionEnabled = true",
        f'{pad}    token visibility = "invisible"',
        f"{pad}    quatd xformOp:orient = (1, 0, 0, 0)",
        f"{pad}    double3 xformOp:scale = (1, 1, 1)",
        f"{pad}    double3 xformOp:translate = (0, 0, 0)",
        f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]',
        f"{pad}}}",
    ]


def _marker(name: str, marker_id: str, xy: tuple[float, float], *, indent: int) -> list[str]:
    x, y = xy
    if not all(math.isfinite(value) for value in (x, y)):
        raise LayoutError(f"marker {marker_id} contains non-finite coordinates")
    pad = " " * indent
    return [
        f'{pad}def Xform "{name}"',
        f"{pad}{{",
        f"{pad}    custom string markerId = {_quote(marker_id)}",
        f"{pad}    double3 xformOp:translate = ({_num(x)}, {_num(y)}, 0)",
        f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate"]',
        f"{pad}}}",
    ]


def _scope_open(name: str, indent: int) -> list[str]:
    pad = " " * indent
    return [f'{pad}def Scope "{name}"', f"{pad}{{"]


def _scope_close(indent: int) -> list[str]:
    return [" " * indent + "}"]


def _prim_name(value: str) -> str:
    if not isinstance(value, str) or not value:
        raise LayoutError("USD marker and collider IDs must be non-empty strings")
    name = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if name[0].isdigit():
        name = "_" + name
    return name


def _quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def _num(value: float) -> str:
    if not math.isfinite(value):
        raise LayoutError("USD numeric value must be finite")
    if value == int(value):
        return str(int(value))
    return format(value, ".12g")
