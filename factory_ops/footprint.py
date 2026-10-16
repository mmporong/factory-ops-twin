"""Qualify measured robot collision bounds against the authored raw grid.

The qualification uses a circular XY envelope so the result remains valid for
every yaw angle.  It is intentionally separate from the layout module's
historic 29x29 planning probe.
"""

from __future__ import annotations

import math
from typing import Any, Sequence

from factory_ops.layout import Layout, LayoutError


_SAMPLE_LIMIT = 8


def qualify_footprint(
    layout: Layout,
    bounds_xyz: Sequence[Sequence[float]],
    clearance_m: float = 0.2,
) -> dict[str, Any]:
    """Check a measured collision envelope at every station and pocket anchor.

    ``bounds_xyz`` is the root-local axis-aligned bound of all enabled robot
    collision geometry: ``(min_xyz, max_xyz)``.  The farthest XY corner from
    the root defines a yaw-invariant circular radius.  Clearance is then added,
    and every raw 50 mm cell touched by that circle must be free.  A touched
    cell outside the map also makes the anchor invalid.
    """

    minimum, maximum = _validated_bounds(bounds_xyz)
    clearance = _finite_number(clearance_m, "clearance_m")
    if clearance < 0:
        raise LayoutError("clearance_m must be non-negative")

    collision_radius = max(
        math.hypot(x, y)
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
    )
    inflated_radius = collision_radius + clearance
    anchors: dict[str, dict[str, Any]] = {}
    for name, xy in {**layout.stations, **layout.pockets}.items():
        bad_cells = _bad_touched_cells(layout, xy, inflated_radius)
        anchors[name] = {
            "xy": [xy[0], xy[1]],
            "valid": not bad_cells,
            "blocked_cell_count": len(bad_cells),
            "sample_bad_cells": [list(cell) for cell in bad_cells[:_SAMPLE_LIMIT]],
        }

    return {
        "passed": all(anchor["valid"] for anchor in anchors.values()),
        "method": (
            "root-local farthest-XY-corner circular yaw envelope; clearance added; "
            "all touched 50 mm raw allowed-minus-obstacles cells must be free"
        ),
        "collision_bounds_xyz": [list(minimum), list(maximum)],
        "collision_radius_m": collision_radius,
        "clearance_m": clearance,
        "inflated_radius_m": inflated_radius,
        "map_resolution_m": layout.resolution_m,
        "anchors": anchors,
    }


def _validated_bounds(
    bounds_xyz: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if isinstance(bounds_xyz, (str, bytes)) or len(bounds_xyz) != 2:
        raise LayoutError("bounds_xyz must contain [min_xyz, max_xyz]")
    points: list[tuple[float, float, float]] = []
    for point_index, point in enumerate(bounds_xyz):
        if isinstance(point, (str, bytes)) or len(point) != 3:
            raise LayoutError(f"bounds_xyz[{point_index}] must contain [x, y, z]")
        points.append(
            (
                _finite_number(point[0], f"bounds_xyz[{point_index}][0]"),
                _finite_number(point[1], f"bounds_xyz[{point_index}][1]"),
                _finite_number(point[2], f"bounds_xyz[{point_index}][2]"),
            )
        )
    minimum, maximum = points
    if any(low >= high for low, high in zip(minimum, maximum)):
        raise LayoutError("bounds_xyz must be ordered and non-degenerate on every axis")
    return minimum, maximum


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LayoutError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise LayoutError(f"{label} must be finite")
    return result


def _bad_touched_cells(
    layout: Layout,
    center_xy: tuple[float, float],
    radius_m: float,
) -> list[tuple[int, int]]:
    resolution = layout.resolution_m
    center_x, center_y = center_xy
    first_col = math.floor((center_x - radius_m) / resolution)
    last_col = math.floor((center_x + radius_m) / resolution)
    first_row = math.floor((center_y - radius_m) / resolution)
    last_row = math.floor((center_y + radius_m) / resolution)
    radius_squared = radius_m * radius_m
    tolerance = max(1.0, radius_squared) * 1e-12
    bad: list[tuple[int, int]] = []

    for row in range(first_row, last_row + 1):
        cell_ymin = row * resolution
        cell_ymax = cell_ymin + resolution
        dy = max(cell_ymin - center_y, 0.0, center_y - cell_ymax)
        for col in range(first_col, last_col + 1):
            cell_xmin = col * resolution
            cell_xmax = cell_xmin + resolution
            dx = max(cell_xmin - center_x, 0.0, center_x - cell_xmax)
            if dx * dx + dy * dy > radius_squared + tolerance:
                continue
            inside = 0 <= col < layout.width and 0 <= row < layout.height
            if not inside or not layout._raw_normal[row * layout.width + col]:
                bad.append((col, row))
    return bad
