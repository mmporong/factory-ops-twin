"""Project estimated map poses onto authored traffic geometry.

The conservative circle is the qualified evaluation radius plus the authored
localization margin.  No simulator pose, contact, or object ID is consumed.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Sequence, TypedDict

if TYPE_CHECKING:
    from factory_ops.traffic import TrafficController


class ProjectedObservation(TypedDict):
    occupied_resources: tuple[str, ...]
    clear_exits: tuple[str, ...]


def _radius(value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError("planning radius must be finite and positive")
    return float(value)


def circle_intersects_polygon(xy: Sequence[float], radius_m: float, polygon: Sequence[Sequence[float]]) -> bool:
    """Closed-boundary circle/polygon intersection, including tangency."""
    radius = _radius(radius_m)
    if len(xy) != 2 or not all(math.isfinite(float(v)) for v in xy) or len(polygon) < 3:
        raise ValueError("finite 2D pose and polygon are required")
    x, y = map(float, xy)
    inside = False
    previous = polygon[-1]
    for current in polygon:
        ax, ay = map(float, previous)
        bx, by = map(float, current)
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        fraction = min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / length2)) if length2 else 0.0
        if math.hypot(x - ax - fraction * dx, y - ay - fraction * dy) <= radius + 1e-9:
            return True
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
        previous = current
    return inside


def exit_xy(traffic: TrafficController, exit_id: str) -> tuple[float, float]:
    resource, port = exit_id.split(":", 1)
    ports = traffic.resources[resource]["ports"]
    if not isinstance(ports, dict) or not isinstance(ports.get(port), dict):
        raise ValueError("invalid traffic exit geometry")
    point = ports[port]["xy"]
    return float(point[0]), float(point[1])


def project_observation(traffic: TrafficController, pose_xy: Sequence[float], planning_radius_m: float) -> ProjectedObservation:
    radius = _radius(planning_radius_m)
    x, y = traffic._validate_xy(pose_xy, "estimated pose")
    occupied = []
    for resource_id, data in traffic.resources.items():
        polygon = data["polygon_xy"]
        if not isinstance(polygon, Sequence):
            raise ValueError("invalid traffic resource geometry")
        if circle_intersects_polygon((x, y), radius, polygon):
            occupied.append(resource_id)
    return {
        "occupied_resources": tuple(sorted(occupied)),
        "clear_exits": tuple(sorted(exit_id for exit_id in traffic.valid_exits
                                     if math.dist((x, y), exit_xy(traffic, exit_id)) > radius + 1e-9)),
    }
