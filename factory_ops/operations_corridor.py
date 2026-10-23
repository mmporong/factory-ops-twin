"""Authored shared-corridor contracts for the kinematic operations model."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence


Point = tuple[float, float]


def _finite_point(value: Sequence[float]) -> Point:
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or any(isinstance(item, bool) or not isinstance(item, (int, float))
                   or not math.isfinite(item) for item in value)):
        raise ValueError("corridor route points must contain two finite coordinates")
    return float(value[0]), float(value[1])


@dataclass(frozen=True, slots=True)
class CorridorSpan:
    corridor_id: str
    points: tuple[Point, ...]


@dataclass(frozen=True, slots=True)
class _Corridor:
    corridor_id: str
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, point: Point) -> bool:
        return self.xmin <= point[0] <= self.xmax and self.ymin <= point[1] <= self.ymax

    def intersects(self, start: Point, end: Point) -> bool:
        """Return whether a closed segment enters or touches the closed region."""

        lower, upper = 0.0, 1.0
        for origin, delta, minimum, maximum in (
            (start[0], end[0] - start[0], self.xmin, self.xmax),
            (start[1], end[1] - start[1], self.ymin, self.ymax),
        ):
            if delta == 0.0:
                if not minimum <= origin <= maximum:
                    return False
                continue
            first, last = sorted(((minimum - origin) / delta, (maximum - origin) / delta))
            lower, upper = max(lower, first), min(upper, last)
            if lower > upper:
                return False
        return True


class OperationsCorridors:
    """Locate an atomic route span through an authored shared corridor.

    The span starts on the approach that first intersects the region and ends
    at the first authored route point outside it.  The caller retains its old
    footprint while atomically reserving this complete span.
    """

    def __init__(self, routing: Mapping[str, Any]) -> None:
        raw_contracts = routing.get("operations_bottlenecks", [])
        regions = routing.get("policy_region_overrides")
        if not isinstance(raw_contracts, list) or not isinstance(regions, Mapping):
            raise ValueError("operations bottlenecks require authored policy regions")
        corridors: list[_Corridor] = []
        for raw in raw_contracts:
            if not isinstance(raw, Mapping) or set(raw) != {"id", "region_id", "release_policy"}:
                raise ValueError("invalid operations bottleneck contract")
            corridor_id, region_id = raw["id"], raw["region_id"]
            if (not isinstance(corridor_id, str) or not corridor_id
                    or not isinstance(region_id, str) or region_id not in regions
                    or raw["release_policy"] != "first_route_point_outside_region"):
                raise ValueError("invalid operations bottleneck contract")
            rectangle = regions[region_id]
            if (not isinstance(rectangle, list) or len(rectangle) != 4
                    or any(isinstance(value, bool) or not isinstance(value, (int, float))
                           or not math.isfinite(value) for value in rectangle)):
                raise ValueError("operations bottleneck region must be a finite rectangle")
            xmin, ymin, xmax, ymax = map(float, rectangle)
            if xmin >= xmax or ymin >= ymax:
                raise ValueError("operations bottleneck region must have positive area")
            corridors.append(_Corridor(corridor_id, xmin, ymin, xmax, ymax))
        if len({item.corridor_id for item in corridors}) != len(corridors):
            raise ValueError("operations bottleneck IDs must be unique")
        self._corridors = tuple(corridors)

    def atomic_span(self, start: Sequence[float], remaining: Sequence[Sequence[float]]) -> CorridorSpan | None:
        origin = _finite_point(start)
        points = tuple(_finite_point(point) for point in remaining)
        if not points:
            return None
        matches = [corridor for corridor in self._corridors
                   if corridor.contains(origin) or corridor.intersects(origin, points[0])]
        if len(matches) > 1:
            raise ValueError("a route approach intersects multiple operations bottlenecks")
        if not matches:
            return None
        corridor = matches[0]
        entered = corridor.contains(origin)
        previous = origin
        for index, point in enumerate(points):
            entered = entered or corridor.intersects(previous, point)
            if entered and not corridor.contains(point):
                return CorridorSpan(corridor.corridor_id, points[:index + 1])
            previous = point
        raise ValueError(f"route enters {corridor.corridor_id} without an authored safe exit")
