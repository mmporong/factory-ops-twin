"""Validated factory geometry and deterministic CPU grid exports.

The grid in this module is an authored planning model.  It deliberately does
not claim to be a Nova Carter footprint, a Nav2 costmap, or a safety model.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


V3_REVISION = "amr-focus-v3"
V4_REVISION = "amr-production-bay-v4"
FLEET4_REVISION = "amr-production-bay-fleet4-v1"
DUAL_LANE_REVISION = "amr-semiconductor-dual-lane-v1"
FLEET_SCALE12_REVISION = "amr-semiconductor-scale12-v1"
FLEET_SCALE16_REVISION = "amr-semiconductor-scale16-v1"
CAMPUS16_REVISION = "amr-semiconductor-campus16-v1"
CAMPUS16_BUFFERED_REVISION = "amr-semiconductor-campus16-v2"
CAMPUS_LAYOUT_REVISIONS = frozenset((CAMPUS16_REVISION, CAMPUS16_BUFFERED_REVISION))
FLEET_SCALE_LAYOUT_REVISIONS = frozenset((FLEET_SCALE12_REVISION, FLEET_SCALE16_REVISION))
FLEET4_LAYOUT_REVISIONS = frozenset((FLEET4_REVISION, DUAL_LANE_REVISION, *CAMPUS_LAYOUT_REVISIONS, *FLEET_SCALE_LAYOUT_REVISIONS))
PRODUCTION_BAY_REVISIONS = frozenset((V4_REVISION, *FLEET4_LAYOUT_REVISIONS))
V3_STATIONS = {
    "S1": (4.0, 4.0),
    "S2": (4.0, 12.0),
    "S3": (4.0, 20.0),
    "CI": (28.0, 8.0),
    "CO": (28.0, 16.0),
    "OUT": (36.0, 12.0),
}
V3_POCKETS = {"P1": (16.0, 1.5), "P2": (16.0, 22.5)}
V4_STATIONS = {
    "S1": (3.5, 4.0),
    "S2": (3.5, 10.0),
    "S3": (3.5, 16.0),
    "CI": (21.5, 7.0),
    "CO": (21.5, 13.0),
    "OUT": (29.0, 10.0),
}
V4_POCKETS = {"P1": (12.0, 1.5), "P2": (12.0, 18.5)}
FLEET4_STATIONS = {name: (xy[0], xy[1] + 1.0) for name, xy in V4_STATIONS.items()}
FLEET4_POCKETS = {"P1": (12.0, 1.5), "P2": (12.0, 20.5),
                  "P3": (17.0, 1.5), "P4": (17.0, 20.5)}
DUAL_LANE_STATIONS = {
    "S1": (3.5, 4.5), "S2": (3.5, 13.5), "S3": (3.5, 22.5),
    "CI": (21.5, 10.5), "CO": (21.5, 16.5), "OUT": (32.5, 13.5),
}
DUAL_LANE_POCKETS = {"P1": (12.0, 1.5), "P2": (12.0, 26.0),
                     "P3": (17.0, 1.5), "P4": (17.0, 26.0)}
FLEET_SCALE12_POCKETS = {
    **DUAL_LANE_POCKETS,
    **{f"P{index}": (x, y) for index, (x, y) in enumerate(
        ((36.0, 1.5), (36.0, 26.0), (40.0, 1.5), (40.0, 26.0),
         (44.0, 1.5), (44.0, 26.0), (48.0, 1.5), (48.0, 26.0)), 5)},
}
FLEET_SCALE16_POCKETS = {
    **FLEET_SCALE12_POCKETS,
    **{f"P{index}": (x, y) for index, (x, y) in enumerate(
        ((52.0, 1.5), (52.0, 26.0), (56.0, 1.5), (56.0, 26.0)), 13)},
}
CAMPUS_BAY_OFFSETS = {"A": (0.0, 0.0), "B": (42.0, 0.0), "C": (0.0, 32.0), "D": (42.0, 32.0)}
CAMPUS_BAY_OFFSETS_BY_REVISION = {
    CAMPUS16_REVISION: CAMPUS_BAY_OFFSETS,
    CAMPUS16_BUFFERED_REVISION: {bay: (dx, dy + 8.0) for bay, (dx, dy) in CAMPUS_BAY_OFFSETS.items()},
}
CAMPUS_BOUNDS_BY_REVISION = {
    CAMPUS16_REVISION: (0.0, 0.0, 78.0, 60.0),
    CAMPUS16_BUFFERED_REVISION: (0.0, 0.0, 82.0, 76.0),
}
CAMPUS_STATIONS = {f"{bay}_{name}": (point[0] + dx, point[1] + dy)
                   for bay, (dx, dy) in CAMPUS_BAY_OFFSETS.items() for name, point in DUAL_LANE_STATIONS.items()}
CAMPUS_POCKETS = {f"{bay}_{name}": (point[0] + dx, point[1] + dy)
                  for bay, (dx, dy) in CAMPUS_BAY_OFFSETS.items() for name, point in DUAL_LANE_POCKETS.items()}
# Compatibility aliases preserve the established V1-V3 mission contracts.
EXPECTED_STATIONS = V3_STATIONS
EXPECTED_POCKETS = V3_POCKETS
EXPECTED_RESOURCE_IDS = {"Z_UPPER", "Z_LOWER", "J_SHIP"}
REQUIRED_FIXTURE_PREFIXES = {"F1", "F2", "F3"}


class LayoutError(ValueError):
    """Raised when authored geometry violates the deterministic contract."""


@dataclass(frozen=True)
class Rect:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @classmethod
    def from_value(cls, value: Any, label: str) -> "Rect":
        if not isinstance(value, list) or len(value) != 4:
            raise LayoutError(f"{label} must be [xmin, ymin, xmax, ymax]")
        numbers = tuple(_finite_number(item, f"{label}[{index}]") for index, item in enumerate(value))
        rect = cls(*numbers)
        if rect.xmin < 0 or rect.ymin < 0 or rect.xmax <= rect.xmin or rect.ymax <= rect.ymin:
            raise LayoutError(f"{label} is not a positive, ordered rectangle")
        return rect

    def contains_point(self, xy: tuple[float, float]) -> bool:
        x, y = xy
        return self.xmin <= x < self.xmax and self.ymin <= y < self.ymax

    def contains_cell_center_mm(self, x_mm: int, y_mm: int) -> bool:
        return (
            _mm(self.xmin) <= x_mm < _mm(self.xmax)
            and _mm(self.ymin) <= y_mm < _mm(self.ymax)
        )

    def overlaps(self, other: "Rect") -> bool:
        return (
            self.xmin < other.xmax
            and other.xmin < self.xmax
            and self.ymin < other.ymax
            and other.ymin < self.ymax
        )


@dataclass(frozen=True)
class PathResult:
    distance_m: float
    cells: tuple[tuple[int, int], ...]


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LayoutError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise LayoutError(f"{label} must be finite")
    return result


def _point(value: Any, label: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise LayoutError(f"{label} must be [x, y]")
    return (_finite_number(value[0], f"{label}[0]"), _finite_number(value[1], f"{label}[1]"))


def _mm(value: float) -> int:
    scaled = value * 1000
    rounded = round(scaled)
    if not math.isclose(scaled, rounded, abs_tol=1e-7):
        raise LayoutError(f"coordinate {value!r} is not an exact millimetre value")
    return int(rounded)


class Layout:
    """Validated layout with a fixed 50 mm, south-origin planning grid."""

    def __init__(self, data: Mapping[str, Any], *, source_sha256: str | None = None) -> None:
        self.data = dict(data)
        self.source_sha256 = source_sha256
        self.validate()
        self.bounds = Rect.from_value(self.data["bounds_xy"], "bounds_xy")
        self.resolution_m = float(self.data["map_resolution_m"])
        self.cell_mm = _mm(self.resolution_m)
        self.width = _mm(self.bounds.xmax) // self.cell_mm
        self.height = _mm(self.bounds.ymax) // self.cell_mm
        self.allowed_regions = tuple(
            Rect.from_value(item["rect"], f"amr_allowed_regions[{index}].rect")
            for index, item in enumerate(self.data["amr_allowed_regions"])
        )
        self.obstacles = tuple(
            Rect.from_value(item["rect"], f"physical_obstacles[{index}].rect")
            for index, item in enumerate(self.data["physical_obstacles"])
        )
        self.blocked_aisle = Rect.from_value(self.data["blocked_aisle"]["rect"], "blocked_aisle.rect")
        self.blocked_aisles = tuple(Rect.from_value(item["rect"], "blocked_aisles.rect")
                                   for item in self.data.get("blocked_aisles", [self.data["blocked_aisle"]]))
        self.stations = {item["id"]: _point(item["xy"], f"station {item['id']}") for item in self.data["stations"]}
        self.pockets = {key: _point(value, f"pocket {key}") for key, value in self.data["pockets"].items()}
        self._raw_normal = self._make_raw_mask()
        self._planning_normal = self._erode_for_probe(self._raw_normal)

    @classmethod
    def from_file(cls, path: str | Path) -> "Layout":
        source = Path(path).read_bytes()
        try:
            data = json.loads(source)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise LayoutError(f"invalid UTF-8 JSON: {exc}") from exc
        if not isinstance(data, dict):
            raise LayoutError("layout root must be an object")
        return cls(data, source_sha256=hashlib.sha256(source).hexdigest())

    def validate(self) -> "Layout":
        required = {
            "units", "frame", "bounds_xy", "map_resolution_m", "amr_allowed_regions",
            "stations", "pockets", "physical_obstacles", "blocked_aisle",
        }
        missing = required - self.data.keys()
        if missing:
            raise LayoutError(f"missing keys: {sorted(missing)}")
        if self.data["units"] != "m" or self.data["frame"] != "map":
            raise LayoutError("layout must use metres in the map frame")
        revision = self.data.get("revision")
        if revision in CAMPUS_LAYOUT_REVISIONS:
            bay_offsets = CAMPUS_BAY_OFFSETS_BY_REVISION[revision]
            expected_bounds = CAMPUS_BOUNDS_BY_REVISION[revision]
            expected_stations = {
                f"{bay}_{name}": (point[0] + dx, point[1] + dy)
                for bay, (dx, dy) in bay_offsets.items() for name, point in DUAL_LANE_STATIONS.items()
            }
            expected_pockets = {
                f"{bay}_{name}": (point[0] + dx, point[1] + dy)
                for bay, (dx, dy) in bay_offsets.items() for name, point in DUAL_LANE_POCKETS.items()
            }
            barrier_y = (3.0 + bay_offsets["A"][1], 6.0 + bay_offsets["A"][1])
        elif revision == FLEET_SCALE12_REVISION:
            expected_bounds = (0.0, 0.0, 51.0, 28.0)
            expected_stations = DUAL_LANE_STATIONS
            expected_pockets = FLEET_SCALE12_POCKETS
            barrier_y = (3.0, 6.0)
        elif revision == FLEET_SCALE16_REVISION:
            expected_bounds = (0.0, 0.0, 59.0, 28.0)
            expected_stations = DUAL_LANE_STATIONS
            expected_pockets = FLEET_SCALE16_POCKETS
            barrier_y = (3.0, 6.0)
        elif revision == DUAL_LANE_REVISION:
            expected_bounds = (0.0, 0.0, 36.0, 28.0)
            expected_stations = DUAL_LANE_STATIONS
            expected_pockets = DUAL_LANE_POCKETS
            barrier_y = (3.0, 6.0)
        elif revision == FLEET4_REVISION:
            expected_bounds = (0.0, 0.0, 32.0, 22.0)
            expected_stations = FLEET4_STATIONS
            expected_pockets = FLEET4_POCKETS
            barrier_y = (3.5, 6.5)
        elif revision == V4_REVISION:
            expected_bounds = (0.0, 0.0, 32.0, 20.0)
            expected_stations = V4_STATIONS
            expected_pockets = V4_POCKETS
            barrier_y = (2.5, 5.5)
        else:
            expected_bounds = (0.0, 0.0, 40.0, 24.0)
            expected_stations = V3_STATIONS
            expected_pockets = V3_POCKETS
            barrier_y = (2.5, 5.5)
        bounds = Rect.from_value(self.data["bounds_xy"], "bounds_xy")
        if tuple((bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax)) != expected_bounds:
            raise LayoutError(f"bounds_xy must remain {list(expected_bounds)} for revision {revision!r}")
        resolution = _finite_number(self.data["map_resolution_m"], "map_resolution_m")
        if resolution != 0.05:
            raise LayoutError("map_resolution_m must remain 0.05")

        allowed = self._validated_named_rects(self.data["amr_allowed_regions"], "amr_allowed_regions", bounds)
        obstacles = self._validated_named_rects(self.data["physical_obstacles"], "physical_obstacles", bounds)
        expected_obstacle_count = 76 if revision in CAMPUS_LAYOUT_REVISIONS else 19
        if len(obstacles) != expected_obstacle_count:
            raise LayoutError(f"physical_obstacles must contain {expected_obstacle_count} authored footprints")
        for allowed_id, allowed_rect in allowed.items():
            for obstacle_id, obstacle_rect in obstacles.items():
                if allowed_rect.overlaps(obstacle_rect):
                    raise LayoutError(f"allowed region {allowed_id} overlaps collider {obstacle_id}")
        for index, obstacle in enumerate(self.data["physical_obstacles"]):
            height = _finite_number(obstacle.get("height_m"), f"physical_obstacles[{index}].height_m")
            if height <= 0:
                raise LayoutError("obstacle height must be positive")

        station_values = self.data["stations"]
        if not isinstance(station_values, list):
            raise LayoutError("stations must be a list")
        station_ids = _unique_ids(station_values, "stations")
        if station_ids != set(expected_stations):
            raise LayoutError("station IDs must remain S1, S2, S3, CI, CO, OUT")
        for station in station_values:
            xy = _point(station.get("xy"), f"station {station['id']}")
            if xy != expected_stations[station["id"]]:
                raise LayoutError(f"station {station['id']} coordinate is immutable")
            _validate_point_in_bounds(xy, bounds, f"station {station['id']}")

        pockets = self.data["pockets"]
        if not isinstance(pockets, dict) or set(pockets) != set(expected_pockets):
            raise LayoutError(f"pockets must remain {sorted(expected_pockets)}")
        for pocket_id, expected in expected_pockets.items():
            xy = _point(pockets[pocket_id], f"pocket {pocket_id}")
            if xy != expected:
                raise LayoutError(f"pocket {pocket_id} coordinate is immutable")
            _validate_point_in_bounds(xy, bounds, f"pocket {pocket_id}")

        barrier = Rect.from_value(self.data["blocked_aisle"].get("rect"), "blocked_aisle.rect")
        _validate_rect_in_bounds(barrier, bounds, "blocked_aisle.rect")
        if (barrier.ymin, barrier.ymax) != barrier_y:
            raise LayoutError("blocked_aisle must close the full upper aisle width")
        if _finite_number(self.data["blocked_aisle"].get("height_m"), "blocked_aisle.height_m") <= 0:
            raise LayoutError("blocked_aisle height must be positive")
        if revision in CAMPUS_LAYOUT_REVISIONS:
            barriers = self.data.get("blocked_aisles")
            if not isinstance(barriers, list) or {row.get("id") for row in barriers} != {
                f"{bay}_BLOCKED_AISLE" for bay in bay_offsets
            }:
                raise LayoutError("campus requires four uniquely identified closure barriers")
            for row in barriers:
                rect = Rect.from_value(row.get("rect"), "campus blocked aisle")
                _validate_rect_in_bounds(rect, bounds, "campus blocked aisle")
                offset_y = bay_offsets[row["id"].split("_")[0]][1]
                if (rect.ymin, rect.ymax) != (3.0 + offset_y, 6.0 + offset_y):
                    raise LayoutError("campus closure must retain its authored lane width")
                if _finite_number(row.get("height_m"), "campus barrier height") <= 0:
                    raise LayoutError("campus barrier height must be positive")
        return self

    @staticmethod
    def _validated_named_rects(values: Any, label: str, bounds: Rect) -> dict[str, Rect]:
        if not isinstance(values, list) or not values:
            raise LayoutError(f"{label} must be a non-empty list")
        _unique_ids(values, label)
        result: dict[str, Rect] = {}
        for index, item in enumerate(values):
            rect = Rect.from_value(item.get("rect"), f"{label}[{index}].rect")
            _validate_rect_in_bounds(rect, bounds, f"{label}[{index}].rect")
            for edge in (rect.xmin, rect.ymin, rect.xmax, rect.ymax):
                if _mm(edge) % 50:
                    raise LayoutError(f"{label}[{index}] edge is not aligned to 50 mm")
            result[item["id"]] = rect
        return result

    def _make_raw_mask(self, extra_obstacles: Iterable[Rect] = ()) -> bytearray:
        mask = bytearray(self.width * self.height)
        for rect in self.allowed_regions:
            self._fill_rect(mask, rect, 1)
        for rect in self.obstacles + tuple(extra_obstacles):
            self._fill_rect(mask, rect, 0)
        return mask

    def _fill_rect(self, mask: bytearray, rect: Rect, value: int) -> None:
        col0 = _mm(rect.xmin) // self.cell_mm
        col1 = _mm(rect.xmax) // self.cell_mm
        row0 = _mm(rect.ymin) // self.cell_mm
        row1 = _mm(rect.ymax) // self.cell_mm
        fill = bytes([value]) * (col1 - col0)
        for row in range(row0, row1):
            offset = row * self.width
            mask[offset + col0 : offset + col1] = fill

    def _erode_for_probe(self, raw: bytearray) -> bytearray:
        radius = 14
        integral = [0] * ((self.width + 1) * (self.height + 1))
        stride = self.width + 1
        for row in range(self.height):
            running = 0
            for col in range(self.width):
                running += raw[row * self.width + col]
                integral[(row + 1) * stride + col + 1] = integral[row * stride + col + 1] + running
        result = bytearray(self.width * self.height)
        required = (radius * 2 + 1) ** 2
        for row in range(radius, self.height - radius):
            y0, y1 = row - radius, row + radius + 1
            for col in range(radius, self.width - radius):
                x0, x1 = col - radius, col + radius + 1
                total = integral[y1 * stride + x1] - integral[y0 * stride + x1] - integral[y1 * stride + x0] + integral[y0 * stride + x0]
                if total == required:
                    result[row * self.width + col] = 1
        return result

    def coordinate_to_cell(self, xy: Sequence[float]) -> tuple[int, int]:
        point = _point(list(xy), "coordinate")
        _validate_point_in_bounds(point, self.bounds, "coordinate")
        return (_mm(point[0]) // self.cell_mm, _mm(point[1]) // self.cell_mm)

    def is_planning_free(self, xy: Sequence[float], *, blocked: bool = False) -> bool:
        col, row = self.coordinate_to_cell(xy)
        mask = self._planning_mask(blocked)
        return bool(mask[row * self.width + col])

    def _planning_mask(self, blocked: bool) -> bytearray:
        if not blocked:
            return self._planning_normal
        return self._erode_for_probe(self._make_raw_mask(self.blocked_aisles))

    def shortest_path(
        self,
        start_xy: Sequence[float],
        goal_xy: Sequence[float],
        *,
        blocked: bool = False,
        forbidden_rects: Iterable[Rect] = (),
    ) -> PathResult:
        start = self.coordinate_to_cell(start_xy)
        goal = self.coordinate_to_cell(goal_xy)
        mask = bytearray(self._planning_mask(blocked))
        for rect in forbidden_rects:
            self._fill_rect(mask, rect, 0)
        for label, (col, row) in (("start", start), ("goal", goal)):
            if not mask[row * self.width + col]:
                raise LayoutError(f"{label} point is not free for the 29x29 planning probe")

        frontier = deque([start])
        parents: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        while frontier:
            current = frontier.popleft()
            if current == goal:
                break
            col, row = current
            for neighbor in ((col + 1, row), (col - 1, row), (col, row + 1), (col, row - 1)):
                ncol, nrow = neighbor
                if 0 <= ncol < self.width and 0 <= nrow < self.height and neighbor not in parents:
                    if mask[nrow * self.width + ncol]:
                        parents[neighbor] = current
                        frontier.append(neighbor)
        if goal not in parents:
            raise LayoutError("no 4-neighbor path between points")
        reversed_path: list[tuple[int, int]] = []
        cursor: tuple[int, int] | None = goal
        while cursor is not None:
            reversed_path.append(cursor)
            cursor = parents[cursor]
        cells = tuple(reversed(reversed_path))
        return PathResult(distance_m=((len(cells) - 1) * self.cell_mm) / 1000, cells=cells)

    def validate_traffic(self, traffic: Mapping[str, Any]) -> dict[str, Any]:
        if traffic.get("units") != "m" or traffic.get("frame") != "map":
            raise LayoutError("traffic must use metres in the map frame")
        resources = traffic.get("resources")
        if not isinstance(resources, list):
            raise LayoutError("traffic resources must be a list")
        resource_ids = _unique_ids(resources, "resources")
        if resource_ids != EXPECTED_RESOURCE_IDS:
            raise LayoutError("traffic resources must be Z_UPPER, Z_LOWER and J_SHIP")
        resource_rects: dict[str, Rect] = {}
        traffic_points: list[tuple[str, tuple[float, float]]] = []
        for index, resource in enumerate(resources):
            polygon = resource.get("polygon_xy")
            if not isinstance(polygon, list) or len(polygon) != 4:
                raise LayoutError(f"resources[{index}].polygon_xy must be a rectangular four-point polygon")
            points = [_point(value, f"resources[{index}].polygon_xy") for value in polygon]
            xs, ys = {p[0] for p in points}, {p[1] for p in points}
            if len(xs) != 2 or len(ys) != 2 or set(points) != {(x, y) for x in xs for y in ys}:
                raise LayoutError(f"resources[{index}] polygon must be axis-aligned rectangle corners")
            rect = Rect(min(xs), min(ys), max(xs), max(ys))
            _validate_rect_in_bounds(rect, self.bounds, f"resource {resource['id']}")
            resource_rects[resource["id"]] = rect
            ports = resource.get("ports")
            if not isinstance(ports, dict) or not ports:
                raise LayoutError(f"resource {resource['id']} must declare ports")
            for port_id, port in ports.items():
                xy = _point(port.get("xy"), f"port {resource['id']}.{port_id}")
                traffic_points.append((f"port {resource['id']}.{port_id}", xy))

        if traffic.get("pockets") != self.data["pockets"]:
            raise LayoutError("traffic pockets must match layout pockets")
        routes = traffic.get("fixture_routes")
        if not isinstance(routes, dict) or not routes:
            raise LayoutError("fixture_routes must be a non-empty object")
        if {name.split("_", 1)[0] for name in routes} != REQUIRED_FIXTURE_PREFIXES:
            raise LayoutError("fixture routes must cover F1, F2 and F3")
        checked_segments = 0
        for route_id, route in routes.items():
            if not isinstance(route, dict):
                raise LayoutError(f"fixture route {route_id} must be an object")
            waypoints = route.get("waypoints_xy", [])
            if waypoints:
                if not isinstance(waypoints, list) or len(waypoints) < 2:
                    raise LayoutError(f"fixture route {route_id} needs at least two waypoints")
                points = [_point(value, f"fixture route {route_id}") for value in waypoints]
                forbidden = tuple(resource_rects[item] for item in route.get("forbidden_resources", []))
                for start, goal in zip(points, points[1:]):
                    self.shortest_path(start, goal, forbidden_rects=forbidden)
                    checked_segments += 1
            for key in ("carter1_xy", "carter2_xy"):
                if key in route:
                    traffic_points.append((f"{route_id}.{key}", _point(route[key], f"{route_id}.{key}")))
            for index, value in enumerate(route.get("goals_xy", [])):
                traffic_points.append((f"{route_id}.goals_xy[{index}]", _point(value, f"{route_id}.goals_xy[{index}]")))
        for label, xy in traffic_points:
            if not self.is_planning_free(xy):
                raise LayoutError(f"{label} is not planning-free")
        return {"resources": len(resources), "fixture_routes": len(routes), "checked_segments": checked_segments}

    def planning_analysis(self, traffic: Mapping[str, Any]) -> dict[str, Any]:
        traffic_summary = self.validate_traffic(traffic)
        normal = self.shortest_path(self.stations["S1"], self.stations["CI"])
        blocked = self.shortest_path(self.stations["S1"], self.stations["CI"], blocked=True)
        alternative_y_mm = {V4_REVISION: 14_500, FLEET4_REVISION: 15_500}.get(
            str(self.data.get("revision", "")), 18_500)
        if not any(row * self.cell_mm + self.cell_mm // 2 >= alternative_y_mm for _, row in blocked.cells):
            raise LayoutError("blocked route does not use the authored lower alternative")
        for label, xy in {**self.stations, **self.pockets}.items():
            if not self.is_planning_free(xy):
                raise LayoutError(f"{label} is not planning-free")
        return {
            "planning_backend": "CPU 4-neighbor BFS over exact 50 mm integer grid",
            "probe": "29x29 whole cells; nominal 0.7 m half-extent, 1.45 m cover",
            "normal_s1_ci_m": normal.distance_m,
            "blocked_s1_ci_m": blocked.distance_m,
            "blocked_uses_y_at_least_m": alternative_y_mm / 1000,
            "stations": len(self.stations),
            "pockets": len(self.pockets),
            **traffic_summary,
            "collision_validation": "not performed for Nova Carter, Nav2 or Isaac Sim",
        }

    def export_maps(self, output_dir: str | Path, *, include_dynamic_barrier: bool = False) -> dict[str, Any]:
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        extra = (self.blocked_aisle,) if include_dynamic_barrier else ()
        physical = self._physical_mask(extra)
        policy_keepout = self._keepout_mask()
        raw_nav = self._make_raw_mask(extra)
        for index in range(len(raw_nav)):
            composited_free = not physical[index] and not policy_keepout[index]
            if composited_free != bool(raw_nav[index]):
                raise LayoutError("physical map and keepout map do not compose to authored nav-free geometry")
        map_path = output / "map.pgm"
        keepout_path = output / "keepout.pgm"
        map_path.write_bytes(self._pgm_bytes(physical))
        keepout_path.write_bytes(self._pgm_bytes(policy_keepout))
        map_yaml = output / "map.yaml"
        keepout_yaml = output / "keepout.yaml"
        map_yaml.write_text(self._yaml("map.pgm"), encoding="utf-8", newline="\n")
        keepout_yaml.write_text(self._yaml("keepout.pgm"), encoding="utf-8", newline="\n")
        files = [map_path, keepout_path, map_yaml, keepout_yaml]
        return {
            "frame": "map",
            "width_cells": self.width,
            "height_cells": self.height,
            "resolution_m": self.resolution_m,
            "origin": [0.0, 0.0, 0.0],
            "row_zero": "south in memory; north is first pixel row in PGM",
            "dynamic_barrier_included": include_dynamic_barrier,
            "planning_backend": "CPU geometry rasterization; not Nav2",
            "collision_validation": "not performed",
            "source_sha256": self.source_sha256,
            "files": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
        }

    def _physical_mask(self, extra: Iterable[Rect]) -> bytearray:
        mask = bytearray(self.width * self.height)
        for rect in self.obstacles + tuple(extra):
            self._fill_rect(mask, rect, 1)
        return mask

    def _keepout_mask(self) -> bytearray:
        mask = bytearray([1]) * (self.width * self.height)
        for rect in self.allowed_regions:
            self._fill_rect(mask, rect, 0)
        return mask

    def _pgm_bytes(self, occupied: bytearray) -> bytes:
        header = f"P5\n# north-up ROS occupancy image\n{self.width} {self.height}\n255\n".encode("ascii")
        pixels = bytearray()
        for row in range(self.height - 1, -1, -1):
            offset = row * self.width
            pixels.extend(0 if occupied[offset + col] else 254 for col in range(self.width))
        return header + pixels

    def _yaml(self, image_name: str) -> str:
        return (
            f"image: {image_name}\n"
            "mode: trinary\n"
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )


def load_traffic(path: str | Path) -> dict[str, Any]:
    source = Path(path).read_bytes()
    try:
        value = json.loads(source)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LayoutError(f"invalid UTF-8 JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise LayoutError("traffic root must be an object")
    return value


def _unique_ids(values: list[Any], label: str) -> set[str]:
    seen: set[str] = set()
    for index, item in enumerate(values):
        if not isinstance(item, dict):
            raise LayoutError(f"{label}[{index}] must be an object")
        identifier = item.get("id")
        if not isinstance(identifier, str) or not identifier:
            raise LayoutError(f"{label}[{index}].id must be a non-empty string")
        if identifier in seen:
            raise LayoutError(f"duplicate {label} ID: {identifier}")
        seen.add(identifier)
    return seen


def _validate_rect_in_bounds(rect: Rect, bounds: Rect, label: str) -> None:
    if rect.xmin < bounds.xmin or rect.ymin < bounds.ymin or rect.xmax > bounds.xmax or rect.ymax > bounds.ymax:
        raise LayoutError(f"{label} lies outside bounds")


def _validate_point_in_bounds(xy: tuple[float, float], bounds: Rect, label: str) -> None:
    x, y = xy
    if x < bounds.xmin or y < bounds.ymin or x >= bounds.xmax or y >= bounds.ymax:
        raise LayoutError(f"{label} lies outside bounds")
