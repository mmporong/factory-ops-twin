"""Authored G3 route/tile compilation, separate from physical navigation evidence.

The route book accepts only the currently observed closure state. It has no
order queue, future removal time, simulator ground truth or action completion
timer. A future ROS adapter must still check its real path and localization.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import heapq
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

from .footprint import _bad_touched_cells
from .layout import PRODUCTION_BAY_REVISIONS, Layout, LayoutError, Rect

Point = tuple[float, float]
Tile = tuple[int, int]


def _number(value: object, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{label} must be finite numeric data")
    if value < 0 or (positive and value == 0):
        raise ValueError(f"{label} is out of range")
    return float(value)


def _point(value: Sequence[float]) -> Point:
    if len(value) != 2:
        raise ValueError("route point must have exactly two coordinates")
    return (_number(value[0], "x"), _number(value[1], "y"))


def _hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    fraction = 0.0 if length_squared == 0 else max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
    return math.hypot(point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy)


def segment_rectangle_distance(start: Point, end: Point, rect: Rect) -> float:
    """Exact Euclidean segment/rectangle distance including boundary contact."""
    lower, upper = 0.0, 1.0
    intersects = True
    for origin, delta, minimum, maximum in (
        (start[0], end[0] - start[0], rect.xmin, rect.xmax),
        (start[1], end[1] - start[1], rect.ymin, rect.ymax),
    ):
        if delta == 0:
            if not minimum <= origin <= maximum:
                intersects = False
                break
        else:
            first, last = sorted(((minimum - origin) / delta, (maximum - origin) / delta))
            lower, upper = max(lower, first), min(upper, last)
            if lower > upper:
                intersects = False
                break
    if intersects:
        return 0.0
    corners = ((rect.xmin, rect.ymin), (rect.xmax, rect.ymin), (rect.xmax, rect.ymax), (rect.xmin, rect.ymax))
    distances = [_point_segment_distance(corner, start, end) for corner in corners]
    for point in (start, end):
        distances.append(math.hypot(max(rect.xmin - point[0], 0.0, point[0] - rect.xmax),
                                    max(rect.ymin - point[1], 0.0, point[1] - rect.ymax)))
    return min(distances)


def swept_tiles(points: Sequence[Sequence[float]], radius_m: float, *, tile_size_m: float = 1.0,
                bounds_xy: Sequence[float] = (0.0, 0.0, 40.0, 24.0)) -> frozenset[Tile]:
    """Cover an entire polyline's circular envelope with intersecting tiles.

    Closed rectangle contact includes both adjacent tiles on a boundary.
    Disjoint sets therefore have disjoint covered envelopes. This compilation
    is conservative and does not grant any permission to move.
    """
    radius = _number(radius_m, "radius_m", positive=True)
    size = _number(tile_size_m, "tile_size_m", positive=True)
    bounds = Rect.from_value(list(bounds_xy), "bounds_xy")
    samples = tuple(_point(point) for point in points)
    if not samples:
        raise ValueError("a route must have at least one point")
    if any(not bounds.contains_point(point) for point in samples):
        raise ValueError("route point lies outside the factory")
    segments = tuple(zip(samples, samples[1:])) or ((samples[0], samples[0]),)
    tiles: set[Tile] = set()
    for start, end in segments:
        tolerance = max(size, radius, 1.0) * 1e-12
        first_col = math.floor((min(start[0], end[0]) - radius - bounds.xmin - tolerance) / size)
        last_col = math.floor((max(start[0], end[0]) + radius - bounds.xmin + tolerance) / size)
        first_row = math.floor((min(start[1], end[1]) - radius - bounds.ymin - tolerance) / size)
        last_row = math.floor((max(start[1], end[1]) + radius - bounds.ymin + tolerance) / size)
        for row in range(first_row, last_row + 1):
            for col in range(first_col, last_col + 1):
                rect = Rect(bounds.xmin + col * size, bounds.ymin + row * size,
                            bounds.xmin + (col + 1) * size, bounds.ymin + (row + 1) * size)
                if segment_rectangle_distance(start, end, rect) <= radius + tolerance:
                    tiles.add((col, row))
    return frozenset(tiles)


@dataclass(frozen=True, slots=True)
class RoutePlan:
    route_id: str
    start: str
    goal: str
    closed: bool
    nodes: tuple[str, ...]
    points: tuple[Point, ...]
    distance_m: float
    tiles: frozenset[Tile]

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["tiles"] = [list(tile) for tile in sorted(self.tiles)]
        return value


@dataclass(frozen=True, slots=True)
class DispatchRouteCompatibility:
    """Static full-task route compatibility with peer parked holds.

    This is a dispatch filter only. It does not grant movement, replace the
    reservation manager, or predict a peer's future state.
    """

    robot_id: str
    task_id: str
    current_anchor: str
    pickup_station: str
    drop_station: str
    closed: bool
    pickup_route_id: str
    loaded_route_id: str
    pickup_distance_m: float
    loaded_distance_m: float
    tile_count: int
    blockers: tuple[tuple[str, str, bool, tuple[Tile, ...]], ...]

    @property
    def compatible(self) -> bool:
        return not self.blockers

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "task_id": self.task_id,
            "current_anchor": self.current_anchor,
            "pickup_station": self.pickup_station,
            "drop_station": self.drop_station,
            "closed": self.closed,
            "pickup_route_id": self.pickup_route_id,
            "loaded_route_id": self.loaded_route_id,
            "pickup_distance_m": self.pickup_distance_m,
            "loaded_distance_m": self.loaded_distance_m,
            "full_task_distance_m": self.pickup_distance_m + self.loaded_distance_m,
            "tile_count": self.tile_count,
            "compatible": self.compatible,
            "reason": None if self.compatible else "peer_parked_hold_overlap",
            "blockers": [
                {
                    "robot_id": robot_id,
                    "tile_hash": tile_hash,
                    "observation_fresh": observation_fresh,
                    "intersection_tiles": [list(tile) for tile in intersection],
                }
                for robot_id, tile_hash, observation_fresh, intersection in self.blockers
            ],
        }


def assess_dispatch_route_compatibility(
    book: "RouteBook",
    *,
    robot_id: str,
    task_id: str,
    current_anchor: str,
    pickup_station: str,
    drop_station: str,
    closed: bool,
    reservation_radius_m: float,
    reservation_snapshot: Mapping[str, Any],
) -> DispatchRouteCompatibility:
    """Compare one full task route with current peer parked reservations.

    Parked holds remain blockers even if their latest pose observation is
    stale. The caller still has to obtain the normal reservation transfer and
    Nav2 path authorization before movement.
    """
    if not robot_id or not task_id:
        raise ValueError("robot_id and task_id are required")
    if type(closed) is not bool:
        raise ValueError("closed must be a boolean")
    if not isinstance(reservation_snapshot, Mapping):
        raise ValueError("reservation_snapshot must be a mapping")
    holds = reservation_snapshot.get("holds")
    observations = reservation_snapshot.get("observations")
    if not isinstance(holds, Mapping) or not isinstance(observations, Mapping):
        raise ValueError("reservation snapshot requires holds and observations")

    pickup = book.plan(current_anchor, pickup_station, closed=closed)
    loaded = book.plan(pickup_station, drop_station, closed=closed)
    bounds = (book.layout.bounds.xmin, book.layout.bounds.ymin,
              book.layout.bounds.xmax, book.layout.bounds.ymax)
    route_tiles = swept_tiles(
        (*pickup.points, *loaded.points[1:]),
        reservation_radius_m,
        tile_size_m=book.tile_size_m,
        bounds_xy=bounds,
    )
    blockers: list[tuple[str, str, bool, tuple[Tile, ...]]] = []
    for peer_id, raw_hold in sorted(holds.items()):
        if peer_id == robot_id:
            continue
        if not isinstance(peer_id, str) or not isinstance(raw_hold, Mapping):
            raise ValueError("reservation snapshot contains an invalid hold")
        if raw_hold.get("parked") is not True:
            continue
        raw_tiles = raw_hold.get("tiles")
        tile_hash = raw_hold.get("tile_hash")
        if (not isinstance(raw_tiles, list) or not isinstance(tile_hash, str)
                or not tile_hash):
            raise ValueError("parked hold requires tiles and tile_hash")
        peer_tiles = frozenset(
            (tile[0], tile[1])
            for tile in raw_tiles
            if isinstance(tile, list) and len(tile) == 2
            and type(tile[0]) is int and type(tile[1]) is int
        )
        if len(peer_tiles) != len(raw_tiles):
            raise ValueError("parked hold contains invalid or duplicate tiles")
        intersection = tuple(sorted(route_tiles & peer_tiles))
        if not intersection:
            continue
        observation = observations.get(peer_id)
        fresh = bool(isinstance(observation, Mapping) and observation.get("fresh") is True)
        blockers.append((peer_id, tile_hash, fresh, intersection))
    return DispatchRouteCompatibility(
        robot_id=robot_id,
        task_id=task_id,
        current_anchor=current_anchor,
        pickup_station=pickup_station,
        drop_station=drop_station,
        closed=closed,
        pickup_route_id=pickup.route_id,
        loaded_route_id=loaded.route_id,
        pickup_distance_m=pickup.distance_m,
        loaded_distance_m=loaded.distance_m,
        tile_count=len(route_tiles),
        blockers=tuple(blockers),
    )


def opposing_task_blockers(*, robot_id: str, pickup_station: str, drop_station: str,
                           active_tasks: Mapping[str, tuple[str, str]]) -> tuple[str, ...]:
    """Defer opposite transport pairs before either pickup commits.

    This is an admission filter over already assigned tasks. It never acquires
    tiles or changes a physical hold, and overlapping non-opposite traffic
    remains subject to the normal full-leg reservation protocol.
    """
    if not all(isinstance(value, str) and value for value in (robot_id, pickup_station, drop_station)):
        raise ValueError("transport robot and stations must be nonempty strings")
    blockers = []
    for peer, stations in active_tasks.items():
        if (not isinstance(peer, str) or not peer or not isinstance(stations, (tuple, list))
                or len(stations) != 2 or not all(isinstance(value, str) and value for value in stations)):
            raise ValueError("active transports require robot IDs and pickup/drop pairs")
        if peer != robot_id and tuple(stations) == (drop_station, pickup_station):
            blockers.append(peer)
    return tuple(sorted(blockers))


class RouteBook:
    """Planning-only routes over a source-pinned authored centerline graph."""

    def __init__(self, base_layout: Layout, config: Mapping[str, Any]) -> None:
        self.config = deepcopy(dict(config))
        if type(self.config.get("schema_version")) is not int or self.config["schema_version"] != 1:
            raise ValueError("unsupported routing schema")
        measurement = self.config["collision_radius"]
        if not isinstance(measurement, dict) or measurement.get("unit") != "m":
            raise ValueError("collision envelope measurement requires provenance")
        for name in ("measured_at", "valid_for", "source_file", "source_run"):
            if not isinstance(measurement.get(name), str) or not measurement[name].strip():
                raise ValueError(f"collision measurement provenance requires {name}")
        for name, length in (("source_archive_sha256", 64), ("source_commit", 40), ("source_run_id", 32)):
            if not isinstance(measurement.get(name), str) or not re.fullmatch(r"[0-9a-f]{" + str(length) + "}", measurement[name]):
                raise ValueError(f"invalid collision measurement provenance {name}")
        try:
            measured_at = datetime.fromisoformat(measurement["measured_at"])
        except ValueError as error:
            raise ValueError("measurement timestamp must be an ISO date/time with timezone") from error
        if measured_at.tzinfo is None:
            raise ValueError("measurement timestamp must include its timezone")
        collision = _number(measurement["value"], "collision_radius", positive=True)
        allowances = self.config["authored_allowances"]
        self.evaluation_radius_m = collision + _number(allowances["evaluation_clearance_m"], "evaluation_clearance_m")
        self.planning_radius_m = self.evaluation_radius_m + _number(allowances["localization_planning_margin_m"], "localization_planning_margin_m")
        self.tile_size_m = _number(allowances["tile_size_m"], "tile_size_m", positive=True)
        variant = deepcopy(base_layout.data)
        regions = {region["id"]: region for region in variant["amr_allowed_regions"]}
        for name, rectangle in self.config["policy_region_overrides"].items():
            if name not in regions:
                raise ValueError(f"unknown policy region override: {name}")
            regions[name]["rect"] = list(rectangle)
        revision = base_layout.data.get("revision")
        variant["revision"] = revision if revision in PRODUCTION_BAY_REVISIONS else "amr-focus-v3-g3a"
        variant["status"] = "G3a single-robot corridor preflight only; no physical G3 qualification"
        self.layout = Layout(variant, source_sha256=_hash(variant))
        self.base_layout = base_layout
        self.config_sha256 = _hash(self.config)
        self.nodes = {name: _point(point) for name, point in self.config["nodes"].items()}
        locations = {**self.layout.stations, **self.layout.pockets}
        if any(self.nodes.get(name) != point for name, point in locations.items()):
            raise ValueError("routing anchors must match every station and parking pocket")
        self.edges: set[tuple[str, str]] = set()
        self.adjacency: dict[str, list[str]] = {name: [] for name in self.nodes}
        for edge in self.config["edges"]:
            if len(edge) != 2 or edge[0] not in self.nodes or edge[1] not in self.nodes or edge[0] == edge[1]:
                raise ValueError("route graph edge references invalid nodes")
            key = tuple(sorted(edge))
            if key in self.edges:
                raise ValueError("duplicate undirected route graph edge")
            self.edges.add(key)
            self.adjacency[edge[0]].append(edge[1])
            self.adjacency[edge[1]].append(edge[0])
        undirected = frozenset(self.edges)
        directed = self.config.get("directed_edges", [])
        if not isinstance(directed, list):
            raise ValueError("directed_edges must be a list of one-way graph edges")
        self.directed_edges: set[tuple[str, str]] = set()
        for edge in directed:
            if (not isinstance(edge, (list, tuple)) or len(edge) != 2
                    or any(not isinstance(name, str) or name not in self.nodes for name in edge)
                    or edge[0] == edge[1]):
                raise ValueError("directed route graph edge references invalid nodes")
            arc = (edge[0], edge[1])
            physical = tuple(sorted(arc))
            if arc in self.directed_edges or physical in undirected:
                raise ValueError("duplicate or mixed-direction route graph edge")
            self.directed_edges.add(arc)
            self.edges.add(physical)
            self.adjacency[edge[0]].append(edge[1])
        self.closed_edge = tuple(sorted(self.config["closure"]["edge"]))
        closures = self.config.get("closures", [self.config["closure"]])
        if not isinstance(closures, list) or not closures:
            raise ValueError("closures must contain authored route edges")
        self.closed_edges = frozenset(tuple(sorted(item["edge"])) for item in closures)
        if self.closed_edge not in self.closed_edges or not self.closed_edges.issubset(self.edges):
            raise ValueError("closure must refer to a route graph edge")
        self._routes: dict[tuple[str, str, bool], RoutePlan] = {}

    @classmethod
    def from_files(cls, layout: str | Path, config: str | Path) -> "RouteBook":
        return cls(Layout.from_file(layout), json.loads(Path(config).read_text(encoding="utf-8")))

    def plan(self, start: str, goal: str, *, closed: bool = False) -> RoutePlan:
        if type(closed) is not bool:
            raise ValueError("route input is the current closure boolean only")
        if start not in self.nodes or goal not in self.nodes:
            raise ValueError("unknown route endpoint")
        key = (start, goal, closed)
        if key in self._routes:
            return self._routes[key]
        pending: list[tuple[float, tuple[str, ...]]] = [(0.0, (start,))]
        best: dict[str, tuple[float, tuple[str, ...]]] = {start: (0.0, (start,))}
        while pending:
            distance, names = heapq.heappop(pending)
            node = names[-1]
            if best[node] != (distance, names):
                continue
            if node == goal:
                points = tuple(self.nodes[name] for name in names)
                identity = {"config": self.config_sha256, "layout": self.layout.source_sha256,
                            "closed": closed, "nodes": names, "radius_m": self.planning_radius_m}
                bounds = (self.layout.bounds.xmin, self.layout.bounds.ymin,
                          self.layout.bounds.xmax, self.layout.bounds.ymax)
                result = RoutePlan(
                    _hash(identity), start, goal, closed, names, points, distance,
                    swept_tiles(points, self.planning_radius_m, tile_size_m=self.tile_size_m, bounds_xy=bounds),
                )
                self._routes[key] = result
                return result
            for neighbor in sorted(self.adjacency[node]):
                if closed and tuple(sorted((node, neighbor))) in self.closed_edges:
                    continue
                candidate = (distance + math.dist(self.nodes[node], self.nodes[neighbor]), (*names, neighbor))
                if neighbor not in best or candidate < best[neighbor]:
                    best[neighbor] = candidate
                    heapq.heappush(pending, candidate)
        raise ValueError("no currently traversable authored route")

    def preflight(self) -> dict[str, Any]:
        """Prove declared segments' conservative map cover, once per edge.

        Adding half a sample interval to the radius covers each intervening
        point by the triangle inequality. Existing 50 mm touched-cell checks
        are reused; the nominal small planning probe is not a robot footprint.
        """
        radius = self.planning_radius_m + self.layout.resolution_m / 2
        checks = []
        errors = []
        baseline_failures = []
        for left, right in sorted(self.edges):
            start, end = self.nodes[left], self.nodes[right]
            distance = math.dist(start, end)
            count = max(1, math.ceil(distance / self.layout.resolution_m))
            variant_bad = baseline_bad = None
            for index in range(count + 1):
                fraction = index / count
                point = (start[0] + fraction * (end[0] - start[0]), start[1] + fraction * (end[1] - start[1]))
                if variant_bad is None and _bad_touched_cells(self.layout, point, radius):
                    variant_bad = point
                if baseline_bad is None and _bad_touched_cells(self.base_layout, point, radius):
                    baseline_bad = point
                if variant_bad is not None and baseline_bad is not None:
                    break
            if variant_bad is not None:
                errors.append({"edge": [left, right], "first_bad_xy": variant_bad})
            if baseline_bad is not None:
                baseline_failures.append({"edge": [left, right], "first_bad_xy": baseline_bad})
            checks.append({"edge": [left, right], "distance_m": distance, "variant_passed": variant_bad is None,
                           "base_passed": baseline_bad is None, "sample_count": count + 1})
        routes = []
        anchors = sorted({**self.layout.stations, **self.layout.pockets})
        barriers = self.layout.blocked_aisles
        for closed in (False, True):
            for start in anchors:
                for goal in anchors:
                    if start == goal:
                        continue
                    route = self.plan(start, goal, closed=closed)
                    if closed and any(segment_rectangle_distance(a, b, barrier) <= radius
                                      for a, b in zip(route.points, route.points[1:]) for barrier in barriers):
                        errors.append({"route_id": route.route_id, "error": "closed-state route envelope touches barrier"})
                    routes.append(route.to_dict())
        preserved = all(self.layout.data[key] == self.base_layout.data[key] for key in ("stations", "pockets", "physical_obstacles"))
        if not preserved:
            raise LayoutError("G3a may only change authored policy passage widths")
        report: dict[str, Any] = {"schema_version": 1, "backend": "authored_route_preflight", "primary_evaluation": False,
                "passed": not errors, "errors": errors, "edge_checks": checks, "base_layout_failed_edges": baseline_failures,
                "planning_radius_m": self.planning_radius_m, "evaluation_radius_m": self.evaluation_radius_m,
                "sampling_cover_radius_m": radius, "config_sha256": self.config_sha256,
                "base_layout_source_sha256": self.base_layout.source_sha256, "variant_layout_sha256": self.layout.source_sha256,
                "single_lane": {"minimum_passing_width_planning_m": 4 * self.planning_radius_m,
                                "minimum_passing_width_evaluation_m": 4 * self.evaluation_radius_m,
                                "scope": "Two conservative circular envelopes, each clear of both walls; current corridors require capacity-one conflict protection"},
                "routes": routes, "route_count": len(routes), "anchors_preserved": preserved,
                "scope": "Authored geometric route/tile compilation only. No Nav2 path, action, reservation grant, drive performance or GPU execution is asserted."}
        if self.directed_edges:
            report["directed_graph"] = {"one_way_edges": len(self.directed_edges),
                "bidirectional_edges": len(self.edges) - len({tuple(sorted(arc)) for arc in self.directed_edges}),
                "scope": "one-way authored lane graph; shared junctions and docking branches still require reservation admission"}
            report["single_lane"]["scope"] = "Circular-envelope width calculation only; directed lanes do not bypass shared junction or docking reservations"
        return report


__all__ = [
    "DispatchRouteCompatibility",
    "RouteBook",
    "RoutePlan",
    "Tile",
    "assess_dispatch_route_compatibility",
    "segment_rectangle_distance",
    "swept_tiles",
]
