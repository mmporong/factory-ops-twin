"""Stopped-anchor detours for persistent operations-mode reservation cycles."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any, Iterable, Mapping, Sequence

from .fleet_routing import swept_tiles


Point = tuple[float, float]
Tile = tuple[int, int]


@dataclass(frozen=True, slots=True)
class AlternativeRoute:
    nodes: tuple[str, ...]
    points: tuple[Point, ...]
    distance_m: float


def anchor_at_pose(nodes: Mapping[str, Point], pose: Mapping[str, Any], *, tolerance_m: float = 1e-8) -> str | None:
    """Return the unique authored anchor occupied by a stopped pose."""

    try:
        point = float(pose["x_m"]), float(pose["y_m"])
    except (KeyError, TypeError, ValueError):
        return None
    matches = [name for name, candidate in nodes.items() if math.dist(point, candidate) <= tolerance_m]
    return matches[0] if len(matches) == 1 else None


def find_alternative_route(
    book: Any,
    *,
    start: str,
    goal: str,
    closed: bool,
    reservation_radius_m: float,
    peer_footprint_tiles: Iterable[Tile],
    existing_points: Sequence[Sequence[float]],
) -> AlternativeRoute | None:
    """Find a different authored route whose edges avoid current peer footprints."""

    if start not in book.nodes or goal not in book.nodes:
        raise ValueError("alternative route endpoints must be authored nodes")
    if type(closed) is not bool:
        raise ValueError("closed must be a boolean")
    if (isinstance(reservation_radius_m, bool) or not isinstance(reservation_radius_m, (int, float))
            or not math.isfinite(reservation_radius_m) or reservation_radius_m <= 0):
        raise ValueError("reservation radius must be a finite positive number")
    peers = frozenset(peer_footprint_tiles)
    if any(not isinstance(tile, tuple) or len(tile) != 2 or any(type(value) is not int for value in tile)
           for tile in peers):
        raise ValueError("peer footprint tiles must contain integer pairs")
    existing = tuple((float(point[0]), float(point[1])) for point in existing_points)
    bounds = book.layout.bounds
    bounds_xy = bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax

    def shortest(excluded_arc: tuple[str, str] | None) -> AlternativeRoute | None:
        pending: list[tuple[float, tuple[str, ...]]] = [(0.0, (start,))]
        best: dict[str, tuple[float, tuple[str, ...]]] = {start: (0.0, (start,))}
        while pending:
            distance, names = heapq.heappop(pending)
            node = names[-1]
            if best[node] != (distance, names):
                continue
            if node == goal:
                points = tuple(book.nodes[name] for name in names)
                return AlternativeRoute(names, points, distance)
            for neighbor in sorted(book.adjacency[node]):
                if excluded_arc == (node, neighbor):
                    continue
                edge = tuple(sorted((node, neighbor)))
                if closed and edge in book.closed_edges:
                    continue
                edge_tiles = swept_tiles(
                    (book.nodes[node], book.nodes[neighbor]),
                    reservation_radius_m,
                    tile_size_m=book.tile_size_m,
                    bounds_xy=bounds_xy,
                )
                if not edge_tiles.isdisjoint(peers):
                    continue
                candidate = (
                    distance + math.dist(book.nodes[node], book.nodes[neighbor]),
                    (*names, neighbor),
                )
                if neighbor not in best or candidate < best[neighbor]:
                    best[neighbor] = candidate
                    heapq.heappush(pending, candidate)
        return None

    primary = shortest(None)
    if primary is None or primary.points[1:] != existing:
        return primary
    point_nodes = {point: name for name, point in book.nodes.items()}
    try:
        existing_nodes = (start, *(point_nodes[point] for point in existing))
    except KeyError:
        return None
    alternatives = [shortest(arc) for arc in zip(existing_nodes, existing_nodes[1:])]
    valid = [route for route in alternatives if route is not None and route.points[1:] != existing]
    return min(valid, key=lambda route: (route.distance_m, route.nodes)) if valid else None
