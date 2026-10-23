"""Typed G3b adapter helpers without ROS, simulator, or physical-run claims."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
from hashlib import sha256
import json
import math
from typing import Any, Mapping, Sequence

from .fleet_routing import Point, RouteBook, Tile, swept_tiles
from .navigation import GoalPose


CHUNK_SCOPE = "g3b_authored_first_edge_planning_chunk_v1"
COLLINEAR_CHUNK_SCOPE = "g3b_authored_bounded_collinear_planning_chunk_v1"
PATH_SCOPE = "g3b_computed_path_geometry_only_not_ros_or_physical_evidence"
FOLLOW_PATH_NORMALIZATION = "final_pose_header_stamp_zero_latest_tf"


class StaleComputedPathError(ValueError):
    """An otherwise valid geometry remains inadmissible at its original expired stamp."""

    def __init__(self, stamp_s: float, now_s: float, max_age_s: float, tiles: frozenset[Tile]):
        super().__init__("computed path is stale")
        self.stamp_s, self.now_s, self.max_age_s = stamp_s, now_s, max_age_s
        self.age_s, self.swept_tiles = now_s-stamp_s, tiles


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be finite numeric data")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite numeric data")
    return result


def _point(value: Sequence[float], label: str) -> Point:
    if len(value) != 2:
        raise ValueError(f"{label} must contain exactly two coordinates")
    return (_number(value[0], f"{label}.x"), _number(value[1], f"{label}.y"))


def _hash(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return sha256(payload.encode("utf-8")).hexdigest()


def path_content_sha256(payload: Mapping[str, Any]) -> str:
    """Hash all decoded Path fields independently of transport serialization."""
    return _hash(payload)


def path_header_stamp_ns(header: Mapping[str, Any]) -> int:
    """Preserve the ROS timestamp's integer boundary before converting seconds."""
    seconds, nanoseconds = header.get("sec"), header.get("nanosec")
    if (type(seconds) is not int or seconds < 0 or type(nanoseconds) is not int
            or not 0 <= nanoseconds < 1_000_000_000):
        raise ValueError("Path header requires nonnegative sec and canonical nanosec integers")
    return seconds * 1_000_000_000 + nanoseconds


def normalize_follow_path_payload(planner_payload: Mapping[str, Any]) -> dict[str, Any]:
    """Copy a planner Path and request latest TF only for its terminal pose."""
    header = planner_payload.get("header")
    if not isinstance(header, Mapping):
        raise ValueError("planner Path header must be a mapping")
    path_header_stamp_ns(header)
    poses = planner_payload.get("poses")
    if not isinstance(poses, Sequence) or isinstance(poses, (str, bytes)) or not poses:
        raise ValueError("planner Path requires at least one pose")
    for index, pose in enumerate(poses):
        if not isinstance(pose, Mapping):
            raise ValueError(f"planner Path pose {index} must be a mapping")
        pose_header = pose.get("header")
        if not isinstance(pose_header, Mapping):
            raise ValueError(f"planner Path pose {index} header must be a mapping")
        path_header_stamp_ns(pose_header)
    normalized = deepcopy(dict(planner_payload))
    normalized["poses"][-1]["header"]["sec"] = 0
    normalized["poses"][-1]["header"]["nanosec"] = 0
    return normalized


def validate_follow_path_normalization(
    planner_payload: Mapping[str, Any], follow_payload: Mapping[str, Any], normalization: object,
) -> None:
    """Allow exactly the terminal pose stamp-zero transform recorded by the runtime."""
    if normalization != FOLLOW_PATH_NORMALIZATION:
        raise ValueError("FollowPath normalization kind is unsupported")
    poses = planner_payload.get("poses")
    if (not isinstance(poses, Sequence) or isinstance(poses, (str, bytes)) or not poses
            or not isinstance(poses[-1], Mapping)):
        raise ValueError("planner terminal pose requires a nonzero source stamp")
    terminal_header = poses[-1].get("header")
    if not isinstance(terminal_header, Mapping) or path_header_stamp_ns(terminal_header) == 0:
        raise ValueError("planner terminal pose requires a nonzero source stamp")
    if normalize_follow_path_payload(planner_payload) != follow_payload:
        raise ValueError("FollowPath differs beyond the terminal pose stamp-zero normalization")


def path_clock_ready(header: Mapping[str, Any], *, clock_ns: int,
                     elapsed_wall_s: float, max_wait_wall_s: float) -> bool:
    """Keep HOLD while a separate ROS clock callback catches up to a Path."""
    stamp = path_header_stamp_ns(header)
    elapsed, limit = _number(elapsed_wall_s, "elapsed_wall_s"), _number(max_wait_wall_s, "max_wait_wall_s")
    if type(clock_ns) is not int or clock_ns < 0 or elapsed < 0 or limit <= 0:
        raise ValueError("Path clock admission requires a nonnegative clock/wait and positive wall bound")
    if stamp <= clock_ns:
        return True
    if elapsed >= limit:
        raise TimeoutError(f"Path clock synchronization exceeded wall bound: path_ns={stamp}, clock_ns={clock_ns}")
    return False


def _tile_list(tiles: frozenset[Tile]) -> list[list[int]]:
    return [[x, y] for x, y in sorted(tiles)]


@dataclass(frozen=True, slots=True)
class RouteChunk:
    route_id: str
    start: str
    goal: str
    points: tuple[Point, ...]
    tiles: frozenset[Tile]
    goal_pose: GoalPose
    final: bool
    target: str
    closed: bool
    final_goal: GoalPose
    policy: str = "first_edge"

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["points"] = [list(point) for point in self.points]
        value["tiles"] = _tile_list(self.tiles)
        value["goal_pose"] = self.goal_pose.to_dict()
        value["final_goal"] = self.final_goal.to_dict()
        if self.policy == "first_edge":
            value.pop("policy")
        return value


def make_chunk(
    book: RouteBook,
    start: str,
    target: str,
    *,
    closed: bool,
    final_goal: GoalPose,
    policy: str = "first_edge",
) -> RouteChunk:
    """Compile one edge, or a straight prefix bounded by the graph's longest edge.

    The optional prefix retains every authored point and stops before a bend.
    Its length cannot exceed an existing single-edge action in this RouteBook.
    Reservation ownership and the final station orientation are unchanged.
    """
    if not isinstance(book, RouteBook):
        raise ValueError("book must be a RouteBook")
    if not isinstance(final_goal, GoalPose):
        raise ValueError("final_goal must be a GoalPose")
    if policy not in ("first_edge", "bounded_collinear"):
        raise ValueError("unsupported route chunk policy")
    plan = book.plan(start, target, closed=closed)
    target_xy = book.nodes[target]
    if math.dist((final_goal.x, final_goal.y), target_xy) > 1e-9:
        raise ValueError("final_goal position must match the authored target anchor")

    if start == target:
        chunk_goal = start
        points = (book.nodes[start],)
        final = True
        yaw = final_goal.yaw
    else:
        end = 1
        if policy == "bounded_collinear":
            lengths = [math.dist(a, b) for a, b in zip(plan.points, plan.points[1:])]
            limit = max(math.dist(book.nodes[a], book.nodes[b]) for a, b in book.edges)
            distance = lengths[0]
            direction = (plan.points[1][0] - plan.points[0][0],
                         plan.points[1][1] - plan.points[0][1])
            while end + 1 < len(plan.points):
                following = (plan.points[end + 1][0] - plan.points[end][0],
                             plan.points[end + 1][1] - plan.points[end][1])
                product = direction[0] * following[0] + direction[1] * following[1]
                cross = direction[0] * following[1] - direction[1] * following[0]
                scale = math.hypot(*direction) * lengths[end]
                if (product <= 0 or abs(cross) > 1e-10 * scale
                        or distance + lengths[end] > limit + 1e-9):
                    break
                distance += lengths[end]
                end += 1
        chunk_goal = plan.nodes[end]
        points = tuple(plan.points[:end + 1])
        final = chunk_goal == target
        yaw = final_goal.yaw if final else math.atan2(
            points[1][1] - points[0][1], points[1][0] - points[0][0]
        )
    goal_xy = book.nodes[chunk_goal]
    goal_pose = GoalPose("map", goal_xy[0], goal_xy[1], yaw)
    tiles = swept_tiles(
        points,
        book.planning_radius_m,
        tile_size_m=book.tile_size_m,
        bounds_xy=(book.layout.bounds.xmin, book.layout.bounds.ymin,
                   book.layout.bounds.xmax, book.layout.bounds.ymax),
    )
    identity = {
        "scope": CHUNK_SCOPE if policy == "first_edge" else COLLINEAR_CHUNK_SCOPE,
        "config_sha256": book.config_sha256,
        "variant_layout_sha256": book.layout.source_sha256,
        "base_layout_sha256": book.base_layout.source_sha256,
        "planning_radius_m": book.planning_radius_m,
        "evaluation_radius_m": book.evaluation_radius_m,
        "tile_size_m": book.tile_size_m,
        "closed": closed,
        "start": start,
        "target": target,
        "chunk_goal": chunk_goal,
        "points": points,
        "goal_pose": goal_pose.to_dict(),
        "final_goal": final_goal.to_dict(),
        "final": final,
    }
    return RouteChunk(
        _hash(identity), start, chunk_goal, points, tiles, goal_pose, final,
        target, closed, final_goal, policy,
    )


def qualify_computed_path(
    book: RouteBook,
    chunk: RouteChunk,
    points: Sequence[Sequence[float]],
    *,
    frame_id: str,
    stamp_s: float,
    now_s: float,
    start_xy: Sequence[float],
    max_age_s: float = 0.5,
) -> dict[str, Any]:
    """Validate computed path geometry against one authored reservation chunk."""
    if not isinstance(book, RouteBook) or not isinstance(chunk, RouteChunk):
        raise ValueError("book and chunk types are required")
    canonical = make_chunk(
        book,
        chunk.start,
        chunk.target,
        closed=chunk.closed,
        final_goal=chunk.final_goal,
        policy=chunk.policy,
    )
    if chunk != canonical:
        raise ValueError("route chunk differs from the current authored RouteBook compilation")
    if frame_id != "map":
        raise ValueError("computed path frame must be map")
    stamp = _number(stamp_s, "stamp_s")
    now = _number(now_s, "now_s")
    age = _number(max_age_s, "max_age_s")
    if stamp < 0 or now < 0 or age < 0:
        raise ValueError("path timestamps and max age must be nonnegative")
    if stamp > now:
        raise ValueError("computed path stamp is in the future")
    samples = tuple(_point(point, f"points[{index}]") for index, point in enumerate(points))
    if not samples:
        raise ValueError("computed path must contain at least one point")
    zero_distance = chunk.start == chunk.goal and len(chunk.points) == 1
    if not zero_distance and len(samples) < 2:
        raise ValueError("nonzero computed path must contain at least two points")
    observed_start = _point(start_xy, "start_xy")
    if math.dist(observed_start, chunk.points[0]) > 0.15:
        raise ValueError("observed start is outside authored chunk start tolerance")
    if math.dist(samples[0], observed_start) > 0.15:
        raise ValueError("computed path start is outside current start tolerance")
    if math.dist(samples[-1], (chunk.goal_pose.x, chunk.goal_pose.y)) > 0.15:
        raise ValueError("computed path goal is outside chunk goal tolerance")
    bounds = (
        book.layout.bounds.xmin,
        book.layout.bounds.ymin,
        book.layout.bounds.xmax,
        book.layout.bounds.ymax,
    )
    actual_tiles = swept_tiles(
        samples,
        book.evaluation_radius_m,
        tile_size_m=book.tile_size_m,
        bounds_xy=bounds,
    )
    if not actual_tiles.issubset(chunk.tiles):
        raise ValueError("computed path swept envelope leaves reserved chunk tiles")
    if now - stamp > age:
        raise StaleComputedPathError(stamp, now, age, actual_tiles)
    point_list = [[x, y] for x, y in samples]
    tile_list = _tile_list(actual_tiles)
    return {
        "passed": True,
        "scope": PATH_SCOPE,
        "frame_id": "map",
        "stamp_s": stamp,
        "now_s": now,
        "planning_radius_m": book.planning_radius_m,
        "evaluation_radius_m": book.evaluation_radius_m,
        "points": point_list,
        "points_sha256": _hash(point_list),
        "swept_tiles": tile_list,
        "swept_tiles_sha256": _hash(tile_list),
        "chunk_route_id": chunk.route_id,
    }


class StoppedGate:
    """Fail-closed stationary-window gate over caller-supplied odometry values."""

    def __init__(
        self,
        *,
        window_s: float = 0.5,
        max_linear_m_s: float = 0.02,
        max_angular_rad_s: float = 0.03,
        ttl_s: float = 0.3,
    ) -> None:
        self.window_s = _number(window_s, "window_s")
        self.max_linear_m_s = _number(max_linear_m_s, "max_linear_m_s")
        self.max_angular_rad_s = _number(max_angular_rad_s, "max_angular_rad_s")
        self.ttl_s = _number(ttl_s, "ttl_s")
        if self.window_s <= 0 or self.max_linear_m_s < 0 or self.max_angular_rad_s < 0 or self.ttl_s <= 0:
            raise ValueError("stopped-gate limits are out of range")
        self._stationary_since_stamp: float | None = None
        self._newest_stamp: float | None = None
        self._last_now: float = 0.0
        self._last_stamp: float | None = None
        self._last_velocity: tuple[float, float] | None = None
        self._samples: list[dict[str, float]] = []

    def update(
        self,
        linear_m_s: float,
        angular_rad_s: float,
        *,
        stamp_s: float,
        now_s: float,
    ) -> bool:
        try:
            linear = _number(linear_m_s, "linear_m_s")
            angular = _number(angular_rad_s, "angular_rad_s")
            stamp = _number(stamp_s, "stamp_s")
            now = _number(now_s, "now_s")
        except ValueError:
            self._clear_window()
            raise
        if stamp < 0 or now < self._last_now:
            self._clear_window()
            raise ValueError("odometry timestamps must be nonnegative and now_s monotonic")
        self._last_now = now
        if stamp > now:
            self._clear_window()
            raise ValueError("odometry stamp is in the future")
        if self._last_stamp is not None and stamp < self._last_stamp:
            self._clear_window()
            raise ValueError("odometry stamp must be monotonic")
        if stamp == self._last_stamp and self._last_velocity != (linear, angular):
            self._clear_window()
            raise ValueError("odometry stamp reused with different velocity")
        self._last_stamp = stamp
        self._last_velocity = (linear, angular)
        if now - stamp > self.ttl_s:
            self._clear_window()
            return False
        if self._newest_stamp is not None and stamp < self._newest_stamp:
            self._clear_window()
            raise ValueError("odometry stamp must be monotonic")
        if abs(linear) > self.max_linear_m_s or abs(angular) > self.max_angular_rad_s:
            self._clear_window()
            return False
        if self._newest_stamp is None:
            self._stationary_since_stamp = stamp
            self._newest_stamp = stamp
        elif stamp > self._newest_stamp:
            gap = stamp - self._newest_stamp
            ttl_boundary = math.isclose(gap, self.ttl_s, rel_tol=1e-12, abs_tol=1e-12)
            if gap > self.ttl_s and not ttl_boundary:
                self._stationary_since_stamp = stamp
                self._samples = []
            self._newest_stamp = stamp
        if not self._samples or stamp > self._samples[-1]["stamp_s"]:
            self._samples.append({"stamp_s": stamp, "linear_m_s": linear, "angular_rad_s": angular})
        return self.is_stopped(now_s=now)

    def is_stopped(self, *, now_s: float) -> bool:
        now = _number(now_s, "now_s")
        if now < self._last_now:
            raise ValueError("now_s must be monotonic")
        self._last_now = now
        if self._newest_stamp is None or self._stationary_since_stamp is None:
            return False
        if now - self._newest_stamp > self.ttl_s:
            self._clear_window()
            return False
        return self._newest_stamp - self._stationary_since_stamp >= self.window_s

    def _clear_window(self) -> None:
        self._stationary_since_stamp = None
        self._newest_stamp = None
        self._samples = []

    @property
    def samples(self) -> list[dict[str, float]]:
        return [dict(sample) for sample in self._samples]

    def reset(self) -> None:
        self._clear_window()
        self._last_now = 0.0
        self._last_stamp = None
        self._last_velocity = None


__all__ = [
    "CHUNK_SCOPE",
    "PATH_SCOPE",
    "RouteChunk",
    "StoppedGate",
    "make_chunk",
    "qualify_computed_path",
]
