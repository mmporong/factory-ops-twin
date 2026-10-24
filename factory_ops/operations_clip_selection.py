"""Bounded rolling selection of a meaningful process-line media window."""

from __future__ import annotations

from collections import Counter, deque
from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Any


LOGISTICS_TASK_TYPES = frozenset({"process_input", "product_move", "empty_return"})


class ClipSelectionError(ValueError):
    pass


@dataclass(frozen=True)
class _Counters:
    shipments: int
    returns: int
    completed_transports: int
    deliveries: tuple[tuple[str, int], ...]


@dataclass(frozen=True)
class _Edge:
    logistics_distance_m: float
    moving_robot_ids: frozenset[str]


def _nonnegative_int(value: Any, label: str) -> int:
    if type(value) is not int or value < 0:
        raise ClipSelectionError(f"{label} must be a nonnegative integer")
    return value


def _finite(value: Any, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ClipSelectionError(f"{label} must be finite")
    return float(value)


def _counters(snapshot: Mapping[str, Any]) -> _Counters:
    line = snapshot.get("production_line")
    traffic = snapshot.get("traffic")
    if not isinstance(line, Mapping) or not isinstance(traffic, Mapping):
        raise ClipSelectionError("snapshot requires production_line and traffic mappings")
    deliveries = traffic.get("deliveries")
    if not isinstance(deliveries, Mapping) or any(not isinstance(key, str) or not key for key in deliveries):
        raise ClipSelectionError("traffic.deliveries must be keyed by robot ID")
    rows = tuple(sorted(
        (robot_id, _nonnegative_int(value, f"traffic.deliveries[{robot_id}]"))
        for robot_id, value in deliveries.items()
    ))
    return _Counters(
        shipments=_nonnegative_int(line.get("completed_lots"), "production_line.completed_lots"),
        returns=_nonnegative_int(line.get("returned"), "production_line.returned"),
        completed_transports=sum(value for _, value in rows),
        deliveries=rows,
    )


def _robots(snapshot: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    rows = snapshot.get("robots")
    if not isinstance(rows, list):
        raise ClipSelectionError("snapshot.robots must be a list")
    result: dict[str, Mapping[str, Any]] = {}
    for robot in rows:
        if not isinstance(robot, Mapping) or not isinstance(robot.get("id"), str) or not robot["id"]:
            raise ClipSelectionError("every robot requires a nonempty ID")
        if robot["id"] in result:
            raise ClipSelectionError(f"duplicate robot ID: {robot['id']}")
        result[robot["id"]] = robot
    return result


def _pose(robot: Mapping[str, Any], robot_id: str) -> tuple[float, float]:
    pose = robot.get("pose")
    if not isinstance(pose, Mapping):
        raise ClipSelectionError(f"robot pose is absent: {robot_id}")
    return _finite(pose.get("x_m"), f"{robot_id}.pose.x_m"), _finite(
        pose.get("y_m"), f"{robot_id}.pose.y_m"
    )


def _edge(previous: Mapping[str, Any], current: Mapping[str, Any]) -> _Edge:
    before, after = _robots(previous), _robots(current)
    distance = 0.0
    moved: set[str] = set()
    for robot_id in before.keys() & after.keys():
        left, right = before[robot_id], after[robot_id]
        # A task change can coincide with a station teleport in an authored model.
        # Count only one stable logistics task edge, including payload-free pickup approach.
        task_id = left.get("task_id")
        if (not isinstance(task_id, str) or not task_id
                or right.get("task_id") != task_id
                or left.get("task_type") not in LOGISTICS_TASK_TYPES
                or right.get("task_type") not in LOGISTICS_TASK_TYPES):
            continue
        x0, y0 = _pose(left, robot_id)
        x1, y1 = _pose(right, robot_id)
        travelled = math.hypot(x1 - x0, y1 - y0)
        if travelled > 1e-9:
            distance += travelled
            moved.add(robot_id)
    return _Edge(distance, frozenset(moved))


class OperationsClipSelector:
    """Retain at most one current window and one best window of immutable snapshots."""

    def __init__(self, sample_count: int = 300, *, time_tolerance_s: float = 1e-4) -> None:
        if type(sample_count) is not int or not 1 <= sample_count <= 300:
            raise ClipSelectionError("sample_count must be an integer in 1..300")
        if not math.isfinite(time_tolerance_s) or not 0 <= time_tolerance_s < 0.5:
            raise ClipSelectionError("time_tolerance_s must be finite and in [0, 0.5)")
        self.sample_count = sample_count
        self.time_tolerance_s = float(time_tolerance_s)
        self._samples: deque[Mapping[str, Any]] = deque()
        self._sample_times: deque[float] = deque()
        self._sample_counters: deque[_Counters] = deque()
        self._edges: deque[_Edge] = deque()
        self._distance_m = 0.0
        self._moving_edge_counts: Counter[str] = Counter()
        self._best: tuple[Mapping[str, Any], ...] = ()
        self._best_evidence: dict[str, Any] | None = None
        self._best_score: tuple[int, int, int, float, int] | None = None
        self._last_offered_time: float | None = None
        self._next_sample_time: float | None = None

    def offer(self, snapshot: Mapping[str, Any]) -> bool:
        """Offer a fresh immutable observation; return whether it became a 1 Hz sample."""
        if not isinstance(snapshot, Mapping):
            raise ClipSelectionError("snapshot must be a mapping")
        stamp = _finite(snapshot.get("sim_time_s"), "snapshot.sim_time_s")
        if stamp < 0:
            raise ClipSelectionError("snapshot.sim_time_s must be nonnegative")
        if self._last_offered_time is not None and stamp <= self._last_offered_time:
            raise ClipSelectionError("snapshot.sim_time_s must increase monotonically")
        self._last_offered_time = stamp
        if self._next_sample_time is not None:
            if stamp < self._next_sample_time - self.time_tolerance_s:
                return False
            if stamp > self._next_sample_time + self.time_tolerance_s:
                raise ClipSelectionError("snapshot stream skipped the next 1 Hz sample")

        counters = _counters(snapshot)
        if self._sample_counters:
            previous = self._sample_counters[-1]
            if (counters.shipments < previous.shipments or counters.returns < previous.returns
                    or counters.completed_transports < previous.completed_transports):
                raise ClipSelectionError("cumulative counters must not decrease")
            old_deliveries, new_deliveries = dict(previous.deliveries), dict(counters.deliveries)
            if set(old_deliveries) != set(new_deliveries) or any(
                new_deliveries[key] < value for key, value in old_deliveries.items()
            ):
                raise ClipSelectionError("per-robot delivery counters must retain keys and not decrease")
            edge = _edge(self._samples[-1], snapshot)
            if len(self._samples) == self.sample_count:
                self._samples.popleft()
                self._sample_times.popleft()
                self._sample_counters.popleft()
                removed = self._edges.popleft()
                self._remove_edge(removed)
            self._edges.append(edge)
            self._add_edge(edge)

        self._samples.append(snapshot)
        self._sample_times.append(stamp)
        self._sample_counters.append(counters)
        self._next_sample_time = stamp + 1.0
        self._consider_current_window()
        return True

    def _add_edge(self, edge: _Edge) -> None:
        self._distance_m += edge.logistics_distance_m
        self._moving_edge_counts.update(edge.moving_robot_ids)

    def _remove_edge(self, edge: _Edge) -> None:
        self._distance_m -= edge.logistics_distance_m
        for robot_id in edge.moving_robot_ids:
            self._moving_edge_counts[robot_id] -= 1
            if self._moving_edge_counts[robot_id] <= 0:
                del self._moving_edge_counts[robot_id]

    def _consider_current_window(self) -> None:
        first, last = self._sample_counters[0], self._sample_counters[-1]
        shipment_delta = last.shipments - first.shipments
        return_delta = last.returns - first.returns
        transport_delta = last.completed_transports - first.completed_transports
        score = (
            int(shipment_delta > 0 and return_delta > 0),
            min(shipment_delta, return_delta),
            transport_delta,
            self._distance_m,
            len(self._moving_edge_counts),
        )
        if (self._best_score is not None and score < self._best_score) or (
            self._best_score == score and len(self._samples) <= len(self._best)
        ):
            return
        self._best_score = score
        self._best = tuple(self._samples)
        self._best_evidence = {
            "scope": (
                "authored process-line snapshots; selection only, not case validity, "
                "physics, sensors, throughput, or completion proof"
            ),
            "sample_hz": 1,
            "configured_sample_count": self.sample_count,
            "selected_sample_count": len(self._samples),
            "first": {"sim_time_s": self._sample_times[0], **self._counter_document(first)},
            "last": {"sim_time_s": self._sample_times[-1], **self._counter_document(last)},
            "score": {
                "shipment_and_return_both_increased": bool(score[0]),
                "minimum_shipment_return_delta": score[1],
                "completed_transport_delta": score[2],
                "logistics_pose_distance_m": score[3],
                "distinct_moving_logistics_robots": score[4],
                "priority_order": [
                    "shipment_and_return_both_increased",
                    "minimum_shipment_return_delta",
                    "completed_transport_delta",
                    "logistics_pose_distance_m",
                    "distinct_moving_logistics_robots",
                ],
            },
            "movement_scope": {
                "included_task_types": sorted(LOGISTICS_TASK_TYPES),
                "excluded_task_types": ["traffic_yield", "parking", "charging"],
                "payload_required": False,
                "task_id_change_edges": "excluded",
            },
        }

    @staticmethod
    def _counter_document(value: _Counters) -> dict[str, int]:
        return {
            "shipments_completed": value.shipments,
            "returns_completed": value.returns,
            "transports_completed": value.completed_transports,
        }

    def best_snapshots(self) -> tuple[Mapping[str, Any], ...]:
        return self._best

    def evidence(self) -> dict[str, Any]:
        if self._best_evidence is None:
            return {
                "scope": "selection only; no snapshot has been accepted",
                "sample_hz": 1,
                "configured_sample_count": self.sample_count,
                "selected_sample_count": 0,
                "first": None,
                "last": None,
                "score": None,
            }
        return dict(self._best_evidence)
