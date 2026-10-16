"""Traffic reservation and recovery decisions for the factory twin.

This module consumes explicit, caller-supplied *estimated* observations.  It
does not read simulator ground truth, execute Nav2 paths, teleport robots, or
claim that a command completed.  A future ROS adapter must provide route
membership, swept-footprint occupancy, exit clearance, and route costs.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence, TypedDict, Unpack


class TrafficError(ValueError):
    """Raised when an input violates the traffic-domain contract."""


class Thresholds(TypedDict):
    pose_freshness_s: float
    heartbeat_ttl_s: float
    cycle_persistence_s: float
    stall_threshold_s: float


class EventDetails(TypedDict, total=False):
    status: str
    motion_allowed: bool
    reason: str
    resources: tuple[str, ...]
    resource: str
    exit_id: str
    blockers: tuple[str, ...]
    cycle: tuple[str, ...]
    command: str
    target: str
    priority_robot: str
    route_cost: float
    goal_xy: tuple[float, float]
    kind: str
    reservation_id: int | None
    predecessor_reservation_id: int
    recovery_id: int
    waiter_pose_time_s: float
    blocker_pose_time_s: float
    planning_radius_m: float
    waiter_pose_xy: tuple[float, float]
    blocker_pose_xy: tuple[float, float]


class EventRecord(EventDetails):
    run_id: str
    event_seq: int
    sim_time_s: float
    event_type: str
    robot_id: str


class FreshnessSnapshot(TypedDict):
    pose_fresh: bool
    heartbeat_fresh: bool
    motion_allowed: bool


class ActiveReservationSnapshot(TypedDict):
    resources: tuple[str, ...]
    exit_id: str
    reservation_id: int


class ReleaseAckSnapshot(TypedDict):
    reservation_id: int
    owner_robot_id: str
    resources: tuple[str, ...]
    exit_id: str
    expected_waiter: str | None
    consumed_by: str | None
    superseded_by: int | None


class PendingSnapshot(TypedDict):
    robot_id: str
    resources: tuple[str, ...]
    exit_id: str
    request_time: float


class TrafficSnapshot(TypedDict):
    backend: str
    run_id: str
    sim_time_s: float
    thresholds: Thresholds
    resource_owner: dict[str, str]
    exit_owner: dict[str, str]
    active_reservations: dict[str, ActiveReservationSnapshot]
    release_acks: list[ReleaseAckSnapshot]
    pending: list[PendingSnapshot]
    freshness: dict[str, FreshnessSnapshot]


class EventsPage(TypedDict):
    run_id: str
    cursor: int
    next_cursor: int
    events: tuple[EventRecord, ...]


class _ReleaseAck(TypedDict):
    owner_robot_id: str
    resources: tuple[str, ...]
    exit_id: str
    consumed_by: str | None
    superseded_by: int | None


@dataclass(frozen=True)
class Decision:
    granted: bool
    reason: str
    robot_id: str
    resources: tuple[str, ...] = ()
    exit_id: str | None = None
    command: str | None = None
    target: str | None = None
    reservation_id: int | None = None
    recovery_id: int | None = None


@dataclass
class Observation:
    pose_xy: tuple[float, float]
    pose_time: float
    heartbeat_time: float
    occupied_resources: frozenset[str]
    clear_exits: frozenset[str]
    status: str
    motion_allowed: bool


@dataclass(frozen=True)
class _Active:
    resources: tuple[str, ...]
    exit_id: str
    reservation_id: int


@dataclass(order=True)
class _Pending:
    sort_key: tuple[float, str] = field(init=False, repr=False)
    request_time: float
    robot_id: str
    resources: tuple[str, ...]
    exit_id: str

    def __post_init__(self) -> None:
        self.sort_key = (self.request_time, self.robot_id)


class TrafficController:
    """Capacity-one reservation domain driven only by estimated observations."""

    pose_freshness_s = 1.0
    heartbeat_ttl_s = 3.0
    cycle_persistence_s = 5.0
    stall_threshold_s = 60.0

    def __init__(
        self,
        resources: Mapping[str, Mapping[str, object]],
        pockets: Mapping[str, Sequence[float]],
        *,
        frame: str = "map",
        units: str = "m",
        run_id: str = "traffic-domain",
    ) -> None:
        if frame != "map":
            raise TrafficError("traffic frame must be 'map'")
        if units != "m":
            raise TrafficError("traffic coordinates must use metres ('m')")
        if not resources:
            raise TrafficError("at least one resource is required")

        self.frame = frame
        self.units = units
        if not isinstance(run_id, str) or not run_id:
            raise TrafficError("run_id must be a non-empty string")
        self.run_id = run_id
        self.resources: dict[str, dict[str, object]] = {}
        self.valid_exits: set[str] = set()
        for resource_id, raw in resources.items():
            capacity = raw.get("capacity", 1)
            if capacity != 1:
                raise TrafficError(f"{resource_id}: only capacity 1 is supported")
            ports = raw.get("ports")
            if not isinstance(ports, Mapping) or not ports:
                raise TrafficError(f"{resource_id}: ports are required")
            polygon = raw.get("polygon_xy")
            if not isinstance(polygon, Sequence) or len(polygon) < 3:
                raise TrafficError(f"{resource_id}: polygon needs at least 3 points")
            for point in polygon:
                self._validate_xy(point, f"{resource_id}.polygon")
            for port_name, port in ports.items():
                if not isinstance(port, Mapping):
                    raise TrafficError(f"{resource_id}.{port_name}: invalid port")
                self._validate_xy(port.get("xy"), f"{resource_id}.{port_name}")
                self.valid_exits.add(f"{resource_id}:{port_name}")
            self.resources[resource_id] = dict(raw)

        self.pockets: dict[str, tuple[float, float]] = {}
        for pocket_id, point in pockets.items():
            self.pockets[pocket_id] = self._validate_xy(point, pocket_id)

        self._resource_owner: dict[str, str] = {}
        self._exit_owner: dict[str, str] = {}
        self._robot_holds: dict[str, set[str]] = {}
        self._active_reservations: dict[str, _Active] = {}
        self._observations: dict[str, Observation] = {}
        self._pending: list[_Pending] = []
        self._wait_edges: dict[tuple[str, str], float] = {}
        self._observed_exit_waits: dict[tuple[str, str], tuple[int, float]] = {}
        self._last_progress: dict[str, tuple[float, str]] = {}
        self._release_acks: dict[int, _ReleaseAck] = {}
        self._ack_expected_waiter: dict[int, str] = {}
        self._events: list[EventRecord] = []
        self._event_seq = 0
        self._reservation_seq = 0
        self._recovery_seq = 0
        self._last_time = 0.0

    @classmethod
    def from_file(cls, path: str | Path, *, run_id: str = "traffic-domain") -> "TrafficController":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        raw_resources = data.get("resources")
        if not isinstance(raw_resources, list):
            raise TrafficError("resources must be a list")
        resources: dict[str, Mapping[str, object]] = {}
        for raw in raw_resources:
            if not isinstance(raw, Mapping) or not isinstance(raw.get("id"), str):
                raise TrafficError("every resource requires a string id")
            resource_id = str(raw["id"])
            if resource_id in resources:
                raise TrafficError(f"duplicate resource: {resource_id}")
            resources[resource_id] = raw
        pockets = data.get("pockets", {})
        if not isinstance(pockets, Mapping):
            raise TrafficError("pockets must be an object")
        return cls(
            resources, pockets, frame=data.get("frame", ""),
            units=data.get("units", ""), run_id=run_id,
        )

    def update_observation(
        self,
        robot_id: str,
        *,
        pose_xy: Sequence[float],
        pose_time: float,
        heartbeat_time: float,
        occupied_resources: Iterable[str] = (),
        clear_exits: Iterable[str] = (),
        status: str = "available",
        motion_allowed: bool = True,
        now: float | None = None,
        frame: str = "map",
        units: str = "m",
    ) -> None:
        """Store estimated evidence; ``status`` is diagnostic, motion gate is explicit."""
        if frame != self.frame or units != self.units:
            raise TrafficError("observation frame/units do not match traffic config")
        pose = self._validate_xy(pose_xy, "pose_xy")
        pose_time = self._validate_time(pose_time, "pose_time", advance=False)
        heartbeat_time = self._validate_time(heartbeat_time, "heartbeat_time", advance=False)
        event_time = max(pose_time, heartbeat_time) if now is None else now
        event_time = self._validate_time(event_time, "now")
        if pose_time > event_time or heartbeat_time > event_time:
            raise TrafficError("observation timestamps cannot be in the future")
        previous = self._observations.get(robot_id)
        if previous and (pose_time < previous.pose_time or heartbeat_time < previous.heartbeat_time):
            raise TrafficError("per-robot observation timestamps cannot move backwards")
        occupied = frozenset(occupied_resources)
        unknown = occupied.difference(self.resources)
        if unknown:
            raise TrafficError(f"unknown occupied resource(s): {sorted(unknown)}")
        clear = frozenset(clear_exits)
        unknown_exits = clear.difference(self.valid_exits)
        if unknown_exits:
            raise TrafficError(f"unknown clear exit(s): {sorted(unknown_exits)}")
        if not isinstance(status, str) or not status:
            raise TrafficError("status must be a non-empty string")
        if not isinstance(motion_allowed, bool):
            raise TrafficError("motion_allowed must be boolean")

        self._observations[robot_id] = Observation(
            pose, pose_time, heartbeat_time, occupied, clear, status, motion_allowed
        )
        self._refresh_exit_waits(event_time)
        self._event(
            event_time, "observation", robot_id, status=status,
            motion_allowed=motion_allowed,
        )

    observe_pose = update_observation

    def heartbeat(self, robot_id: str, *, heartbeat_time: float, now: float | None = None) -> None:
        """Refresh heartbeat without inventing a pose update."""
        if robot_id not in self._observations:
            raise TrafficError("heartbeat requires an existing observation")
        heartbeat_time = self._validate_time(heartbeat_time, "heartbeat_time", advance=False)
        event_time = self._validate_time(heartbeat_time if now is None else now, "now")
        if heartbeat_time > event_time:
            raise TrafficError("heartbeat timestamp cannot be in the future")
        obs = self._observations[robot_id]
        if heartbeat_time < obs.heartbeat_time:
            raise TrafficError("heartbeat timestamp cannot move backwards")
        self._observations[robot_id] = Observation(
            obs.pose_xy, obs.pose_time, heartbeat_time, obs.occupied_resources,
            obs.clear_exits, obs.status, obs.motion_allowed
        )
        self._refresh_exit_waits(event_time)
        self._event(event_time, "heartbeat", robot_id)

    def request(
        self,
        robot_id: str,
        resources: Sequence[str],
        exit_id: str,
        *,
        now: float,
    ) -> Decision:
        """Atomically reserve all route resources and its exit holding point."""
        now = self._validate_time(now, "now")
        requested = tuple(resources)
        if not requested or len(set(requested)) != len(requested):
            raise TrafficError("resources must be a non-empty unique sequence")
        unknown = set(requested).difference(self.resources)
        if unknown:
            raise TrafficError(f"unknown resource(s): {sorted(unknown)}")
        if exit_id not in self.valid_exits:
            raise TrafficError(f"unknown exit: {exit_id}")
        if exit_id.split(":", 1)[0] not in requested:
            raise TrafficError("exit must belong to the atomic resource request")
        freshness = self._freshness_reason(robot_id, now)
        if freshness:
            self.cancel_pending(robot_id, now=now, reason="stale_requester")
            decision = Decision(
                False, freshness, robot_id, requested, exit_id,
                command="hold_at_entry",
            )
            self._event(now, "request_denied", robot_id, reason=freshness)
            return decision
        if not self._observations[robot_id].motion_allowed:
            self.cancel_pending(robot_id, now=now, reason="motion_blocked")
            decision = Decision(False, "motion_not_allowed", robot_id, requested, exit_id,
                                command="hold_at_entry")
            self._event(now, "request_denied", robot_id, reason=decision.reason)
            return decision
        active = self._active_reservations.get(robot_id)
        if active:
            if (active.resources, active.exit_id) == (requested, exit_id):
                return Decision(
                    True, "already_granted", robot_id, requested, exit_id,
                    command="proceed_to_entry", reservation_id=active.reservation_id,
                )
            raise TrafficError("robot already owns a different atomic reservation")

        same = next((p for p in self._pending if p.robot_id == robot_id), None)
        if same and (same.resources != requested or same.exit_id != exit_id):
            raise TrafficError("robot already has a different pending request")
        if not same:
            self._pending.append(_Pending(now, robot_id, requested, exit_id))
        self._pending.sort()
        return self._try_grant(robot_id, now)

    def release(
        self,
        robot_id: str,
        resources: Sequence[str],
        exit_id: str,
        *,
        now: float,
    ) -> Decision:
        """Release only after fresh estimated full-exit and destination clearance."""
        now = self._validate_time(now, "now")
        requested = tuple(resources)
        if not requested:
            raise TrafficError("release requires resources")
        active = self._active_reservations.get(robot_id)
        if active is None:
            decision = Decision(False, "not_owner", robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason="not_owner")
            return decision
        if (active.resources, active.exit_id) != (requested, exit_id):
            decision = Decision(False, "reservation_bundle_mismatch", robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason="reservation_bundle_mismatch")
            return decision
        if any(self._resource_owner.get(r) != robot_id for r in requested):
            decision = Decision(False, "not_owner", robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason="not_owner")
            return decision
        if self._exit_owner.get(exit_id) != robot_id:
            decision = Decision(False, "not_exit_owner", robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason="not_exit_owner")
            return decision
        freshness = self._freshness_reason(robot_id, now)
        if freshness:
            decision = Decision(False, freshness, robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason=freshness)
            return decision
        obs = self._observations[robot_id]
        if set(requested).intersection(obs.occupied_resources):
            decision = Decision(False, "footprint_not_clear", robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason="footprint_not_clear")
            return decision
        if exit_id not in obs.clear_exits:
            decision = Decision(False, "destination_or_port_not_clear", robot_id, requested, exit_id)
            self._event(now, "release_denied", robot_id, reason="destination_or_port_not_clear")
            return decision

        for resource_id in requested:
            del self._resource_owner[resource_id]
            self._robot_holds.get(robot_id, set()).discard(resource_id)
        del self._exit_owner[exit_id]
        reservation_id = active.reservation_id
        del self._active_reservations[robot_id]
        self._release_acks[reservation_id] = {
            "owner_robot_id": robot_id,
            "resources": requested,
            "exit_id": exit_id,
            "consumed_by": None,
            "superseded_by": None,
        }
        self._event(
            now, "release_ack", robot_id, resources=requested, exit_id=exit_id,
            reservation_id=reservation_id,
        )
        self._clear_waits_for(robot_id)
        self._grant_pending(now)
        self._assert_mapping_invariants()
        return Decision(
            True, "release_ack", robot_id, requested, exit_id,
            command="release_ack", reservation_id=reservation_id,
        )

    def claim_observed_occupancy(
        self, robot_id: str, resource_id: str, exit_id: str, *, now: float
    ) -> Decision:
        """Register a trial-start occupancy from fresh external evidence."""
        now = self._validate_time(now, "now")
        if resource_id not in self.resources or exit_id not in self.valid_exits:
            raise TrafficError("unknown resource or exit")
        if exit_id.split(":", 1)[0] != resource_id:
            raise TrafficError("claim exit must belong to resource")
        freshness = self._freshness_reason(robot_id, now)
        if freshness:
            return Decision(False, freshness, robot_id, (resource_id,), exit_id)
        obs = self._observations[robot_id]
        if resource_id not in obs.occupied_resources:
            return Decision(False, "occupancy_not_observed", robot_id, (resource_id,), exit_id)
        if any(pending.robot_id == robot_id for pending in self._pending):
            return Decision(
                False, "pending_request_conflict", robot_id,
                (resource_id,), exit_id, command="hold_at_entry",
            )
        active = self._active_reservations.get(robot_id)
        if active:
            if (active.resources, active.exit_id) == ((resource_id,), exit_id):
                return Decision(
                    True, "already_claimed", robot_id, (resource_id,), exit_id,
                    command=None if obs.motion_allowed else "request_stop",
                    reservation_id=active.reservation_id,
                )
            return Decision(
                False, "active_reservation_conflict", robot_id,
                (resource_id,), exit_id, reservation_id=active.reservation_id,
            )
        if resource_id in self._resource_owner or exit_id in self._exit_owner:
            return Decision(False, "occupied", robot_id, (resource_id,), exit_id)
        reservation_id = self._next_reservation(
            robot_id, (resource_id,), exit_id, now
        )
        self._resource_owner[resource_id] = robot_id
        self._exit_owner[exit_id] = robot_id
        self._robot_holds.setdefault(robot_id, set()).add(resource_id)
        self._active_reservations[robot_id] = _Active((resource_id,), exit_id, reservation_id)
        self._assert_mapping_invariants()
        self._event(
            now, "occupancy_claimed", robot_id, resource=resource_id,
            exit_id=exit_id, reservation_id=reservation_id,
        )
        return Decision(
            True, "observed_occupancy_claimed", robot_id,
            (resource_id,), exit_id,
            command=None if obs.motion_allowed else "request_stop",
            reservation_id=reservation_id,
        )

    def record_wait(
        self, robot_id: str, blockers: Iterable[str], *, now: float, reason: str = "reservation_wait"
    ) -> None:
        now = self._validate_time(now, "now")
        blockers = tuple(sorted(set(blockers)))
        if robot_id in blockers:
            raise TrafficError("robot cannot wait for itself")
        for blocker in blockers:
            self._wait_edges.setdefault((robot_id, blocker), now)
        self._event(now, "wait", robot_id, blockers=blockers, reason=reason)

    def detect_cycle(self, *, now: float) -> tuple[str, ...] | None:
        now = self._validate_time(now, "now")
        self._refresh_exit_waits(now)
        graph: dict[str, set[str]] = {}
        for (waiter, blocker), since in self._wait_edges.items():
            if now - since >= self.cycle_persistence_s:
                graph.setdefault(waiter, set()).add(blocker)
        for start in sorted(graph):
            stack: list[tuple[str, tuple[str, ...]]] = [(start, (start,))]
            while stack:
                node, path = stack.pop()
                for nxt in sorted(graph.get(node, ())):
                    if nxt == start and len(path) > 1:
                        return path
                    if nxt not in path:
                        stack.append((nxt, path + (nxt,)))
        return None

    def observe_exit_wait(
        self, robot_id: str, blocker_id: str, *, now: float, planning_radius_m: float
    ) -> bool:
        """Record a wait only for fresh estimated blockage of an owner's exit.

        The owner must already intersect its reserved resource.  A peer's
        circle blocks the destination circle when centre distance <= 2r.
        The edge is revalidated on every cycle query and cannot survive a
        stale pose, resource generation change, or cleared exit.
        """
        from factory_ops.traffic_observation import _radius

        now = self._validate_time(now, "now")
        radius = _radius(planning_radius_m)
        active = self._active_reservations.get(robot_id)
        edge = (robot_id, blocker_id)
        if active is None or not self._exit_blocked(robot_id, blocker_id, radius, now):
            if edge in self._observed_exit_waits:
                self._observed_exit_waits.pop(edge)
                self._wait_edges.pop(edge, None)
            return False
        previous = self._observed_exit_waits.get(edge)
        if previous != (active.reservation_id, radius):
            self._wait_edges.pop(edge, None)
        self._observed_exit_waits[edge] = (active.reservation_id, radius)
        self._wait_edges.setdefault(edge, now)
        self._event(
            now, "wait", robot_id, blockers=(blocker_id,), reason="observed_exit_blocked",
            exit_id=active.exit_id, reservation_id=active.reservation_id,
            waiter_pose_time_s=self._observations[robot_id].pose_time,
            blocker_pose_time_s=self._observations[blocker_id].pose_time,
            planning_radius_m=radius,
            waiter_pose_xy=self._observations[robot_id].pose_xy,
            blocker_pose_xy=self._observations[blocker_id].pose_xy,
        )
        return True

    def refresh_exit_waits(self, *, now: float) -> None:
        self._refresh_exit_waits(self._validate_time(now, "now"))

    def _refresh_exit_waits(self, now: float) -> None:
        for edge, (generation, radius) in list(self._observed_exit_waits.items()):
            active = self._active_reservations.get(edge[0])
            if (active is None or active.reservation_id != generation
                    or not self._exit_blocked(*edge, radius, now)):
                self._observed_exit_waits.pop(edge)
                self._wait_edges.pop(edge, None)

    def _exit_blocked(self, robot_id: str, blocker_id: str, radius: float, now: float) -> bool:
        from factory_ops.traffic_observation import exit_xy, project_observation

        if robot_id == blocker_id or any(self._freshness_reason(robot, now) for robot in (robot_id, blocker_id)):
            return False
        active = self._active_reservations.get(robot_id)
        if active is None:
            return False
        owner = self._observations[robot_id]
        peer = self._observations[blocker_id]
        occupied = project_observation(self, owner.pose_xy, radius)["occupied_resources"]
        return (bool(set(occupied).intersection(active.resources))
                and math.dist(peer.pose_xy, exit_xy(self, active.exit_id)) <= 2.0 * radius + 1e-9)

    def recovery_decision(
        self,
        *,
        now: float,
        pocket_route_costs: Mapping[str, Mapping[str, float | None]],
    ) -> Decision | None:
        """Return a recovery command request; this method never executes motion."""
        cycle = self.detect_cycle(now=now)
        if not cycle:
            return None
        recovery_id = self._next_recovery_id()
        for robot_id in sorted(cycle):
            reason = self._freshness_reason(robot_id, now)
            if reason or not self._observations[robot_id].motion_allowed:
                reason = reason or "motion_not_allowed"
                reservation_id = self._reservation_for(robot_id)
                self._event(
                    now, "recovery_blocked", robot_id, reason=reason,
                    cycle=cycle, recovery_id=recovery_id,
                    reservation_id=reservation_id,
                )
                return Decision(
                    False, reason, robot_id, command="request_stop",
                    reservation_id=reservation_id, recovery_id=recovery_id,
                )
        priority = min(cycle)
        yielding = max(cycle)
        reservation_id = self._reservation_for(yielding)
        raw_costs = pocket_route_costs.get(yielding, {})
        feasible: list[tuple[float, str]] = []
        for pocket_id in self.pockets:
            cost = raw_costs.get(pocket_id)
            if cost is not None and self._finite_number(cost) and float(cost) >= 0:
                feasible.append((float(cost), pocket_id))
        if not feasible:
            self._event(
                now, "recovery_fault", yielding, reason="no_feasible_pocket",
                cycle=cycle, recovery_id=recovery_id,
                reservation_id=reservation_id,
            )
            return Decision(
                False, "no_feasible_pocket", yielding, command="fault",
                reservation_id=reservation_id, recovery_id=recovery_id,
            )
        cost, pocket = min(feasible, key=lambda item: (item[0], item[1] != "P1", item[1]))
        self._event(
            now, "recovery_decision", yielding, command="cancel_and_yield",
            target=pocket, priority_robot=priority, route_cost=cost,
            recovery_id=recovery_id, reservation_id=reservation_id,
        )
        return Decision(
            True, "persistent_wait_cycle", yielding,
            command="cancel_and_yield", target=pocket,
            reservation_id=reservation_id, recovery_id=recovery_id,
        )

    def resume_after_release_ack(
        self,
        robot_id: str,
        *,
        owner_robot_id: str,
        exit_id: str,
        expected_reservation_id: int,
        goal_xy: Sequence[float],
        now: float,
    ) -> Decision:
        """Consume one generation-specific release ACK for one waiting robot."""
        now = self._validate_time(now, "now")
        goal = self._validate_xy(goal_xy, "goal_xy")
        if not isinstance(expected_reservation_id, int) or isinstance(expected_reservation_id, bool):
            raise TrafficError("expected_reservation_id must be an integer")
        freshness = self._freshness_reason(robot_id, now)
        if freshness:
            return Decision(
                False, freshness, robot_id, command="request_stop",
                reservation_id=expected_reservation_id,
            )
        if not self._observations[robot_id].motion_allowed:
            return Decision(
                False, "motion_not_allowed", robot_id, command="request_stop",
                reservation_id=expected_reservation_id,
            )
        ack = self._release_acks.get(expected_reservation_id)
        if not ack:
            active = self._active_reservations.get(owner_robot_id)
            if (
                active is None
                or active.reservation_id != expected_reservation_id
                or active.exit_id != exit_id
            ):
                return Decision(
                    False, "release_ack_generation_mismatch", robot_id,
                    command="hold", reservation_id=expected_reservation_id,
                )
        elif ack["owner_robot_id"] != owner_robot_id or ack["exit_id"] != exit_id:
            return Decision(False, "release_ack_generation_mismatch", robot_id, command="hold",
                            reservation_id=expected_reservation_id)
        expected_waiter = self._ack_expected_waiter.setdefault(expected_reservation_id, robot_id)
        if expected_waiter != robot_id:
            return Decision(False, "release_ack_waiter_mismatch", robot_id, command="hold",
                            reservation_id=expected_reservation_id)
        if not ack:
            return Decision(False, "release_ack_required", robot_id, command="hold",
                            reservation_id=expected_reservation_id)
        if ack["consumed_by"] is not None:
            return Decision(False, "release_ack_consumed", robot_id, command="hold",
                            reservation_id=expected_reservation_id)
        ack["consumed_by"] = robot_id
        self._event(now, "resume_requested", robot_id, command="replan_resume",
                    goal_xy=goal, reservation_id=expected_reservation_id)
        return Decision(True, "release_ack_observed", robot_id, command="replan_resume",
                        reservation_id=expected_reservation_id)

    def cancel_pending(
        self, robot_id: str, *, now: float, reason: str = "caller_cancelled"
    ) -> bool:
        """Remove only an ungranted request; held occupancy is never released here."""
        now = self._validate_time(now, "now")
        before = len(self._pending)
        self._pending = [pending for pending in self._pending if pending.robot_id != robot_id]
        removed = len(self._pending) != before
        if removed:
            self._clear_waits_for(robot_id)
            self._event(now, "pending_cancelled", robot_id, reason=reason)
        return removed

    def update_progress(self, robot_id: str, *, now: float, kind: str) -> None:
        now = self._validate_time(now, "now")
        if kind in {"reservation_wait", "process_wait"}:
            self._event(now, "non_progress_wait", robot_id, kind=kind)
            return
        self._last_progress[robot_id] = (now, kind)
        self._event(now, "progress", robot_id, kind=kind)

    def safety_decisions(self, *, now: float) -> tuple[Decision, ...]:
        """Request stops for stale observed robots; execution belongs to an adapter."""
        now = self._validate_time(now, "now")
        decisions: list[Decision] = []
        for robot_id in sorted(self._observations):
            reason = self._freshness_reason(robot_id, now)
            if reason or not self._observations[robot_id].motion_allowed:
                reason = reason or "motion_not_allowed"
                decisions.append(
                    Decision(False, reason, robot_id, command="request_stop")
                )
        return tuple(decisions)

    def is_stalled(self, robot_id: str, *, now: float) -> bool:
        now = self._validate_time(now, "now")
        progress = self._last_progress.get(robot_id)
        return bool(progress and now - progress[0] >= self.stall_threshold_s)

    def snapshot(self, *, now: float) -> TrafficSnapshot:
        now = self._validate_time(now, "now")
        freshness: dict[str, FreshnessSnapshot] = {
            robot: {
                "pose_fresh": now - obs.pose_time <= self.pose_freshness_s,
                "heartbeat_fresh": now - obs.heartbeat_time <= self.heartbeat_ttl_s,
                "motion_allowed": obs.motion_allowed,
            }
            for robot, obs in sorted(self._observations.items())
        }
        return {
            "backend": (
                "in-memory estimated-observation-domain; durable DB/broker and "
                "simulator/Nav2 execution not verified"
            ),
            "run_id": self.run_id,
            "sim_time_s": now,
            "thresholds": self.thresholds(),
            "resource_owner": dict(sorted(self._resource_owner.items())),
            "exit_owner": dict(sorted(self._exit_owner.items())),
            "active_reservations": {
                robot: {
                    "resources": bundle.resources,
                    "exit_id": bundle.exit_id,
                    "reservation_id": bundle.reservation_id,
                }
                for robot, bundle in sorted(self._active_reservations.items())
            },
            "release_acks": [
                {
                    "reservation_id": reservation_id,
                    "owner_robot_id": ack["owner_robot_id"],
                    "resources": ack["resources"],
                    "exit_id": ack["exit_id"],
                    "expected_waiter": self._ack_expected_waiter.get(reservation_id),
                    "consumed_by": ack["consumed_by"],
                    "superseded_by": ack["superseded_by"],
                }
                for reservation_id, ack in sorted(self._release_acks.items())
            ],
            "pending": [
                {"robot_id": p.robot_id, "resources": p.resources, "exit_id": p.exit_id,
                 "request_time": p.request_time}
                for p in sorted(self._pending)
            ],
            "freshness": freshness,
        }

    @property
    def events(self) -> tuple[EventRecord, ...]:
        return tuple(event.copy() for event in self._events)

    @property
    def event_sequence(self) -> int:
        """Current event cursor without copying the accumulated journal."""
        return self._event_seq

    def events_since(self, cursor: int) -> EventsPage:
        """Read in-memory events after a validated monotonic sequence cursor."""
        if not isinstance(cursor, int) or isinstance(cursor, bool) or cursor < 0:
            raise TrafficError("event cursor must be a non-negative integer")
        if cursor > self._event_seq:
            raise TrafficError("event cursor is ahead of the in-memory event sequence")
        return {
            "run_id": self.run_id,
            "cursor": cursor,
            "next_cursor": self._event_seq,
            "events": tuple(
                event.copy()
                for event in self._events
                if event["event_seq"] > cursor
            ),
        }

    @classmethod
    def thresholds(cls) -> Thresholds:
        return {
            "pose_freshness_s": cls.pose_freshness_s,
            "heartbeat_ttl_s": cls.heartbeat_ttl_s,
            "cycle_persistence_s": cls.cycle_persistence_s,
            "stall_threshold_s": cls.stall_threshold_s,
        }

    def _try_grant(self, robot_id: str, now: float) -> Decision:
        pending = next(p for p in self._pending if p.robot_id == robot_id)
        freshness = self._freshness_reason(robot_id, now)
        if freshness:
            self.cancel_pending(robot_id, now=now, reason="stale_pending")
            self._event(now, "request_denied", robot_id, reason=freshness)
            return Decision(
                False, freshness, robot_id, pending.resources, pending.exit_id,
                command="hold_at_entry",
            )
        if not self._observations[robot_id].motion_allowed:
            self.cancel_pending(robot_id, now=now, reason="motion_blocked_pending")
            self._event(now, "request_denied", robot_id, reason="motion_not_allowed")
            return Decision(False, "motion_not_allowed", robot_id, pending.resources,
                            pending.exit_id, command="hold_at_entry")
        earlier = [
            p for p in self._pending
            if p.sort_key < pending.sort_key
            and self._overlap(p, pending)
            and self._eligible_requester(p.robot_id, now)
        ]
        if earlier:
            return Decision(False, "queued_by_fairness", robot_id, pending.resources, pending.exit_id)
        blockers = {
            owner for resource in pending.resources
            if (owner := self._resource_owner.get(resource)) not in (None, robot_id)
        }
        exit_owner = self._exit_owner.get(pending.exit_id)
        if exit_owner not in (None, robot_id):
            blockers.add(exit_owner)
        if blockers:
            for blocker in blockers:
                self._wait_edges.setdefault((robot_id, blocker), now)
            stale = any(self._freshness_reason(owner, now) for owner in blockers)
            reason = "stale_occupancy" if stale else "occupied"
            self._event(now, "request_denied", robot_id, reason=reason, blockers=tuple(sorted(blockers)))
            return Decision(
                False, reason, robot_id, pending.resources, pending.exit_id,
                command="hold_at_entry",
            )
        reservation_id = self._next_reservation(
            robot_id, pending.resources, pending.exit_id, now
        )
        for resource in pending.resources:
            self._resource_owner[resource] = robot_id
            self._robot_holds.setdefault(robot_id, set()).add(resource)
        self._exit_owner[pending.exit_id] = robot_id
        self._active_reservations[robot_id] = _Active(
            pending.resources, pending.exit_id, reservation_id
        )
        self._assert_mapping_invariants()
        self._pending.remove(pending)
        self._clear_waits_for(robot_id)
        self._event(now, "reservation_granted", robot_id, resources=pending.resources,
                    exit_id=pending.exit_id, reservation_id=reservation_id)
        return Decision(True, "granted", robot_id, pending.resources, pending.exit_id,
                        command="proceed_to_entry", reservation_id=reservation_id)

    def _grant_pending(self, now: float) -> None:
        for pending in list(self._pending):
            if not self._eligible_requester(pending.robot_id, now):
                self.cancel_pending(pending.robot_id, now=now, reason="ineligible_pending")
        for pending in list(sorted(self._pending)):
            self._try_grant(pending.robot_id, now)

    def _eligible_requester(self, robot_id: str, now: float) -> bool:
        return (
            self._freshness_reason(robot_id, now) is None
            and self._observations[robot_id].motion_allowed
        )

    def _next_reservation(
        self,
        robot_id: str,
        resources: tuple[str, ...],
        exit_id: str,
        now: float,
    ) -> int:
        self._reservation_seq += 1
        new_reservation_id = self._reservation_seq
        stale_ids = [
            reservation_id
            for reservation_id, ack in self._release_acks.items()
            if (
                ack["exit_id"] == exit_id
                or set(ack["resources"]).intersection(resources)
            )
        ]
        for reservation_id in stale_ids:
            ack = self._release_acks[reservation_id]
            bound_waiter = self._ack_expected_waiter.get(reservation_id)
            if bound_waiter == robot_id and ack["consumed_by"] is None:
                ack["superseded_by"] = new_reservation_id
                self._event(
                    now, "release_ack_linked_to_successor", robot_id,
                    predecessor_reservation_id=reservation_id,
                    reservation_id=new_reservation_id,
                )
                continue
            del self._release_acks[reservation_id]
            self._ack_expected_waiter.pop(reservation_id, None)
            self._event(
                now, "release_ack_invalidated", robot_id,
                predecessor_reservation_id=reservation_id,
                reservation_id=new_reservation_id,
            )
        return new_reservation_id

    def _next_recovery_id(self) -> int:
        self._recovery_seq += 1
        return self._recovery_seq

    def _reservation_for(self, robot_id: str) -> int | None:
        active = self._active_reservations.get(robot_id)
        return None if active is None else active.reservation_id

    def _assert_mapping_invariants(self) -> None:
        expected_resources: dict[str, str] = {}
        expected_exits: dict[str, str] = {}
        for robot_id, active in self._active_reservations.items():
            for resource_id in active.resources:
                if resource_id in expected_resources:
                    raise TrafficError("resource appears in multiple active reservations")
                expected_resources[resource_id] = robot_id
            if active.exit_id in expected_exits:
                raise TrafficError("exit appears in multiple active reservations")
            expected_exits[active.exit_id] = robot_id
        if expected_resources != self._resource_owner:
            raise TrafficError("resource ownership mapping invariant violated")
        if expected_exits != self._exit_owner:
            raise TrafficError("exit ownership mapping invariant violated")
        expected_holds: dict[str, set[str]] = {}
        for resource_id, robot_id in expected_resources.items():
            expected_holds.setdefault(robot_id, set()).add(resource_id)
        actual_holds = {
            robot_id: set(resources)
            for robot_id, resources in self._robot_holds.items()
            if resources
        }
        if expected_holds != actual_holds:
            raise TrafficError("robot hold mapping invariant violated")

    def _freshness_reason(self, robot_id: str, now: float) -> str | None:
        obs = self._observations.get(robot_id)
        if obs is None:
            return "unknown_robot_observation"
        if now - obs.pose_time > self.pose_freshness_s:
            return "stale_pose"
        if now - obs.heartbeat_time > self.heartbeat_ttl_s:
            return "stale_heartbeat"
        return None

    def _clear_waits_for(self, robot_id: str) -> None:
        self._wait_edges = {
            edge: since for edge, since in self._wait_edges.items()
            if robot_id not in edge
        }
        self._observed_exit_waits = {
            edge: evidence for edge, evidence in self._observed_exit_waits.items()
            if robot_id not in edge
        }

    @staticmethod
    def _overlap(left: _Pending, right: _Pending) -> bool:
        return bool(set(left.resources).intersection(right.resources) or left.exit_id == right.exit_id)

    def _validate_time(self, value: float, name: str, *, advance: bool = True) -> float:
        if not self._finite_number(value) or isinstance(value, bool):
            raise TrafficError(f"{name} must be a finite simulation-seconds value")
        result = float(value)
        if result < 0:
            raise TrafficError(f"{name} cannot be negative")
        if advance and result < self._last_time:
            raise TrafficError(f"{name} cannot move backwards")
        if advance:
            self._last_time = result
        return result

    @classmethod
    def _validate_xy(cls, value: object, name: str) -> tuple[float, float]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
            raise TrafficError(f"{name} must contain two metre coordinates")
        if not all(cls._finite_number(item) and not isinstance(item, bool) for item in value):
            raise TrafficError(f"{name} must contain finite metre coordinates")
        return float(value[0]), float(value[1])

    @staticmethod
    def _finite_number(value: object) -> bool:
        return isinstance(value, (int, float)) and math.isfinite(float(value))

    def _event(
        self,
        now: float,
        event_type: str,
        robot_id: str,
        **details: Unpack[EventDetails],
    ) -> None:
        self._event_seq += 1
        event = EventRecord(
            run_id=self.run_id,
            event_seq=self._event_seq,
            sim_time_s=now,
            event_type=event_type,
            robot_id=robot_id,
            **details,
        )
        self._events.append(event)
