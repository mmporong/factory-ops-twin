"""Process-local tile reservations for G3 route handoff experiments.

Inputs are caller-supplied estimated ``map`` footprints.  This module does not
verify their physical source, run Nav2, inspect wheel speed, or create ROS
permissions.  The later adapter/validator owns those evidence boundaries.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from copy import deepcopy
from dataclasses import asdict, dataclass
from hashlib import sha256
import json
import math
from typing import Any
from uuid import UUID


Tile = tuple[int, int]


class RouteReservationError(ValueError):
    """Raised when a reservation request violates the coordinator contract."""


@dataclass(frozen=True, slots=True)
class ReservationDecision:
    granted: bool
    reason: str
    command: str
    robot_id: str
    generation: int | None
    predecessor_generation: int | None
    route_id: str | None
    goal_uuid: str | None
    tile_hash: str | None
    duplicate: bool = False
    blockers: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class _Observation:
    tiles: frozenset[Tile]
    pose_stamp_s: float
    observed_at_s: float


@dataclass(slots=True)
class _Hold:
    route_id: str
    goal_uuid: str
    tiles: frozenset[Tile]
    generation: int
    predecessor_generation: int | None
    accepted_goal_uuid: str | None = None
    parked: bool = False


@dataclass(slots=True)
class _Pending:
    sequence: int
    robot_id: str
    kind: str
    route_id: str
    goal_uuid: str
    tiles: frozenset[Tile]
    predecessor_generation: int | None
    since_s: float


class RouteReservationManager:
    """Own route tiles until an explicit, validated atomic handoff or shrink."""

    def __init__(
        self,
        run_id: str,
        robot_ids: Sequence[str],
        *,
        pose_ttl_s: float = 0.5,
        cycle_persistence_s: float = 5.0,
        record_pose_events: bool = True,
    ) -> None:
        robots = tuple(robot_ids)
        if not run_id:
            raise RouteReservationError("run_id is required")
        if not robots or any(not item for item in robots) or len(set(robots)) != len(robots):
            raise RouteReservationError("robot IDs must be unique and non-empty")
        ttl = self._finite(pose_ttl_s, "pose_ttl_s")
        persistence = self._finite(cycle_persistence_s, "cycle_persistence_s")
        if ttl <= 0 or persistence <= 0:
            raise RouteReservationError("pose TTL and cycle persistence must be positive")
        self.run_id = run_id
        self.robot_ids = tuple(sorted(robots))
        self.pose_ttl_s = ttl
        self.cycle_persistence_s = persistence
        self.record_pose_events = record_pose_events
        self._now = 0.0
        self._generation = 0
        self._pending_sequence = 0
        self._event_sequence = 0
        self._observations: dict[str, _Observation] = {}
        self._holds: dict[str, _Hold] = {}
        self._pending: dict[str, _Pending] = {}
        self._waits: dict[str, tuple[tuple[str, ...], float, str]] = {}
        self._events: list[dict[str, Any]] = []
        self._transfer_receipts: dict[tuple[str, int], tuple[str, int]] = {}
        self._shrink_receipts: dict[tuple[str, int], tuple[str, int]] = {}

    @property
    def events(self) -> list[dict[str, Any]]:
        return deepcopy(self._events)

    def events_since(self, cursor: int) -> list[dict[str, Any]]:
        """Read only the suffix after an in-process event sequence cursor."""
        if type(cursor) is not int or not 0 <= cursor <= self._event_sequence:
            raise RouteReservationError("event cursor must be an existing nonnegative sequence")
        return deepcopy(self._events[cursor:])

    def observe(
        self,
        robot_id: str,
        occupied_tiles: Iterable[Tile],
        *,
        pose_stamp_s: float,
        now_s: float,
        frame: str = "map",
    ) -> None:
        """Store one fresh estimated footprint without changing ownership."""
        self._robot(robot_id)
        now = self._validate_now(now_s)
        stamp = self._finite(pose_stamp_s, "pose_stamp_s")
        if frame != "map":
            raise RouteReservationError("observation frame must be map")
        if stamp < 0 or stamp > now:
            raise RouteReservationError("pose stamp must be nonnegative and not in the future")
        if now - stamp > self.pose_ttl_s:
            raise RouteReservationError("observation must be fresh when submitted")
        previous = self._observations.get(robot_id)
        if previous is not None and stamp < previous.pose_stamp_s:
            raise RouteReservationError("pose stamp must be monotonic per robot")
        tiles = self._tiles(occupied_tiles, "occupied_tiles")
        self._commit_time(now)
        self._observations[robot_id] = _Observation(tiles, stamp, now)
        if self.record_pose_events:
            self._event(now, "footprint_observed", robot_id, tile_hash=self._tile_hash(tiles))

    def reserve(
        self,
        robot_id: str,
        route_id: str,
        goal_uuid: str,
        next_tiles: Iterable[Tile],
        *,
        now_s: float,
    ) -> ReservationDecision:
        """Atomically acquire an initial route while retaining parked peers."""
        self._robot(robot_id)
        now = self._validate_now(now_s)
        route, goal, tiles = self._request(route_id, goal_uuid, next_tiles)
        self._commit_time(now)
        active = self._holds.get(robot_id)
        if active is not None:
            if (active.route_id, active.goal_uuid, active.tiles) == (route, goal, tiles):
                return self._decision(True, "duplicate_current_reservation", "proceed", robot_id, active, True)
            return self._hold(robot_id, "active_reservation_requires_transfer", active)
        if not self._all_fresh(now):
            return self._hold(robot_id, "all_robot_footprints_must_be_fresh")
        observation = self._observations[robot_id]
        if not observation.tiles.issubset(tiles):
            return self._hold(robot_id, "current_footprint_not_contained_in_route")
        older = self._older_conflicting_pending(robot_id, tiles, now)
        if older is not None:
            return self._wait(robot_id, "older_conflicting_request", route, goal, tiles, None, now, (older,))
        blockers = self._blockers(robot_id, tiles)
        if blockers:
            return self._wait(robot_id, "route_tiles_occupied", route, goal, tiles, None, now, blockers)
        hold = self._grant(robot_id, route, goal, tiles, None, now, "reservation_granted")
        return self._decision(True, "reservation_granted", "proceed", robot_id, hold)

    def transfer(
        self,
        robot_id: str,
        expected_generation: int,
        next_route_id: str,
        next_goal_uuid: str,
        next_tiles: Iterable[Tile],
        *,
        now_s: float,
    ) -> ReservationDecision:
        """Atomically replace old tiles only after the successor is fully valid."""
        self._robot(robot_id)
        now = self._validate_now(now_s)
        route, goal, tiles = self._request(next_route_id, next_goal_uuid, next_tiles)
        predecessor = self._generation_value(expected_generation)
        self._commit_time(now)
        active = self._holds.get(robot_id)
        if active is None:
            return self._hold(robot_id, "no_active_reservation")
        body_hash = self._request_hash(route, goal, tiles)
        receipt = self._transfer_receipts.get((robot_id, predecessor))
        if receipt is not None and receipt == (body_hash, active.generation):
            return self._decision(True, "duplicate_atomic_transfer", "proceed", robot_id, active, True)
        if active.generation != predecessor:
            return self._hold(robot_id, "stale_predecessor_generation", active)
        if not self._all_fresh(now):
            return self._hold(robot_id, "all_robot_footprints_must_be_fresh", active)
        overlap = active.tiles & tiles
        if not self._observations[robot_id].tiles.issubset(overlap):
            return self._hold(robot_id, "footprint_not_contained_in_transfer_overlap", active)
        extra = tiles - active.tiles
        older = self._older_conflicting_pending(robot_id, extra, now)
        if older is not None:
            return self._wait(robot_id, "older_conflicting_request", route, goal, tiles, predecessor, now, (older,))
        blockers = self._blockers(robot_id, extra)
        if blockers:
            return self._wait(robot_id, "successor_tiles_occupied", route, goal, tiles, predecessor, now, blockers)
        hold = self._grant(robot_id, route, goal, tiles, predecessor, now, "reservation_transferred")
        self._transfer_receipts[(robot_id, predecessor)] = (body_hash, hold.generation)
        return self._decision(True, "reservation_transferred", "proceed", robot_id, hold)

    def mark_goal_accepted(
        self,
        robot_id: str,
        generation: int,
        accepted_goal_uuid: str,
        *,
        now_s: float,
    ) -> ReservationDecision:
        self._robot(robot_id)
        now = self._validate_now(now_s)
        expected = self._generation_value(generation)
        accepted = self._uuid(accepted_goal_uuid, "accepted_goal_uuid")
        self._commit_time(now)
        active = self._holds.get(robot_id)
        if active is None or active.generation != expected:
            return self._hold(robot_id, "reservation_generation_mismatch", active)
        if active.goal_uuid != accepted:
            return self._hold(robot_id, "accepted_goal_uuid_mismatch", active)
        if active.accepted_goal_uuid == accepted:
            return self._decision(True, "duplicate_goal_acceptance", "proceed", robot_id, active, True)
        active.accepted_goal_uuid = accepted
        self._event(
            now,
            "goal_accepted",
            robot_id,
            generation=active.generation,
            goal_uuid=accepted,
            tile_hash=self._tile_hash(active.tiles),
        )
        return self._decision(True, "goal_accepted", "proceed", robot_id, active)

    def permission(self, robot_id: str, generation: int, goal_uuid: str, *, now_s: float) -> bool:
        """Query the logical binding; a ROS adapter must create any message."""
        self._robot(robot_id)
        now = self._validate_now(now_s)
        expected = self._generation_value(generation)
        goal = self._uuid(goal_uuid, "goal_uuid")
        self._commit_time(now)
        active = self._holds.get(robot_id)
        allowed = bool(
            active is not None
            and active.generation == expected
            and active.goal_uuid == goal
            and active.accepted_goal_uuid == goal
            and not active.parked
            and self._all_fresh(now)
        )
        self._event(
            now,
            "permission_queried",
            robot_id,
            generation=expected,
            goal_uuid=goal,
            allowed=allowed,
        )
        return allowed

    def mark_goal_finished(
        self,
        robot_id: str,
        generation: int,
        goal_uuid: str,
        *,
        now_s: float,
    ) -> ReservationDecision:
        """Retain route tiles as a parked hold after the accepted goal ends."""
        self._robot(robot_id)
        now = self._validate_now(now_s)
        expected = self._generation_value(generation)
        goal = self._uuid(goal_uuid, "goal_uuid")
        self._commit_time(now)
        active = self._holds.get(robot_id)
        if active is None or active.generation != expected:
            return self._hold(robot_id, "reservation_generation_mismatch", active)
        if active.goal_uuid != goal or active.accepted_goal_uuid != goal:
            return self._hold(robot_id, "finished_goal_binding_mismatch", active)
        if active.parked:
            return self._decision(True, "duplicate_parked_hold", "parked_hold", robot_id, active, True)
        active.parked = True
        self._event(
            now,
            "goal_finished_parked_hold_retained",
            robot_id,
            generation=active.generation,
            goal_uuid=goal,
            tile_hash=self._tile_hash(active.tiles),
        )
        return self._decision(True, "parked_hold_retained", "parked_hold", robot_id, active)

    def shrink_hold(
        self,
        robot_id: str,
        expected_generation: int,
        held_tiles: Iterable[Tile],
        *,
        now_s: float,
        stopped: bool,
    ) -> ReservationDecision:
        """Shrink a parked route hold after an external stopped assertion.

        ``stopped`` is only an adapter assertion here.  G3b must validate real
        wheel/encoder evidence before calling this method.
        """
        self._robot(robot_id)
        now = self._validate_now(now_s)
        predecessor = self._generation_value(expected_generation)
        tiles = self._tiles(held_tiles, "held_tiles")
        if type(stopped) is not bool:
            raise RouteReservationError("stopped must be a boolean")
        self._commit_time(now)
        active = self._holds.get(robot_id)
        if active is None:
            return self._hold(robot_id, "no_active_reservation")
        body_hash = self._tile_hash(tiles)
        receipt = self._shrink_receipts.get((robot_id, predecessor))
        if receipt is not None and receipt == (body_hash, active.generation):
            return self._decision(True, "duplicate_hold_shrink", "parked_hold", robot_id, active, True)
        if active.generation != predecessor:
            return self._hold(robot_id, "stale_predecessor_generation", active)
        if not stopped:
            return self._hold(robot_id, "moving_hold_truncation_denied", active)
        if not active.parked:
            return self._hold(robot_id, "goal_must_finish_before_hold_shrink", active)
        if not self._all_fresh(now):
            return self._hold(robot_id, "all_robot_footprints_must_be_fresh", active)
        if not tiles.issubset(active.tiles):
            return self._hold(robot_id, "hold_shrink_must_be_subset", active)
        if not self._observations[robot_id].tiles.issubset(tiles):
            return self._hold(robot_id, "current_footprint_not_contained_in_hold", active)
        hold = self._grant(
            robot_id,
            active.route_id,
            active.goal_uuid,
            tiles,
            predecessor,
            now,
            "parked_hold_shrunk",
            accepted_goal_uuid=active.accepted_goal_uuid,
            parked=True,
        )
        self._shrink_receipts[(robot_id, predecessor)] = (body_hash, hold.generation)
        return self._decision(True, "parked_hold_shrunk", "parked_hold", robot_id, hold)

    def detect_wait_cycle(self, *, now_s: float) -> tuple[str, ...] | None:
        """Report a persistent wait-for cycle without changing any hold."""
        now = self._validate_now(now_s)
        self._commit_time(now)
        graph: dict[str, tuple[str, ...]] = {}
        for robot, (recorded_blockers, since, fingerprint) in list(self._waits.items()):
            pending = self._pending.get(robot)
            if pending is None or not self._fresh(robot, now):
                self._invalidate_wait(robot, now, "missing_pending_or_stale_footprint")
                continue
            current_fingerprint = self._pending_fingerprint(pending)
            current_blockers = self._current_pending_blockers(pending)
            if current_fingerprint != fingerprint or current_blockers is None:
                self._invalidate_wait(robot, now, "pending_identity_or_generation_changed")
                continue
            if current_blockers != recorded_blockers:
                if current_blockers:
                    self._waits[robot] = (current_blockers, now, current_fingerprint)
                    self._event(
                        now,
                        "wait_evidence_reset",
                        robot,
                        reason="blocker_relation_changed",
                        blockers=current_blockers,
                        request_fingerprint=current_fingerprint,
                    )
                else:
                    self._invalidate_wait(robot, now, "blocker_relation_cleared")
                continue
            graph[robot] = recorded_blockers
        cycles: set[tuple[str, ...]] = set()
        for start in sorted(graph):
            path: list[str] = []
            cursor = start
            while cursor in graph and cursor not in path:
                path.append(cursor)
                candidates = tuple(item for item in graph[cursor] if item in graph)
                if not candidates:
                    break
                cursor = min(candidates)
            else:
                if cursor in path:
                    raw = path[path.index(cursor) :]
                    rotations = [tuple(raw[index:] + raw[:index]) for index in range(len(raw))]
                    cycles.add(min(rotations))
        for cycle in sorted(cycles):
            if not all(self._fresh(robot, now) for robot in cycle):
                for robot in cycle:
                    self._invalidate_wait(robot, now, "cycle_member_footprint_stale")
                continue
            since = max(self._waits[robot][1] for robot in cycle)
            if now - since >= self.cycle_persistence_s:
                self._event(now, "persistent_wait_cycle_reported", None, cycle=cycle, since_s=since)
                return cycle
        return None

    def snapshot(self, *, now_s: float) -> dict[str, Any]:
        now = self._validate_now(now_s)
        self._commit_time(now)
        return {
            "scope": (
                "process-local synthetic reservation contract; estimated footprints only; "
                "no Nav2, physical-source, stopped-wheel, or ROS permission claim"
            ),
            "run_id": self.run_id,
            "sim_time": now,
            "pose_ttl_s": self.pose_ttl_s,
            "cycle_persistence_s": self.cycle_persistence_s,
            "observations": {
                robot: {
                    "tiles": self._tile_list(value.tiles),
                    "tile_hash": self._tile_hash(value.tiles),
                    "pose_stamp_s": value.pose_stamp_s,
                    "fresh": self._fresh(robot, now),
                }
                for robot, value in sorted(self._observations.items())
            },
            "holds": {
                robot: {
                    "route_id": hold.route_id,
                    "goal_uuid": hold.goal_uuid,
                    "accepted_goal_uuid": hold.accepted_goal_uuid,
                    "generation": hold.generation,
                    "predecessor_generation": hold.predecessor_generation,
                    "tiles": self._tile_list(hold.tiles),
                    "tile_hash": self._tile_hash(hold.tiles),
                    "parked": hold.parked,
                }
                for robot, hold in sorted(self._holds.items())
            },
            "pending": [
                {
                    "sequence": pending.sequence,
                    "robot_id": pending.robot_id,
                    "kind": pending.kind,
                    "route_id": pending.route_id,
                    "goal_uuid": pending.goal_uuid,
                    "predecessor_generation": pending.predecessor_generation,
                    "tiles": self._tile_list(pending.tiles),
                    "tile_hash": self._tile_hash(pending.tiles),
                    "since_s": pending.since_s,
                }
                for pending in sorted(self._pending.values(), key=lambda item: item.sequence)
            ],
            "waits": {
                robot: {
                    "blockers": blockers,
                    "since_s": since,
                    "request_fingerprint": fingerprint,
                }
                for robot, (blockers, since, fingerprint) in sorted(self._waits.items())
            },
            "events": self.events,
        }

    def _grant(
        self,
        robot_id: str,
        route_id: str,
        goal_uuid: str,
        tiles: frozenset[Tile],
        predecessor: int | None,
        now: float,
        event_type: str,
        *,
        accepted_goal_uuid: str | None = None,
        parked: bool = False,
    ) -> _Hold:
        self._generation += 1
        hold = _Hold(
            route_id,
            goal_uuid,
            tiles,
            self._generation,
            predecessor,
            accepted_goal_uuid,
            parked,
        )
        self._holds[robot_id] = hold
        self._pending.pop(robot_id, None)
        self._waits.pop(robot_id, None)
        self._event(
            now,
            event_type,
            robot_id,
            predecessor_generation=predecessor,
            successor_generation=hold.generation,
            route_id=route_id,
            goal_uuid=goal_uuid,
            tile_hash=self._tile_hash(tiles),
            tiles=self._tile_list(tiles),
            parked=parked,
            accepted_goal_uuid=accepted_goal_uuid,
        )
        return hold

    def _wait(
        self,
        robot_id: str,
        reason: str,
        route_id: str,
        goal_uuid: str,
        tiles: frozenset[Tile],
        predecessor: int | None,
        now: float,
        blockers: tuple[str, ...],
    ) -> ReservationDecision:
        prior = self._pending.get(robot_id)
        fingerprint = (
            (prior.kind, prior.route_id, prior.goal_uuid, prior.tiles, prior.predecessor_generation)
            if prior
            else None
        )
        requested = ("transfer" if predecessor is not None else "reserve", route_id, goal_uuid, tiles, predecessor)
        request_changed = fingerprint != requested
        if request_changed:
            self._pending_sequence += 1
            prior = _Pending(
                self._pending_sequence,
                robot_id,
                requested[0],
                route_id,
                goal_uuid,
                tiles,
                predecessor,
                now,
            )
            self._pending[robot_id] = prior
        assert prior is not None
        normalized_blockers = tuple(sorted(set(blockers)))
        request_fingerprint = self._pending_fingerprint(prior)
        previous_wait = self._waits.get(robot_id)
        wait_changed = (
            request_changed
            or previous_wait is None
            or previous_wait[0] != normalized_blockers
            or previous_wait[2] != request_fingerprint
        )
        wait_since = now
        if not wait_changed:
            assert previous_wait is not None
            wait_since = previous_wait[1]
        self._waits[robot_id] = (normalized_blockers, wait_since, request_fingerprint)
        self._event(
            now,
            "reservation_wait",
            robot_id,
            reason=reason,
            blockers=blockers,
            predecessor_generation=predecessor,
            route_id=route_id,
            goal_uuid=goal_uuid,
            tile_hash=self._tile_hash(tiles),
            pending_sequence=prior.sequence,
            request_fingerprint=request_fingerprint,
        )
        return self._hold(robot_id, reason, self._holds.get(robot_id), blockers=blockers)

    def _older_conflicting_pending(
        self, robot_id: str, tiles: frozenset[Tile], now: float
    ) -> str | None:
        current = self._pending.get(robot_id)
        current_sequence = math.inf if current is None else current.sequence
        for pending in sorted(self._pending.values(), key=lambda item: item.sequence):
            if pending.robot_id == robot_id or pending.sequence >= current_sequence:
                continue
            if pending.tiles.isdisjoint(tiles):
                continue
            if self._pending_eligible(pending, now):
                return pending.robot_id
        return None

    def _pending_eligible(self, pending: _Pending, now: float) -> bool:
        if not self._all_fresh(now):
            return False
        observation = self._observations[pending.robot_id]
        active = self._holds.get(pending.robot_id)
        if pending.kind == "reserve":
            return active is None and observation.tiles.issubset(pending.tiles) and not self._blockers(
                pending.robot_id, pending.tiles
            )
        if active is None or active.generation != pending.predecessor_generation:
            return False
        return observation.tiles.issubset(active.tiles & pending.tiles) and not self._blockers(
            pending.robot_id, pending.tiles - active.tiles
        )

    def _current_pending_blockers(self, pending: _Pending) -> tuple[str, ...] | None:
        observation = self._observations.get(pending.robot_id)
        active = self._holds.get(pending.robot_id)
        if observation is None:
            return None
        if pending.kind == "reserve":
            if active is not None or not observation.tiles.issubset(pending.tiles):
                return None
            return self._blockers(pending.robot_id, pending.tiles)
        if active is None or active.generation != pending.predecessor_generation:
            return None
        if not observation.tiles.issubset(active.tiles & pending.tiles):
            return None
        return self._blockers(pending.robot_id, pending.tiles - active.tiles)

    def _invalidate_wait(self, robot_id: str, now: float, reason: str) -> None:
        if self._waits.pop(robot_id, None) is not None:
            self._event(now, "wait_evidence_invalidated", robot_id, reason=reason)

    def _blockers(self, robot_id: str, tiles: frozenset[Tile]) -> tuple[str, ...]:
        blockers: set[str] = set()
        for peer, observation in self._observations.items():
            if peer != robot_id and not observation.tiles.isdisjoint(tiles):
                blockers.add(peer)
        for peer, hold in self._holds.items():
            if peer != robot_id and not hold.tiles.isdisjoint(tiles):
                blockers.add(peer)
        return tuple(sorted(blockers))

    def _all_fresh(self, now: float) -> bool:
        return all(self._fresh(robot, now) for robot in self.robot_ids)

    def _fresh(self, robot_id: str, now: float) -> bool:
        observation = self._observations.get(robot_id)
        return observation is not None and 0 <= now - observation.pose_stamp_s <= self.pose_ttl_s

    def _request(
        self, route_id: str, goal_uuid: str, tiles: Iterable[Tile]
    ) -> tuple[str, str, frozenset[Tile]]:
        if not isinstance(route_id, str) or not route_id:
            raise RouteReservationError("route_id is required")
        return route_id, self._uuid(goal_uuid, "goal_uuid"), self._tiles(tiles, "next_tiles")

    @classmethod
    def _pending_fingerprint(cls, pending: _Pending) -> str:
        payload = {
            "kind": pending.kind,
            "route_id": pending.route_id,
            "goal_uuid": pending.goal_uuid,
            "tiles": cls._tile_list(pending.tiles),
            "predecessor_generation": pending.predecessor_generation,
        }
        return sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("ascii")
        ).hexdigest()

    def _hold(
        self,
        robot_id: str,
        reason: str,
        active: _Hold | None = None,
        *,
        blockers: tuple[str, ...] = (),
    ) -> ReservationDecision:
        return ReservationDecision(
            False,
            reason,
            "hold",
            robot_id,
            None if active is None else active.generation,
            None if active is None else active.predecessor_generation,
            None if active is None else active.route_id,
            None if active is None else active.goal_uuid,
            None if active is None else self._tile_hash(active.tiles),
            blockers=blockers,
        )

    def _decision(
        self,
        granted: bool,
        reason: str,
        command: str,
        robot_id: str,
        hold: _Hold,
        duplicate: bool = False,
    ) -> ReservationDecision:
        return ReservationDecision(
            granted,
            reason,
            command,
            robot_id,
            hold.generation,
            hold.predecessor_generation,
            hold.route_id,
            hold.goal_uuid,
            self._tile_hash(hold.tiles),
            duplicate,
        )

    def _robot(self, robot_id: str) -> None:
        if robot_id not in self.robot_ids:
            raise RouteReservationError("unknown robot_id")

    def _validate_now(self, value: float) -> float:
        now = self._finite(value, "now_s")
        if now < self._now:
            raise RouteReservationError("now_s must be monotonic")
        return now

    def _commit_time(self, now: float) -> None:
        self._now = now

    @staticmethod
    def _finite(value: float, label: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RouteReservationError(f"{label} must be a number")
        number = float(value)
        if not math.isfinite(number):
            raise RouteReservationError(f"{label} must be finite")
        return number

    @staticmethod
    def _generation_value(value: int) -> int:
        if type(value) is not int or value <= 0:
            raise RouteReservationError("generation must be a positive integer")
        return value

    @staticmethod
    def _uuid(value: str, label: str) -> str:
        if not isinstance(value, str):
            raise RouteReservationError(f"{label} must be a UUID string")
        try:
            UUID(value)
        except (ValueError, AttributeError) as exc:
            raise RouteReservationError(f"{label} must be a UUID string") from exc
        return value

    @staticmethod
    def _tiles(values: Iterable[Tile], label: str) -> frozenset[Tile]:
        try:
            raw = tuple(values)
        except TypeError as exc:
            raise RouteReservationError(f"{label} must be an iterable of integer tile pairs") from exc
        if not raw:
            raise RouteReservationError(f"{label} must not be empty")
        result: set[Tile] = set()
        for value in raw:
            if (
                not isinstance(value, tuple)
                or len(value) != 2
                or any(type(coordinate) is not int for coordinate in value)
            ):
                raise RouteReservationError(f"{label} must contain integer (x, y) tuples")
            result.add((value[0], value[1]))
        return frozenset(result)

    @classmethod
    def _tile_hash(cls, tiles: frozenset[Tile]) -> str:
        payload = json.dumps(cls._tile_list(tiles), separators=(",", ":"))
        return sha256(payload.encode("ascii")).hexdigest()

    @staticmethod
    def _tile_list(tiles: frozenset[Tile]) -> list[list[int]]:
        return [[x, y] for x, y in sorted(tiles)]

    @classmethod
    def _request_hash(cls, route_id: str, goal_uuid: str, tiles: frozenset[Tile]) -> str:
        payload = {"route_id": route_id, "goal_uuid": goal_uuid, "tiles": cls._tile_list(tiles)}
        return sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("ascii")).hexdigest()

    def _event(self, now: float, event_type: str, robot_id: str | None, **body: Any) -> None:
        self._event_sequence += 1
        self._events.append(
            {
                "run_id": self.run_id,
                "event_sequence": self._event_sequence,
                "sim_time": now,
                "event_type": event_type,
                "robot_id": robot_id,
                **deepcopy(body),
            }
        )


__all__ = [
    "ReservationDecision",
    "RouteReservationError",
    "RouteReservationManager",
    "Tile",
]
