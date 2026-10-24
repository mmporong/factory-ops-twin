"""Configured-fleet kinematic operations, distinct from Isaac/ROS evidence.

Bounded straight segments retain their occupied footprint at every stop. Station
gates keep opposing handovers out of the same dead-end approach. Accepted work drains before a planned
aisle closure applies. This is not sudden-obstacle detection or motor dynamics.
"""
from __future__ import annotations

from collections import deque
from functools import lru_cache
import json
import math
from pathlib import Path
from typing import Any, Mapping
from uuid import NAMESPACE_URL, uuid5

from .dashboard_model import DashboardModel
from .drive_profile import DriveLimits
from .fleet_routing import swept_tiles
from .motion import DriveGeometry
from .operations import Transition
from .operations_corridor import OperationsCorridors
from .operations_bridge import BridgeAdmission
from .operations_charging import OperationsCharging, SCOPE as OPERATIONS_CHARGING_SCOPE
from .operations_motion import PathMotion
from .operations_replan import anchor_at_pose, find_alternative_route
from .route_reservation import RouteReservationManager


class OperationsModel(DashboardModel):
    def __init__(self, config: Path, *, policy: str = "fifo", initial_orders: int = 4,
                 run_id: str | None = None,
                 operations_charging_profile: Mapping[str, Any] | None = None) -> None:
        if policy not in ("fifo", "eta"):
            raise ValueError("operations policy must be fifo or eta")
        self.closed = False
        self.requested_closed = False
        self.dispatch_policy = policy
        fleet_path = config / "fleet-profile.json"
        stations = ("P1", "P2", "P3", "P4")
        if fleet_path.is_file():
            fleet = json.loads(fleet_path.read_text(encoding="utf-8"))
            configured = fleet.get("robot_stations") if isinstance(fleet, dict) else None
            if (not isinstance(fleet, dict) or type(fleet.get("schema_version")) is not int
                    or fleet["schema_version"] != 1 or not isinstance(configured, list)
                    or not configured or type(fleet.get("fleet_count")) is not int
                    or fleet["fleet_count"] != len(configured)):
                raise ValueError("fleet profile requires schema 1 and matching nonempty fleet_count/robot_stations")
            stations = tuple(configured)
        super().__init__(config, initial_orders=initial_orders, run_id=run_id,
                         robot_stations=stations)
        drive = json.loads((config / "g2.json").read_text(encoding="utf-8"))
        self.geometry = DriveGeometry(**{k: v for k, v in drive["drive_geometry"].items() if k != "provenance"})
        self.limits = DriveLimits(**{k: v for k, v in drive["drive_profile"].items() if k != "provenance"})
        self.motions = {r: PathMotion(self.limits, self.geometry) for r in self.ledger.robots}
        self.corridors = OperationsCorridors(self.book.config)
        self.bridges = BridgeAdmission(self.book.config)
        raw_staging = self.book.config.get("staging_after_drop", {})
        if (not isinstance(raw_staging, Mapping)
                or any(not isinstance(drop, str) or drop not in self.book.nodes
                       or not isinstance(stage, str) or stage not in self.book.nodes
                       for drop, stage in raw_staging.items())):
            raise ValueError("staging_after_drop must map route nodes to route nodes")
        self.staging_after_drop = dict(raw_staging)
        self.homes = dict(zip(self.ledger.robots, stations))
        self.radius = self.book.config["authored_allowances"].get("reservation_radius_including_tracking_m", self.book.planning_radius_m)
        self.collision_radius = self.book.config["collision_radius"]["value"]
        self.reservations = RouteReservationManager(self.ledger.run_id, tuple(self.ledger.robots), record_pose_events=False)
        self._leases: dict[str, dict[str, Any]] = {}
        self._waiting: dict[str, dict[str, Any]] = {}
        self._idle_since = {r: 0.0 for r in self.ledger.robots}
        self._travel_m = {r: 0.0 for r in self.ledger.robots}
        self._wait_s = {r: 0.0 for r in self.ledger.robots}
        self._assignments = {r: 0 for r in self.ledger.robots}
        self._deliveries = {r: 0 for r in self.ledger.robots}
        self._station_owners: dict[str, str] = {}
        self._jobs: dict[str, tuple[str, str]] = {}
        self._chunks: dict[str, dict[str, Any]] = {}
        self._admission_seq = 0
        self._chunk_seq = {r: 0 for r in self.ledger.robots}
        self._alignment_seq = {r: 0 for r in self.ledger.robots}
        self._max_segment_m = max(math.dist(self.book.nodes[a], self.book.nodes[b]) for a, b in self.book.edges)
        self.min_clearance_m: float | None = None
        self.peak_moving_robots = 0
        self.wait_cycle: tuple[str, ...] | None = None
        self.wait_cycle_episodes = 0
        self._bridge_cycle_candidate: tuple[str, ...] | None = None
        self._bridge_cycle_since_s: float | None = None
        self._last_cycle_check_s = -1.0
        self._replans: dict[str, dict[str, Any]] = {}
        self._replan_examined: set[tuple[Any, ...]] = set()
        self.replan_attempt_count = 0
        self.replan_applied_count = 0
        self.replan_unavailable_count = 0
        self.last_replan_reason: str | None = None
        self.traffic_yield_count = 0
        self.traffic_yield_completed_count = 0
        self._charging_parking_resume: dict[str, str] = {}
        self._observe_all()
        for robot in self.ledger.robots:
            point = self.book.nodes[self.homes[robot]]
            if not self._claim(robot, "initial-park", self._tiles((point,))):
                raise ValueError("initial parking footprints conflict")
            self._park_hold(robot)
        profile_path = config / "operations-charging.json"
        profile = (operations_charging_profile if operations_charging_profile is not None
                   else json.loads(profile_path.read_text(encoding="utf-8")) if profile_path.is_file() else None)
        self.operations_charging = OperationsCharging(self, profile) if profile is not None else None

    def _route(self, start: str, goal: str):
        return self.book.plan(start, goal, closed=self.closed).points

    @lru_cache(maxsize=512)
    def _tiles(self, points: tuple[tuple[float, float], ...]):
        bounds = self.book.layout.bounds
        return swept_tiles(points, self.radius, tile_size_m=self.book.tile_size_m,
                           bounds_xy=(bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax))

    def _footprint(self, robot: str):
        p = self._poses[robot]
        return self._tiles(((p["x_m"], p["y_m"]),))

    def _observe_all(self) -> None:
        for robot in self.ledger.robots:
            self.reservations.observe(robot, self._footprint(robot),
                                      pose_stamp_s=self.sim_time, now_s=self.sim_time)

    def _claim(self, robot: str, identity: str, tiles) -> bool:
        route_id = f"operations:{identity}:{self.closed}"
        goal = str(uuid5(NAMESPACE_URL, f"{self.ledger.run_id}/{robot}/{route_id}"))
        previous = self._leases.get(robot)
        if previous is None:
            decision = self.reservations.reserve(robot, route_id, goal, tiles, now_s=self.sim_time)
        else:
            decision = self.reservations.transfer(robot, previous["generation"], route_id, goal, tiles, now_s=self.sim_time)
        if not decision.granted:
            self._waiting[robot] = {"reason": decision.reason, "blockers": list(decision.blockers)}
            return False
        self.reservations.mark_goal_accepted(robot, decision.generation, goal, now_s=self.sim_time)
        self._leases[robot] = {"generation": decision.generation, "goal": goal, "tiles": tiles}
        self._waiting.pop(robot, None)
        return True

    def _park_hold(self, robot: str) -> None:
        lease = self._leases[robot]
        self.reservations.mark_goal_finished(robot, lease["generation"], lease["goal"], now_s=self.sim_time)
        decision = self.reservations.shrink_hold(robot, lease["generation"], self._footprint(robot),
                                               now_s=self.sim_time, stopped=self.motions[robot].stopped)
        if not decision.granted:
            raise RuntimeError(f"parked occupancy cannot be retained: {decision.reason}")
        lease.update(generation=decision.generation, tiles=self._footprint(robot))

    def _release_station_gates(self) -> None:
        for station, robot in list(self._station_owners.items()):
            job = self._jobs.get(robot)
            trip = self._trips.get(robot)
            if job and (station == job[1] or (trip and trip["phase"] == "pickup")):
                continue
            pose = self._poses[robot]
            # Keep a full pair of reservation envelopes between arrivals/departures.
            if math.dist((pose["x_m"], pose["y_m"]), self.book.nodes[station]) > 2 * self.radius + self.book.tile_size_m:
                del self._station_owners[station]

    def _dispatch(self) -> None:
        self._observe_all()
        self._release_station_gates()
        draining = self.requested_closed != self.closed
        charging_idle = (self.operations_charging is None
                         or not self.operations_charging.queue
                         and not self.operations_charging.aux
                         and not self.operations_charging.relocating
                         and not self.operations_charging.faults)
        if (draining and charging_idle and not self._trips
                and all(self._locations[r] == self.homes[r] for r in self.homes)):
            self.closed = self.requested_closed
            self._event("aisle_closure_applied", "계획 통로 폐쇄 적용" if self.closed else "통로 재개방 적용", None)
            draining = False
        self._waiting = {r: w for r, w in self._waiting.items() if r in self._trips}
        while not draining:
            available = [r for r, state in self.ledger.robots.items()
                         if r not in self._trips and state.available and state.task_id is None and state.payload_id is None
                         and (self.operations_charging is None or self.operations_charging.available(r))]
            jobs = [(t.ready_time, t.task_id, "order", t.pickup_station, t.drop_station, t)
                    for t in self.ledger.ready_tasks if self.ledger.can_assign(t)]
            jobs.extend((t["ready_time_s"], t["id"], "empty_return", t["source"], t["destination"], t)
                        for t in self._return_tasks.values() if t["state"] == "ready")
            pairs = []
            for ready, identity, kind, pickup, destination, task in jobs:
                for robot in available:
                    first_distance = self.book.plan(self._locations[robot], pickup, closed=self.closed).distance_m
                    distance = first_distance + self.book.plan(pickup, destination, closed=self.closed).distance_m
                    rank = ((ready, identity, first_distance, robot) if self.dispatch_policy == "fifo"
                            else (distance / self.limits.max_linear_m_s, ready, identity, robot))
                    pairs.append((rank, robot, identity, kind, task, pickup, destination))
            assigned = False
            for _, robot, identity, kind, task, pickup, destination in sorted(pairs, key=lambda p: p[0]):
                if (self.operations_charging is not None
                        and not self.operations_charging.can_assign(self, robot, pickup, destination)):
                    continue
                blockers = sorted({self._station_owners[s] for s in (pickup, destination)
                                   if s in self._station_owners and self._station_owners[s] != robot})
                if blockers:
                    self._waiting[robot] = {"reason": "handover_station_occupied", "blockers": blockers}
                    continue
                self._station_owners.update({pickup: robot, destination: robot})
                self._jobs[robot] = (pickup, destination)
                if kind == "order":
                    self._apply(task, robot, Transition.ASSIGN)
                else:
                    task.update(state="assigned", robot_id=robot)
                self._trip(robot, task, self._locations[robot], pickup, "pickup", kind=kind)
                self._assignments[robot] += 1
                self._event("assigned", f"{robot} {identity} 배정 · {self.dispatch_policy}", robot)
                assigned = True
                break
            if not assigned:
                break
        self._dispatch_extra_jobs(draining)
        for robot in self.ledger.robots:
            if robot in self._trips or self._locations[robot] == self.homes[robot]:
                continue
            if self.operations_charging is not None and not self.operations_charging.can_park(robot):
                continue
            # Vacate an idle handover without claiming a completed parking move.
            if draining or self.sim_time - self._idle_since[robot] >= 2.0:
                interrupted_goal = self._charging_parking_resume.get(robot)
                self._trip(
                    robot, {}, self._locations[robot], self.homes[robot], "parking",
                    kind="parking", purpose="home",
                )
                self._waiting.pop(robot, None)
                if (interrupted_goal is not None and self.operations_charging is not None
                        and robot not in self.operations_charging.queue
                        and robot not in self.operations_charging.aux):
                    del self._charging_parking_resume[robot]
                    self._event(
                        "charge_parking_resumed",
                        f"{robot} 충전 후 주차 복귀 재개",
                        robot,
                        interrupted_goal=interrupted_goal,
                        resume_goal=self.homes[robot],
                    )

    def _dispatch_extra_jobs(self, draining: bool) -> None:
        """Admit specialized production jobs before sending idle vehicles home."""

    def _trip(self, robot_id, task, start, goal, phase, *, kind="order", purpose=None) -> None:
        self._replans.pop(robot_id, None)
        super()._trip(robot_id, task, start, goal, phase, kind=kind)
        route = self.book.plan(start, goal, closed=self.closed)
        remaining_nodes = route.nodes[1:]
        if len(remaining_nodes) != len(self._trips[robot_id]["points"]):
            raise RuntimeError("authored route nodes and points must remain one-to-one")
        self._admission_seq += 1
        self._trips[robot_id]["nodes"] = deque(remaining_nodes)
        self._trips[robot_id]["admission_seq"] = self._admission_seq
        if kind == "parking":
            self._trips[robot_id]["purpose"] = purpose or "home"
        self._refresh_admission_preview(robot_id, self._trips[robot_id])
        if self.bridges.has_window(robot_id):
            if not self._trips[robot_id]["admission_tiles"]:
                self.bridges.release(robot_id, stopped=self.motions[robot_id].stopped)
            else:
                self.bridges.retarget(
                    robot_id,
                    self._trips[robot_id]["admission_nodes"],
                    route_tiles=self._trips[robot_id]["admission_tiles"],
                    stopped=self.motions[robot_id].stopped,
                )
        if hasattr(self, "operations_charging") and self.operations_charging is not None:
            self.operations_charging.denied.discard(robot_id)
        self._waiting.pop(robot_id, None)

    def _align_at_hold(self, robot: str, yaw_rad: float, dt: float) -> bool:
        motion = self.motions[robot]
        if motion.stopped:
            self._alignment_seq[robot] += 1
            if not self._claim(
                robot,
                f"dock-align-{self._alignment_seq[robot]}",
                self._footprint(robot),
            ):
                raise RuntimeError("dock alignment could not retain its current footprint")
        lease = self._leases[robot]
        if not self.reservations.permission(
            robot, lease["generation"], lease["goal"], now_s=self.sim_time,
        ):
            raise RuntimeError("dock alignment lost its reservation")
        aligned = motion.rotate_to(self._poses[robot], yaw_rad, dt)
        if not self._footprint(robot).issubset(lease["tiles"]):
            raise RuntimeError("dock alignment escaped its retained footprint")
        if aligned:
            self._observe_all()
            self._park_hold(robot)
        return aligned

    def _schedule_cycle_replan(self, cycle: tuple[str, ...]) -> None:
        if any(robot in self._replans for robot in cycle):
            return
        signature = (
            cycle,
            tuple((robot, self._poses[robot]["x_m"], self._poses[robot]["y_m"],
                   self._trips.get(robot, {}).get("goal")) for robot in cycle),
            tuple(
                (robot, tuple(sorted(self._footprint(robot))))
                for robot in sorted(self.ledger.robots)
            ),
        )
        if signature in self._replan_examined:
            return
        self._replan_examined.add(signature)
        self.replan_attempt_count += 1
        self.last_replan_reason = "persistent_reservation_wait_cycle"
        reservation = self.reservations.snapshot(now_s=self.sim_time)
        observations = reservation["observations"]
        candidates = []
        for robot in cycle:
            trip = self._trips.get(robot)
            if (trip is None or robot in self._chunks or not self.motions[robot].stopped
                    or not trip["points"]):
                continue
            start = anchor_at_pose(self.book.nodes, self._poses[robot])
            if start is None:
                continue
            # A queue anchor has one authored way out: its direction-specific
            # safe exit.  A generic shortest-path replan can look clear while
            # silently dropping that mandatory handover contract.
            if start in self.bridges.queue_nodes:
                continue
            peer_tiles = frozenset(
                tuple(tile)
                for peer, observation in observations.items() if peer != robot
                for tile in observation["tiles"]
            )
            alternative = find_alternative_route(
                self.book,
                start=start,
                goal=trip["goal"],
                closed=self.closed,
                reservation_radius_m=self.radius,
                peer_footprint_tiles=peer_tiles,
                existing_points=tuple(trip["points"]),
            )
            if alternative is None:
                continue
            window = self._window_for_trip(
                alternative.points[0], alternative.points[1:], alternative.nodes[1:], trip,
                robot_id=robot,
            )
            if self.bridges.blockers(
                robot,
                window["nodes"],
                route_tiles=window["tiles"],
                observed_tiles=self._admission_observations(robot, trip),
            ):
                continue
            if self._safe_exit_lookahead_blockers(
                robot, trip, window.get("queue_barrier"),
            ):
                continue
            candidates.append((alternative.distance_m, robot, start, alternative))
        if not candidates:
            if self._schedule_traffic_yield(cycle, observations):
                return
            self.replan_unavailable_count += 1
            self._event(
                "traffic_replan_unavailable",
                f"순환 대기 대체 경로 없음: {', '.join(cycle)}",
                None,
                reason=self.last_replan_reason,
                cycle=list(cycle),
            )
            return
        _, robot, start, alternative = min(candidates, key=lambda item: (item[0], item[1]))
        self._replans[robot] = {
            "points": alternative.points[1:],
            "nodes": alternative.nodes,
            "start": start,
            "reason": self.last_replan_reason,
            "cycle": cycle,
            "attempt": self.replan_attempt_count,
        }
        self._event(
            "traffic_replan_selected",
            f"{robot} 순환 대기 우회 선택 · {start}→{self._trips[robot]['goal']}",
            robot,
            reason=self.last_replan_reason,
            cycle=list(cycle),
            route_nodes=list(alternative.nodes),
        )

    def _schedule_traffic_yield(
        self, cycle: tuple[str, ...], observations: Mapping[str, Any],
    ) -> bool:
        """Move one empty pickup vehicle to a clear same-bay parking refuge."""

        candidates = []
        safe_exits = frozenset(self.bridges.safe_exit_nodes.values())
        inventory_loads = getattr(self.inventory, "robot_loads", {})
        for robot in cycle:
            trip = self._trips.get(robot)
            state = self.ledger.robots[robot]
            if (trip is None or trip["kind"] == "parking" or trip["phase"] != "pickup"
                    or robot in self._chunks or not self.motions[robot].stopped
                    or self.bridges.has_window(robot) or self.bridges.held(robot)
                    or state.payload_id is not None
                    or robot in inventory_loads or not trip["points"]):
                continue
            start = anchor_at_pose(self.book.nodes, self._poses[robot])
            if start is None or start in self.bridges.queue_nodes:
                continue
            bay = start.split("_", 1)[0]
            if not bay:
                continue
            peer_tiles = frozenset(
                tuple(tile)
                for peer, observation in observations.items() if peer != robot
                for tile in observation["tiles"]
            )
            for refuge in sorted(set(self.homes.values())):
                if not refuge.startswith(f"{bay}_"):
                    continue
                alternative = find_alternative_route(
                    self.book,
                    start=start,
                    goal=refuge,
                    closed=self.closed,
                    reservation_radius_m=self.radius,
                    peer_footprint_tiles=peer_tiles,
                    existing_points=(),
                )
                if alternative is None or any(
                    node in self.bridges.queue_nodes or node in safe_exits
                    or not node.startswith(f"{bay}_")
                    for node in alternative.nodes[1:]
                ):
                    continue
                yield_trip = {
                    "kind": "parking", "phase": "pickup", "goal": refuge,
                    "purpose": "traffic_yield",
                }
                window = self._window_for_trip(
                    alternative.points[0], alternative.points[1:], alternative.nodes[1:],
                    yield_trip, robot_id=robot,
                )
                if len(window["nodes"]) != len(alternative.nodes) - 1:
                    continue
                if self.bridges.blockers(
                    robot,
                    window["nodes"],
                    route_tiles=window["tiles"],
                    observed_tiles=self._admission_observations(robot, yield_trip),
                ):
                    continue
                candidates.append((alternative.distance_m, robot, start, refuge, trip, alternative))
        if not candidates:
            return False
        distance, robot, start, refuge, resume, alternative = min(
            candidates, key=lambda item: (item[0], item[1], item[3]),
        )
        self.reservations.withdraw_pending(
            robot,
            now_s=self.sim_time,
            reason="persistent_cycle_traffic_yield",
        )
        self.bridges.withdraw_waiter(robot)
        self._waiting.pop(robot, None)
        self._trip(
            robot, {}, start, refuge, "pickup",
            kind="parking", purpose="traffic_yield",
        )
        yield_trip = self._trips[robot]
        yield_trip["points"] = deque(alternative.points[1:])
        yield_trip["nodes"] = deque(alternative.nodes[1:])
        yield_trip["traffic_yield_resume"] = resume
        self._refresh_admission_preview(robot, yield_trip)
        self.traffic_yield_count += 1
        self._event(
            "traffic_yield_selected",
            f"{robot} 순환 대기 양보 이동 · {start}→{refuge}",
            robot,
            reason=self.last_replan_reason,
            cycle=list(cycle),
            start=start,
            refuge=refuge,
            distance_m=distance,
            route_nodes=list(alternative.nodes),
        )
        return True

    @staticmethod
    def _consume_chunk_route(pose, trip, chunk) -> None:
        """Remove authored waypoints as the active kinematic chunk passes them."""
        if chunk["corridor"] is not None:
            reached = chunk["count"] - len(chunk["points"])
        else:
            travelled = math.dist(chunk["start"], (pose["x_m"], pose["y_m"]))
            route_points = list(trip["points"])
            reached = chunk["consumed"]
            while reached < chunk["count"]:
                candidate = route_points[reached - chunk["consumed"]]
                if math.dist(chunk["start"], candidate) > travelled + 1e-9:
                    break
                reached += 1
            # The endpoint may be within numeric tolerance while PathMotion is
            # still finishing its quintic deceleration. Retain that final
            # waypoint until the motion deque confirms completion.
            reached = min(reached, chunk["count"] - len(chunk["points"]))
        while chunk["consumed"] < reached:
            trip["points"].popleft()
            trip["nodes"].popleft()
            chunk["consumed"] += 1

    def _next_route_window(
        self, start, route_points, route_nodes, *, allow_terminal: bool = False,
    ) -> dict[str, Any]:
        """Return the next stoppable route window and its exact node mapping."""
        route_points = tuple(route_points)
        route_nodes = tuple(route_nodes)
        if not route_points or len(route_points) != len(route_nodes):
            raise RuntimeError("route window requires one node for each remaining point")
        holdable = self.bridges.holdable_nodes
        barrier = next(
            ((index, node) for index, node in enumerate(route_nodes)
             if node in self.bridges.queue_nodes
             or holdable is not None and node in holdable),
            None,
        )
        queue_barrier = (
            barrier if barrier is not None and barrier[1] in self.bridges.queue_nodes else None
        )
        if barrier is not None:
            barrier_index, _ = barrier
            route_points = route_points[:barrier_index + 1]
            route_nodes = route_nodes[:barrier_index + 1]
        corridor = self.corridors.atomic_span(start, route_points)
        if corridor is not None:
            selected = corridor.points
            identity = f"corridor-{corridor.corridor_id}"
            corridor_id = corridor.corridor_id
        else:
            end = route_points[0]
            count = 1
            dx, dy = end[0] - start[0], end[1] - start[1]
            for candidate in route_points[1:]:
                cx, cy = candidate[0] - start[0], candidate[1] - start[1]
                if (abs(dx * cy - dy * cx) > 1e-8 or dx * cx + dy * cy <= 0
                        or math.hypot(cx, cy) > self._max_segment_m + 1e-9):
                    break
                end, count = candidate, count + 1
            selected = route_points[:count]
            identity = "segment"
            corridor_id = None
        count = len(selected)
        terminal_service = False
        if holdable is not None and route_nodes[count - 1] not in holdable:
            next_hold = next(
                (index for index in range(count, len(route_nodes))
                 if route_nodes[index] in holdable),
                None,
            )
            if next_hold is None:
                if not allow_terminal or queue_barrier is not None:
                    raise RuntimeError(
                        f"route window has no holdable endpoint after {route_nodes[count - 1]}"
                    )
                next_hold = len(route_nodes) - 1
                terminal_service = True
            count = next_hold + 1
            selected = route_points[:count]
            identity = f"{identity}-to-holdable"
            if corridor_id is None and count > 1:
                corridor_id = "holdable-tail"
        return {
            "points": selected,
            "nodes": route_nodes[:count],
            "tiles": self._tiles((start, *selected)),
            "identity": identity,
            "corridor": corridor_id,
            "queue_barrier": None if queue_barrier is None else queue_barrier[1],
            "terminal_service": terminal_service,
            "retain_at_handover": False,
        }

    def _service_exit_after_drop(self, robot_id: str | None, station: str) -> str | None:
        explicit = self.staging_after_drop.get(station)
        if explicit is not None or self.bridges.holdable_nodes is None:
            return explicit
        if robot_id is None:
            raise RuntimeError(f"robot is required to select a holdable exit after {station}")
        home = self.homes[robot_id]
        route = self.book.plan(station, home, closed=self.closed)
        exit_node = next(
            (node for node in route.nodes[1:] if node in self.bridges.holdable_nodes),
            None,
        )
        if exit_node is None:
            raise RuntimeError(f"no holdable service exit from {station} toward {home}")
        return exit_node

    def _window_for_trip(
        self, start, route_points, route_nodes, trip, *, robot_id: str | None = None,
    ) -> dict[str, Any]:
        """Extend only a final handover window far enough to guarantee its exit."""
        window = self._next_route_window(
            start, route_points, route_nodes, allow_terminal=True,
        )
        if len(window["points"]) != len(tuple(route_points)):
            return window
        extension_points: tuple[tuple[float, float], ...] = ()
        extension_nodes: tuple[str, ...] = ()
        extension_tiles = frozenset()
        extension_queue_barrier = None
        if trip["kind"] == "process_line" and trip["phase"] == "pickup":
            loaded = self.book.plan(trip["goal"], trip["task"].drop_station, closed=self.closed)
            if len(loaded.points) > 1:
                next_loaded = self._window_for_trip(
                    loaded.points[0], loaded.points[1:], loaded.nodes[1:],
                    {**trip, "phase": "drop", "goal": trip["task"].drop_station},
                    robot_id=robot_id,
                )
                extension_points = next_loaded["points"]
                extension_nodes = next_loaded["nodes"]
                extension_tiles = next_loaded["tiles"]
                extension_queue_barrier = next_loaded["queue_barrier"]
        elif trip["kind"] == "process_line" and trip["phase"] == "drop":
            staging = self._service_exit_after_drop(robot_id, trip["goal"])
            if staging is not None:
                egress = self.book.plan(trip["goal"], staging, closed=self.closed)
                next_egress = self._next_route_window(
                    egress.points[0], egress.points[1:], egress.nodes[1:],
                    allow_terminal=True,
                )
                extension_points = next_egress["points"]
                extension_nodes = next_egress["nodes"]
                extension_tiles = next_egress["tiles"]
                extension_queue_barrier = next_egress["queue_barrier"]
        elif (trip["kind"] == "parking" and trip.get("purpose") == "staging"
              and window["queue_barrier"] is None and self.bridges.holdable_nodes is None):
            # The complete authored spur ends at an off-corridor wait pocket.
            window["points"] = tuple(route_points)
            window["nodes"] = tuple(route_nodes)
            window["tiles"] = self._tiles((start, *window["points"]))
            return window
        if extension_points:
            window["nodes"] = window["nodes"] + extension_nodes
            window["tiles"] = window["tiles"] | extension_tiles
            window["queue_barrier"] = extension_queue_barrier
            window["retain_at_handover"] = True
        return window

    def _route_via_safe_exit(self, start, trip, route_points, route_nodes):
        """Route a stopped queue owner through its direction-specific safe exit."""
        matches = [
            queue for queue in self.bridges.safe_exit_nodes
            if math.dist(start, self.book.nodes[queue]) <= 1e-8
        ]
        if not matches:
            return tuple(route_points), tuple(route_nodes), None
        if len(matches) != 1:
            raise RuntimeError("bridge queue pose must identify one safe exit")
        queue = matches[0]
        safe_exit = self.bridges.safe_exit_nodes[queue]
        first = self.book.plan(queue, safe_exit, closed=self.closed)
        second = self.book.plan(safe_exit, trip["goal"], closed=self.closed)
        points = first.points[1:] + second.points[1:]
        nodes = first.nodes[1:] + second.nodes[1:]
        if len(points) != len(nodes) or not points or safe_exit not in nodes:
            raise RuntimeError("safe exit route must preserve node and point mapping")
        return points, nodes, safe_exit

    def _safe_exit_lookahead_blockers(
        self, robot_id: str, trip: Mapping[str, Any], queue: str | None,
    ) -> tuple[str, ...]:
        """Check the complete queue-to-exit path before occupying the queue."""
        if queue is None or queue not in self.bridges.safe_exit_nodes:
            return ()
        safe_exit = self.bridges.safe_exit_nodes[queue]
        route = self.book.plan(queue, safe_exit, closed=self.closed)
        if route.nodes[0] != queue or route.nodes[-1] != safe_exit:
            raise RuntimeError("safe exit lookahead must connect its queue to its exit")
        protected = self._tiles(route.points)
        blockers = set(self.bridges.blockers(
            robot_id,
            route.nodes[1:],
            route_tiles=protected,
            observed_tiles=self._admission_observations(robot_id, trip),
        ))
        # BridgeAdmission skips pose checks for a robot that already owns a
        # retained window.  Lookahead still has to see every peer's current
        # footprint; it must not release or retarget that owned window.
        blockers.update(
            peer for peer in self.ledger.robots
            if peer != robot_id and not protected.isdisjoint(self._footprint(peer))
        )
        return tuple(sorted(blockers))

    def _refresh_admission_preview(self, robot_id: str, trip: dict[str, Any]) -> None:
        if not trip["points"]:
            trip["admission_nodes"] = ()
            trip["admission_tiles"] = frozenset()
            return
        pose = self._poses[robot_id]
        route_points, route_nodes, _ = self._route_via_safe_exit(
            (pose["x_m"], pose["y_m"]), trip, tuple(trip["points"]), tuple(trip["nodes"]),
        )
        window = self._window_for_trip(
            (pose["x_m"], pose["y_m"]), route_points, route_nodes, trip,
            robot_id=robot_id,
        )
        trip["admission_nodes"] = window["nodes"]
        trip["admission_tiles"] = window["tiles"]

    def _move_trip(self, robot_id, trip, dt) -> None:
        if not trip["points"]:
            return
        chunk = self._chunks.get(robot_id)
        if chunk is None:
            pose = self._poses[robot_id]
            start = (pose["x_m"], pose["y_m"])
            replan = self._replans.get(robot_id)
            route_points = tuple(replan["points"] if replan is not None else trip["points"])
            route_nodes = tuple(replan["nodes"][1:] if replan is not None else trip["nodes"])
            route_points, route_nodes, safe_exit = self._route_via_safe_exit(
                start, trip, route_points, route_nodes,
            )
            if safe_exit is not None:
                self._replans.pop(robot_id, None)
                replan = None
                trip["points"] = deque(route_points)
                trip["nodes"] = deque(route_nodes)
            window = self._window_for_trip(
                start, route_points, route_nodes, trip, robot_id=robot_id,
            )
            exit_blockers = self._safe_exit_lookahead_blockers(
                robot_id, trip, window["queue_barrier"],
            )
            if exit_blockers:
                self.bridges.record_wait(robot_id, exit_blockers)
                self.reservations.withdraw_pending(
                    robot_id,
                    now_s=self.sim_time,
                    reason="safe_exit_lookahead_occupied",
                )
                self._waiting[robot_id] = {
                    "reason": "safe_exit_lookahead_occupied",
                    "blockers": list(exit_blockers),
                }
                return
            blockers = self.bridges.admit(
                robot_id,
                window["nodes"],
                route_tiles=window["tiles"],
                observed_tiles=self._admission_observations(robot_id, trip),
                handover_protected=window["retain_at_handover"],
            )
            if blockers:
                self.reservations.withdraw_pending(
                    robot_id,
                    now_s=self.sim_time,
                    reason="rolling_route_window_admission_denied",
                )
                self._waiting[robot_id] = {
                    "reason": "bridge_admission_occupied", "blockers": list(blockers),
                }
                return
            sequence = self._chunk_seq[robot_id] + 1
            route_kind = f"replan-{replan['attempt']}-" if replan is not None else ""
            if not self._claim(
                robot_id, f"{route_kind}{window['identity']}-{sequence}", window["tiles"],
            ):
                self.bridges.release(robot_id, stopped=True)
                return
            if replan is not None:
                trip["points"] = deque(route_points)
                trip["nodes"] = deque(route_nodes)
                self.replan_applied_count += 1
                self._event(
                    "traffic_replan_applied",
                    f"{robot_id} 순환 대기 우회 적용 · {replan['start']}→{trip['goal']}",
                    robot_id,
                    reason=replan["reason"],
                    cycle=list(replan["cycle"]),
                    route_nodes=list(replan["nodes"]),
                    reservation_generation=self._leases[robot_id]["generation"],
                )
                del self._replans[robot_id]
            self._chunk_seq[robot_id] = sequence
            chunk = {
                "points": deque(window["points"] if window["corridor"] is not None
                                else (window["points"][-1],)),
                "count": len(window["points"]),
                "consumed": 0,
                "start": start,
                "end": window["points"][-1],
                "corridor": window["corridor"],
                "retain_at_handover": window["retain_at_handover"],
            }
            self._chunks[robot_id] = chunk
        lease = self._leases[robot_id]
        if not self.reservations.permission(robot_id, lease["generation"], lease["goal"], now_s=self.sim_time):
            raise RuntimeError("operations movement lost its reservation")
        self.motions[robot_id].advance(self._poses[robot_id], chunk["points"], dt)
        if not self._footprint(robot_id).issubset(lease["tiles"]):
            raise RuntimeError("operations movement escaped reserved footprint")
        self._consume_chunk_route(self._poses[robot_id], trip, chunk)
        if not chunk["points"]:
            if chunk["consumed"] != chunk["count"]:
                raise RuntimeError("operations route snapshot did not consume the completed chunk")
            del self._chunks[robot_id]
            self._observe_all()
            self._park_hold(robot_id)
            if chunk["retain_at_handover"] and not trip["points"]:
                return
            self.bridges.release(robot_id, stopped=True)
            self._refresh_admission_preview(robot_id, trip)

    def _admission_observations(self, robot_id: str, trip: Mapping[str, Any]) -> dict[str, frozenset]:
        """Return observed poses; acquired windows are enforced by BridgeAdmission.

        An unacquired preview is not occupancy. Treating it as occupancy can
        form a false two-vehicle cycle when one preview reaches a peer's stopped
        pose and the peer is ordered behind that preview. Movement admission is
        sequential, so the first acquired window becomes protected before the
        next vehicle is examined.
        """
        return {peer: self._footprint(peer) for peer in self.ledger.robots}

    def _prospective_trip_blockers(
        self,
        robot_id: str,
        pickup: str,
        destination: str,
        *,
        pickup_route: Any | None = None,
        loaded_route: Any | None = None,
    ) -> tuple[str, ...]:
        """Check the first stoppable route window before business assignment."""
        first = pickup_route or self.book.plan(self._locations[robot_id], pickup, closed=self.closed)
        if len(first.points) == 1:
            return ()
        window = self._next_route_window(
            first.points[0], first.points[1:], first.nodes[1:], allow_terminal=True,
        )
        observations = {}
        for peer in self.ledger.robots:
            observations[peer] = self._footprint(peer)
        return self.bridges.blockers(
            robot_id,
            window["nodes"],
            route_tiles=window["tiles"],
            observed_tiles=observations,
        )

    def _charging_approach_option(self, robot_id: str, goal: str) -> dict[str, Any] | None:
        """Describe one dock approach that can start from the current safe hold."""

        if (robot_id in self._chunks or not self.motions[robot_id].stopped
                or self.bridges.has_window(robot_id) or self.bridges.held(robot_id)):
            return None
        start = anchor_at_pose(self.book.nodes, self._poses[robot_id])
        if start is None:
            return None
        route = self.book.plan(start, goal, closed=self.closed)
        if len(route.points) == 1:
            return {
                "start": start,
                "goal": goal,
                "distance_m": 0.0,
                "blockers": (),
                "window_nodes": (),
            }
        trip = {"kind": "charging", "phase": "charge_approach", "goal": goal}
        route_points, route_nodes, _ = self._route_via_safe_exit(
            route.points[0], trip, route.points[1:], route.nodes[1:],
        )
        window = self._window_for_trip(
            route.points[0], route_points, route_nodes, trip, robot_id=robot_id,
        )
        blockers = set(self._safe_exit_lookahead_blockers(
            robot_id, trip, window["queue_barrier"],
        ))
        blockers.update(self.bridges.blockers(
            robot_id,
            window["nodes"],
            route_tiles=window["tiles"],
            observed_tiles=self._admission_observations(robot_id, trip),
        ))
        distance_m = sum(
            math.dist(start_point, end_point)
            for start_point, end_point in zip((route.points[0], *route_points), route_points)
        )
        return {
            "start": start,
            "goal": goal,
            "distance_m": distance_m,
            "blockers": tuple(sorted(blockers)),
            "window_nodes": tuple(window["nodes"]),
        }

    def _reassign_charging_approach(self, robot_id: str, goal: str) -> None:
        """Replace one stopped pre-contact approach without releasing pose occupancy."""

        trip = self._trips.get(robot_id)
        if (trip is None or trip.get("kind") != "charging"
                or trip.get("phase") != "charge_approach"
                or self._charging_approach_option(robot_id, goal) is None):
            raise RuntimeError("charging approach reassignment requires one stopped safe hold")
        self.reservations.withdraw_pending(
            robot_id,
            now_s=self.sim_time,
            reason="charging_approach_reassigned",
        )
        self.bridges.withdraw_waiter(robot_id)
        self._waiting.pop(robot_id, None)
        self._replans.pop(robot_id, None)
        self._trip(
            robot_id, {}, self._locations[robot_id], goal,
            "charge_approach", kind="charging",
        )

    def _can_withdraw_queued_parking(self, robot_id: str) -> bool:
        trip = self._trips.get(robot_id)
        if (trip is None or trip["kind"] != "parking" or trip.get("purpose", "home") != "home"
                or robot_id in self._chunks or not self.motions[robot_id].stopped
                or self.bridges.has_window(robot_id)):
            return False
        anchor = self.book.nodes[self._locations[robot_id]]
        pose = self._poses[robot_id]
        return math.dist(anchor, (pose["x_m"], pose["y_m"])) <= 1e-9

    def _withdraw_queued_parking(self, robot_id: str) -> bool:
        """Make an unstarted home trip available for higher-priority work."""
        if not self._can_withdraw_queued_parking(robot_id):
            return False
        self.reservations.withdraw_pending(
            robot_id,
            now_s=self.sim_time,
            reason="queued_home_parking_preempted",
        )
        self.bridges.withdraw_waiter(robot_id)
        self._waiting.pop(robot_id, None)
        self._replans.pop(robot_id, None)
        del self._trips[robot_id]
        self._event("parking_preempted", f"{robot_id} 업무 우선 배정을 위해 귀가 대기 철회", robot_id)
        return True

    def _withdraw_parking_for_charge(self, robot_id: str) -> bool:
        """Interrupt a home trip only at a stopped, unowned authored hold."""

        trip = self._trips.get(robot_id)
        if (trip is None or trip["kind"] != "parking" or trip.get("purpose", "home") != "home"
                or robot_id in self._chunks or not self.motions[robot_id].stopped
                or self.bridges.has_window(robot_id) or self.bridges.held(robot_id)):
            return False
        anchor = anchor_at_pose(self.book.nodes, self._poses[robot_id])
        if (anchor is None
                or (self.bridges.holdable_nodes is not None
                    and anchor not in self.bridges.holdable_nodes)
                or anchor in self.bridges.queue_nodes):
            return False
        self.reservations.withdraw_pending(
            robot_id,
            now_s=self.sim_time,
            reason="queued_home_parking_preempted_for_charge",
        )
        self.bridges.withdraw_waiter(robot_id)
        self._waiting.pop(robot_id, None)
        self._replans.pop(robot_id, None)
        interrupted_goal = trip["goal"]
        del self._trips[robot_id]
        self._locations[robot_id] = anchor
        self._charging_parking_resume[robot_id] = interrupted_goal
        self._event(
            "charge_parking_preempted",
            f"{robot_id} 정지 주차 경로 철회 · 충전 접근 우선",
            robot_id,
            anchor=anchor,
            interrupted_goal=interrupted_goal,
        )
        return True

    def _arrive_trip(self, robot_id, trip) -> None:
        if not self.motions[robot_id].stopped:
            raise RuntimeError("handover attempted before kinematic stop")
        self.bridges.release(robot_id, stopped=True)
        if self.operations_charging is not None and self.operations_charging.arrive(self, robot_id, trip):
            self._observe_all()
            self._park_hold(robot_id)
        elif trip["kind"] == "parking" and trip.get("purpose") == "traffic_yield":
            resume = trip.get("traffic_yield_resume")
            if not isinstance(resume, dict) or resume.get("phase") != "pickup":
                raise RuntimeError("traffic yield must resume one pickup trip")
            refuge = trip["goal"]
            self._locations[robot_id] = refuge
            self._trip(
                robot_id,
                resume["task"],
                refuge,
                resume["goal"],
                resume["phase"],
                kind=resume["kind"],
            )
            self.traffic_yield_completed_count += 1
            self._event(
                "traffic_yield_completed",
                f"{robot_id} 양보 지점 도착 · 원래 픽업 재개",
                robot_id,
                refuge=refuge,
                pickup=resume["goal"],
            )
        elif trip["kind"] == "parking":
            self._locations[robot_id] = trip["goal"]
            del self._trips[robot_id]
            self._observe_all()
            self._park_hold(robot_id)
            if trip.get("purpose", "home") == "staging":
                self._event("staged", f"{robot_id} 하차 후 대기점 진입", robot_id)
            else:
                self._event("parked", f"{robot_id} 통로 밖 주차 완료", robot_id)
        else:
            if trip["phase"] == "drop":
                self._deliveries[robot_id] += 1
                self._jobs.pop(robot_id, None)
            super()._arrive_trip(robot_id, trip)
        self._idle_since[robot_id] = self.sim_time

    def _advance_auxiliary(self, dt, before) -> None:
        if self.operations_charging is None:
            return
        self.operations_charging.record(self, dt)
        if self.operations_charging.faults:
            robots = ", ".join(sorted(self.operations_charging.faults))
            raise RuntimeError(f"operations charging observation fault: {robots}")
        self.operations_charging.prepare(self)

    def _prepare_auxiliary(self) -> None:
        # Operations charging admission already ran immediately before dispatch.
        return

    def _advance_step(self, dt) -> None:
        before = {r: (p["x_m"], p["y_m"]) for r, p in self._poses.items()}
        super()._advance_step(dt)
        self._observe_all()
        for robot, pose in self._poses.items():
            self._travel_m[robot] += math.dist(before[robot], (pose["x_m"], pose["y_m"]))
            if robot in self._waiting and self.motions[robot].stopped:
                self._wait_s[robot] += dt
        moving = sum(not motion.stopped for motion in self.motions.values())
        self.peak_moving_robots = max(self.peak_moving_robots, moving)
        if self.sim_time - self._last_cycle_check_s >= 1.0:
            cycle = self.reservations.detect_wait_cycle(now_s=self.sim_time)
            bridge_cycle = self.bridges.wait_cycle()
            if bridge_cycle != self._bridge_cycle_candidate:
                self._bridge_cycle_candidate = bridge_cycle
                self._bridge_cycle_since_s = self.sim_time if bridge_cycle is not None else None
            persistent_bridge_cycle = (
                bridge_cycle
                if bridge_cycle is not None
                and self._bridge_cycle_since_s is not None
                and self.sim_time - self._bridge_cycle_since_s
                >= self.reservations.cycle_persistence_s
                else None
            )
            if cycle is not None and cycle != self.wait_cycle:
                self.wait_cycle_episodes += 1
                self._event("traffic_wait_cycle", f"지속 순환 대기: {', '.join(cycle)}", None)
            if cycle is not None:
                self._schedule_cycle_replan(cycle)
            elif persistent_bridge_cycle is not None:
                self._schedule_cycle_replan(persistent_bridge_cycle)
            self.wait_cycle = cycle
            self._last_cycle_check_s = self.sim_time
        ids = list(self._poses)
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                pa, pb = self._poses[a], self._poses[b]
                clearance = math.hypot(pa["x_m"] - pb["x_m"], pa["y_m"] - pb["y_m"]) - 2 * self.collision_radius
                self.min_clearance_m = min(self.min_clearance_m, clearance) if self.min_clearance_m is not None else clearance
                if clearance < 0:
                    raise RuntimeError("operations circular collision envelopes overlap")

    def advance(self, dt: float) -> None:
        if type(dt) not in (float, int) or not math.isfinite(dt) or not 0 <= dt <= 1:
            raise ValueError("operations step must be between zero and one second")
        if self.paused or dt == 0:
            return
        # Keep coordinator observations fresh independently of API polling cadence.
        steps = max(1, math.ceil(dt / 0.1))
        for _ in range(steps):
            self._advance_step(dt / steps)

    def set_aisle_closed(self, parameters: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(parameters, dict) or set(parameters) != {"closed"} or type(parameters["closed"]) is not bool:
            raise ValueError("aisle change requires a boolean closed")
        self.requested_closed = parameters["closed"]
        self._event("aisle_closure_requested", "통로 변경 요청 · 승인된 운송 후 적용", None)
        self._dispatch()
        return {"requested_closed": self.requested_closed, "closed": self.closed,
                "status": "applied" if self.requested_closed == self.closed else "draining"}

    def snapshot(self) -> dict[str, Any]:
        result = super().snapshot()
        result.update(execution_mode="operations", dispatch_policy=self.dispatch_policy,
                      model_scope="Kinematic operations with bounded segments, atomic shared-corridor reservations and station gates; no ROS, contact physics or calibrated real factory")
        result["capabilities"].append("set_aisle_closed")
        result["traffic"] = {
            "reservation_policy": "bounded_segment_with_atomic_shared_corridors_and_station_gates",
            "closed": self.closed,
            "requested_closed": self.requested_closed, "closure_pending": self.closed != self.requested_closed,
            "wait_s": dict(self._wait_s), "travel_m": dict(self._travel_m),
            "assignments": dict(self._assignments), "deliveries": dict(self._deliveries),
            "observed_min_clearance_m": self.min_clearance_m, "peak_moving_robots": self.peak_moving_robots,
            "reservation_radius_m": self.radius, "motion_model": "quintic_rest_to_rest",
            "quiescent": (not self._trips and all(self._locations[r] == self.homes[r] for r in self.homes)
                          and (self.operations_charging is None
                               or not self.operations_charging.queue
                               and not self.operations_charging.aux
                               and not self.operations_charging.relocating
                               and not self.operations_charging.faults)),
            "station_owners": dict(self._station_owners),
            "bridge_admission": self.bridges.snapshot(),
            "reservation_wait_cycle": list(self.wait_cycle) if self.wait_cycle else None,
            "reservation_wait_cycle_episodes": self.wait_cycle_episodes,
            "replan_attempt_count": self.replan_attempt_count,
            "replan_applied_count": self.replan_applied_count,
            "replan_unavailable_count": self.replan_unavailable_count,
            "last_replan_reason": self.last_replan_reason,
            "pending_replans": sorted(self._replans),
            "traffic_yield_count": self.traffic_yield_count,
            "traffic_yield_completed_count": self.traffic_yield_completed_count,
            "traffic_yield_robots": sorted(
                robot for robot, trip in self._trips.items()
                if trip["kind"] == "parking" and trip.get("purpose") == "traffic_yield"
            ),
        }
        for row in result["robots"]:
            robot = row["id"]
            motion = self.motions[robot]
            row.update(speed_mps=0.0 if self.paused else motion.linear_m_s,
                       angular_speed_rad_s=0.0 if self.paused else motion.angular_rad_s,
                       navigation_phase=motion.phase, pose_source="kinematic_operations",
                       reservation_generation=self._leases[robot]["generation"])
            trip = self._trips.get(robot)
            if (trip is not None and trip["kind"] == "parking"
                    and trip.get("purpose") == "staging"):
                row.update(
                    task_id=f"stage:{robot}", task_type="staging",
                    task_label="하차 후 통로 비움", movement_purpose="post_drop_staging",
                )
            elif (trip is not None and trip["kind"] == "parking"
                  and trip.get("purpose") == "traffic_yield"):
                resume_task = trip["traffic_yield_resume"]["task"]
                resume_task_id = (
                    resume_task.get("id") if isinstance(resume_task, Mapping)
                    else resume_task.task_id
                )
                row.update(
                    task_id=resume_task_id,
                    task_label="교통 양보 후 픽업 재개",
                    movement_purpose="traffic_yield",
                )
            if self.operations_charging is not None:
                row.update(self.operations_charging.robot_snapshot(robot))
                phase = row["charging_phase"]
                auxiliary = self.operations_charging.aux.get(robot)
                if phase is not None:
                    trip = self._trips.get(robot)
                    slot = self.operations_charging.slots[auxiliary["dock_id"]] if auxiliary else None
                    destination = (trip["goal"] if trip and trip["kind"] == "charging"
                                   else slot["exit_anchor"] if auxiliary and phase == "exit_dwell"
                                   else slot["anchor"] if slot else self.homes[robot])
                    row.update(
                        task_id=f"charge:{robot}", task_type="charging",
                        task_label="운영 가상 충전", destination=destination,
                        status="paused" if self.paused else "moving" if not motion.stopped else "waiting",
                        waiting_reason="작업 일시정지" if self.paused else f"운영 충전: {phase}",
                    )
            if robot in self._waiting and motion.stopped:
                wait = self._waiting[robot]
                row.update(status="waiting", waiting_reason=f"교통 예약 대기: {wait['reason']}", blockers=wait["blockers"])
            elif self.requested_closed != self.closed and robot not in self._trips:
                row.update(status="waiting", waiting_reason="통로 변경 전 승인된 운송 종료 대기")
        result["kpis"].update(traffic_wait_s=sum(self._wait_s.values()),
                              peak_moving_robots=self.peak_moving_robots)
        if self.operations_charging is not None:
            charging_tasks = []
            for robot in self.operations_charging.queue:
                charging_tasks.append({
                    "id": f"charge:{robot}", "order_id": "", "type": "charging", "state": "queued",
                    "robot_id": robot, "source": self._locations[robot], "destination": self.homes[robot],
                    "payload_id": None, "progress": None,
                })
            for robot, auxiliary in self.operations_charging.aux.items():
                trip = self._trips.get(robot)
                slot = self.operations_charging.slots[auxiliary["dock_id"]]
                destination = (trip["goal"] if trip and trip["kind"] == "charging"
                               else slot["exit_anchor"] if auxiliary["phase"] == "exit_dwell"
                               else slot["anchor"])
                charging_tasks.append({
                    "id": f"charge:{robot}", "order_id": "", "type": "charging",
                    "state": auxiliary["phase"], "robot_id": robot, "source": self._locations[robot],
                    "destination": destination, "payload_id": None, "progress": None,
                })
            result["tasks"].extend(charging_tasks)
            result["charging"] = self.operations_charging.snapshot()
            result["kpis"]["active_robots"] = len(set(self._trips) | set(self.operations_charging.aux))
            result["kpis"]["pending_tasks"] = sum(t["state"] != "complete" for t in result["tasks"])
            result["model_scope"] += "; " + OPERATIONS_CHARGING_SCOPE
        return result
