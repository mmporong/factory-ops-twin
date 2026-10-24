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
        self._last_cycle_check_s = -1.0
        self._replans: dict[str, dict[str, Any]] = {}
        self._replan_examined: set[tuple[Any, ...]] = set()
        self.replan_attempt_count = 0
        self.replan_applied_count = 0
        self.replan_unavailable_count = 0
        self.last_replan_reason: str | None = None
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
                self._trip(
                    robot, {}, self._locations[robot], self.homes[robot], "parking",
                    kind="parking", purpose="home",
                )
                self._waiting.pop(robot, None)

    def _dispatch_extra_jobs(self, draining: bool) -> None:
        """Admit specialized production jobs before sending idle vehicles home."""

    def _trip(self, robot_id, task, start, goal, phase, *, kind="order", purpose=None) -> None:
        self._replans.pop(robot_id, None)
        super()._trip(robot_id, task, start, goal, phase, kind=kind)
        route = self.book.plan(start, goal, closed=self.closed)
        route_nodes = route.nodes
        admission_points = route.points
        if kind == "process_line":
            destination = task.drop_station
            if phase == "pickup":
                # Acquire the loaded leg's bridges before occupying its pickup bay.
                loaded = self.book.plan(goal, destination, closed=self.closed)
                route_nodes += loaded.nodes
                admission_points += loaded.points[1:]
            staging = self.staging_after_drop.get(destination)
            if staging is not None:
                # The drop is not a safe release point. Protect its authored
                # egress spur in the same admission transaction.
                egress = self.book.plan(destination, staging, closed=self.closed)
                route_nodes += egress.nodes
                admission_points += egress.points[1:]
        self._admission_seq += 1
        self._trips[robot_id]["route_nodes"] = route_nodes
        self._trips[robot_id]["admission_tiles"] = self._tiles(tuple(admission_points))
        self._trips[robot_id]["admission_seq"] = self._admission_seq
        if kind == "parking":
            self._trips[robot_id]["purpose"] = purpose or "home"
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
            if (alternative is not None
                    and self.bridges.required(alternative.nodes).issubset(self.bridges.held(robot))):
                candidates.append((alternative.distance_m, robot, start, alternative))
        if not candidates:
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
            chunk["consumed"] += 1

    def _move_trip(self, robot_id, trip, dt) -> None:
        if not trip["points"]:
            return
        blockers = self.bridges.admit(
            robot_id,
            trip["route_nodes"],
            route_tiles=trip["admission_tiles"],
            observed_tiles=self._admission_observations(robot_id, trip),
        )
        if blockers:
            self.reservations.withdraw_pending(
                robot_id,
                now_s=self.sim_time,
                reason="macro_bridge_admission_denied",
            )
            self._waiting[robot_id] = {"reason": "bridge_admission_occupied", "blockers": list(blockers)}
            return
        chunk = self._chunks.get(robot_id)
        if chunk is None:
            pose = self._poses[robot_id]
            start = (pose["x_m"], pose["y_m"])
            replan = self._replans.get(robot_id)
            route_points = tuple(replan["points"] if replan is not None else trip["points"])
            corridor = self.corridors.atomic_span(start, route_points)
            if corridor is not None:
                selected = corridor.points
                identity = f"corridor-{corridor.corridor_id}"
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
            sequence = self._chunk_seq[robot_id] + 1
            route_kind = f"replan-{replan['attempt']}-" if replan is not None else ""
            if not self._claim(
                robot_id, f"{route_kind}{identity}-{sequence}", self._tiles((start, *selected)),
            ):
                return
            if replan is not None:
                trip["points"] = deque(route_points)
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
                "points": deque(selected if corridor is not None else (selected[-1],)),
                "count": len(selected),
                "consumed": 0,
                "start": start,
                "end": selected[-1],
                "corridor": corridor.corridor_id if corridor is not None else None,
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

    def _admission_observations(self, robot_id: str, trip: Mapping[str, Any]) -> dict[str, frozenset]:
        """Include earlier route intent so a bridge grant cannot strand it later.

        The monotonically assigned sequence breaks simultaneous intent ties. An
        earlier trip clears first; a later bridge request waits before entering.
        Current footprints are always included, independent of that priority.
        """
        observations = {}
        for peer in self.ledger.robots:
            tiles = self._footprint(peer)
            peer_trip = self._trips.get(peer)
            if (peer != robot_id and peer_trip is not None
                    and (peer in self._chunks or not self.motions[peer].stopped
                         or peer_trip["admission_seq"] < trip["admission_seq"])):
                tiles = tiles | peer_trip["admission_tiles"]
            observations[peer] = tiles
        return observations

    def _prospective_trip_blockers(
        self,
        robot_id: str,
        pickup: str,
        destination: str,
        *,
        pickup_route: Any | None = None,
        loaded_route: Any | None = None,
    ) -> tuple[str, ...]:
        """Check a complete pickup/drop intent before business assignment."""
        first = pickup_route or self.book.plan(self._locations[robot_id], pickup, closed=self.closed)
        loaded = loaded_route or self.book.plan(pickup, destination, closed=self.closed)
        nodes = first.nodes + loaded.nodes
        points = first.points + loaded.points[1:]
        staging = self.staging_after_drop.get(destination)
        if staging is not None:
            egress = self.book.plan(destination, staging, closed=self.closed)
            nodes += egress.nodes
            points += egress.points[1:]
        tiles = self._tiles(tuple(points))
        observations = {}
        for peer in self.ledger.robots:
            occupied = self._footprint(peer)
            peer_trip = self._trips.get(peer)
            if peer != robot_id and peer_trip is not None:
                occupied = occupied | peer_trip["admission_tiles"]
            observations[peer] = occupied
        return self.bridges.blockers(
            robot_id,
            nodes,
            route_tiles=tiles,
            observed_tiles=observations,
        )

    def _can_withdraw_queued_parking(self, robot_id: str) -> bool:
        trip = self._trips.get(robot_id)
        if (trip is None or trip["kind"] != "parking" or trip.get("purpose", "home") != "home"
                or robot_id in self._chunks or not self.motions[robot_id].stopped
                or self.bridges.held(robot_id)):
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

    def _arrive_trip(self, robot_id, trip) -> None:
        if not self.motions[robot_id].stopped:
            raise RuntimeError("handover attempted before kinematic stop")
        self.bridges.release(robot_id, stopped=True)
        if self.operations_charging is not None and self.operations_charging.arrive(self, robot_id, trip):
            self._observe_all()
            self._park_hold(robot_id)
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
            if cycle is not None and cycle != self.wait_cycle:
                self.wait_cycle_episodes += 1
                self._event("traffic_wait_cycle", f"지속 순환 대기: {', '.join(cycle)}", None)
            if cycle is not None:
                self._schedule_cycle_replan(cycle)
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
