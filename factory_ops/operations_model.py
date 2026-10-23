"""Four-AMR kinematic operations, distinct from preview and Isaac/ROS evidence.

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
from typing import Any
from uuid import NAMESPACE_URL, uuid5

from .dashboard_model import DashboardModel
from .drive_profile import DriveLimits
from .fleet_routing import swept_tiles
from .motion import DriveGeometry
from .operations import Transition
from .operations_motion import PathMotion
from .route_reservation import RouteReservationManager


class OperationsModel(DashboardModel):
    def __init__(self, config: Path, *, policy: str = "fifo", initial_orders: int = 4,
                 run_id: str | None = None) -> None:
        if policy not in ("fifo", "eta"):
            raise ValueError("operations policy must be fifo or eta")
        self.closed = False
        self.requested_closed = False
        self.dispatch_policy = policy
        super().__init__(config, initial_orders=initial_orders, run_id=run_id,
                         robot_stations=("P1", "P2", "P3", "P4"))
        drive = json.loads((config / "g2.json").read_text(encoding="utf-8"))
        self.geometry = DriveGeometry(**{k: v for k, v in drive["drive_geometry"].items() if k != "provenance"})
        self.limits = DriveLimits(**{k: v for k, v in drive["drive_profile"].items() if k != "provenance"})
        self.motions = {r: PathMotion(self.limits, self.geometry) for r in self.ledger.robots}
        self.homes = {r: f"P{i}" for i, r in enumerate(self.ledger.robots, 1)}
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
        self._chunk_seq = {r: 0 for r in self.ledger.robots}
        self._max_segment_m = max(math.dist(self.book.nodes[a], self.book.nodes[b]) for a, b in self.book.edges)
        self.min_clearance_m: float | None = None
        self.peak_moving_robots = 0
        self._observe_all()
        for robot in self.ledger.robots:
            point = self.book.nodes[self.homes[robot]]
            if not self._claim(robot, "initial-park", self._tiles((point,))):
                raise ValueError("initial parking footprints conflict")
            self._park_hold(robot)

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
        if draining and not self._trips and all(self._locations[r] == self.homes[r] for r in self.homes):
            self.closed = self.requested_closed
            self._event("aisle_closure_applied", "계획 통로 폐쇄 적용" if self.closed else "통로 재개방 적용", None)
            draining = False
        self._waiting = {r: w for r, w in self._waiting.items() if r in self._trips}
        while not draining:
            available = [r for r, state in self.ledger.robots.items()
                         if r not in self._trips and state.available and state.task_id is None and state.payload_id is None]
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
        for robot in self.ledger.robots:
            if robot in self._trips or self._locations[robot] == self.homes[robot]:
                continue
            # Vacate an idle handover without claiming a completed parking move.
            if draining or self.sim_time - self._idle_since[robot] >= 2.0:
                self._trip(robot, {}, self._locations[robot], self.homes[robot], "parking", kind="parking")
                self._waiting.pop(robot, None)

    def _trip(self, robot_id, task, start, goal, phase, *, kind="order") -> None:
        super()._trip(robot_id, task, start, goal, phase, kind=kind)
        self._waiting.pop(robot_id, None)

    def _move_trip(self, robot_id, trip, dt) -> None:
        if not trip["points"]:
            return
        chunk = self._chunks.get(robot_id)
        if chunk is None:
            pose = self._poses[robot_id]
            start = (pose["x_m"], pose["y_m"])
            end = trip["points"][0]
            count = 1
            dx, dy = end[0] - start[0], end[1] - start[1]
            for candidate in list(trip["points"])[1:]:
                cx, cy = candidate[0] - start[0], candidate[1] - start[1]
                if abs(dx * cy - dy * cx) > 1e-8 or dx * cx + dy * cy <= 0 or math.hypot(cx, cy) > self._max_segment_m + 1e-9:
                    break
                end, count = candidate, count + 1
            sequence = self._chunk_seq[robot_id] + 1
            if not self._claim(robot_id, f"segment-{sequence}", self._tiles((start, end))):
                return
            self._chunk_seq[robot_id] = sequence
            chunk = {"points": deque([end]), "count": count}
            self._chunks[robot_id] = chunk
        lease = self._leases[robot_id]
        if not self.reservations.permission(robot_id, lease["generation"], lease["goal"], now_s=self.sim_time):
            raise RuntimeError("operations movement lost its reservation")
        self.motions[robot_id].advance(self._poses[robot_id], chunk["points"], dt)
        if not self._footprint(robot_id).issubset(lease["tiles"]):
            raise RuntimeError("operations movement escaped reserved footprint")
        if not chunk["points"]:
            for _ in range(chunk["count"]):
                trip["points"].popleft()
            del self._chunks[robot_id]
            self._observe_all()
            self._park_hold(robot_id)

    def _arrive_trip(self, robot_id, trip) -> None:
        if not self.motions[robot_id].stopped:
            raise RuntimeError("handover attempted before kinematic stop")
        if trip["kind"] == "parking":
            self._locations[robot_id] = trip["goal"]
            del self._trips[robot_id]
            self._observe_all()
            self._park_hold(robot_id)
            self._event("parked", f"{robot_id} 통로 밖 주차 완료", robot_id)
        else:
            if trip["phase"] == "drop":
                self._deliveries[robot_id] += 1
                self._jobs.pop(robot_id, None)
            super()._arrive_trip(robot_id, trip)
        self._idle_since[robot_id] = self.sim_time

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
                      model_scope="Kinematic operations with bounded segment reservations and station gates; no ROS, contact physics or calibrated real factory")
        result["capabilities"].append("set_aisle_closed")
        result["traffic"] = {
            "reservation_policy": "bounded_segment_with_station_gates", "closed": self.closed,
            "requested_closed": self.requested_closed, "closure_pending": self.closed != self.requested_closed,
            "wait_s": dict(self._wait_s), "travel_m": dict(self._travel_m),
            "assignments": dict(self._assignments), "deliveries": dict(self._deliveries),
            "observed_min_clearance_m": self.min_clearance_m, "peak_moving_robots": self.peak_moving_robots,
            "reservation_radius_m": self.radius, "motion_model": "quintic_rest_to_rest",
            "quiescent": not self._trips and all(self._locations[r] == self.homes[r] for r in self.homes),
            "station_owners": dict(self._station_owners),
        }
        for row in result["robots"]:
            robot = row["id"]
            motion = self.motions[robot]
            row.update(speed_mps=0.0 if self.paused else motion.linear_m_s,
                       angular_speed_rad_s=0.0 if self.paused else motion.angular_rad_s,
                       navigation_phase=motion.phase, pose_source="kinematic_operations",
                       reservation_generation=self._leases[robot]["generation"])
            if robot in self._waiting and motion.stopped:
                wait = self._waiting[robot]
                row.update(status="waiting", waiting_reason=f"교통 예약 대기: {wait['reason']}", blockers=wait["blockers"])
            elif self.requested_closed != self.closed and robot not in self._trips:
                row.update(status="waiting", waiting_reason="통로 변경 전 승인된 운송 종료 대기")
        result["kpis"].update(traffic_wait_s=sum(self._wait_s.values()),
                              peak_moving_robots=self.peak_moving_robots)
        return result
