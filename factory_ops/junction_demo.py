"""Encoder-driven, capacity-one junction demonstration, separate from Nav2.

Authored spawn + integrated wheel angles are the only map pose inputs. This
bounded demonstration does not localize from simulator ground truth.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .traffic import TrafficController


@dataclass
class Route:
    spawn: tuple[float, float, float]
    points: list[tuple[float, float]]
    entry_index: int | None = None
    exit_id: str | None = None
    index: int = 0
    reserved: bool = False
    released: bool = False
    finished: bool = False


class JunctionDemo:
    radius_m = 0.6371090915442774
    junction = (13.5, 15.5, 16.5, 18.5)

    def __init__(self, run_id: str):
        self.routes = {
            "carter1": Route((11., 17., 0.), [(12.5, 17.), (18., 17.), (21.5, 17.)], 1, "J:east"),
            "carter2": Route((15., 13.5, math.pi/2), [(15., 14.6), (15., 17.), (12., 17.), (9., 17.), (9., 11.)], 1, "J:west"),
            "carter3": Route((25., 11., math.pi/2), [(25., 17.), (29., 17.)]),
            "carter4": Route((9., 5., math.pi/2), [(9., 11.), (6., 11.)]),
        }
        self.traffic = TrafficController({"J": {"capacity": 1,
            "polygon_xy": [(13.5, 15.5), (16.5, 15.5), (16.5, 18.5), (13.5, 18.5)],
            "ports": {"east": {"xy": (18., 17.)}, "west": {"xy": (12., 17.)}}}}, {}, run_id=run_id)
        self.events: list[dict] = []
        self.states: dict[str, str] = {}
        self.clear_exits: dict[str, tuple[str, ...]] = {}

    def _event(self, robot: str, state: str, now: float):
        if self.states.get(robot) != state:
            self.states[robot] = state
            self.events.append({"robot_id": robot, "state": state, "sim_time_s": now})

    @staticmethod
    def map_pose(spawn, encoder):
        x, y, yaw = spawn
        return (x+math.cos(yaw)*encoder.x_m-math.sin(yaw)*encoder.y_m,
                y+math.sin(yaw)*encoder.x_m+math.cos(yaw)*encoder.y_m,
                yaw+encoder.yaw_rad)

    def observe(self, poses: dict[str, tuple[float, float, float]], now: float):
        lo_x, lo_y, hi_x, hi_y = self.junction
        for robot, (x, y, _) in poses.items():
            r = self.radius_m
            occupied = lo_x-r <= x <= hi_x+r and lo_y-r <= y <= hi_y+r
            exits = {"J:east": (18., 17.), "J:west": (12., 17.)}
            clear_exits = tuple(name for name, point in exits.items() if all(
                other == robot or math.hypot(point[0]-p[0], point[1]-p[1]) > 2*r
                for other, p in poses.items()))
            self.clear_exits[robot] = clear_exits
            self.traffic.update_observation(robot, pose_xy=(x, y), pose_time=now,
                heartbeat_time=now, occupied_resources=("J",) if occupied else (),
                clear_exits=clear_exits, now=now)

    def command(self, robot: str, pose, now: float, front_range_m: float):
        if not math.isfinite(front_range_m) or front_range_m <= .1:
            self._event(robot, "sensor_fault", now)
            return 0., 0., True
        route = self.routes[robot]
        x, y, yaw = pose
        if route.finished:
            return 0., 0., False
        if route.reserved and not route.released:
            lo_x, lo_y, hi_x, hi_y = self.junction
            r = self.radius_m
            clear = not (lo_x-r <= x <= hi_x+r and lo_y-r <= y <= hi_y+r)
            # An admission before entry must not qualify as a completed crossing.
            beyond = x > hi_x+r if robot == "carter1" else x < lo_x-r
            if clear and beyond:
                result = self.traffic.release(robot, ("J",), route.exit_id, now=now)
                if result.granted:
                    route.released = True
                    self._event(robot, "junction_cleared", now)
        if route.entry_index is not None and not route.reserved:
            # Reserve the crossing and its downstream space during approach.
            # A waiter cannot occupy a committed exit and then enter first.
            decision = None
            if route.exit_id in self.clear_exits.get(robot, ()):
                decision = self.traffic.request(robot, ("J",), route.exit_id, now=now)
            if decision is not None and decision.granted:
                route.reserved = True
                self._event(robot, "crossing_granted", now)
            elif route.index >= route.entry_index:
                self._event(robot, "yield_to_cross_traffic", now)
                return 0., 0., False
        tx, ty = route.points[route.index]
        distance = math.hypot(tx-x, ty-y)
        if distance < .12:
            route.index += 1
            if route.index == len(route.points):
                route.finished = True
                self._event(robot, "delivery_arrived", now)
                return 0., 0., False
            return self.command(robot, pose, now, front_range_m)
        error = math.atan2(math.sin(math.atan2(ty-y, tx-x)-yaw), math.cos(math.atan2(ty-y, tx-x)-yaw))
        if abs(error) > .22:
            self._event(robot, "turning", now)
            return 0., max(-.8, min(.8, 1.5*error)), False
        if front_range_m < 1.0:
            self._event(robot, "range_guard_stop", now)
            return 0., 0., True
        self._event(robot, "transport_resumed" if route.reserved else "transporting", now)
        # Stopping distance is calculated from the existing .35m/s² envelope.
        linear = min(.55, .8*distance, math.sqrt(2*.35*max(0., distance-.05)))
        return linear, max(-.6, min(.6, 1.3*error)), False

    def result(self):
        a = [e for e in self.events if e["robot_id"] == "carter1"]
        b = [e for e in self.events if e["robot_id"] == "carter2"]
        times = lambda rows, state: [e["sim_time_s"] for e in rows if e["state"] == state]
        wait, clear, resume = times(b, "yield_to_cross_traffic"), times(a, "junction_cleared"), times(b, "crossing_granted")
        sequence = bool(wait and clear and resume and wait[0] < clear[0] <= resume[0])
        return {"all_arrived": all(r.finished for r in self.routes.values()),
            "yield_clear_resume_observed": sequence,
            "wait_start_s": wait[0] if wait else None,
            "priority_clear_s": clear[0] if clear else None,
            "waiter_resume_s": resume[0] if resume else None,
            "events": self.events, "control_pose_source": "wheel encoders + authored spawn",
            "active_detour_planning": False, "nav2_integration": False}
