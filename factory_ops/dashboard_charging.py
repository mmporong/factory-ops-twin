"""Optional CPU energy and geometric charging, separate from physical evidence."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from functools import lru_cache
import math
from typing import TYPE_CHECKING, Any, Mapping, cast

from .charging import ChargingFleet
from .energy import EnergyFleet, number

if TYPE_CHECKING:
    from .dashboard_model import DashboardModel


SCOPE = "CPU virtual battery/geometric docking; no physics, contact sensor or ROS evidence"


def angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def move_path(pose: dict[str, float], points: deque, dt: float, speed: float, turn_rate: float) -> None:
    """Rotate and translate over authored CPU time; never jump to a dock pose."""
    remaining = dt
    while points and remaining > 1e-12:
        x, y = points[0]
        dx, dy = x - pose["x_m"], y - pose["y_m"]
        distance = math.hypot(dx, dy)
        if distance <= 1e-10:
            points.popleft()
            continue
        error = angle(math.atan2(dy, dx) - pose["yaw_rad"])
        rotation = math.copysign(min(abs(error), turn_rate * remaining), error)
        pose["yaw_rad"] = angle(pose["yaw_rad"] + rotation)
        remaining = max(0., remaining - abs(rotation) / turn_rate)
        if abs(error - rotation) > 1e-10 or remaining <= 0:
            break
        travel = min(distance, speed * remaining)
        pose["x_m"] += dx / distance * travel
        pose["y_m"] += dy / distance * travel
        remaining = max(0., remaining - travel / speed)
        if travel >= distance:
            points.popleft()


class DashboardCharging:
    def __init__(self, model: DashboardModel, profile: Mapping[str, Any]) -> None:
        self.profile = deepcopy(dict(profile))
        p = self.profile
        if p.get("schema_version") != 1 or set(p["initial_soc"]) != set(model.ledger.robots):
            raise ValueError("CPU charging profile must cover exactly the model fleet")
        for key in ("trigger_soc", "resume_soc", "speed_m_s", "angular_speed_rad_s", "max_step_s",
                    "alignment_dwell_s", "contact_dwell_s", "exit_dwell_s", "pad_radius_m", "yaw_tolerance_rad"):
            number(p[key], key, positive=True)
        number(p["assignment_wait_s"], "assignment_wait_s")
        if not 0 < p["energy"]["reserve_fraction"] < p["trigger_soc"] < p["resume_soc"] <= 1:
            raise ValueError("reserve < charge trigger < resume SOC is required")
        self.energy = EnergyFleet(p["energy"], p["initial_soc"])
        if p["energy"]["charger_power_w"] * p["energy"]["charger_efficiency"] <= p["energy"]["idle_power_w"]:
            raise ValueError("CPU charging needs positive net charger power")
        if p["max_step_s"] > p["energy"]["max_observation_gap_s"] / 2:
            raise ValueError("CPU substep must leave margin below the energy observation gap")
        self.slots = p["docks"]
        if not self.slots or len({s["exit_anchor"] for s in self.slots.values()}) != len(self.slots):
            raise ValueError("CPU docks require distinct exit anchors")
        for slot in self.slots.values():
            if (slot["anchor"] not in model.book.layout.pockets or slot["exit_anchor"] not in model.book.nodes
                    or slot["anchor"] == slot["exit_anchor"]
                    or type(slot["yaw_rad"]) not in (int, float) or not math.isfinite(slot["yaw_rad"])):
                raise ValueError("dock anchor/exit/yaw differs from authored geometry")
        self.run_id, self.book = model.ledger.run_id, model.book
        if any(math.dist(self.book.nodes[s["anchor"]], self.book.nodes[s["exit_anchor"]]) <= p["pad_radius_m"]
               for s in self.slots.values()):
            raise ValueError("dock exits must lie outside their geometric pads")
        self.docks = ChargingFleet(self.run_id, {key: s["anchor"] for key, s in self.slots.items()},
            max_gap_s=p["energy"]["max_observation_gap_s"], contact_dwell_s=p["contact_dwell_s"],
            stopped_linear_m_s=p["energy"]["charging_stopped_linear_m_s"],
            stopped_angular_rad_s=p["energy"]["charging_stopped_angular_rad_s"], resume_soc=p["resume_soc"])
        self.queue: list[str] = []
        self.aux: dict[str, dict[str, Any]] = {}
        self.exit_holds: dict[str, str] = {}
        self.denied: set[str] = set()
        self.faults: dict[str, str] = {}
        self.speeds = {r: 0. for r in model.ledger.robots}
        net_power = p["energy"]["charger_power_w"] * p["energy"]["charger_efficiency"] - p["energy"]["idle_power_w"]
        approach_bound = max(self.plan(node, s["anchor"]).distance_m / p["speed_m_s"]
            + math.pi * len(self.plan(node, s["anchor"]).points) / p["angular_speed_rad_s"]
            for node in self.book.nodes for s in self.slots.values())
        exit_bound = max(self.plan(s["anchor"], s["exit_anchor"]).distance_m / p["speed_m_s"]
            + math.pi * len(self.plan(s["anchor"], s["exit_anchor"]).points) / p["angular_speed_rad_s"]
            for s in self.slots.values())
        cycle_bound = (p["energy"]["capacity_wh"] * p["resume_soc"] * 3600 / net_power + approach_bound + exit_bound
                       + p["alignment_dwell_s"] + p["contact_dwell_s"] + p["exit_dwell_s"])
        self.wait_budget_s = p["assignment_wait_s"] + math.ceil((len(model.ledger.robots) - 1) / len(self.slots)) * cycle_bound
        for robot in model.ledger.robots:
            self.energy.observe(robot, model.sim_time, 0., 0., loaded=False)
        self.prepare(model)

    @property
    def max_step_s(self) -> float:
        return self.profile["max_step_s"]

    @lru_cache(maxsize=256)
    def plan(self, start: str, goal: str):
        return self.book.plan(start, goal)

    def loaded(self, model: DashboardModel, robot: str) -> bool:
        inventory = cast(dict[str, Any], model.inventory.snapshot())
        return (model.ledger.robots[robot].payload_id is not None
                or robot in inventory["robot_loads"])

    def available(self, robot: str) -> bool:
        return robot not in self.queue and self.docks.can_dispatch(robot) and robot not in self.faults

    def can_assign(self, model: DashboardModel, robot: str, pickup: str, destination: str) -> bool:
        if not self.available(robot):
            return False
        first = self.plan(model._locations[robot], pickup)
        second = self.plan(pickup, destination)
        if any(owner != robot and anchor in (*first.nodes, *second.nodes)
               for anchor, owner in self.exit_holds.items()):
            return False
        # Include other robots' approach, charge-from-zero, exit and dwell costs.
        p = self.profile
        for slot in self.slots.values():
            returning = self.plan(destination, slot["anchor"])
            points = (*first.points, *second.points[1:], *returning.points[1:])
            yaw, turns = model._poses[robot]["yaw_rad"], 0.
            for a, b in zip(points, points[1:]):
                if a == b:
                    continue
                heading = math.atan2(b[1] - a[1], b[0] - a[0])
                turns += abs(angle(heading - yaw))
                yaw = heading
            turns += abs(angle(slot["yaw_rad"] - yaw))
            dwell = p["alignment_dwell_s"] + p["contact_dwell_s"]
            if self.energy.can_assign(robot, outbound_m=first.distance_m + second.distance_m,
                    charger_return_m=returning.distance_m,
                    expected_wait_s=self.wait_budget_s + dwell + turns / p["angular_speed_rad_s"],
                    nominal_speed_m_s=p["speed_m_s"], expected_rotation_rad=turns,
                    loaded=True, observed_at_s=model.sim_time):
                return True
        self.denied.add(robot)
        return False

    def prepare(self, model: DashboardModel) -> None:
        for robot in model.ledger.robots:
            if robot in model._trips or robot in self.faults or self.loaded(model, robot):
                continue
            soc = self.energy.snapshot(robot)["battery_pct"] / 100
            needs_charge = soc <= self.profile["trigger_soc"] or (robot in self.denied and soc < self.profile["resume_soc"])
            if needs_charge and self.available(robot):
                self.queue.append(robot)
                model._event("charge_queued", f"{robot} CPU 가상 충전 대기", robot)
        for robot in list(self.queue):
            dock_id = self.docks.reserve(robot, carrying_payload=self.loaded(model, robot))
            if dock_id is None:
                break
            slot = self.slots[dock_id]
            self.aux[robot] = {"phase": "approach", "points": deque(self.plan(model._locations[robot], slot["anchor"]).points[1:]),
                               "dock_id": dock_id, "goal": slot["anchor"], "stationary_since": None}
            self.queue.remove(robot)
            model._event("charge_reserved", f"{robot} {dock_id} CPU 슬롯 예약", robot)

    def _exit_available(self, model: DashboardModel, robot: str, anchor: str) -> bool:
        if self.exit_holds.get(anchor) not in (None, robot):
            return False
        # Drain accepted current legs, including empty returns, before blocking new dispatch.
        return not any(peer != robot and anchor in self.plan(trip["start"], trip["goal"]).nodes
                       for peer, trip in model._trips.items())

    def move(self, model: DashboardModel, dt: float) -> None:
        p = self.profile
        for robot, trip in self.aux.items():
            if robot in self.faults:
                continue
            pose, slot = model._poses[robot], self.slots[trip["dock_id"]]
            if trip["phase"] in ("approach", "undocking"):
                move_path(pose, trip["points"], dt, p["speed_m_s"], p["angular_speed_rad_s"])
                if not trip["points"]:
                    if trip["phase"] == "approach":
                        trip["phase"] = "aligning"
                        model._locations[robot] = slot["anchor"]
                    else:
                        model._locations[robot] = slot["exit_anchor"]
            elif trip["phase"] == "aligning":
                error = angle(slot["yaw_rad"] - pose["yaw_rad"])
                pose["yaw_rad"] = angle(pose["yaw_rad"] + math.copysign(min(abs(error), p["angular_speed_rad_s"] * dt), error))

    def pad_contact(self, pose: Mapping[str, float], dock_id: str) -> bool:
        slot = self.slots[dock_id]
        x, y = self.book.nodes[slot["anchor"]]
        return (math.hypot(pose["x_m"] - x, pose["y_m"] - y) <= self.profile["pad_radius_m"]
                and abs(angle(pose["yaw_rad"] - slot["yaw_rad"])) <= self.profile["yaw_tolerance_rad"])

    def record(self, model: DashboardModel, dt: float, before: Mapping[str, Mapping[str, float]]) -> None:
        for robot in model.ledger.robots:
            if robot in self.faults:
                continue
            pose, previous = model._poses[robot], before[robot]
            dx, dy = pose["x_m"] - previous["x_m"], pose["y_m"] - previous["y_m"]
            linear = math.copysign(math.hypot(dx, dy) / dt,
                                   dx * math.cos(pose["yaw_rad"]) + dy * math.sin(pose["yaw_rad"]))
            angular = angle(pose["yaw_rad"] - previous["yaw_rad"]) / dt
            self.speeds[robot] = linear
            stationary = (abs(linear) <= self.profile["energy"]["charging_stopped_linear_m_s"]
                          and abs(angular) <= self.profile["energy"]["charging_stopped_angular_rad_s"])
            confirmed = False
            try:
                if robot in self.aux:
                    confirmed = self._dock_observation(model, robot, stationary, linear, angular)
                self.energy.observe(robot, model.sim_time, linear, angular,
                                    loaded=self.loaded(model, robot), dock_contact_confirmed=confirmed)
            except ValueError as error:
                self.faults[robot] = str(error)
                model._event("charge_fault", f"{robot} CPU 에너지 오류: {error}", robot)

    def _dock_observation(self, model: DashboardModel, robot: str, stationary: bool,
                          linear: float, angular: float) -> bool:
        trip = self.aux[robot]
        dock_id = trip["dock_id"]
        contact = self.pad_contact(model._poses[robot], dock_id)
        if trip["phase"] == "approach":
            return False
        if trip["phase"] == "aligning":
            if not (contact and stationary):
                trip["stationary_since"] = None
            elif trip["stationary_since"] is None:
                trip["stationary_since"] = model.sim_time
            elif model.sim_time - trip["stationary_since"] >= self.profile["alignment_dwell_s"]:
                self.docks.arrived(robot, run_id=self.run_id, anchor=self.slots[dock_id]["anchor"], navigation_succeeded=True)
                trip["phase"], trip["stationary_since"] = "contact_pending", None
            return False
        soc = self.energy.snapshot(robot)["battery_pct"] / 100
        state = self.docks.observe(robot, run_id=self.run_id, dock_id=dock_id, stamp_s=model.sim_time,
                                  contact=contact, linear_m_s=linear, angular_rad_s=angular, soc=soc)
        if state["phase"] in ("undock_pending", "ready_to_leave"):
            if trip["phase"] != "undocking":
                slot = self.slots[dock_id]
                if not self._exit_available(model, robot, slot["exit_anchor"]):
                    trip.update(phase="undock_pending", stationary_since=None)
                    return False
                self.exit_holds[slot["exit_anchor"]] = robot
                trip.update(phase="undocking", goal=slot["exit_anchor"], stationary_since=None,
                            points=deque(self.plan(slot["anchor"], slot["exit_anchor"]).points[1:]))
                model._event("charge_undocking", f"{robot} CPU 충전 종료·출구 이동", robot)
            if trip["points"] or contact or not stationary:
                trip["stationary_since"] = None
            elif trip["stationary_since"] is None:
                trip["stationary_since"] = model.sim_time
            elif model.sim_time - trip["stationary_since"] >= self.profile["exit_dwell_s"]:
                self.docks.release(robot, run_id=self.run_id, observed_at_s=model.sim_time)
                del self.exit_holds[self.slots[dock_id]["exit_anchor"]]
                del self.aux[robot]
                self.denied.discard(robot)
                model._event("charge_released", f"{robot} CPU 충전 슬롯 반납·업무 복귀", robot)
            return False
        trip["phase"] = state["phase"]
        return state["contact_confirmed"] and stationary and contact

    def robot_snapshot(self, robot: str) -> dict[str, Any]:
        trip = self.aux.get(robot)
        phase = "fault" if robot in self.faults else trip["phase"] if trip else "queued" if robot in self.queue else None
        return {"battery_pct": None if robot in self.faults else self.energy.snapshot(robot)["battery_pct"],
                "battery_source": "authored_cpu_virtual_energy_model",
                "charging_phase": phase, "charging_dock": trip["dock_id"] if trip else None}

    def snapshot(self) -> dict[str, Any]:
        return {"scope": SCOPE, "docks": [{"id": key, "anchor": slot["anchor"], "owner": self.docks.owners.get(key),
                 "phase": self.aux[self.docks.owners[key]]["phase"] if key in self.docks.owners else "available"}
                 for key, slot in self.slots.items()], "queue": list(self.queue)}
