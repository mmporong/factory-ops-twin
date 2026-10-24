"""Virtual energy and reserved geometric docking for operations mode.

The adapter consumes the operations model's kinematic pose and motion state.
It does not infer physical contact, battery calibration, ROS execution, or
actuator behaviour from a completed authored route.
"""
from __future__ import annotations

from copy import deepcopy
import math
from typing import Any, Mapping, Protocol, cast

from .charging import ChargingFleet
from .energy import EnergyFleet, number


SCOPE = "Operations virtual battery/geometric docking; no physics, contact sensor, hardware calibration or ROS evidence"


class OperationsModelLike(Protocol):
    book: Any
    ledger: Any
    inventory: Any
    motions: dict[str, Any]
    homes: dict[str, str]
    closed: bool
    sim_time: float
    _locations: dict[str, str]
    _poses: dict[str, dict[str, float]]
    _trips: dict[str, dict[str, Any]]

    def _trip(self, robot_id: str, task: dict[str, Any], start: str, goal: str,
              phase: str, *, kind: str = "order") -> None: ...

    def _align_at_hold(self, robot: str, yaw_rad: float, dt: float) -> bool: ...

    def _event(self, kind: str, message: str, robot: str | None, **details: Any) -> None: ...


def _angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class OperationsCharging:
    """Coordinate charge demand without bypassing operations reservations."""

    def __init__(self, model: OperationsModelLike, profile: Mapping[str, Any]) -> None:
        self.profile = deepcopy(dict(profile))
        p = self.profile
        robots = tuple(model.ledger.robots)
        if p.get("schema_version") != 1 or set(p.get("initial_soc", {})) != set(robots):
            raise ValueError("operations charging profile must cover exactly the model fleet")
        for key in ("trigger_soc", "resume_soc", "max_step_s", "assignment_wait_s", "contact_dwell_s",
                    "exit_dwell_s", "pad_radius_m", "yaw_tolerance_rad"):
            number(p[key], key, positive=key not in {"assignment_wait_s"})
        if not 0 < p["energy"]["reserve_fraction"] < p["trigger_soc"] < p["resume_soc"] <= 1:
            raise ValueError("reserve < charge trigger < resume SOC is required")
        self.energy = EnergyFleet(p["energy"], p["initial_soc"])
        if p["max_step_s"] > 0.1 or p["max_step_s"] > p["energy"]["max_observation_gap_s"] / 2:
            raise ValueError("operations charging samples must retain a 0.1 second stale-observation margin")
        if p["energy"]["charger_power_w"] * p["energy"]["charger_efficiency"] <= p["energy"]["idle_power_w"]:
            raise ValueError("operations charging requires positive net charger power")

        self.slots = deepcopy(p["docks"])
        if not self.slots or len({slot["anchor"] for slot in self.slots.values()}) != len(self.slots):
            raise ValueError("operations charging requires distinct dock anchors")
        if len({slot["exit_anchor"] for slot in self.slots.values()}) != len(self.slots):
            raise ValueError("operations charging requires distinct exit anchors")
        for slot in self.slots.values():
            if (slot["anchor"] not in model.book.nodes or slot["exit_anchor"] not in model.book.nodes
                    or slot["anchor"] == slot["exit_anchor"]
                    or type(slot["yaw_rad"]) not in (int, float) or not math.isfinite(slot["yaw_rad"])):
                raise ValueError("dock anchor, exit or yaw differs from authored routing geometry")
            if math.dist(model.book.nodes[slot["anchor"]], model.book.nodes[slot["exit_anchor"]]) <= p["pad_radius_m"]:
                raise ValueError("dock exit must lie outside the authored pad radius")

        self.run_id = model.ledger.run_id
        self.docks = ChargingFleet(
            self.run_id,
            {dock: slot["anchor"] for dock, slot in self.slots.items()},
            max_gap_s=p["energy"]["max_observation_gap_s"],
            contact_dwell_s=p["contact_dwell_s"],
            stopped_linear_m_s=p["energy"]["charging_stopped_linear_m_s"],
            stopped_angular_rad_s=p["energy"]["charging_stopped_angular_rad_s"],
            resume_soc=p["resume_soc"],
        )
        self.queue: list[str] = []
        self.aux: dict[str, dict[str, Any]] = {}
        self.denied: set[str] = set()
        self.faults: dict[str, str] = {}
        self.relocating: dict[str, str] = {}
        self.home_swaps: list[dict[str, str]] = []
        self.parking_preempt_deferred: set[str] = set()
        for robot in robots:
            self.energy.observe(robot, model.sim_time, 0.0, 0.0, loaded=False)

        # Full-charge queueing is an authored upper bound, not a measured SLA.
        net_power_w = p["energy"]["charger_power_w"] * p["energy"]["charger_efficiency"] - p["energy"]["idle_power_w"]
        full_charge_s = p["energy"]["capacity_wh"] * p["resume_soc"] * 3600.0 / net_power_w
        route_bound_s = 0.0
        for start in model.book.nodes:
            for slot in self.slots.values():
                plan = self._plan(model, start, slot["anchor"])
                route_bound_s = max(route_bound_s,
                    model.motions[robots[0]].estimated_duration(plan.points, 0.0, slot["yaw_rad"]))
        exit_bound_s = max(
            model.motions[robots[0]].estimated_duration(
                self._plan(model, slot["anchor"], slot["exit_anchor"]).points,
                slot["yaw_rad"],
            )
            for slot in self.slots.values()
        )
        waves = math.ceil(max(0, len(robots) - 1) / len(self.slots))
        self.wait_budget_s = p["assignment_wait_s"] + waves * (
            full_charge_s + route_bound_s + exit_bound_s + p["contact_dwell_s"] + p["exit_dwell_s"]
        )
        self.prepare(model)

    def _plan(self, model: OperationsModelLike, start: str, goal: str):
        return model.book.plan(start, goal, closed=model.closed)

    def _loaded(self, model: OperationsModelLike, robot: str) -> bool:
        loads = getattr(model.inventory, "robot_loads", None)
        if not isinstance(loads, Mapping):
            loads = cast(dict[str, Any], model.inventory.snapshot())["robot_loads"]
        return model.ledger.robots[robot].payload_id is not None or robot in loads

    def available(self, robot: str) -> bool:
        return (robot not in self.queue and self.docks.can_dispatch(robot)
                and robot not in self.faults and robot not in self.relocating)

    def can_park(self, robot: str) -> bool:
        """Permit queue relocation while freezing owned/faulted dock lifecycles."""
        return robot not in self.aux and robot not in self.faults

    def _route_rotation(self, points: tuple[tuple[float, float], ...], yaw_rad: float,
                        final_yaw_rad: float | None = None) -> tuple[float, float]:
        total = 0.0
        for start, end in zip(points, points[1:]):
            if start == end:
                continue
            heading = math.atan2(end[1] - start[1], end[0] - start[0])
            total += abs(_angle(heading - yaw_rad))
            yaw_rad = heading
        if final_yaw_rad is not None:
            total += abs(_angle(final_yaw_rad - yaw_rad))
            yaw_rad = final_yaw_rad
        return total, yaw_rad

    def can_assign(self, model: OperationsModelLike, robot: str, pickup: str, destination: str) -> bool:
        if not self.available(robot):
            return False
        first = self._plan(model, model._locations[robot], pickup)
        loaded = self._plan(model, pickup, destination)
        distance = first.distance_m + loaded.distance_m
        initial_yaw = model._poses[robot]["yaw_rad"]
        for dock_id in sorted(self.slots):
            slot = self.slots[dock_id]
            returning = self._plan(model, destination, slot["anchor"])
            rotations_1, yaw_after_first = self._route_rotation(first.points, initial_yaw)
            rotations_2, yaw_after_loaded = self._route_rotation(loaded.points, yaw_after_first)
            rotations_3, _ = self._route_rotation(returning.points, yaw_after_loaded, slot["yaw_rad"])
            rotation = rotations_1 + rotations_2 + rotations_3
            travel_s = (
                model.motions[robot].estimated_duration(first.points, initial_yaw)
                + model.motions[robot].estimated_duration(loaded.points, yaw_after_first)
                + model.motions[robot].estimated_duration(returning.points, yaw_after_loaded, slot["yaw_rad"])
            )
            nominal_s = (distance + returning.distance_m) / model.limits.max_linear_m_s
            if self.energy.can_assign(
                robot,
                outbound_m=distance,
                charger_return_m=returning.distance_m,
                expected_wait_s=self.wait_budget_s + max(0.0, travel_s - nominal_s),
                nominal_speed_m_s=model.limits.max_linear_m_s,
                loaded=True,
                expected_rotation_rad=rotation,
                observed_at_s=model.sim_time,
            ):
                return True
        self.denied.add(robot)
        return False

    def _pad_occupied_by_peer(self, model: OperationsModelLike, robot: str, anchor: str) -> str | None:
        x, y = model.book.nodes[anchor]
        for peer, pose in model._poses.items():
            if peer != robot and math.hypot(pose["x_m"] - x, pose["y_m"] - y) <= self.profile["pad_radius_m"]:
                return peer
        return None

    def _prepare_home_swap(self, model: OperationsModelLike, robot: str, anchor: str) -> bool:
        peer = self._pad_occupied_by_peer(model, robot, anchor)
        if peer is None:
            return True
        if model.homes.get(peer) != anchor:
            return False
        old_home = model.homes[robot]
        owned_dock_anchors = {self.slots[dock_id]["anchor"] for dock_id in self.docks.owners}
        if old_home in owned_dock_anchors:
            return False
        if old_home == anchor or old_home in (model.homes[r] for r in model.homes if r not in {robot, peer}):
            raise RuntimeError("charging home swap would duplicate or cross another robot home")
        model.homes[robot], model.homes[peer] = anchor, old_home
        self.relocating[peer] = old_home
        self.home_swaps.append({"charging_robot": robot, "relocating_robot": peer,
                                "dock_home": anchor, "relocation_home": old_home})
        model._event("charge_home_swapped", f"{robot} 충전 진입을 위해 {peer} 주차 위치 교환", robot,
                     dock_anchor=anchor, relocating_robot=peer, relocation_home=old_home)
        return False

    def prepare(self, model: OperationsModelLike) -> None:
        for robot, destination in list(self.relocating.items()):
            if robot not in model._trips and model._locations[robot] == destination:
                del self.relocating[robot]
                model._event("charge_home_relocated", f"{robot} 교환 주차 위치 도착", robot,
                             parking_anchor=destination)
        for robot in model.ledger.robots:
            trip = model._trips.get(robot)
            parking_home = (trip is not None and trip.get("kind") == "parking"
                            and trip.get("purpose", "home") == "home")
            if ((trip is not None and not parking_home)
                    or robot in self.faults or self._loaded(model, robot)):
                continue
            soc = self.energy.snapshot(robot)["battery_pct"] / 100.0
            needs_charge = soc <= self.profile["trigger_soc"] or (robot in self.denied and soc < self.profile["resume_soc"])
            if needs_charge and robot not in self.queue and self.docks.can_dispatch(robot):
                self.queue.append(robot)
                model._event("charge_queued", f"{robot} 운영 가상 충전 대기", robot)

        for robot in list(self.queue):
            # Queue members may use the ordinary reserved parking path. Never
            # replace that accepted route in flight.  A stopped home-parking
            # route may be withdrawn by the operations model at an authored
            # hold while retaining its current reservation footprint.
            if self._loaded(model, robot):
                continue
            free_dock = next((dock for dock in sorted(self.slots) if dock not in self.docks.owners), None)
            if free_dock is None:
                break
            slot = self.slots[free_dock]
            # A home swap relocates the current pad occupant first.  Keep the
            # accepted parking route while that peer is still on the pad;
            # withdrawing and recreating it every tick cannot improve access.
            if (robot in model._trips
                    and self._pad_occupied_by_peer(model, robot, slot["anchor"]) is not None):
                continue
            if robot not in model._trips:
                self.parking_preempt_deferred.discard(robot)
            if robot in model._trips:
                withdraw = getattr(model, "_withdraw_parking_for_charge", None)
                if not callable(withdraw) or not withdraw(robot):
                    if robot not in self.parking_preempt_deferred:
                        self.parking_preempt_deferred.add(robot)
                        model._event(
                            "charge_parking_preempt_deferred",
                            f"{robot} 충전 접근 전환 대기 · 안전 정지점 미확보",
                            robot,
                            reason="parking_in_motion_or_owned_stop",
                        )
                    continue
                self.parking_preempt_deferred.discard(robot)
            if not self._prepare_home_swap(model, robot, slot["anchor"]):
                continue
            # The sorted free dock above must match ChargingFleet's deterministic choice.
            dock_id = self.docks.reserve(robot, carrying_payload=self._loaded(model, robot))
            if dock_id != free_dock:
                raise RuntimeError("charging slot selection diverged from the exclusive dock state")
            self.aux[robot] = {"dock_id": dock_id, "phase": "approach", "stationary_since": None}
            self.queue.remove(robot)
            model._trip(robot, {}, model._locations[robot], slot["anchor"], "charge_approach", kind="charging")
            model._event("charge_reserved", f"{robot} {dock_id} 운영 충전 슬롯 예약", robot,
                         dock_id=dock_id, anchor=slot["anchor"])

    def arrive(self, model: OperationsModelLike, robot: str, trip: Mapping[str, Any]) -> bool:
        if trip.get("kind") != "charging":
            return False
        state = self.aux.get(robot)
        if state is None:
            raise RuntimeError("charging trip arrived without an exclusive slot")
        phase = trip.get("phase")
        if phase == "charge_approach":
            expected = self.slots[state["dock_id"]]["anchor"]
            if trip.get("goal") != expected:
                raise RuntimeError("charging approach arrived at a different anchor")
            model._locations[robot] = expected
            del model._trips[robot]
            state.update(phase="aligning", stationary_since=None)
            model._event("charge_aligning", f"{robot} 충전 패드 정렬", robot, dock_id=state["dock_id"])
            return True
        if phase == "charge_exit":
            expected = self.slots[state["dock_id"]]["exit_anchor"]
            if trip.get("goal") != expected:
                raise RuntimeError("charging exit arrived at a different anchor")
            model._locations[robot] = expected
            del model._trips[robot]
            state.update(phase="exit_dwell", stationary_since=None)
            model._event("charge_exit_arrived", f"{robot} 충전 패드 이탈 확인 대기", robot,
                         dock_id=state["dock_id"], exit_anchor=expected)
            return True
        raise RuntimeError("unknown charging trip phase")

    def pad_contact(self, model: OperationsModelLike, robot: str, dock_id: str) -> bool:
        slot = self.slots[dock_id]
        x, y = model.book.nodes[slot["anchor"]]
        pose = model._poses[robot]
        return (math.hypot(pose["x_m"] - x, pose["y_m"] - y) <= self.profile["pad_radius_m"]
                and abs(_angle(pose["yaw_rad"] - slot["yaw_rad"])) <= self.profile["yaw_tolerance_rad"])

    def _start_exit(self, model: OperationsModelLike, robot: str, state: dict[str, Any]) -> None:
        slot = self.slots[state["dock_id"]]
        if robot in model._trips:
            return
        state.update(phase="exiting", stationary_since=None)
        model._trip(robot, {}, slot["anchor"], slot["exit_anchor"], "charge_exit", kind="charging")
        model._event("charge_exiting", f"{robot} 충전 종료·출구 이동", robot,
                     dock_id=state["dock_id"], exit_anchor=slot["exit_anchor"])

    def _dock_observation(self, model: OperationsModelLike, robot: str, dt: float) -> bool:
        state = self.aux[robot]
        dock_id = state["dock_id"]
        motion = model.motions[robot]
        if state["phase"] == "approach":
            return False
        if state["phase"] == "aligning":
            aligned = model._align_at_hold(robot, self.slots[dock_id]["yaw_rad"], dt)
            if not aligned:
                return False
            self.docks.arrived(robot, run_id=self.run_id, anchor=self.slots[dock_id]["anchor"],
                               navigation_succeeded=True)
            state.update(phase="contact_pending", stationary_since=None)

        contact = self.pad_contact(model, robot, dock_id)
        soc = self.energy.snapshot(robot)["battery_pct"] / 100.0
        dock_state = self.docks.observe(
            robot,
            run_id=self.run_id,
            dock_id=dock_id,
            stamp_s=model.sim_time,
            contact=contact,
            linear_m_s=motion.linear_m_s,
            angular_rad_s=motion.angular_rad_s,
            soc=soc,
        )
        if dock_state["phase"] in {"undock_pending", "ready_to_leave"}:
            if state["phase"] == "exit_dwell":
                stationary = (not contact and motion.stopped
                              and abs(motion.linear_m_s) <= self.profile["energy"]["charging_stopped_linear_m_s"]
                              and abs(motion.angular_rad_s) <= self.profile["energy"]["charging_stopped_angular_rad_s"])
                if not stationary:
                    state["stationary_since"] = None
                elif state["stationary_since"] is None:
                    state["stationary_since"] = model.sim_time
                elif model.sim_time - state["stationary_since"] >= self.profile["exit_dwell_s"]:
                    self.docks.release(robot, run_id=self.run_id, observed_at_s=model.sim_time)
                    del self.aux[robot]
                    self.denied.discard(robot)
                    model._event("charge_released", f"{robot} 충전 슬롯 반납·업무 복귀", robot,
                                 dock_id=dock_id)
                return False
            if state["phase"] not in {"exiting", "exit_dwell"}:
                self._start_exit(model, robot, state)
            return False
        state["phase"] = dock_state["phase"]
        return bool(dock_state["contact_confirmed"] and contact and motion.stopped)

    def record(self, model: OperationsModelLike, dt: float) -> None:
        if (type(dt) not in (float, int) or not math.isfinite(dt) or dt <= 0
                or dt > self.profile["max_step_s"]):
            raise ValueError("charging record interval must be positive and at most max_step_s")
        for robot in model.ledger.robots:
            if robot in self.faults:
                continue
            motion = model.motions[robot]
            confirmed = False
            try:
                if robot in self.aux:
                    confirmed = self._dock_observation(model, robot, float(dt))
                self.energy.observe(
                    robot,
                    model.sim_time,
                    motion.linear_m_s,
                    motion.angular_rad_s,
                    loaded=self._loaded(model, robot),
                    dock_contact_confirmed=confirmed,
                )
            except (KeyError, RuntimeError, ValueError) as error:
                self.faults[robot] = str(error)
                model._event("charge_fault", f"{robot} 운영 충전 오류: {error}", robot)

    def robot_snapshot(self, robot: str) -> dict[str, Any]:
        state = self.aux.get(robot)
        phase = "fault" if robot in self.faults else state["phase"] if state else "queued" if robot in self.queue else None
        energy = self.energy.snapshot(robot)
        return {
            "battery_pct": None if robot in self.faults else energy["battery_pct"],
            "battery_source": "authored_operations_virtual_energy_model",
            "charging_phase": phase,
            "charging_dock": state["dock_id"] if state else None,
            "energy_wh": energy["energy_wh"],
            "consumed_wh": energy["consumed_wh"],
            "charged_wh": energy["charged_wh"],
            "hardware_calibrated": False,
        }

    def snapshot(self) -> dict[str, Any]:
        return {
            "scope": SCOPE,
            "queue": list(self.queue),
            "docks": [
                {
                    "id": dock_id,
                    "anchor": slot["anchor"],
                    "exit_anchor": slot["exit_anchor"],
                    "owner": self.docks.owners.get(dock_id),
                    "phase": (self.aux[self.docks.owners[dock_id]]["phase"]
                              if dock_id in self.docks.owners else "available"),
                }
                for dock_id, slot in self.slots.items()
            ],
            "faults": dict(self.faults),
            "relocating": dict(self.relocating),
            "home_swaps": deepcopy(self.home_swaps),
            "parking_preempt_deferred": sorted(self.parking_preempt_deferred),
            "hardware_calibrated": False,
        }
