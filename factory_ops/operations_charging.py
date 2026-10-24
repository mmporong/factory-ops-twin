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

    def _charging_approach_option(self, robot_id: str, goal: str) -> dict[str, Any] | None: ...

    def _reassign_charging_approach(self, robot_id: str, goal: str) -> None: ...

    def _charging_departure_requirement(self, robot_id: str) -> dict[str, Any] | None: ...


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
        self.approach_deferred: dict[str, dict[str, Any]] = {}
        self.departure_deferred: dict[str, dict[str, Any]] = {}
        self.traffic_wait_budget_s = 0.0
        self._assignment_requirement_cache: dict[tuple[Any, ...], float] = {}
        waits = getattr(model, "_transport_wait_s", {})
        deliveries = getattr(model, "_deliveries", {})
        self._traffic_delivery_samples = {
            robot: (int(deliveries.get(robot, 0)), float(waits.get(robot, 0.0)))
            for robot in robots
        }
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

    def _assignment_required_wh(
        self, model: OperationsModelLike, robot: str, pickup: str, destination: str,
    ) -> float:
        pose = model._poses[robot]
        dispatch_origin = getattr(model, "_dispatch_origin", None)
        origin = dispatch_origin(robot) if callable(dispatch_origin) else model._locations[robot]
        cache_key = (
            robot,
            origin,
            round(pose["x_m"], 12),
            round(pose["y_m"], 12),
            round(pose["yaw_rad"], 12),
            model.closed,
            pickup,
            destination,
            round(self.wait_budget_s, 9),
        )
        cached = self._assignment_requirement_cache.get(cache_key)
        if cached is not None:
            return cached + self.energy.profile["idle_power_w"] * self.traffic_wait_budget_s / 3600.0
        first = self._plan(model, origin, pickup)
        loaded = self._plan(model, pickup, destination)
        distance = first.distance_m + loaded.distance_m
        initial_yaw = pose["yaw_rad"]
        requirements = []
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
            requirements.append(self.energy.assignment_required_wh(
                outbound_m=distance,
                charger_return_m=returning.distance_m,
                expected_wait_s=self.wait_budget_s + max(0.0, travel_s - nominal_s),
                nominal_speed_m_s=model.limits.max_linear_m_s,
                loaded=True,
                expected_rotation_rad=rotation,
            ))
        required_wh = min(requirements)
        self._assignment_requirement_cache[cache_key] = required_wh
        return required_wh + self.energy.profile["idle_power_w"] * self.traffic_wait_budget_s / 3600.0

    def can_assign(self, model: OperationsModelLike, robot: str, pickup: str, destination: str) -> bool:
        if not self.available(robot):
            return False
        required = self._assignment_required_wh(model, robot, pickup, destination)
        energy = self.energy.snapshot(robot)
        observed_at_s = energy["observed_sim_time_s"]
        if (observed_at_s is not None
                and 0 <= model.sim_time - observed_at_s <= self.profile["energy"]["max_observation_gap_s"]
                and energy["energy_wh"] >= required):
            return True
        self.denied.add(robot)
        return False

    def _update_traffic_wait_budget(self, model: OperationsModelLike) -> None:
        """Retain the largest completed or currently accumulating transport wait."""

        waits = getattr(model, "_transport_wait_s", None)
        deliveries = getattr(model, "_deliveries", None)
        if not isinstance(waits, Mapping) or not isinstance(deliveries, Mapping):
            return
        for robot, (prior_count, prior_wait) in self._traffic_delivery_samples.items():
            count = int(deliveries.get(robot, 0))
            wait = float(waits.get(robot, 0.0))
            if count < prior_count or wait < prior_wait:
                raise RuntimeError("operations traffic counters moved backward during charging observation")
            ongoing = wait - prior_wait
            if count == prior_count:
                self.traffic_wait_budget_s = max(self.traffic_wait_budget_s, ongoing)
                continue
            sample = ongoing / (count - prior_count)
            self.traffic_wait_budget_s = max(self.traffic_wait_budget_s, sample)
            self._traffic_delivery_samples[robot] = (count, wait)

    def _pad_occupied_by_peer(self, model: OperationsModelLike, robot: str, anchor: str) -> str | None:
        x, y = model.book.nodes[anchor]
        for peer, pose in model._poses.items():
            if peer != robot and math.hypot(pose["x_m"] - x, pose["y_m"] - y) <= self.profile["pad_radius_m"]:
                return peer
        return None

    def _prepare_home_swap(self, model: OperationsModelLike, robot: str, anchor: str) -> bool:
        occupied_peer = self._pad_occupied_by_peer(model, robot, anchor)
        home_peer = next(
            (peer for peer, home in model.homes.items() if peer != robot and home == anchor),
            None,
        )
        peer = occupied_peer if occupied_peer is not None else home_peer
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
        return occupied_peer is None

    def _approach_candidates(
        self, model: OperationsModelLike, robot: str, docks: list[str],
    ) -> tuple[list[dict[str, Any]], dict[str, list[str]]]:
        inspect = getattr(model, "_charging_approach_option", None)
        candidates: list[dict[str, Any]] = []
        blocked: dict[str, list[str]] = {}
        for dock_id in docks:
            slot = self.slots[dock_id]
            if callable(inspect):
                option = inspect(robot, slot["anchor"])
                if option is None:
                    blocked[dock_id] = ["unsafe_or_moving_hold"]
                    continue
                blockers = list(option["blockers"])
                if blockers:
                    blocked[dock_id] = blockers
                    continue
                distance_m = float(option["distance_m"])
            else:
                distance_m = float(self._plan(model, model._locations[robot], slot["anchor"]).distance_m)
            candidates.append({"dock_id": dock_id, "distance_m": distance_m})
        # Keep the two-dock legacy adapter deterministic. Multi-bay campuses
        # rank only startable candidates by the authored route distance.
        if len(self.slots) > 2:
            candidates.sort(key=lambda row: (row["distance_m"], row["dock_id"]))
        else:
            candidates.sort(key=lambda row: row["dock_id"])
        return candidates, blocked

    def _record_approach_deferred(
        self, model: OperationsModelLike, robot: str, *, dock_id: str | None,
        blockers: Mapping[str, list[str]], reason: str,
    ) -> None:
        state = {"dock_id": dock_id, "reason": reason, "blockers": deepcopy(dict(blockers))}
        if self.approach_deferred.get(robot) == state:
            return
        self.approach_deferred[robot] = state
        model._event(
            "charge_approach_deferred",
            f"{robot} 충전 접근 대기 · 안전한 도크 경로 없음",
            robot,
            dock_id=dock_id,
            reason=reason,
            blockers=deepcopy(dict(blockers)),
        )

    def _reassign_blocked_approaches(self, model: OperationsModelLike) -> None:
        inspect = getattr(model, "_charging_approach_option", None)
        reassign = getattr(model, "_reassign_charging_approach", None)
        if not callable(inspect) or not callable(reassign):
            return
        for robot, state in list(self.aux.items()):
            if state["phase"] != "approach":
                continue
            current_dock = state["dock_id"]
            current = inspect(robot, self.slots[current_dock]["anchor"])
            if current is None or not current["blockers"]:
                self.approach_deferred.pop(robot, None)
                continue
            free = [dock for dock in sorted(self.slots) if dock not in self.docks.owners]
            candidates, blocked = self._approach_candidates(model, robot, free)
            if not candidates:
                self._record_approach_deferred(
                    model,
                    robot,
                    dock_id=current_dock,
                    blockers={current_dock: list(current["blockers"]), **blocked},
                    reason="reserved_approach_and_free_alternatives_blocked",
                )
                continue
            alternative = candidates[0]
            new_dock = alternative["dock_id"]
            if not self._prepare_home_swap(model, robot, self.slots[new_dock]["anchor"]):
                continue
            old_dock = self.docks.cancel_approach(robot)
            reserved = self.docks.reserve(
                robot, carrying_payload=False, preferred_dock=new_dock,
            )
            if reserved != new_dock:
                raise RuntimeError("charging approach reassignment lost its selected free dock")
            state["dock_id"] = new_dock
            reassign(robot, self.slots[new_dock]["anchor"])
            self.approach_deferred.pop(robot, None)
            model._event(
                "charge_reassigned",
                f"{robot} 정지 충전 접근 재배정 · {old_dock}→{new_dock}",
                robot,
                old_dock_id=old_dock,
                dock_id=new_dock,
                prior_blockers=list(current["blockers"]),
                distance_m=alternative["distance_m"],
            )

    def prepare(self, model: OperationsModelLike) -> None:
        self._update_traffic_wait_budget(model)
        for robot, destination in list(self.relocating.items()):
            retarget = getattr(model, "_retarget_relocated_home_parking", None)
            if callable(retarget):
                retarget(robot, destination)
            if robot not in model._trips and model._locations[robot] == destination:
                del self.relocating[robot]
                model._event("charge_home_relocated", f"{robot} 교환 주차 위치 도착", robot,
                             parking_anchor=destination)
        for robot in model.ledger.robots:
            trip = model._trips.get(robot)
            parking_home = (trip is not None and trip.get("kind") == "parking"
                            and trip.get("purpose", "home") == "home")
            pickup_preemptible = (
                trip is not None
                and callable(getattr(model, "_can_withdraw_pickup_for_charge", None))
                and model._can_withdraw_pickup_for_charge(robot)
            )
            if ((trip is not None and not parking_home and not pickup_preemptible)
                    or robot in self.faults or self._loaded(model, robot)):
                continue
            soc = self.energy.snapshot(robot)["battery_pct"] / 100.0
            needs_charge = soc <= self.profile["trigger_soc"] or robot in self.denied
            if needs_charge and robot not in self.queue and self.docks.can_dispatch(robot):
                self.queue.append(robot)
                model._event("charge_queued", f"{robot} 운영 가상 충전 대기", robot)

        self._reassign_blocked_approaches(model)

        for robot in list(self.queue):
            # Queue members may use the ordinary reserved parking path. Never
            # replace that accepted route in flight.  A stopped home-parking
            # route may be withdrawn by the operations model at an authored
            # hold while retaining its current reservation footprint.
            if self._loaded(model, robot):
                continue
            free = [dock for dock in sorted(self.slots) if dock not in self.docks.owners]
            if not free:
                break
            candidates, blocked = self._approach_candidates(model, robot, free)
            if not candidates:
                unsafe_hold = blocked and all(
                    values == ["unsafe_or_moving_hold"] for values in blocked.values()
                )
                if robot in model._trips and unsafe_hold:
                    if robot not in self.parking_preempt_deferred:
                        self.parking_preempt_deferred.add(robot)
                        model._event(
                            "charge_parking_preempt_deferred",
                            f"{robot} 충전 접근 전환 대기 · 안전 정지점 미확보",
                            robot,
                            reason="parking_in_motion_or_owned_stop",
                        )
                    self.approach_deferred.pop(robot, None)
                    continue
                self._record_approach_deferred(
                    model,
                    robot,
                    dock_id=None,
                    blockers=blocked,
                    reason="free_approaches_blocked_or_not_at_safe_hold",
                )
                continue
            free_dock = candidates[0]["dock_id"]
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
                withdraw = getattr(model, "_withdraw_trip_for_charge", None)
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
            # The explicit choice must match ChargingFleet's exclusive owner state.
            dock_id = self.docks.reserve(
                robot,
                carrying_payload=self._loaded(model, robot),
                preferred_dock=free_dock,
            )
            if dock_id != free_dock:
                raise RuntimeError("charging slot selection diverged from the exclusive dock state")
            self.aux[robot] = {"dock_id": dock_id, "phase": "approach", "stationary_since": None}
            self.queue.remove(robot)
            self.approach_deferred.pop(robot, None)
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
            self.approach_deferred.pop(robot, None)
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
        departure_soc = self.profile["resume_soc"]
        departure_allowed = True
        requirement = None
        requirement_hook = getattr(model, "_charging_departure_requirement", None)
        if callable(requirement_hook):
            requirement = requirement_hook(robot)
        if requirement is not None:
            required_wh = float(requirement["required_wh"])
            capacity_wh = float(self.profile["energy"]["capacity_wh"])
            hysteresis_soc = self.profile["resume_soc"] - self.profile["trigger_soc"]
            departure_soc = max(
                departure_soc,
                min(1.0, required_wh / capacity_wh + hysteresis_soc),
            )
            departure_allowed = required_wh <= capacity_wh
            deferred = {
                "reason": ("observed_traffic_funding" if departure_allowed
                           else "assignment_funding_exceeds_capacity"),
                "required_wh": required_wh,
                "departure_soc": departure_soc,
                "task_id": requirement["task_id"],
                "traffic_wait_budget_s": self.traffic_wait_budget_s,
            }
            if (not departure_allowed or soc < departure_soc) and self.departure_deferred.get(robot) != deferred:
                self.departure_deferred[robot] = deferred
                model._event(
                    "charge_departure_deferred",
                    f"{robot} 충전 접촉 유지 · 다음 운송 에너지 확보 대기",
                    robot,
                    **deferred,
                )
            elif departure_allowed and soc >= departure_soc:
                self.departure_deferred.pop(robot, None)
        else:
            self.departure_deferred.pop(robot, None)
        dock_state = self.docks.observe(
            robot,
            run_id=self.run_id,
            dock_id=dock_id,
            stamp_s=model.sim_time,
            contact=contact,
            linear_m_s=motion.linear_m_s,
            angular_rad_s=motion.angular_rad_s,
            soc=soc,
            departure_soc=departure_soc,
            departure_allowed=departure_allowed,
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
                    self.departure_deferred.pop(robot, None)
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
            "approach_deferred": deepcopy(self.approach_deferred),
            "departure_deferred": deepcopy(self.departure_deferred),
            "authored_assignment_wait_budget_s": self.wait_budget_s,
            "observed_traffic_wait_budget_s": self.traffic_wait_budget_s,
            "hardware_calibrated": False,
        }
