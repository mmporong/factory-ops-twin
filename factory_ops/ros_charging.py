"""Virtual geometric docking from caller-supplied ROS evidence; no ROS imports.

Nav2 arrival is a prerequisite, never a contact sensor. G3 retains route and
body-occupancy reservations after the simulated connector is disengaged.
"""
from __future__ import annotations

from copy import deepcopy
import math
import threading
from typing import Any, Mapping
from uuid import UUID

from .charging import ChargingFleet
from .energy_runtime import EnergyRuntime, GEOMETRIC_CONTACT_SOURCE, _si


class RosChargingRuntime:
    """Exclusive owner of one opted-in EnergyRuntime's odometry observations.

    All timestamps use the same absolute ROS simulation seconds. ``observe``
    replaces, rather than supplements, EnergyRuntime.observe in the callback.
    """

    def __init__(self, energy: EnergyRuntime, docks: Mapping[str, Mapping[str, Any]], *,
                 contact_dwell_s: float, position_tolerance_m: float,
                 yaw_tolerance_rad: float, resume_soc: float) -> None:
        self.energy = energy
        self._run = energy.run_id
        self._robots = frozenset(energy.snapshot()["robots"])
        self._docks: dict[str, dict[str, Any]] = {}
        for key, dock in docks.items():
            if (not isinstance(key, str) or not key or not isinstance(dock, Mapping)
                    or not isinstance(dock.get("anchor"), str) or not dock["anchor"]):
                raise ValueError("named docks require an authored routing anchor and map pose")
            self._docks[key] = {"anchor": dock["anchor"],
                **{field: _si(dock[field], "dock " + field) for field in ("x_m", "y_m", "yaw_rad")}}
        self._xy = _si(position_tolerance_m, "position tolerance", nonnegative=True)
        self._yaw = _si(yaw_tolerance_rad, "yaw tolerance", nonnegative=True)
        if self._xy <= 0 or not 0 < self._yaw <= math.pi:
            raise ValueError("positive authored position/yaw bounds required; yaw must not exceed pi")
        limits = energy.charging_limits
        self._gap = limits["max_observation_gap_s"]
        self._fleet = ChargingFleet(self._run, {key: dock["anchor"] for key, dock in self._docks.items()},
            max_gap_s=self._gap, contact_dwell_s=contact_dwell_s,
            stopped_linear_m_s=limits["charging_stopped_linear_m_s"],
            stopped_angular_rad_s=limits["charging_stopped_angular_rad_s"], resume_soc=resume_soc)
        self._reservations: dict[str, dict[str, Any]] = {}
        self._used_goals: set[str] = set()
        self._last: dict[str, tuple[float, float, float, bool]] = {}
        self._lock = threading.RLock()
        energy.claim_virtual_charging_source(self, source=GEOMETRIC_CONTACT_SOURCE)

    def _identity(self, run_id: str, robot_id: str) -> None:
        if run_id != self._run or robot_id not in self._robots:
            raise ValueError("charging evidence belongs to another run or unknown robot")

    def _reservation(self, robot_id: str, dock_id: str) -> dict[str, Any]:
        state = self._fleet.states.get(robot_id)
        if state is None or state.dock_id != dock_id or self._fleet.owners.get(dock_id) != robot_id:
            raise ValueError("charging observation differs from the exclusive reserved dock owner")
        return self._reservations[robot_id]

    def _invalidate(self, robot_id: str, reason: str) -> None:
        state = self._fleet.states.get(robot_id)
        if state is None:
            return
        state.contact_confirmed, state.contact_since_s = False, None
        reservation = self._reservations[robot_id]
        reservation["reason"] = reason
        if state.phase in {"charging", "contact_pending"}:
            state.phase = "contact_pending"
            reservation["virtual_connector_state"] = "not_engaged"
        elif state.phase == "ready_to_leave":
            state.phase = "undock_pending"
            reservation["virtual_connector_state"] = "simulated_disengaging"

    def _dock_view(self, robot_id: str) -> dict[str, Any] | None:
        state = self._fleet.states.get(robot_id)
        if state is None:
            return None
        return {**deepcopy(self._reservations[robot_id]), "dock_id": state.dock_id,
            "phase": state.phase, "geometric_dock_confirmed": state.contact_confirmed,
            "contact_since_s": state.contact_since_s, "last_stamp_s": state.last_stamp_s,
            "fault": state.fault, "contact_source": GEOMETRIC_CONTACT_SOURCE,
            "virtual": True, "hardware_calibrated": False, "physical_contact_verified": False,
            "body_occupancy_owner": "G3 reservation manager"}

    def _view(self, robot_id: str, now: float) -> dict[str, Any]:
        return {"run_id": self._run, "robot_id": robot_id,
                "energy": self.energy.projection(self._run, robot_id, now),
                "charging": self._dock_view(robot_id)}

    def reserve(self, run_id: str, robot_id: str, *, carrying_payload: bool,
                observed_at_s: float) -> dict[str, Any] | None:
        with self._lock:
            self._identity(run_id, robot_id)
            now = _si(observed_at_s, "reservation time", nonnegative=True)
            dock_id = self._fleet.reserve(robot_id, carrying_payload=carrying_payload)
            if dock_id is None:
                return None
            if robot_id not in self._reservations:
                self._reservations[robot_id] = {"anchor": self._docks[dock_id]["anchor"],
                    "reserved_at_s": now, "arrival": None, "virtual_connector_state": "not_engaged",
                    "reason": "awaiting_real_nav2_arrival"}
            return self._dock_view(robot_id)

    def arrived(self, run_id: str, robot_id: str, *, dock_id: str, anchor: str,
                goal_id: str, navigation_succeeded: bool, stamp_s: float,
                observed_at_s: float) -> dict[str, Any] | None:
        with self._lock:
            self._identity(run_id, robot_id)
            reservation = self._reservation(robot_id, dock_id)
            stamp = _si(stamp_s, "arrival time", nonnegative=True)
            now = _si(observed_at_s, "arrival observation time", nonnegative=True)
            try:
                if not isinstance(goal_id, str) or UUID(goal_id).int == 0:
                    raise ValueError("positive Nav2 goal UUID required")
                goal = UUID(goal_id).hex
            except (ValueError, AttributeError) as error:
                raise ValueError("positive Nav2 goal UUID required") from error
            if (goal in self._used_goals or stamp <= 0 or stamp < reservation["reserved_at_s"]
                    or not 0 <= now - stamp <= self._gap):
                raise ValueError("arrival must be fresh, after reservation, with a new successful goal")
            self._fleet.arrived(robot_id, run_id=run_id, anchor=anchor,
                                navigation_succeeded=navigation_succeeded)
            self._used_goals.add(goal)
            reservation.update(arrival={"goal_id": goal, "stamp_s": stamp, "anchor": anchor,
                "navigation_succeeded": True}, reason="awaiting_fresh_geometric_dwell")
            return self._dock_view(robot_id)

    def _geometry(self, run_id: str, robot_id: str, dock_id: str,
                  pose: Mapping[str, Any] | None, stamp: float, now: float) -> bool:
        if pose is None:
            return False
        if (not isinstance(pose, Mapping) or pose.get("run_id") != run_id
                or pose.get("robot_id") != robot_id or pose.get("frame_id") != "map"):
            raise ValueError("estimated map pose must have the same run and robot identity")
        try:
            x, y, yaw = (_si(pose[key], "estimated pose " + key) for key in ("x_m", "y_m", "yaw_rad"))
            pose_stamp = _si(pose["stamp_s"], "pose stamp", nonnegative=True)
        except KeyError as error:
            raise ValueError("estimated pose requires map x/y/yaw and its actual stamp") from error
        if not 0 <= now - pose_stamp <= self._gap or pose_stamp > stamp:
            raise ValueError("estimated pose is stale or in the future of odometry/ROS clock")
        dock = self._docks[dock_id]
        return (math.hypot(x - dock["x_m"], y - dock["y_m"]) <= self._xy
                and abs(math.atan2(math.sin(yaw - dock["yaw_rad"]), math.cos(yaw - dock["yaw_rad"]))) <= self._yaw)

    def observe(self, run_id: str, robot_id: str, stamp_s: float, linear_m_s: float,
                angular_rad_s: float, *, loaded: bool, observed_at_s: float,
                dock_id: str | None = None, pose: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """One signed encoder callback; duplicates cannot advance dwell or SOC.

        Pose keys: run_id, robot_id, frame_id='map', x_m, y_m, yaw_rad,
        stamp_s. Missing geometry cannot engage the virtual connector.
        """
        with self._lock:
            self._identity(run_id, robot_id)
            state = self._fleet.states.get(robot_id)
            if state is not None:
                self._reservation(robot_id, dock_id)
            elif dock_id is not None or pose is not None:
                raise ValueError("unreserved robot cannot submit docking evidence")
            try:
                stamp = _si(stamp_s, "odometry stamp", nonnegative=True)
                now = _si(observed_at_s, "ROS observation time", nonnegative=True)
                linear, angular = _si(linear_m_s, "signed linear velocity"), _si(angular_rad_s, "signed angular velocity")
                if type(loaded) is not bool or not 0 <= now - stamp <= self._gap:
                    raise ValueError("fresh odometry and explicit operational load required")
                geometry = self._geometry(run_id, robot_id, dock_id, pose, stamp, now) if state else False
                sample = (stamp, linear, angular, loaded)
                last = self._last.get(robot_id)
                if last is not None and stamp <= last[0]:
                    if sample != last:
                        raise ValueError("odometry is backward or conflicts at the same timestamp")
                    if state and (not geometry or loaded or abs(linear) > self._fleet.stopped_linear_m_s
                                  or abs(angular) > self._fleet.stopped_angular_rad_s):
                        self._invalidate(robot_id, "duplicate_has_no_valid_geometric_dwell")
                    return self._view(robot_id, now)
                confirmed = False
                if state and state.phase != "approach":
                    if state.phase == "fault":
                        raise ValueError("docking lifecycle is faulted")
                    projection = self.energy.projection(run_id, robot_id, stamp)
                    # A missing seed observation cannot fund charging. Establish a baseline first.
                    soc = projection["battery_pct"]
                    arrival = self._reservations[robot_id]["arrival"]
                    contact = geometry and not loaded and soc is not None and stamp >= arrival["stamp_s"]
                    if state.phase in {"undock_pending", "ready_to_leave"}:
                        contact = False  # Explicitly simulated connector opening; no physical signal.
                    self._fleet.observe(robot_id, run_id=run_id, dock_id=dock_id, stamp_s=stamp,
                        contact=contact, linear_m_s=linear, angular_rad_s=angular,
                        soc=0. if soc is None else soc / 100.)
                    confirmed = state.contact_confirmed
                energy = self.energy.observe_virtual_charging(self, run_id, robot_id, stamp,
                    linear, angular, loaded=loaded, geometric_dock_confirmed=confirmed)
                self._last[robot_id] = sample
                if state:
                    reservation = self._reservations[robot_id]
                    if state.phase == "charging" and energy["battery_pct"] >= self._fleet.resume_soc * 100:
                        state.phase, state.contact_confirmed, state.contact_since_s = "undock_pending", False, None
                    connector = {"charging": "simulated_engaged", "undock_pending": "simulated_disengaging",
                                 "ready_to_leave": "simulated_disengaged"}.get(state.phase, "not_engaged")
                    reservation.update(virtual_connector_state=connector, reason=state.phase)
                return self._view(robot_id, now)
            except (ValueError, TypeError, OverflowError) as error:
                self._invalidate(robot_id, str(error))
                raise ValueError(f"virtual geometric charging observation rejected: {error}") from error

    def snapshot(self, run_id: str, *, observed_at_s: float) -> dict[str, Any]:
        with self._lock:
            if run_id != self._run:
                raise ValueError("charging snapshot belongs to another run")
            now = _si(observed_at_s, "snapshot time", nonnegative=True)
            return {"run_id": self._run, "robots": {robot: self._view(robot, now) for robot in sorted(self._robots)},
                "owners": dict(self._fleet.owners), "docks": deepcopy(self._docks),
                "contact_source": GEOMETRIC_CONTACT_SOURCE, "virtual": True, "hardware_calibrated": False,
                "physical_contact_verified": False, "physical_electrical_charging_verified": False,
                "body_occupancy_owner": "G3 reservation manager"}

    def release(self, run_id: str, robot_id: str, *, dock_id: str,
                observed_at_s: float) -> dict[str, Any]:
        with self._lock:
            self._identity(run_id, robot_id)
            reservation = self._reservation(robot_id, dock_id)
            now = _si(observed_at_s, "release time", nonnegative=True)
            if (reservation["virtual_connector_state"] != "simulated_disengaged"
                    or self.energy.projection(run_id, robot_id, now)["freshness"] != "fresh"):
                raise ValueError("release requires fresh odometry and simulated connector disengagement")
            self._fleet.release(robot_id, run_id=run_id, observed_at_s=now)
            del self._reservations[robot_id]
            return {"run_id": run_id, "robot_id": robot_id, "dock_id": dock_id,
                "virtual_connector_state": "simulated_disengaged", "charging_reservation_released": True,
                "body_occupancy_released": False, "contact_source": GEOMETRIC_CONTACT_SOURCE,
                "virtual": True, "hardware_calibrated": False}
