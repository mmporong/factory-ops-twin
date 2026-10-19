"""Run-bound virtual energy from odometry; no ROS or charging contact source."""
from __future__ import annotations

from copy import deepcopy
import math
import threading
from typing import Any, Mapping

from .energy import EnergyFleet


SCOPE = "odometry-driven authored virtual energy; no hardware battery or charging proof"


def _si(value: float, label: str, *, nonnegative: bool = False) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be finite numeric SI data")
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError(f"{label} exceeds finite SI range") from error
    if not math.isfinite(result) or nonnegative and result < 0:
        raise ValueError(f"{label} must be finite{' nonnegative' if nonnegative else ''} SI data")
    return result


class EnergyRuntime:
    """Latch per-robot faults without integrating unobserved time or resetting SOC.

    ``observe`` raises ValueError for invalid evidence and for a latched fault.
    Identity errors and projection queries never change observation state.
    ``snapshot`` exposes model balances, not a freshness-qualified SOC; callers
    displaying SOC must use ``projection`` with their current simulation clock.
    """

    def __init__(self, run_id: str, profile: Mapping[str, Any], initial_soc: Mapping[str, float]) -> None:
        if not isinstance(run_id, str) or not run_id or run_id.strip() != run_id:
            raise ValueError("a nonempty run identity without surrounding whitespace is required")
        self._run_id = run_id
        self._fleet = EnergyFleet(profile, initial_soc)
        self._faults: dict[str, dict[str, str]] = {}
        self._lock = threading.RLock()

    @property
    def run_id(self) -> str:
        return self._run_id

    def _identity(self, run_id: str, robot_id: str) -> None:
        if run_id != self._run_id:
            raise ValueError("energy observation/query belongs to another run")
        if not isinstance(robot_id, str) or robot_id not in self._fleet.batteries:
            raise ValueError("unknown energy robot")

    def observe(self, run_id: str, robot_id: str, stamp_s: float, linear_m_s: float,
                angular_rad_s: float, *, loaded: bool) -> dict[str, Any]:
        """Integrate one callback sample; exact repeats do not debit energy twice."""
        with self._lock:
            self._identity(run_id, robot_id)
            if robot_id in self._faults:
                raise ValueError(f"energy robot is faulted: {self._faults[robot_id]['reason']}")
            try:
                stamp = _si(stamp_s, "sample stamp", nonnegative=True)
                linear = _si(linear_m_s, "linear velocity")
                angular = _si(angular_rad_s, "angular velocity")
                if type(loaded) is not bool:
                    raise ValueError("loaded must be an explicit boolean operational input")
                # No contact source is owned here: never credit charger power.
                self._fleet.observe(robot_id, stamp, linear, angular, loaded=loaded)
            except (ValueError, TypeError, OverflowError) as error:
                self._faults[robot_id] = {"code": "invalid_observation", "reason": str(error)}
                raise ValueError(f"energy observation fault for {robot_id}: {error}") from error
            return self.projection(run_id, robot_id, stamp)

    def projection(self, run_id: str, robot_id: str, observed_at_s: float) -> dict[str, Any]:
        """Qualify the last virtual SOC against the caller's simulation time."""
        with self._lock:
            self._identity(run_id, robot_id)
            now = _si(observed_at_s, "projection stamp", nonnegative=True)
            state = self._fleet.snapshot(robot_id)
            stamp = state["observed_sim_time_s"]
            age = None if stamp is None else now - stamp
            fault = self._faults.get(robot_id)
            if fault is not None:
                freshness = "fault"
            elif age is None:
                freshness = "missing"
            elif age < 0:
                freshness = "future"
            elif age > self._fleet.profile["max_observation_gap_s"]:
                freshness = "stale"
            else:
                freshness = "fresh"
            # Ratio first also avoids overflowing 100 * a large finite balance.
            percent = 100 * (state["energy_wh"] / self._fleet.profile["capacity_wh"])
            return {"run_id": self._run_id, "robot_id": robot_id,
                    "battery_pct": percent if freshness == "fresh" else None,
                    "battery_source": state["battery_source"], "observed_sim_time_s": stamp,
                    "observed_at_s": now, "observation_age_s": age, "freshness": freshness,
                    "virtual": True, "hardware_calibrated": False, "charging_supported": False,
                    "fault": deepcopy(fault), "scope": SCOPE}

    def can_assign(self, run_id: str, robot_id: str, *, outbound_m: float,
                   charger_return_m: float, expected_wait_s: float, nominal_speed_m_s: float,
                   loaded: bool, expected_rotation_rad: float, observed_at_s: float) -> bool:
        """Read-only full-trip funding gate; queries never latch observation faults."""
        with self._lock:
            self._identity(run_id, robot_id)
            inputs = {"outbound_m": _si(outbound_m, "outbound distance", nonnegative=True),
                      "charger_return_m": _si(charger_return_m, "charger return distance", nonnegative=True),
                      "expected_wait_s": _si(expected_wait_s, "expected wait", nonnegative=True),
                      "nominal_speed_m_s": _si(nominal_speed_m_s, "planning speed", nonnegative=True),
                      "expected_rotation_rad": _si(expected_rotation_rad, "route rotation", nonnegative=True),
                      "observed_at_s": _si(observed_at_s, "decision stamp", nonnegative=True)}
            if inputs["nominal_speed_m_s"] == 0 or type(loaded) is not bool:
                raise ValueError("planning speed must be positive and load must be boolean")
            if self.projection(run_id, robot_id, inputs["observed_at_s"])["freshness"] != "fresh":
                return False
            return self._fleet.can_assign(robot_id, loaded=loaded, **inputs)

    def snapshot(self) -> dict[str, Any]:
        """Return detached model balances and first faults, with no sample history."""
        with self._lock:
            robots = {}
            for robot_id in self._fleet.batteries:
                state = self._fleet.snapshot(robot_id)
                state.pop("battery_pct")
                robots[robot_id] = {**state, "virtual": True, "charging_supported": False,
                                    "fault": deepcopy(self._faults.get(robot_id))}
            return {"run_id": self._run_id, "robots": robots,
                    "provenance": deepcopy(self._fleet.profile["provenance"]),
                    "observation_source": "caller-supplied signed wheel odometry; operational load input",
                    "contact_source": "unavailable", "charging_supported": False, "scope": SCOPE}
