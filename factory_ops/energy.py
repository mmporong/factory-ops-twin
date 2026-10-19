"""Virtual battery accounting from observed wheel odometry, in SI units.

The coefficients describe an authored virtual battery, not hardware calibration.
The runtime adapter supplies odometry and a separately observed dock contact.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Any, Mapping


def number(value: Any, name: str, *, positive: bool = False) -> float:
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or positive and value == 0:
        raise ValueError(f"{name} requires a finite nonnegative SI value")
    return float(value)


@dataclass
class Battery:
    energy_wh: float
    consumed_wh: float = 0.0
    charged_wh: float = 0.0
    last_sample: tuple[float, float, float, bool, bool] | None = None


class EnergyFleet:
    """Integrate observed time and motion; stale gaps never become fabricated SOC."""

    def __init__(self, profile: Mapping[str, Any], initial_soc: Mapping[str, float]) -> None:
        self.profile = deepcopy(dict(profile))
        for key in ("capacity_wh", "idle_power_w", "translation_wh_per_m", "rotation_wh_per_rad",
                    "loaded_factor", "charger_power_w", "charger_efficiency", "max_observation_gap_s",
                    "charging_stopped_linear_m_s", "charging_stopped_angular_rad_s"):
            number(self.profile[key], key, positive=True)
        if (self.profile["loaded_factor"] < 1 or self.profile["charger_efficiency"] > 1
                or not 0 < number(self.profile["reserve_fraction"], "reserve_fraction") < 1
                or self.profile.get("provenance", {}).get("kind") != "authored_virtual_model"):
            raise ValueError("energy profile needs authored provenance and valid factors")
        if not initial_soc or any(not isinstance(robot, str) or not robot for robot in initial_soc):
            raise ValueError("energy fleet requires named robots")
        self.batteries: dict[str, Battery] = {}
        for robot, soc in initial_soc.items():
            if number(soc, "initial SOC") > 1:
                raise ValueError("SOC is a fraction between zero and one")
            self.batteries[robot] = Battery(self.profile["capacity_wh"] * soc)

    def observe(self, robot: str, stamp_s: float, linear_m_s: float, angular_rad_s: float,
                *, loaded: bool, dock_contact_confirmed: bool = False) -> dict[str, Any]:
        stamp = number(stamp_s, "sample stamp")
        for value in (linear_m_s, angular_rad_s):
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError("observed signed velocities must be finite numbers")
        speed, turn = abs(float(linear_m_s)), abs(float(angular_rad_s))
        if type(loaded) is not bool or type(dock_contact_confirmed) is not bool:
            raise ValueError("load and dock confirmations must be boolean observations")
        battery = self.batteries[robot]
        if dock_contact_confirmed and (speed > self.profile["charging_stopped_linear_m_s"]
                                      or turn > self.profile["charging_stopped_angular_rad_s"]):
            raise ValueError("charging requires confirmed contact and observed stationary bounds")
        sample = (stamp, float(linear_m_s), float(angular_rad_s), loaded, dock_contact_confirmed)
        if battery.last_sample is not None:
            dt = stamp - battery.last_sample[0]
            if sample == battery.last_sample:
                return self.snapshot(robot)
            if dt <= 0 or dt > self.profile["max_observation_gap_s"]:
                raise ValueError("energy observation is backward, conflicting or stale")
            previous = battery.last_sample
            distance = (abs(previous[1]) + speed) * dt / 2
            rotation = (abs(previous[2]) + turn) * dt / 2
            consumed = (self.profile["idle_power_w"] * dt / 3600
                + self.profile["translation_wh_per_m"] * distance * (self.profile["loaded_factor"] if loaded or previous[3] else 1)
                + self.profile["rotation_wh_per_rad"] * rotation)
            # Credit only intervals whose two endpoints confirm stationary contact.
            charging = dock_contact_confirmed and previous[4]
            supplied = self.profile["charger_power_w"] * self.profile["charger_efficiency"] * dt / 3600 if charging else 0.0
            available = battery.energy_wh + supplied
            if consumed > available:
                raise ValueError("virtual battery exhausted; observed motion cannot be funded")
            charged = min(supplied, self.profile["capacity_wh"] - battery.energy_wh + consumed)
            battery.energy_wh += charged - consumed
            battery.consumed_wh += consumed
            battery.charged_wh += charged
        battery.last_sample = sample
        return self.snapshot(robot)

    def can_assign(self, robot: str, *, outbound_m: float, charger_return_m: float,
                   expected_wait_s: float, nominal_speed_m_s: float, loaded: bool,
                   expected_rotation_rad: float = 0.0, observed_at_s: float) -> bool:
        if type(loaded) is not bool:
            raise ValueError("load state must be a boolean")
        distance = number(outbound_m, "outbound distance") + number(charger_return_m, "charger return distance")
        speed = number(nominal_speed_m_s, "planning speed", positive=True)
        duration = distance / speed + number(expected_wait_s, "expected wait")
        required = (self.profile["translation_wh_per_m"] * distance * (self.profile["loaded_factor"] if loaded else 1)
                    + self.profile["idle_power_w"] * duration / 3600
                    + self.profile["rotation_wh_per_rad"] * number(expected_rotation_rad, "route rotation")
                    + self.profile["capacity_wh"] * self.profile["reserve_fraction"])
        battery = self.batteries[robot]
        stamp = number(observed_at_s, "decision stamp")
        if battery.last_sample is None or not 0 <= stamp - battery.last_sample[0] <= self.profile["max_observation_gap_s"]:
            return False
        return battery.energy_wh >= required

    def snapshot(self, robot: str) -> dict[str, Any]:
        battery = self.batteries[robot]
        return {"robot_id": robot, "battery_pct": 100 * battery.energy_wh / self.profile["capacity_wh"],
                "battery_source": "authored_virtual_energy_model", "energy_wh": battery.energy_wh,
                "consumed_wh": battery.consumed_wh, "charged_wh": battery.charged_wh,
                "observed_sim_time_s": battery.last_sample[0] if battery.last_sample else None,
                "hardware_calibrated": False}
