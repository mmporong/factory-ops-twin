"""Opt-in encoder target hold; imports neither SDK nor NumPy at module load."""
from __future__ import annotations

import math
from numbers import Integral, Real
from typing import Any


# Authored response, not a measured settling time. Kp = Kd / response_time_s.
RESPONSE_TIME_S = 0.2


def _values(values, count: int, label: str) -> list[float]:
    try:
        result = list(values)
        if len(result) != count or any(isinstance(v, bool) or not isinstance(v, Real) for v in result):
            raise ValueError
        result = [float(v) for v in result]
        if not all(math.isfinite(v) for v in result):
            raise ValueError
        return result
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must contain {count} finite real values") from error


def _array(values, *, integer=False):
    # Called only on the enabled, initialized native control path.
    import numpy as np

    return np.asarray(values, dtype=np.int32 if integer else np.float32)


class WheelParkingHold:
    def __init__(self, native: Any, wheel_names, action_factory, *, array_factory=None):
        self.native, self.action_factory = native, action_factory
        self.array = array_factory or _array
        self.names = tuple(wheel_names)
        dofs = tuple(native.dof_names)
        if len(self.names) != 2 or len(set(self.names)) != 2 or any(name not in dofs for name in self.names):
            raise ValueError("parking hold requires two distinct named wheel DOFs")
        indices = tuple(native.get_dof_index(name) for name in self.names)
        native_indices = tuple(native.wheel_dof_indices)
        if (any(isinstance(i, bool) or not isinstance(i, Integral) or not 0 <= i < len(dofs) for i in indices)
                or any(isinstance(i, bool) or not isinstance(i, Integral) for i in native_indices)
                or len(set(indices)) != 2 or native_indices != indices
                or any(dofs[i] != name for i, name in zip(indices, self.names))):
            raise ValueError("native wheel indices differ from configured wheel names")
        self.indices, self.count = tuple(int(i) for i in indices), len(dofs)
        self.controller = native.get_articulation_controller()
        kps, kds = self._gains()
        efforts, modes = self._limits()
        self.original = {"kps": self._pair(kps), "kds": self._pair(kds),
                         "max_efforts": efforts, "effort_modes": modes}
        if (any(v != 0 for v in self.original["kps"])
                or any(v <= 0 for v in self.original["kds"] + efforts)):
            raise ValueError("parking hold requires zero velocity Kp and positive damping/effort limits")
        self.hold_kps = _values([v / RESPONSE_TIME_S for v in self.original["kds"]], 2, "hold Kp")
        if any(v <= 0 for v in self.hold_kps):
            raise ValueError("derived hold Kp must be positive")
        self.anchor: list[float] | None = None
        self.hold_entries = self.hold_releases = self.hold_actions = self.velocity_actions = 0
        self.failure: str | None = None

    def _pair(self, values):
        return [values[i] for i in self.indices]

    def _gains(self):
        kps, kds = self.controller.get_gains()
        return _values(kps, self.count, "joint Kp"), _values(kds, self.count, "joint Kd")

    def _limits(self):
        efforts, modes = list(self.controller.get_max_efforts()), list(self.controller.get_effort_modes())
        if len(efforts) != self.count or len(modes) != self.count:
            raise ValueError("native effort/mode readback shape differs from DOFs")
        selected_efforts = _values(self._pair(efforts), 2, "wheel effort limits")
        selected_modes = self._pair(modes)
        if any(mode not in ("force", "acceleration") for mode in selected_modes):
            raise ValueError("wheel effort mode must be force or acceleration")
        return selected_efforts, selected_modes

    def _gain_update(self, desired_kps):
        kps, kds = self._gains()
        for slot, index in enumerate(self.indices):
            kps[index], kds[index] = desired_kps[slot], self.original["kds"][slot]
        kp_array, kd_array = self.array(kps), self.array(kds)
        prepared_kps = _values(kp_array, self.count, "prepared joint Kp")
        prepared_kds = _values(kd_array, self.count, "prepared joint Kd")
        if any(desired > 0 and prepared_kps[i] <= 0 for desired, i in zip(desired_kps, self.indices)):
            raise ValueError("prepared hold Kp lost its positive value")
        if self._pair(prepared_kds) != self.original["kds"] or any(
                (prepared_kps[i], prepared_kds[i]) != (kps[i], kds[i])
                for i in range(self.count) if i not in self.indices):
            raise ValueError("gain conversion changed original damping or nonwheel gains")
        return kp_array, kd_array, self._pair(prepared_kps)

    def apply(self, vector) -> None:
        if self.failure is not None:
            raise RuntimeError("parking hold is faulted: " + self.failure)
        targets = _values(vector, 2, "wheel targets")
        holding = all(v == 0 for v in targets)
        entering, releasing = holding and self.anchor is None, not holding and self.anchor is not None
        anchor = self.anchor
        if entering:
            anchor = self._pair(_values(self.native.get_joint_positions(), self.count, "encoder positions"))
        velocity = self.array(targets)
        _values(velocity, 2, "prepared wheel targets")
        indices = self.array(self.indices, integer=True)
        if (any(isinstance(i, bool) or not isinstance(i, Integral) for i in indices)
                or list(indices) != list(self.indices)):
            raise ValueError("prepared wheel indices changed")
        positions = self.array(anchor) if holding else None
        if holding:
            _values(positions, 2, "prepared encoder anchor")
        action = self.action_factory(joint_positions=positions, joint_velocities=velocity, joint_indices=indices)
        gains = self._gain_update(self.hold_kps if entering else self.original["kps"]) if entering or releasing else None
        # Validate the entire command and converted arrays before any drive/target write.
        try:
            if gains is not None:
                self.controller.set_gains(kps=gains[0], kds=gains[1], save_to_usd=False)
                kps, kds = self._gains()
                efforts, modes = self._limits()
                if (self._pair(kps) != gains[2] or self._pair(kds) != self.original["kds"]
                        or efforts != self.original["max_efforts"] or modes != self.original["effort_modes"]):
                    raise RuntimeError("parking hold drive readback differs from requested gains/original limits")
            self.native.apply_action(action)
        except Exception as error:
            self.failure = f"{type(error).__name__}: {error}"
            raise
        self.anchor = list(anchor) if holding else None
        self.hold_entries += int(entering)
        self.hold_releases += int(releasing)
        self.hold_actions += int(holding)
        self.velocity_actions += int(not holding)

    def report(self) -> dict[str, Any]:
        kps, kds = self._gains()
        efforts, modes = self._limits()
        return {"enabled": True, "state": "faulted" if self.failure else "hold" if self.anchor is not None else "velocity",
            "wheel_names": list(self.names), "wheel_indices": list(self.indices),
            "response_time_s": RESPONSE_TIME_S, "response_source": "authored_not_measured",
            "hold_kps": list(self.hold_kps), "original": {k: list(v) for k, v in self.original.items()},
            "readback": {"kps": self._pair(kps), "kds": self._pair(kds), "max_efforts": efforts, "effort_modes": modes},
            "anchor_rad": list(self.anchor) if self.anchor is not None else None,
            "hold_entries": self.hold_entries, "hold_releases": self.hold_releases,
            "hold_actions": self.hold_actions, "velocity_actions": self.velocity_actions,
            "failure": self.failure, "physical_effect_verified": False}
