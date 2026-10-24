"""Exclusive charging slots and fail-closed docking confirmation.

This state machine never navigates or infers contact from a successful Nav2 goal.
An adapter must provide fresh, run-bound contact and odometry observations.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping


def _number(value: Any, label: str) -> float:
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
        raise ValueError(f"{label} must be finite and nonnegative")
    return float(value)


@dataclass
class DockState:
    robot_id: str
    dock_id: str
    phase: str = "approach"
    last_stamp_s: float | None = None
    contact_since_s: float | None = None
    contact_confirmed: bool = False
    fault: str | None = None


class ChargingFleet:
    def __init__(self, run_id: str, docks: Mapping[str, str], *,
                 max_gap_s: float, contact_dwell_s: float, stopped_linear_m_s: float,
                 stopped_angular_rad_s: float, resume_soc: float) -> None:
        if not run_id or not docks or any(not key or not anchor for key, anchor in docks.items()):
            raise ValueError("run and named charging anchors are required")
        if len(set(docks.values())) != len(docks):
            raise ValueError("charging slots require distinct anchors")
        values = {"max_gap_s": max_gap_s, "contact_dwell_s": contact_dwell_s,
                  "stopped_linear_m_s": stopped_linear_m_s, "stopped_angular_rad_s": stopped_angular_rad_s,
                  "resume_soc": resume_soc}
        for key, value in values.items():
            if _number(value, key) <= 0:
                raise ValueError(f"{key} must be positive")
        if resume_soc > 1:
            raise ValueError("resume SOC is a fraction")
        self.run_id, self.docks = run_id, dict(docks)
        self.max_gap_s, self.contact_dwell_s = max_gap_s, contact_dwell_s
        self.stopped_linear_m_s, self.stopped_angular_rad_s = stopped_linear_m_s, stopped_angular_rad_s
        self.resume_soc = resume_soc
        self.states: dict[str, DockState] = {}
        self.owners: dict[str, str] = {}

    def reserve(self, robot_id: str, *, carrying_payload: bool,
                preferred_dock: str | None = None) -> str | None:
        if not robot_id or type(carrying_payload) is not bool:
            raise ValueError("robot identity and explicit load state are required")
        if carrying_payload:
            raise ValueError("complete payload delivery before charging")
        if robot_id in self.states:
            return self.states[robot_id].dock_id
        if preferred_dock is not None and preferred_dock not in self.docks:
            raise ValueError("preferred charging slot is not authored")
        dock_id = (preferred_dock if preferred_dock is not None and preferred_dock not in self.owners
                   else None if preferred_dock is not None
                   else next((key for key in sorted(self.docks) if key not in self.owners), None))
        if dock_id is not None:
            self.owners[dock_id] = robot_id
            self.states[robot_id] = DockState(robot_id, dock_id)
        return dock_id

    def cancel_approach(self, robot_id: str) -> str:
        """Release only a dock that has not entered contact confirmation."""

        state = self.states[robot_id]
        if (state.phase != "approach" or state.last_stamp_s is not None
                or state.contact_since_s is not None or state.contact_confirmed):
            raise RuntimeError("only a pre-contact charging approach may be cancelled")
        if self.owners.get(state.dock_id) != robot_id:
            raise RuntimeError("charging approach owner differs from the dock state")
        dock_id = state.dock_id
        del self.owners[dock_id]
        del self.states[robot_id]
        return dock_id

    def arrived(self, robot_id: str, *, run_id: str, anchor: str, navigation_succeeded: bool) -> None:
        state = self.states[robot_id]
        if (run_id != self.run_id or type(navigation_succeeded) is not bool or not navigation_succeeded
                or anchor != self.docks[state.dock_id] or state.phase != "approach"):
            raise ValueError("dock approach requires a successful run-bound arrival at its reserved anchor")
        state.phase = "contact_pending"

    def observe(self, robot_id: str, *, run_id: str, dock_id: str, stamp_s: float,
                contact: bool, linear_m_s: float, angular_rad_s: float, soc: float,
                departure_soc: float | None = None,
                departure_allowed: bool = True) -> dict[str, Any]:
        state = self.states[robot_id]
        stamp, fraction = _number(stamp_s, "stamp"), _number(soc, "SOC")
        threshold = self.resume_soc if departure_soc is None else _number(departure_soc, "departure SOC")
        if (fraction > 1 or threshold < self.resume_soc or threshold > 1
                or type(contact) is not bool or type(departure_allowed) is not bool):
            raise ValueError("contact must be boolean and SOC must be a fraction")
        if any(type(v) not in (float, int) or not math.isfinite(v) for v in (linear_m_s, angular_rad_s)):
            raise ValueError("odometry velocities must be finite")
        if run_id != self.run_id or dock_id != state.dock_id or self.owners.get(dock_id) != robot_id:
            raise ValueError("contact observation differs from the run or slot owner")
        if state.phase not in {"contact_pending", "charging", "undock_pending", "ready_to_leave"}:
            raise ValueError("contact observation is outside docking lifecycle")
        if state.last_stamp_s is not None:
            dt = stamp - state.last_stamp_s
            if dt <= 0:
                raise ValueError("dock observations must advance")
            if dt > self.max_gap_s:
                state.contact_confirmed, state.contact_since_s = False, None
                state.phase, state.fault = "fault", "stale dock observation"
                raise ValueError(state.fault)
        state.last_stamp_s = stamp
        stationary = abs(linear_m_s) <= self.stopped_linear_m_s and abs(angular_rad_s) <= self.stopped_angular_rad_s
        if state.phase in {"undock_pending", "ready_to_leave"}:
            state.contact_confirmed = False
            state.contact_since_s = None
            state.phase = "ready_to_leave" if not contact and stationary else "undock_pending"
            return asdict(state)
        if not (contact and stationary):
            state.contact_confirmed, state.contact_since_s = False, None
            state.phase = "contact_pending"
        else:
            if state.contact_since_s is None:
                state.contact_since_s = stamp
            state.contact_confirmed = stamp - state.contact_since_s >= self.contact_dwell_s
            state.phase = "charging" if state.contact_confirmed else "contact_pending"
            if state.contact_confirmed and departure_allowed and fraction >= threshold:
                state.phase, state.contact_confirmed = "undock_pending", False
        return asdict(state)

    def release(self, robot_id: str, *, run_id: str, observed_at_s: float) -> None:
        state = self.states[robot_id]
        stamp = _number(observed_at_s, "release stamp")
        if (run_id != self.run_id or state.phase != "ready_to_leave" or state.last_stamp_s is None
                or not 0 <= stamp - state.last_stamp_s <= self.max_gap_s):
            raise ValueError("release requires observed disengagement and a stationary robot")
        del self.owners[state.dock_id]
        del self.states[robot_id]

    def can_dispatch(self, robot_id: str) -> bool:
        return robot_id not in self.states
