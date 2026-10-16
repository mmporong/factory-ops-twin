"""Differential-drive command and encoder math, independent of simulator truth."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, sin


@dataclass(frozen=True, slots=True)
class DriveGeometry:
    wheel_radius_m: float
    wheel_separation_m: float

    def __post_init__(self) -> None:
        if not all(isfinite(v) and v > 0 for v in (self.wheel_radius_m, self.wheel_separation_m)):
            raise ValueError("drive dimensions must be finite positive metres")

    def wheel_targets(self, linear_m_s: float, angular_rad_s: float) -> tuple[float, float]:
        if not all(isfinite(v) for v in (linear_m_s, angular_rad_s)):
            raise ValueError("command velocities must be finite")
        # [unit-mismatch: paradigmatic]
        # Angular rad/s * half separation m is tangential m/s (radian is dimensionless).
        # Drive dimensions are supplied from the run's asset-geometry provenance.
        turn_m_s = angular_rad_s * self.wheel_separation_m / 2.0
        return ((linear_m_s - turn_m_s) / self.wheel_radius_m,
                (linear_m_s + turn_m_s) / self.wheel_radius_m)


@dataclass(frozen=True, slots=True)
class EncoderSample:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    linear_m_s: float
    angular_rad_s: float


class WheelOdometry:
    """Integrate measured continuous joint angles in a local odom frame.

    The origin is zero at initialization. The authored map spawn and simulator
    world pose are deliberately absent from this interface. Wheel slip remains
    an estimation error for a sensor-based localizer to correct.
    """

    def __init__(self, geometry: DriveGeometry) -> None:
        self.geometry = geometry
        self._previous: tuple[float, float, float] | None = None
        self.sample = EncoderSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def update(self, stamp_s: float, left_rad: float, right_rad: float) -> EncoderSample:
        if not all(isfinite(v) for v in (stamp_s, left_rad, right_rad)) or stamp_s < 0:
            raise ValueError("encoder input must be finite with a nonnegative stamp")
        if self._previous is None:
            self._previous = (stamp_s, left_rad, right_rad)
            self.sample = EncoderSample(stamp_s, 0.0, 0.0, 0.0, 0.0, 0.0)
            return self.sample
        before_s, before_left_rad, before_right_rad = self._previous
        dt_s = stamp_s - before_s
        if dt_s <= 0:
            raise ValueError("encoder stamps must strictly increase")
        left_m = (left_rad - before_left_rad) * self.geometry.wheel_radius_m
        right_m = (right_rad - before_right_rad) * self.geometry.wheel_radius_m
        distance_m = (left_m + right_m) / 2.0
        rotation_rad = (right_m - left_m) / self.geometry.wheel_separation_m
        # Exact planar arc integration, including the straight-line limit.
        if abs(rotation_rad) < 1e-9:
            local_x_m, local_y_m = distance_m, 0.0
        else:
            arc_radius_m = distance_m / rotation_rad
            local_x_m = arc_radius_m * sin(rotation_rad)
            local_y_m = arc_radius_m * (1.0 - cos(rotation_rad))
        old = self.sample
        self.sample = EncoderSample(
            stamp_s,
            old.x_m + cos(old.yaw_rad) * local_x_m - sin(old.yaw_rad) * local_y_m,
            old.y_m + sin(old.yaw_rad) * local_x_m + cos(old.yaw_rad) * local_y_m,
            old.yaw_rad + rotation_rad,
            distance_m / dt_s,
            rotation_rad / dt_s,
        )
        self._previous = (stamp_s, left_rad, right_rad)
        return self.sample


class VelocityWatchdog:
    """A stale, future, nonfinite or out-of-range command cannot keep moving."""

    def __init__(self, *, max_age_s: float, max_linear_m_s: float, max_angular_rad_s: float) -> None:
        if not all(isfinite(v) and v > 0 for v in (max_age_s, max_linear_m_s, max_angular_rad_s)):
            raise ValueError("watchdog limits must be positive and finite")
        self.max_age_s = max_age_s
        self.max_linear_m_s = max_linear_m_s
        self.max_angular_rad_s = max_angular_rad_s
        self._command: tuple[float, float, float] | None = None

    def receive(self, stamp_s: float, linear_m_s: float, angular_rad_s: float) -> None:
        self._command = None
        if not all(isfinite(v) for v in (stamp_s, linear_m_s, angular_rad_s)) or stamp_s < 0:
            raise ValueError("velocity input must be finite with a nonnegative stamp")
        if abs(linear_m_s) > self.max_linear_m_s or abs(angular_rad_s) > self.max_angular_rad_s:
            raise ValueError("velocity command exceeds configured limits")
        self._command = (stamp_s, linear_m_s, angular_rad_s)

    def command(self, now_s: float) -> tuple[float, float]:
        if not isfinite(now_s) or now_s < 0:
            self._command = None
            raise ValueError("current simulation time must be finite and nonnegative")
        if self._command is None:
            return 0.0, 0.0
        stamp_s, linear_m_s, angular_rad_s = self._command
        if not 0 <= now_s - stamp_s <= self.max_age_s:
            self._command = None
            return 0.0, 0.0
        return linear_m_s, angular_rad_s
