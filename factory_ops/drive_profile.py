"""Body and wheel command envelopes for forward, curved and in-place motion.

These are authored simulation limits, not calibrated motor dynamics. Commands
remain wheel velocity targets; the physics engine determines actual motion.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from .motion import DriveGeometry


@dataclass(frozen=True, slots=True)
class DriveLimits:
    max_linear_m_s: float = 0.6
    max_angular_rad_s: float = 1.2
    linear_accel_m_s2: float = 0.25
    linear_decel_m_s2: float = 0.35
    angular_accel_rad_s2: float = 0.5
    angular_decel_rad_s2: float = 0.7
    lateral_accel_m_s2: float = 0.2
    max_wheel_rad_s: float = 6.0
    wheel_accel_rad_s2: float = 4.0

    def __post_init__(self) -> None:
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) or value <= 0 for value in asdict(self).values()):
            raise ValueError("drive profile limits must be finite positive numbers")


@dataclass(frozen=True, slots=True)
class DriveCommand:
    linear_m_s: float
    angular_rad_s: float
    wheel_targets_rad_s: tuple[float, float]
    phase: str
    curvature_limited: bool
    wheel_speed_limited: bool
    emergency_stop: bool


class DriveProfile:
    """Shape normal commands at physics cadence and retain a separate fault stop.

    Normal reversals brake through zero. A requested in-place turn first brakes
    translation, and departure waits for the preceding in-place rotation to
    settle. Curvature speed scaling retains the requested curvature; body and
    wheel acceleration bounds then constrain the applied command together.
    """

    def __init__(self, geometry: DriveGeometry, limits: DriveLimits) -> None:
        self.geometry = geometry
        self.limits = limits
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0
        self._in_place = False

    @staticmethod
    def _ramp(current: float, target: float, accel: float, decel: float, dt: float) -> float:
        if current * target < 0:
            target = 0.0
        braking = abs(target) < abs(current)
        maximum_step = (decel if braking else accel) * dt
        difference = target - current
        return current + max(-maximum_step, min(maximum_step, difference))

    def step(
        self, linear_m_s: float, angular_rad_s: float, dt_s: float, *, emergency_stop: bool = False,
    ) -> DriveCommand:
        if emergency_stop:
            self.linear_m_s = self.angular_rad_s = 0.0
            self._in_place = False
            return DriveCommand(0.0, 0.0, (0.0, 0.0), "fault_stop", False, False, True)
        if (not all(not isinstance(value, bool) and isinstance(value, (int, float))
                    and math.isfinite(value) for value in (linear_m_s, angular_rad_s, dt_s))
                or dt_s <= 0):
            self.linear_m_s = self.angular_rad_s = 0.0
            self._in_place = False
            raise ValueError("drive profile input must be finite with positive dt")
        limits = self.limits
        if abs(linear_m_s) > limits.max_linear_m_s or abs(angular_rad_s) > limits.max_angular_rad_s:
            self.linear_m_s = self.angular_rad_s = 0.0
            self._in_place = False
            raise ValueError("requested velocity exceeds the authored drive envelope")
        phase = "curve" if linear_m_s and angular_rad_s else "translate"
        if linear_m_s == 0.0 and angular_rad_s != 0.0:
            self._in_place = True
            if self.linear_m_s != 0.0:
                angular_rad_s = 0.0
                phase = "brake_before_turn"
            else:
                phase = "turn_in_place"
        elif linear_m_s != 0.0 and self._in_place:
            if self.angular_rad_s != 0.0:
                linear_m_s = angular_rad_s = 0.0
                phase = "settle_before_departure"
            else:
                self._in_place = False
        elif linear_m_s == angular_rad_s == 0.0:
            phase = "normal_stop"

        lateral = abs(linear_m_s * angular_rad_s)
        curvature_limited = lateral > limits.lateral_accel_m_s2
        if curvature_limited:
            scale = math.sqrt(limits.lateral_accel_m_s2 / lateral)
            linear_m_s *= scale
            angular_rad_s *= scale
        largest_requested_wheel = max(abs(value) for value in self.geometry.wheel_targets(linear_m_s, angular_rad_s))
        wheel_speed_limited = largest_requested_wheel > limits.max_wheel_rad_s
        if wheel_speed_limited:
            scale = limits.max_wheel_rad_s / largest_requested_wheel
            linear_m_s *= scale
            angular_rad_s *= scale
        candidate_linear = self._ramp(self.linear_m_s, linear_m_s,
                                      limits.linear_accel_m_s2, limits.linear_decel_m_s2, dt_s)
        candidate_angular = self._ramp(self.angular_rad_s, angular_rad_s,
                                       limits.angular_accel_rad_s2, limits.angular_decel_rad_s2, dt_s)
        before_wheels = self.geometry.wheel_targets(self.linear_m_s, self.angular_rad_s)

        def wheel_fraction(linear: float, angular: float) -> float:
            nonlocal wheel_speed_limited
            after_wheels = self.geometry.wheel_targets(linear, angular)
            largest_step = max(abs(after - before) for before, after in zip(before_wheels, after_wheels))
            fraction = min(1.0, limits.wheel_accel_rad_s2 * dt_s / largest_step) if largest_step else 1.0
            for before, after in zip(before_wheels, after_wheels):
                delta = after - before
                if abs(after) > limits.max_wheel_rad_s + 1e-12 and after * delta > 0.0:
                    wheel_speed_limited = True
                    boundary = math.copysign(limits.max_wheel_rad_s, after)
                    fraction = min(fraction, max(0.0, (boundary - before) / delta))
            return fraction

        fraction = wheel_fraction(candidate_linear, candidate_angular)
        applied_linear = self.linear_m_s + fraction * (candidate_linear - self.linear_m_s)
        applied_angular = self.angular_rad_s + fraction * (candidate_angular - self.angular_rad_s)
        lateral_exceeded = abs(applied_linear * applied_angular) > limits.lateral_accel_m_s2 + 1e-12
        if lateral_exceeded or fraction < 1e-12:
            curvature_limited = curvature_limited or lateral_exceeded
            # Coupled limits can prevent both independently ramped components
            # from advancing. Apply decreasing components first to make room
            # for the next tick without exceeding either axis acceleration.
            if abs(candidate_linear) >= abs(self.linear_m_s):
                candidate_linear = self.linear_m_s
            if abs(candidate_angular) >= abs(self.angular_rad_s):
                candidate_angular = self.angular_rad_s
            fraction = wheel_fraction(candidate_linear, candidate_angular)
        self.linear_m_s += fraction * (candidate_linear - self.linear_m_s)
        self.angular_rad_s += fraction * (candidate_angular - self.angular_rad_s)
        # Remove only accumulated floating-point residue at a commanded zero.
        if linear_m_s == 0.0 and abs(self.linear_m_s) < 1e-12:
            self.linear_m_s = 0.0
        if angular_rad_s == 0.0 and abs(self.angular_rad_s) < 1e-12:
            self.angular_rad_s = 0.0
        return DriveCommand(self.linear_m_s, self.angular_rad_s,
                            self.geometry.wheel_targets(self.linear_m_s, self.angular_rad_s),
                            phase, curvature_limited, wheel_speed_limited, False)
