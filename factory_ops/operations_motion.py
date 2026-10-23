"""Deterministic kinematic path motion for operations-mode simulation.

This module does not model tire contact, actuators, sensors, or ROS feedback.  It
only advances an authored planar pose with a rest-to-rest motion envelope so the
operations model can account for travel time without claiming physics evidence.
"""

from __future__ import annotations

from collections import deque
import math

from .drive_profile import DriveLimits
from .motion import DriveGeometry


_POSITION_EPS_M = 1e-12
_ANGLE_EPS_RAD = 1e-12
_QUINTIC_MAX_RATE = 15.0 / 8.0
_QUINTIC_MAX_ACCEL = 10.0 * math.sqrt(3.0) / 3.0


def _finite_number(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _shortest_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _duration(distance: float, max_rate: float, max_accel: float) -> float:
    """Return the minimum quintic rest-to-rest duration for both limits.

    For ``s(u)=10u^3-15u^4+6u^5``, max(ds/du)=15/8 and
    max(abs(d2s/du2))=10*sqrt(3)/3.  The constants are exercised by the
    operations-motion limit tests rather than copied into an unevaluated
    numeric walkthrough.
    """

    return max(
        _QUINTIC_MAX_RATE * distance / max_rate,
        math.sqrt(_QUINTIC_MAX_ACCEL * distance / max_accel),
    )


def _quintic(progress: float) -> tuple[float, float]:
    """Return normalized position and first derivative for progress in [0, 1]."""

    squared = progress * progress
    cubed = squared * progress
    position = cubed * (10.0 + progress * (-15.0 + 6.0 * progress))
    derivative = 30.0 * squared * (1.0 - progress) * (1.0 - progress)
    return position, derivative


class PathMotion:
    """Advance waypoint motion with separate in-place turns and translations."""

    def __init__(self, limits: DriveLimits, geometry: DriveGeometry) -> None:
        if not isinstance(limits, DriveLimits) or not isinstance(geometry, DriveGeometry):
            raise TypeError("PathMotion requires DriveLimits and DriveGeometry")
        self.limits = limits
        self.geometry = geometry
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0
        self.phase = "stopped"
        self._route: deque[tuple[float, float]] | None = None
        self._expected_points: tuple[tuple[float, float], ...] = ()
        self._elapsed_s = 0.0
        self._duration_s = 0.0
        self._start_x_m = 0.0
        self._start_y_m = 0.0
        self._start_yaw_rad = 0.0
        self._distance = 0.0
        self._heading_rad = 0.0
        self._alignment_target: float | None = None

        wheel_linear_m_s = limits.max_wheel_rad_s * geometry.wheel_radius_m
        wheel_linear_accel_m_s2 = limits.wheel_accel_rad_s2 * geometry.wheel_radius_m
        self._max_linear_m_s = min(limits.max_linear_m_s, wheel_linear_m_s)
        self._max_linear_accel_m_s2 = min(
            limits.linear_accel_m_s2,
            limits.linear_decel_m_s2,
            wheel_linear_accel_m_s2,
        )
        # [unit-mismatch: paradigmatic]
        # Wheel tangential speed [m/s] divided by half-separation [m] yields
        # body angular speed [rad/s]; radians are dimensionless.  Geometry is
        # supplied by the caller from the run's asset-geometry provenance.
        self._max_angular_rad_s = min(
            limits.max_angular_rad_s,
            2.0 * wheel_linear_m_s / geometry.wheel_separation_m,
        )
        self._max_angular_accel_rad_s2 = min(
            limits.angular_accel_rad_s2,
            limits.angular_decel_rad_s2,
            2.0 * wheel_linear_accel_m_s2 / geometry.wheel_separation_m,
        )

    @property
    def stopped(self) -> bool:
        return self.phase == "stopped"

    def rotate_to(self, pose: dict[str, float], yaw_rad: float, dt: float) -> bool:
        """Align at a retained circular footprint with the same wheel/body limits."""
        self._validate_inputs(pose, deque(), dt)
        if not _finite_number(yaw_rad):
            raise ValueError("alignment yaw must be finite")
        if self.phase not in ("stopped", "align"):
            raise ValueError("alignment requires a completed path segment")
        if self.phase == "align" and yaw_rad != self._alignment_target:
            raise ValueError("alignment target cannot change during motion")
        if self.stopped:
            rotation = _shortest_angle(yaw_rad - pose["yaw_rad"])
            if abs(rotation) <= _ANGLE_EPS_RAD:
                return True
            self._alignment_target = yaw_rad
            self._start_yaw_rad = pose["yaw_rad"]
            self._distance = rotation
            self._elapsed_s = 0.0
            self._duration_s = _duration(abs(rotation), self._max_angular_rad_s, self._max_angular_accel_rad_s2)
            self.phase = "align"
        self._elapsed_s = min(self._duration_s, self._elapsed_s + dt)
        position, derivative = _quintic(self._elapsed_s / self._duration_s)
        pose["yaw_rad"] = self._start_yaw_rad + self._distance * position
        self.linear_m_s = 0.0
        self.angular_rad_s = self._distance * derivative / self._duration_s
        if self._elapsed_s >= self._duration_s:
            self.phase = "stopped"
            self.angular_rad_s = 0.0
            self._alignment_target = None
        return self.stopped

    def estimated_duration(self, points: tuple[tuple[float, float], ...], yaw_rad: float,
                           final_yaw_rad: float | None = None) -> float:
        """Conservative travel-only duration, stopping at every authored waypoint."""
        if not _finite_number(yaw_rad) or final_yaw_rad is not None and not _finite_number(final_yaw_rad):
            raise ValueError("planning headings must be finite")
        self._validate_inputs({"x_m": 0.0, "y_m": 0.0, "yaw_rad": yaw_rad}, deque(points), 1.0)
        total = 0.0
        for start, end in zip(points, points[1:]):
            distance = math.dist(start, end)
            if distance <= _POSITION_EPS_M:
                continue
            heading = math.atan2(end[1] - start[1], end[0] - start[0])
            rotation = abs(_shortest_angle(heading - yaw_rad))
            total += _duration(rotation, self._max_angular_rad_s, self._max_angular_accel_rad_s2)
            total += _duration(distance, self._max_linear_m_s, self._max_linear_accel_m_s2)
            yaw_rad = heading
        if final_yaw_rad is not None:
            total += _duration(abs(_shortest_angle(final_yaw_rad - yaw_rad)),
                               self._max_angular_rad_s, self._max_angular_accel_rad_s2)
        return total

    @staticmethod
    def _validate_inputs(
        pose: dict[str, float], points: deque[tuple[float, float]], dt: float,
    ) -> tuple[float, float, float, tuple[tuple[float, float], ...]]:
        if not isinstance(pose, dict):
            raise TypeError("pose must be a mutable dict")
        try:
            x_m, y_m, yaw_rad = pose["x_m"], pose["y_m"], pose["yaw_rad"]
        except KeyError as error:
            raise ValueError("pose requires x_m, y_m, and yaw_rad") from error
        if not all(_finite_number(value) for value in (x_m, y_m, yaw_rad)):
            raise ValueError("pose values must be finite numbers")
        if not _finite_number(dt) or dt <= 0:
            raise ValueError("dt must be a finite positive number")
        if not isinstance(points, deque):
            raise TypeError("points must be a deque")
        snapshot = tuple(points)
        if any(
            not isinstance(point, tuple)
            or len(point) != 2
            or not all(_finite_number(value) for value in point)
            for point in snapshot
        ):
            raise ValueError("each path point must be a finite (x_m, y_m) tuple")
        return float(x_m), float(y_m), float(yaw_rad), snapshot

    def _accept_route(
        self, points: deque[tuple[float, float]], snapshot: tuple[tuple[float, float], ...],
    ) -> None:
        if not self.stopped and (points is not self._route or snapshot != self._expected_points):
            raise ValueError("path cannot change while motion is in progress")
        if self.stopped:
            self._route = points
            self._expected_points = snapshot

    def _sync_expected(self) -> None:
        assert self._route is not None
        self._expected_points = tuple(self._route)

    def _begin_next(self, pose: dict[str, float]) -> bool:
        assert self._route is not None
        while self._route:
            target_x_m, target_y_m = self._route[0]
            delta_x_m = target_x_m - pose["x_m"]
            delta_y_m = target_y_m - pose["y_m"]
            distance_m = math.hypot(delta_x_m, delta_y_m)
            if distance_m <= _POSITION_EPS_M:
                self._route.popleft()
                self._sync_expected()
                continue
            heading_rad = math.atan2(delta_y_m, delta_x_m)
            rotation_rad = _shortest_angle(heading_rad - pose["yaw_rad"])
            self._start_x_m = pose["x_m"]
            self._start_y_m = pose["y_m"]
            self._start_yaw_rad = pose["yaw_rad"]
            self._elapsed_s = 0.0
            if abs(rotation_rad) > _ANGLE_EPS_RAD:
                self.phase = "turn"
                self._distance = rotation_rad
                self._heading_rad = heading_rad
                self._duration_s = _duration(
                    abs(rotation_rad), self._max_angular_rad_s, self._max_angular_accel_rad_s2,
                )
            else:
                self.phase = "translate"
                self._distance = distance_m
                self._heading_rad = heading_rad
                self._duration_s = _duration(
                    distance_m, self._max_linear_m_s, self._max_linear_accel_m_s2,
                )
            return True
        self.phase = "stopped"
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0
        return False

    def _finish_phase(self, pose: dict[str, float]) -> None:
        if self.phase == "turn":
            pose["yaw_rad"] = self._start_yaw_rad + self._distance
            self.phase = "translate"
            assert self._route is not None
            target_x_m, target_y_m = self._route[0]
            self._start_x_m = pose["x_m"]
            self._start_y_m = pose["y_m"]
            self._start_yaw_rad = pose["yaw_rad"]
            self._distance = math.hypot(target_x_m - pose["x_m"], target_y_m - pose["y_m"])
            self._heading_rad = pose["yaw_rad"]
            self._elapsed_s = 0.0
            self._duration_s = _duration(
                self._distance, self._max_linear_m_s, self._max_linear_accel_m_s2,
            )
        elif self.phase == "translate":
            assert self._route is not None
            target_x_m, target_y_m = self._route.popleft()
            pose["x_m"] = target_x_m
            pose["y_m"] = target_y_m
            self._sync_expected()
            self.phase = "stopped"
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0

    def _evaluate(self, pose: dict[str, float]) -> None:
        progress = min(1.0, self._elapsed_s / self._duration_s)
        position, derivative = _quintic(progress)
        if self.phase == "turn":
            pose["yaw_rad"] = self._start_yaw_rad + self._distance * position
            self.linear_m_s = 0.0
            self.angular_rad_s = self._distance * derivative / self._duration_s
        else:
            travelled_m = self._distance * position
            pose["x_m"] = self._start_x_m + math.cos(self._heading_rad) * travelled_m
            pose["y_m"] = self._start_y_m + math.sin(self._heading_rad) * travelled_m
            self.linear_m_s = self._distance * derivative / self._duration_s
            self.angular_rad_s = 0.0

    def advance(
        self, pose: dict[str, float], points: deque[tuple[float, float]], dt: float,
    ) -> None:
        """Advance ``pose`` and consume reached ``points`` in place."""

        if self.phase == "align":
            raise ValueError("finish alignment before starting a path")
        x_m, y_m, yaw_rad, snapshot = self._validate_inputs(pose, points, dt)
        self._accept_route(points, snapshot)
        pose["x_m"], pose["y_m"], pose["yaw_rad"] = x_m, y_m, yaw_rad
        remaining_s = float(dt)
        while remaining_s > 0.0:
            if self.stopped and not self._begin_next(pose):
                return
            available_s = self._duration_s - self._elapsed_s
            step_s = min(remaining_s, available_s)
            self._elapsed_s += step_s
            remaining_s -= step_s
            self._evaluate(pose)
            if self._elapsed_s >= self._duration_s:
                self._finish_phase(pose)
            else:
                break
