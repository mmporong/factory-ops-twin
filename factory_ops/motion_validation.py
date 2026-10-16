"""Independent dimensional checks of recorded wheel-drive commands and response."""

from __future__ import annotations

from collections import Counter
import math
from typing import Any, Mapping, Sequence

from .drive_profile import DriveLimits
from .motion import DriveGeometry


def _finite(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("expected a finite number")
    return float(value)


def validate_drive_trace(
    samples: Sequence[Mapping[str, Any]], geometry: DriveGeometry, limits: DriveLimits, dt_s: float,
) -> dict[str, Any]:
    """Check limits without replaying the command-shaping implementation.

    Command limits and physical response are reported separately. Encoder speed
    follows wheel motion, which can differ from chassis motion because of slip.
    Ground-truth poses are read for evaluation, never returned as drive inputs.
    """
    errors: list[str] = []
    phases: Counter[str] = Counter()
    maxima: dict[str, float] = dict.fromkeys(("linear_accel_m_s2", "linear_decel_m_s2", "angular_accel_rad_s2",
                           "angular_decel_rad_s2", "lateral_accel_m_s2", "wheel_accel_rad_s2",
                           "wheel_speed_rad_s"), 0.0)
    encoder_max = {"linear_m_s": 0.0, "angular_rad_s": 0.0}
    emergency_count = 0
    previous: tuple[float, int, float, float, tuple[float, float], list[float], float] | None = None
    measured_distance_m = measured_yaw_rad = 0.0
    if len(samples) < 2:
        errors.append("drive trace must contain at least two physics samples")
    if not math.isfinite(dt_s) or dt_s <= 0:
        raise ValueError("physics interval must be finite and positive")
    for index, sample in enumerate(samples):
        try:
            stamp = _finite(sample["sim_time_s"])
            frame = sample["physics_frame"]
            if stamp < 0 or type(frame) is not int or frame < 0:
                raise ValueError("invalid physics time or frame")
            command, profile, odometry = sample["command"], sample["motion_profile"], sample["odometry"]
            v, w = _finite(command["linear_m_s"]), _finite(command["angular_rad_s"])
            wheels_raw = sample["wheel_target_rad_s"]
            if not isinstance(wheels_raw, list) or len(wheels_raw) != 2:
                raise ValueError("wheel target must contain two values")
            wheels = (_finite(wheels_raw[0]), _finite(wheels_raw[1]))
            emergency = profile["emergency_stop"]
            phase = profile["phase"]
            phases_allowed = {"translate", "curve", "brake_before_turn", "turn_in_place",
                              "settle_before_departure", "normal_stop", "fault_stop"}
            if type(emergency) is not bool or phase not in phases_allowed:
                raise ValueError("invalid profile state")
            if any(type(profile[key]) is not bool for key in ("curvature_limited", "wheel_speed_limited")):
                raise ValueError("invalid profile limiting flags")
            input_command = profile["input_command"]
            input_v = _finite(input_command["linear_m_s"])
            input_w = _finite(input_command["angular_rad_s"])
            if abs(input_v) > limits.max_linear_m_s or abs(input_w) > limits.max_angular_rad_s:
                raise ValueError("profile input exceeds the authored envelope")
            reason = profile["stop_reason"]
            normal_reasons = {"none", "normal_mission_complete", "normal_schedule_stop"}
            fault_reasons = {"watchdog_stale", "mission_failed", "excessive_tilt", "injected_fault",
                             "exception_cleanup"}
            if (emergency != (phase == "fault_stop")
                    or reason not in (fault_reasons if emergency else normal_reasons)):
                raise ValueError("profile fault and stop reason do not agree")
            if (phase == "normal_stop" or reason in {"normal_mission_complete", "normal_schedule_stop"}) and (input_v or input_w):
                raise ValueError("normal stop requires a zero profile input")
            phases[phase] += 1
            for name in encoder_max:
                encoder_max[name] = max(encoder_max[name], abs(_finite(odometry[name])))
            gt = sample["ground_truth"]
            position = [_finite(value) for value in gt["position_xyz_m"]]
            orientation = [_finite(value) for value in gt["orientation_wxyz"]]
            if len(position) != 3 or len(orientation) != 4:
                raise ValueError("invalid ground-truth pose size")
            if abs(sum(value * value for value in orientation) - 1.0) > 0.002:
                raise ValueError("ground-truth quaternion is not normalized")
            qw, qx, qy, qz = orientation
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
            if max(abs(roll), abs(pitch)) > 0.35:
                errors.append(f"sample {index}: excessive physical tilt")
            if abs(v) > limits.max_linear_m_s + 1e-9 or abs(w) > limits.max_angular_rad_s + 1e-9:
                errors.append(f"sample {index}: body speed limit exceeded")
            max_wheel = max(abs(value) for value in wheels)
            lateral = abs(v * w)
            maxima["wheel_speed_rad_s"] = max(maxima["wheel_speed_rad_s"], max_wheel)
            maxima["lateral_accel_m_s2"] = max(maxima["lateral_accel_m_s2"], lateral)
            if max_wheel > limits.max_wheel_rad_s + 1e-9:
                errors.append(f"sample {index}: wheel speed limit exceeded")
            if lateral > limits.lateral_accel_m_s2 + 1e-9:
                errors.append(f"sample {index}: command lateral acceleration exceeded")
            # Independent differential-drive equation, rather than calling the profile.
            expected = ((v - w * geometry.wheel_separation_m / 2) / geometry.wheel_radius_m,
                        (v + w * geometry.wheel_separation_m / 2) / geometry.wheel_radius_m)
            if any(abs(a - b) > 1e-6 for a, b in zip(wheels, expected)):
                errors.append(f"sample {index}: wheel target does not match body command")
            if emergency:
                emergency_count += 1
                if phase != "fault_stop" or any(value != 0.0 for value in (v, w, *wheels)):
                    errors.append(f"sample {index}: fault stop is not a zero actuator command")
            if index == 0:
                # Both workers create the profile at rest before the first recorded tick.
                previous = (stamp - dt_s, frame - 1, 0.0, 0.0, (0.0, 0.0), position, yaw)
            if previous is not None:
                before_t, before_frame, before_v, before_w, before_wheels, before_position, before_yaw = previous
                actual_dt = stamp - before_t
                if frame != before_frame + 1 or abs(actual_dt - dt_s) > 1e-6:
                    errors.append(f"sample {index}: missing or irregular physics sample")
                measured_distance_m += math.hypot(position[0] - before_position[0], position[1] - before_position[1])
                measured_yaw_rad += math.atan2(math.sin(yaw - before_yaw), math.cos(yaw - before_yaw))
                if not emergency and actual_dt > 0:
                    for axis, before, after in (("linear", before_v, v), ("angular", before_w, w)):
                        direction = "decel" if abs(after) < abs(before) else "accel"
                        unit = "m_s2" if axis == "linear" else "rad_s2"
                        key = f"{axis}_{direction}_{unit}"
                        value = abs(after - before) / actual_dt
                        maxima[key] = max(maxima[key], value)
                        if value > getattr(limits, key) + 1e-6:
                            errors.append(f"sample {index}: {key} limit exceeded")
                        if before * after < -1e-12:
                            errors.append(f"sample {index}: {axis} reversal did not pass through zero")
                    wheel_accel = max(abs(a - b) for a, b in zip(wheels, before_wheels)) / actual_dt
                    maxima["wheel_accel_rad_s2"] = max(maxima["wheel_accel_rad_s2"], wheel_accel)
                    if wheel_accel > limits.wheel_accel_rad_s2 + 1e-6:
                        errors.append(f"sample {index}: wheel acceleration limit exceeded")
            previous = (stamp, frame, v, w, wheels, position, yaw)
        except (KeyError, TypeError, ValueError, IndexError) as error:
            errors.append(f"sample {index}: malformed motion evidence ({error})")
            previous = None
    return {
        "passed": not errors, "errors": errors, "sample_count": len(samples),
        "command_maxima": maxima, "phase_samples": dict(phases), "fault_stop_samples": emergency_count,
        "physical_response": {
            "encoder_maximum_absolute_speed": encoder_max,
            "ground_truth_path_length_m": measured_distance_m,
            "ground_truth_net_yaw_rad": measured_yaw_rad,
            "scope": "observed response; command bounds do not certify measured chassis acceleration",
        },
        "limit_provenance": "authored simulation limits; no real motor or payload calibration",
    }
