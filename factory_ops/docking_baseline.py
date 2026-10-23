"""Encoder-only low-speed pose alignment for an authored docking pad.

Simulator ground truth is accepted only by ``evaluate_docking_truth`` after a
run. The controller sees wheel-encoder odometry and the authored odom-frame
target pose.
"""
from __future__ import annotations

from dataclasses import asdict
import math
from typing import Any, Mapping, Sequence

from .drive_profile import DriveCommand, DriveLimits, DriveProfile
from .motion import DriveGeometry, EncoderSample


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if positive and result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def validate_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    if raw.get("qualification") != "encoder_docking_alignment_baseline":
        raise ValueError("wrong docking qualification")
    if _number(raw.get("physics_dt_s"), "physics_dt_s", positive=True) != 1.0 / 60.0:
        raise ValueError("docking physics cadence must be exactly 60 Hz")
    maximum_sim = _number(raw.get("maximum_sim_time_s"), "maximum_sim_time_s", positive=True)
    maximum_wall = _number(raw.get("maximum_wall_time_s"), "maximum_wall_time_s", positive=True)
    if maximum_sim > 45.0 or maximum_wall > 600.0:
        raise ValueError("docking time budget exceeds the bounded contract")
    capture_period = raw.get("capture_every_physics_steps")
    if isinstance(capture_period, bool) or not isinstance(capture_period, int) or capture_period <= 0:
        raise ValueError("capture cadence must be a positive integer")
    maximum_frames = math.ceil(maximum_sim / (float(raw["physics_dt_s"]) * capture_period))
    if maximum_frames > 180:
        raise ValueError("docking video window exceeds 180 frames")
    maximum_rows = math.ceil(maximum_sim / float(raw["physics_dt_s"]))
    if maximum_rows > 3000:
        raise ValueError("docking data window exceeds 3000 rows")
    if raw.get("unloaded") is not True:
        raise ValueError("docking baseline must declare an unloaded robot")
    geometry = DriveGeometry(**raw["drive_geometry"])
    limits = DriveLimits(**raw["drive_limits"])
    if limits.max_linear_m_s > 0.15 or limits.max_angular_rad_s > 0.4:
        raise ValueError("docking command limits exceed 0.15 m/s or 0.4 rad/s")
    target = raw["target_pose_odom"]
    target_x = _number(target.get("x_m"), "target x")
    target_y = _number(target.get("y_m"), "target y")
    target_yaw = _number(target.get("yaw_rad"), "target yaw")
    approach_distance = math.hypot(target_x, target_y)
    if not 1.2 <= approach_distance <= 1.8:
        raise ValueError("authored docking approach must be approximately 1.5 metres")
    if not 0.01 <= abs(target_y) <= 0.2 or not 0.02 <= abs(target_yaw) <= 0.2:
        raise ValueError("docking target needs small nonzero lateral and yaw offsets")
    control = raw["controller"]
    for name in ("cruise_linear_m_s", "heading_gain", "yaw_gain", "heading_gate_rad",
                 "position_tolerance_m", "yaw_tolerance_rad", "stop_linear_m_s",
                 "stop_angular_rad_s", "stationary_dwell_s", "post_stop_observation_s"):
        _number(control.get(name), name, positive=True)
    if (float(control["cruise_linear_m_s"]) > limits.max_linear_m_s
            or float(control["position_tolerance_m"]) > 0.025
            or float(control["yaw_tolerance_rad"]) > 0.035
            or float(control["stationary_dwell_s"]) < 1.0):
        raise ValueError("docking controller tolerance, speed or dwell exceeds the contract")
    evaluation = raw["evaluation"]
    for name in ("maximum_endpoint_error_m", "maximum_yaw_error_rad", "maximum_post_stop_motion_m",
                 "maximum_tilt_rad", "maximum_actual_linear_m_s", "maximum_actual_angular_rad_s"):
        _number(evaluation.get(name), name, positive=True)
    if (float(evaluation["maximum_endpoint_error_m"]) > 0.06
            or float(evaluation["maximum_yaw_error_rad"]) > 0.08
            or float(evaluation["maximum_post_stop_motion_m"]) > 0.01
            or float(evaluation["maximum_actual_linear_m_s"]) > limits.max_linear_m_s
            or float(evaluation["maximum_actual_angular_rad_s"]) > limits.max_angular_rad_s):
        raise ValueError("ground-truth docking thresholds exceed the strict evaluation contract")
    result = dict(raw)
    result["computed"] = {
        "approach_distance_m": approach_distance,
        "maximum_video_frames": maximum_frames,
        "maximum_data_rows": maximum_rows,
        "drive_geometry": asdict(geometry),
        "drive_limits": asdict(limits),
    }
    return result


class EncoderDockingController:
    """Approach, align yaw, and accept a stationary dwell from encoder odometry."""

    def __init__(self, config: Mapping[str, Any], geometry: DriveGeometry) -> None:
        self.config = config
        self.control = config["controller"]
        self.target = config["target_pose_odom"]
        self.limits = DriveLimits(**config["drive_limits"])
        self.profile = DriveProfile(geometry, self.limits)
        self.stationary_since_s: float | None = None
        self.complete = False

    def step(self, stamp_s: float, pose: EncoderSample, dt_s: float) -> dict[str, Any]:
        target_x = float(self.target["x_m"])
        target_y = float(self.target["y_m"])
        target_yaw = float(self.target["yaw_rad"])
        dx, dy = target_x-pose.x_m, target_y-pose.y_m
        position_error = math.hypot(dx, dy)
        yaw_error = wrap_angle(target_yaw-pose.yaw_rad)
        position_tolerance = float(self.control["position_tolerance_m"])
        yaw_tolerance = float(self.control["yaw_tolerance_rad"])
        phase = "complete" if self.complete else "approach"
        requested_linear = requested_angular = 0.0
        if not self.complete and position_error > position_tolerance:
            heading_error = wrap_angle(math.atan2(dy, dx)-pose.yaw_rad)
            requested_angular = max(-self.limits.max_angular_rad_s, min(
                self.limits.max_angular_rad_s, float(self.control["heading_gain"]) * heading_error))
            if abs(heading_error) <= float(self.control["heading_gate_rad"]):
                braking_distance = max(0.0, position_error-position_tolerance)
                braking_speed = math.sqrt(2.0*self.limits.linear_decel_m_s2*braking_distance)
                requested_linear = min(float(self.control["cruise_linear_m_s"]), braking_speed)
        elif not self.complete and abs(yaw_error) > yaw_tolerance:
            phase = "yaw_alignment"
            requested_angular = max(-self.limits.max_angular_rad_s, min(
                self.limits.max_angular_rad_s, float(self.control["yaw_gain"]) * yaw_error))
        elif not self.complete:
            phase = "stationary_dwell"
        command = self.profile.step(requested_linear, requested_angular, dt_s)
        stopped = (position_error <= position_tolerance and abs(yaw_error) <= yaw_tolerance
                   and abs(pose.linear_m_s) <= float(self.control["stop_linear_m_s"])
                   and abs(pose.angular_rad_s) <= float(self.control["stop_angular_rad_s"])
                   and command.linear_m_s == 0.0 and command.angular_rad_s == 0.0)
        self.stationary_since_s = (stamp_s if self.stationary_since_s is None else self.stationary_since_s) if stopped else None
        if (not self.complete and self.stationary_since_s is not None
                and stamp_s-self.stationary_since_s >= float(self.control["stationary_dwell_s"])):
            self.complete = True
            phase = "complete"
        return {
            "phase": phase,
            "control_input": "wheel_encoder_odometry_only",
            "encoder_pose": {"x_m": pose.x_m, "y_m": pose.y_m, "yaw_rad": pose.yaw_rad,
                             "linear_m_s": pose.linear_m_s, "angular_rad_s": pose.angular_rad_s},
            "target_pose_odom": {"x_m": target_x, "y_m": target_y, "yaw_rad": target_yaw},
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "stationary_since_s": self.stationary_since_s,
            "accepted_from_encoder": self.complete,
            "command": asdict(command),
        }


def evaluate_docking_truth(samples: Sequence[Mapping[str, Any]], config: Mapping[str, Any]) -> dict[str, Any]:
    """Evaluate the final pad alignment from ground truth after the run."""
    if not samples:
        raise ValueError("docking ground-truth evaluation needs samples")
    target = config["target_pose_odom"]
    final = samples[-1]["ground_truth"]
    endpoint_error = math.hypot(float(final["x_m"])-float(target["x_m"]),
                                float(final["y_m"])-float(target["y_m"]))
    yaw_error = abs(wrap_angle(float(final["yaw_rad"])-float(target["yaw_rad"])))
    stop_index = next((index for index, sample in enumerate(samples)
                       if sample.get("controller", {}).get("phase") == "complete"), len(samples)-1)
    stopped = samples[stop_index]["ground_truth"]
    post_stop_motion = max(math.hypot(float(sample["ground_truth"]["x_m"])-float(stopped["x_m"]),
                                      float(sample["ground_truth"]["y_m"])-float(stopped["y_m"]))
                           for sample in samples[stop_index:])
    maximum_tilt = max(max(abs(float(sample["ground_truth"]["roll_rad"])),
                           abs(float(sample["ground_truth"]["pitch_rad"]))) for sample in samples)
    maximum_linear = max(abs(float(sample["ground_truth"]["forward_speed_m_s"])) for sample in samples)
    maximum_angular = max(abs(float(sample["ground_truth"]["yaw_rate_rad_s"])) for sample in samples)
    metrics = {
        "endpoint_error_m": endpoint_error,
        "yaw_error_rad": yaw_error,
        "post_stop_motion_m": post_stop_motion,
        "maximum_tilt_rad": maximum_tilt,
        "maximum_actual_linear_m_s": maximum_linear,
        "maximum_actual_angular_rad_s": maximum_angular,
    }
    limits = config["evaluation"]
    checks = {
        "endpoint": endpoint_error <= float(limits["maximum_endpoint_error_m"]),
        "yaw": yaw_error <= float(limits["maximum_yaw_error_rad"]),
        "post_stop_motion": post_stop_motion <= float(limits["maximum_post_stop_motion_m"]),
        "tilt": maximum_tilt <= float(limits["maximum_tilt_rad"]),
        "actual_linear_speed": maximum_linear <= float(limits["maximum_actual_linear_m_s"]) + 1e-9,
        "actual_angular_speed": maximum_angular <= float(limits["maximum_actual_angular_rad_s"]) + 1e-9,
    }
    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics,
            "truth_usage": "independent post-run evaluation only; never controller or dwell input",
            "physical_contact_verified": False, "electrical_contact_verified": False}
