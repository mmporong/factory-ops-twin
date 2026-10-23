"""Encoder-controlled path-following baseline for a differential-drive robot.

The controller consumes wheel-odometry poses only. Simulator ground truth is
accepted by the evaluator after a run and is never exposed to ``step``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
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


@dataclass(frozen=True, slots=True)
class PathPoint:
    x_m: float
    y_m: float
    yaw_rad: float
    s_m: float


def build_reference_path(path: Mapping[str, Any]) -> tuple[PathPoint, ...]:
    """Build a straight followed by a tangent quarter arc in the odom frame."""
    straight = _number(path.get("straight_length_m"), "straight_length_m", positive=True)
    radius = _number(path.get("quarter_arc_radius_m"), "quarter_arc_radius_m", positive=True)
    spacing = _number(path.get("sample_spacing_m"), "sample_spacing_m", positive=True)
    if spacing > min(straight, radius) / 2.0:
        raise ValueError("path sample spacing is too coarse")
    points: list[PathPoint] = []
    straight_steps = max(1, math.ceil(straight / spacing))
    for index in range(straight_steps + 1):
        x = straight * index / straight_steps
        points.append(PathPoint(x, 0.0, 0.0, x))
    arc_length = math.pi * radius / 2.0
    arc_steps = max(2, math.ceil(arc_length / spacing))
    for index in range(1, arc_steps + 1):
        angle = math.pi * index / (2.0 * arc_steps)
        points.append(PathPoint(
            straight + radius * math.sin(angle),
            radius * (1.0 - math.cos(angle)),
            angle,
            straight + radius * angle,
        ))
    return tuple(points)


def _segment_projection(x: float, y: float, first: PathPoint, second: PathPoint) -> tuple[float, float, float]:
    dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
    length2 = dx * dx + dy * dy
    fraction = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((x-first.x_m)*dx + (y-first.y_m)*dy) / length2))
    px, py = first.x_m + fraction * dx, first.y_m + fraction * dy
    return first.s_m + fraction * (second.s_m-first.s_m), px, py


def _point_at_s(points: Sequence[PathPoint], target_s: float) -> PathPoint:
    target_s = max(points[0].s_m, min(points[-1].s_m, target_s))
    for first, second in zip(points, points[1:]):
        if target_s <= second.s_m:
            span = second.s_m - first.s_m
            fraction = 0.0 if span == 0.0 else (target_s-first.s_m) / span
            return PathPoint(
                first.x_m + fraction * (second.x_m-first.x_m),
                first.y_m + fraction * (second.y_m-first.y_m),
                first.yaw_rad + fraction * wrap_angle(second.yaw_rad-first.yaw_rad),
                target_s,
            )
    return points[-1]


def validate_config(raw: Mapping[str, Any]) -> dict[str, Any]:
    if raw.get("qualification") != "encoder_path_following_baseline":
        raise ValueError("wrong path-following qualification")
    if _number(raw.get("physics_dt_s"), "physics_dt_s", positive=True) != 1.0 / 60.0:
        raise ValueError("path-following physics cadence must be exactly 60 Hz")
    maximum_sim = _number(raw.get("maximum_sim_time_s"), "maximum_sim_time_s", positive=True)
    maximum_wall = _number(raw.get("maximum_wall_time_s"), "maximum_wall_time_s", positive=True)
    if maximum_sim > 120.0 or maximum_wall > 600.0:
        raise ValueError("path-following time budget exceeds the bounded contract")
    capture_period = raw.get("capture_every_physics_steps")
    if isinstance(capture_period, bool) or not isinstance(capture_period, int) or capture_period <= 0:
        raise ValueError("capture cadence must be a positive integer")
    maximum_frames = math.ceil(maximum_sim / (float(raw["physics_dt_s"]) * capture_period))
    if maximum_frames > 180:
        raise ValueError("path-following video window exceeds 180 frames")
    maximum_rows = math.ceil(maximum_sim / float(raw["physics_dt_s"]))
    if maximum_rows > 10_000:
        raise ValueError("path-following data window exceeds 10000 rows")
    limits = DriveLimits(**raw["drive_limits"])
    if limits.max_linear_m_s > 0.6 or limits.max_angular_rad_s > 1.2:
        raise ValueError("path-following commands exceed the current G2 envelope")
    geometry = DriveGeometry(**raw["drive_geometry"])
    points = build_reference_path(raw["path"])
    controller = raw["controller"]
    for name in ("lookahead_m", "cruise_linear_m_s", "yaw_gain", "final_yaw_gain",
                 "position_tolerance_m", "yaw_tolerance_rad", "stop_linear_m_s",
                 "stop_angular_rad_s", "stationary_dwell_s", "post_stop_observation_s"):
        _number(controller.get(name), name, positive=True)
    if float(controller["cruise_linear_m_s"]) > limits.max_linear_m_s:
        raise ValueError("controller cruise speed exceeds drive limits")
    evaluation = raw["evaluation"]
    for name in ("maximum_rms_cross_track_m", "maximum_cross_track_m", "maximum_endpoint_error_m",
                 "maximum_heading_error_rad", "maximum_post_stop_motion_m", "maximum_tilt_rad",
                 "maximum_actual_linear_m_s", "maximum_actual_angular_rad_s"):
        _number(evaluation.get(name), name, positive=True)
    if (float(evaluation["maximum_actual_linear_m_s"]) > limits.max_linear_m_s
            or float(evaluation["maximum_actual_angular_rad_s"]) > limits.max_angular_rad_s):
        raise ValueError("actual-speed evaluation limits cannot exceed the G2 command envelope")
    result = dict(raw)
    result["computed"] = {
        "path_length_m": points[-1].s_m,
        "path_point_count": len(points),
        "maximum_video_frames": maximum_frames,
        "maximum_data_rows": maximum_rows,
        "drive_geometry": asdict(geometry),
        "drive_limits": asdict(limits),
    }
    return result


class EncoderPathController:
    """Monotonic pure-pursuit controller with terminal braking and yaw alignment."""

    def __init__(self, config: Mapping[str, Any], geometry: DriveGeometry) -> None:
        self.config = config
        self.points = build_reference_path(config["path"])
        self.control = config["controller"]
        self.limits = DriveLimits(**config["drive_limits"])
        self.profile = DriveProfile(geometry, self.limits)
        self.progress_s_m = 0.0
        self.stationary_since_s: float | None = None
        self.complete = False

    def _progress(self, pose: EncoderSample) -> tuple[float, float]:
        best: tuple[float, float] | None = None
        for first, second in zip(self.points, self.points[1:]):
            if second.s_m + 1e-9 < self.progress_s_m:
                continue
            s_m, px, py = _segment_projection(pose.x_m, pose.y_m, first, second)
            candidate = (math.hypot(pose.x_m-px, pose.y_m-py), s_m)
            if best is None or candidate < best:
                best = candidate
        if best is None:
            return self.progress_s_m, math.inf
        self.progress_s_m = max(self.progress_s_m, best[1])
        return self.progress_s_m, best[0]

    def step(self, stamp_s: float, pose: EncoderSample, dt_s: float) -> dict[str, Any]:
        if self.complete:
            command = self.profile.step(0.0, 0.0, dt_s)
            return self._payload("complete", command, pose, self.points[-1], 0.0, 0.0)
        progress, cross_track = self._progress(pose)
        goal = self.points[-1]
        remaining = max(0.0, goal.s_m-progress)
        position_error = math.hypot(goal.x_m-pose.x_m, goal.y_m-pose.y_m)
        yaw_error = wrap_angle(goal.yaw_rad-pose.yaw_rad)
        position_tolerance = float(self.control["position_tolerance_m"])
        yaw_tolerance = float(self.control["yaw_tolerance_rad"])
        phase = "path_tracking"
        target = _point_at_s(self.points, progress + float(self.control["lookahead_m"]))
        if position_error <= position_tolerance:
            if abs(yaw_error) > yaw_tolerance:
                phase = "final_yaw"
                requested_linear = 0.0
                requested_angular = max(-self.limits.max_angular_rad_s, min(
                    self.limits.max_angular_rad_s, float(self.control["final_yaw_gain"]) * yaw_error))
            else:
                phase = "stationary_dwell"
                requested_linear = requested_angular = 0.0
        else:
            braking_distance = max(remaining, max(0.0, position_error-position_tolerance))
            braking_speed = math.sqrt(max(0.0, 2.0 * self.limits.linear_decel_m_s2 * braking_distance))
            requested_linear = min(float(self.control["cruise_linear_m_s"]), braking_speed)
            dx, dy = target.x_m-pose.x_m, target.y_m-pose.y_m
            local_x = math.cos(pose.yaw_rad)*dx + math.sin(pose.yaw_rad)*dy
            local_y = -math.sin(pose.yaw_rad)*dx + math.cos(pose.yaw_rad)*dy
            distance2 = max(dx*dx + dy*dy, 1e-9)
            curvature = 2.0 * local_y / distance2
            requested_angular = requested_linear * curvature
            heading_error = wrap_angle(math.atan2(dy, dx)-pose.yaw_rad)
            requested_angular += float(self.control["yaw_gain"]) * heading_error
            requested_angular = max(-self.limits.max_angular_rad_s,
                                    min(self.limits.max_angular_rad_s, requested_angular))
            if remaining <= max(float(self.control["lookahead_m"]), position_tolerance * 4.0):
                phase = "terminal_brake"
            if local_x < 0.0:
                requested_linear = 0.0
        command = self.profile.step(requested_linear, requested_angular, dt_s)
        stopped = (abs(pose.linear_m_s) <= float(self.control["stop_linear_m_s"])
                   and abs(pose.angular_rad_s) <= float(self.control["stop_angular_rad_s"])
                   and command.linear_m_s == 0.0 and command.angular_rad_s == 0.0
                   and position_error <= position_tolerance and abs(yaw_error) <= yaw_tolerance)
        self.stationary_since_s = (stamp_s if self.stationary_since_s is None else self.stationary_since_s) if stopped else None
        if self.stationary_since_s is not None and stamp_s-self.stationary_since_s >= float(self.control["stationary_dwell_s"]):
            self.complete = True
            phase = "complete"
        payload = self._payload(phase, command, pose, target, remaining, cross_track)
        payload["goal"] = {
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "stationary_since_s": self.stationary_since_s,
            "accepted_from_encoder": self.complete,
        }
        return payload

    def _payload(self, phase: str, command: DriveCommand, pose: EncoderSample,
                 target: PathPoint, remaining: float, cross_track: float) -> dict[str, Any]:
        return {
            "phase": phase,
            "control_input": "wheel_encoder_odometry_only",
            "progress_s_m": self.progress_s_m,
            "remaining_s_m": remaining,
            "encoder_cross_track_m": cross_track,
            "lookahead": asdict(target),
            "encoder_pose": {"x_m": pose.x_m, "y_m": pose.y_m, "yaw_rad": pose.yaw_rad},
            "command": asdict(command),
        }


def evaluate_ground_truth(samples: Sequence[Mapping[str, Any]], config: Mapping[str, Any]) -> dict[str, Any]:
    """Evaluate GT independently after encoder-only goal acceptance."""
    if not samples:
        raise ValueError("ground-truth evaluation needs samples")
    points = build_reference_path(config["path"])
    errors: list[float] = []
    heading_errors: list[float] = []
    for sample in samples:
        truth = sample["ground_truth"]
        x, y, yaw = (float(truth[name]) for name in ("x_m", "y_m", "yaw_rad"))
        nearest = min(
            (_segment_projection(x, y, first, second) for first, second in zip(points, points[1:])),
            key=lambda item: math.hypot(x-item[1], y-item[2]),
        )
        errors.append(math.hypot(x-nearest[1], y-nearest[2]))
        reference = _point_at_s(points, nearest[0])
        heading_errors.append(abs(wrap_angle(yaw-reference.yaw_rad)))
    goal = points[-1]
    final = samples[-1]["ground_truth"]
    endpoint_error = math.hypot(float(final["x_m"])-goal.x_m, float(final["y_m"])-goal.y_m)
    stop_index = next((index for index, sample in enumerate(samples)
                       if sample.get("controller", {}).get("phase") == "complete"), len(samples)-1)
    stopped_truth = samples[stop_index]["ground_truth"]
    post_stop_motion = max(
        math.hypot(float(sample["ground_truth"]["x_m"])-float(stopped_truth["x_m"]),
                   float(sample["ground_truth"]["y_m"])-float(stopped_truth["y_m"]))
        for sample in samples[stop_index:]
    )
    maximum_tilt = max(max(abs(float(sample["ground_truth"]["roll_rad"])),
                           abs(float(sample["ground_truth"]["pitch_rad"]))) for sample in samples)
    maximum_linear_speed = max(abs(float(sample["ground_truth"]["forward_speed_m_s"])) for sample in samples)
    maximum_angular_speed = max(abs(float(sample["ground_truth"]["yaw_rate_rad_s"])) for sample in samples)
    metrics = {
        "rms_cross_track_m": math.sqrt(sum(value*value for value in errors) / len(errors)),
        "maximum_cross_track_m": max(errors),
        "maximum_heading_error_rad": max(heading_errors),
        "endpoint_error_m": endpoint_error,
        "post_stop_motion_m": post_stop_motion,
        "maximum_tilt_rad": maximum_tilt,
        "maximum_actual_linear_m_s": maximum_linear_speed,
        "maximum_actual_angular_rad_s": maximum_angular_speed,
    }
    thresholds = config["evaluation"]
    checks = {
        "rms_cross_track": metrics["rms_cross_track_m"] <= float(thresholds["maximum_rms_cross_track_m"]),
        "maximum_cross_track": metrics["maximum_cross_track_m"] <= float(thresholds["maximum_cross_track_m"]),
        "heading": metrics["maximum_heading_error_rad"] <= float(thresholds["maximum_heading_error_rad"]),
        "endpoint": endpoint_error <= float(thresholds["maximum_endpoint_error_m"]),
        "post_stop_motion": post_stop_motion <= float(thresholds["maximum_post_stop_motion_m"]),
        "tilt": maximum_tilt <= float(thresholds["maximum_tilt_rad"]),
        "actual_linear_speed": maximum_linear_speed <= float(thresholds["maximum_actual_linear_m_s"]) + 1e-9,
        "actual_angular_speed": maximum_angular_speed <= float(thresholds["maximum_actual_angular_rad_s"]) + 1e-9,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": metrics,
        "truth_usage": "independent evaluation only; never controller or goal-acceptance input",
    }
