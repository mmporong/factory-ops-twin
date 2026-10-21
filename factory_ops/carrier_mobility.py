"""Loaded-coupon time commands and separate evaluation; no simulator imports.

DriveProfile is the only command shaper. Ground truth never enters MotionPlan.
All thresholds/schedules are authored component assumptions, not robot ratings.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import math
from typing import Any, Mapping

from .carrier_robot import ROBOT_PATH, inverse_rigid, transformed_box
from .drive_profile import DriveLimits, DriveProfile
from .motion import DriveGeometry, WheelOdometry


def finite(value: Any, name: str, low: float, high: float) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(name+" outside finite authored bounds")
    return float(value)


def pair(value: Any, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(name+" requires left/right readbacks")
    return [finite(v, name, -1e6, 1e6) for v in value]


def validate_motion(value: Mapping[str, Any], static_probe: Mapping[str, Any]) -> dict:
    p = deepcopy(dict(value))
    if type(p["schema_version"]) is not int or p["schema_version"] != 1:
        raise ValueError("unsupported loaded-motion schema")
    if p["geometry_source"] != "fresh_pre_step_wheel_collision_bounds":
        raise ValueError("loaded motion requires current native wheel geometry")
    limits = DriveLimits(**p["drive_limits"])
    if limits.max_linear_m_s > .25 or limits.max_angular_rad_s > .5 or limits.max_wheel_rad_s > 4.:
        raise ValueError("loaded component drive envelope too large")
    if (limits.linear_accel_m_s2 > .15 or limits.linear_decel_m_s2 > .25
            or limits.angular_accel_rad_s2 > .4 or limits.angular_decel_rad_s2 > .6):
        raise ValueError("loaded component acceleration envelope too large")
    phases = p["phases"]
    if [r["name"] for r in phases] != ["forward", "curve", "brake_before_turn", "in_place", "final_brake"]:
        raise ValueError("loaded motion requires ordered forward/curve/brake/turn/final brake")
    for row in phases:
        finite(row["duration_s"], "phase duration", .5, 3.)
        v = finite(row["linear_m_s"], "phase linear speed", 0., limits.max_linear_m_s)
        w = finite(row["angular_rad_s"], "phase angular speed", 0., limits.max_angular_rad_s)
        name = row["name"]
        if (name == "forward" and not (v > 0 and w == 0)
                or name == "curve" and not (v > 0 and w > 0)
                or name == "in_place" and not (v == 0 and w > 0)
                or "brake" in name and not (v == w == 0)):
            raise ValueError("phase velocity does not match intended direction")
    duration = math.fsum(r["duration_s"] for r in phases)
    finite(duration, "motion schedule", 8., 10.)
    finite(p["maximum_sim_time_s"], "fixture sim bound", duration+1., 20.)
    finite(p["maximum_wall_time_s"], "fixture wall bound", 30., 360.)
    finite(p["maximum_torque_n_m"], "torque observation bound", 1., 200.)
    finite(p["maximum_tilt_rad"], "tilt evaluation bound", .001, .05)
    finite(p["minimum_forward_m"], "forward evidence", .05, .2)
    finite(p["minimum_curve_yaw_rad"], "curve evidence", .05, .2)
    finite(p["minimum_turn_yaw_rad"], "turn evidence", .1, .4)
    finite(p["maximum_displacement_m"], "fixed camera/ground bound", .8, 1.5)
    if (type(p["rgb_fps"]) is not int or type(p["maximum_rgb_frames"]) is not int
            or (p["rgb_fps"], p["maximum_rgb_frames"]) not in ((20, 240), (60, 660))):
        raise ValueError("only bounded real20fps/240 or real60fps/660 frames admitted")
    dt = static_probe["physics_dt_s"]
    period = 1/(p["rgb_fps"]*dt)
    if abs(period-round(period)) > 1e-9 or period < 1:
        raise ValueError("RGB cadence must divide existing physics cadence")
    if math.ceil((duration+static_probe["contact_dwell_s"]+1)*p["rgb_fps"]) > p["maximum_rgb_frames"]:
        raise ValueError("schedule leaves insufficient real RGB budget for final settling")
    p["schedule_duration_s"] = duration
    p["rgb_period_physics_frames"] = round(period)
    return p


def native_drive_geometry(measured: Mapping[str, Any]) -> dict:
    """Fresh computed collider AABBs already collected by the original probe."""
    if measured["robot_path"] != ROBOT_PATH or measured["meters_per_unit"] != 1. or measured["up_axis"] != "Z":
        raise ValueError("native wheel geometry identity/units mismatch")
    inverse = inverse_rigid(measured["robot_world_from_local"])
    # World AABBs cannot recover oriented exact radial dimensions. The probe
    # spawns with identity root rotation; reject any other pre-step orientation.
    if any(abs(inverse[i][j]-(i == j)) > 1e-6 for i in range(3) for j in range(3)):
        raise ValueError("pre-step wheel geometry needs identity root rotation")
    unions, records = {}, []
    for side in ("left", "right"):
        selected = [r for r in measured["colliders"] if r["path"].startswith(ROBOT_PATH+"/wheel_"+side+"/")]
        if len(selected) != 3 or len({r["path"] for r in selected}) != 3:
            raise ValueError("expected three unique active colliders per wheel")
        boxes = [transformed_box(r["world_aabb_m"], inverse) for r in selected]
        low = [min(b["center_m"][i]-b["size_m"][i]/2 for b in boxes) for i in range(3)]
        high = [max(b["center_m"][i]+b["size_m"][i]/2 for b in boxes) for i in range(3)]
        size = [b-a for a, b in zip(low, high)]
        if abs(size[0]-size[2]) > 1e-5 or not 0 < size[1] < min(size[0], size[2]):
            raise ValueError("wheel collider union is not Y-axis radial")
        unions[side] = {"center_m": [(a+b)/2 for a, b in zip(low, high)], "size_m": size}
        records.extend(selected)
    a, b = unions["left"], unions["right"]
    if (any(abs(x-y) > 1e-5 for x, y in zip(a["size_m"], b["size_m"]))
            or abs(a["center_m"][0]-b["center_m"][0]) > 1e-5
            or abs(a["center_m"][2]-b["center_m"][2]) > 1e-5
            or abs(a["center_m"][1]+b["center_m"][1]) > 1e-5):
        raise ValueError("wheel geometry lacks expected left/right symmetry")
    radius = finite((a["size_m"][0]+b["size_m"][0])/4, "wheel radius", .05, .25)
    separation = finite(a["center_m"][1]-b["center_m"][1], "wheel separation", .3, .8)
    return {"wheel_radius_m": radius, "wheel_separation_m": separation,
        "source": "current pre-step ComputeExtentFromPlugins collider bounds; identity root orientation; not GT steering",
        "native_wheel_unions": unions, "native_collision_records": records}


class MotionPlan:
    """Only actual time/encoder inputs; DriveProfile is the sole limiter."""
    def __init__(self, config: Mapping[str, Any], geometry: DriveGeometry, start_s: float,
                 wheel_position_rad: list[float]):
        self.config = config
        self.start_s = finite(start_s, "motion origin", 0., 1e6)
        self.last_command_s = start_s
        self.drive = DriveProfile(geometry, DriveLimits(**config["drive_limits"]))
        self.encoder = WheelOdometry(geometry)
        self.encoder.update(start_s, *pair(wheel_position_rad, "wheel position"))

    def command(self, now_s: float, dt_s: float) -> dict:
        now = finite(now_s, "command clock", self.last_command_s, 1e6)
        elapsed = now-self.start_s
        end = 0.
        phase = {"name": "final_settle", "linear_m_s": 0., "angular_rad_s": 0.}
        for row in self.config["phases"]:
            end += row["duration_s"]
            if elapsed < end:
                phase = row
                break
        self.last_command_s = now
        target = self.drive.step(phase["linear_m_s"], phase["angular_rad_s"], dt_s)
        return {"phase": phase["name"], "elapsed_s": elapsed, "issued_world_time_s": now,
            "requested_linear_m_s": phase["linear_m_s"], "requested_angular_rad_s": phase["angular_rad_s"],
            "target": asdict(target)}

    def observe_wheels(self, stamp_s: float, positions, velocities, efforts, max_efforts) -> dict:
        positions, velocities = pair(positions, "wheel position"), pair(velocities, "wheel velocity")
        efforts, maximum = pair(efforts, "wheel measured torque"), pair(max_efforts, "native maximum effort")
        if (any(v <= 0 for v in maximum)
                or any(abs(v) > self.config["drive_limits"]["max_wheel_rad_s"] for v in velocities)
                or any(abs(v) > min(m, self.config["maximum_torque_n_m"])+1e-6 for v, m in zip(efforts, maximum))):
            raise ValueError("native wheel speed/torque observation exceeds bounds")
        return {"encoder": asdict(self.encoder.update(stamp_s, *positions)),
            "measured_wheel_velocity_rad_s": velocities, "measured_wheel_torque_n_m": efforts,
            "native_maximum_effort_n_m": maximum}


class MotionEvidence:
    """GT/contact evaluation only. It never returns actuator commands."""
    def __init__(self, config: Mapping[str, Any], static_probe: Mapping[str, Any]):
        self.config, self.static = config, static_probe
        self.phases: dict[str, dict] = {}
        self.final_since: float | None = None
        self.last_s = -1.
        self.in_place_encoder_directions_seen = False

    def observe(self, phase: str, stamp_s: float, *, position_m, yaw_rad: float,
                tilt_rad: float, encoder: Mapping[str, Any], wheel_velocity,
                load_supported: bool, stationary: bool, target_zero: bool) -> bool:
        stamp = finite(stamp_s, "evaluation clock", 0., 1e6)
        if stamp <= self.last_s: raise ValueError("evaluation clock did not progress")
        self.last_s = stamp
        if type(load_supported) is not bool or not load_supported:
            raise ValueError("loaded contact/relative-pose support lost")
        if type(stationary) is not bool or type(target_zero) is not bool:
            raise ValueError("evaluation flags must be bool")
        finite(tilt_rad, "native chassis tilt", 0., self.config["maximum_tilt_rad"])
        if len(position_m) != 3: raise ValueError("native position dimension mismatch")
        position = [finite(v, "native relative position", -2., 2.) for v in position_m]
        if math.hypot(*position[:2]) > self.config["maximum_displacement_m"]:
            raise ValueError("native chassis left fixed camera/ground evaluation bounds")
        yaw = finite(yaw_rad, "native relative yaw", -math.pi, math.pi)
        wheels = pair(wheel_velocity, "encoder wheel speed")
        record = {"world_time_s": stamp, "position_m": position, "yaw_rad": yaw,
            "encoder_x_m": finite(encoder["x_m"], "encoder x", -10., 10.),
            "encoder_yaw_rad": finite(encoder["yaw_rad"], "encoder yaw", -10., 10.)}
        if phase not in self.phases:
            if phase == "in_place" and self.phases.get("brake_before_turn", {}).get("last_stationary") is not True:
                raise ValueError("native brake before in-place turn was not stationary")
            self.phases[phase] = {"first": record, "last": record, "last_stationary": stationary}
        self.phases[phase].update(last=record, last_stationary=stationary)
        if phase == "in_place" and wheels[0] < 0 < wheels[1]:
            self.in_place_encoder_directions_seen = True
        qualified = phase == "final_settle" and stationary and target_zero
        self.final_since = (stamp if self.final_since is None else self.final_since) if qualified else None
        return self.final_since is not None and stamp-self.final_since >= self.static["contact_dwell_s"]

    def result(self) -> dict:
        metrics, failures = {}, []
        for name, field, enc_field, minimum in (
                ("forward", "forward_m", "encoder_forward_m", self.config["minimum_forward_m"]),
                ("curve", "yaw_rad", "encoder_yaw_rad", self.config["minimum_curve_yaw_rad"]),
                ("in_place", "yaw_rad", "encoder_yaw_rad", self.config["minimum_turn_yaw_rad"])):
            row = self.phases.get(name)
            if row is None:
                failures.append(name+" phase missing")
                continue
            first, last = row["first"], row["last"]
            gt = last["position_m"][0]-first["position_m"][0] if name == "forward" else last["yaw_rad"]-first["yaw_rad"]
            en = last["encoder_x_m"]-first["encoder_x_m"] if name == "forward" else last["encoder_yaw_rad"]-first["encoder_yaw_rad"]
            metrics[name] = {field: gt, enc_field: en,
                "native_xy_displacement_m": math.dist(first["position_m"][:2], last["position_m"][:2])}
            if gt < minimum or en < minimum: failures.append(name+" lacks expected-direction GT/encoder motion")
        if not self.in_place_encoder_directions_seen: failures.append("no opposite native wheel directions during in-place")
        if self.final_since is None or self.last_s-self.final_since < self.static["contact_dwell_s"]:
            failures.append("final native stationary dwell incomplete")
        return {"passed": not failures, "failures": failures, "metrics": metrics,
            "phases": self.phases, "final_stationary_dwell_s": 0. if self.final_since is None else self.last_s-self.final_since,
            "in_place_encoder_directions_seen": self.in_place_encoder_directions_seen,
            "scope": "native component evaluation; no docking, route or industrial payload qualification"}


class RgbCadence:
    """Actual physics-frame cadence; no duplication/interpolation/catch-up."""
    def __init__(self, start_frame: int, period_frames: int, maximum: int = 240):
        if (type(start_frame) is not int or start_frame < 0 or type(period_frames) is not int
                or period_frames < 1 or type(maximum) is not int or not 1 <= maximum <= 660):
            raise ValueError("invalid native RGB cadence")
        self.last_frame, self.previous_frame = start_frame, start_frame
        self.period, self.maximum, self.count = period_frames, maximum, 0

    def due(self, frame: int) -> bool:
        if type(frame) is not int or frame != self.previous_frame+1:
            raise ValueError("RGB physics frames did not advance one step")
        self.previous_frame = frame
        if frame-self.last_frame < self.period: return False
        if self.count >= self.maximum: raise ValueError("maximum real RGB frame count reached")
        self.last_frame = frame
        self.count += 1
        return True

    def validate_records(self, rows: list[dict]) -> dict:
        if len(rows) != self.count or self.count < 2:
            raise ValueError("real RGB sequence missing capture records")
        distinct = len({r["sha256"] for r in rows})
        if distinct < 2:
            raise ValueError("motion RGB stayed identical; frozen buffer cannot prove a clip")
        frames = [r["world_checkpoint_frame"] for r in rows]
        if any(b-a != self.period for a,b in zip(frames,frames[1:])):
            raise ValueError("real RGB sequence lost physics-frame cadence")
        return {"frame_count": self.count, "distinct_image_sha256": distinct,
            "first_world_frame": frames[0], "last_world_frame": frames[-1]}
