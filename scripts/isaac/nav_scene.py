#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""G1 Isaac worker: bridge one encoder-driven Nova Carter to ROS 2/Nav2."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import traceback
from typing import Any, Mapping, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.compat import replace_shared_file
from factory_ops.drive_profile import DriveCommand, DriveLimits, DriveProfile
from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout
from factory_ops.motion import DriveGeometry, VelocityWatchdog, WheelOdometry
from scripts.isaac import stage_smoke as base
from scripts.isaac.robot_qualification import (
    CameraSpec,
    CaptureSession,
    ROBOT_PATH,
    WHEEL_JOINTS,
    _capture_factory_overview,
    _clock_graph,
    _collision_bounds,
)

RESULT_FILE = "g1-scene.json"
READY_FILE = "sim-ready.json"
CONTROL_FILE = "mission-control.json"
TRAJECTORY_FILE = "trajectory.jsonl"
MEDIA_INDEX_FILE = "media/frames.json"
WARMUP_STEPS = 120
SETTLE_STEPS = 30
DRIVE_LIMIT_FIELDS = (
    "max_linear_m_s", "max_angular_rad_s", "linear_accel_m_s2", "linear_decel_m_s2",
    "angular_accel_rad_s2", "angular_decel_rad_s2", "lateral_accel_m_s2",
    "max_wheel_rad_s", "wheel_accel_rad_s2",
)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--layout", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    return parser.parse_args()


def _canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def _atomic_json(path: Path, value: Any) -> None:
    """Publish a complete JSON document without exposing a partial write."""
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
    data = json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with temporary.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    replace_shared_file(temporary, path)


def _close_app(app: Any, code: int, output: Path | None, result: dict[str, Any]) -> int:
    try:
        app.close(wait_for_replicator=False, exit_code=code)
    except BaseException as error:
        result.update({"status": "failed", "cleanup": "app_close_failed",
                       "close_error": f"{type(error).__name__}: {error}"})
        if output is not None:
            _atomic_json(output / RESULT_FILE, result)
        return 1
    return code


def _load_config(path: Path) -> dict[str, Any]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("qualification") != "g1_single_order":
        raise base.SmokeError("G1 config has the wrong qualification")
    required_positive = (
        "physics_dt_s", "encoder_publish_every_steps", "camera_capture_every_steps",
        "max_sim_time_s", "max_wall_time_s", "command_timeout_s",
        "max_linear_m_s", "max_angular_rad_s", "max_linear_accel_m_s2", "max_angular_accel_rad_s2",
    )
    for name in required_positive:
        value = raw.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise base.SmokeError(f"G1 config field must be finite and positive: {name}")
    if abs(float(raw["physics_dt_s"]) - 1.0 / 60.0) > 1e-12:
        raise base.SmokeError("G1 physics_dt_s must be fixed at 1/60 second")
    geometry = raw.get("drive_geometry")
    lidar = raw.get("lidar")
    if not isinstance(geometry, dict) or not isinstance(geometry.get("provenance"), dict):
        raise base.SmokeError("G1 drive geometry provenance is missing")
    if not isinstance(raw.get("control_limit_provenance"), dict):
        raise base.SmokeError("G1 control limit provenance is missing")
    if not isinstance(lidar, dict) or lidar.get("topic") != "/scan" or lidar.get("frame_id") != "base_scan":
        raise base.SmokeError("G1 lidar contract must publish /scan in base_scan")
    DriveGeometry(float(geometry["wheel_radius_m"]), float(geometry["wheel_separation_m"]))
    limits = _drive_limits(raw)
    legacy_pairs = (
        ("max_linear_m_s", "max_linear_m_s"),
        ("max_angular_rad_s", "max_angular_rad_s"),
        ("max_linear_accel_m_s2", "linear_accel_m_s2"),
        ("max_angular_accel_rad_s2", "angular_accel_rad_s2"),
    )
    for legacy_name, profile_name in legacy_pairs:
        if float(raw[legacy_name]) != getattr(limits, profile_name):
            raise base.SmokeError(
                f"G1 legacy control limit must equal drive_profile.{profile_name}: {legacy_name}"
            )
    return raw


def _drive_limits(config: Mapping[str, Any]) -> DriveLimits:
    profile = config.get("drive_profile")
    if not isinstance(profile, Mapping) or not isinstance(profile.get("provenance"), Mapping):
        raise base.SmokeError("G1 drive profile provenance is missing")
    if profile["provenance"].get("kind") != "authored-not-motor-calibrated":
        raise base.SmokeError("G1 drive profile provenance kind must be authored-not-motor-calibrated")
    values: dict[str, float] = {}
    for name in DRIVE_LIMIT_FIELDS:
        value = profile.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise base.SmokeError(f"G1 drive profile field must be finite and positive: {name}")
        values[name] = float(value)
    try:
        return DriveLimits(**values)
    except ValueError as error:
        raise base.SmokeError(f"G1 drive profile is invalid: {error}") from error


def _settle_step_limit(config: Mapping[str, Any]) -> int:
    limits = _drive_limits(config)
    dt_s = float(config["physics_dt_s"])
    longest_stop_s = max(
        limits.max_linear_m_s / limits.linear_decel_m_s2,
        limits.max_angular_rad_s / limits.angular_decel_rad_s2,
        limits.max_wheel_rad_s / limits.wheel_accel_rad_s2,
    )
    return math.ceil(longest_stop_s / dt_s) + SETTLE_STEPS + 2


def _motion_profile_payload(
    command: DriveCommand, *, input_linear_m_s: float, input_angular_rad_s: float,
    stop_reason: str,
) -> dict[str, Any]:
    return {
        "phase": command.phase,
        "curvature_limited": command.curvature_limited,
        "wheel_speed_limited": command.wheel_speed_limited,
        "emergency_stop": command.emergency_stop,
        "input_command": {"linear_m_s": input_linear_m_s, "angular_rad_s": input_angular_rad_s},
        "stop_reason": stop_reason,
    }


def _stop_mode(*, mission_settling: bool, watchdog_active: bool, mission_failed: bool) -> tuple[str, bool]:
    if mission_failed:
        return "mission_failed", True
    if not mission_settling and not watchdog_active:
        return "watchdog_stale", True
    if mission_settling:
        return "normal_mission_complete", False
    return "none", False


def _apply_cleanup_fault_stop(
    *, profile: Any, robot: Any, physics_dt_s: float | None,
    array_module: Any, failure_phase: str, failure_error: BaseException,
) -> tuple[dict[str, Any], list[str]]:
    """Best-effort zero targets after an exceptional worker exit."""
    response: dict[str, Any] = {
        "stop_reason": "exception_cleanup",
        "failure_phase": failure_phase,
        "error": {"type": type(failure_error).__name__, "message": str(failure_error)},
        "profile_emergency_stop": False,
        "robot_available": robot is not None,
        "applied_zero": False,
    }
    errors: list[str] = []
    targets: tuple[float, float] = (0.0, 0.0)
    if profile is not None and physics_dt_s is not None:
        try:
            command = profile.step(0.0, 0.0, physics_dt_s, emergency_stop=True)
            response["profile_emergency_stop"] = command.emergency_stop
            response["motion_profile"] = _motion_profile_payload(
                command, input_linear_m_s=0.0, input_angular_rad_s=0.0,
                stop_reason="exception_cleanup",
            )
            if command.wheel_targets_rad_s != targets:
                errors.append("cleanup fault profile returned nonzero wheel targets")
        except BaseException as error:
            errors.append(f"cleanup fault profile: {type(error).__name__}: {error}")
    if robot is not None:
        try:
            wheel_targets = (
                array_module.asarray(targets, dtype=array_module.float32)
                if array_module is not None else targets
            )
            robot.apply_wheel_actions(wheel_targets)
            response["applied_zero"] = all(value == 0.0 for value in targets)
            response["wheel_target_rad_s"] = list(targets)
        except BaseException as error:
            errors.append(f"cleanup zero wheel targets: {type(error).__name__}: {error}")
    return response, errors


def _read_control(path: Path, run_id: str) -> dict[str, Any] | None:
    if not path.exists():
        return None
    if path.is_symlink() or not path.is_file():
        raise base.SmokeError("mission control must be a regular file")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("run_id") != run_id:
        raise base.SmokeError("mission control run ID does not match this simulation")
    if value.get("status") not in {"passed", "failed", "stop"}:
        raise base.SmokeError("mission control status is invalid")
    return value


def _ready_payload(
    *, run_id: str, asset: str, footprint: Mapping[str, Any], config: Mapping[str, Any],
    lidar_metadata: Mapping[str, Any], source_sha256: str, sim_time_s: float = 0.0,
) -> dict[str, Any]:
    limits = _drive_limits(config)
    profile_config = config["drive_profile"]
    ros2 = {
        "domain_id": int(config["ros_domain_id"]),
        "clock": "/clock", "cmd_vel": "/cmd_vel", "odom": "/odom",
        "tf": "/tf", "tf_static": "/tf_static", "scan": "/scan",
        "frames": {"odom": "odom", "base": "base_link", "scan": "base_scan"},
    }
    return {
        "schema_version": 1,
        "run_id": run_id,
        "status": "ready",
        "sim_time": sim_time_s,
        "footprint": dict(footprint),
        "ros2": ros2,
        "robot": {"asset_url": asset, "prim_path": ROBOT_PATH, "footprint": dict(footprint)},
        "ros": ros2,
        "readers": {
            "odometry": "measured wheel joint angles integrated in local odom; zero at initialization",
            "laser_scan": "Isaac Sim RTX lidar ROS 2 writer",
            "simulator_truth": "JSONL evaluation and media only; unavailable to the ROS control boundary",
        },
        "lidar": dict(lidar_metadata),
        "drive_geometry": dict(config["drive_geometry"]),
        "control_boundary": {
            "input": "ROS 2 geometry_msgs/Twist on /cmd_vel",
            "watchdog_sim_seconds": float(config["command_timeout_s"]),
            "max_linear_accel_m_s2": float(config["max_linear_accel_m_s2"]),
            "max_angular_accel_rad_s2": float(config["max_angular_accel_rad_s2"]),
            "limit_provenance": dict(config["control_limit_provenance"]),
            "drive_profile": {
                "limits": {name: getattr(limits, name) for name in DRIVE_LIMIT_FIELDS},
                "provenance": dict(profile_config["provenance"]),
                "owner": "Isaac physics loop at physics_dt_s; no second normal-motion limiter",
                "normal_stop": f"ramp to zero, then hold zero for {SETTLE_STEPS} physics steps",
                "fault_stop": "immediate zero wheel targets for watchdog, mission failure, or tilt fault",
            },
            "output": "Nova Carter wheel velocity targets",
            "completion": "external mission-control.json after Nav2 action results and ledger commits",
        },
        "source_sha256": source_sha256,
    }


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _wheel_angles(robot: Any) -> tuple[float, float]:
    names = list(robot.dof_names)
    try:
        indices = [names.index(name) for name in WHEEL_JOINTS]
    except ValueError as error:
        raise base.SmokeError(f"wheel joints missing from articulation: {names}") from error
    values = robot.get_dof_positions()
    array = values.numpy() if hasattr(values, "numpy") else values
    if getattr(array, "ndim", 1) == 2:
        array = array[0]
    return float(array[indices[0]]), float(array[indices[1]])


def _joint_positions(robot: Any) -> list[float]:
    values = robot.get_dof_positions()
    array = values.numpy() if hasattr(values, "numpy") else values
    if getattr(array, "ndim", 1) == 2:
        array = array[0]
    return [float(value) for value in array]


def _drive_geometry_from_colliders(
    colliders: Sequence[Mapping[str, Any]], *, robot_path: str = ROBOT_PATH,
) -> dict[str, Any]:
    """Measure Carter's differential-drive geometry in the robot root frame."""
    wheel_records: dict[str, list[Mapping[str, Any]]] = {"left": [], "right": []}
    for record in colliders:
        path = str(record.get("path", ""))
        for side in wheel_records:
            if path.startswith(f"{robot_path}/wheel_{side}/"):
                wheel_records[side].append(record)
                break
    if any(len(records) != 3 for records in wheel_records.values()):
        raise base.SmokeError(
            "expected exactly three enabled collision geometries for each Nova Carter drive wheel"
        )

    unions: dict[str, list[list[float]]] = {}
    dimensions: dict[str, list[float]] = {}
    centers: dict[str, list[float]] = {}
    radii: dict[str, float] = {}
    for side, records in wheel_records.items():
        try:
            minimum = [min(float(record["min"][axis]) for record in records) for axis in range(3)]
            maximum = [max(float(record["max"][axis]) for record in records) for axis in range(3)]
        except (KeyError, TypeError, ValueError, IndexError) as error:
            raise base.SmokeError(f"invalid {side} wheel collider bounds") from error
        if not all(math.isfinite(value) for point in (minimum, maximum) for value in point):
            raise base.SmokeError(f"non-finite {side} wheel collider bounds")
        size = [maximum[axis] - minimum[axis] for axis in range(3)]
        if any(value <= 0 for value in size):
            raise base.SmokeError(f"non-positive {side} wheel collider union")
        if abs(size[0] - size[2]) > 1e-5 or size[1] >= min(size[0], size[2]):
            raise base.SmokeError(f"{side} wheel collider union is not a Y-axis radial wheel")
        unions[side] = [minimum, maximum]
        dimensions[side] = size
        centers[side] = [(minimum[axis] + maximum[axis]) / 2.0 for axis in range(3)]
        radii[side] = max(size[0], size[2]) / 2.0

    if any(abs(left - right) > 1e-5 for left, right in zip(dimensions["left"], dimensions["right"])):
        raise base.SmokeError("left and right wheel collider unions have different dimensions")
    if abs(radii["left"] - radii["right"]) > 1e-5:
        raise base.SmokeError("left and right wheel radii differ")
    if abs(centers["left"][0] - centers["right"][0]) > 1e-5 or abs(
        centers["left"][2] - centers["right"][2]
    ) > 1e-5 or abs(centers["left"][1] + centers["right"][1]) > 1e-5:
        raise base.SmokeError("left and right wheel collider unions are not root-frame symmetric")
    separation = abs(centers["left"][1] - centers["right"][1])
    geometry = DriveGeometry((radii["left"] + radii["right"]) / 2.0, separation)
    return {
        "wheel_radius_m": geometry.wheel_radius_m,
        "wheel_separation_m": geometry.wheel_separation_m,
        "provenance": {
            "kind": "fresh_asset_collision_geometry_measurement",
            "method": (
                "union enabled wheel collision geometry in the Carter root frame; wheel axis is root Y; "
                "radius is max(root-X diameter, root-Z diameter)/2; separation is absolute Y-center distance"
            ),
            "extent_source": "ComputeExtentFromPlugins and root-relative XformCache transform",
            "wheel_paths": {
                side: sorted(str(record["path"]) for record in records)
                for side, records in wheel_records.items()
            },
            "wheel_union_bounds_xyz": unions,
            "wheel_union_dimensions_xyz_m": dimensions,
            "wheel_centers_xyz_m": centers,
            "side_radii_m": radii,
        },
    }


def _apply_measured_drive_geometry(
    config: dict[str, Any], colliders: Sequence[Mapping[str, Any]], *, robot_path: str = ROBOT_PATH,
) -> dict[str, Any]:
    measured = _drive_geometry_from_colliders(colliders, robot_path=robot_path)
    expected = config["drive_geometry"]
    for field in ("wheel_radius_m", "wheel_separation_m"):
        if abs(float(expected[field]) - float(measured[field])) > 1e-4:
            raise base.SmokeError(
                f"configured expected {field} differs from fresh asset collision geometry: "
                f"{expected[field]} != {measured[field]}"
            )
    config["drive_geometry"] = measured
    return measured


def _roll_pitch_from_wxyz(orientation_wxyz: Sequence[float]) -> tuple[float, float]:
    if len(orientation_wxyz) != 4 or not all(math.isfinite(float(value)) for value in orientation_wxyz):
        raise base.SmokeError("ground-truth orientation is not a finite quaternion")
    w, x, y, z = (float(value) for value in orientation_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if abs(norm - 1.0) > 1e-3:
        raise base.SmokeError("ground-truth orientation quaternion is not unit length")
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(pitch_term)


def _tilt_evaluation(orientation_wxyz: Sequence[float], *, threshold_rad: float = 0.35) -> dict[str, Any]:
    roll, pitch = _roll_pitch_from_wxyz(orientation_wxyz)
    return {"roll_rad": roll, "pitch_rad": pitch, "threshold_rad": threshold_rad,
            "passed": abs(roll) <= threshold_rad and abs(pitch) <= threshold_rad,
            "use": "simulator-truth run validity evaluation only; never navigation or steering"}


def _pose(robot: Any) -> tuple[list[float], list[float]]:
    positions, orientations = robot.get_world_poses()
    p = positions.numpy() if hasattr(positions, "numpy") else positions
    q = orientations.numpy() if hasattr(orientations, "numpy") else orientations
    return [float(v) for v in p[0]], [float(v) for v in q[0]]


def _create_robot(
    stage_utils: Any, stage: Any, app: Any, WheeledRobot: Any, UsdGeom: Any, Gf: Any,
    asset: str, spawn_xy: tuple[float, float],
) -> Any:
    stage_utils.add_reference_to_stage(usd_path=asset, path=ROBOT_PATH)
    base._wait_for_stage(app, stage_utils)
    root = stage.GetPrimAtPath(ROBOT_PATH)
    if not root.IsValid():
        raise base.SmokeError("Nova Carter reference did not produce a valid root prim")
    UsdGeom.Xformable(root).AddTranslateOp(opSuffix="g1Spawn").Set(
        Gf.Vec3d(spawn_xy[0], spawn_xy[1], 0.1)
    )
    return WheeledRobot(ROBOT_PATH, wheel_dof_names=WHEEL_JOINTS)


def _append_jsonl(path: Path, value: Any) -> None:
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(_canonical_bytes(value).decode())
        stream.flush()


def _scene_owned_artifacts(output: Path) -> list[Path]:
    fixed = [output / READY_FILE, output / TRAJECTORY_FILE, output / "factory-overview.png"]
    nested = [
        path for root in (output / "diagnostic-load", output / "media") if root.is_dir()
        for path in root.rglob("*") if path.is_file()
    ]
    return sorted(path for path in [*fixed, *nested] if path.is_file())


def _capture_frame(
    *, output: Path, session: CaptureSession, rep: Any, np: Any, Gf: Any,
    UsdGeom: Any, SimulationManager: Any, robot: Any, sample: Mapping[str, Any], index: int,
) -> dict[str, Any]:
    from PIL import Image

    before_time = float(SimulationManager.get_simulation_time())
    before_pose = _pose(robot)
    relative = f"media/frame-{index:05d}.png"
    path = output / relative
    image = Image.fromarray(np.asarray(session.medium_rgb.get_data())[:, :, :3].astype(np.uint8))
    try:
        image.save(path)
    finally:
        image.close()
    width, height = base._png_dimensions(path)
    overview_relative = f"media/overview-{index:05d}.png"
    overview_path = output / overview_relative
    overview = Image.fromarray(np.asarray(session.overview_rgb.get_data())[:, :, :3].astype(np.uint8))
    try:
        overview.save(overview_path)
    finally:
        overview.close()
    if abs(float(SimulationManager.get_simulation_time()) - before_time) > 1e-8 or _pose(robot) != before_pose:
        raise base.SmokeError("G1 camera capture advanced simulation state")
    overview_width, overview_height = base._png_dimensions(overview_path)
    return {
        "index": index, "path": relative, "sha256": base._sha256(path), "bytes": path.stat().st_size,
        "width": width, "height": height, "physics_frame": sample["physics_frame"],
        "sim_time_s": sample["sim_time_s"], "ground_truth": sample["ground_truth"],
        "overview_path": overview_relative, "overview_sha256": base._sha256(overview_path),
        "overview_bytes": overview_path.stat().st_size,
        "overview_width": overview_width, "overview_height": overview_height,
        "camera": {
            "prim_path": session.medium_camera.path,
            "parent_frame": session.medium_camera.frame,
            "local_eye_xyz_m": list(session.medium_camera.eye_xyz),
            "local_target_xyz_m": list(session.medium_camera.target_xyz),
            "extrinsic_source": "authored relative to Carter chassis before physics start",
        },
        "source": "actual persistent Isaac Sim RGB render product; no interpolation",
    }


def _setup_ros(node: Any, config: Mapping[str, Any], geometry: DriveGeometry, watchdog: VelocityWatchdog) -> dict[str, Any]:
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    state: dict[str, Any] = {"cmd_vel": 0, "cmd_rejected": 0, "odom": 0, "tf": 0, "tf_static": 0,
                             "scan": 0, "scan_samples": [], "latest_sim_time": 0.0}

    def command(message: Any) -> None:
        try:
            watchdog.receive(float(state["latest_sim_time"]), float(message.linear.x), float(message.angular.z))
            state["cmd_vel"] += 1
        except (TypeError, ValueError):
            state["cmd_rejected"] += 1

    def scan(message: Any) -> None:
        state["scan"] += 1
        if len(state["scan_samples"]) < 5:
            finite = [float(value) for value in message.ranges if math.isfinite(value)]
            state["scan_samples"].append({
                "stamp_s": float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1e9,
                "frame_id": message.header.frame_id, "range_count": len(message.ranges),
                "finite_count": len(finite), "minimum_finite_m": min(finite) if finite else None,
                "maximum_finite_m": max(finite) if finite else None,
            })

    node._g1_cmd_subscription = node.create_subscription(Twist, "/cmd_vel", command, 10)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node._g1_scan_subscription = node.create_subscription(LaserScan, "/scan", scan, qos)
    odom_publisher = node.create_publisher(Odometry, "/odom", 10)
    tf_broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)

    translation = [float(v) for v in config["lidar"]["translation_xyz_m"]]
    static = TransformStamped()
    static.header.stamp = node.get_clock().now().to_msg()
    static.header.frame_id = "base_link"
    static.child_frame_id = str(config["lidar"]["frame_id"])
    static.transform.translation.x, static.transform.translation.y, static.transform.translation.z = translation
    static.transform.rotation.w = 1.0
    static_broadcaster.sendTransform(static)
    state["tf_static"] = 1

    def publish_odom(sample: Any) -> None:
        sec = int(sample.stamp_s)
        nanosec = int(round((sample.stamp_s - sec) * 1_000_000_000))
        if nanosec == 1_000_000_000:
            sec, nanosec = sec + 1, 0
        odom = Odometry()
        odom.header.stamp.sec, odom.header.stamp.nanosec = sec, nanosec
        odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = sample.x_m, sample.y_m
        qx, qy, qz, qw = _quaternion_from_yaw(sample.yaw_rad)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = sample.linear_m_s, sample.angular_rad_s
        odom_publisher.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, "base_link"
        transform.transform.translation.x, transform.transform.translation.y = sample.x_m, sample.y_m
        transform.transform.rotation = odom.pose.pose.orientation
        tf_broadcaster.sendTransform(transform)
        state["odom"] += 1
        state["tf"] += 1

    state["publish_odom"] = publish_odom
    state["geometry"] = {"wheel_radius_m": geometry.wheel_radius_m, "wheel_separation_m": geometry.wheel_separation_m}
    state["resources"] = [odom_publisher, tf_broadcaster, static_broadcaster]
    return state


def _setup_lidar(config: Mapping[str, Any], SimulationManager: Any, app: Any) -> tuple[Any, dict[str, Any]]:
    import isaacsim.core.experimental.utils.prim as prim_utils
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    lidar_config = config["lidar"]
    lidar = Lidar.create(
        path=ROBOT_PATH + "/chassis_link/lidar_2d",
        config=str(lidar_config["profile"]),
        tick_rate=10.0,
        translations=[lidar_config["translation_xyz_m"]],
    )
    SimulationManager.setup_simulation(dt=float(config["physics_dt_s"]), device="cpu")
    app.update()
    sensor = LidarSensor(lidar, annotators=[])
    prim = prim_utils.get_prim_at_path(lidar.paths[0])
    rotation_frequency = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_frequency = float(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_frequency <= 0 or firing_frequency <= 0:
        raise base.SmokeError("RTX lidar scan or firing frequency is not positive")
    horizontal_resolution = 360.0 * rotation_frequency / firing_frequency
    sensor.attach_writer(
        "RtxLidarROS2PublishLaserScan", topicName=str(lidar_config["topic"]).lstrip("/"),
        frameId=str(lidar_config["frame_id"]), horizontalFov=360.0,
        horizontalResolution=horizontal_resolution, depthRange=[near_range, far_range],
        rotationRate=rotation_frequency, azimuthRange=[-180.0, 180.0],
    )
    metadata = {
        "profile": lidar_config["profile"], "prim_path": ROBOT_PATH + "/chassis_link/lidar_2d",
        "topic": lidar_config["topic"], "frame_id": lidar_config["frame_id"],
        "translation_xyz_m": lidar_config["translation_xyz_m"], "tick_rate_hz": 10.0,
        "firing_frequency_hz": firing_frequency, "rotation_frequency_hz": rotation_frequency,
        "near_range_m": near_range, "far_range_m": far_range,
        "horizontal_resolution_deg": horizontal_resolution,
        "provenance": "Isaac Sim 6.2 RTX lidar attributes supplied to RtxLidarROS2PublishLaserScan",
    }
    return sensor, metadata


def _verify_lidar_transform(stage: Any, Usd: Any, UsdGeom: Any, configured_xyz: list[float]) -> dict[str, Any]:
    lidar_prim = stage.GetPrimAtPath(ROBOT_PATH + "/chassis_link/lidar_2d")
    chassis_prim = stage.GetPrimAtPath(ROBOT_PATH + "/chassis_link")
    if not lidar_prim.IsValid() or not chassis_prim.IsValid():
        raise base.SmokeError("RTX lidar or Carter chassis prim is missing")
    matrix, resets = UsdGeom.XformCache(Usd.TimeCode.Default()).ComputeRelativeTransform(lidar_prim, chassis_prim)
    if resets:
        raise base.SmokeError("RTX lidar resets the chassis transform stack")
    translation = [float(value) for value in matrix.ExtractTranslation()]
    rotation = matrix.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    quaternion_xyzw = [float(imaginary[0]), float(imaginary[1]), float(imaginary[2]), float(rotation.GetReal())]
    if any(abs(actual - expected) > 1e-5 for actual, expected in zip(translation, configured_xyz)):
        raise base.SmokeError(f"RTX lidar USD translation differs from config: {translation} != {configured_xyz}")
    if any(abs(value) > 1e-5 for value in quaternion_xyzw[:3]) or abs(abs(quaternion_xyzw[3]) - 1.0) > 1e-5:
        raise base.SmokeError(f"RTX lidar local rotation is not identity: {quaternion_xyzw}")
    return {"parent_frame": "base_link", "child_frame": "base_scan",
            "translation_xyz_m": translation, "rotation_xyzw": quaternion_xyzw,
            "source": "USD local transform relative to Nova Carter chassis before physics start"}


def _run(args: argparse.Namespace) -> int:
    started = time.monotonic()
    app = rclpy = node = session = lidar_sensor = robot = profile = np = None
    physics_dt_s: float | None = None
    failure_error: BaseException | None = None
    rep: Any = None
    output: Path | None = None
    phase = "preflight"
    valid = False
    result: dict[str, Any] = {"schema_version": 1, "run_id": args.run_id, "qualification": "g1_single_order",
                              "status": "running", "stage": phase, "cleanup": "pending"}
    try:
        if not re.fullmatch(r"[0-9a-f]{32}", args.run_id):
            raise base.SmokeError("--run-id must be 32 lowercase hexadecimal characters")
        scene, output = base._prepare_worker_paths(args.scene, args.output)
        layout_path = args.layout.resolve(strict=True)
        config_path = args.config.resolve(strict=True)
        if not layout_path.is_file() or not config_path.is_file():
            raise base.SmokeError("layout and config must be existing files")
        layout = Layout.from_file(layout_path)
        config = _load_config(config_path)
        physics_dt_s = float(config["physics_dt_s"])
        os.environ["ROS_DOMAIN_ID"] = str(int(config["ros_domain_id"]))
        os.environ.setdefault("ROS_DISTRO", "jazzy")
        source_path = Path(__file__).resolve()
        result["inputs"] = {
            "scene": {"path": str(scene), "sha256": base._sha256(scene)},
            "layout": {"path": str(layout_path), "sha256": base._sha256(layout_path)},
            "config": {"path": str(config_path), "sha256": base._sha256(config_path)},
            "source": {"path": str(source_path), "sha256": base._sha256(source_path)},
        }

        def checkpoint(name: str) -> None:
            nonlocal phase
            phase = name
            result["stage"] = name
            result["elapsed_wall_s"] = time.monotonic() - started
            _atomic_json(output / RESULT_FILE, result)

        checkpoint("simulation_app")
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RealTimePathTracing", "sync_loads": True, "fast_shutdown": True})
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.timeline
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
        from isaacsim.storage.native import get_assets_root_path

        checkpoint("open_factory")
        opened, stage = stage_utils.open_stage(str(scene))
        if not opened or stage is None:
            raise base.SmokeError("could not open authored factory")
        app.reset_render_settings()
        stage.SetTimeCodesPerSecond(60.0)
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(36000.0)
        base._wait_for_stage(app, stage_utils)
        observed, mismatches = base._inspect_stage(stage, UsdGeom, UsdPhysics)
        result["observed_factory"] = observed
        if mismatches:
            raise base.SmokeError(f"factory invariants failed: {mismatches}")

        checkpoint("load_robot")
        asset = get_assets_root_path() + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        spawn_x, spawn_y = layout.stations[str(config["source_station"])]
        robot = _create_robot(
            stage_utils, stage, app, WheeledRobot, UsdGeom, Gf, asset, (spawn_x, spawn_y),
        )

        def save_rgb(data: Any, path: Path) -> None:
            image = Image.fromarray(np.asarray(data)[:, :, :3].astype(np.uint8))
            try:
                image.save(path)
            finally:
                image.close()

        session = CaptureSession(
            output, app, stage, rep, UsdGeom, Gf, SimulationManager, save_rgb,
            medium_camera=CameraSpec(
                ROBOT_PATH + "/chassis_link/G1EvidenceCamera",
                (-3.0, -4.0, 3.0), (0.0, 0.0, 0.25), "base_link",
            ),
        )
        result["load_preview"] = session.capture_load_preview()
        result["factory_overview"] = _capture_factory_overview(output, session)
        bounds, colliders = _collision_bounds(stage, Usd, UsdGeom, UsdPhysics)
        footprint = qualify_footprint(layout, bounds)
        result["robot"] = {
            "asset_url": asset, "prim_path": ROBOT_PATH, "wheel_dof_names": WHEEL_JOINTS,
            "collider_count": len(colliders), "collision_bounds_xyz": bounds,
            "collision_geometry_records": colliders, "footprint": footprint,
        }
        if not footprint["passed"]:
            raise base.SmokeError("Nova Carter collision footprint does not fit factory anchors")
        measured_drive_geometry = _apply_measured_drive_geometry(config, colliders)
        measured_drive_geometry["provenance"].update({
            "measured_at": datetime.now(timezone.utc).isoformat(),
            "unit": "m",
            "measurement_context": (
                f"{ROBOT_PATH} root frame after asset load and before SimulationManager.setup_simulation"
            ),
        })
        result["robot"]["drive_geometry"] = measured_drive_geometry

        checkpoint("ros_and_lidar")
        rclpy, node, clock_stamps = _clock_graph(stage, app)
        lidar_sensor, lidar_metadata = _setup_lidar(config, SimulationManager, app)
        lidar_metadata["local_transform"] = _verify_lidar_transform(
            stage, Usd, UsdGeom, [float(value) for value in config["lidar"]["translation_xyz_m"]]
        )
        geometry = DriveGeometry(float(config["drive_geometry"]["wheel_radius_m"]),
                                 float(config["drive_geometry"]["wheel_separation_m"]))
        watchdog = VelocityWatchdog(max_age_s=float(config["command_timeout_s"]),
                                    max_linear_m_s=float(config["max_linear_m_s"]),
                                    max_angular_rad_s=float(config["max_angular_rad_s"]))
        profile = DriveProfile(geometry, _drive_limits(config))
        ros = _setup_ros(node, config, geometry, watchdog)
        odometry = WheelOdometry(geometry)
        timeline = omni.timeline.get_timeline_interface()
        # RTX ROS writers need the continuous dispatch gate used by the official example.
        rep.orchestrator.set_capture_on_play(True)
        timeline.play()
        for _ in range(WARMUP_STEPS):
            app.update()
            ros["latest_sim_time"] = float(SimulationManager.get_simulation_time())
            rclpy.spin_once(node, timeout_sec=0.0)
        for _ in range(180):
            if ros["scan"] > 0:
                break
            app.update()
            ros["latest_sim_time"] = float(SimulationManager.get_simulation_time())
            rclpy.spin_once(node, timeout_sec=0.0)
        if ros["scan"] == 0:
            raise base.SmokeError("RTX LaserScan writer produced no scan during continuous warmup")
        initial_time = float(SimulationManager.get_simulation_time())
        left, right = _wheel_angles(robot)
        initial_odom = odometry.update(initial_time, left, right)
        ros["publish_odom"](initial_odom)
        initial_position, initial_orientation = _pose(robot)
        initial_tilt = _tilt_evaluation(initial_orientation)
        result["initial_tilt_evaluation"] = initial_tilt
        if not initial_tilt["passed"]:
            robot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
            failure = {"run_id": args.run_id, "status": "failed", "reason": "excessive_initial_tilt",
                       "sim_time_s": initial_time, "position_xyz_m": initial_position,
                       "orientation_wxyz": initial_orientation, "tilt_evaluation": initial_tilt}
            _atomic_json(output / "evaluation-failure.json", failure)
            raise base.SmokeError("Nova Carter exceeded the evaluation tilt limit during warmup")
        _atomic_json(output / READY_FILE, _ready_payload(
            run_id=args.run_id, asset=asset, footprint=footprint, config=config,
            lidar_metadata=lidar_metadata, source_sha256=result["inputs"]["source"]["sha256"],
            sim_time_s=initial_time,
        ))

        checkpoint("mission")
        (output / "media").mkdir()
        trajectory_path = output / TRAJECTORY_FILE
        frames: list[dict[str, Any]] = []
        start_sim = initial_time
        mission: dict[str, Any] | None = None
        step = 0
        settle_remaining: int | None = None
        settle_elapsed = 0
        settle_limit = _settle_step_limit(config)
        capture_period = int(config["camera_capture_every_steps"])
        encoder_period = int(config["encoder_publish_every_steps"])
        while True:
            now = float(SimulationManager.get_simulation_time())
            if now - start_sim > float(config["max_sim_time_s"]):
                raise base.OperationTimeout("G1 mission exceeded configured simulation time")
            if time.monotonic() - started > float(config["max_wall_time_s"]):
                raise base.OperationTimeout("G1 mission exceeded configured wall time")
            ros["latest_sim_time"] = now
            rclpy.spin_once(node, timeout_sec=0.0)
            control = _read_control(output / CONTROL_FILE, args.run_id)
            terminal_error: str | None = None
            if control is not None and mission is None:
                mission = control
                if control["status"] == "passed":
                    settle_remaining = SETTLE_STEPS
                else:
                    terminal_error = f"mission controller ended with {control['status']}"
            watchdog_linear, watchdog_angular = watchdog.command(now)
            watchdog_active = watchdog.active
            requested_linear, requested_angular = watchdog_linear, watchdog_angular
            if settle_remaining is not None:
                requested_linear = requested_angular = 0.0
            stop_reason, emergency_stop = _stop_mode(
                mission_settling=settle_remaining is not None,
                watchdog_active=watchdog_active,
                mission_failed=terminal_error is not None,
            )
            drive_command = profile.step(
                requested_linear, requested_angular, float(config["physics_dt_s"]),
                emergency_stop=emergency_stop,
            )
            linear, angular = drive_command.linear_m_s, drive_command.angular_rad_s
            targets = drive_command.wheel_targets_rad_s
            robot.apply_wheel_actions(np.asarray(targets, dtype=np.float32))
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            step += 1
            stamp = float(SimulationManager.get_simulation_time())
            left, right = _wheel_angles(robot)
            odom = odometry.update(stamp, left, right)
            if step % encoder_period == 0:
                ros["publish_odom"](odom)
            gt_position, gt_orientation = _pose(robot)
            sample = {
                "physics_frame": int(SimulationManager.get_num_physics_steps()), "sim_time_s": stamp,
                "wheel_angles_rad": [left, right], "wheel_target_rad_s": list(targets),
                "joint_positions_rad": _joint_positions(robot), "joint_names": list(robot.dof_names),
                "requested_command": {"linear_m_s": watchdog_linear, "angular_rad_s": watchdog_angular,
                                      "watchdog_active": watchdog_active},
                "command": {"linear_m_s": linear, "angular_rad_s": angular},
                "motion_profile": _motion_profile_payload(
                    drive_command, input_linear_m_s=requested_linear,
                    input_angular_rad_s=requested_angular, stop_reason=stop_reason,
                ),
                "odometry": {"x_m": odom.x_m, "y_m": odom.y_m, "yaw_rad": odom.yaw_rad,
                               "linear_m_s": odom.linear_m_s, "angular_rad_s": odom.angular_rad_s},
                "ground_truth": {"position_xyz_m": gt_position, "orientation_wxyz": gt_orientation,
                                 "use": "evaluation and media only"},
            }
            tilt = _tilt_evaluation(gt_orientation)
            sample["ground_truth"]["tilt_evaluation"] = tilt
            if not tilt["passed"]:
                fault_stop = profile.step(0.0, 0.0, float(config["physics_dt_s"]), emergency_stop=True)
                robot.apply_wheel_actions(np.asarray(fault_stop.wheel_targets_rad_s, dtype=np.float32))
                sample["motion_profile"]["fault_response"] = _motion_profile_payload(
                    fault_stop, input_linear_m_s=0.0, input_angular_rad_s=0.0,
                    stop_reason="excessive_tilt",
                )
                _append_jsonl(trajectory_path, sample)
                failure = {"run_id": args.run_id, "status": "failed", "reason": "excessive_tilt",
                           "sim_time_s": stamp, "physics_frame": sample["physics_frame"],
                           "position_xyz_m": gt_position, "orientation_wxyz": gt_orientation,
                           "tilt_evaluation": tilt, "ground_truth_use": "run validity evaluation only",
                           "motion_profile": sample["motion_profile"]["fault_response"]}
                _atomic_json(output / "evaluation-failure.json", failure)
                timeline.pause()
                try:
                    frame = _capture_frame(output=output, session=session, rep=rep, np=np, Gf=Gf,
                                           UsdGeom=UsdGeom, SimulationManager=SimulationManager,
                                           robot=robot, sample=sample, index=len(frames))
                    frame["evaluation_failure"] = "excessive_tilt"
                    frames.append(frame)
                    _atomic_json(output / MEDIA_INDEX_FILE, {"schema_version": 1, "frames": frames})
                except BaseException as capture_error:
                    failure["capture_error"] = f"{type(capture_error).__name__}: {capture_error}"
                    _atomic_json(output / "evaluation-failure.json", failure)
                raise base.SmokeError("Nova Carter exceeded the evaluation tilt limit")
            _append_jsonl(trajectory_path, sample)
            capture_due = step == 1 or step % capture_period == 0
            if settle_remaining is not None:
                settle_elapsed += 1
                if drive_command.linear_m_s == 0.0 and drive_command.angular_rad_s == 0.0:
                    settle_remaining -= 1
                else:
                    settle_remaining = SETTLE_STEPS
                if settle_elapsed > settle_limit:
                    raise base.SmokeError("normal mission stop did not settle within the configured drive envelope")
                if settle_remaining == 0:
                    capture_due = True
            if capture_due:
                timeline.pause()
                frame = _capture_frame(output=output, session=session, rep=rep, np=np, Gf=Gf,
                                       UsdGeom=UsdGeom, SimulationManager=SimulationManager,
                                       robot=robot, sample=sample, index=len(frames))
                frames.append(frame)
                _atomic_json(output / MEDIA_INDEX_FILE, {"schema_version": 1, "frames": frames})
                _atomic_json(output / "sim-progress.json", {
                    "run_id": args.run_id, "sim_time_s": stamp,
                    "physics_frame": sample["physics_frame"], "frames": len(frames),
                    "ros_counts": {name: ros[name] for name in ("scan", "odom", "tf", "cmd_vel", "cmd_rejected")},
                })
                timeline.play()
            if terminal_error is not None:
                raise base.SmokeError(terminal_error)
            if settle_remaining == 0:
                break

        robot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
        timeline.pause()
        rep.orchestrator.set_capture_on_play(False)
        session.close()
        session = None
        result["mission_control"] = mission
        result["trajectory"] = {"path": TRAJECTORY_FILE, "sample_count": sum(1 for _ in trajectory_path.open(encoding="utf-8")),
                                "pose_source": "wheel encoders plus separate simulator truth evaluation fields"}
        result["media"] = {"index": MEDIA_INDEX_FILE, "frame_count": len(frames),
                           "capture_period_steps": capture_period, "initial_and_final": True}
        result["ros"] = {key: value for key, value in ros.items() if key not in {"publish_odom", "resources"}}
        result["ros"]["clock"] = {"count": len(clock_stamps), "advanced": len(clock_stamps) >= 2 and clock_stamps[-1] > clock_stamps[0]}
        result["lidar"] = lidar_metadata
        result["timing"] = {"physics_dt_s": float(config["physics_dt_s"]), "warmup_steps": WARMUP_STEPS,
                            "mission_steps": step, "settle_steps": settle_elapsed,
                            "required_zero_settle_steps": SETTLE_STEPS,
                            "settle_step_limit": settle_limit,
                            "sim_seconds": float(SimulationManager.get_simulation_time()) - start_sim,
                            "wall_seconds": time.monotonic() - started}
        if result["ros"]["scan"] <= 0 or result["ros"]["odom"] <= 0 or result["ros"]["cmd_vel"] <= 0:
            raise base.SmokeError("G1 ROS evidence is incomplete")
        result["artifact_manifest_scope"] = "Isaac worker-owned stable files; parent hashes shared ROS artifacts after both workers exit"
        result["artifact_manifest"] = [
            {"path": str(path.relative_to(output)).replace("\\", "/"), "bytes": path.stat().st_size,
             "sha256": base._sha256(path)}
            for path in _scene_owned_artifacts(output)
        ]
        valid = True
    except BaseException as error:
        failure_error = error
        result.update({"status": "failed", "stage": phase, "failure_stage": phase,
                       "error": {"type": type(error).__name__, "message": str(error),
                                 "traceback": traceback.format_exc(limit=20)}})
        if output is not None:
            _atomic_json(output / RESULT_FILE, result)
        print(f"FACTORY_OPS_G1_ERROR={phase}: {type(error).__name__}: {error}", flush=True)
    finally:
        cleanup_errors: list[str] = []
        if not valid and failure_error is not None:
            cleanup_fault_response, fault_stop_errors = _apply_cleanup_fault_stop(
                profile=profile, robot=robot, physics_dt_s=physics_dt_s,
                array_module=np, failure_phase=phase, failure_error=failure_error,
            )
            result["cleanup_fault_response"] = cleanup_fault_response
            cleanup_errors.extend(fault_stop_errors)
        actions = ([lambda: rep.orchestrator.set_capture_on_play(False), session.close] if session is not None else [])
        actions += ([lambda: lidar_sensor.detach_writer("RtxLidarROS2PublishLaserScan")] if lidar_sensor is not None else [])
        actions += ([node.destroy_node] if node is not None else [])
        actions += ([rclpy.shutdown] if rclpy is not None else [])
        for action in actions:
            try:
                action()
            except BaseException as error:
                cleanup_errors.append(f"{type(error).__name__}: {error}")
        if cleanup_errors:
            valid = False
            result["cleanup_errors"] = cleanup_errors
        code = 0 if valid else 1
        if output is not None:
            result["cleanup"] = "worker_exit_requested" if valid else "cleanup_failed"
            result["status"] = "validated" if valid else "failed"
            result["stage"] = "worker_exit_requested" if valid else result.get("failure_stage", phase)
            _atomic_json(output / RESULT_FILE, result)
        if app is not None:
            code = _close_app(app, code, output, result)
    return code


def main() -> int:
    code = _run(_args())
    print(f"FACTORY_OPS_G1_STATUS={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
