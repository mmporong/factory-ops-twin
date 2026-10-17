#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""G2 Isaac worker: two lease-gated Nova Carters with isolated ROS namespaces."""
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
from factory_ops.drive_profile import DriveProfile
from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout
from factory_ops.motion import DriveGeometry, VelocityWatchdog, WheelOdometry
from factory_ops.motion_authorization import AuthorizationError, AuthorizationState, LeaseGuard
from scripts.isaac import stage_smoke as base
from scripts.isaac.nav_scene import (
    DRIVE_LIMIT_FIELDS,
    SETTLE_STEPS,
    _atomic_json,
    _canonical_bytes,
    _drive_geometry_from_colliders,
    _drive_limits,
    _joint_positions,
    _motion_profile_payload,
    _pose,
    _quaternion_from_yaw,
    _settle_step_limit,
    _tilt_evaluation,
    _wheel_angles,
)
from scripts.isaac.robot_qualification import (
    CameraSpec,
    CaptureSession,
    WHEEL_JOINTS,
    _camera,
    _clock_graph,
    _collision_bounds,
)

RESULT_FILE = "g2-scene.json"
READY_FILE = "sim-ready.json"
CONTROL_FILE = "mission-control.json"
MEDIA_INDEX_FILE = "media/frames.json"
SCENARIOS = {"P0", "F1", "F2", "F3"}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--layout", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--scenarios", required=True, type=Path)
    parser.add_argument("--scenario", required=True, choices=sorted(SCENARIOS))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    return parser.parse_args()


def _positive(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise base.SmokeError(f"G2 config field must be finite and positive: {name}")
    return float(value)


def _json_safe_setting(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_json_safe_setting(item) for item in value]
    if isinstance(value, Mapping):
        return {str(key): _json_safe_setting(item) for key, item in value.items()}
    return str(value)


def _load_inputs(
    config_path: Path, scenarios_path: Path, scenario_id: str, *,
    qualification: str = "g2_multi_robot_traffic", scenario_ids: set[str] = SCENARIOS,
) -> tuple[dict[str, Any], dict[str, Any]]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    document = json.loads(scenarios_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("qualification") != qualification:
        raise base.SmokeError("G2 config has the wrong qualification")
    for name in (
        "physics_dt_s", "encoder_publish_every_steps", "camera_capture_every_steps", "warmup_steps",
        "command_timeout_s", "authorization_ttl_s", "max_sim_time_s", "max_wall_time_s",
        "max_linear_m_s", "max_angular_rad_s", "max_linear_accel_m_s2", "max_angular_accel_rad_s2",
    ):
        _positive(config.get(name), name)
    if abs(float(config["physics_dt_s"]) - 1.0 / 60.0) > 1e-12:
        raise base.SmokeError("G2 physics_dt_s must be fixed at 1/60 second")
    if abs(float(config["authorization_ttl_s"]) - 0.5) > 1e-12:
        raise base.SmokeError("G2 authorization_ttl_s must be fixed at 0.5 second")
    limits = _drive_limits(config)
    legacy = (
        ("max_linear_m_s", limits.max_linear_m_s),
        ("max_angular_rad_s", limits.max_angular_rad_s),
        ("max_linear_accel_m_s2", limits.linear_accel_m_s2),
        ("max_angular_accel_rad_s2", limits.angular_accel_rad_s2),
    )
    if any(float(config[name]) != expected for name, expected in legacy):
        raise base.SmokeError("G2 legacy command limits must equal drive_profile")
    geometry = config.get("drive_geometry")
    if not isinstance(geometry, dict) or not isinstance(geometry.get("provenance"), dict):
        raise base.SmokeError("G2 drive geometry provenance is missing")
    DriveGeometry(float(geometry["wheel_radius_m"]), float(geometry["wheel_separation_m"]))
    robots = config.get("robots")
    if not isinstance(robots, list) or len(robots) != 2:
        raise base.SmokeError("G2 requires exactly two robot definitions")
    identities: list[tuple[str, str, str]] = []
    for robot in robots:
        if not isinstance(robot, dict):
            raise base.SmokeError("G2 robot definition must be an object")
        values = tuple(robot.get(name) for name in ("robot_id", "namespace", "prim_path"))
        if any(not isinstance(value, str) or not value for value in values):
            raise base.SmokeError("G2 robot identity fields are required")
        identities.append(values)  # type: ignore[arg-type]
    if any(len({value[index] for value in identities}) != 2 for index in range(3)):
        raise base.SmokeError("G2 robot IDs, namespaces and prim paths must be unique")
    lidars = config.get("lidars")
    if not isinstance(lidars, list) or {item.get("name") for item in lidars if isinstance(item, dict)} != {"front", "rear"}:
        raise base.SmokeError("G2 requires front and rear lidar definitions")
    for lidar in lidars:
        if not isinstance(lidar, dict) or not isinstance(lidar.get("frame_id"), str):
            raise base.SmokeError("G2 lidar frame is invalid")
        xyz = lidar.get("translation_xyz_m")
        if not isinstance(xyz, list) or len(xyz) != 3 or not all(
            not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) for value in xyz
        ):
            raise base.SmokeError("G2 lidar translation is invalid")
        yaw = lidar.get("yaw_rad")
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
            raise base.SmokeError("G2 lidar yaw is invalid")
    scenarios = document.get("scenarios") if isinstance(document, dict) else None
    scenario = scenarios.get(scenario_id) if isinstance(scenarios, dict) else None
    if scenario_id not in scenario_ids or not isinstance(scenario, dict):
        raise base.SmokeError("G2 scenario is missing")
    spawns = scenario.get("spawns")
    if not isinstance(spawns, dict) or set(spawns) != {value[0] for value in identities}:
        raise base.SmokeError("G2 scenario spawns must match both robots")
    for robot_id, spawn in spawns.items():
        if not isinstance(spawn, list) or len(spawn) != 3 or not all(
            not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) for value in spawn
        ):
            raise base.SmokeError(f"G2 spawn is invalid: {robot_id}")
    _positive(scenario.get("max_sim_time_s"), "scenario.max_sim_time_s")
    return config, scenario


def _read_regular_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    if path.is_symlink() or not path.is_file():
        raise base.SmokeError(f"G2 control input must be a regular file: {path.name}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise base.SmokeError(f"G2 control input must be an object: {path.name}")
    return value


def _read_mission(path: Path, run_id: str, scenario_id: str) -> dict[str, Any] | None:
    value = _read_regular_json(path)
    if value is None:
        return None
    if value.get("run_id") != run_id or value.get("scenario_id") != scenario_id:
        raise base.SmokeError("G2 mission control identity mismatch")
    if value.get("status") not in {"passed", "failed"}:
        raise base.SmokeError("G2 mission control status is invalid")
    return value


def _scope_identity(state: AuthorizationState) -> tuple[str, int] | None:
    if state.mode != "navigate" or state.payload is None:
        return None
    scope = state.payload["scope"]
    return str(scope["goal_uuid"]), int(scope["reservation_id"])


def _authorization_record(
    state: AuthorizationState, raw: Mapping[str, Any] | None, *, evaluation_sim_time_s: float,
) -> dict[str, Any]:
    return {
        "evaluation_sim_time_s": evaluation_sim_time_s,
        "payload": dict(raw) if raw is not None else None,
        "mode": state.mode,
        "reason": state.reason,
        "valid": state.valid,
        "sequence": state.sequence,
    }


def _require_scenario_scope(state: AuthorizationState, expected_scenario_id: str) -> None:
    if state.payload is None:
        return
    scope = state.payload.get("scope")
    if not isinstance(scope, Mapping) or scope.get("scenario_id") != expected_scenario_id:
        raise base.SmokeError("G2 authorization scenario scope mismatch")


def _command_audit_snapshot(ros: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "sequence": int(ros["command_sequence"]),
        "received_sim_time_s": ros["command_sim_time_s"],
    }


def _control_decision(
    state: AuthorizationState, *, watchdog_active: bool, command_sequence: int,
    active_scope: tuple[str, int] | None, minimum_command_sequence: int,
) -> tuple[float, float, bool, str]:
    if state.mode == "hold" and state.valid:
        return 0.0, 0.0, False, "normal_schedule_stop"
    scope = _scope_identity(state)
    if state.mode == "navigate" and state.valid:
        if scope != active_scope or command_sequence < minimum_command_sequence:
            return 0.0, 0.0, True, "injected_fault"
        if not watchdog_active:
            return 0.0, 0.0, True, "watchdog_stale"
        return math.nan, math.nan, False, "none"
    return 0.0, 0.0, True, "injected_fault"


def _create_robot(
    stage_utils: Any, stage: Any, app: Any, WheeledRobot: Any, UsdGeom: Any, Gf: Any,
    asset: str, prim_path: str, spawn_xy_yaw: Sequence[float],
) -> Any:
    stage_utils.add_reference_to_stage(usd_path=asset, path=prim_path)
    base._wait_for_stage(app, stage_utils)
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        raise base.SmokeError(f"Nova Carter reference did not produce a valid root prim: {prim_path}")
    xform = UsdGeom.Xformable(root)
    suffix = prim_path.rsplit("/", 1)[-1]
    xform.AddTranslateOp(opSuffix=f"g2{suffix}Spawn").Set(
        Gf.Vec3d(float(spawn_xy_yaw[0]), float(spawn_xy_yaw[1]), 0.1)
    )
    xform.AddRotateZOp(opSuffix=f"g2{suffix}Yaw").Set(math.degrees(float(spawn_xy_yaw[2])))
    return WheeledRobot(prim_path, wheel_dof_names=WHEEL_JOINTS)


def _author_lidar_local_transform(prim: Any, UsdGeom: Any, Gf: Any, xyz: Sequence[float], yaw_rad: float) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), math.degrees(yaw_rad)))
    matrix.SetTranslateOnly(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2])))
    xform.AddTransformOp(opSuffix="g2LocalMount").Set(matrix)


def _verify_lidar_local_transform(
    stage: Any, Usd: Any, UsdGeom: Any, lidar_path: str, chassis_path: str,
    xyz: Sequence[float], yaw_rad: float,
) -> dict[str, Any]:
    lidar, chassis = stage.GetPrimAtPath(lidar_path), stage.GetPrimAtPath(chassis_path)
    if not lidar.IsValid() or not chassis.IsValid():
        raise base.SmokeError("G2 lidar or chassis prim is missing")
    matrix, resets = UsdGeom.XformCache(Usd.TimeCode.Default()).ComputeRelativeTransform(lidar, chassis)
    if resets:
        raise base.SmokeError("G2 lidar resets the chassis transform stack")
    translation = [float(value) for value in matrix.ExtractTranslation()]
    rotation = matrix.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    actual = [float(imaginary[0]), float(imaginary[1]), float(imaginary[2]), float(rotation.GetReal())]
    expected = [0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)]
    same = all(abs(left - right) <= 1e-5 for left, right in zip(actual, expected))
    opposite = all(abs(left + right) <= 1e-5 for left, right in zip(actual, expected))
    if any(abs(left - float(right)) > 1e-5 for left, right in zip(translation, xyz)) or not (same or opposite):
        raise base.SmokeError(f"G2 lidar local mount differs from config: {lidar_path}")
    return {"translation_xyz_m": translation, "rotation_xyzw": actual, "parent_frame": "base_link"}


def _setup_lidars(
    config: Mapping[str, Any], robots: Mapping[str, dict[str, Any]], stage: Any,
    Usd: Any, UsdGeom: Any, Gf: Any, SimulationManager: Any, app: Any,
) -> tuple[list[Any], list[dict[str, Any]]]:
    import isaacsim.core.experimental.utils.prim as prim_utils
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    handles: list[tuple[Any, dict[str, Any], dict[str, Any], str]] = []
    for runtime in robots.values():
        definition = runtime["definition"]
        for lidar_config in config["lidars"]:
            name = str(lidar_config["name"])
            lidar_path = f"{definition['prim_path']}/chassis_link/lidar_{name}"
            lidar = Lidar.create(
                path=lidar_path, config=str(config["lidar_profile"]), tick_rate=10.0,
                translations=[[0.0, 0.0, 0.0]],
            )
            prim = prim_utils.get_prim_at_path(lidar.paths[0])
            _author_lidar_local_transform(
                prim, UsdGeom, Gf, lidar_config["translation_xyz_m"], float(lidar_config["yaw_rad"])
            )
            handles.append((lidar, definition, lidar_config, lidar_path))
    SimulationManager.setup_simulation(dt=float(config["physics_dt_s"]), device="cpu")
    app.update()
    sensors: list[Any] = []
    metadata: list[dict[str, Any]] = []
    for lidar, definition, lidar_config, lidar_path in handles:
        sensor = LidarSensor(lidar, annotators=[])
        prim = prim_utils.get_prim_at_path(lidar.paths[0])
        tick_rate_attribute = prim.GetAttribute("omni:sensor:tickRate")
        accumulate_attribute = prim.GetAttribute("omni:sensor:Core:accumulateOutputs")
        if not tick_rate_attribute.IsValid() or not accumulate_attribute.IsValid():
            raise base.SmokeError("G2 RTX lidar tickRate or accumulateOutputs attribute is missing")
        tick_rate = tick_rate_attribute.Get()
        accumulate_outputs = accumulate_attribute.Get()
        if (isinstance(tick_rate, bool) or not isinstance(tick_rate, (int, float))
                or not math.isfinite(tick_rate) or tick_rate <= 0):
            raise base.SmokeError("G2 RTX lidar tickRate attribute is not finite and positive")
        if not isinstance(accumulate_outputs, bool):
            raise base.SmokeError("G2 RTX lidar accumulateOutputs attribute is not boolean")
        rotation_frequency = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
        firing_frequency = float(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
        near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
        far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
        if rotation_frequency <= 0 or firing_frequency <= 0:
            raise base.SmokeError("G2 RTX lidar scan or firing frequency is not positive")
        resolution = 360.0 * rotation_frequency / firing_frequency
        topic = f"/{definition['namespace']}/scan_{lidar_config['name']}_raw"
        sensor.attach_writer(
            "RtxLidarROS2PublishLaserScan", topicName=topic,
            frameId=str(lidar_config["frame_id"]), horizontalFov=360.0,
            horizontalResolution=resolution, depthRange=[near_range, far_range],
            rotationRate=rotation_frequency, azimuthRange=[-180.0, 180.0],
        )
        local = _verify_lidar_local_transform(
            stage, Usd, UsdGeom, lidar_path, f"{definition['prim_path']}/chassis_link",
            lidar_config["translation_xyz_m"], float(lidar_config["yaw_rad"]),
        )
        sensors.append(sensor)
        metadata.append({
            "robot_id": definition["robot_id"], "name": lidar_config["name"],
            "prim_path": lidar_path, "topic": topic, "frame_id": lidar_config["frame_id"],
            "translation_xyz_m": [float(value) for value in lidar_config["translation_xyz_m"]],
            "yaw_rad": float(lidar_config["yaw_rad"]),
            "local_transform": local, "raw_field_of_view_deg": 360.0,
            "mission_filter": "sensor-frame forward half-plane to namespaced filtered topic",
            "rotation_frequency_hz": rotation_frequency, "firing_frequency_hz": firing_frequency,
            "near_range_m": near_range, "far_range_m": far_range,
            "horizontal_resolution_deg": resolution,
            "tick_rate_hz": float(tick_rate), "accumulate_outputs": accumulate_outputs,
        })
    return sensors, metadata


def _setup_robot_ros(
    rclpy: Any, definition: Mapping[str, Any], lidar_configs: Sequence[Mapping[str, Any]],
    geometry: DriveGeometry, watchdog: VelocityWatchdog,
) -> tuple[Any, dict[str, Any]]:
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    namespace = str(definition["namespace"])
    node = rclpy.create_node(
        "factory_g2_bridge", namespace=namespace,
        cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
    )
    state: dict[str, Any] = {
        "cmd_vel": 0, "cmd_rejected": 0, "command_sequence": 0, "command_sim_time_s": None,
        "odom": 0, "tf": 0, "tf_static": 0, "latest_sim_time": 0.0,
        "scan_front_raw": 0, "scan_rear_raw": 0, "scan_samples": [],
    }

    def command(message: Any) -> None:
        try:
            watchdog.receive(float(state["latest_sim_time"]), float(message.linear.x), float(message.angular.z))
            state["cmd_vel"] += 1
            state["command_sequence"] += 1
            state["command_sim_time_s"] = float(state["latest_sim_time"])
        except (TypeError, ValueError):
            state["cmd_rejected"] += 1

    node._g2_cmd_subscription = node.create_subscription(Twist, "cmd_vel", command, 50)
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    scan_subscriptions = []
    for name in ("front", "rear"):
        def scan(message: Any, *, sensor_name: str = name) -> None:
            state[f"scan_{sensor_name}_raw"] += 1
            if len(state["scan_samples"]) < 10:
                finite = [float(value) for value in message.ranges if math.isfinite(value)]
                state["scan_samples"].append({
                    "name": sensor_name, "frame_id": message.header.frame_id,
                    "range_count": len(message.ranges), "finite_count": len(finite),
                    "minimum_finite_m": min(finite) if finite else None,
                })
        scan_subscriptions.append(node.create_subscription(LaserScan, f"scan_{name}_raw", scan, qos))
    odom_publisher = node.create_publisher(Odometry, "odom", 20)
    tf_broadcaster = TransformBroadcaster(node)
    static_broadcaster = StaticTransformBroadcaster(node)
    statics = []
    for lidar in lidar_configs:
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = str(lidar["frame_id"])
        xyz = lidar["translation_xyz_m"]
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        qx, qy, qz, qw = _quaternion_from_yaw(float(lidar["yaw_rad"]))
        transform.transform.rotation.x, transform.transform.rotation.y = qx, qy
        transform.transform.rotation.z, transform.transform.rotation.w = qz, qw
        statics.append(transform)
    static_broadcaster.sendTransform(statics)
    state["tf_static"] = len(statics)

    def publish_odom(sample: Any) -> None:
        sec = int(sample.stamp_s)
        nanosec = int(round((sample.stamp_s - sec) * 1_000_000_000))
        if nanosec == 1_000_000_000:
            sec, nanosec = sec + 1, 0
        message = Odometry()
        message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
        message.header.frame_id, message.child_frame_id = "odom", "base_link"
        message.pose.pose.position.x, message.pose.pose.position.y = sample.x_m, sample.y_m
        qx, qy, qz, qw = _quaternion_from_yaw(sample.yaw_rad)
        message.pose.pose.orientation.x, message.pose.pose.orientation.y = qx, qy
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = qz, qw
        message.twist.twist.linear.x, message.twist.twist.angular.z = sample.linear_m_s, sample.angular_rad_s
        odom_publisher.publish(message)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = message.header, "base_link"
        transform.transform.translation.x, transform.transform.translation.y = sample.x_m, sample.y_m
        transform.transform.rotation = message.pose.pose.orientation
        tf_broadcaster.sendTransform(transform)
        state["odom"] += 1
        state["tf"] += 1

    state["publish_odom"] = publish_odom
    state["resources"] = [*scan_subscriptions, odom_publisher, tf_broadcaster, static_broadcaster]
    return node, state


def _spin_nodes(rclpy: Any, nodes: Sequence[Any], passes: int = 4) -> None:
    for _ in range(passes):
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.0)


def _append_jsonl(path: Path, value: Any) -> None:
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(_canonical_bytes(value).decode())
        stream.flush()


def _save_rgb(annotator: Any, path: Path, np: Any, Image: Any) -> dict[str, Any]:
    image = Image.fromarray(np.asarray(annotator.get_data())[:, :, :3].astype(np.uint8))
    try:
        image.save(path)
    finally:
        image.close()
    width, height = base._png_dimensions(path)
    return {"path": path.as_posix(), "sha256": base._sha256(path), "bytes": path.stat().st_size,
            "width": width, "height": height}


def _capture_frame(
    *, output: Path, session: CaptureSession, second_rgb: Any, robots: Mapping[str, dict[str, Any]],
    SimulationManager: Any, np: Any, Image: Any, index: int,
) -> dict[str, Any]:
    before_time = float(SimulationManager.get_simulation_time())
    before_frame = int(SimulationManager.get_num_physics_steps())
    before_poses = {robot_id: _pose(runtime["robot"]) for robot_id, runtime in robots.items()}
    sources = {}
    for name, annotator in (
        ("overview", session.overview_rgb), ("carter1", session.medium_rgb), ("carter2", second_rgb),
    ):
        relative = Path("media") / f"{name}-{index:05d}.png"
        evidence = _save_rgb(annotator, output / relative, np, Image)
        evidence["path"] = relative.as_posix()
        sources[name] = evidence
    after_poses = {robot_id: _pose(runtime["robot"]) for robot_id, runtime in robots.items()}
    if (abs(float(SimulationManager.get_simulation_time()) - before_time) > 1e-8
            or int(SimulationManager.get_num_physics_steps()) != before_frame or after_poses != before_poses):
        raise base.SmokeError("G2 camera capture advanced simulation state")
    return {
        "index": index, "physics_frame": before_frame, "sim_time_s": before_time,
        "sources": sources,
        "ground_truth": {
            robot_id: {"position_xyz_m": pose[0], "orientation_wxyz": pose[1]}
            for robot_id, pose in before_poses.items()
        },
        "source": "three persistent RGB annotators read while the timeline is paused; no interpolation",
    }


def _ready_payload(
    *, run_id: str, scenario_id: str, sim_time_s: float, config: Mapping[str, Any],
    scenario: Mapping[str, Any], robots: Mapping[str, dict[str, Any]], lidar_metadata: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    lidars_by_robot = {
        robot_id: [dict(item) for item in lidar_metadata if item["robot_id"] == robot_id]
        for robot_id in robots
    }
    return {
        "schema_version": 1, "run_id": run_id, "scenario_id": scenario_id,
        "status": "ready", "sim_time_s": sim_time_s,
        "control_boundary": {
            "owner": "one DriveProfile per robot in the 60 Hz Isaac physics loop",
            "limits": {name: getattr(_drive_limits(config), name) for name in DRIVE_LIMIT_FIELDS},
            "authorization_ttl_s": float(config["authorization_ttl_s"]),
            "twist_scope_note": "Twist has no goal UUID; accepted goal scope is asserted by the dispatcher lease",
        },
        "robots": {
            robot_id: {
                "prim_path": runtime["definition"]["prim_path"],
                "namespace": runtime["definition"]["namespace"],
                "spawn_xy_yaw": list(scenario["spawns"][robot_id]),
                "footprint": runtime["footprint"],
                "drive_geometry": runtime["drive_geometry"],
                "ros": {
                    "cmd_vel": f"/{runtime['definition']['namespace']}/cmd_vel",
                    "odom": f"/{runtime['definition']['namespace']}/odom",
                    "tf": f"/{runtime['definition']['namespace']}/tf",
                    "tf_static": f"/{runtime['definition']['namespace']}/tf_static",
                    "scan_front_raw": f"/{runtime['definition']['namespace']}/scan_front_raw",
                    "scan_rear_raw": f"/{runtime['definition']['namespace']}/scan_rear_raw",
                },
                "lidars": lidars_by_robot[robot_id],
                "startup_authorization": runtime["startup_authorization"],
            }
            for robot_id, runtime in robots.items()
        },
    }


def _run(
    args: argparse.Namespace, *, qualification: str = "g2_multi_robot_traffic",
    scenario_ids: set[str] = SCENARIOS, result_file: str = RESULT_FILE,
    scene_extension: Any = None, extra_inputs: Mapping[str, Path] | None = None,
) -> int:
    started = time.monotonic()
    app = rclpy = clock_node = session = timeline = np = rep = None
    output: Path | None = None
    robot_nodes: list[Any] = []
    lidar_sensors: list[Any] = []
    robots: dict[str, dict[str, Any]] = {}
    robot_handles: dict[str, Any] = {}
    valid = False
    phase = "preflight"
    result: dict[str, Any] = {
        "schema_version": 1, "run_id": args.run_id, "scenario_id": args.scenario,
        "qualification": qualification, "status": "running", "stage": phase,
    }
    try:
        if not re.fullmatch(r"[0-9a-f]{32}", args.run_id):
            raise base.SmokeError("--run-id must be 32 lowercase hexadecimal characters")
        scene, output = base._prepare_worker_paths(args.scene, args.output)
        layout_path = args.layout.resolve(strict=True)
        config_path = args.config.resolve(strict=True)
        scenarios_path = args.scenarios.resolve(strict=True)
        layout = Layout.from_file(layout_path)
        config, scenario = _load_inputs(config_path, scenarios_path, args.scenario,
                                        qualification=qualification, scenario_ids=scenario_ids)
        os.environ["ROS_DOMAIN_ID"] = str(int(config["ros_domain_id"]))
        os.environ.setdefault("ROS_DISTRO", "jazzy")
        source_path = Path(__file__).resolve()
        result["inputs"] = {
            "scene": {"path": str(scene), "sha256": base._sha256(scene)},
            "layout": {"path": str(layout_path), "sha256": base._sha256(layout_path)},
            "config": {"path": str(config_path), "sha256": base._sha256(config_path)},
            "scenarios": {"path": str(scenarios_path), "sha256": base._sha256(scenarios_path)},
            "source": {"path": str(source_path), "sha256": base._sha256(source_path)},
        }
        for name, path in (extra_inputs or {}).items():
            source = path.resolve(strict=True)
            result["inputs"][name] = {"path": str(source), "sha256": base._sha256(source)}

        def checkpoint(name: str) -> None:
            nonlocal phase
            phase = name
            result["stage"] = name
            result["elapsed_wall_s"] = time.monotonic() - started
            _atomic_json(output / result_file, result)

        checkpoint("simulation_app")
        from isaacsim import SimulationApp
        app = SimulationApp({
            "headless": True, "renderer": "RealTimePathTracing", "sync_loads": True,
            "fast_shutdown": True, "enable_motion_bvh": True,
        })
        import carb
        renderer_settings = carb.settings.get_settings()
        motion_bvh_enabled = renderer_settings.get("/renderer/raytracingMotion/enabled") is True
        result["renderer"] = {
            "motion_bvh": {
                "requested": True,
                "enabled_readback": motion_bvh_enabled,
                "hydra_engine_masking": _json_safe_setting(
                    renderer_settings.get("/renderer/raytracingMotion/enableHydraEngineMasking")
                ),
                "enabled_hydra_engines": _json_safe_setting(
                    renderer_settings.get("/renderer/raytracingMotion/enabledForHydraEngines")
                ),
                "source": "Isaac Sim 6.2 SimulationApp enable_motion_bvh launch option",
            }
        }
        if not motion_bvh_enabled:
            raise base.SmokeError("G2 renderer Motion BVH enablement readback is not true")
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
        stage.SetEndTimeCode(54000.0)
        base._wait_for_stage(app, stage_utils)
        observed, mismatches = base._inspect_stage(stage, UsdGeom, UsdPhysics)
        result["observed_factory"] = observed
        if mismatches:
            raise base.SmokeError(f"factory invariants failed: {mismatches}")
        if scene_extension is not None:
            result["scene_extension"] = scene_extension.setup(stage, UsdGeom, UsdPhysics, Gf, output)

        checkpoint("load_robots")
        asset = get_assets_root_path() + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        result["robots"] = {}
        for definition in config["robots"]:
            robot_id = str(definition["robot_id"])
            robot = _create_robot(
                stage_utils, stage, app, WheeledRobot, UsdGeom, Gf, asset,
                str(definition["prim_path"]), scenario["spawns"][robot_id],
            )
            robot_handles[robot_id] = robot
            bounds, colliders = _collision_bounds(
                stage, Usd, UsdGeom, UsdPhysics, robot_path=str(definition["prim_path"])
            )
            footprint = qualify_footprint(layout, bounds)
            if not footprint["passed"]:
                raise base.SmokeError(f"Nova Carter collision footprint does not fit factory anchors: {robot_id}")
            measured = _drive_geometry_from_colliders(colliders, robot_path=str(definition["prim_path"]))
            for field in ("wheel_radius_m", "wheel_separation_m"):
                if abs(float(measured[field]) - float(config["drive_geometry"][field])) > 1e-4:
                    raise base.SmokeError(f"G2 measured drive geometry differs for {robot_id}: {field}")
            measured["provenance"]["measured_at"] = datetime.now(timezone.utc).isoformat()
            geometry = DriveGeometry(float(measured["wheel_radius_m"]), float(measured["wheel_separation_m"]))
            robots[robot_id] = {
                "definition": definition, "robot": robot, "footprint": footprint,
                "drive_geometry": measured, "geometry": geometry,
                "watchdog": VelocityWatchdog(
                    max_age_s=float(config["command_timeout_s"]),
                    max_linear_m_s=float(config["max_linear_m_s"]),
                    max_angular_rad_s=float(config["max_angular_rad_s"]),
                ),
                "profile": DriveProfile(geometry, _drive_limits(config)),
                "guard": LeaseGuard(args.run_id, robot_id, float(config["authorization_ttl_s"])),
                "active_scope": None, "minimum_command_sequence": 1,
            }
            result["robots"][robot_id] = {
                "asset_url": asset, "prim_path": definition["prim_path"],
                "namespace": definition["namespace"],
                "spawn_xy_yaw": list(scenario["spawns"][robot_id]),
                "wheel_dof_names": WHEEL_JOINTS, "collider_count": len(colliders),
                "collision_bounds_xyz": bounds, "footprint": footprint,
                "drive_geometry": measured,
            }

        def save_rgb(data: Any, path: Path) -> None:
            image = Image.fromarray(np.asarray(data)[:, :, :3].astype(np.uint8))
            try:
                image.save(path)
            finally:
                image.close()

        c1 = robots["carter1"]["definition"]
        session = CaptureSession(
            output, app, stage, rep, UsdGeom, Gf, SimulationManager, save_rgb,
            medium_camera=CameraSpec(
                f"{c1['prim_path']}/chassis_link/G2EvidenceCamera", (-3.0, -4.0, 3.0),
                (0.0, 0.0, 0.25), "base_link",
            ),
        )
        result["load_preview"] = session.capture_load_preview()
        result["factory_overview"] = session.capture_factory_overview()
        c2 = robots["carter2"]["definition"]
        second_spec = CameraSpec(
            f"{c2['prim_path']}/chassis_link/G2EvidenceCamera", (-3.0, 4.0, 3.0),
            (0.0, 0.0, 0.25), "base_link",
        )
        second_camera = _camera(
            stage, Gf, UsdGeom, second_spec.path, second_spec.eye_xyz, second_spec.target_xyz,
            base.RENDER_RESOLUTION,
        )
        second_product = session._product(str(second_camera.GetPath()), base.RENDER_RESOLUTION)
        second_rgb = session._annotator("rgb", second_product)

        checkpoint("ros_and_lidars")
        rclpy, clock_node, clock_stamps = _clock_graph(stage, app)
        lidar_sensors, lidar_metadata = _setup_lidars(
            config, robots, stage, Usd, UsdGeom, Gf, SimulationManager, app,
        )
        result["lidars"] = lidar_metadata
        for runtime in robots.values():
            node, ros = _setup_robot_ros(
                rclpy, runtime["definition"], config["lidars"], runtime["geometry"], runtime["watchdog"]
            )
            runtime["node"], runtime["ros"] = node, ros
            robot_nodes.append(node)
        timeline = omni.timeline.get_timeline_interface()
        rep.orchestrator.set_capture_on_play(True)
        timeline.play()
        for _ in range(int(config["warmup_steps"])):
            app.update()
            now = float(SimulationManager.get_simulation_time())
            for runtime in robots.values():
                runtime["ros"]["latest_sim_time"] = now
                state = runtime["guard"].evaluate(None, now_s=now)
                command = runtime["profile"].step(0.0, 0.0, float(config["physics_dt_s"]), emergency_stop=True)
                runtime["robot"].apply_wheel_actions(np.asarray(command.wheel_targets_rad_s, dtype=np.float32))
                runtime["startup_authorization"] = _authorization_record(
                    state, None, evaluation_sim_time_s=now,
                )
            _spin_nodes(rclpy, [clock_node, *robot_nodes])
        for _ in range(240):
            if all(runtime["ros"]["scan_front_raw"] > 0 and runtime["ros"]["scan_rear_raw"] > 0
                   for runtime in robots.values()):
                break
            app.update()
            now = float(SimulationManager.get_simulation_time())
            for runtime in robots.values():
                runtime["ros"]["latest_sim_time"] = now
            _spin_nodes(rclpy, [clock_node, *robot_nodes])
        if not all(runtime["ros"]["scan_front_raw"] > 0 and runtime["ros"]["scan_rear_raw"] > 0
                   for runtime in robots.values()):
            raise base.SmokeError("G2 four RTX LaserScan writers did not all produce data")
        initial_time = float(SimulationManager.get_simulation_time())
        for robot_id, runtime in robots.items():
            left, right = _wheel_angles(runtime["robot"])
            odometry = WheelOdometry(runtime["geometry"])
            sample = odometry.update(initial_time, left, right)
            runtime["odometry"] = odometry
            runtime["ros"]["publish_odom"](sample)
            runtime["trajectory_path"] = output / "robots" / robot_id / "trajectory.jsonl"
            runtime["trajectory_path"].parent.mkdir(parents=True)
            orientation = _pose(runtime["robot"])[1]
            if not _tilt_evaluation(orientation)["passed"]:
                raise base.SmokeError(f"G2 robot exceeded initial tilt threshold: {robot_id}")
        _atomic_json(output / READY_FILE, _ready_payload(
            run_id=args.run_id, scenario_id=args.scenario, sim_time_s=initial_time,
            config=config, scenario=scenario, robots=robots, lidar_metadata=lidar_metadata,
        ))

        checkpoint("mission")
        (output / "media").mkdir()
        frames: list[dict[str, Any]] = []
        step = 0
        mission: dict[str, Any] | None = None
        settle_remaining: int | None = None
        settle_elapsed = 0
        settle_limit = _settle_step_limit(config)
        start_sim = initial_time
        max_sim = min(float(config["max_sim_time_s"]), float(scenario["max_sim_time_s"]))
        while True:
            now = float(SimulationManager.get_simulation_time())
            if scene_extension is not None:
                scene_extension.step(now, int(SimulationManager.get_num_physics_steps()))
            if now - start_sim > max_sim:
                raise base.OperationTimeout("G2 scenario exceeded configured simulation time")
            if time.monotonic() - started > float(config["max_wall_time_s"]):
                raise base.OperationTimeout("G2 scenario exceeded configured wall time")
            for runtime in robots.values():
                runtime["ros"]["latest_sim_time"] = now
            _spin_nodes(rclpy, [clock_node, *robot_nodes])
            control = _read_mission(output / CONTROL_FILE, args.run_id, args.scenario)
            terminal_error: str | None = None
            if control is not None and mission is None:
                mission = control
                if control["status"] == "passed":
                    settle_remaining = SETTLE_STEPS
                else:
                    terminal_error = "G2 mission controller ended with failed"

            commands: dict[str, Any] = {}
            for robot_id, runtime in robots.items():
                raw = _read_regular_json(output / "robots" / robot_id / "authorization.json")
                try:
                    authorization = runtime["guard"].evaluate(raw, now_s=now)
                except AuthorizationError as error:
                    raise base.SmokeError(f"G2 authorization rejected for {robot_id}: {error}") from error
                _require_scenario_scope(authorization, args.scenario)
                watchdog_linear, watchdog_angular = runtime["watchdog"].command(now)
                watchdog_active = runtime["watchdog"].active
                command_audit = _command_audit_snapshot(runtime["ros"])
                scope = _scope_identity(authorization)
                if authorization.mode != "navigate" or not authorization.valid:
                    runtime["active_scope"] = None
                elif scope != runtime["active_scope"]:
                    runtime["active_scope"] = scope
                    runtime["minimum_command_sequence"] = int(runtime["ros"]["command_sequence"]) + 1
                if terminal_error is not None:
                    requested_linear = requested_angular = 0.0
                    emergency = True
                    stop_reason = "mission_failed"
                elif settle_remaining is not None:
                    requested_linear = requested_angular = 0.0
                    emergency = False
                    stop_reason = "normal_mission_complete"
                else:
                    requested_linear, requested_angular, emergency, stop_reason = _control_decision(
                        authorization, watchdog_active=watchdog_active,
                        command_sequence=int(runtime["ros"]["command_sequence"]),
                        active_scope=runtime["active_scope"],
                        minimum_command_sequence=int(runtime["minimum_command_sequence"]),
                    )
                    if math.isnan(requested_linear):
                        requested_linear, requested_angular = watchdog_linear, watchdog_angular
                command = runtime["profile"].step(
                    requested_linear, requested_angular, float(config["physics_dt_s"]),
                    emergency_stop=emergency,
                )
                runtime["robot"].apply_wheel_actions(np.asarray(command.wheel_targets_rad_s, dtype=np.float32))
                commands[robot_id] = {
                    "drive": command, "authorization": authorization, "raw": raw,
                    "watchdog_linear": watchdog_linear, "watchdog_angular": watchdog_angular,
                    "watchdog_active": watchdog_active, "requested_linear": requested_linear,
                    "requested_angular": requested_angular, "stop_reason": stop_reason,
                    "command_audit": command_audit,
                }
            app.update()
            _spin_nodes(rclpy, [clock_node, *robot_nodes])
            step += 1
            stamp = float(SimulationManager.get_simulation_time())
            physics_frame = int(SimulationManager.get_num_physics_steps())
            all_zero = True
            for robot_id, runtime in robots.items():
                left, right = _wheel_angles(runtime["robot"])
                odom = runtime["odometry"].update(stamp, left, right)
                if step % int(config["encoder_publish_every_steps"]) == 0:
                    runtime["ros"]["publish_odom"](odom)
                position, orientation = _pose(runtime["robot"])
                tilt = _tilt_evaluation(orientation)
                if not tilt["passed"]:
                    raise base.SmokeError(f"G2 robot exceeded tilt threshold: {robot_id}")
                details = commands[robot_id]
                command = details["drive"]
                all_zero = all_zero and command.linear_m_s == 0.0 and command.angular_rad_s == 0.0
                sample = {
                    "physics_frame": physics_frame, "sim_time_s": stamp,
                    "wheel_angles_rad": [left, right],
                    "wheel_target_rad_s": list(command.wheel_targets_rad_s),
                    "joint_positions_rad": _joint_positions(runtime["robot"]),
                    "joint_names": list(runtime["robot"].dof_names),
                    "requested_command": {
                        "linear_m_s": details["watchdog_linear"],
                        "angular_rad_s": details["watchdog_angular"],
                        "watchdog_active": details["watchdog_active"],
                        "sequence": details["command_audit"]["sequence"],
                        "received_sim_time_s": details["command_audit"]["received_sim_time_s"],
                    },
                    "command": {"linear_m_s": command.linear_m_s, "angular_rad_s": command.angular_rad_s},
                    "motion_profile": _motion_profile_payload(
                        command, input_linear_m_s=details["requested_linear"],
                        input_angular_rad_s=details["requested_angular"],
                        stop_reason=details["stop_reason"],
                    ),
                    "authorization": _authorization_record(
                        details["authorization"], details["raw"], evaluation_sim_time_s=now,
                    ),
                    "odometry": {
                        "x_m": odom.x_m, "y_m": odom.y_m, "yaw_rad": odom.yaw_rad,
                        "linear_m_s": odom.linear_m_s, "angular_rad_s": odom.angular_rad_s,
                    },
                    "ground_truth": {
                        "position_xyz_m": position, "orientation_wxyz": orientation,
                        "tilt_evaluation": tilt, "use": "evaluation and media only",
                    },
                }
                _append_jsonl(runtime["trajectory_path"], sample)
                if scene_extension is not None:
                    scene_extension.after_control(robot_id, sample)
            if settle_remaining is not None:
                settle_elapsed += 1
                settle_remaining = settle_remaining - 1 if all_zero else SETTLE_STEPS
                if settle_elapsed > settle_limit:
                    raise base.SmokeError("G2 normal mission stop did not settle within the drive envelope")
            capture_due = step == 1 or step % int(config["camera_capture_every_steps"]) == 0 or settle_remaining == 0
            if capture_due:
                timeline.pause()
                frames.append(_capture_frame(
                    output=output, session=session, second_rgb=second_rgb, robots=robots,
                    SimulationManager=SimulationManager, np=np, Image=Image, index=len(frames),
                ))
                _atomic_json(output / MEDIA_INDEX_FILE, {"schema_version": 1, "frames": frames})
                timeline.play()
            if terminal_error is not None:
                raise base.SmokeError(terminal_error)
            if settle_remaining == 0:
                break

        for runtime in robots.values():
            runtime["robot"].apply_wheel_actions(np.zeros(2, dtype=np.float32))
        timeline.pause()
        rep.orchestrator.set_capture_on_play(False)
        session.close()
        session = None
        result["mission_control"] = mission
        result["media"] = {"index": MEDIA_INDEX_FILE, "frame_count": len(frames), "same_frame_sources": 3}
        for robot_id, runtime in robots.items():
            result["robots"][robot_id].update({
                "trajectory": str(runtime["trajectory_path"].relative_to(output)).replace("\\", "/"),
                "trajectory_samples": sum(1 for _ in runtime["trajectory_path"].open(encoding="utf-8")),
                "ros": {key: value for key, value in runtime["ros"].items()
                        if key not in {"publish_odom", "resources"}},
            })
        result["clock"] = {"count": len(clock_stamps), "advanced": len(clock_stamps) >= 2 and clock_stamps[-1] > clock_stamps[0]}
        result["timing"] = {
            "physics_dt_s": float(config["physics_dt_s"]), "mission_steps": step,
            "settle_steps": settle_elapsed, "required_zero_settle_steps": SETTLE_STEPS,
            "settle_step_limit": settle_limit,
            "sim_seconds": float(SimulationManager.get_simulation_time()) - start_sim,
            "wall_seconds": time.monotonic() - started,
        }
        for robot_id, runtime in robots.items():
            if any(runtime["ros"][name] <= 0 for name in ("odom", "tf", "scan_front_raw", "scan_rear_raw")):
                raise base.SmokeError(f"G2 ROS evidence is incomplete: {robot_id}")
        valid = True
    except BaseException as error:
        result.update({
            "status": "failed", "stage": phase, "failure_stage": phase,
            "error": {"type": type(error).__name__, "message": str(error),
                      "traceback": traceback.format_exc(limit=20)},
        })
        print(f"FACTORY_OPS_G2_ERROR={phase}: {type(error).__name__}: {error}", flush=True)
    finally:
        cleanup_errors: list[str] = []
        if not valid:
            cleanup_fault_response: dict[str, Any] = {}
            for robot_id, robot in robot_handles.items():
                runtime = robots.get(robot_id)
                response = {"stop_reason": "exception_cleanup", "profile_emergency_stop": False,
                            "applied_zero": False}
                if runtime is not None:
                    try:
                        command = runtime["profile"].step(0.0, 0.0, 1.0 / 60.0, emergency_stop=True)
                        response["profile_emergency_stop"] = command.emergency_stop
                    except BaseException as error:
                        cleanup_errors.append(f"{robot_id} fault profile: {type(error).__name__}: {error}")
                try:
                    targets: Any = [0.0, 0.0]
                    if np is not None:
                        targets = np.zeros(2, dtype=np.float32)
                    robot.apply_wheel_actions(targets)
                    response["applied_zero"] = True
                except BaseException as error:
                    cleanup_errors.append(f"{robot_id} fault stop: {type(error).__name__}: {error}")
                cleanup_fault_response[robot_id] = response
            result["cleanup_fault_response"] = cleanup_fault_response
        actions = []
        if session is not None and rep is not None:
            actions.extend([lambda: rep.orchestrator.set_capture_on_play(False), session.close])
        for sensor in lidar_sensors:
            actions.append(lambda active=sensor: active.detach_writer("RtxLidarROS2PublishLaserScan"))
        actions.extend(node.destroy_node for node in robot_nodes)
        if clock_node is not None:
            actions.append(clock_node.destroy_node)
        if rclpy is not None:
            actions.append(rclpy.shutdown)
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
            result["status"] = "validated" if valid else "failed"
            result["cleanup"] = "worker_exit_requested" if valid else "cleanup_failed"
            _atomic_json(output / result_file, result)
        if app is not None:
            try:
                app.close(wait_for_replicator=False, exit_code=code)
            except BaseException as error:
                code = 1
                result["status"] = "failed"
                result["close_error"] = f"{type(error).__name__}: {error}"
                if output is not None:
                    _atomic_json(output / result_file, result)
    return code


def main() -> int:
    code = _run(_args())
    print(f"FACTORY_OPS_G2_STATUS={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
