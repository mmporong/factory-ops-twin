#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""G1 Isaac worker: bridge one encoder-driven Nova Carter to ROS 2/Nav2."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import traceback
from typing import Any, Mapping

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout
from factory_ops.motion import DriveGeometry, VelocityWatchdog, WheelOdometry
from scripts.isaac import stage_smoke as base
from scripts.isaac.robot_qualification import (
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
    os.replace(temporary, path)


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
        "max_linear_m_s", "max_angular_rad_s",
    )
    for name in required_positive:
        value = raw.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise base.SmokeError(f"G1 config field must be finite and positive: {name}")
    geometry = raw.get("drive_geometry")
    lidar = raw.get("lidar")
    if not isinstance(geometry, dict) or not isinstance(geometry.get("provenance"), dict):
        raise base.SmokeError("G1 drive geometry provenance is missing")
    if not isinstance(lidar, dict) or lidar.get("topic") != "/scan" or lidar.get("frame_id") != "base_scan":
        raise base.SmokeError("G1 lidar contract must publish /scan in base_scan")
    DriveGeometry(float(geometry["wheel_radius_m"]), float(geometry["wheel_separation_m"]))
    return raw


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


def _pose(robot: Any) -> tuple[list[float], list[float]]:
    positions, orientations = robot.get_world_poses()
    p = positions.numpy() if hasattr(positions, "numpy") else positions
    q = orientations.numpy() if hasattr(orientations, "numpy") else orientations
    return [float(v) for v in p[0]], [float(v) for v in q[0]]


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


def _set_follow_camera(session: CaptureSession, Gf: Any, UsdGeom: Any, position: list[float]) -> None:
    eye = Gf.Vec3d(position[0] - 5.5, position[1] - 7.0, position[2] + 5.0)
    target = Gf.Vec3d(position[0], position[1], position[2] + 0.25)
    matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
    camera = UsdGeom.Xformable(session.stage.GetPrimAtPath("/World/RobotMotionCamera"))
    ops = camera.GetOrderedXformOps()
    if len(ops) != 1:
        raise base.SmokeError("follow camera does not have exactly one authored transform")
    ops[0].Set(matrix)


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
    app = rclpy = node = session = lidar_sensor = None
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
        stage_utils.add_reference_to_stage(usd_path=asset, path=ROBOT_PATH)
        base._wait_for_stage(app, stage_utils)
        spawn_x, spawn_y = layout.stations[str(config["source_station"])]
        UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PATH)).AddTranslateOp(opSuffix="g1Spawn").Set(Gf.Vec3d(spawn_x, spawn_y, 0.1))
        robot = WheeledRobot(ROBOT_PATH, wheel_dof_names=WHEEL_JOINTS)

        def save_rgb(data: Any, path: Path) -> None:
            image = Image.fromarray(np.asarray(data)[:, :, :3].astype(np.uint8))
            try:
                image.save(path)
            finally:
                image.close()

        session = CaptureSession(output, app, stage, rep, UsdGeom, Gf, SimulationManager, save_rgb)
        result["load_preview"] = session.capture_load_preview()
        result["factory_overview"] = _capture_factory_overview(output, session)
        bounds, colliders = _collision_bounds(stage, Usd, UsdGeom, UsdPhysics)
        footprint = qualify_footprint(layout, bounds)
        if not footprint["passed"]:
            raise base.SmokeError("Nova Carter collision footprint does not fit factory anchors")
        result["robot"] = {"asset_url": asset, "prim_path": ROBOT_PATH, "wheel_dof_names": WHEEL_JOINTS,
                           "collider_count": len(colliders), "collision_bounds_xyz": bounds, "footprint": footprint}

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
            if control is not None and mission is None:
                mission = control
                if control["status"] != "passed":
                    raise base.SmokeError(f"mission controller ended with {control['status']}")
                settle_remaining = SETTLE_STEPS
            linear, angular = (0.0, 0.0) if settle_remaining is not None else watchdog.command(now)
            targets = geometry.wheel_targets(linear, angular)
            robot.apply_wheel_actions(np.asarray(targets, dtype=np.float32))
            camera_position, _ = _pose(robot)
            _set_follow_camera(session, Gf, UsdGeom, camera_position)
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
                "command": {"linear_m_s": linear, "angular_rad_s": angular},
                "odometry": {"x_m": odom.x_m, "y_m": odom.y_m, "yaw_rad": odom.yaw_rad,
                               "linear_m_s": odom.linear_m_s, "angular_rad_s": odom.angular_rad_s},
                "ground_truth": {"position_xyz_m": gt_position, "orientation_wxyz": gt_orientation,
                                 "use": "evaluation and media only"},
            }
            _append_jsonl(trajectory_path, sample)
            capture_due = step == 1 or step % capture_period == 0
            if settle_remaining is not None:
                settle_remaining -= 1
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
                            "mission_steps": step, "settle_steps": SETTLE_STEPS,
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
        result.update({"status": "failed", "stage": phase, "failure_stage": phase,
                       "error": {"type": type(error).__name__, "message": str(error),
                                 "traceback": traceback.format_exc(limit=20)}})
        if output is not None:
            _atomic_json(output / RESULT_FILE, result)
        print(f"FACTORY_OPS_G1_ERROR={phase}: {type(error).__name__}: {error}", flush=True)
    finally:
        cleanup_errors: list[str] = []
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
