"""Bounded native SDK4.5 adapter admission; authored motion, no Nav2 missions."""
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
from typing import Any


DEPENDENCIES = ["factory_ops/__init__.py", "factory_ops/isaac45_runtime.py",
    "factory_ops/drive_profile.py", "factory_ops/motion.py", "scripts/isaac/local_asset_probe.py"]
WHEELS = ["joint_wheel_left", "joint_wheel_right"]


def save(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def author_mount(prim, UsdGeom, Gf, xyz, yaw):
    transform = UsdGeom.Xformable(prim)
    transform.ClearXformOpOrder()
    matrix = Gf.Matrix4d(1.)
    matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0., 0., 1.), math.degrees(yaw)))
    matrix.SetTranslateOnly(Gf.Vec3d(*xyz))
    transform.AddTransformOp(opSuffix="Local45Mount").Set(matrix)


def verify_mount(stage, Usd, UsdGeom, path, chassis, xyz, yaw):
    matrix, resets = UsdGeom.XformCache(Usd.TimeCode.Default()).ComputeRelativeTransform(
        stage.GetPrimAtPath(path), stage.GetPrimAtPath(chassis))
    quaternion = matrix.ExtractRotationQuat()
    actual = [*quaternion.GetImaginary(), quaternion.GetReal()]
    expected = [0., 0., math.sin(yaw / 2.), math.cos(yaw / 2.)]
    translation = list(matrix.ExtractTranslation())
    if (resets or any(abs(a-b) > 1e-5 for a, b in zip(translation, xyz))
            or min(max(abs(a-b) for a, b in zip(actual, expected)),
                   max(abs(a+b) for a, b in zip(actual, expected))) > 1e-5):
        raise ValueError("actual sensor mount differs: " + path)
    return {"translation_xyz_m": translation, "rotation_xyzw": actual, "parent_frame": "base_link"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    if args.output.exists() or cfg["robot_count"] not in (1, 4):
        parser.error("new output and one/four configured robots required")
    args.output.mkdir(parents=True)
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    scene = (root / cfg["scene"]).resolve(strict=True)
    if sha(scene) != cfg["scene_sha256"]:
        raise ValueError("authored scene bytes differ from the input contract")
    report = {"schema_version": 1, "run_id": args.run_id, "status": "starting", "stage": "preflight",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "native4.5 runtime adapter, wheel motion and local ROS scan identity; no Nav2, transport or cargo qualification",
        "source_sha256": sha(Path(__file__)), "config_sha256": sha(args.config),
        "project_source_sha256": {name: sha(root/name) for name in DEPENDENCIES},
        "scene_sha256": sha(scene), "four_amr_transport_verified": False,
        "physical_loading_verified": False, "physical_charging_verified": False,
        "samples": [], "scans": {}, "scan_callbacks": {}, "media": []}
    app: Any = None
    ros: Any = None
    node: Any = None
    rep: Any = None
    runtime = rgb = product = None
    sensors = []
    scan_annotators = []
    robots = {}
    started = time.monotonic()
    deadline = started + cfg["maximum_wall_time_s"]
    code = 1

    def phase(name):
        report["stage"] = name
        save(args.output/"result.json", report)

    def update():
        if time.monotonic() >= deadline:
            raise TimeoutError("native runtime probe deadline exceeded")
        app.update()
        if node is not None:
            ros.spin_once(node, timeout_sec=0.)

    def scan_received(message, topic, expected_frame):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        valid = sum(math.isfinite(v) and message.range_min <= v <= message.range_max for v in message.ranges)
        callbacks = report["scan_callbacks"].setdefault(topic, {"count": 0})
        callbacks.update(count=callbacks["count"]+1, last_stamp_s=stamp,
            frame_id=message.header.frame_id, range_count=len(message.ranges),
            valid_fraction=valid/len(message.ranges) if message.ranges else 0.)
        if stamp <= 0 or message.header.frame_id != expected_frame:
            return
        if not message.ranges or valid / len(message.ranges) < .1:
            return
        row = report["scans"].setdefault(topic, {"count": 0, "first_stamp_s": stamp,
            "last_stamp_s": None, "frame_id": message.header.frame_id})
        if row["last_stamp_s"] is not None and stamp <= row["last_stamp_s"]:
            return
        row.update(count=row["count"]+1, last_stamp_s=stamp, range_count=len(message.ranges),
            valid_fraction=valid/len(message.ranges), range_min_m=message.range_min,
            range_max_m=message.range_max, angle_increment_rad=message.angle_increment,
            sample_policy="counts and header summary only; raw scan quality qualification remains separate")

    def wait_stage():
        if runtime is None:
            raise RuntimeError("runtime must exist before waiting for its stage")
        quiet = 0
        while quiet < 3:
            update()
            quiet = 0 if runtime.stage_utils.is_stage_loading() else quiet+1

    phase("sdk_startup")
    try:
        from factory_ops import isaac45_runtime
        report["ros_environment"] = isaac45_runtime.configure_ros_environment(cfg["domain_id"])
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "multi_gpu": False, "width": 960, "height": 600})
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        update()
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from factory_ops.drive_profile import DriveLimits, DriveProfile
        from factory_ops.motion import DriveGeometry
        from scripts.isaac.local_asset_probe import measure_drive_geometry
        runtime = isaac45_runtime.create_runtime(app)
        report["isaac_version"] = (Path(os.environ["ISAAC_PATH"])/"VERSION").read_text().strip()
        phase("open_factory")
        opened, stage = runtime.stage_utils.open_stage(str(scene))
        if not opened or stage is None:
            raise RuntimeError("factory stage did not open")
        wait_stage()
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if not scenes:
            UsdPhysics.Scene.Define(stage, "/World/Local45PhysicsScene")
        elif len(scenes) > 1:
            raise RuntimeError("factory has multiple physics scenes")
        phase("load_robots")
        asset = runtime.asset_url
        report["asset_url"] = asset
        definitions = cfg["robots"][:cfg["robot_count"]]
        for definition in definitions:
            path = definition["prim_path"]
            runtime.stage_utils.add_reference_to_stage(usd_path=asset, path=path)
            wait_stage()
            transform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            x, y, yaw = definition["spawn_xy_yaw"]
            transform.AddTranslateOp(opSuffix="Local45Spawn").Set(Gf.Vec3d(x, y, .1))
            transform.AddRotateZOp(opSuffix="Local45Yaw").Set(math.degrees(yaw))
            measured = measure_drive_geometry(stage, path, Usd, UsdGeom, UsdPhysics, Gf, deadline)
            geometry = DriveGeometry(measured["wheel_radius_m"], measured["wheel_separation_m"])
            robots[definition["robot_id"]] = {"definition": definition,
                "robot": runtime.WheeledRobot(path, wheel_dof_names=WHEELS),
                "profile": DriveProfile(geometry, DriveLimits()), "geometry": measured}
        phase("ros_lidar_setup")
        ros = rclpy
        ros.init(args=[])
        node = ros.create_node("local45_fleet_probe_"+args.run_id[:8])
        subscriptions = []
        for definition in definitions:
            for lidar in cfg["lidars"]:
                topic = f"/{definition['namespace']}/scan_{lidar['name']}_raw"
                subscriptions.append(node.create_subscription(LaserScan, topic,
                    lambda message, t=topic, f=lidar["frame_id"]: scan_received(message, t, f), qos_profile_sensor_data))
        sensors, metadata = runtime.setup_lidars(cfg, robots, stage, Usd, UsdGeom, Gf,
            runtime.SimulationManager, app, author_mount, verify_mount)
        if cfg.get("static_reference_lidar", True):
            import omni.kit.commands as kit_commands
            ok, prim = kit_commands.execute("IsaacSensorCreateRtxLidar", path="/World/ReferenceLidar",
                parent=None, config="RPLIDAR_S2E", translation=(12., 5., 1.),
                orientation=Gf.Quatd(1., 0., 0., 0.))
            if not ok or prim is None:
                raise RuntimeError("static reference lidar creation failed")
            reference = type(sensors[0])(rep.create.render_product(prim.GetPath(), (1, 1), name="Local45Reference"))
            reference._writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            reference._writer.initialize(topicName="/local45_reference_scan", frameId="reference_scan")
            reference._writer.attach([reference._product])
            sensors.append(reference)
            metadata.append({"topic": "/local45_reference_scan", "frame_id": "reference_scan",
                "role": "static reference created after World reset; diagnostic only"})
            subscriptions.append(node.create_subscription(LaserScan, "/local45_reference_scan",
                lambda message: scan_received(message, "/local45_reference_scan", "reference_scan"), qos_profile_sensor_data))
        for sensor in sensors:
            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacComputeRTXLidarFlatScanSimulationTime")
            annotator.attach([sensor._product])
            scan_annotators.append(annotator)
        report["lidars"] = metadata
        report["drive_geometry"] = {key: row["geometry"] for key, row in robots.items()}
        UsdLux.DomeLight.Define(stage, "/World/Local45Light").CreateIntensityAttr(1000.)
        camera = rep.create.camera(position=(43., -22., 35.), look_at=(16., 11., 0.))
        product = rep.create.render_product(camera, (960, 600))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        manager = runtime.SimulationManager
        world = manager.world
        if world is None:
            raise RuntimeError("runtime did not initialize an owned World")
        start_time, start_frame = manager.get_simulation_time(), manager.get_num_physics_steps()
        initial = {key: list(row["robot"].get_world_poses()[0][0]) for key, row in robots.items()}
        rep.orchestrator.set_capture_on_play(True)
        world.play()
        phase("authored_wheel_motion")
        previous_time = start_time
        for index in range(420):
            elapsed = manager.get_simulation_time()-start_time
            speed = .2 if 1. <= elapsed < 4. else 0.
            for row in robots.values():
                command = row["profile"].step(speed, 0., cfg["physics_dt_s"])
                row["robot"].apply_wheel_actions(np.asarray(command.wheel_targets_rad_s, dtype=np.float32))
            update()
            if index % 6 == 0:
                before_render = (manager.get_simulation_time(), manager.get_num_physics_steps())
                world.render()
                if (manager.get_simulation_time(), manager.get_num_physics_steps()) != before_render:
                    raise RuntimeError("render-only refresh advanced the actual physics clock")
                ros.spin_once(node, timeout_sec=0.)
            stamp, frame = manager.get_simulation_time(), manager.get_num_physics_steps()
            if stamp <= previous_time or frame <= start_frame:
                raise RuntimeError("app.update did not advance the actual World physics timer")
            previous_time = stamp
            if index % 6 == 0 or index == 419:
                report["samples"].append({"sim_time_s": stamp, "physics_frame": frame,
                    "robots": {key: {"position_m": [float(v) for v in row["robot"].get_world_poses()[0][0]],
                        "joints_rad": [float(v) for v in row["robot"].get_dof_positions()]}
                        for key, row in robots.items()}})
        phase("evaluate_and_capture")
        report["ros_graph"] = {"effective_env": {name: os.environ.get(name) for name in
            ("ROS_DOMAIN_ID", "ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY")},
            "rclpy_domain_id": node.context.get_domain_id(),
            "nodes": node.get_node_names_and_namespaces(), "topics": node.get_topic_names_and_types(),
            "endpoints": {}}
        for row in metadata:
            topic = row["topic"]
            report["ros_graph"]["endpoints"][topic] = {
                side: [{"node_name": item.node_name, "node_namespace": item.node_namespace,
                    "topic_type": item.topic_type, "qos": str(item.qos_profile)} for item in getter(topic)]
                for side, getter in (("publishers", node.get_publishers_info_by_topic),
                                     ("subscriptions", node.get_subscriptions_info_by_topic))}
        report["native_scan_buffers"] = []
        for annotator in scan_annotators:
            data = annotator.get_data()
            depths = np.asarray(data.get("linearDepthData", [])).reshape(-1)
            report["native_scan_buffers"].append({"fields": sorted(data), "range_count": int(depths.size),
                "positive_finite_count": int(np.count_nonzero(np.isfinite(depths) & (depths > 0.)))})
        last = report["samples"][-1]
        previous = report["samples"][-2]
        result = {}
        for key in robots:
            position = last["robots"][key]["position_m"]
            distance = math.dist(position[:2], initial[key][:2])
            tail_speed = math.dist(position[:2], previous["robots"][key]["position_m"][:2])/(last["sim_time_s"]-previous["sim_time_s"])
            result[key] = {"displacement_m": distance, "tail_speed_m_s": tail_speed,
                "passed": .15 <= distance <= 1. and tail_speed < .03 and -.03 <= position[2] <= .3}
        report["robot_results"] = result
        expected_topics = {row["topic"] for row in metadata}
        scans_passed = (set(report["scans"]) == expected_topics
            and all(row["count"] >= 3 for row in report["scans"].values()))
        report["scans_identity_progress_passed"] = scans_passed
        report["actual_physics_steps"] = manager.get_num_physics_steps()-start_frame
        report["actual_sim_time_s"] = manager.get_simulation_time()-start_time
        for _ in range(8):
            world.render()
        destination = args.output/"runtime-overview.png"
        data = np.asarray(rgb.get_data())
        if data.ndim != 3 or data.shape[:2] != (600, 960):
            raise RuntimeError("overview RGB unavailable")
        Image.fromarray(data[:, :, :3].astype(np.uint8)).save(destination)
        report["media"].append({"path": destination.name, "sha256": sha(destination),
            "scope": "adapter debug overview; not final portfolio render"})
        if not scans_passed or not all(row["passed"] for row in result.values()):
            raise RuntimeError("one or more native wheel/scan admission checks failed")
        report["status"] = "passed"
        code = 0
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
            "traceback": traceback.format_exc(limit=15)})
    finally:
        cleanup_errors = []
        for key, row in robots.items():
            try:
                row["robot"].apply_wheel_actions([0., 0.])
            except Exception as error:
                cleanup_errors.append(key+": "+str(error))
        for annotator in scan_annotators:
            try:
                annotator.detach()
            except Exception as error:
                cleanup_errors.append(str(error))
        for sensor in sensors:
            try:
                sensor.detach_writer("RtxLidarROS2PublishLaserScan")
            except Exception as error:
                cleanup_errors.append(str(error))
        if rgb is not None:
            rep.orchestrator.set_capture_on_play(False)
            rgb.detach()
        if product is not None:
            product.destroy()
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        report["cleanup_errors"] = cleanup_errors
        if cleanup_errors:
            report["status"] = "failed"
            code = 1
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report["elapsed_wall_s"] = time.monotonic()-started
        save(args.output/"result.json", report)
        print("LOCAL_FLEET_RUNTIME_RESULT", report["status"], flush=True)
        if app is not None:
            app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
