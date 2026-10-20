"""Isolated SDK4.5 physics/render cadence admission; never a mission runner."""
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import traceback
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from factory_ops import isaac45_runtime, flat_scan_observation
from factory_ops.local45_stepping import Local45Stepper, physics_state
from scripts.isaac.local_fleet_runtime_probe import WHEELS, author_mount, save, verify_mount
from scripts.isaac.robot_qualification import _clock_graph


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def project_hashes() -> dict[str, str]:
    """Pin the cold-import closure as well as the lazy native tracker module."""
    paths = set()
    for module in tuple(sys.modules.values()):
        filename = getattr(module, "__file__", None)
        if isinstance(filename, str) and filename.endswith(".py"):
            paths.add(Path(filename).resolve())
    return {path.relative_to(ROOT).as_posix(): sha(path) for path in sorted(paths)
            if path.is_relative_to(ROOT) and path.relative_to(ROOT).parts[0] in {"factory_ops", "scripts"}
            and path != Path(__file__).resolve()}


def inputs(config: Path, output: Path, run_id: str) -> tuple[dict[str, Any], dict[str, Any], Path]:
    if not re.fullmatch(r"[0-9a-f]{32}", run_id):
        raise ValueError("run-id must contain 32 lowercase hex characters")
    if not output.is_absolute() or output.exists() or output.is_symlink():
        raise ValueError("a new absolute output directory is required")
    if any(parent.is_symlink() for parent in output.parents):
        raise ValueError("output must not traverse a symlink")
    cfg = json.loads(config.read_text(encoding="utf-8-sig"))
    dt, factor = cfg["physics_dt_s"], cfg["render_every_steps"]
    if (type(dt) not in (float, int) or not math.isfinite(dt)
            or not math.isclose(dt, 1 / 60, rel_tol=0., abs_tol=1e-12)
            or type(factor) is not int or factor not in (1, 6)
            or not math.isclose(cfg["rendering_dt_s"], dt * factor, rel_tol=0., abs_tol=1e-12)
            or type(cfg["warmup_physics_steps"]) is not int or not 6 <= cfg["warmup_physics_steps"] <= 120
            or cfg["warmup_physics_steps"] % factor
            or type(cfg["measurement_physics_steps"]) is not int or cfg["measurement_physics_steps"] != 120
            or type(cfg["maximum_wall_time_s"]) not in (float, int)
            or not 0 < cfg["maximum_wall_time_s"] <= 420):
        raise ValueError("bounded 60Hz/120-step experiment with render factor 1 or 6 required")
    fleet_path = (ROOT / cfg["fleet_config"]).resolve(strict=True)
    if not fleet_path.is_relative_to(ROOT) or sha(fleet_path) != cfg["fleet_config_sha256"]:
        raise ValueError("fleet input SHA/path differs")
    fleet = json.loads(fleet_path.read_text(encoding="utf-8"))
    scene = (ROOT / fleet["scene"]).resolve(strict=True)
    if (not scene.is_relative_to(ROOT) or sha(scene) != fleet["scene_sha256"]
            or fleet["robot_count"] != 4 or len(fleet["robots"]) != 4
            or len(fleet["lidars"]) != 2 or fleet.get("static_reference_lidar") is not False
            or fleet["physics_dt_s"] != dt):
        raise ValueError("four-robot/eight-lidar existing authored scene contract differs")
    return cfg, fleet, scene


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    cfg, fleet, scene = inputs(args.config, args.output, args.run_id)
    args.output.mkdir(parents=True, exist_ok=False)
    pins = project_hashes()
    report: dict[str, Any] = {"schema_version": 1, "run_id": args.run_id, "status": "starting",
        "scope": cfg["scope"], "source_sha256": sha(Path(__file__)), "config_sha256": sha(args.config),
        "project_source_sha256": pins, "fleet_config_sha256": cfg["fleet_config_sha256"],
        "scene_sha256": sha(scene), "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": cfg, "physics_samples": [], "native_observations": [],
        "ros_scans": {}, "ros_clock_receipts": [], "media": [], "mismatches": [],
        "transport_verified": False, "navigation_verified": False, "physical_loading_verified": False,
        "physical_charging_verified": False, "sensor_range_quality_verified": False,
        "host_process_closure_verified": False}
    started = time.monotonic()
    deadline = started + cfg["maximum_wall_time_s"]
    app: Any = None
    world: Any = None
    rep: Any = None
    ros: Any = None
    node: Any = None
    robots: dict[str, Any] = {}
    sensors: list[Any] = []
    observers: list[Any] = []
    products: list[Any] = []
    rgbs: list[Any] = []
    subscriptions: list[Any] = []
    metadata: list[dict[str, Any]] = []
    clock_stamps: list[float] = []
    phase = "startup"
    measured = False
    clock_index = 0

    def bound() -> None:
        if time.monotonic() >= deadline:
            raise TimeoutError("probe wall-time bound reached; host guard also required for blocking SDK calls")

    def mismatch(kind: str, **details: Any) -> None:
        report["mismatches"].append({"kind": kind, **details})

    def poses() -> dict[str, Any]:
        result = {}
        for key, row in robots.items():
            positions, orientations = row["robot"].get_world_poses()
            position, orientation = [float(v) for v in positions[0]], [float(v) for v in orientations[0]]
            joints = [float(v) for v in row["robot"].get_dof_positions()]
            if (len(position) != 3 or len(orientation) != 4 or not joints
                    or not all(math.isfinite(v) for v in position + orientation + joints)):
                raise RuntimeError("nonfinite/incomplete actual robot pose or joints: " + key)
            result[key] = {"position_m": position, "orientation_wxyz": orientation, "joints_rad": joints}
        return result

    def receive_scan(message: Any, topic: str, frame_id: str) -> None:
        if not measured:
            return
        frame, now = physics_state(world)
        stamp = float(message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
        rows = report["ros_scans"].setdefault(topic, [])
        row = {"stamp_s": stamp, "read_at_sim_time_s": now, "read_at_physics_frame": frame,
               "frame_id": message.header.frame_id, "range_count": len(message.ranges)}
        rows.append(row)
        if (not math.isfinite(stamp) or stamp <= 0 or message.header.frame_id != frame_id
                or not message.ranges or (len(rows) > 1 and stamp <= rows[-2]["stamp_s"])):
            mismatch("ROS scan identity/positive increasing stamp", topic=topic, receipt=row)

    def observe() -> None:
        nonlocal clock_index
        bound()
        frame, now = physics_state(world)
        # Drain a bounded number of real callbacks without advancing Kit/physics.
        for _ in range(32):
            ros.spin_once(node, timeout_sec=0.)
        new_clocks = clock_stamps[clock_index:]
        clock_index = len(clock_stamps)
        if measured:
            for stamp in new_clocks:
                receipts = report["ros_clock_receipts"]
                receipts.append({"stamp_s": stamp, "read_at_sim_time_s": now,
                                 "read_at_physics_frame": frame, "world_minus_ros_s": now - stamp})
                if (not math.isfinite(stamp) or stamp <= 0
                        or (len(receipts) > 1 and stamp < receipts[-2]["stamp_s"])
                        or not -cfg["physics_dt_s"] - 1e-6 <= now - stamp <= cfg["rendering_dt_s"] + cfg["physics_dt_s"] + 1e-6):
                    mismatch("World/actual ROS clock differ", receipt=receipts[-1])
        for observer in observers:
            row = observer.read(read_at_sim_time_s=now, read_at_physics_frame=frame,
                                timeline_playing=bool(world.is_playing()), phase=phase)
            if measured:
                report["native_observations"].append(row)
                if row["status"] in ("regressed", "invalid", "error"):
                    mismatch("native observation invalid/regressed", topic=row["topic"], reason=row["reason"])

    code = 1
    try:
        report["ros_environment"] = isaac45_runtime.configure_ros_environment(fleet["domain_id"])
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "multi_gpu": False, "width": 1280, "height": 720})
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        app.update()
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        runtime = isaac45_runtime.create_runtime(app)
        report["isaac_version"] = (Path(os.environ["ISAAC_PATH"]) / "VERSION").read_text().strip()

        def wait_stage() -> None:
            quiet = 0
            while quiet < 3:
                bound()
                app.update()
                quiet = 0 if runtime.stage_utils.is_stage_loading() else quiet + 1

        phase = "load_existing_scene_and_four_references"
        opened, stage = runtime.stage_utils.open_stage(str(scene))
        if not opened or stage is None:
            raise RuntimeError("existing factory scene failed to open")
        wait_stage()
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if not scenes:
            UsdPhysics.Scene.Define(stage, "/World/Local45PhysicsScene")
        elif len(scenes) != 1:
            raise RuntimeError("exactly one PhysicsScene required")
        report["asset_url"] = runtime.asset_url
        for definition in fleet["robots"]:
            path = definition["prim_path"]
            runtime.stage_utils.add_reference_to_stage(usd_path=report["asset_url"], path=path)
            wait_stage()
            transform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            x, y, yaw = definition["spawn_xy_yaw"]
            transform.AddTranslateOp(opSuffix="Local45Spawn").Set(Gf.Vec3d(x, y, .1))
            transform.AddRotateZOp(opSuffix="Local45Yaw").Set(math.degrees(yaw))
            robots[definition["robot_id"]] = {"definition": definition,
                "robot": runtime.WheeledRobot(path, wheel_dof_names=WHEELS)}
        manager = runtime.SimulationManager
        manager.setup_simulation(dt=cfg["physics_dt_s"], device="cpu", rendering_dt=cfg["rendering_dt_s"])
        world = manager.world
        if world is None:
            raise RuntimeError("facade did not create World")
        world.pause()
        world.render()
        report["world_before_products"] = {"physics_frame": physics_state(world)[0],
            "sim_time_s": physics_state(world)[1], "physics_dt_s": float(world.get_physics_dt()),
            "rendering_dt_s": float(world.get_rendering_dt())}
        if (not math.isclose(world.get_physics_dt(), cfg["physics_dt_s"], abs_tol=1e-12)
                or not math.isclose(world.get_rendering_dt(), cfg["rendering_dt_s"], abs_tol=1e-12)):
            raise RuntimeError("actual configured physics/rendering dt differs")
        # Existing OnPlaybackTick/IsaacReadSimulationTime graph; never replace with World counters.
        ros, node, clock_stamps = _clock_graph(stage, app)
        sensors, metadata = runtime.setup_lidars(fleet, robots, stage, Usd, UsdGeom, Gf,
            manager, app, author_mount, verify_mount)
        report["lidars"] = metadata
        for sensor, row in zip(sensors, metadata):
            identity = {key: row[key] for key in ("robot_id", "prim_path", "topic", "frame_id")}
            identity.update(run_id=args.run_id, scenario_id="local45-cadence", side=row["name"],
                            actual_profile_sha256=row["profile_sha256"])
            observers.append(sensor.open_native_observer(identity, row["near_range_m"], row["far_range_m"]))
            subscriptions.append(node.create_subscription(LaserScan, row["topic"],
                lambda msg, t=row["topic"], f=row["frame_id"]: receive_scan(msg, t, f), qos_profile_sensor_data))
        runtime.create_capture_light()
        camera_specs = [("overview", (43., -22., 35.), (16., 11., 0.)),
                        ("south", (18., -5., 5.), (13., 2., .3)),
                        ("north", (12., 27., 5.), (16., 20., .3))]
        for name, eye, target in camera_specs:
            camera = rep.create.camera(position=eye, look_at=target)
            product = rep.create.render_product(camera, (1280, 720), name="Cadence_" + name)
            products.append(product)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgbs.append(rgb)
            rgb.attach([product])
        if len(sensors) != 8 or len(products) != 3:
            raise RuntimeError("expected exactly eight RTX and three persistent RGB products")
        rep.orchestrator.set_capture_on_play(True)
        world.play()
        startup_before = physics_state(world)
        app.update()  # Let Kit initialize the playing PhysX scene before manual simulate/actions.
        startup_after = physics_state(world)
        report["native_startup_update"] = {"before_frame": startup_before[0],
            "after_frame": startup_after[0], "before_time_s": startup_before[1],
            "after_time_s": startup_after[1], "scope": "native warmup, outside the measured interval"}
        if startup_after[0] <= startup_before[0]:
            raise RuntimeError("native startup update did not initialize advancing physics")
        if not world.is_playing():
            raise RuntimeError("World did not enter playing state")
        phase = "warmup"
        stepper = Local45Stepper(world, cfg["physics_dt_s"], cfg["render_every_steps"])
        for _ in range(cfg["warmup_physics_steps"]):
            bound()
            sample = stepper.step()
            for row in robots.values():
                row["robot"].apply_wheel_actions(np.zeros(2, dtype=np.float32))
            if sample["rendered"]:
                observe()
        # Flush pending warmup callbacks before defining the measurement interval.
        observe()
        before = physics_state(world)
        report["measurement_start"] = {"physics_frame": before[0], "sim_time_s": before[1], "robots": poses()}
        measurement_started = time.monotonic()
        phase, measured = "measurement", True
        for _ in range(cfg["measurement_physics_steps"]):
            bound()
            for row in robots.values():
                row["robot"].apply_wheel_actions(np.zeros(2, dtype=np.float32))
            sample = stepper.step()
            sample["robots"] = poses()
            report["physics_samples"].append(sample)
            if sample["rendered"]:
                observe()
        after = physics_state(world)
        report["measurement_delta"] = {"actual_physics_steps": after[0] - before[0],
            "actual_sim_time_s": after[1] - before[1],
            "elapsed_wall_s": time.monotonic() - measurement_started,
            "render_calls_in_measurement": sum(int(r["rendered"]) for r in report["physics_samples"])}
        if after[0] - before[0] != 120 or not math.isclose(after[1] - before[1], 2., rel_tol=1e-7, abs_tol=1e-8):
            raise RuntimeError("measurement actual World delta differs from 120 steps/two seconds")
        measured = False
        phase = "frozen_three_rgb_capture"
        world.pause()
        world.render()
        frozen, frozen_poses = physics_state(world), poses()
        if frozen != after:
            raise RuntimeError("pause advanced physics before capture")
        bound()
        rep.orchestrator.step(delta_time=0., rt_subframes=4, pause_timeline=True)
        if physics_state(world) != frozen or poses() != frozen_poses:
            raise RuntimeError("zero-delta capture changed actual physics time/frame/pose")
        for (name, _, _), rgb in zip(camera_specs, rgbs):
            bound()
            data = np.asarray(rgb.get_data())
            if (data.ndim != 3 or data.shape[:2] != (720, 1280) or data.shape[2] not in (3, 4)
                    or data.dtype != np.uint8 or not np.any(data[:, :, :3])
                    or int(data[:, :, :3].max()) == int(data[:, :, :3].min())):
                raise RuntimeError("RGB buffer empty/invalid: " + name)
            destination = args.output / (name + ".png")
            Image.fromarray(data[:, :, :3]).save(destination)
            if physics_state(world) != frozen or poses() != frozen_poses:
                raise RuntimeError("three-source readback changed actual physics time/frame/pose")
            report["media"].append({"path": destination.name, "sha256": sha(destination),
                "width": 1280, "height": 720, "physics_frame": frozen[0], "sim_time_s": frozen[1]})
        report["frozen_capture"] = {"physics_frame": frozen[0], "sim_time_s": frozen[1],
            "same_physics_time_frame_and_pose": True, "robots": frozen_poses}
        phase = "evaluate_independent_clocks"
        bound()
        report["ros_publishers"] = {topic: [{"node_name": endpoint.node_name,
            "node_namespace": endpoint.node_namespace, "topic_type": endpoint.topic_type}
            for endpoint in node.get_publishers_info_by_topic(topic)]
            for topic in ["/clock"] + [row["topic"] for row in metadata]}
        for topic, publishers in report["ros_publishers"].items():
            if len(publishers) != 1:
                mismatch("expected one isolated publisher", topic=topic, observed=len(publishers))
        summary = {}
        for meta in metadata:
            topic = meta["topic"]
            native = [r for r in report["native_observations"] if r["topic"] == topic and r["status"] == "new"]
            received = report["ros_scans"].get(topic, [])
            scan_period = 1. / float(meta["rotation_frequency_hz"])
            lag_limit = scan_period + cfg["rendering_dt_s"] + cfg["physics_dt_s"]
            offsets = [r["read_minus_sensor_time_s"] for r in native]
            for row in native:
                if (not -cfg["physics_dt_s"] - 1e-6 <= row["read_minus_sensor_time_s"] <= lag_limit + 1e-6
                        or row["range_statistics"]["in_range_count"] == 0):
                    mismatch("native scan/World time or empty valid ranges", topic=topic,
                             native_stamp_s=row["sensor_timestamp_s"], world_minus_native_s=row["read_minus_sensor_time_s"])
            for row in received:
                offset = row["read_at_sim_time_s"] - row["stamp_s"]
                if not -cfg["physics_dt_s"] - 1e-6 <= offset <= lag_limit + 1e-6:
                    mismatch("ROS scan/World time differ", topic=topic, world_minus_ros_s=offset)
            pair_errors = [min(abs(r["stamp_s"] - n["sensor_timestamp_s"]) for n in native)
                           for r in received] if native else []
            if (len(native) < 3 or len(received) < 3 or not pair_errors
                    or max(pair_errors) > scan_period + cfg["physics_dt_s"] + 1e-6):
                mismatch("eight-scan native/ROS progression or timestamp correspondence missing", topic=topic)
            summary[topic] = {"native_unique_observations_in_measurement": len(native),
                "actual_ros_receipts": len(received), "world_minus_native_s": offsets,
                "nearest_native_to_ros_error_s": pair_errors, "maximum_allowed_lag_s": lag_limit,
                "future_tolerance_s": cfg["physics_dt_s"], "range_quality_verified": False}
        report["scan_comparison"] = summary
        positive_clocks = {r["stamp_s"] for r in report["ros_clock_receipts"] if r["stamp_s"] > 0}
        if len(positive_clocks) < 3:
            mismatch("actual increasing ROS clock receipts missing")
        if (project_hashes() != pins or sha(args.config) != report["config_sha256"]
                or sha(ROOT / cfg["fleet_config"]) != report["fleet_config_sha256"]
                or sha(scene) != report["scene_sha256"]
                or sha(Path(__file__)) != report["source_sha256"]):
            raise RuntimeError("source/config/dependency/scene changed during experiment")
        if report["mismatches"]:
            raise RuntimeError("independent clock/sensor comparison failed; no clock substitution applied")
        report["status"], code = "passed", 0
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
            "phase": phase, "traceback": traceback.format_exc(limit=12)})
    finally:
        cleanup_errors = []

        def cleanup(name: str, operation: Any) -> None:
            try:
                operation()
            except BaseException as error:
                cleanup_errors.append(name + ": " + str(error))

        for key, row in robots.items():
            cleanup(key + ".zero", lambda r=row: r["robot"].apply_wheel_actions([0., 0.]))
        if world is not None:
            cleanup("World.pause", world.pause)
        if rep is not None:
            cleanup("capture_on_play_false", lambda: rep.orchestrator.set_capture_on_play(False))
        for sensor in sensors:
            cleanup("RTX.detach", lambda s=sensor: s.detach_writer("RtxLidarROS2PublishLaserScan"))
        for rgb in rgbs:
            cleanup("RGB.detach", rgb.detach)
        for product in products:
            cleanup("RGB.product.destroy", product.destroy)
        if node is not None:
            cleanup("ROS.node.destroy", node.destroy_node)
        if ros is not None and ros.ok():
            cleanup("ROS.shutdown", ros.shutdown)
        report.update(cleanup_errors=cleanup_errors, elapsed_wall_s=time.monotonic() - started,
                      finished_at_utc=datetime.now(timezone.utc).isoformat(), sdk_close_requested=app is not None)
        if cleanup_errors:
            report["status"], code = "failed", 1
        save(args.output / "result.json", report)
        print("LOCAL45_CADENCE_RESULT", report["status"], flush=True)
        if app is not None:
            # Save before fast SDK close; the existing host guard owns exit/descendant closure proof.
            cleanup("SimulationApp.close", app.close)
            if cleanup_errors:
                report["status"], code = "failed", 1
            save(args.output / "result.json", report)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
