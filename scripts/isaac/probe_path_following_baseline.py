#!/usr/bin/env python3
"""Run one unloaded Nova Carter on an encoder-controlled path baseline."""
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


ASSET_PATH = "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd"
ROBOT_PATH = "/World/NovaCarter"
WHEEL_NAMES = ["joint_wheel_left", "joint_wheel_right"]


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def yaw_from_quaternion(value: list[float]) -> tuple[float, float, float]:
    w, x, y, z = (float(item) for item in value)
    norm = math.sqrt(w*w+x*x+y*y+z*z)
    if abs(norm-1.0) > 1e-3:
        raise ValueError("measured pose quaternion is not unit length")
    roll = math.atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y-z*x))))
    yaw = math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
    return roll, pitch, yaw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if (not args.output.is_absolute() or args.output.exists() or args.output.is_symlink()
            or re.fullmatch(r"[0-9a-f]{32}", args.run_id or "") is None):
        parser.error("new absolute output and 32-character lowercase hex run-id required")
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    from factory_ops.motion import DriveGeometry, WheelOdometry
    from factory_ops.path_following_baseline import EncoderPathController, evaluate_ground_truth, validate_config, wrap_angle

    args.output.mkdir(parents=True, exist_ok=False)
    try:
        config = validate_config(json.loads(args.config.read_text(encoding="utf-8")))
    except BaseException as error:
        failure = {
            "schema_version": 1, "run_id": args.run_id, "status": "failed", "stage": "preflight",
            "requires_host_closure": True, "config_path": str(args.config),
            "error": {"type": type(error).__name__, "message": str(error),
                      "traceback": traceback.format_exc(limit=12)},
        }
        save(args.output/"result.json", failure)
        print("PATH_FOLLOWING_BASELINE_RESULT failed", flush=True)
        return 1
    source_files = [
        "factory_ops/path_following_baseline.py",
        "factory_ops/drive_profile.py",
        "factory_ops/motion.py",
        "scripts/isaac/probe_path_following_baseline.py",
    ]
    report = {
        "schema_version": 1,
        "run_id": args.run_id,
        "status": "starting",
        "stage": "preflight",
        "scope": config["scope"],
        "requires_host_closure": True,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "config_sha256": sha256(args.config),
        "source_sha256": {name: sha256(root/name) for name in source_files},
        "control_boundary": "wheel encoder odometry and authored path only; simulator truth is evaluation only",
        "payload_or_contact_verified": False,
        "nav2_verified": False,
        "cleanup_errors": [],
    }
    save(args.output/"result.json", report)
    app = robot = rgb = render_product = None
    action_type = np_module = None
    code = 1
    started = time.monotonic()
    deadline = started + float(config["maximum_wall_time_s"])

    def checkpoint(name: str) -> None:
        report["stage"] = name
        save(args.output/"result.json", report)

    def bounded() -> None:
        if time.monotonic() >= deadline:
            raise TimeoutError("path-following baseline exceeded its wall-time budget")

    try:
        sdk = Path(os.environ["ISAAC_PATH"])
        report["isaac_version"] = (sdk/"VERSION").read_text(encoding="utf-8").strip()
        if not report["isaac_version"].startswith("4.5.0"):
            raise ValueError("this local baseline requires the installed Isaac Sim 4.5.0 SDK")
        checkpoint("native_startup")
        from isaacsim import SimulationApp
        width, height = (int(value) for value in config["image_size_px"])
        app = SimulationApp({"headless": True, "width": width, "height": height,
                             "renderer": "RayTracedLighting", "multi_gpu": False})
        bounded()
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd
        from pxr import UsdLux
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import is_stage_loading
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.storage.native import get_assets_root_path
        action_type, np_module = ArticulationAction, np

        dt = float(config["physics_dt_s"])
        world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt,
                      backend="numpy", device="cpu")
        physics = world.get_physics_context()
        physics.enable_gpu_dynamics(False)
        physics.set_broadphase_type("MBP")
        physics.set_gravity(-9.81)
        world.scene.add_ground_plane(size=30.0, restitution=0.0, static_friction=0.8, dynamic_friction=0.6)
        light = UsdLux.DomeLight.Define(omni.usd.get_context().get_stage(), "/World/BaselineDome")
        light.CreateIntensityAttr(1000.0)
        assets = get_assets_root_path()
        if not isinstance(assets, str) or not assets:
            raise RuntimeError("installed asset resolver returned no root")
        asset_url = assets.rstrip("/") + ASSET_PATH
        robot = world.scene.add(WheeledRobot(
            prim_path=ROBOT_PATH, name="path_baseline_nova_carter", wheel_dof_names=WHEEL_NAMES,
            create_robot=True, usd_path=asset_url, position=np.asarray(config["spawn_xyz_m"], dtype=np.float32),
        ))
        checkpoint("asset_loading")
        quiet = 0
        load_deadline = min(deadline, time.monotonic()+90.0)
        for _ in range(20_000):
            bounded()
            if time.monotonic() >= load_deadline or not app.is_running():
                raise TimeoutError("Nova Carter composition/load deadline exceeded")
            app.update()
            quiet = 0 if is_stage_loading() else quiet+1
            if quiet >= 3:
                break
        else:
            raise TimeoutError("asset update budget exhausted")

        checkpoint("physics_warmup")
        world.reset()
        zero = ArticulationAction(joint_velocities=np.zeros(2, dtype=np.float32))
        robot.apply_wheel_actions(zero)
        for _ in range(int(config["warmup_steps"])):
            bounded()
            world.step(render=True)
        checkpoint("rgb_graph_warmup")
        camera = rep.create.camera(position=(2.0, -6.0, 7.0), look_at=(2.0, 1.0, 0.2))
        render_product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([render_product])
        for _ in range(36):
            bounded()
            robot.apply_wheel_actions(zero)
            world.step(render=True)

        def rgb_diagnostic(data) -> dict:
            shape = [int(value) for value in data.shape]
            spatial_std = (float(data[:, :, :3].std())
                           if data.ndim == 3 and data.shape[2] >= 3 and data.size else None)
            return {"shape": shape, "dtype": str(data.dtype), "spatial_std_rgb": spatial_std,
                    "expected_shape_yx": [height, width], "expected_dtype": "uint8"}

        warmup_rgb = np.asarray(rgb.get_data())
        report["rgb_graph_warmup"] = rgb_diagnostic(warmup_rgb)
        if (warmup_rgb.ndim != 3 or warmup_rgb.shape[:2] != (height, width)
                or warmup_rgb.shape[2] not in (3, 4) or warmup_rgb.dtype != np.uint8
                or report["rgb_graph_warmup"]["spatial_std_rgb"] is None
                or report["rgb_graph_warmup"]["spatial_std_rgb"] < 1.0):
            raise RuntimeError("genuine RGB unavailable after 36-step render-graph warmup")

        # Establish both control and evaluation origins only after media graph
        # startup, so those extra stationary physics steps are outside the run.
        authored = config["drive_geometry"]
        geometry = DriveGeometry(float(authored["wheel_radius_m"]), float(authored["wheel_separation_m"]))
        odometry = WheelOdometry(geometry)
        wheel_position = [float(value) for value in robot.get_wheel_positions()]
        if len(wheel_position) != 2 or not all(math.isfinite(value) for value in wheel_position):
            raise ValueError("initial native wheel positions are invalid")
        initial_stamp = float(world.current_time)
        initial_odom = odometry.update(initial_stamp, *wheel_position)
        if (initial_odom.x_m, initial_odom.y_m, initial_odom.yaw_rad) != (0.0, 0.0, 0.0):
            raise RuntimeError("wheel odometry failed to establish its zero frame")
        initial_position, initial_orientation = robot.get_world_pose()
        initial_position = [float(value) for value in initial_position]
        initial_orientation = [float(value) for value in initial_orientation]
        initial_roll, initial_pitch, initial_yaw = yaw_from_quaternion(initial_orientation)
        report["frame_alignment"] = {
            "odom_zero": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "ground_truth_origin_world": {"position_xyz_m": initial_position,
                                          "orientation_wxyz": initial_orientation},
            "ground_truth_use": "evaluation frame origin only; never supplied to controller",
        }
        controller = EncoderPathController(config, geometry)
        media_dir = args.output/"media"
        media_dir.mkdir()
        observations_path = args.output/"observations.jsonl"
        frames: list[dict] = []
        samples: list[dict] = []
        maximum_steps = math.ceil(float(config["maximum_sim_time_s"])/dt)
        capture_period = int(config["capture_every_physics_steps"])
        post_stop_steps = math.ceil(float(config["controller"]["post_stop_observation_s"])/dt)
        accepted_step: int | None = None
        previous_odom = initial_odom
        checkpoint("encoder_path_following")

        with observations_path.open("x", encoding="utf-8", newline="\n") as stream:
            for step in range(1, maximum_steps+1):
                bounded()
                control = controller.step(float(world.current_time), previous_odom, dt)
                targets = np.asarray(control["command"]["wheel_targets_rad_s"], dtype=np.float32)
                robot.apply_wheel_actions(ArticulationAction(joint_velocities=targets))
                due = step % capture_period == 0
                world.step(render=due)
                stamp = float(world.current_time)
                wheel_position = [float(value) for value in robot.get_wheel_positions()]
                odom = odometry.update(stamp, *wheel_position)
                position, orientation = robot.get_world_pose()
                position = [float(value) for value in position]
                orientation = [float(value) for value in orientation]
                roll, pitch, yaw = yaw_from_quaternion(orientation)
                world_dx, world_dy = position[0]-initial_position[0], position[1]-initial_position[1]
                gt_x = math.cos(initial_yaw)*world_dx + math.sin(initial_yaw)*world_dy
                gt_y = -math.sin(initial_yaw)*world_dx + math.cos(initial_yaw)*world_dy
                linear_velocity = [float(value) for value in robot.get_linear_velocity()]
                angular_velocity = [float(value) for value in robot.get_angular_velocity()]
                forward_speed = (math.cos(yaw)*linear_velocity[0] + math.sin(yaw)*linear_velocity[1])
                sample = {
                    "sample_index": step-1,
                    "physics_frame": int(world.current_time_step_index),
                    "sim_time_s": stamp,
                    "wheel_positions_rad": wheel_position,
                    "wheel_velocities_rad_s": [float(value) for value in robot.get_joint_velocities()[robot.wheel_dof_indices]],
                    "wheel_targets_rad_s": [float(value) for value in targets],
                    "odometry": {"x_m": odom.x_m, "y_m": odom.y_m, "yaw_rad": odom.yaw_rad,
                                 "linear_m_s": odom.linear_m_s, "angular_rad_s": odom.angular_rad_s},
                    "controller": control,
                    "ground_truth": {
                        "x_m": gt_x, "y_m": gt_y, "yaw_rad": wrap_angle(yaw-initial_yaw),
                        "roll_rad": wrap_angle(roll-initial_roll), "pitch_rad": wrap_angle(pitch-initial_pitch),
                        "forward_speed_m_s": forward_speed, "yaw_rate_rad_s": angular_velocity[2],
                        "use": "independent evaluation and media only; never controller or goal input",
                    },
                }
                stream.write(json.dumps(sample, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n")
                stream.flush()
                samples.append(sample)
                if due:
                    data = np.asarray(rgb.get_data())
                    report["last_rgb_diagnostic"] = rgb_diagnostic(data)
                    if (data.ndim != 3 or data.shape[:2] != (height, width) or data.shape[2] not in (3, 4)
                            or data.dtype != np.uint8
                            or report["last_rgb_diagnostic"]["spatial_std_rgb"] is None
                            or report["last_rgb_diagnostic"]["spatial_std_rgb"] < 1.0):
                        raise RuntimeError("genuine RGB frame unavailable")
                    frame_path = media_dir/("frame-%04d.png" % len(frames))
                    Image.fromarray(data[:, :, :3]).save(frame_path)
                    frames.append({"path": str(frame_path.relative_to(args.output)).replace("\\", "/"),
                                   "sha256": sha256(frame_path), "physics_frame": sample["physics_frame"],
                                   "sim_time_s": stamp})
                    if len(frames) > 180:
                        raise RuntimeError("media frame budget exceeded")
                if controller.complete and accepted_step is None:
                    accepted_step = step
                if accepted_step is not None and step-accepted_step >= post_stop_steps:
                    break
                previous_odom = odom
            else:
                raise TimeoutError("encoder controller did not converge within simulation budget")

        robot.apply_wheel_actions(zero)
        if len(samples) > 10_000:
            raise RuntimeError("observation row budget exceeded")
        gt_evaluation = evaluate_ground_truth(samples, config)
        encoder_goal = samples[accepted_step-1]["controller"]["goal"] if accepted_step is not None else None
        report.update({
            "status": "passed" if controller.complete and gt_evaluation["passed"] else "failed",
            "stage": "complete",
            "encoder_goal": encoder_goal,
            "ground_truth_evaluation": gt_evaluation,
            "trajectory": {"path": observations_path.name, "rows": len(samples), "maximum_rows": 10_000},
            "media": {"frames": frames, "frame_count": len(frames), "maximum_frames": 180,
                      "source": "actual Isaac Sim RGB frames; no interpolation or duplication"},
            "drive_geometry": {"authored": config["drive_geometry"],
                               "provenance": "current G2 authored Nova Carter geometry; no motor calibration claim"},
            "drive_limits": {**config["drive_limits"],
                             "provenance": "current G2 authored simulation command envelope"},
            "goal_acceptance_input": "wheel encoder odometry only",
            "payload_or_contact_verified": False,
        })
        if report["status"] != "passed":
            raise RuntimeError("independent ground-truth path evaluation failed")
        code = 0
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
                      "traceback": traceback.format_exc(limit=12)})
    finally:
        if robot is not None and action_type is not None and np_module is not None:
            try:
                robot.apply_wheel_actions(action_type(joint_velocities=np_module.zeros(2, dtype=np_module.float32)))
                report["final_zero_requested"] = True
            except BaseException as error:
                report["cleanup_errors"].append("wheel_zero: "+str(error))
        if rgb is not None:
            try:
                rgb.detach()
            except BaseException as error:
                report["cleanup_errors"].append("rgb_detach: "+str(error))
        if render_product is not None:
            try:
                render_product.destroy()
            except BaseException as error:
                report["cleanup_errors"].append("render_product_destroy: "+str(error))
        if report["cleanup_errors"]:
            report["status"], code = "failed", 1
        report.update(finished_at_utc=datetime.now(timezone.utc).isoformat(),
                      elapsed_wall_s=time.monotonic()-started, app_close_requested=app is not None,
                      requires_host_closure=True)
        try:
            save(args.output/"result.json", report)
            print("PATH_FOLLOWING_BASELINE_RESULT", report["status"], flush=True)
        finally:
            if app is not None:
                app.close(wait_for_replicator=False)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
