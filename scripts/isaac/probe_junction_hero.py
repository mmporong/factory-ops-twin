"""One-camera native factory logistics/yield capture; no ROS or GT steering."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
DEPENDENCIES = (
    "factory_ops/__init__.py", "factory_ops/junction_demo.py", "factory_ops/motion.py",
    "factory_ops/drive_profile.py", "factory_ops/traffic.py", "factory_ops/layout.py",
    "factory_ops/usda.py", "factory_ops/visual_scene.py", "factory_ops/visual_state.py",
    "factory_ops/physx_planar.py", "scripts/isaac/factory_visuals.py", "scripts/isaac/local_asset_probe.py",
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if (not args.output.is_absolute() or args.output.exists() or args.output.is_symlink()
            or not re.fullmatch("[0-9a-f]{32}", args.run_id)):
        parser.error("new absolute output and lowercase hex run ID required")
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    folder = ROOT/cfg["factory_config"]
    args.output.mkdir(parents=True)
    (args.output/"rgb").mkdir()
    started = time.monotonic()
    report = {"run_id": args.run_id, "status": "starting", "stage": "setup",
        "source_sha256": sha(__file__), "config_sha256": sha(args.config),
        "project_source_sha256": {p: sha(ROOT/p) for p in DEPENDENCIES},
        "factory_input_sha256": {p: sha(folder/p) for p in ("layout.json", "visual-assets.json", "render-profile.json")},
        "scope": cfg["scope"], "requires_host_closure": True, "cleanup_errors": [],
        "started_at_utc": datetime.now(timezone.utc).isoformat(), "media": [],
        "runtime_pose_or_velocity_setters": False, "ground_truth_control": False,
        "cargo_physical_loading": False, "nav2_integration": False,
        "control_inputs": ["native wheel joint angles", "authored map/spawn", "native PhysX front ranges"],
        "media_clock": "RGB buffer read after one World.step at unchanged World frame; no hardware timestamp claim"}
    app = rgb = product = None
    robots, sensors = {}, {}
    code = 1

    def save():
        temporary = args.output/"result.tmp"
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        temporary.replace(args.output/"result.json")

    def bound():
        if time.monotonic()-started > cfg["maximum_wall_time_s"]:
            raise TimeoutError("junction capture wall deadline exceeded")

    save()
    try:
        from isaacsim import SimulationApp
        width, height = cfg["image_size_px"]
        app = SimulationApp({"headless": True, "width": width, "height": height,
            "multi_gpu": False, "renderer": "RayTracedLighting"})
        import numpy as np
        import omni.usd
        import omni.kit.commands
        import omni.replicator.core as rep
        from PIL import Image
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.stage import is_stage_loading
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Gf, UsdGeom, UsdPhysics
        from factory_ops.junction_demo import JunctionDemo
        from factory_ops.motion import DriveGeometry, WheelOdometry
        from factory_ops.drive_profile import DriveProfile, DriveLimits
        from factory_ops.layout import Layout
        from factory_ops.usda import export_usda
        from factory_ops.physx_planar import LIDAR_PARAMETERS
        from scripts.isaac.factory_visuals import FactoryVisualRuntime
        from scripts.isaac.local_asset_probe import ASSET_PATH

        enable_extension("isaacsim.sensors.physx")
        from isaacsim.sensors.physx import _range_sensor
        interface = _range_sensor.acquire_lidar_sensor_interface()
        layout = Layout.from_file(folder/"layout.json")
        base = args.output/"factory-v3.usda"
        export_usda(layout, base)
        context = omni.usd.get_context()
        context.open_stage(str(base))
        while context.get_stage_loading_status()[2] > 0:
            bound()
            app.update()
        stage = context.get_stage()
        dt = cfg["physics_dt_s"]
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt, backend="numpy", device="cpu")
        physics = world.get_physics_context()
        physics.enable_gpu_dynamics(False)
        physics.set_broadphase_type("MBP")
        visuals = FactoryVisualRuntime(folder/"layout.json", base, args.output/"visuals", 0,
            args.run_id, "junction-hero", config_directory=folder)
        visuals.setup(stage, UsdGeom, UsdPhysics, Gf)
        demo = JunctionDemo(args.run_id)
        asset = get_assets_root_path().rstrip("/")+ASSET_PATH
        report["asset_url"] = asset
        geometry = DriveGeometry(.14, .4132)
        report["drive_geometry"] = {**asdict(geometry), "source": "existing Carter collider-derived expected geometry; no wheel calibration claim"}
        palette = [(0.02, .50, .90), (1., .55, .03), (.07, .65, .45), (.65, .25, .8)]
        for (robot_id, route), color in zip(demo.routes.items(), palette):
            path = "/World/"+robot_id
            x, y, yaw = route.spawn
            robot = world.scene.add(WheeledRobot(prim_path=path, name=robot_id,
                wheel_dof_names=["joint_wheel_left", "joint_wheel_right"], create_robot=True, usd_path=asset,
                position=np.array([x, y, .15]), orientation=np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)])))
            robots[robot_id] = {"robot": robot, "odom": WheelOdometry(geometry),
                "drive": DriveProfile(geometry, DriveLimits()), "range": 30., "range_frame": -1}
        quiet = 0
        while quiet < 3:
            bound()
            app.update()
            quiet = 0 if is_stage_loading() else quiet+1
        # These colored totes identify logistics roles only; they add no collider or mass.
        for (robot_id, runtime), color in zip(robots.items(), palette):
            path = "/World/"+robot_id+"/chassis_link"
            cube = UsdGeom.Cube.Define(stage, path+"/PresentationTote")
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(0., 0., .44))
            cube.AddScaleOp().Set(Gf.Vec3d(.44, .34, .18))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            sensor_path = path+"/JunctionFrontRange"
            ok, schema = omni.kit.commands.execute("RangeSensorCreateLidar", path=sensor_path,
                parent=None, **dict(LIDAR_PARAMETERS))
            if not ok or not schema:
                raise RuntimeError("owned front range sensor creation failed")
            xf = UsdGeom.Xformable(schema.GetPrim())
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(.40, 0., .18))
            sensors[robot_id] = sensor_path
        world.reset()
        for runtime in robots.values():
            robot = runtime["robot"]
            if not robot.handles_initialized or len(robot.wheel_dof_indices) != 2:
                raise RuntimeError("native wheel handles unavailable")
            runtime["indices"] = np.asarray(robot.wheel_dof_indices, dtype=np.int64)
        camera = rep.create.camera(position=cfg["camera_eye_m"], look_at=cfg["camera_target_m"],
            focal_length=cfg["camera_focal_length_mm"])
        product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        for _ in range(36):
            for runtime in robots.values():
                runtime["robot"].apply_wheel_actions(ArticulationAction(joint_velocities=np.zeros(2)))
            world.step(render=True)
            bound()
        origin = float(world.current_time)
        for runtime in robots.values():
            angles = runtime["robot"].get_joint_positions(joint_indices=runtime["indices"])
            runtime["odom"].update(origin, *map(float, angles))
        stage.GetRootLayer().Export(str(args.output/"scene.usda"))
        report.update(stage="logistics_running", physics_start_s=origin, source_rgb_fps=60,
            renderer_readback=visuals.runtime_manifest.get("renderer") if visuals.runtime_manifest else None)
        save()
        minimum_separation = float("inf")
        completed_at = None
        with (args.output/"observations.jsonl").open("x", encoding="utf-8") as observations:
            for step in range(math.ceil(cfg["maximum_sim_time_s"]/dt)):
                bound()
                stamp, frame = float(world.current_time), int(world.current_time_step_index)
                now = stamp-origin
                poses = {rid: demo.map_pose(demo.routes[rid].spawn, row["odom"].sample) for rid, row in robots.items()}
                demo.observe(poses, now)
                commands = {}
                for rid, runtime in robots.items():
                    if step % 6 == 0:
                        path = sensors[rid]
                        ranges = np.asarray(interface.get_linear_depth_data(path)).reshape(-1)
                        angles = np.asarray(interface.get_azimuth_data(path)).reshape(-1)
                        if ranges.size != angles.size or ranges.size != 720 or not np.isfinite(ranges).all():
                            raise RuntimeError("native front range acquisition shape/values unavailable")
                        sector = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) < .26
                        runtime["range"] = float(ranges[sector].min())
                        runtime["range_frame"] = frame
                    stale = frame-runtime["range_frame"] > 6
                    v, w, emergency = demo.command(rid, poses[rid], now, runtime["range"] if not stale else float("nan"))
                    command = runtime["drive"].step(v, w, dt, emergency_stop=emergency)
                    runtime["robot"].apply_wheel_actions(ArticulationAction(joint_velocities=np.asarray(command.wheel_targets_rad_s)))
                    commands[rid] = asdict(command)
                world.step(render=True)
                bound()
                after, after_frame = float(world.current_time), int(world.current_time_step_index)
                if after_frame != frame+1 or abs(after-stamp-dt) > 1e-6:
                    raise RuntimeError("World physics cadence mismatch")
                truth = {}
                for rid, runtime in robots.items():
                    robot = runtime["robot"]
                    angles = robot.get_joint_positions(joint_indices=runtime["indices"])
                    encoder = runtime["odom"].update(after, *map(float, angles))
                    p, q = robot.get_world_pose()
                    truth[rid] = [float(v) for v in p]
                    if not all(math.isfinite(v) for v in truth[rid]):
                        raise RuntimeError("nonfinite physical evaluation pose")
                for i, a in enumerate(truth):
                    for b in list(truth)[i+1:]:
                        separation = math.hypot(truth[a][0]-truth[b][0], truth[a][1]-truth[b][1])
                        minimum_separation = min(minimum_separation, separation)
                        if separation < 2*demo.radius_m:
                            raise RuntimeError("evaluation: physical planning envelopes overlap")
                data = np.asarray(rgb.get_data())
                if data.shape[:2] != (height, width) or data.dtype != np.uint8 or data[:, :, :3].std() < 1.:
                    raise RuntimeError("native RGB unavailable")
                image_path = args.output/"rgb"/("frame-%05d.jpg" % step)
                Image.fromarray(data[:, :, :3]).save(image_path, quality=94, subsampling=0)
                report["media"].append({"path": image_path.relative_to(args.output).as_posix(),
                    "sha256": sha(image_path), "sim_time_s": after-origin, "physics_frame": after_frame})
                if float(world.current_time) != after or int(world.current_time_step_index) != after_frame:
                    raise RuntimeError("RGB read advanced physics")
                if step % 6 == 0:
                    observations.write(json.dumps({"sim_time_s": after-origin, "physics_frame": after_frame,
                        "states": demo.states, "estimated_poses": poses, "commands": commands,
                        "front_ranges_m": {r: row["range"] for r, row in robots.items()},
                        "evaluation_only_world_positions": truth}, allow_nan=False)+"\n")
                if step % 60 == 0:
                    observations.flush()
                    report.update(progress_sim_s=after-origin, captured_frames=len(report["media"]),
                        traffic=demo.result(), minimum_evaluation_center_distance_m=minimum_separation)
                    save()
                if all(r.finished for r in demo.routes.values()):
                    completed_at = after if completed_at is None else completed_at
                    if after-completed_at >= 1.:
                        break
        result = demo.result()
        report.update(traffic=result, minimum_evaluation_center_distance_m=minimum_separation,
            captured_frames=len(report["media"]), progress_sim_s=float(world.current_time)-origin)
        if not result["yield_clear_resume_observed"] or not result["all_arrived"]:
            raise RuntimeError("native logistics/yield sequence did not complete within authored bound")
        report.update(status="passed", stage="logistics_complete")
        code = 0
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
            "traceback": traceback.format_exc(limit=10)})
    finally:
        for rid, runtime in robots.items():
            try:
                runtime["robot"].apply_wheel_actions(ArticulationAction(joint_velocities=np.zeros(2)))
            except Exception as error:
                report["cleanup_errors"].append(rid+": "+str(error))
        for label, resource, method in (("rgb", rgb, "detach"), ("render_product", product, "destroy")):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as error:
                    report["cleanup_errors"].append(label+": "+str(error))
        if report["cleanup_errors"]:
            report["status"], code = "failed", 1
        report.update(elapsed_wall_s=time.monotonic()-started, finished_at_utc=datetime.now(timezone.utc).isoformat())
        save()
        print("JUNCTION_HERO_RESULT", report["status"], flush=True)
        if app is not None:
            app.close(wait_for_replicator=False)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
