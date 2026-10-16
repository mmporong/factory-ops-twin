#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""G0b: qualify one Nova Carter, a camera, and bounded open-loop motion."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import traceback
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout
from scripts.isaac import stage_smoke as base
from scripts.isaac.robot_evidence import validate_robot_evidence

ROBOT_PATH = "/World/Carter"
WHEEL_JOINTS = ["joint_wheel_left", "joint_wheel_right"]
WARMUP_STEPS = 120
MEASUREMENT_STEPS = 600
CAPTURE_PERIOD = 6
SENSOR_SIZE = (640, 360)
OPTICAL_FRAME = "carter/camera_optical"


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=800.0)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--run-id", help=argparse.SUPPRESS)
    return parser.parse_args()


def _json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _capture_factory_overview(output: Path, app: Any, rep: Any) -> dict[str, Any]:
    capture = output / "final-overview"
    capture.mkdir()
    rgb = base._capture_overview(capture, app, rep, create_light=False)
    destination = output / base.IMAGE_FILENAME
    shutil.move(str(capture / base.IMAGE_FILENAME), destination)
    rgb["path"] = str(destination)
    return rgb


def _collision_bounds(stage: Any, Usd: Any, UsdGeom: Any, UsdPhysics: Any) -> tuple[list[list[float]], list[dict[str, Any]]]:
    from pxr import Gf
    root = stage.GetPrimAtPath(ROBOT_PATH)
    if not root.IsValid():
        raise base.SmokeError("Nova Carter reference did not produce a valid prim")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    geometry_sources: dict[str, tuple[Any, Any]] = {}
    for collider in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not collider.HasAPI(UsdPhysics.CollisionAPI) or collider.GetAttribute("physics:collisionEnabled").Get() is False:
            continue
        candidates = [prim for prim in Usd.PrimRange(collider, Usd.TraverseInstanceProxies()) if prim.IsA(UsdGeom.Boundable)]
        if not candidates:
            raise base.SmokeError(f"enabled collision prim contains no boundable geometry: {collider.GetPath()}")
        for prim in candidates:
            cursor = prim
            disabled = False
            while cursor.IsValid() and cursor != collider:
                if cursor.HasAPI(UsdPhysics.CollisionAPI) and cursor.GetAttribute("physics:collisionEnabled").Get() is False:
                    disabled = True
                    break
                cursor = cursor.GetParent()
            if not disabled:
                geometry_sources[str(prim.GetPath())] = (prim, collider)
    records = []
    for prim, collider in geometry_sources.values():
        geometry = UsdGeom.Boundable(prim)
        extent = UsdGeom.Boundable.ComputeExtentFromPlugins(geometry, Usd.TimeCode.Default())
        if extent is None or len(extent) != 2:
            raise base.SmokeError(f"could not compute collision geometry extent: {prim.GetPath()}")
        transform, resets = cache.ComputeRelativeTransform(prim, root)
        if resets:
            raise base.SmokeError(f"robot collider resets the root transform stack: {prim.GetPath()}")
        bound = Gf.BBox3d(Gf.Range3d(Gf.Vec3d(*extent[0]), Gf.Vec3d(*extent[1])), transform).ComputeAlignedRange()
        if bound.IsEmpty():
            raise base.SmokeError(f"empty robot collider bound: {prim.GetPath()}")
        authored = geometry.GetExtentAttr().Get()
        records.append({"path": str(prim.GetPath()), "min": list(bound.GetMin()), "max": list(bound.GetMax()),
                        "collision_api_path": str(collider.GetPath()),
                        "extent_method": "ComputeExtentFromPlugins and root-relative XformCache transform",
                        "geometry_extent": [list(point) for point in extent],
                        "authored_extent": [list(point) for point in authored] if authored is not None else None})
    if not records:
        raise base.SmokeError("Nova Carter has no enabled collision geometry")
    bounds = [[min(record["min"][i] for record in records) for i in range(3)],
              [max(record["max"][i] for record in records) for i in range(3)]]
    if not all(math.isfinite(value) for point in bounds for value in point):
        raise base.SmokeError("robot collision bounds contain non-finite values")
    return bounds, records


def _clock_graph(stage: Any, app: Any) -> tuple[Any, Any, list[float]]:
    import omni.graph.core as og
    import omni.kit.app
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    app.update()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    og.Controller.edit(
        {"graph_path": "/World/ROSClock", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("Context.outputs:context", "Clock.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [("Clock.inputs:topicName", "/clock"),
                                           ("Context.inputs:domain_id", int(os.environ["ROS_DOMAIN_ID"]))],
        },
    )
    rclpy.init()
    node = rclpy.create_node("factory_g0b_clock_observer")
    stamps: list[float] = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscription = node.create_subscription(
        Clock, "/clock", lambda msg: stamps.append(msg.clock.sec + msg.clock.nanosec / 1e9), qos,
    )
    # Keep a strong subscription reference until the node is destroyed.
    node._factory_clock_subscription = subscription
    return rclpy, node, stamps


def _camera(stage: Any, Gf: Any, UsdGeom: Any, path: str, eye: tuple[float, ...], target: tuple[float, ...], size: tuple[int, int]) -> Any:
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateHorizontalApertureAttr(24.0)
    cam.CreateVerticalApertureAttr(24.0 * size[1] / size[0])
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    pose = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(pose)
    return cam


def _capture_motion(output: Path, app: Any, stage: Any, robot: Any, rep: Any, np: Any, UsdGeom: Any, Gf: Any, SimulationManager: Any, rclpy: Any, node: Any) -> dict[str, Any]:
    from PIL import Image, ImageDraw
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    (output / "media").mkdir()
    (output / "sensors").mkdir()
    medium = _camera(stage, Gf, UsdGeom, "/World/RobotMotionCamera", (8, -5, 8), (8, 4, 0.2), base.RENDER_RESOLUTION)
    camera = _camera(stage, Gf, UsdGeom, ROBOT_PATH + "/chassis_link/QualificationCamera", (0.35, 0, 0.65), (4, 0, 0.5), SENSOR_SIZE)
    products = [rep.create.render_product(str(medium.GetPath()), base.RENDER_RESOLUTION),
                rep.create.render_product(str(camera.GetPath()), SENSOR_SIZE)]
    medium_rgb = rep.annotators.get("rgb")
    medium_rgb.attach(products[0])
    annotators = {name: rep.annotators.get(name) for name in ["rgb", "distance_to_image_plane", "semantic_segmentation", "bounding_box_2d_tight", "camera_params"]}
    for annotator in annotators.values():
        annotator.attach(products[1])
    trajectory: list[dict[str, Any]] = []
    frames: list[dict[str, Any]] = []
    sensor_frames: list[dict[str, Any]] = []
    gif_images: list[Any] = []

    def pose() -> tuple[list[float], list[float]]:
        positions, orientations = robot.get_world_poses()
        return positions.numpy()[0].tolist(), orientations.numpy()[0].tolist()

    def observe(targets: list[float]) -> dict[str, Any]:
        position, orientation = pose()
        sample = {"physics_frame": int(SimulationManager.get_num_physics_steps()),
                  "sim_time": float(SimulationManager.get_simulation_time()),
                  "position": position, "orientation_wxyz": orientation,
                  "wheel_target_rad_s": targets}
        trajectory.append(sample)
        return sample

    start_sim = float(SimulationManager.get_simulation_time())
    started = time.monotonic()
    robot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
    observe([0.0, 0.0])
    try:
        for step in range(MEASUREMENT_STEPS):
            targets = [2.0, 2.0] if step < 480 else [0.0, 0.0]
            robot.apply_wheel_actions(np.array(targets, dtype=np.float32))
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            sample = observe(targets)
            if (step + 1) % CAPTURE_PERIOD:
                continue
            timeline.pause()
            before = float(SimulationManager.get_simulation_time())
            base._with_alarm(120, "G0b sensor capture", lambda: rep.orchestrator.step(delta_time=0.0))
            after = float(SimulationManager.get_simulation_time())
            after_position, _ = pose()
            if abs(after - before) > 1e-8 or any(abs(a-b) > 1e-5 for a,b in zip(after_position, sample["position"])):
                raise base.SmokeError("sensor capture advanced physics while the frame was frozen")
            index = len(frames)
            medium_data = np.asarray(medium_rgb.get_data())[:, :, :3]
            medium_image = Image.fromarray(medium_data.astype(np.uint8))
            path = f"media/frame-{index:04d}.png"
            medium_image.save(output / path)
            preview = medium_image.resize((640, 360))
            draw = ImageDraw.Draw(preview)
            draw.rectangle((0, 0, 640, 20), fill="white")
            draw.text((6, 4), f"Nova Carter | sim {sample['sim_time']:.3f} s | frame {index:03d} | open-loop qualification", fill="black")
            gif_images.append(preview.convert("P", palette=Image.Palette.ADAPTIVE))
            preview.close()
            medium_image.close()
            frames.append({"index": index, "physics_frame": sample["physics_frame"], "sim_time": sample["sim_time"],
                           "path": path, "pose_xyz": sample["position"]})
            if index in (0, 49, 99):
                rgb = np.asarray(annotators["rgb"].get_data())[:, :, :3]
                depth = np.asarray(annotators["distance_to_image_plane"].get_data()).astype("<f4").reshape(360, 640)
                segmentation = annotators["semantic_segmentation"].get_data()
                boxes = annotators["bounding_box_2d_tight"].get_data()
                params = annotators["camera_params"].get_data()
                prefix = f"sensors/frame-{index:04d}"
                paths = {"rgb": prefix+".png", "depth": prefix+"-depth.npy", "semantic": prefix+"-semantic.npy",
                         "boxes": prefix+"-boxes.json", "camera": prefix+"-camera.json"}
                Image.fromarray(rgb.astype(np.uint8)).save(output / paths["rgb"])
                np.save(output / paths["depth"], depth, allow_pickle=False)
                np.save(output / paths["semantic"], np.asarray(segmentation["data"]).astype("<u4").reshape(360, 640), allow_pickle=False)
                labels = {str(k): v for k,v in segmentation["info"]["idToLabels"].items()}
                labels.update({str(k): v for k,v in boxes["info"]["idToLabels"].items()})
                box_records = [{"semantic_id": int(box["semanticId"]), "x_min": int(box["x_min"]),
                                "y_min": int(box["y_min"]), "x_max": int(box["x_max"]), "y_max": int(box["y_max"])}
                               for box in boxes["data"]]
                _json(output / paths["boxes"], {"boxes": box_records, "id_to_labels": labels})
                view = np.asarray(params["cameraViewTransform"]).reshape(4, 4)
                optical_to_world = np.diag([1.0, -1.0, -1.0, 1.0]) @ np.linalg.inv(view)
                optical_quaternion = Gf.Matrix4d(*optical_to_world.flatten().tolist()).ExtractRotationQuat()
                optical_wxyz = [float(optical_quaternion.GetReal()), *[float(value) for value in optical_quaternion.GetImaginary()]]
                fx = 18.0 / 24.0 * SENSOR_SIZE[0]
                _json(output / paths["camera"], {"camera_intrinsics": [fx, 0, 320, 0, fx, 180, 0, 0, 1],
                      "camera_to_world": optical_to_world.flatten().tolist(), "sim_time": sample["sim_time"],
                      "pose_xyz": optical_to_world[3, :3].tolist(), "pose_wxyz": optical_wxyz,
                      "optical_frame": OPTICAL_FRAME, "matrix_convention": "row-vector optical +X right, +Y down, +Z forward",
                      "depth_semantics": "distance_to_image_plane in metres", "pixel_origin": "top-left"})
                sensor_frames.append({"physics_frame": sample["physics_frame"], "sim_time": sample["sim_time"],
                                      "pose_xyz": sample["position"], "pose_wxyz": sample["orientation_wxyz"], "paths": paths})
            timeline.play()
        wall = time.monotonic() - started
        sim = float(SimulationManager.get_simulation_time()) - start_sim
        timeline.pause()
        base._with_alarm(120, "G0b writer drain", rep.orchestrator.wait_until_complete)
        gif_images[0].save(output / "media/amr-motion.gif", save_all=True, append_images=gif_images[1:], duration=100, loop=0)
        with Image.open(output / "media/amr-motion.gif") as animation:
            if animation.n_frames != len(frames) or animation.size != (640, 360) or animation.info.get("loop") != 0:
                raise base.SmokeError("GIF encoder did not retain every captured frame")
            for frame in range(animation.n_frames):
                animation.seek(frame)
                animation.load()
                if animation.info.get("duration") != 100:
                    raise base.SmokeError("GIF frame duration differs from the raw frame cadence")
        shutil.copyfile(output / frames[0]["path"], output / "carter-closeup.png")
        _json(output / "media/frames.json", {"fps": 10, "frames": frames, "source": "actual Isaac Sim render products; no interpolation",
              "gif_derivation": "640x360 resize and measured sim-time/frame-index overlay; original PNGs retained"})
        _json(output / "sensor-frames.json", {"width": 640, "height": 360, "optical_frame": OPTICAL_FRAME, "frames": sensor_frames})
        _json(output / "trajectory.json", {"samples": trajectory, "pose_source": "Isaac articulation simulation ground truth; evaluation only"})
        displacement = math.dist(trajectory[0]["position"][:2], trajectory[-1]["position"][:2])
        gpu = subprocess.check_output(["nvidia-smi", "--query-gpu=name,memory.used", "--format=csv,noheader,nounits"], text=True).strip()
        return {"motion": {"kind": "open_loop_wheel_velocity", "trajectory_file": "trajectory.json", "displacement_m": displacement,
                           "nav2": "unmeasured", "physical_contact_count": None, "scope": "bounded forward wheel command; no Nav2 goal or safety claim"},
                "performance": {"sim_seconds": sim, "wall_seconds": wall, "rtf": sim/wall, "warmup_steps": WARMUP_STEPS,
                                "measurement_loop_iterations": MEASUREMENT_STEPS, "capture_included": True,
                                "gif_encoding_included": False, "gpu_observation": gpu}}
    finally:
        active_error = sys.exception()
        cleanup_errors = []
        actions = [*[image.close for image in gif_images], medium_rgb.detach,
                   *[annotator.detach for annotator in annotators.values()], *[product.destroy for product in products]]
        for action in actions:
            try:
                action()
            except BaseException as error:
                cleanup_errors.append(f"{type(error).__name__}: {error}")
        if cleanup_errors:
            message = "sensor cleanup failed: " + "; ".join(cleanup_errors)
            if active_error is not None:
                active_error.add_note(message)
            else:
                raise base.SmokeError(message)


def _worker(args: argparse.Namespace) -> int:
    started = time.monotonic()
    app = None
    output = None
    phase = "preflight"
    result: dict[str, Any] = {"schema_version": 2, "run_id": args.run_id, "qualification": "g0b",
                             "primary_evaluation": False, "status": "running", "cleanup": "pending"}
    rclpy = node = None
    valid = False
    try:
        if not re.fullmatch(r"[0-9a-f]{32}", args.run_id or ""):
            raise base.SmokeError("worker requires a supervisor run ID")
        scene, output = base._prepare_worker_paths(args.scene, args.output)
        layout_path = args.layout.resolve(strict=True)
        layout = Layout.from_file(layout_path)
        result.update({"scene_path": str(scene), "scene_sha256": base._sha256(scene),
                       "layout_path": str(layout_path), "layout_sha256": base._sha256(layout_path)})

        def checkpoint(name: str) -> None:
            nonlocal phase
            phase = name
            base._checkpoint(output, result, started, phase)

        os.environ.setdefault("ROS_DOMAIN_ID", "71")
        os.environ.setdefault("ROS_DISTRO", "jazzy")
        checkpoint("simulation_app")
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RealTimePathTracing", "sync_loads": True, "fast_shutdown": True})
        import numpy as np
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import Gf, Semantics, Usd, UsdGeom, UsdPhysics
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
        result.update({"observed": observed, "observed_stage_scope": "factory_before_robot_insertion"})
        if mismatches:
            raise base.SmokeError(f"factory invariants failed: {mismatches}")
        checkpoint("load_carter")
        asset = get_assets_root_path() + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        stage_utils.add_reference_to_stage(usd_path=asset, path=ROBOT_PATH)
        base._wait_for_stage(app, stage_utils)
        spawn_x, spawn_y = layout.stations["S1"]
        UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PATH)).AddTranslateOp(opSuffix="qualificationSpawn").Set(Gf.Vec3d(spawn_x, spawn_y, 0.1))
        robot = WheeledRobot(ROBOT_PATH, wheel_dof_names=WHEEL_JOINTS)
        checkpoint("capture_load_preview")
        diagnostic = output / "diagnostic-load"
        diagnostic.mkdir()
        result["load_preview"] = base._capture_overview(
            diagnostic, app, rep, camera_position=(spawn_x + 3.0, spawn_y - 3.2, 2.8),
            camera_look_at=(spawn_x, spawn_y, 0.5),
        )
        checkpoint("inspect_robot_collision_geometry")
        bounds, colliders = _collision_bounds(stage, Usd, UsdGeom, UsdPhysics)
        footprint = qualify_footprint(layout, bounds)
        result["robot"] = {"prim_path": ROBOT_PATH, "asset_url": asset, "wheel_dof_names": WHEEL_JOINTS,
                           "enabled_collision_count": len(colliders), "colliders": colliders,
                           "collision_scope": "geometry descendants of enabled CollisionAPI prims; not cooked PhysX contact shapes",
                           "collision_bounds_xyz": bounds, "footprint": footprint}
        if not footprint["passed"]:
            raise base.SmokeError("actual collision envelope does not fit one or more authored anchors")
        for prim, label in [(stage.GetPrimAtPath("/World/Floor"), "floor"),
                            *[(p, "equipment") for p in stage.GetPrimAtPath("/World/Obstacles").GetChildren()],
                            (stage.GetPrimAtPath(ROBOT_PATH), "amr")]:
            semantic = Semantics.SemanticsAPI.Apply(prim, "Semantics")
            semantic.CreateSemanticTypeAttr("class")
            semantic.CreateSemanticDataAttr(label)
        checkpoint("ros_clock_setup")
        rclpy, node, stamps = _clock_graph(stage, app)
        SimulationManager.setup_simulation(dt=1.0/60.0)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        checkpoint("stationary_warmup")
        for _ in range(WARMUP_STEPS):
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
        result["robot"]["dof_names"] = robot.dof_names
        checkpoint("capture_overview")
        timeline.pause()
        result["rgb"] = _capture_factory_overview(output, app, rep)
        timeline.play()
        checkpoint("physics_motion_and_sensors")
        result.update(_capture_motion(output, app, stage, robot, rep, np, UsdGeom, Gf, SimulationManager, rclpy, node))
        result["ros2"] = {"distro": "jazzy", "domain_id": os.environ["ROS_DOMAIN_ID"], "clock_topic": "/clock",
                          "clock_messages": len(stamps), "clock_advanced": len(stamps) >= 2 and stamps[-1] > stamps[0] and all(b >= a for a,b in zip(stamps, stamps[1:])),
                          "clock_samples": stamps, "subscriber_qos": "BEST_EFFORT depth 10",
                          "sensor_topics": "not yet published; G1 integration"}
        stage.Export(str(output / "robot-scene.usda"))
        result["artifact_manifest"] = [{"path": str(path.relative_to(output)).replace("\\", "/"), "bytes": path.stat().st_size,
                                        "sha256": base._sha256(path)} for path in sorted(output.rglob("*"))
                                       if path.is_file() and path.name != base.RESULT_FILENAME]
        errors = validate_robot_evidence(output, result, layout_path, result["layout_sha256"])
        if errors:
            raise base.SmokeError("; ".join(errors))
        valid = True
    except BaseException as error:
        valid = False
        result.update({"status": "failed", "stage": phase, "failure_stage": phase,
                       "error": {"type": type(error).__name__, "message": str(error), "traceback": traceback.format_exc(limit=20)}})
        if output is not None:
            base._write_result(output, result)
        print(f"FACTORY_OPS_G0B_ERROR={phase}: {type(error).__name__}: {error}", flush=True)
    finally:
        cleanup_errors = []
        for action in ([node.destroy_node] if node is not None else []) + ([rclpy.shutdown] if rclpy is not None else []):
            try:
                action()
            except BaseException as error:
                cleanup_errors.append(f"{type(error).__name__}: {error}")
        if cleanup_errors:
            valid = False
            result["cleanup_errors"] = cleanup_errors
            result["status"] = "failed"
            result.setdefault("failure_stage", "ros_cleanup")
        code = 0 if valid else 1
        try:
            if output is not None:
                if valid:
                    result.update({"status": "validated", "cleanup": "worker_exit_requested"})
                    base._checkpoint(output, result, started, "worker_exit_requested")
                else:
                    base._write_result(output, result)
        except BaseException as error:
            code = 1
            print(f"FACTORY_OPS_G0B_FINAL_CHECKPOINT_ERROR={type(error).__name__}: {error}", flush=True)
        finally:
            if app is not None:
                app.close(wait_for_replicator=False, exit_code=code)
                code = 1
    return code


def main() -> int:
    args = _args()
    if args.worker:
        return _worker(args)
    try:
        layout_path = args.layout.resolve(strict=True)
        layout_sha = base._sha256(layout_path)
        code = base._supervisor_main(
            args,
            command_factory=lambda scene, output, run_id: [sys.executable, str(Path(__file__).resolve()),
                "--worker", "--scene", str(scene), "--layout", str(layout_path), "--output", str(output), "--run-id", run_id],
            additional_evidence_validator=lambda _s, output, result, _r, _sha: validate_robot_evidence(output, result, layout_path, layout_sha),
        )
    except BaseException as error:
        print(f"FACTORY_OPS_G0B_SUPERVISOR_ERROR={type(error).__name__}: {error}", flush=True)
        code = 1
    print(f"FACTORY_OPS_G0B_STATUS={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
