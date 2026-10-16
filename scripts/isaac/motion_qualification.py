#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Measure Nova Carter acceleration, braking, reversal and turning in Isaac Sim 6.2."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import re
import sys
import time
import traceback
from typing import Any, Mapping, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.drive_profile import DriveLimits, DriveProfile
from factory_ops.motion import DriveGeometry, WheelOdometry
from scripts.isaac import stage_smoke as base
from scripts.isaac.nav_scene import (
    _apply_measured_drive_geometry,
    _atomic_json,
    _joint_positions,
    _pose,
    _tilt_evaluation,
    _wheel_angles,
)
from scripts.isaac.robot_qualification import CameraSpec, CaptureSession, ROBOT_PATH, WHEEL_JOINTS, _collision_bounds

RESULT_FILE = "motion-scene.json"
TRAJECTORY_FILE = "trajectory.jsonl"
MEDIA_INDEX_FILE = "media/frames.json"


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    return parser.parse_args()


def _finite_number(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _load_config(path: Path) -> dict[str, Any]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("qualification") != "nova_carter_motion_physics":
        raise base.SmokeError("motion qualification config has the wrong qualification")
    for name in ("physics_dt_s", "warmup_steps", "capture_every_steps", "max_sim_time_s", "max_wall_time_s", "tilt_threshold_rad"):
        if not _finite_number(raw.get(name)) or float(raw[name]) <= 0:
            raise base.SmokeError(f"motion qualification field must be finite and positive: {name}")
    if abs(float(raw["physics_dt_s"]) - 1.0 / 60.0) > 1e-12:
        raise base.SmokeError("motion qualification physics cadence must be exactly 60 Hz")
    if int(raw["capture_every_steps"]) != 6:
        raise base.SmokeError("motion qualification RGB cadence must be 10 Hz at 60 Hz physics")
    geometry = raw.get("drive_geometry")
    limits = raw.get("drive_limits")
    if not isinstance(geometry, dict) or not isinstance(geometry.get("provenance"), dict):
        raise base.SmokeError("motion qualification drive geometry provenance is missing")
    if not isinstance(limits, dict) or limits.get("provenance") != "authored simulation command envelope; not measured motor calibration":
        raise base.SmokeError("motion qualification must identify authored limits as non-calibrated")
    DriveGeometry(float(geometry["wheel_radius_m"]), float(geometry["wheel_separation_m"]))
    _drive_limits(limits)
    schedule = raw.get("schedule")
    if not isinstance(schedule, list) or not schedule:
        raise base.SmokeError("motion qualification schedule is empty")
    cursor = 0.0
    names: list[str] = []
    for entry in schedule:
        if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
            raise base.SmokeError("motion qualification schedule entry is invalid")
        values = [entry.get(name) for name in ("start_s", "end_s", "linear_m_s", "angular_rad_s")]
        if not all(_finite_number(value) for value in values):
            raise base.SmokeError(f"motion qualification schedule contains non-finite values: {entry.get('name')}")
        if abs(float(entry["start_s"]) - cursor) > 1e-9 or float(entry["end_s"]) <= cursor:
            raise base.SmokeError("motion qualification schedule must be contiguous and increasing from zero")
        if not isinstance(entry.get("emergency_stop"), bool):
            raise base.SmokeError("motion qualification schedule emergency_stop must be boolean")
        cursor = float(entry["end_s"])
        names.append(str(entry["name"]))
    required = {
        "straight_accel_cruise", "normal_brake", "turn_approach", "translate_to_inplace_turn", "turn_settle_departure",
        "curved_turn", "opposite_curvature", "normal_stop", "reverse_through_zero", "fault_stop",
    }
    if not required.issubset(names):
        raise base.SmokeError("motion qualification schedule omits a required maneuver")
    if cursor > float(raw["max_sim_time_s"]) or cursor >= 80.0:
        raise base.SmokeError("motion qualification schedule must finish before 80 simulation seconds")
    windows = raw.get("steady_evaluation_windows")
    if not isinstance(windows, list) or not windows:
        raise base.SmokeError("motion qualification steady evaluation windows are missing")
    by_name = {str(entry["name"]): entry for entry in schedule}
    for window in windows:
        if not isinstance(window, dict) or window.get("schedule_segment") not in by_name:
            raise base.SmokeError("steady evaluation window references an unknown schedule segment")
        duration = float(by_name[str(window["schedule_segment"])]["end_s"]) - float(by_name[str(window["schedule_segment"])]["start_s"])
        if not _finite_number(window.get("last_s")) or float(window["last_s"]) < 1.0 or float(window["last_s"]) > duration:
            raise base.SmokeError("steady evaluation window must cover at least one second within its segment")
    return raw


def _drive_limits(raw: Mapping[str, Any]) -> DriveLimits:
    names = (
        "max_linear_m_s", "max_angular_rad_s", "linear_accel_m_s2", "linear_decel_m_s2",
        "angular_accel_rad_s2", "angular_decel_rad_s2", "lateral_accel_m_s2",
        "max_wheel_rad_s", "wheel_accel_rad_s2",
    )
    return DriveLimits(**{name: float(raw[name]) for name in names})


def _prepare_output(output_arg: Path, run_id: str) -> Path:
    if re.fullmatch(r"[0-9a-f]{32}", run_id or "") is None:
        raise base.SmokeError("motion qualification requires a 32-character lowercase hex run ID")
    if not output_arg.is_absolute():
        raise base.SmokeError("motion qualification output path must be absolute")
    output = output_arg.resolve(strict=False)
    if output.exists():
        raise base.SmokeError("motion qualification output path must not already exist")
    output.mkdir(parents=True)
    return output


def _testbench_usda(config: Mapping[str, Any]) -> str:
    size = config["testbench"]["visual_floor_size_xyz_m"]
    half_x, half_y, half_z = (float(value) / 2.0 for value in size)
    top = float(config["testbench"]["visual_floor_top_z_m"])
    grid = config["testbench"]["visual_reference_grid"]
    grid_min, grid_max = (int(value) for value in grid["range_xy_m"])
    line_top = float(grid["line_top_z_m"])
    line_half_height = line_top / 2.0
    lines = [f'''#usda 1.0
(
    defaultPrim = "World"
    upAxis = "Z"
    metersPerUnit = 1
)

def Xform "World"
{{
    custom string qualificationScope = "independent unobstructed motion test pad; not a factory transport run"
    def Cube "FloorVisual"
    {{
        bool physics:collisionEnabled = false
        color3f[] primvars:displayColor = [(0.18, 0.20, 0.22)]
        double size = 2
        double3 xformOp:scale = ({half_x}, {half_y}, {half_z})
        double3 xformOp:translate = (0, 0, {top - half_z})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
    def Plane "Floor" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        uniform token axis = "Z"
        float3[] extent = [(-1, -1, 0), (1, 1, 0)]
        bool physics:collisionEnabled = true
        token visibility = "invisible"
        quatd xformOp:orient = (1, 0, 0, 0)
        double3 xformOp:scale = (1, 1, 1)
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}
    def Scope "VisualReferenceGrid"
    {{
        custom string scope = "render-only measurement reference; not physical factory geometry or a collision obstacle"
''']

    def safe_coordinate(value: int) -> str:
        return f"neg_{abs(value):02d}" if value < 0 else f"pos_{value:02d}"

    def grid_cube(name: str, scale: tuple[float, float, float], position: tuple[float, float, float], color: tuple[float, float, float]) -> list[str]:
        return [
            f'        def Cube "{name}"',
            "        {",
            "            bool physics:collisionEnabled = false",
            f"            color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})]",
            "            double size = 2",
            f"            double3 xformOp:scale = ({scale[0]}, {scale[1]}, {scale[2]})",
            f"            double3 xformOp:translate = ({position[0]}, {position[1]}, {position[2]})",
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
            "        }",
        ]

    for coordinate in range(grid_min, grid_max + 1):
        if coordinate == 0:
            continue
        color = (0.55, 0.58, 0.62) if coordinate % 5 == 0 else (0.30, 0.33, 0.37)
        thickness = 0.025 if coordinate % 5 == 0 else 0.01
        suffix = safe_coordinate(coordinate)
        lines.extend(grid_cube(
            f"GridX_{suffix}", (thickness / 2.0, half_y, line_half_height),
            (float(coordinate), 0.0, line_half_height), color,
        ))
        lines.extend(grid_cube(
            f"GridY_{suffix}", (half_x, thickness / 2.0, line_half_height),
            (0.0, float(coordinate), line_half_height), color,
        ))
    lines.extend(grid_cube(
        "WorldXAxis", (half_x, 0.025, line_half_height),
        (0.0, 0.0, line_half_height), (0.90, 0.12, 0.10),
    ))
    lines.extend(grid_cube(
        "WorldYAxis", (0.025, half_y, line_half_height),
        (0.0, 0.0, line_half_height), (0.10, 0.75, 0.20),
    ))
    lines.extend(["    }", "}", ""])
    return "\n".join(lines)


def _schedule_entry(schedule: Sequence[Mapping[str, Any]], elapsed_s: float) -> Mapping[str, Any]:
    for entry in schedule:
        if float(entry["start_s"]) <= elapsed_s < float(entry["end_s"]):
            return entry
    if abs(elapsed_s - float(schedule[-1]["end_s"])) <= 1e-8:
        return schedule[-1]
    raise base.SmokeError(f"motion qualification schedule has no command at {elapsed_s:.9f}s")


def _joint_velocities(robot: Any) -> dict[str, float]:
    names = list(robot.dof_names)
    values = robot.get_dof_velocities()
    array = values.numpy() if hasattr(values, "numpy") else values
    if getattr(array, "ndim", 1) == 2:
        array = array[0]
    result = {name: float(value) for name, value in zip(names, array)}
    if len(result) != len(names):
        raise base.SmokeError("articulation DOF velocity count does not match DOF names")
    return result


def _append_jsonl(path: Path, value: Any) -> None:
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
        stream.flush()


def _capture_rgb(output: Path, session: CaptureSession, np: Any, Image: Any, index: int, sample: Mapping[str, Any]) -> dict[str, Any]:
    relative = f"media/chase-{index:05d}.png"
    path = output / relative
    image = Image.fromarray(np.asarray(session.medium_rgb.get_data())[:, :, :3].astype(np.uint8))
    try:
        image.save(path)
    finally:
        image.close()
    width, height = base._png_dimensions(path)
    return {
        "index": index,
        "path": relative,
        "sha256": base._sha256(path),
        "bytes": path.stat().st_size,
        "width": width,
        "height": height,
        "physics_frame": sample["physics_frame"],
        "sim_time_s": sample["sim_time_s"],
        "schedule_segment": sample["schedule_segment"],
        "source": "actual Isaac Sim persistent Replicator RGB annotator; no interpolation",
    }


def _run(args: argparse.Namespace) -> int:
    started = time.monotonic()
    app = timeline = rep = robot = session = None
    output: Path | None = None
    phase = "preflight"
    valid = False
    result: dict[str, Any] = {
        "schema_version": 1, "qualification": "nova_carter_motion_physics",
        "run_id": args.run_id, "status": "running", "cleanup": "pending",
    }
    try:
        config_path = args.config.resolve(strict=True)
        config = _load_config(config_path)
        output = _prepare_output(args.output, args.run_id)
        scene_path = output / "motion-testbench.usda"
        scene_path.write_text(_testbench_usda(config), encoding="utf-8", newline="\n")
        result["inputs"] = {
            "config": {"path": str(config_path), "sha256": base._sha256(config_path)},
            "testbench": {"path": str(scene_path), "sha256": base._sha256(scene_path)},
        }

        def checkpoint(name: str) -> None:
            nonlocal phase
            phase = name
            result.update({"stage": name, "elapsed_wall_s": time.monotonic() - started})
            _atomic_json(output / RESULT_FILE, result)

        checkpoint("simulation_app")
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RealTimePathTracing", "sync_loads": True, "fast_shutdown": True})
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep_module
        import omni.timeline
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
        from isaacsim.storage.native import get_assets_root_path
        rep = rep_module

        checkpoint("open_testbench")
        opened, stage = stage_utils.open_stage(str(scene_path))
        if not opened or stage is None:
            raise base.SmokeError("could not open motion qualification testbench")
        app.reset_render_settings()
        stage.SetTimeCodesPerSecond(60.0)
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(10000.0)
        base._wait_for_stage(app, stage_utils)

        checkpoint("load_robot")
        asset = get_assets_root_path() + str(config["asset"]["relative_path"])
        stage_utils.add_reference_to_stage(usd_path=asset, path=ROBOT_PATH)
        base._wait_for_stage(app, stage_utils)
        root = stage.GetPrimAtPath(ROBOT_PATH)
        if not root.IsValid():
            raise base.SmokeError("Nova Carter reference did not produce a valid root prim")
        spawn = [float(value) for value in config["spawn_xyz_m"]]
        UsdGeom.Xformable(root).AddTranslateOp(opSuffix="motionQualificationSpawn").Set(Gf.Vec3d(*spawn))
        robot = WheeledRobot(ROBOT_PATH, wheel_dof_names=WHEEL_JOINTS)

        def save_rgb(data: Any, path: Path) -> None:
            image = Image.fromarray(np.asarray(data)[:, :, :3].astype(np.uint8))
            try:
                image.save(path)
            finally:
                image.close()

        (output / "media").mkdir()
        session = CaptureSession(
            output, app, stage, rep, UsdGeom, Gf, SimulationManager, save_rgb,
            medium_camera=CameraSpec(
                ROBOT_PATH + "/chassis_link/MotionQualificationCamera",
                (-3.0, -4.0, 3.0), (0.0, 0.0, 0.25), "base_link",
            ),
        )
        result["load_preview"] = session.capture_load_preview()
        bounds, colliders = _collision_bounds(stage, Usd, UsdGeom, UsdPhysics)
        measured = _apply_measured_drive_geometry(config, colliders)
        result["robot"] = {
            "asset_url": asset, "asset_relative_path": config["asset"]["relative_path"],
            "prim_path": ROBOT_PATH, "spawn_xyz_m": spawn, "wheel_dof_names": WHEEL_JOINTS,
            "collision_bounds_xyz_m": bounds, "collider_count": len(colliders),
            "drive_geometry": measured,
        }
        result["simulation_run"] = {
            "run_id": args.run_id,
            "kind": "independent Nova Carter physics motion qualification",
            "factory_transport": False,
            "asset_url": asset,
            "asset_root_prim": ROBOT_PATH,
            "command_input": "fixed time schedule only",
            "ground_truth_input": False,
        }
        result["drive_geometry"] = measured
        result["testbench"] = dict(config["testbench"])

        checkpoint("physics_warmup")
        dt = float(config["physics_dt_s"])
        SimulationManager.setup_simulation(dt=dt, device="cpu")
        timeline = omni.timeline.get_timeline_interface()
        rep.orchestrator.set_capture_on_play(True)
        timeline.play()
        zero = np.zeros(2, dtype=np.float32)
        # Match the validated G1 sequence: let the experimental articulation
        # initialize on one physics tick before its first actuator write.
        app.update()
        robot.apply_wheel_actions(zero)
        for _ in range(int(config["warmup_steps"]) - 1):
            app.update()
        geometry = DriveGeometry(float(config["drive_geometry"]["wheel_radius_m"]), float(config["drive_geometry"]["wheel_separation_m"]))
        limits = _drive_limits(config["drive_limits"])
        profile = DriveProfile(geometry, limits)
        odometry = WheelOdometry(geometry)
        initial_time = float(SimulationManager.get_simulation_time())
        left, right = _wheel_angles(robot)
        odometry.update(initial_time, left, right)

        checkpoint("motion_schedule")
        trajectory = output / TRAJECTORY_FILE
        schedule = config["schedule"]
        total_s = float(schedule[-1]["end_s"])
        expected_steps = round(total_s / dt)
        capture_every = int(config["capture_every_steps"])
        frames: list[dict[str, Any]] = []
        timestep_errors: list[float] = []
        tilt_passed = True
        command_envelope_passed = True
        previous_time = initial_time
        for step in range(1, expected_steps + 1):
            elapsed_before = (step - 1) * dt
            segment = _schedule_entry(schedule, elapsed_before)
            requested_linear = float(segment["linear_m_s"])
            requested_angular = float(segment["angular_rad_s"])
            emergency = bool(segment["emergency_stop"])
            command = profile.step(requested_linear, requested_angular, dt, emergency_stop=emergency)
            robot.apply_wheel_actions(np.asarray(command.wheel_targets_rad_s, dtype=np.float32))
            app.update()
            stamp = float(SimulationManager.get_simulation_time())
            observed_dt = stamp - previous_time
            previous_time = stamp
            timestep_errors.append(abs(observed_dt - dt))
            left, right = _wheel_angles(robot)
            odom = odometry.update(stamp, left, right)
            gt_position, gt_orientation = _pose(robot)
            tilt = _tilt_evaluation(gt_orientation, threshold_rad=float(config["tilt_threshold_rad"]))
            tilt_passed = tilt_passed and bool(tilt["passed"])
            command_envelope_passed = command_envelope_passed and (
                abs(command.linear_m_s) <= limits.max_linear_m_s + 1e-9
                and abs(command.angular_rad_s) <= limits.max_angular_rad_s + 1e-9
                and max(abs(value) for value in command.wheel_targets_rad_s) <= limits.max_wheel_rad_s + 1e-9
            )
            sample = {
                "sample_index": step - 1,
                "physics_frame": int(SimulationManager.get_num_physics_steps()),
                "sim_time_s": stamp,
                "schedule_elapsed_s": step * dt,
                "schedule_segment": segment["name"],
                "requested_command": {"linear_m_s": requested_linear, "angular_rad_s": requested_angular},
                "command": {"linear_m_s": command.linear_m_s, "angular_rad_s": command.angular_rad_s},
                "motion_profile": {
                    "phase": command.phase,
                    "curvature_limited": command.curvature_limited,
                    "wheel_speed_limited": command.wheel_speed_limited,
                    "emergency_stop": command.emergency_stop,
                    "input_command": {"linear_m_s": requested_linear, "angular_rad_s": requested_angular},
                    "stop_reason": "injected_fault" if emergency else ("normal_schedule_stop" if requested_linear == requested_angular == 0.0 else "none"),
                },
                "wheel_target_rad_s": list(command.wheel_targets_rad_s),
                "wheel_angles_rad": [left, right],
                "joint_positions_rad_by_name": dict(zip(list(robot.dof_names), _joint_positions(robot))),
                "raw_joint_velocities_rad_s_by_name": _joint_velocities(robot),
                "odometry": {
                    "x_m": odom.x_m, "y_m": odom.y_m, "yaw_rad": odom.yaw_rad,
                    "linear_m_s": odom.linear_m_s, "angular_rad_s": odom.angular_rad_s,
                },
                "ground_truth": {
                    "position_xyz_m": gt_position, "orientation_wxyz": gt_orientation,
                    "tilt_evaluation": tilt,
                    "use": "evaluation and media only; never command, steering, or profile input",
                },
            }
            _append_jsonl(trajectory, sample)
            if step == 1 or step % capture_every == 0 or step == expected_steps:
                timeline.pause()
                frames.append(_capture_rgb(output, session, np, Image, len(frames), sample))
                _atomic_json(output / MEDIA_INDEX_FILE, {
                    "schema_version": 1, "capture_hz": 10, "frames": frames,
                    "source": "actual Isaac Sim renders; raw PNG files retained",
                })
                timeline.play()
            if time.monotonic() - started > float(config["max_wall_time_s"]):
                raise base.OperationTimeout("motion qualification exceeded configured wall time")

        robot.apply_wheel_actions(zero)
        timeline.pause()
        rep.orchestrator.set_capture_on_play(False)
        max_dt_error = max(timestep_errors, default=math.inf)
        actual_samples = sum(1 for _ in trajectory.open(encoding="utf-8"))
        expected_frames = 1 + expected_steps // capture_every
        result["motion"] = {
            "schedule": schedule,
            "steady_evaluation_windows": config["steady_evaluation_windows"],
            "limits": {**asdict(limits), "provenance": config["drive_limits"]["provenance"]},
            "trajectory_file": TRAJECTORY_FILE,
            "sample_count": actual_samples,
            "expected_sample_count": expected_steps,
            "ground_truth_control_boundary": "excluded; fixed time schedule is the only command input",
        }
        result["drive_profile"] = {
            "limits": {**asdict(limits), "provenance": config["drive_limits"]["provenance"]},
            "schedule": schedule,
            "steady_evaluation_windows": config["steady_evaluation_windows"],
        }
        result["physics_dt_s"] = dt
        result["trajectory"] = {
            "path": TRAJECTORY_FILE,
            "sample_count": actual_samples,
            "expected_sample_count": expected_steps,
        }
        result["media"] = {
            "index": MEDIA_INDEX_FILE, "frame_count": len(frames), "expected_frame_count": expected_frames,
            "capture_every_steps": capture_every, "capture_hz": 10,
        }
        result["physics"] = {
            "requested_dt_s": dt, "observed_max_abs_dt_error_s": max_dt_error,
            "timestep_passed": max_dt_error <= 1e-6,
            "device": "cpu", "actual_asset_dynamics": True,
        }
        result["qualification_checks"] = {
            "raw_complete": actual_samples == expected_steps and len(frames) == expected_frames,
            "tilt_passed": tilt_passed,
            "command_envelope_passed": command_envelope_passed,
            "timestep_passed": max_dt_error <= 1e-6,
            "fidelity_metrics_scope": "steady response and stopping-distance dimensions are evaluated by the independent validator",
        }
        if not all(bool(value) for key, value in result["qualification_checks"].items() if key != "fidelity_metrics_scope"):
            raise base.SmokeError(f"motion qualification worker checks failed: {result['qualification_checks']}")
        session.close()
        session = None
        result["artifact_manifest"] = [
            {"path": path.relative_to(output).as_posix(), "bytes": path.stat().st_size, "sha256": base._sha256(path)}
            for path in sorted(output.rglob("*")) if path.is_file() and path.name != RESULT_FILE
        ]
        valid = True
    except BaseException as error:
        result.update({
            "status": "failed", "stage": phase, "failure_stage": phase,
            "error": {"type": type(error).__name__, "message": str(error), "traceback": traceback.format_exc(limit=20)},
        })
        if output is not None:
            _atomic_json(output / RESULT_FILE, result)
        print(f"FACTORY_OPS_MOTION_ERROR={phase}: {type(error).__name__}: {error}", flush=True)
    finally:
        cleanup_errors: list[str] = []
        if robot is not None:
            try:
                import numpy as np
                robot.apply_wheel_actions(np.zeros(2, dtype=np.float32))
            except BaseException as error:
                cleanup_errors.append(f"zero actuators: {type(error).__name__}: {error}")
        if timeline is not None:
            try:
                timeline.pause()
            except BaseException as error:
                cleanup_errors.append(f"pause timeline: {type(error).__name__}: {error}")
        if rep is not None:
            try:
                rep.orchestrator.set_capture_on_play(False)
            except BaseException as error:
                cleanup_errors.append(f"disable capture: {type(error).__name__}: {error}")
        if session is not None:
            try:
                session.close()
            except BaseException as error:
                cleanup_errors.append(f"capture close: {type(error).__name__}: {error}")
        if cleanup_errors:
            valid = False
            result["cleanup_errors"] = cleanup_errors
        code = 0 if valid else 1
        if output is not None:
            result.update({
                "status": "validated" if valid else "failed",
                "cleanup": "complete" if not cleanup_errors else "failed",
                "stage": "worker_exit_requested" if valid else result.get("failure_stage", phase),
            })
            _atomic_json(output / RESULT_FILE, result)
        if app is not None:
            try:
                app.close(wait_for_replicator=False, exit_code=code)
            except BaseException as error:
                code = 1
                result.update({
                    "status": "failed", "cleanup": "failed",
                    "close_error": f"{type(error).__name__}: {error}",
                })
                if output is not None:
                    _atomic_json(output / RESULT_FILE, result)
                print(f"FACTORY_OPS_MOTION_CLOSE_ERROR={type(error).__name__}: {error}", flush=True)
    return code


def main() -> int:
    code = _run(_args())
    print(f"FACTORY_OPS_MOTION_STATUS={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
