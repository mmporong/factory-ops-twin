#!/usr/bin/env python3
"""Validate recorded Isaac motion data without importing the simulator SDK."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.drive_profile import DriveLimits
from factory_ops.motion import DriveGeometry, WheelOdometry
from factory_ops.motion_transients import measure_transients
from factory_ops.motion_validation import validate_drive_trace
from scripts.isaac.stage_smoke import _png_dimensions
from scripts.ros.g1_evidence import _load_json, _load_jsonl, _manifest, _safe_relative, _sha256


def _yaw(sample: dict[str, Any]) -> float:
    w, x, y, z = sample["ground_truth"]["orientation_wxyz"]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def steady_response(samples: list[dict[str, Any]]) -> dict[str, float]:
    if len(samples) < 2:
        raise ValueError("steady response needs consecutive physical samples")
    duration = samples[-1]["sim_time_s"] - samples[0]["sim_time_s"]
    distance = yaw_change = 0.0
    for before, after in zip(samples, samples[1:]):
        old_yaw, new_yaw = _yaw(before), _yaw(after)
        delta_yaw = math.atan2(math.sin(new_yaw - old_yaw), math.cos(new_yaw - old_yaw))
        mid_yaw = old_yaw + delta_yaw / 2
        old_pos, new_pos = (sample["ground_truth"]["position_xyz_m"] for sample in (before, after))
        distance += (new_pos[0] - old_pos[0]) * math.cos(mid_yaw) + (new_pos[1] - old_pos[1]) * math.sin(mid_yaw)
        yaw_change += delta_yaw
    return {
        "encoder_linear_m_s": sum(sample["odometry"]["linear_m_s"] for sample in samples) / len(samples),
        "encoder_angular_rad_s": sum(sample["odometry"]["angular_rad_s"] for sample in samples) / len(samples),
        "chassis_linear_m_s": distance / duration,
        "chassis_angular_rad_s": yaw_change / duration,
    }


def validate_capture_alignment(
    frames: list[dict[str, Any]], samples: list[dict[str, Any]], *, total_count: int, period: int,
) -> list[str]:
    expected_indices = [index for index in range(total_count)
                        if index == 0 or (index + 1) % period == 0 or index == total_count - 1]
    errors = []
    if len(frames) != len(expected_indices):
        errors.append("raw motion RGB count differs from the scheduled cadence")
    for index, frame in enumerate(frames):
        sample = samples[expected_indices[index]] if index < len(expected_indices) and expected_indices[index] < len(samples) else None
        if (frame.get("index") != index or sample is None
                or any(frame.get(key) != sample.get(key) for key in ("physics_frame", "sim_time_s", "schedule_segment"))):
            errors.append(f"RGB {index}: expected capture frame/time/segment binding mismatch")
    return errors


def validate_motion_evidence(output: Path, config_path: Path, run_id: str) -> dict[str, Any]:
    errors: list[str] = []
    scene = _load_json(output / "motion-scene.json", errors)
    worker = _load_json(output / "worker-exit.json", errors)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if (not isinstance(scene, dict) or not isinstance(worker, dict)
            or scene.get("run_id") != run_id or worker.get("run_id") != run_id
            or scene.get("status") != "validated" or scene.get("cleanup") != "complete"
            or worker.get("worker_exit_code") != 0 or worker.get("motion-scene.json") != scene):
        return {"passed": False, "run_id": run_id, "errors": [*errors, "worker completion evidence mismatch"]}
    if scene.get("simulation_run", {}).get("ground_truth_input") is not False:
        errors.append("motion worker does not exclude ground truth from control")
    if scene.get("inputs", {}).get("config", {}).get("sha256") != _sha256(config_path):
        errors.append("motion config hash does not match the validating config")
    limits_raw = {key: config["drive_limits"][key] for key in asdict(DriveLimits())}
    limits = DriveLimits(**limits_raw)
    geometry_raw = scene["drive_geometry"]
    geometry = DriveGeometry(geometry_raw["wheel_radius_m"], geometry_raw["wheel_separation_m"])
    for key in ("wheel_radius_m", "wheel_separation_m"):
        if abs(geometry_raw[key] - config["drive_geometry"][key]) > 1e-4:
            errors.append(f"fresh measured {key} differs from the expected asset")
    expected_limits = dict(limits_raw, provenance=config["drive_limits"]["provenance"])
    if scene.get("drive_profile", {}).get("limits") != expected_limits:
        errors.append("recorded motion limits differ from the config")
    dt = config["physics_dt_s"]
    samples = _load_jsonl(output / "trajectory.jsonl", errors)
    expected_count = round(config["schedule"][-1]["end_s"] / dt)
    if len(samples) != expected_count or scene.get("trajectory", {}).get("sample_count") != len(samples):
        errors.append("motion trace does not contain the entire scheduled duration")
    command_report = validate_drive_trace(samples, geometry, limits, dt)
    errors.extend(command_report["errors"])
    required_phases = {"translate", "curve", "brake_before_turn", "turn_in_place",
                       "settle_before_departure", "normal_stop", "fault_stop"}
    if not required_phases.issubset(command_report["phase_samples"]):
        errors.append("motion trace omits an executed maneuver phase")
    encoder = WheelOdometry(geometry)
    encoder_origin: dict[str, Any] | None = None
    for index, sample in enumerate(samples):
        try:
            elapsed_before = index * dt
            segment = next(item for item in config["schedule"] if item["start_s"] <= elapsed_before < item["end_s"])
            if (sample["sample_index"] != index or abs(sample["schedule_elapsed_s"] - (index + 1) * dt) > 1e-9
                    or sample["schedule_segment"] != segment["name"]
                    or sample["requested_command"] != {"linear_m_s": segment["linear_m_s"], "angular_rad_s": segment["angular_rad_s"]}
                    or sample["motion_profile"]["input_command"] != sample["requested_command"]
                    or sample["motion_profile"]["emergency_stop"] != segment["emergency_stop"]):
                errors.append(f"sample {index}: fixed-schedule command binding mismatch")
            named = sample["joint_positions_rad_by_name"]
            velocities = sample["raw_joint_velocities_rad_s_by_name"]
            wheels = sample["wheel_angles_rad"]
            names = ("joint_wheel_left", "joint_wheel_right")
            if any(not math.isfinite(velocities[name]) or not math.isfinite(named[name])
                   or abs(named[name] - wheels[i]) > 1e-6 for i, name in enumerate(names)):
                errors.append(f"sample {index}: raw joint and encoder binding mismatch")
            integrated = encoder.update(sample["sim_time_s"], *wheels)
            if encoder_origin is None:
                encoder_origin = sample["odometry"]
            else:
                observed = sample["odometry"]
                yaw_origin = encoder_origin["yaw_rad"]
                x_expected = encoder_origin["x_m"] + math.cos(yaw_origin) * integrated.x_m - math.sin(yaw_origin) * integrated.y_m
                y_expected = encoder_origin["y_m"] + math.sin(yaw_origin) * integrated.x_m + math.cos(yaw_origin) * integrated.y_m
                if any(not math.isfinite(observed[key]) or abs(observed[key] - value) > 1e-5 for key, value in (
                    ("x_m", x_expected), ("y_m", y_expected),
                    ("yaw_rad", encoder_origin["yaw_rad"] + integrated.yaw_rad),
                    ("linear_m_s", integrated.linear_m_s), ("angular_rad_s", integrated.angular_rad_s))):
                    errors.append(f"sample {index}: odometry does not recompute from raw wheel angles")
            if any(not math.isfinite(sample["odometry"][key]) for key in ("x_m", "y_m", "yaw_rad", "linear_m_s", "angular_rad_s")):
                errors.append(f"sample {index}: nonfinite wheel odometry")
        except (KeyError, TypeError, ValueError, StopIteration, IndexError) as error:
            errors.append(f"sample {index}: invalid physical evidence ({error})")
    steady: list[dict[str, Any]] = []
    for window in config["steady_evaluation_windows"]:
        segment = next(item for item in config["schedule"] if item["name"] == window["schedule_segment"])
        selected = [sample for sample in samples if sample.get("schedule_segment") == segment["name"]
                    and segment["end_s"] - window["last_s"] < sample.get("schedule_elapsed_s", -1) <= segment["end_s"]]
        if len(selected) < round(window["last_s"] / dt) - 1:
            errors.append(f"steady window {window['name']} lacks full-rate observations")
            continue
        response = steady_response(selected)
        for axis in ("linear_m_s", "angular_rad_s"):
            interval = window.get("expected_encoder_" + axis)
            absolute_interval = window.get("expected_abs_encoder_" + axis)
            maximum = window.get("max_abs_encoder_" + axis)
            for source in ("encoder", "chassis"):
                actual = response[source + "_" + axis]
                if ((interval and not interval[0] <= actual <= interval[1])
                        or (absolute_interval and not absolute_interval[0] <= abs(actual) <= absolute_interval[1])
                        or (maximum is not None and abs(actual) > maximum)):
                    errors.append(f"steady window {window['name']}: {source} {axis} outside declared response range")
        steady.append({"name": window["name"], "sample_count": len(selected), "response": response})
    transients = measure_transients(samples, config)
    errors.extend(transients["errors"])
    media = _load_json(output / "media/frames.json", errors)
    frames = media.get("frames", []) if isinstance(media, dict) else []
    errors.extend(validate_capture_alignment(frames, samples, total_count=expected_count,
                                             period=config["capture_every_steps"]))
    if scene.get("media", {}).get("frame_count") != len(frames):
        errors.append("raw motion RGB count differs from the worker result")
    for index, frame in enumerate(frames):
        relative = _safe_relative(frame.get("path"))
        path = output / relative if relative else None
        if (path is None or path.is_symlink() or not path.is_file() or _png_dimensions(path) != (1280, 720)
                or frame.get("bytes") != path.stat().st_size or frame.get("sha256") != _sha256(path)):
            errors.append(f"RGB {index}: original PNG metadata mismatch")
    declared = {item["path"]: item for item in scene.get("artifact_manifest", [])}
    expected = _manifest(output, {"motion-scene.json", "worker-exit.json", "motion-validation.json"}, errors)
    if declared != {item["path"]: item for item in expected}:
        errors.append("worker original artifact manifest mismatch")
    manifest = _manifest(output, {"motion-validation.json"}, errors)
    return {"schema_version": 1, "run_id": run_id, "passed": not errors, "errors": errors,
            "command_profile": command_report, "steady_physical_response": steady,
            "transient_physical_response": transients,
            "media_frame_count": len(frames), "config_sha256": _sha256(config_path),
            "artifact_manifest": manifest,
            "scope": "independent unobstructed motion test pad; not a factory transport or real motor calibration"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    try:
        report = validate_motion_evidence(args.output, args.config, args.run_id)
    except (OSError, KeyError, TypeError, ValueError, IndexError, ZeroDivisionError) as error:
        report = {"passed": False, "run_id": args.run_id,
                  "errors": [f"invalid motion evidence: {type(error).__name__}: {error}"]}
    (args.output / "motion-validation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("FACTORY_OPS_MOTION_VALIDATION=" + ("PASS" if report["passed"] else "FAIL"))
    print(json.dumps({"errors": report["errors"], "steady_physical_response": report.get("steady_physical_response")}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
