#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Validate a completed G1 worker pair and its ROS bag evidence."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import sqlite3
import sys
from typing import Any, Mapping, Sequence
from uuid import UUID

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.footprint import _bad_touched_cells, qualify_footprint
from factory_ops.layout import Layout
from factory_ops.motion import DriveGeometry, WheelOdometry
from factory_ops.operations import Ledger
from scripts.isaac.stage_smoke import _png_dimensions


REQUIRED_TOPICS = {
    "/clock", "/scan", "/odom", "/tf", "/tf_static", "/map", "/navigation_map",
    "/amcl_pose", "/cmd_vel", "/factory/mission_events",
    "/navigate_to_pose/_action/status",
}
EXPECTED_STATIONS = ("S1", "CI", "CO", "OUT")
EXPECTED_TRANSITIONS = ("pickup", "drop", "pickup", "drop")
WHEEL_JOINTS = ("joint_wheel_left", "joint_wheel_right")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path, errors: list[str]) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        errors.append(f"cannot read {path.name}: {exc}")
        return None


def _load_jsonl(path: Path, errors: list[str]) -> list[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as exc:
        errors.append(f"cannot read {path.name}: {exc}")
        return []
    values: list[dict[str, Any]] = []
    for index, line in enumerate(lines, 1):
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"{path.name} line {index} is invalid JSON: {exc}")
            continue
        if not isinstance(value, dict):
            errors.append(f"{path.name} line {index} is not an object")
            continue
        values.append(value)
    return values


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _safe_relative(value: object) -> str | None:
    if not isinstance(value, str) or not value or "\\" in value:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or "." in path.parts:
        return None
    return path.as_posix()


def _manifest(output: Path, excluded: set[str], errors: list[str]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for path in sorted(output.rglob("*")):
        relative = path.relative_to(output).as_posix()
        if relative in excluded:
            continue
        if path.is_symlink():
            errors.append(f"artifact is a symlink: {relative}")
        elif path.is_file():
            entries.append({"path": relative, "bytes": path.stat().st_size, "sha256": _sha256(path)})
    return entries


def _validate_scene_manifest(output: Path, scene: Mapping[str, Any], errors: list[str]) -> None:
    manifest = scene.get("artifact_manifest")
    if not isinstance(manifest, list):
        errors.append("scene artifact_manifest is missing")
        return
    expected: set[str] = set()
    for fixed in ("sim-ready.json", "trajectory.jsonl", "factory-overview.png"):
        if (output / fixed).is_file():
            expected.add(fixed)
    for root_name in ("diagnostic-load", "media"):
        root = output / root_name
        if root.is_dir():
            expected.update(path.relative_to(output).as_posix() for path in root.rglob("*") if path.is_file())
    declared: set[str] = set()
    for index, entry in enumerate(manifest):
        if not isinstance(entry, dict):
            errors.append(f"scene artifact_manifest[{index}] is not an object")
            continue
        relative = _safe_relative(entry.get("path"))
        if relative is None or relative in declared:
            errors.append(f"scene manifest path is unsafe or duplicated at index {index}")
            continue
        declared.add(relative)
        path = output / PurePosixPath(relative)
        if not path.is_file() or path.is_symlink():
            errors.append(f"scene manifest artifact is missing or unsafe: {relative}")
            continue
        if entry.get("bytes") != path.stat().st_size or entry.get("sha256") != _sha256(path):
            errors.append(f"scene manifest metadata mismatch: {relative}")
    if declared != expected:
        errors.append(f"scene manifest ownership mismatch: missing={sorted(expected-declared)}, extra={sorted(declared-expected)}")


def _validate_trajectory(
    output: Path, scene: Mapping[str, Any], layout: Layout, errors: list[str]
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    samples = _load_jsonl(output / "trajectory.jsonl", errors)
    evaluation: dict[str, Any] = {}
    if len(samples) < 2:
        errors.append("trajectory must contain at least two samples")
        return samples, evaluation
    times: list[float] = []
    frames: list[int] = []
    wheel_errors: list[float] = []
    encoder_samples: list[tuple[float, float, float, dict[str, float]]] = []
    first_gt: tuple[float, float] | None = None
    ready = _load_json(output / "sim-ready.json", errors)
    ready_geometry = ready.get("drive_geometry") if isinstance(ready, dict) else None
    geometry: DriveGeometry | None = None
    if isinstance(ready_geometry, dict) and isinstance(ready_geometry.get("provenance"), dict):
        radius_m = _number(ready_geometry.get("wheel_radius_m"))
        separation_m = _number(ready_geometry.get("wheel_separation_m"))
        if radius_m is not None and separation_m is not None:
            try:
                geometry = DriveGeometry(radius_m, separation_m)
            except ValueError:
                pass
    if geometry is None:
        errors.append("sim-ready drive geometry/provenance is invalid")
    robot = scene.get("robot")
    footprint = robot.get("footprint", {}) if isinstance(robot, dict) else {}
    radius = _number(footprint.get("inflated_radius_m")) if isinstance(footprint, dict) else None
    bounds = robot.get("collision_bounds_xyz") if isinstance(robot, dict) else None
    try:
        recalculated = qualify_footprint(layout, bounds, 0.2)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        recalculated = None
        errors.append(f"scene collision footprint cannot be recalculated: {exc}")
    if (radius is None or footprint.get("passed") is not True or footprint.get("clearance_m") != 0.2
            or recalculated != footprint):
        errors.append("scene footprint is not a strict 0.2 m qualified envelope")
    for index, sample in enumerate(samples):
        frame = sample.get("physics_frame")
        stamp = _number(sample.get("sim_time_s"))
        gt = sample.get("ground_truth")
        odom = sample.get("odometry")
        if type(frame) is not int or stamp is None or not isinstance(gt, dict) or not isinstance(odom, dict):
            errors.append(f"trajectory sample {index} has invalid frame/time/pose structure")
            continue
        position = gt.get("position_xyz_m")
        if not isinstance(position, list) or len(position) != 3 or any(_number(v) is None for v in position):
            errors.append(f"trajectory sample {index} has invalid ground truth position")
            continue
        x, y = float(position[0]), float(position[1])
        if radius is not None and _bad_touched_cells(layout, (x, y), radius):
            errors.append(f"trajectory sample {index} violates the qualified raw-map footprint")
        if gt.get("use") != "evaluation and media only":
            errors.append(f"trajectory sample {index} does not constrain ground truth to evaluation")
        if first_gt is None:
            first_gt = (x, y)
        ox, oy = _number(odom.get("x_m")), _number(odom.get("y_m"))
        odometry_values = {
            name: _number(odom.get(name))
            for name in ("x_m", "y_m", "yaw_rad", "linear_m_s", "angular_rad_s")
        }
        if any(value is None for value in odometry_values.values()):
            errors.append(f"trajectory sample {index} has nonfinite odometry")
        joint_names = sample.get("joint_names")
        joint_positions = sample.get("joint_positions_rad")
        wheel_angles = sample.get("wheel_angles_rad")
        typed_joint_names: list[str] | None = None
        if isinstance(joint_names, list) and all(isinstance(name, str) and bool(name) for name in joint_names):
            typed_joint_names = [str(name) for name in joint_names]
        unique_joint_names = (
            typed_joint_names is not None
            and len(set(typed_joint_names)) == len(typed_joint_names)
        )
        if (not unique_joint_names or typed_joint_names is None or not isinstance(joint_positions, list)
                or len(typed_joint_names) != len(joint_positions)
                or not isinstance(wheel_angles, list) or len(wheel_angles) != 2
                or any(_number(value) is None for value in [*joint_positions, *wheel_angles])):
            errors.append(f"trajectory sample {index} has invalid joint/encoder fields")
        else:
            try:
                observed_angles = tuple(float(joint_positions[typed_joint_names.index(name)]) for name in WHEEL_JOINTS)
            except ValueError:
                errors.append(f"trajectory sample {index} is missing named wheel joints")
            else:
                if any(abs(observed - float(recorded)) > 1e-6 for observed, recorded in zip(observed_angles, wheel_angles)):
                    errors.append(f"trajectory sample {index} wheel angles do not match named joint positions")
                if all(value is not None for value in odometry_values.values()):
                    encoder_samples.append((stamp, float(wheel_angles[0]), float(wheel_angles[1]),
                                            {name: float(value) for name, value in odometry_values.items() if value is not None}))
        if ox is not None and oy is not None and first_gt is not None:
            wheel_errors.append(math.hypot(first_gt[0] + ox - x, first_gt[1] + oy - y))
        times.append(stamp)
        frames.append(frame)
    if len(times) != len(samples):
        return samples, evaluation
    for index in range(1, len(times)):
        if times[index] <= times[index - 1] or frames[index] <= frames[index - 1]:
            errors.append("trajectory frame/time is not strictly monotonic")
            break
    timing = scene.get("timing")
    trajectory_meta = scene.get("trajectory")
    if isinstance(timing, dict):
        dt = _number(timing.get("physics_dt_s"))
    else:
        dt = None
    frame_deltas = [frames[i] - frames[i - 1] for i in range(1, len(frames))]
    time_deltas = [times[i] - times[i - 1] for i in range(1, len(times))]
    if not frame_deltas or any(frame_delta != 1 for frame_delta in frame_deltas):
        errors.append("trajectory must contain every adjacent physics frame without decimation")
    if dt is None or any(abs(actual - dt) > 1e-4 for actual in time_deltas):
        errors.append("trajectory delta time does not match one physics_dt interval")
    if not isinstance(trajectory_meta, dict) or trajectory_meta.get("sample_count") != len(samples):
        errors.append("scene trajectory sample_count does not match JSONL")
    if geometry is not None and len(encoder_samples) == len(samples):
        integrator = WheelOdometry(geometry)
        first_stamp, first_left, first_right, first_odom = encoder_samples[0]
        integrator.update(first_stamp, first_left, first_right)
        origin_x, origin_y, origin_yaw = first_odom["x_m"], first_odom["y_m"], first_odom["yaw_rad"]
        for index, (stamp, left, right, observed) in enumerate(encoder_samples[1:], 1):
            relative = integrator.update(stamp, left, right)
            expected = {
                "x_m": origin_x + math.cos(origin_yaw) * relative.x_m - math.sin(origin_yaw) * relative.y_m,
                "y_m": origin_y + math.sin(origin_yaw) * relative.x_m + math.cos(origin_yaw) * relative.y_m,
                "yaw_rad": origin_yaw + relative.yaw_rad,
                "linear_m_s": relative.linear_m_s,
                "angular_rad_s": relative.angular_rad_s,
            }
            if any(abs(observed[name] - expected[name]) > 1e-5 for name in expected):
                errors.append(f"trajectory sample {index} odometry does not recompute from wheel encoders")
                break
    if wheel_errors:
        evaluation["wheel_odometry_vs_ground_truth"] = {
            "sample_count": len(wheel_errors), "maximum_xy_error_m": max(wheel_errors),
            "rms_xy_error_m": math.sqrt(sum(v * v for v in wheel_errors) / len(wheel_errors)),
        }
    return samples, evaluation


def _validate_media(
    output: Path, scene: Mapping[str, Any], trajectory: Sequence[Mapping[str, Any]], errors: list[str]
) -> None:
    document = _load_json(output / "media/frames.json", errors)
    if not isinstance(document, dict) or not isinstance(document.get("frames"), list):
        errors.append("media/frames.json has no frame list")
        return
    frames = document["frames"]
    if len(frames) < 5:
        errors.append("media must contain at least 5 actual RGB frame pairs")
    trajectory_pairs: list[tuple[float, int]] = []
    for item in trajectory:
        trajectory_time = _number(item.get("sim_time_s"))
        trajectory_frame = item.get("physics_frame")
        if trajectory_time is not None and type(trajectory_frame) is int:
            trajectory_pairs.append((trajectory_time, trajectory_frame))
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict) or frame.get("index") != index:
            errors.append(f"media frame {index} has invalid index")
            continue
        stamp = _number(frame.get("sim_time_s"))
        physics_frame = frame.get("physics_frame")
        if stamp is None or type(physics_frame) is not int or not trajectory_pairs:
            errors.append(f"media frame {index} has invalid join fields")
        else:
            validated_stamp = stamp
            nearest = min(trajectory_pairs, key=lambda item: abs(item[0] - validated_stamp), default=None)
            if (nearest is None or abs(nearest[0] - validated_stamp) > 0.06
                    or abs(nearest[1] - physics_frame) > 4):
                errors.append(f"media frame {index} is not within 0.06 s of trajectory")
        for prefix in ("", "overview_"):
            relative = _safe_relative(frame.get(prefix + "path"))
            path = output / PurePosixPath(relative) if relative else None
            if path is None or not path.is_file() or path.is_symlink():
                errors.append(f"media frame {index} {prefix}path is missing or unsafe")
                continue
            try:
                dimensions = _png_dimensions(path)
            except Exception as exc:
                errors.append(f"media PNG is invalid: {relative}: {exc}")
                continue
            if dimensions != (1280, 720) or frame.get(prefix + "width") != 1280 or frame.get(prefix + "height") != 720:
                errors.append(f"media PNG dimensions are not 1280x720: {relative}")
            if frame.get(prefix + "bytes") != path.stat().st_size or frame.get(prefix + "sha256") != _sha256(path):
                errors.append(f"media PNG metadata mismatch: {relative}")
        if frame.get("source") != "actual persistent Isaac Sim RGB render product; no interpolation":
            errors.append(f"media frame {index} lacks actual-render provenance")
    if frames and trajectory:
        final_stamp = _number(frames[-1].get("sim_time_s"))
        trajectory_final = _number(trajectory[-1].get("sim_time_s"))
        if final_stamp is None or trajectory_final is None or abs(final_stamp - trajectory_final) > 0.06:
            errors.append("final media frame is not joined to final trajectory sample")
    meta = scene.get("media")
    if not isinstance(meta, dict) or meta.get("frame_count") != len(frames) or meta.get("initial_and_final") is not True:
        errors.append("scene media metadata does not match frames.json")


def _validate_scene_images(output: Path, scene: Mapping[str, Any], errors: list[str]) -> None:
    for key, relative in (
        ("load_preview", "diagnostic-load/factory-overview.png"),
        ("factory_overview", "factory-overview.png"),
    ):
        metadata = scene.get(key)
        path = output / PurePosixPath(relative)
        if not isinstance(metadata, dict) or not path.is_file() or path.is_symlink():
            errors.append(f"scene {key} image evidence is missing")
            continue
        try:
            dimensions = _png_dimensions(path)
        except Exception as exc:
            errors.append(f"scene {key} PNG is invalid: {exc}")
            continue
        if dimensions != (1280, 720) or metadata.get("width") != 1280 or metadata.get("height") != 720:
            errors.append(f"scene {key} PNG dimensions are not 1280x720")
        if metadata.get("bytes") != path.stat().st_size or metadata.get("sha256") != _sha256(path):
            errors.append(f"scene {key} PNG metadata mismatch")
        if metadata.get("source") != "persistent Replicator RGB annotator; no interpolation":
            errors.append(f"scene {key} lacks actual-render provenance")


def _validate_actions(
    output: Path, mission_result: Mapping[str, Any], bag: Mapping[str, Any], errors: list[str]
) -> None:
    actions = _load_jsonl(output / "actions.jsonl", errors)
    groups = {kind: [item for item in actions if item.get("kind") == kind] for kind in
              ("goal_sent", "goal_accepted", "action_result", "handoff_commit")}
    if any(len(values) != 4 for values in groups.values()):
        errors.append("actions.jsonl must contain exactly 4 sent, accepted, result, and handoff records")
        return
    mission = mission_result.get("mission")
    events = mission.get("events") if isinstance(mission, dict) else None
    navigation_journal = mission.get("navigation_journal") if isinstance(mission, dict) else None
    if not isinstance(events, list):
        errors.append("mission ledger events are missing")
        return
    if not isinstance(navigation_journal, list):
        errors.append("mission navigation journal is missing")
        return
    issued = [item for item in navigation_journal if isinstance(item, dict) and item.get("kind") == "intent_issued"]
    accepted_journal = [item for item in navigation_journal if isinstance(item, dict) and item.get("kind") == "result_accepted"]
    failed_journal = [item for item in navigation_journal if isinstance(item, dict) and item.get("kind") in {"mission_failed", "result_rejected"}]
    if len(issued) != 4 or len(accepted_journal) != 4 or failed_journal:
        errors.append("mission navigation journal is not exactly four clean intent/result handoffs")
        return
    transport = [item for item in events if isinstance(item, dict) and item.get("transition") in {"pickup", "drop"}]
    if len(transport) != 4:
        errors.append("ledger does not contain exactly four pickup/drop commits")
        return
    goal_ids: list[str] = []
    for index, (sent, accepted, action, handoff, ledger_event) in enumerate(zip(
        groups["goal_sent"], groups["goal_accepted"], groups["action_result"], groups["handoff_commit"], transport
    )):
        intent = sent.get("intent")
        result = action.get("result")
        receipt = handoff.get("receipt")
        if not all(isinstance(value, dict) for value in (intent, result, receipt)):
            errors.append(f"action group {index} is structurally invalid")
            continue
        assert isinstance(intent, dict) and isinstance(result, dict) and isinstance(receipt, dict)
        if issued[index].get("intent") != intent:
            errors.append(f"mission intent journal {index} differs from actions.jsonl")
        if (accepted_journal[index].get("result") != result
                or accepted_journal[index].get("receipt") != receipt):
            errors.append(f"mission accepted-result journal {index} differs from the action handoff")
        goal_id = intent.get("goal_id")
        try:
            canonical = str(UUID(str(goal_id)))
        except ValueError:
            errors.append(f"action group {index} has invalid goal UUID")
            continue
        goal_ids.append(canonical)
        bindings = ("goal_id", "task_id", "station", "attempt")
        if any(result.get(key) != intent.get(key) for key in bindings):
            errors.append(f"action result {index} does not bind to its intent")
        if accepted.get("goal_id") != goal_id or accepted.get("server_goal_uuid") != UUID(canonical).hex:
            errors.append(f"action acceptance {index} does not bind to its UUID")
        if result.get("status") != 4 or action.get("nav2_error_code") != 0:
            errors.append(f"action result {index} is not Nav2 SUCCEEDED/error_code 0")
        goal = intent.get("goal")
        pose = result.get("pose")
        received_time = _number(result.get("received_sim_time"))
        if not isinstance(goal, dict) or not isinstance(pose, dict):
            errors.append(f"action result {index} has no independent goal/localization pose")
        else:
            goal_values = [_number(goal.get(name)) for name in ("x", "y", "yaw")]
            pose_values = [_number(pose.get(name)) for name in ("x", "y", "yaw")]
            pose_stamp = _number(pose.get("stamp"))
            if (goal.get("frame_id") != "map" or pose.get("frame_id") != "map"
                    or received_time is None or received_time <= 0 or pose_stamp is None
                    or any(value is None for value in [*goal_values, *pose_values])):
                errors.append(f"action result {index} goal/localization pose is invalid")
            else:
                gx, gy, gyaw = (float(value) for value in goal_values if value is not None)
                px, py, pyaw = (float(value) for value in pose_values if value is not None)
                pose_age = received_time - pose_stamp
                yaw_error = abs(math.atan2(math.sin(pyaw - gyaw), math.cos(pyaw - gyaw)))
                if not 0 <= pose_age <= 0.5:
                    errors.append(f"action result {index} localization pose is future or stale")
                if math.hypot(px - gx, py - gy) > 0.25 or yaw_error > 0.25:
                    errors.append(f"action result {index} localization pose is outside goal tolerance")
        if receipt.get("accepted") is not True or receipt.get("goal_id") != goal_id or receipt.get("committed_transition") != EXPECTED_TRANSITIONS[index]:
            errors.append(f"handoff receipt {index} is invalid")
        handoff_snapshot = handoff.get("snapshot")
        snapshot_seq = handoff_snapshot.get("ledger_seq") if isinstance(handoff_snapshot, dict) else None
        receipt_seq = receipt.get("ledger_seq")
        if type(snapshot_seq) is not int or type(receipt_seq) is not int or snapshot_seq < receipt_seq:
            errors.append(f"handoff snapshot {index} predates its receipt ledger sequence")
        if intent.get("station") != EXPECTED_STATIONS[index]:
            errors.append(f"intent {index} targets the wrong station")
        if not (ledger_event.get("task_id") == result.get("task_id") and
                ledger_event.get("station_id") == result.get("station") and
                ledger_event.get("attempt") == result.get("attempt")):
            errors.append(f"ledger handoff {index} does not bind to its result")
        result_time = _number(result.get("received_sim_time"))
        event_time = _number(ledger_event.get("sim_time"))
        if result_time is None or event_time != result_time or ledger_event.get("ledger_seq") != receipt.get("ledger_seq"):
            errors.append(f"ledger handoff {index} time/sequence does not match receipt")
    status_map = bag.get("goal_statuses")
    if not isinstance(status_map, dict) or any(status_map.get(UUID(goal).hex) != 4 for goal in goal_ids):
        errors.append("bag action statuses do not prove four matching SUCCEEDED goal UUIDs")
    bag_events = bag.get("mission_events")
    if bag_events != actions:
        errors.append("bag mission events do not exactly match actions.jsonl")


def _validate_mission(mission_result: Mapping[str, Any], run_id: str, errors: list[str]) -> None:
    mission = mission_result.get("mission")
    if mission_result.get("run_id") != run_id or mission_result.get("status") != "passed":
        errors.append("mission result did not pass with the requested run ID")
    if mission_result.get("ground_truth_input") is not False:
        errors.append("mission did not prove ground_truth_input=false")
    if not isinstance(mission, dict):
        errors.append("mission export is missing")
        return
    expected = {"completed": True, "failed": False, "active_intent": None, "replay_matches": True}
    if any(mission.get(key) != value for key, value in expected.items()):
        errors.append("mission export is not a completed, nonfailed, inactive replay match")
    events, snapshot = mission.get("events"), mission.get("snapshot")
    if not isinstance(events, list) or not isinstance(snapshot, dict):
        errors.append("mission events/snapshot are missing")
        return
    try:
        replay = Ledger.replay(events)
        if replay.snapshot() != snapshot or replay.events != events:
            errors.append("independent ledger replay differs from mission snapshot/events")
    except (ValueError, KeyError, TypeError) as exc:
        errors.append(f"independent ledger replay failed: {exc}")


def _validate_bag_observation(bag: Mapping[str, Any], errors: list[str]) -> None:
    if bag.get("integrity_check") != "ok":
        errors.append("rosbag SQLite integrity_check did not return ok")
    counts = bag.get("topic_counts")
    if not isinstance(counts, dict):
        errors.append("rosbag topic counts are missing")
    else:
        for topic in sorted(REQUIRED_TOPICS):
            if type(counts.get(topic)) is not int or counts[topic] <= 0:
                errors.append(f"rosbag required topic has no messages: {topic}")
    if type(bag.get("scan_positive_finite_points")) is not int or bag.get("scan_positive_finite_points", 0) <= 0:
        errors.append("rosbag scan has no positive finite range samples")


def _validate_runtime_counts(
    scene: Mapping[str, Any], mission_result: Mapping[str, Any], errors: list[str]
) -> None:
    ros = scene.get("ros")
    if not isinstance(ros, dict):
        errors.append("scene ROS counters are missing")
    else:
        for name in ("scan", "odom", "tf", "tf_static", "cmd_vel"):
            if type(ros.get(name)) is not int or ros[name] <= 0:
                errors.append(f"scene ROS counter is empty: {name}")
        clock = ros.get("clock")
        if not isinstance(clock, dict) or type(clock.get("count")) is not int or clock["count"] < 2 or clock.get("advanced") is not True:
            errors.append("scene ROS clock did not advance")
        samples = ros.get("scan_samples")
        if not isinstance(samples, list) or not any(
            isinstance(item, dict) and type(item.get("finite_count")) is int and item["finite_count"] > 0
            for item in samples
        ):
            errors.append("scene scan samples have no finite points")
    sensor = mission_result.get("sensor_messages")
    if not isinstance(sensor, dict) or any(type(sensor.get(name)) is not int or sensor[name] <= 0 for name in ("scan", "odom")):
        errors.append("mission sensor message counters are incomplete")


def validate_g1_evidence(
    output: Path,
    layout_path: Path,
    run_id: str,
    *,
    bag_observation: Mapping[str, Any] | None = None,
    result_name: str = "g1-validation.json",
) -> dict[str, Any]:
    errors: list[str] = []
    if not re.fullmatch(r"[0-9a-f]{32}", run_id):
        errors.append("run_id must be 32 lowercase hexadecimal characters")
    output = output.resolve()
    layout_path = layout_path.resolve()
    scene = _load_json(output / "g1-scene.json", errors)
    mission_result = _load_json(output / "mission-result.json", errors)
    pair = _load_json(output / "pair-exit.json", errors)
    if not isinstance(scene, dict) or not isinstance(mission_result, dict) or not isinstance(pair, dict):
        return {"passed": False, "run_id": run_id, "errors": errors, "artifact_manifest": []}
    if pair.get("run_id") != run_id or pair.get("nav_exit_code") != 0 or pair.get("sim_exit_code") != 0:
        errors.append("worker pair did not exit zero with the requested run ID")
    if scene.get("run_id") != run_id or scene.get("qualification") != "g1_single_order" or scene.get("status") != "validated":
        errors.append("scene worker result is not validated for this G1 run")
    if scene.get("cleanup") != "worker_exit_requested":
        errors.append("scene worker did not finish cleanup before validation")
    if scene.get("mission_control") != {"run_id": run_id, "status": "passed"}:
        errors.append("scene mission_control does not record the paired pass")
    if pair.get("mission-result.json") != mission_result or pair.get("g1-scene.json") != scene:
        errors.append("pair-exit embedded worker results do not match the final result files")
    _validate_mission(mission_result, run_id, errors)
    _validate_runtime_counts(scene, mission_result, errors)
    layout = Layout.from_file(layout_path)
    _validate_scene_manifest(output, scene, errors)
    _validate_scene_images(output, scene, errors)
    trajectory, evaluation = _validate_trajectory(output, scene, layout, errors)
    _validate_media(output, scene, trajectory, errors)
    if bag_observation is not None:
        bag = dict(bag_observation)
    else:
        try:
            bag = inspect_rosbag(output / "rosbag")
        except (OSError, ValueError, sqlite3.Error, ImportError, json.JSONDecodeError) as exc:
            errors.append(f"rosbag inspection failed: {type(exc).__name__}: {exc}")
            bag = {"inspection_error": f"{type(exc).__name__}: {exc}"}
    _validate_bag_observation(bag, errors)
    _validate_actions(output, mission_result, bag, errors)
    if trajectory:
        final = trajectory[-1].get("ground_truth", {}).get("position_xyz_m")
        if isinstance(final, list) and len(final) >= 2:
            distance = math.hypot(float(final[0]) - layout.stations["OUT"][0], float(final[1]) - layout.stations["OUT"][1])
            evaluation["final_ground_truth_out_distance_m"] = distance
            if distance > 0.35:
                errors.append("final simulator truth is more than 0.35 m from OUT")
    localization = _load_jsonl(output / "localization.jsonl", errors)
    local_errors: list[float] = []
    for pose in localization:
        stamp = _number(pose.get("stamp"))
        x, y = _number(pose.get("x")), _number(pose.get("y"))
        candidates = [
            (candidate_time, item)
            for item in trajectory
            if (candidate_time := _number(item.get("sim_time_s"))) is not None
        ]
        if stamp is None or x is None or y is None or not candidates:
            continue
        validated_stamp = stamp
        nearest_time, nearest = min(candidates, key=lambda item: abs(item[0] - validated_stamp))
        if abs(nearest_time - validated_stamp) <= 0.06:
            gt = nearest["ground_truth"]["position_xyz_m"]
            local_errors.append(math.hypot(x - float(gt[0]), y - float(gt[1])))
    if not local_errors:
        errors.append("localization samples do not join trajectory within 0.06 s")
    else:
        evaluation["localization_vs_ground_truth"] = {
            "sample_count": len(local_errors), "maximum_xy_error_m": max(local_errors),
            "rms_xy_error_m": math.sqrt(sum(v * v for v in local_errors) / len(local_errors)),
        }
    project = Path(__file__).resolve().parents[2]
    source_paths = (project / "scripts/isaac/nav_scene.py", project / "scripts/ros/g1_mission.py")
    source_pins = {path.relative_to(project).as_posix(): _sha256(path) for path in source_paths}
    scene_source = scene.get("inputs", {}).get("source", {}) if isinstance(scene.get("inputs"), dict) else {}
    if not isinstance(scene_source, dict) or scene_source.get("sha256") != source_pins["scripts/isaac/nav_scene.py"]:
        errors.append("scene source pin does not match the validating nav_scene.py")
    result_safe = _safe_relative(result_name)
    if result_safe is None:
        errors.append("validation result name is unsafe")
        result_safe = "g1-validation.json"
    manifest = _manifest(output, {result_safe}, errors)
    return {
        "schema_version": 1,
        "qualification": "g1_single_order",
        "run_id": run_id,
        "passed": not errors,
        "errors": errors,
        "source_pins": source_pins,
        "bag": bag,
        "evaluation": evaluation,
        "artifact_manifest_scope": "all stable caller-host files after both workers exited; this report excluded",
        "artifact_manifest": manifest,
        "claims": {
            "navigation_inputs": "ROS localization/action evidence; simulator truth excluded",
            "ground_truth_use": "footprint, final arrival, and localization/odometry error evaluation only",
            "physical_run": "requires decoded rosbag evidence; JSON journals alone are insufficient",
        },
    }


def inspect_rosbag(bag_directory: Path) -> dict[str, Any]:
    """Inspect SQLite first, then decode selected messages inside a ROS runtime."""
    databases = sorted(bag_directory.glob("*.db3"))
    if not databases:
        raise ValueError(f"no rosbag SQLite database found in {bag_directory}")
    topic_counts: Counter[str] = Counter()
    serialized: list[tuple[str, str, bytes]] = []
    integrity: list[str] = []
    for database in databases:
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            integrity.extend(str(row[0]) for row in connection.execute("PRAGMA integrity_check"))
            topics = {int(row[0]): (str(row[1]), str(row[2])) for row in connection.execute("SELECT id,name,type FROM topics")}
            for topic_id, count in connection.execute("SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id"):
                topic_counts[topics[int(topic_id)][0]] += int(count)
            selected = {topic_id for topic_id, (name, _) in topics.items() if name in {
                "/scan", "/factory/mission_events", "/navigate_to_pose/_action/status"
            }}
            if selected:
                marks = ",".join("?" for _ in selected)
                for topic_id, data in connection.execute(
                    f"SELECT topic_id,data FROM messages WHERE topic_id IN ({marks}) ORDER BY timestamp", tuple(selected)
                ):
                    name, type_name = topics[int(topic_id)]
                    serialized.append((name, type_name, bytes(data)))
        finally:
            connection.close()
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    goal_statuses: dict[str, int] = {}
    mission_events: list[dict[str, Any]] = []
    positive_ranges = 0
    for topic, type_name, data in serialized:
        message = deserialize_message(data, get_message(type_name))
        if topic == "/scan":
            positive_ranges += sum(math.isfinite(float(value)) and float(value) > 0 for value in message.ranges)
        elif topic == "/factory/mission_events":
            value = json.loads(str(message.data))
            if isinstance(value, dict):
                mission_events.append(value)
        else:
            for status in message.status_list:
                goal_statuses[bytes(status.goal_info.goal_id.uuid).hex()] = int(status.status)
    return {
        "integrity_check": "ok" if integrity and all(item == "ok" for item in integrity) else integrity,
        "topic_counts": dict(topic_counts),
        "scan_positive_finite_points": positive_ranges,
        "goal_statuses": goal_statuses,
        "mission_events": mission_events,
        "databases": [{"path": path.name, "bytes": path.stat().st_size, "sha256": _sha256(path)} for path in databases],
    }


def _result_path(output: Path, value: str) -> tuple[str, Path]:
    safe = _safe_relative(value)
    if safe is None:
        raise ValueError("--write-result must be a safe relative path")
    path = (output / PurePosixPath(safe)).resolve(strict=False)
    if not path.is_relative_to(output):
        raise ValueError("--write-result must remain inside --output")
    return safe, path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--layout", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--write-result", default="g1-validation.json")
    args = parser.parse_args()
    if not args.output.is_absolute():
        parser.error("--output must be absolute")
    output = args.output.resolve(strict=True)
    try:
        result_name, report_path = _result_path(output, args.write_result)
    except ValueError as exc:
        parser.error(str(exc))
    report = validate_g1_evidence(output, args.layout, args.run_id, result_name=result_name)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print("FACTORY_OPS_G1_VALIDATION=" + ("PASS" if report["passed"] else "FAIL"), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
