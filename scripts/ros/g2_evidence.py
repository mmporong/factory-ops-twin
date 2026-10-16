#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Independent G2 trajectory, authorization, media and decoded ROS evidence checks."""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sqlite3
import sys
from typing import Any, Mapping, Sequence
from uuid import UUID

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.drive_profile import DriveLimits
from factory_ops.footprint import _bad_touched_cells, qualify_footprint
from factory_ops.g2_fault_evidence import validate_f2_fault
from factory_ops.layout import Layout
from factory_ops.motion import DriveGeometry, WheelOdometry
from factory_ops.motion_authorization import AuthorizationError, LeaseGuard
from factory_ops.motion_validation import validate_drive_trace
from factory_ops.traffic import TrafficController
from factory_ops.traffic_observation import circle_intersects_polygon
from factory_ops.traffic_observation import exit_xy, project_observation
from scripts.isaac.stage_smoke import _png_dimensions
from scripts.ros.g1_evidence import _load_json, _load_jsonl, _manifest, _safe_relative, _sha256

ROBOTS = ("carter1", "carter2")
PROJECT = Path(__file__).resolve().parents[2]


def qualify_authored_routes(
    layout: Layout, traffic: TrafficController, scenario: Mapping[str, Any], planning_radius_m: float,
) -> dict[str, Any]:
    """Check a conservative continuous corridor around declared line segments.

    Samples are <= one map cell apart, with half that interval added to the
    circle.  This covers the whole intervening segment by the triangle
    inequality.  Actual Nav2 paths still require separate execution checks.
    """
    errors: list[str] = []
    routes: dict[str, Any] = {}
    step_m = layout.resolution_m
    radius = planning_radius_m + step_m / 2
    for route_id, route in scenario.get("routes", {}).items():
        points = [route["start_pose"], *route["poses"]]
        checked = 0
        distance = 0.0
        for start, goal in zip(points, points[1:]):
            segment = math.dist(start[:2], goal[:2])
            distance += segment
            count = max(1, math.ceil(segment / step_m))
            for index in range(count + 1):
                fraction = index / count
                xy = (start[0] + fraction * (goal[0] - start[0]), start[1] + fraction * (goal[1] - start[1]))
                checked += 1
                if _bad_touched_cells(layout, xy, radius):
                    errors.append(f"{route_id}: declared route corridor leaves the planning map at {xy}")
                    break
                for resource_id in route.get("forbidden_resources", []):
                    polygon: Any = traffic.resources[resource_id]["polygon_xy"]
                    if circle_intersects_polygon(xy, radius, polygon):
                        errors.append(f"{route_id}: declared route corridor touches forbidden {resource_id}")
                        break
        routes[route_id] = {"distance_m": distance, "checked_points": checked}
    return {"passed": not errors, "errors": errors, "routes": routes,
            "planning_radius_m": planning_radius_m, "sampling_cover_radius_m": radius,
            "scope": "authored straight segments only; not an actual Nav2 path or execution result"}


def _nearest(samples: Sequence[Mapping[str, Any]], times: Sequence[float], stamp: float, tolerance: float = .06) -> Mapping[str, Any] | None:
    index = bisect_left(times, stamp)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
    if not candidates:
        return None
    chosen = min(candidates, key=lambda i: abs(times[i] - stamp))
    return samples[chosen] if abs(times[chosen] - stamp) <= tolerance else None


def _yaw(quaternion: Sequence[float]) -> float:
    w, x, y, z = map(float, quaternion)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def unique_sequence_index(
    records: Sequence[Mapping[str, Any]], field: str, errors: list[str], label: str,
) -> dict[int, Mapping[str, Any]]:
    result: dict[int, Mapping[str, Any]] = {}
    expected = 1
    for row in records:
        sequence = row.get(field)
        if type(sequence) is not int or sequence != expected:
            errors.append(f"{label}: missing, duplicate, reversed or invalid {field}; expected {expected}")
            continue
        expected += 1
        result[sequence] = row
    return result


def validate_scan_pairs(
    raw_scans: Mapping[tuple[str, float], Sequence[Mapping[str, Any]]],
    filtered_scans: Mapping[tuple[str, float], Sequence[Mapping[str, Any]]],
    *, coverage_window: tuple[float, float] | None = None,
) -> dict[str, Any]:
    """Match every filtered scan, independently of DDS recording order."""
    errors: list[str] = []
    paired: Counter[str] = Counter()
    covered: Counter[str] = Counter()
    for (topic, stamp), records in raw_scans.items():
        if len(records) != 1:
            errors.append(f"{topic}@{stamp}: duplicate raw scan timestamp")
        if coverage_window is None or coverage_window[0] <= stamp <= coverage_window[1]:
            covered[topic] += 1
            if len(filtered_scans.get((topic, stamp), [])) != 1:
                errors.append(f"{topic}@{stamp}: qualified raw scan lacks exactly one filtered output")
    for (topic, stamp), records in filtered_scans.items():
        source = raw_scans.get((topic, stamp), [])
        if len(records) != 1 or len(source) != 1:
            errors.append(f"{topic}@{stamp}: filtered scan must have one unique raw timestamp peer")
            continue
        raw, derived = source[0], records[0]
        metadata = ("frame_id", "angle_min", "angle_max", "angle_increment", "time_increment",
                    "scan_time", "range_min", "range_max", "intensities")
        if any(raw.get(key) != derived.get(key) for key in metadata) or len(raw["ranges"]) != len(derived["ranges"]):
            errors.append(f"{topic}@{stamp}: filter changed scan metadata")
            continue
        angular = (raw["angle_min"], raw["angle_max"], raw["angle_increment"])
        count = len(raw["ranges"])
        float32_tolerance = 8 * 2 ** -23 * max(1.0, abs(angular[0]), abs(angular[1]), count * abs(angular[2]))
        end_errors = [abs(angular[0] + intervals * angular[2] - angular[1]) for intervals in (count - 1, count)]
        if (not all(math.isfinite(value) for value in angular) or angular[2] <= 0
                or angular[1] < angular[0] or count == 0 or min(end_errors) > float32_tolerance):
            errors.append(f"{topic}@{stamp}: invalid raw angular metadata")
            continue
        for index, (before, after) in enumerate(zip(raw["ranges"], derived["ranges"])):
            angle = math.atan2(math.sin(raw["angle_min"] + index * raw["angle_increment"]),
                               math.cos(raw["angle_min"] + index * raw["angle_increment"]))
            if (abs(angle) <= math.pi / 2 + 1e-12 and not (before == after or math.isnan(before) and math.isnan(after))
                    or abs(angle) > math.pi / 2 + 1e-12 and after != math.inf):
                errors.append(f"{topic}@{stamp}: filtered ray violates the static half-plane contract")
                break
        else:
            paired[topic] += 1
    return {"passed": not errors, "errors": errors, "paired_counts": dict(paired),
            "covered_raw_counts": dict(covered), "coverage_window_sim_s": coverage_window,
            "raw_timestamp_count": len(raw_scans), "filtered_timestamp_count": len(filtered_scans)}


def decoded_map_pose(edges: Mapping[str, Sequence[Mapping[str, Any]]], stamp: float) -> tuple[float, float]:
    """Reconstruct planar map->base_link from actual recorded TF messages."""
    poses = []
    for edge in ("map->odom", "odom->base_link"):
        unique = {row["stamp_s"]: row for row in edges.get(edge, [])}
        times = sorted(unique)
        at = bisect_left(times, stamp)
        if at < len(times) and abs(times[at] - stamp) < 1e-8:
            a = b = unique[times[at]]
            fraction = 0.0
        elif at == 0 or at == len(times):
            raise ValueError(f"recorded TF does not bracket {edge} at {stamp}")
        else:
            a, b = unique[times[at - 1]], unique[times[at]]
            if b["stamp_s"] - a["stamp_s"] > TrafficController.pose_freshness_s:
                raise ValueError(f"recorded TF has a stale interpolation gap for {edge}")
            fraction = (stamp - a["stamp_s"]) / (b["stamp_s"] - a["stamp_s"])
        yaw_a, yaw_b = _yaw(a["wxyz"]), _yaw(b["wxyz"])
        yaw = yaw_a + fraction * math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))
        poses.append((a["translation"][0] + fraction * (b["translation"][0] - a["translation"][0]),
                      a["translation"][1] + fraction * (b["translation"][1] - a["translation"][1]), yaw))
    map_odom, odom_base = poses
    return (map_odom[0] + math.cos(map_odom[2]) * odom_base[0] - math.sin(map_odom[2]) * odom_base[1],
            map_odom[1] + math.sin(map_odom[2]) * odom_base[0] + math.cos(map_odom[2]) * odom_base[1])


def validate_f3_cycle(
    events: Sequence[Mapping[str, Any]], traffic: TrafficController,
    tf_samples: Mapping[str, Mapping[str, Sequence[Mapping[str, Any]]]],
    *, planning_radius_m: float,
) -> dict[str, Any]:
    errors: list[str] = []
    recoveries = [row for row in events if row["event_type"] == "recovery_decision"]
    if len(recoveries) != 1:
        return {"passed": False, "errors": ["F3 requires exactly one actual recovery decision"]}
    recovery = recoveries[0]
    if (recovery.get("robot_id") != "carter2" or recovery.get("priority_robot") != "carter1"
            or recovery.get("target") != "P2" or recovery.get("command") != "cancel_and_yield"):
        errors.append("F3 recovery owner/priority/pocket/command mismatch")
    end = recovery["sim_time_s"]
    observed = [row for row in events if row.get("reason") == "observed_exit_blocked" and row["sim_time_s"] <= end]
    if not observed:
        return {"passed": False, "errors": [*errors, "F3 has no observed exit-blockage evidence"]}
    last = observed[-1]
    generation = last.get("reservation_id")
    contiguous = []
    for row in reversed(observed):
        if (row.get("robot_id") != "carter1" or row.get("blockers") not in (["carter2"], ("carter2",))
                or row.get("reservation_id") != generation or row.get("exit_id") != "Z_UPPER:east"):
            break
        contiguous.append(row)
        if row["sim_time_s"] <= end - traffic.cycle_persistence_s + 1e-8:
            break
    contiguous.reverse()
    if (not contiguous or contiguous[0]["sim_time_s"] > end - traffic.cycle_persistence_s + 1e-8
            or end - contiguous[-1]["sim_time_s"] > traffic.pose_freshness_s
            or any(b["sim_time_s"] - a["sim_time_s"] > traffic.pose_freshness_s for a, b in zip(contiguous, contiguous[1:]))):
        errors.append("F3 same-generation exit blockage is not continuously evidenced for five seconds")
    for row in contiguous:
        radius = row["planning_radius_m"]
        if abs(radius - planning_radius_m) > 1e-9:
            errors.append("F3 observed exit wait changed the qualified planning radius")
        for prefix, robot in (("waiter", "carter1"), ("blocker", "carter2")):
            stamp = row[prefix + "_pose_time_s"]
            if not 0 <= row["sim_time_s"] - stamp <= traffic.pose_freshness_s + 1e-9:
                errors.append("F3 exit-wait pose evidence is stale or future")
                continue
            try:
                decoded = decoded_map_pose(tf_samples.get(robot, {}), stamp)
                if math.dist(decoded, row[prefix + "_pose_xy"]) > .02:
                    errors.append("F3 estimated exit-wait pose does not match decoded TF")
                if prefix == "waiter" and "Z_UPPER" not in project_observation(traffic, decoded, radius)["occupied_resources"]:
                    errors.append("F3 waiting owner's estimated footprint is outside its resource")
                if prefix == "blocker" and math.dist(decoded, exit_xy(traffic, "Z_UPPER:east")) > 2 * radius + 1e-9:
                    errors.append("F3 peer does not block the reserved exit in decoded TF")
            except (ValueError, KeyError) as error:
                errors.append(f"F3 TF evidence unavailable: {error}")
    start = contiguous[0]["sim_time_s"] if contiguous else end
    for robot in ROBOTS:
        timestamps = {row["stamp_s"] for row in tf_samples.get(robot, {}).get("odom->base_link", [])
                      if start <= row["stamp_s"] <= end}
        for stamp in timestamps:
            try:
                decoded = decoded_map_pose(tf_samples.get(robot, {}), stamp)
                if robot == "carter1":
                    valid = "Z_UPPER" in project_observation(traffic, decoded, planning_radius_m)["occupied_resources"]
                else:
                    valid = math.dist(decoded, exit_xy(traffic, "Z_UPPER:east")) <= 2 * planning_radius_m + 1e-9
                if not valid:
                    errors.append("F3 decoded TF interrupts the claimed continuous exit blockage")
                    break
            except (ValueError, KeyError) as error:
                errors.append(f"F3 continuous TF evidence unavailable: {error}")
                break
    reverse_wait = [row for row in events if row["event_type"] == "request_denied" and row["robot_id"] == "carter2"
                    and "carter1" in row.get("blockers", []) and row["sim_time_s"] <= start]
    if not reverse_wait:
        errors.append("F3 reverse resource-wait edge is absent before the observed cycle")
    grants = [row for row in events if row.get("reservation_id") == generation and row["event_type"] == "reservation_granted"]
    release = [row for row in events if row.get("reservation_id") == generation and row["event_type"] == "release_ack"]
    resumed = [row for row in events if row["event_type"] == "resume_requested"]
    if (len(grants) != 1 or grants[0]["robot_id"] != "carter1" or grants[0].get("resources") not in (["Z_UPPER"], ("Z_UPPER",))
            or grants[0].get("exit_id") != "Z_UPPER:east" or grants[0]["sim_time_s"] > start
            or len(release) != 1 or release[0]["sim_time_s"] <= end
            or len(resumed) != 1 or resumed[0].get("robot_id") != "carter2" or resumed[0].get("reservation_id") != generation
            or resumed[0]["sim_time_s"] < release[0]["sim_time_s"]):
        errors.append("F3 recovery and one-shot resume do not share the original release-ACK generation")
    return {"passed": not errors, "errors": errors, "recovery_id": recovery.get("recovery_id"),
            "reservation_id": generation, "observed_wait_rows": len(contiguous), "observed_interval_s": end - start}


def _validate_robot(
    output: Path, robot_id: str, ready: Mapping[str, Any], scene: Mapping[str, Any],
    layout: Layout, config: Mapping[str, Any], errors: list[str],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    samples = _load_jsonl(output / "robots" / robot_id / "trajectory.jsonl", errors)
    if len(samples) < 2:
        errors.append(f"{robot_id}: insufficient full-rate trajectory")
        return samples, {}
    meta = ready["robots"][robot_id]
    footprint = meta["footprint"]
    measured = qualify_footprint(layout, footprint["collision_bounds_xyz"], .2)
    if measured != footprint or not measured["passed"]:
        errors.append(f"{robot_id}: collision footprint cannot be independently qualified")
    dimensions = meta["drive_geometry"]
    geometry = DriveGeometry(dimensions["wheel_radius_m"], dimensions["wheel_separation_m"])
    limits = DriveLimits(**{key: config["drive_profile"][key] for key in DriveLimits.__dataclass_fields__})
    drive = validate_drive_trace(samples, geometry, limits, config["physics_dt_s"])
    errors.extend(f"{robot_id}: {error}" for error in drive["errors"])
    guard = LeaseGuard(ready["run_id"], robot_id, config["authorization_ttl_s"])
    integrator = WheelOdometry(geometry)
    first_odom = samples[0]["odometry"]
    origin = (first_odom["x_m"], first_odom["y_m"], first_odom["yaw_rad"])
    malformed = raw_map_bad = encoder_bad = 0
    authorization_counts: Counter[str] = Counter()
    for index, sample in enumerate(samples):
        try:
            position = sample["ground_truth"]["position_xyz_m"]
            if sample["ground_truth"].get("use") != "evaluation and media only":
                raise ValueError("ground-truth use boundary missing")
            if _bad_touched_cells(layout, tuple(position[:2]), measured["inflated_radius_m"]):
                raw_map_bad += 1
            names = sample["joint_names"]
            joints = sample["joint_positions_rad"]
            angles = sample["wheel_angles_rad"]
            if (len(names) != len(joints) or len(names) != len(set(names)) or len(angles) != 2
                    or any(abs(joints[names.index(name)] - angle) > 1e-6 for name, angle in
                           zip(("joint_wheel_left", "joint_wheel_right"), angles))):
                raise ValueError("named encoder/joint readings mismatch")
            integrated = integrator.update(sample["sim_time_s"], *angles)
            expected = {"x_m": origin[0] + math.cos(origin[2]) * integrated.x_m - math.sin(origin[2]) * integrated.y_m,
                        "y_m": origin[1] + math.sin(origin[2]) * integrated.x_m + math.cos(origin[2]) * integrated.y_m,
                        "yaw_rad": origin[2] + integrated.yaw_rad,
                        "linear_m_s": integrated.linear_m_s, "angular_rad_s": integrated.angular_rad_s}
            if index and any(abs(sample["odometry"][key] - value) > 1e-5 for key, value in expected.items()):
                encoder_bad += 1
            authorization = sample["authorization"]
            evaluated = guard.evaluate(authorization["payload"], now_s=authorization["evaluation_sim_time_s"])
            if any(authorization[key] != getattr(evaluated, key) for key in ("mode", "reason", "valid", "sequence")):
                raise ValueError("recorded authorization disagrees with independent evaluation")
            authorization_counts[evaluated.reason] += 1
            if abs(sample["sim_time_s"] - authorization["evaluation_sim_time_s"] - config["physics_dt_s"]) > 1e-4:
                raise ValueError("authorization timestamp does not precede the applied physics frame")
            if (not evaluated.valid or evaluated.mode == "emergency") and any(sample["wheel_target_rad_s"]):
                raise ValueError("invalid/emergency authorization has nonzero wheel target")
            if evaluated.mode == "hold" and any(sample["motion_profile"]["input_command"].values()):
                raise ValueError("hold permission did not request normal braking")
        except (KeyError, TypeError, ValueError, AuthorizationError) as error:
            malformed += 1
            if malformed <= 5:
                errors.append(f"{robot_id} sample {index}: {error}")
    if raw_map_bad:
        errors.append(f"{robot_id}: {raw_map_bad} frames violate the qualified raw-map footprint")
    if encoder_bad:
        errors.append(f"{robot_id}: {encoder_bad} frames do not recompute from wheel encoders")
    if malformed > 5:
        errors.append(f"{robot_id}: {malformed} invalid trajectory/authorization samples total")
    reported = scene.get("robots", {}).get(robot_id, {})
    if reported.get("trajectory_samples") != len(samples):
        errors.append(f"{robot_id}: scene sample count does not match raw JSONL")
    return samples, {"sample_count": len(samples), "drive_profile": drive,
                     "raw_map_violation_frames": raw_map_bad, "encoder_mismatch_frames": encoder_bad,
                     "authorization_reasons": dict(authorization_counts), "footprint": measured}


def _validate_media(output: Path, scene: Mapping[str, Any], traces: Mapping[str, list[dict[str, Any]]], errors: list[str]) -> dict[str, Any]:
    document = _load_json(output / "media/frames.json", errors)
    frames = document.get("frames", []) if isinstance(document, dict) else []
    if len(frames) < 5 or scene.get("media", {}).get("frame_count") != len(frames):
        errors.append("three-camera media count is missing or inconsistent")
    by_frame = {robot: {sample["physics_frame"]: sample for sample in samples} for robot, samples in traces.items()}
    for index, frame in enumerate(frames):
        if frame.get("index") != index or set(frame.get("sources", {})) != {"overview", *ROBOTS}:
            errors.append(f"media frame {index}: three synchronized sources are required")
            continue
        for robot in ROBOTS:
            sample = by_frame[robot].get(frame.get("physics_frame"))
            pose = frame.get("ground_truth", {}).get(robot, {})
            if (sample is None or frame.get("sim_time_s") != sample["sim_time_s"]
                    or any(pose.get(key) != sample["ground_truth"].get(key) for key in ("position_xyz_m", "orientation_wxyz"))):
                errors.append(f"media frame {index}: {robot} physics/pose join mismatch")
        for name, meta in frame["sources"].items():
            relative = _safe_relative(meta.get("path"))
            path = output / relative if relative else None
            if path is None or not path.is_file() or path.is_symlink():
                errors.append(f"media frame {index}: invalid {name} path")
                continue
            if (_png_dimensions(path) != (1280, 720) or meta.get("width") != 1280 or meta.get("height") != 720
                    or meta.get("sha256") != _sha256(path) or meta.get("bytes") != path.stat().st_size):
                errors.append(f"media frame {index}: {name} image metadata mismatch")
    if frames and any(traces[robot] and frames[-1]["physics_frame"] != traces[robot][-1]["physics_frame"] for robot in ROBOTS):
        errors.append("media does not include the final physics frame")
    return {"frame_count": len(frames), "image_count": len(frames) * 3}


def inspect_rosbag(output: Path, traces: Mapping[str, list[dict[str, Any]]], ready: Mapping[str, Any]) -> dict[str, Any]:
    """Decode actual CDR messages; JSON summaries cannot substitute for this pass."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    counts: Counter[str] = Counter()
    errors: list[str] = []
    scan_points: Counter[str] = Counter()
    scan_geometry: dict[str, dict[str, Any]] = {}
    peer_hits: Counter[str] = Counter()
    frames: dict[str, set[str]] = defaultdict(set)
    transforms: dict[str, dict[str, Any]] = defaultdict(dict)
    tf_samples: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    statuses: dict[str, dict[str, int]] = defaultdict(dict)
    authorization_rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
    amcl_poses: dict[str, list[dict[str, Any]]] = defaultdict(list)
    events: list[dict[str, Any]] = []
    raw_scans: dict[tuple[str, float], list[Mapping[str, Any]]] = defaultdict(list)
    filtered_scans: dict[tuple[str, float], list[Mapping[str, Any]]] = defaultdict(list)
    filtered_count: Counter[str] = Counter()
    times = {robot: [row["sim_time_s"] for row in trace] for robot, trace in traces.items()}
    databases = sorted((output / "rosbag").glob("*.db3"))
    if not databases:
        raise ValueError("no actual rosbag SQLite database")
    integrity: list[str] = []
    types: dict[str, Any] = {}
    for database in databases:
        with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as connection:
            integrity.extend(str(row[0]) for row in connection.execute("PRAGMA integrity_check"))
            topics = {int(row[0]): (str(row[1]), str(row[2])) for row in connection.execute("SELECT id,name,type FROM topics")}
            for tid, count in connection.execute("SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id"):
                counts[topics[int(tid)][0]] += count
            for tid, data in connection.execute("SELECT topic_id,data FROM messages ORDER BY timestamp"):
                topic, type_name = topics[int(tid)]
                selected = ("/scan_" in topic or topic.endswith(("/tf", "/tf_static", "/_action/status", "/amcl_pose"))
                            or topic in ("/factory/g2/events", "/factory/g2/authorizations"))
                if not selected:
                    continue
                if type_name not in types:
                    types[type_name] = get_message(type_name)
                message = deserialize_message(bytes(data), types[type_name])
                if topic in ("/factory/g2/events", "/factory/g2/authorizations"):
                    value = json.loads(message.data)
                    if (not isinstance(value, dict) or value.get("run_id") != ready["run_id"]
                            or (value.get("scope", {}).get("scenario_id") if topic.endswith("authorizations") else value.get("scenario_id")) != ready["scenario_id"]):
                        errors.append(f"{topic}: decoded event run/scenario identity mismatch")
                        continue
                    if topic.endswith("authorizations"):
                        if value.get("robot_id") not in ROBOTS:
                            errors.append("decoded authorization has an unknown robot identity")
                            continue
                        authorization_rows[value["robot_id"]].append(value)
                    else:
                        events.append(value)
                elif topic.endswith("/_action/status"):
                    for status in message.status_list:
                        statuses[topic][bytes(status.goal_info.goal_id.uuid).hex()] = int(status.status)
                elif topic.endswith("/amcl_pose"):
                    position, orientation = message.pose.pose.position, message.pose.pose.orientation
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                    values = [position.x, position.y, orientation.w, orientation.x, orientation.y, orientation.z]
                    if (message.header.frame_id != "map" or not all(math.isfinite(v) for v in values)
                            or abs(sum(v * v for v in values[2:]) - 1) > 1e-3):
                        errors.append(f"{topic}: invalid decoded AMCL pose")
                    else:
                        amcl_poses[topic.split("/")[1]].append({"stamp_s": stamp, "x_m": position.x,
                                                               "y_m": position.y, "yaw_rad": _yaw(values[2:])})
                elif topic.endswith(("/tf", "/tf_static")):
                    for transform in message.transforms:
                        key = f"{transform.header.frame_id}->{transform.child_frame_id}"
                        translation, rotation = transform.transform.translation, transform.transform.rotation
                        transforms[topic][key] = {"translation": [translation.x, translation.y, translation.z],
                                                  "wxyz": [rotation.w, rotation.x, rotation.y, rotation.z]}
                        if key in ("map->odom", "odom->base_link"):
                            tf_samples[topic.split("/")[1]][key].append({**transforms[topic][key],
                                "stamp_s": transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9})
                else:
                    robot = topic.split("/")[1]
                    if robot not in ROBOTS:
                        errors.append(f"unexpected scan namespace {topic}")
                        continue
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                    frames[topic].add(message.header.frame_id)
                    ranges = list(map(float, message.ranges))
                    scan_record = {"ranges": ranges, "frame_id": message.header.frame_id,
                                   "intensities": list(message.intensities),
                                   **{key: float(getattr(message, key)) for key in
                                      ("angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min", "range_max")}}
                    scan_points[topic] += sum(math.isfinite(v) and message.range_min <= v <= message.range_max for v in ranges)
                    scan_geometry[topic] = {"range_count": len(ranges), **{key: scan_record[key] for key in
                                           ("angle_min", "angle_max", "angle_increment", "range_min", "range_max", "scan_time")}}
                    if topic.endswith("_raw"):
                        raw_scans[(topic.removesuffix("_raw"), stamp)].append(scan_record)
                        if ready["scenario_id"] == "P0":
                            own = _nearest(traces[robot], times[robot], stamp)
                            peer = ROBOTS[1] if robot == ROBOTS[0] else ROBOTS[0]
                            other = _nearest(traces[peer], times[peer], stamp)
                            if own is None or other is None:
                                continue
                            gt = own["ground_truth"]
                            sensor = next(item for item in ready["robots"][robot]["lidars"] if
                                          item["name"] in topic)
                            # Local transform is independently checked against actual tf_static below.
                            mount = sensor["translation_xyz_m"]
                            yaw = _yaw(gt["orientation_wxyz"])
                            origin = (gt["position_xyz_m"][0] + math.cos(yaw) * mount[0] - math.sin(yaw) * mount[1],
                                      gt["position_xyz_m"][1] + math.sin(yaw) * mount[0] + math.cos(yaw) * mount[1])
                            target = other["ground_truth"]["position_xyz_m"][:2]
                            radius = ready["robots"][peer]["footprint"]["collision_radius_m"] + .05
                            sensor_yaw = yaw + sensor["yaw_rad"]
                            for index, distance in enumerate(ranges):
                                if not math.isfinite(distance) or not message.range_min <= distance <= message.range_max:
                                    continue
                                angle = sensor_yaw + message.angle_min + index * message.angle_increment
                                endpoint = (origin[0] + distance * math.cos(angle), origin[1] + distance * math.sin(angle))
                                if math.dist(endpoint, target) <= radius:
                                    peer_hits[topic] += 1
                    else:
                        filtered_scans[(topic, stamp)].append(scan_record)
    ready_events = [row for row in events if row.get("kind") == "navigation_ready"]
    complete_events = [row for row in events if row.get("kind") == "scenario_complete"]
    window = None
    if len(ready_events) != 1 or len(complete_events) != 1 or ready_events[0]["sim_time_s"] >= complete_events[0]["sim_time_s"]:
        errors.append("decoded ROS sensor qualification window is missing or unordered")
    else:
        window = (ready_events[0]["sim_time_s"], complete_events[0]["sim_time_s"])
    scan_pairs = validate_scan_pairs(raw_scans, filtered_scans, coverage_window=window)
    errors.extend(scan_pairs["errors"])
    filtered_count.update(scan_pairs["paired_counts"])
    authorizations = {robot: unique_sequence_index(rows, "authorization_seq", errors, robot + " authorizations")
                      for robot, rows in authorization_rows.items()}
    for lane, rows in (("traffic", [row for row in events if "event_type" in row]),
                       ("mission", [row for row in events if "kind" in row])):
        unique_sequence_index(rows, "event_seq", errors, lane + " ROS events")
    if integrity != ["ok"] * len(databases):
        errors.append("rosbag SQLite integrity check failed")
    for robot in ROBOTS:
        prefix = f"/{robot}"
        for suffix in ("odom", "tf", "tf_static", "amcl_pose", "map", "navigation_map", "scan_front_raw", "scan_rear_raw", "scan_front", "scan_rear"):
            if counts[prefix + "/" + suffix] <= 0:
                errors.append(f"missing actual ROS messages {prefix}/{suffix}")
        for side in ("front", "rear"):
            for raw in ("", "_raw"):
                topic = f"{prefix}/scan_{side}{raw}"
                if scan_points[topic] <= 0 or frames[topic] != {f"{side}_scan"}:
                    errors.append(f"{topic}: scanner points/frame ID qualification failed")
            if filtered_count[f"{prefix}/scan_{side}"] < 5 or scan_pairs["covered_raw_counts"].get(f"{prefix}/scan_{side}", 0) < 5:
                errors.append(f"{prefix}/scan_{side}: insufficient raw/filtered timestamp pairs")
            transform = transforms[f"{prefix}/tf_static"].get(f"base_link->{side}_scan")
            definition = next(lidar for lidar in ready["robots"][robot]["lidars"] if lidar["name"] == side)
            if (transform is None or math.dist(transform["translation"], definition["translation_xyz_m"]) > 1e-6
                    or abs(math.atan2(math.sin(_yaw(transform["wxyz"]) - definition["yaw_rad"]),
                                      math.cos(_yaw(transform["wxyz"]) - definition["yaw_rad"]))) > 1e-6):
                errors.append(f"{robot}: {side} scanner TF is inconsistent with its authored local mount")
        if "odom->base_link" not in transforms[f"{prefix}/tf"] or "map->odom" not in transforms[f"{prefix}/tf"]:
            errors.append(f"{robot}: isolated TF localization chain is incomplete")
        if ready["scenario_id"] == "P0" and sum(count for topic, count in peer_hits.items() if topic.startswith(prefix + "/")) <= 0:
            errors.append(f"{robot}: no raw RTX returns join the peer's evaluated collision envelope")
    for forbidden in ("/tf", "/tf_static", "/odom", "/cmd_vel", "/scan_front", "/scan_rear"):
        if counts[forbidden]:
            errors.append(f"unexpected unnamespaced robot topic: {forbidden}")
    if not counts["/clock"] or not authorizations:
        errors.append("missing recorded clock/authorization stream")
    return {"decoded_cdr": True, "errors": errors, "integrity_check": "ok" if not any(v != "ok" for v in integrity) else integrity,
            "topic_counts": dict(counts), "positive_scan_points": dict(scan_points), "peer_envelope_ray_hits": dict(peer_hits),
            "filtered_scan_pairs": dict(filtered_count), "scan_frame_ids": {k: sorted(v) for k, v in frames.items()},
            "scan_pair_coverage": {key: value for key, value in scan_pairs.items() if key != "errors"},
            "scan_geometry": scan_geometry,
            "transforms": dict(transforms), "goal_statuses": dict(statuses), "authorizations": dict(authorizations), "events": events,
            "amcl_poses": dict(amcl_poses),
            "tf_samples": {robot: dict(edges) for robot, edges in tf_samples.items()},
            "databases": [{"path": path.name, "bytes": path.stat().st_size, "sha256": _sha256(path)} for path in databases]}


def _validate_dispatch(
    output: Path, run_id: str, scenario_id: str, mission: Mapping[str, Any],
    traces: Mapping[str, list[dict[str, Any]]], traffic: TrafficController,
    bag: Mapping[str, Any], errors: list[str], scenario: Mapping[str, Any],
) -> dict[str, Any]:
    actions = mission.get("actions", [])
    expected = {
        "P0": [], "F1": [("carter1_cross", 4), ("carter2_cross", 4)],
        "F2": [("carter2_exit", 5), ("carter2_exit", 4), ("carter1_cross", 4)],
        "F3": [("carter1_inside", 4), ("carter2_blocked", 5), ("carter2_yield", 4),
               ("carter1_exit", 4), ("carter2_resume", 4)],
    }[scenario_id]
    if [(row.get("route_id"), row.get("terminal_status")) for row in actions] != expected:
        errors.append("actual action sequence does not match the qualification scenario")
    accepted: dict[str, Mapping[str, Any]] = {}
    for action in actions:
        try:
            uid = UUID(action["goal_uuid"]).hex
            robot = action["robot_id"]
            if robot not in ROBOTS or uid in accepted:
                raise ValueError("invalid robot or duplicate goal UUID")
            route = scenario["routes"][action["route_id"]]
            if robot != route["robot_id"] or action["action_type"] != ("navigate_to_pose" if len(route["poses"]) == 1 else "navigate_through_poses"):
                raise ValueError("action does not match the authored robot/route/action type")
            accepted[uid] = action
            if not 0 <= action["accepted_sim_time_s"] <= action["terminal_sim_time_s"]:
                raise ValueError("invalid action acceptance/terminal time")
            if action["terminal_status"] == 5 and not (
                action["accepted_sim_time_s"] <= action["cancel_requested_sim_time_s"] <= action["terminal_sim_time_s"]
            ):
                raise ValueError("canceled goal lacks an ordered cancel request")
            statuses = bag.get("goal_statuses", {}).get(f"/{robot}/{action['action_type']}/_action/status", {})
            if statuses.get(uid) != action["terminal_status"]:
                raise ValueError("terminal action UUID/status is absent from decoded ROS status")
            if action["terminal_status"] == 4:
                trace = traces.get(robot, [])
                times = [row["sim_time_s"] for row in trace]
                final = _nearest(trace, times, action["terminal_sim_time_s"])
                goal = route["poses"][-1]
                if (final is None or math.dist(final["ground_truth"]["position_xyz_m"][:2], goal[:2]) > .35
                        or abs(math.atan2(math.sin(_yaw(final["ground_truth"]["orientation_wxyz"]) - goal[2]),
                                          math.cos(_yaw(final["ground_truth"]["orientation_wxyz"]) - goal[2]))) > .35):
                    raise ValueError("successful action does not reach its authored goal in independent physics evaluation")
        except (KeyError, TypeError, ValueError) as error:
            errors.append(f"action evidence invalid: {error}")
    if any(a.get("terminal_sim_time_s", math.inf) > b.get("accepted_sim_time_s", -math.inf) for a, b in zip(actions, actions[1:])):
        errors.append("scenario action order overlaps a required predecessor terminal result")

    events = _load_jsonl(output / "traffic-events.jsonl", errors)
    bag_events = bag.get("events", [])
    sent = [row for row in bag_events if row.get("kind") == "goal_sent"]
    if len(sent) != len(actions):
        errors.append("decoded ROS goal-sent journal does not match the actual action count")
    for action, event in zip(actions, sent):
        route = scenario.get("routes", {}).get(action.get("route_id"), {})
        if (event.get("robot_id") != route.get("robot_id") or event.get("route_id") != action.get("route_id")
                or event.get("poses") != route.get("poses") or event.get("action_type") != action.get("action_type")):
            errors.append("decoded ROS goal request differs from its authored route/robot/poses")
    bag_traffic = unique_sequence_index([row for row in bag_events if "event_type" in row], "event_seq", errors, "traffic ROS events")
    grants: dict[int, Mapping[str, Any]] = {}
    release_time: dict[int, float] = {}
    owners: dict[str, str] = {}
    snapshots: list[tuple[float, dict[str, str]]] = []
    last_seq = 0
    for event in events:
        if (event.get("run_id") != run_id or event.get("scenario_id") != scenario_id
                or event.get("event_seq") != last_seq + 1):
            errors.append("traffic event identity or sequence continuity failed")
            break
        last_seq += 1
        kind, robot = event["event_type"], event["robot_id"]
        if kind in {"reservation_granted", "occupancy_claimed"}:
            resources = event.get("resources", [event.get("resource")])
            if any(resource in owners for resource in resources):
                errors.append("traffic journal grants an already-owned resource")
            grants[event["reservation_id"]] = {**event, "resources": list(resources)}
            for resource in resources:
                owners[resource] = robot
        elif kind == "release_ack":
            release_time[event["reservation_id"]] = event["sim_time_s"]
            for resource in event["resources"]:
                if owners.pop(resource, None) != robot:
                    errors.append("traffic release ACK does not match its actual owner")
        if kind in {"reservation_granted", "occupancy_claimed", "release_ack", "recovery_decision", "resume_requested"}:
            if bag_traffic.get(event["event_seq"]) != event:
                errors.append(f"traffic event {event['event_seq']} is absent or changed in decoded ROS bag")
        snapshots.append((event["sim_time_s"], dict(owners)))
    if owners:
        errors.append("successful scenario retains an unreleased resource")
    snapshot_times = [row[0] for row in snapshots]
    ownership_violations = 0
    for robot, samples in traces.items():
        bag_leases = bag.get("authorizations", {}).get(robot, {})
        radius = json.loads((output / "sim-ready.json").read_text())["robots"][robot]["footprint"]["inflated_radius_m"]
        for sample in samples:
            authorization = sample["authorization"]
            payload = authorization.get("payload")
            if payload is not None:
                if payload.get("scope", {}).get("scenario_id") != scenario_id:
                    errors.append(f"{robot}: authorization scenario identity mismatch")
                    break
                seq = payload["authorization_seq"]
                recorded = bag_leases.get(seq, bag_leases.get(str(seq)))
                if recorded != payload:
                    errors.append(f"{robot}: applied authorization {seq} is absent or changed in decoded bag")
                    break
            if authorization["mode"] == "navigate" and authorization["valid"]:
                if payload is None:
                    errors.append(f"{robot}: navigate authorization payload is missing")
                    break
                scope = payload["scope"]
                goal = accepted.get(UUID(scope["goal_uuid"]).hex)
                grant = grants.get(scope["reservation_id"])
                route = scenario.get("routes", {}).get(scope["route_id"])
                if (goal is None or goal["robot_id"] != robot or goal["route_id"] != scope["route_id"]
                        or scope["scenario_id"] != scenario_id or grant is None or grant["robot_id"] != robot
                        or route is None or route["robot_id"] != robot
                        or list(scope["resources"]) != route["resources"] or scope["exit_id"] != route["exit_id"]
                        or list(grant["resources"]) != list(scope["resources"]) or grant["exit_id"] != scope["exit_id"]
                        or goal["accepted_sim_time_s"] > payload["issued_sim_time_s"] + 1e-9
                        or grant["sim_time_s"] > payload["issued_sim_time_s"] + 1e-9):
                    errors.append(f"{robot}: navigate permission is not bound to an accepted goal and granted bundle")
                    break
                if scope["reservation_id"] in release_time and payload["issued_sim_time_s"] >= release_time[scope["reservation_id"]]:
                    errors.append(f"{robot}: navigate authorization issued after its reservation release")
                    break
            if scenario_id != "P0":
                # Ground truth checks the executed path only; this data never feeds the dispatcher.
                stamp = authorization["evaluation_sim_time_s"]
                at = bisect_left(snapshot_times, stamp + 1e-8) - 1
                current = snapshots[at][1] if at >= 0 else {}
                moving = (any(abs(value) > 1e-8 for value in sample["wheel_target_rad_s"])
                          or abs(sample["odometry"]["linear_m_s"]) > .02)
                if moving:
                    xy = sample["ground_truth"]["position_xyz_m"][:2]
                    for resource_id, resource in traffic.resources.items():
                        polygon: Any = resource["polygon_xy"]
                        if circle_intersects_polygon(xy, radius, polygon) and current.get(resource_id) != robot:
                            ownership_violations += 1
    if ownership_violations:
        errors.append(f"{ownership_violations} moving footprint/resource frames lack actual traffic ownership")
    if scenario_id == "F1" and not any(row["event_type"] == "request_denied" and row["robot_id"] == "carter2" for row in events):
        errors.append("F1 has no actual contention denial")
    fault: dict[str, Any] = {}
    cycle: dict[str, Any] = {}
    if scenario_id == "F2":
        fault = validate_f2_fault(scenario, events, bag_events, traces["carter2"])
        errors.extend(fault["errors"])
    if scenario_id == "F3":
        ready = json.loads((output / "sim-ready.json").read_text())
        config = json.loads((PROJECT / "configs/factory-v3/g2.json").read_text())
        planning_radius = ready["robots"]["carter1"]["footprint"]["inflated_radius_m"] + config["localization_planning_margin_m"]
        cycle = validate_f3_cycle(events, traffic, bag.get("tf_samples", {}), planning_radius_m=planning_radius)
        errors.extend(cycle["errors"])
        for samples in traces.values():
            for sample in samples:
                scope = sample["authorization"].get("payload", {}).get("scope", {}) if sample["authorization"].get("payload") else {}
                if scope.get("route_id") in ("carter2_yield", "carter2_resume") and scope.get("recovery_id") != cycle.get("recovery_id"):
                    errors.append("F3 yield/resume lease uses a different recovery generation")
                    break
    return {"action_count": len(actions), "traffic_event_count": len(events),
            "reservation_count": len(grants), "release_ack_count": len(release_time),
            "moving_ownership_violation_frames": ownership_violations,
            **({"fault": fault["evaluation"]} if scenario_id == "F2" else {}),
            **({"cycle": cycle} if scenario_id == "F3" else {})}


def validate_g2_evidence(
    output: Path, run_id: str, scenario_id: str, *, bag_observation: Mapping[str, Any] | None = None,
    result_name: str = "g2-validation.json",
) -> dict[str, Any]:
    errors: list[str] = []
    evaluation: dict[str, Any] = {}
    ready = _load_json(output / "sim-ready.json", errors)
    scene = _load_json(output / "g2-scene.json", errors)
    mission = _load_json(output / "mission-result.json", errors)
    control = _load_json(output / "mission-control.json", errors)
    pair = _load_json(output / "pair-exit.json", errors)
    bag: dict[str, Any] = {}
    traces: dict[str, list[dict[str, Any]]] = {}
    documents = ((ready, "ready"), (scene, "validated"), (mission, "passed"), (control, "passed"))
    for document, status in documents:
        if (not isinstance(document, dict) or document.get("run_id") != run_id
                or document.get("scenario_id") != scenario_id or document.get("status") != status):
            errors.append(f"worker document is not a matching {scenario_id} {status} result")
    if not isinstance(pair, dict) or pair.get("run_id") != run_id or any(pair.get(key) != 0 for key in ("nav_exit_code", "sim_exit_code")):
        errors.append("worker pair exit codes are not both zero")
    if not errors:
        try:
            config = json.loads((PROJECT / "configs/factory-v3/g2.json").read_text())
            scenarios = json.loads((PROJECT / "configs/factory-v3/g2-scenarios.json").read_text())
            layout = Layout.from_file(PROJECT / "configs/factory-v3/layout.json")
            traffic = TrafficController.from_file(PROJECT / "configs/factory-v3/traffic.json", run_id=run_id)
            motion_bvh = scene.get("renderer", {}).get("motion_bvh", {})
            if motion_bvh.get("requested") is not True or motion_bvh.get("enabled_readback") is not True:
                errors.append("moving RTX sensors require an enabled Motion BVH runtime readback")
            sim_log = (output / "isaac.log").read_text(encoding="utf-8", errors="replace")
            bvh_warnings = ("Multi-tick is enabled but motion BVH is not active", "MotionBVH for lidar model not enabled")
            warning_counts = {warning: sim_log.count(warning) for warning in bvh_warnings}
            if any(warning_counts.values()):
                errors.append("Isaac reported unsupported multi-tick/missing Motion BVH sensor rendering")
            evaluation["renderer"] = {"motion_bvh": motion_bvh, "motion_bvh_warning_counts": warning_counts}
            for robot in ROBOTS:
                for lidar in ready["robots"][robot]["lidars"]:
                    if lidar.get("tick_rate_hz") != 10 or lidar.get("rotation_frequency_hz") != 10 or lidar.get("accumulate_outputs") is not True:
                        errors.append(f"{robot}: rotary RTX scan accumulation/tick configuration is invalid")
            for robot in ROBOTS:
                traces[robot], evaluation[robot] = _validate_robot(output, robot, ready, scene, layout, config, errors)
            first, second = traces[ROBOTS[0]], traces[ROBOTS[1]]
            if len(first) != len(second) or any((a["physics_frame"], a["sim_time_s"]) != (b["physics_frame"], b["sim_time_s"]) for a, b in zip(first, second)):
                errors.append("two robots do not have identical full-rate physics frame/time keys")
            distances = [math.dist(a["ground_truth"]["position_xyz_m"][:2], b["ground_truth"]["position_xyz_m"][:2]) for a, b in zip(first, second)]
            minimum = min(distances) if distances else 0.0
            required = sum(ready["robots"][robot]["footprint"]["inflated_radius_m"] for robot in ROBOTS)
            evaluation["separation"] = {"minimum_centre_distance_m": minimum, "required_circle_separation_m": required,
                                        "scope": "conservative collision-envelope plus clearance evaluation; contact sensor count not measured"}
            if minimum < required:
                errors.append("two qualified robot envelopes overlap during actual physics")
            evaluation["media"] = _validate_media(output, scene, traces, errors)
            planning_radius = max(ready["robots"][robot]["footprint"]["inflated_radius_m"] for robot in ROBOTS) + config["localization_planning_margin_m"]
            evaluation["authored_routes"] = qualify_authored_routes(layout, traffic, scenarios["scenarios"][scenario_id], planning_radius)
            errors.extend(evaluation["authored_routes"]["errors"])
            bag = dict(bag_observation) if bag_observation is not None else inspect_rosbag(output, traces, ready)
            if bag.get("decoded_cdr") is not True:
                errors.append("actual ROS CDR decoding is required")
            errors.extend(bag.get("errors", []))
            evaluation["dispatch"] = _validate_dispatch(output, run_id, scenario_id, mission, traces, traffic, bag, errors, scenarios["scenarios"][scenario_id])
            for robot in ROBOTS:
                times = [row["sim_time_s"] for row in traces[robot]]
                local_errors = []
                for row in bag.get("amcl_poses", {}).get(robot, []):
                    joined = _nearest(traces[robot], times, row["stamp_s"])
                    if joined is not None:
                        local_errors.append(math.dist((row["x_m"], row["y_m"]), joined["ground_truth"]["position_xyz_m"][:2]))
                if not local_errors:
                    errors.append(f"{robot}: no time-joined independent localization/ground-truth samples")
                else:
                    evaluation[robot]["localization_vs_ground_truth"] = {
                        "source": "actual decoded /" + robot + "/amcl_pose CDR, not the localization JSON journal",
                        "sample_count": len(local_errors), "maximum_xy_error_m": max(local_errors),
                        "rms_xy_error_m": math.sqrt(sum(value * value for value in local_errors) / len(local_errors)),
                    }
            if scenario_id == "P0":
                for robot in ROBOTS:
                    displacement = math.dist(traces[robot][0]["ground_truth"]["position_xyz_m"][:2], traces[robot][-1]["ground_truth"]["position_xyz_m"][:2])
                    if displacement > .02 or any(any(row["wheel_target_rad_s"]) for row in traces[robot]):
                        errors.append(f"{robot}: P0 stationary qualification moved")
            for key, relative in (("source", "scripts/isaac/g2_scene.py"), ("config", "configs/factory-v3/g2.json"),
                                  ("scenarios", "configs/factory-v3/g2-scenarios.json"), ("layout", "configs/factory-v3/layout.json")):
                if scene.get("inputs", {}).get(key, {}).get("sha256") != _sha256(PROJECT / relative):
                    errors.append(f"runtime/validator source pin mismatch: {relative}")
            for key, relative in (("mission_source", "scripts/ros/g2_mission.py"), ("launch_source", "scripts/ros/g2.launch.py"),
                                  ("config", "configs/factory-v3/g2.json"), ("scenarios", "configs/factory-v3/g2-scenarios.json"),
                                  ("nav2_template", "configs/factory-v3/nav2-g2.yaml")):
                if mission.get("inputs", {}).get(key, {}).get("sha256") != _sha256(PROJECT / relative):
                    errors.append(f"ROS runtime/validator source pin mismatch: {relative}")
        except (OSError, ValueError, TypeError, KeyError, IndexError, ImportError, sqlite3.Error) as error:
            errors.append(f"G2 evidence inspection failed: {type(error).__name__}: {error}")
    result_safe = _safe_relative(result_name)
    if result_safe is None:
        errors.append("unsafe validation result name")
        result_safe = "g2-validation.json"
    # The raw authorization/event streams stay in the bag. Keep the report bounded.
    bounded_bag = {key: value for key, value in bag.items() if key not in {"authorizations", "events", "amcl_poses", "tf_samples"}}
    manifest = _manifest(output, {result_safe}, errors)
    return {"schema_version": 1, "qualification": "g2_multi_robot_traffic", "run_id": run_id,
            "scenario_id": scenario_id, "passed": not errors, "errors": errors, "evaluation": evaluation,
            "bag": bounded_bag, "artifact_manifest": manifest,
            "claims": {"navigation_inputs": "per-namespace encoders and RTX scans with authored initial pose/maps; no online simulator truth",
                       "ground_truth_use": "post-run clearance, peer perception, localization error and media evaluation only",
                       "scope": "one actual two-robot qualification scenario, not repeatability or production safety certification"}}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario", choices=("P0", "F1", "F2", "F3"), required=True)
    parser.add_argument("--write-result", default="g2-validation.json")
    args = parser.parse_args()
    if not args.output.is_absolute() or _safe_relative(args.write_result) is None:
        parser.error("absolute output and safe relative result path required")
    output = args.output.resolve(strict=True)
    target = (output / args.write_result).resolve(strict=False)
    if not target.is_relative_to(output):
        parser.error("validation result must remain inside output")
    report = validate_g2_evidence(output, args.run_id, args.scenario, result_name=args.write_result)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print("FACTORY_OPS_G2_VALIDATION=" + ("PASS" if report["passed"] else "FAIL"))
    if report["errors"]:
        print(json.dumps(report["errors"], ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
