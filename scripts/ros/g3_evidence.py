#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Post-worker G3 physical evidence validator; never a primary comparison."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import sys
from typing import Any, Mapping, Sequence
from uuid import UUID

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.fleet_execution import make_chunk, path_content_sha256, qualify_computed_path, path_header_stamp_ns
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.g3_session import SCOPE, sha, verify_session
from factory_ops.layout import Layout
from factory_ops.motion_authorization import AuthorizationError, LeaseGuard
from factory_ops.navigation import GoalPose
from factory_ops.operations import Ledger
from scripts.ros.g1_evidence import _load_json, _load_jsonl, _manifest, _safe_relative
from scripts.ros.g2_evidence import (
    ROBOTS, _nearest, _validate_media, _validate_robot, decoded_map_pose, inspect_rosbag,
    index_decoded_tf, decoded_map_pose_indexed,
)
from scripts.isaac.nav_scene import _tilt_evaluation
from scripts.isaac.g2_scene import _ros_stamp_parts
from scripts.ros.g3_mission import terminal_recording_boundary

PROJECT = Path(__file__).resolve().parents[2]
RESULT = "g3-evidence.json"


def _hash_json(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _uuid_hex(value: object) -> str | None:
    try:
        return UUID(str(value)).hex
    except (ValueError, TypeError, AttributeError):
        return None


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _path_payload(path: Any) -> dict[str, Any]:
    def header(value: Any) -> dict[str, Any]:
        return {"frame_id": str(value.frame_id), "sec": int(value.stamp.sec), "nanosec": int(value.stamp.nanosec)}

    return {
        "header": header(path.header),
        "poses": [
            {
                "header": header(item.header),
                "position_xyz_m": [float(item.pose.position.x), float(item.pose.position.y), float(item.pose.position.z)],
                "orientation_xyzw": [float(item.pose.orientation.x), float(item.pose.orientation.y),
                                     float(item.pose.orientation.z), float(item.pose.orientation.w)],
            }
            for item in path.poses
        ],
    }


def _decode_g3_topics(output: Path, run_id: str, scenario_id: str) -> dict[str, Any]:
    """Decode G3 JSON streams, Path CDR and action statuses from immutable bag DBs."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    databases = sorted((output / "rosbag").glob("*.db3"))
    if not databases:
        raise ValueError("no actual rosbag SQLite database")
    integrity: list[str] = []
    counts: Counter[str] = Counter()
    events: list[dict[str, Any]] = []
    authorizations: list[dict[str, Any]] = []
    reservations: list[dict[str, Any]] = []
    plan_records: dict[str, list[dict[str, Any]]] = defaultdict(list)
    odometry: dict[str, list[dict[str, Any]]] = defaultdict(list)
    statuses: dict[str, dict[str, int]] = defaultdict(dict)
    follow_rpc: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(lambda: defaultdict(list))
    errors: list[str] = []
    types: dict[str, Any] = {}
    for database in databases:
        with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as connection:
            integrity.extend(str(row[0]) for row in connection.execute("PRAGMA integrity_check"))
            topics = {int(row[0]): (str(row[1]), str(row[2])) for row in connection.execute("SELECT id,name,type FROM topics")}
            for topic_id, count in connection.execute("SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id"):
                counts[topics[int(topic_id)][0]] += int(count)
            for topic_id, data in connection.execute("SELECT topic_id,data FROM messages ORDER BY timestamp"):
                topic, type_name = topics[int(topic_id)]
                selected = (topic in {"/factory/g3/events", "/factory/g3/authorizations", "/factory/g3/reservations"}
                            or topic.endswith(("/plan", "/odom", "/_action/status", "/_service_event")))
                if not selected:
                    continue
                raw = bytes(data)
                if topic.endswith("/plan"):
                    robot = topic.split("/")[1]
                    if robot in ROBOTS:
                        if type_name not in types:
                            types[type_name] = get_message(type_name)
                        path = deserialize_message(raw, types[type_name])
                        payload = _path_payload(path)
                        plan_records[robot].append({"cdr_sha256": hashlib.sha256(raw).hexdigest(),
                                                    "content_sha256": path_content_sha256(payload),
                                                    "payload": payload})
                    continue
                if type_name not in types:
                    types[type_name] = get_message(type_name)
                message = deserialize_message(raw, types[type_name])
                if topic.endswith("/_service_event"):
                    robot = topic.split("/")[1]
                    operation = next((name for name in ("send_goal", "get_result", "cancel_goal")
                                      if f"/{name}/_service_event" in topic), None)
                    if robot not in ROBOTS or operation is None:
                        errors.append(f"{topic}: unknown FollowPath service event")
                        continue
                    info = message.info
                    transaction = f"{bytes(info.client_gid).hex()}:{int(info.sequence_number)}"
                    event_type = int(info.event_type)
                    record: dict[str, Any] = {"transaction": transaction, "event_type": event_type}
                    if event_type == 0 and len(message.request) == 1:
                        request = message.request[0]
                        request_goal_id = request.goal_info.goal_id if operation == "cancel_goal" else request.goal_id
                        record["goal_uuid"] = bytes(request_goal_id.uuid).hex()
                        if operation == "send_goal":
                            payload = _path_payload(request.goal.path)
                            record.update({"path_content_sha256": path_content_sha256(payload), "path": payload,
                                           "controller_id": str(request.goal.controller_id),
                                           "goal_checker_id": str(request.goal.goal_checker_id),
                                           "progress_checker_id": str(request.goal.progress_checker_id)})
                    elif event_type == 3 and len(message.response) == 1:
                        response = message.response[0]
                        if operation == "send_goal":
                            record.update({"accepted": bool(response.accepted),
                                           "stamp": {"sec": int(response.stamp.sec), "nanosec": int(response.stamp.nanosec)}})
                        elif operation == "get_result":
                            record.update({"status": int(response.status), "error_code": int(response.result.error_code),
                                           "error_msg": str(response.result.error_msg)})
                    else:
                        errors.append(f"{topic}: malformed or unsupported service event phase")
                        continue
                    follow_rpc[robot][operation].append(record)
                    continue
                if topic.endswith("/odom"):
                    robot = topic.split("/")[1]
                    if robot in ROBOTS:
                        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                        stamp_ns = path_header_stamp_ns({"sec": int(message.header.stamp.sec),
                                                        "nanosec": int(message.header.stamp.nanosec)})
                        odometry[robot].append({"stamp_s": stamp, "stamp_ns": stamp_ns,
                                                "linear_m_s": float(message.twist.twist.linear.x),
                                                "angular_rad_s": float(message.twist.twist.angular.z)})
                    continue
                if topic.endswith("/_action/status"):
                    for status in message.status_list:
                        statuses[topic][bytes(status.goal_info.goal_id.uuid).hex()] = int(status.status)
                    continue
                value = json.loads(str(message.data))
                expected_scenario = value.get("scope", {}).get("scenario_id") if topic.endswith("authorizations") else value.get("scenario_id")
                if topic.endswith("reservations"):
                    expected_scenario = scenario_id
                if not isinstance(value, dict) or value.get("run_id") != run_id or expected_scenario != scenario_id:
                    errors.append(f"{topic}: decoded identity mismatch")
                elif topic.endswith("authorizations"):
                    authorizations.append(value)
                elif topic.endswith("reservations"):
                    reservations.append(value)
                else:
                    events.append(value)
    if integrity != ["ok"] * len(databases):
        errors.append("rosbag SQLite integrity check failed")
    return {
        "decoded_cdr": True,
        "integrity_check": "ok" if not any(value != "ok" for value in integrity) else integrity,
        "topic_counts": dict(counts),
        "events": events,
        "authorizations": authorizations,
        "reservations": reservations,
        "plan_records": dict(plan_records),
        "odometry": dict(odometry),
        "goal_statuses": dict(statuses),
        "follow_rpc": {robot: dict(value) for robot, value in follow_rpc.items()},
        "errors": errors,
        "databases": [{"path": path.name, "bytes": path.stat().st_size, "sha256": sha(path)} for path in databases],
    }


def validate_stop_evidence(
    actions: Sequence[Mapping[str, Any]], traces: Mapping[str, Sequence[Mapping[str, Any]]],
    runtime: Mapping[str, Any], errors: list[str], bag: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Prove hold application precedes a fresh, continuous odometry stop window."""
    passed = 0
    for index, action in enumerate(actions):
        robot = action.get("robot_id")
        ack, samples = action.get("hold_ack"), action.get("stopped_odometry")
        try:
            if robot not in ROBOTS or not isinstance(ack, Mapping) or not isinstance(samples, list):
                raise ValueError("missing hold ACK or stopped odometry")
            sequence = ack.get("authorization_seq")
            frame = ack.get("physics_frame")
            scope = ack.get("scope", {})
            if (ack.get("mode") != "hold" or ack.get("valid") is not True
                    or scope.get("reservation_id") != action.get("generation")
                    or scope.get("goal_uuid") != action.get("follow_action_uuid")):
                raise ValueError("hold ACK scope does not bind the terminal goal reservation")
            matched = [row for row in traces[robot] if row.get("physics_frame") == frame]
            if len(matched) != 1:
                raise ValueError("hold ACK does not bind one applied physics frame")
            authorization = matched[0].get("authorization", {})
            command = matched[0].get("motion_profile", {}).get("input_command")
            if (authorization.get("sequence") != sequence or authorization.get("mode") != "hold"
                    or authorization.get("valid") is not True or command != {"linear_m_s": 0.0, "angular_rad_s": 0.0}):
                raise ValueError("hold ACK does not match the applied zero-input sample")
            stamps = [float(row["stamp_s"]) for row in samples]
            if (len(stamps) < 2 or any(not math.isfinite(value) for value in stamps)
                    or stamps != sorted(set(stamps)) or stamps[0] <= float(ack["evaluation_sim_time_s"])
                    or stamps[-1] - stamps[0] + 1e-12 < float(runtime["stopped_window_s"])
                    or any(b - a > float(runtime["odom_ttl_s"]) + 1e-12 for a, b in zip(stamps, stamps[1:]))):
                raise ValueError("post-ACK odometry is stale, sparse, repeated, or too short")
            if any(abs(float(row["linear_m_s"])) > float(runtime["stopped_linear_m_s"])
                   or abs(float(row["angular_rad_s"])) > float(runtime["stopped_angular_rad_s"])
                   for row in samples):
                raise ValueError("post-ACK odometry exceeds stopped limits")
            trace_by_stamp = {float(row["sim_time_s"]): row for row in traces[robot] if "sim_time_s" in row}
            bag_odometry = bag.get("odometry", {}).get(robot, []) if bag is not None else []
            for row in samples:
                stamp = float(row["stamp_s"])
                actual = next((value for time, value in trace_by_stamp.items() if abs(time - stamp) < 1e-8), None)
                odometry = actual.get("odometry", {}) if actual is not None else {}
                if (actual is None
                        or not math.isclose(float(row["linear_m_s"]), float(odometry.get("linear_m_s", math.nan)), abs_tol=1e-9)
                        or not math.isclose(float(row["angular_rad_s"]), float(odometry.get("angular_rad_s", math.nan)), abs_tol=1e-9)):
                    raise ValueError("stopped odometry does not match the actual trajectory sample")
            if bag is not None:
                interval = [row for row in bag_odometry
                            if float(ack["evaluation_sim_time_s"]) < float(row["stamp_s"]) <= float(action["stopped_sim_time_s"])]
                interval_stamps = [float(row["stamp_s"]) for row in interval]
                if not interval or interval_stamps != sorted(set(interval_stamps)):
                    raise ValueError("decoded post-ACK /odom stamps are missing, duplicated, or nonmonotonic")
                segment: list[dict[str, float]] = []
                for row in interval:
                    normalized = {"stamp_s": float(row["stamp_s"]), "linear_m_s": float(row["linear_m_s"]),
                                  "angular_rad_s": float(row["angular_rad_s"])}
                    moving = (abs(normalized["linear_m_s"]) > float(runtime["stopped_linear_m_s"])
                              or abs(normalized["angular_rad_s"]) > float(runtime["stopped_angular_rad_s"]))
                    stale_gap = bool(segment) and normalized["stamp_s"] - segment[-1]["stamp_s"] > float(runtime["odom_ttl_s"]) + 1e-12
                    if moving:
                        segment = []
                    else:
                        if stale_gap:
                            segment = []
                        segment.append(normalized)
                claimed = [{"stamp_s": float(row["stamp_s"]), "linear_m_s": float(row["linear_m_s"]),
                            "angular_rad_s": float(row["angular_rad_s"])} for row in samples]
                measurement_start = float(action["stop_measurement_start_stamp_s"])
                starts = [index for index, row in enumerate(segment)
                          if abs(row["stamp_s"] - measurement_start) < 1e-8]
                suffix = segment[starts[0]:] if len(starts) == 1 else []
                if (not math.isclose(measurement_start, claimed[0]["stamp_s"], abs_tol=1e-8)
                        or measurement_start <= float(ack["evaluation_sim_time_s"])
                        or suffix != claimed or len(suffix) < 2
                        or suffix[-1]["stamp_s"] - suffix[0]["stamp_s"] + 1e-12 < float(runtime["stopped_window_s"])):
                    raise ValueError("claimed stop window is not an exact continuous suffix of decoded /odom")
            if float(action["stopped_sim_time_s"]) < stamps[-1]:
                raise ValueError("goal was marked finished before the stop window")
            if action.get("leg_kind") == "business" and action.get("chunk", {}).get("final"):
                if float(action.get("handoff_sim_time_s", -math.inf)) - float(action["stopped_sim_time_s"]) + 1e-12 < 5.0:
                    raise ValueError("business handoff does not preserve the authored five-second dwell")
            passed += 1
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"action {index} stop evidence: {exc}")
    return {"action_count": len(actions), "passed_stop_count": passed}


def validate_actions(
    actions: Sequence[Mapping[str, Any]], book: RouteBook, bag: Mapping[str, Any],
    errors: list[str],
) -> dict[str, Any]:
    """Recompile chunks and bind planner/follower UUIDs, complete Path content and reservations."""
    expected_plans: dict[str, list[dict[str, Any]]] = defaultdict(list)
    payload_hashes: list[str] = []
    action_uuids: set[tuple[str, str, str]] = set()
    for index, action in enumerate(actions):
        try:
            robot = str(action["robot_id"])
            chunk = action["chunk"]
            final_goal = GoalPose(**chunk["final_goal"])
            canonical = make_chunk(book, chunk["start"], chunk["target"], closed=chunk["closed"], final_goal=final_goal)
            if canonical.to_dict() != chunk:
                raise ValueError("chunk differs from current source compilation")
            path = action["path"]
            points = [[pose["position_xyz_m"][0], pose["position_xyz_m"][1]] for pose in path["poses"]]
            stamp = path_header_stamp_ns(path["header"]) / 1e9
            requested = float(action["requested_sim_time_s"])
            admission = float(action["path_geometry"]["now_s"])
            accepted = float(action["accepted_sim_time_s"])
            if not requested <= admission <= accepted:
                raise ValueError("computed path admission time is outside request/acceptance order")
            start_xy = decoded_map_pose(bag.get("tf_samples", {}).get(robot, {}), admission)
            geometry = qualify_computed_path(
                book, canonical, points, frame_id=path["header"]["frame_id"], stamp_s=stamp,
                now_s=admission, start_xy=start_xy,
            )
            if geometry != action["path_geometry"]:
                raise ValueError("path geometry report does not rederive")
            reservation = {tuple(tile) for tile in action["reservation_tiles"]}
            if not {tuple(tile) for tile in geometry["swept_tiles"]}.issubset(reservation):
                raise ValueError("computed path leaves its reservation")
            compute, follow = _uuid_hex(action["planner_action_uuid"]), _uuid_hex(action["follow_action_uuid"])
            if compute is None or follow is None:
                raise ValueError("action UUID is malformed")
            for name, uid in (("compute_path_to_pose", compute), ("follow_path", follow)):
                identity = (robot, name, uid)
                if identity in action_uuids:
                    raise ValueError(f"duplicate {name} UUID in action records")
                action_uuids.add(identity)
                topic = f"/{robot}/{name}/_action/status"
                if bag.get("goal_statuses", {}).get(topic, {}).get(uid) != 4:
                    raise ValueError(f"{name} UUID lacks decoded SUCCEEDED status")
            if action.get("planner_status") != 4 or action.get("planner_error_code") != 0 or action.get("status") != 4 or action.get("error_code") != 0:
                raise ValueError("journaled action terminal status is not successful")
            content_hash = path_content_sha256(path)
            if (action.get("path_content_sha256") != content_hash
                    or action.get("follow_goal_path_content_sha256") != content_hash):
                raise ValueError("FollowPath goal does not contain the admitted complete Path content")
            if chunk["final"] and action.get("leg_kind") == "business" and action.get("mission_goal_uuid") != action["follow_action_uuid"]:
                raise ValueError("final station FollowPath UUID differs from mission intent UUID")
            expected_plans[robot].append({"content_sha256": content_hash, "payload": path})
            payload_hashes.append(content_hash)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append(f"action {index} binding: {exc}")
    for robot in ROBOTS:
        observed = bag.get("plan_records", {}).get(robot, [])
        observed_content = [{"content_sha256": row.get("content_sha256"), "payload": row.get("payload")}
                            for row in observed]
        if observed_content != expected_plans[robot]:
            errors.append(f"{robot}: decoded /plan complete content differs from ComputePath results")
        if any(not re.fullmatch(r"[0-9a-f]{64}", str(row.get("cdr_sha256"))) for row in observed):
            errors.append(f"{robot}: decoded /plan raw CDR integrity hash is malformed")
        if bag.get("topic_counts", {}).get(f"/{robot}/received_global_plan", 0) <= 0:
            errors.append(f"{robot}: transformed/pruned received_global_plan evidence is absent")
    return {"action_count": len(actions), "path_payload_sha256": payload_hashes,
            "observed_plan_cdr_sha256": {robot: [row["cdr_sha256"] for row in bag.get("plan_records", {}).get(robot, [])]
                                          for robot in ROBOTS}}


def validate_follow_rpc(
    actions: Sequence[Mapping[str, Any]], bag: Mapping[str, Any], errors: list[str],
) -> dict[str, Any]:
    """Bind actual introspected FollowPath SendGoal/GetResult RPC transactions."""
    passed = 0
    seen_goals: set[tuple[str, str]] = set()
    for robot in ROBOTS:
        expected = [row for row in actions if row.get("robot_id") == robot]
        operations = bag.get("follow_rpc", {}).get(robot, {})
        paired: dict[str, dict[str, dict[str, Any]]] = {}
        for operation in ("send_goal", "get_result"):
            groups: dict[str, dict[str, dict[str, Any]]] = defaultdict(dict)
            for row in operations.get(operation, []):
                phase = "request" if row.get("event_type") == 0 else "response"
                transaction = str(row.get("transaction"))
                if phase in groups[transaction]:
                    errors.append(f"{robot}: duplicate {operation} {phase} transaction")
                groups[transaction][phase] = row
            if len(groups) != len(expected) or any(set(group) != {"request", "response"} for group in groups.values()):
                errors.append(f"{robot}: FollowPath {operation} RPC cardinality/phase differs from actions")
            paired[operation] = groups  # type: ignore[assignment]
        if operations.get("cancel_goal"):
            errors.append(f"{robot}: unexpected FollowPath cancellation RPC")
        send_by_goal = {group.get("request", {}).get("goal_uuid"): group
                        for group in paired.get("send_goal", {}).values()}
        result_by_goal = {group.get("request", {}).get("goal_uuid"): group
                          for group in paired.get("get_result", {}).values()}
        if len(send_by_goal) != len(paired.get("send_goal", {})) or len(result_by_goal) != len(paired.get("get_result", {})):
            errors.append(f"{robot}: duplicate FollowPath goal UUID in RPC evidence")
        for index, action in enumerate(expected):
            goal_hex = _uuid_hex(action.get("follow_action_uuid"))
            key = (robot, str(goal_hex))
            if goal_hex is None or key in seen_goals:
                errors.append(f"{robot}: action {index} has a duplicate/malformed FollowPath UUID")
                continue
            seen_goals.add(key)
            send, result = send_by_goal.get(goal_hex), result_by_goal.get(goal_hex)
            if send is None or result is None:
                errors.append(f"{robot}: action {index} lacks matching FollowPath RPC transactions")
                continue
            request, response = send["request"], send["response"]
            result_response = result["response"]
            if (request.get("path") != action.get("path")
                    or request.get("path_content_sha256") != action.get("path_content_sha256")
                    or request.get("controller_id") != "FollowPath"
                    or request.get("goal_checker_id") != "goal_checker"
                    or request.get("progress_checker_id") != "progress_checker"
                    or response.get("accepted") is not True
                    or response.get("stamp") != action.get("server_accepted_stamp")
                    or result_response.get("status") != action.get("status")
                    or result_response.get("error_code") != action.get("error_code")
                    or result_response.get("error_msg") != action.get("error_msg")):
                errors.append(f"{robot}: action {index} differs from actual FollowPath RPC request/response")
                continue
            passed += 1
    return {"action_count": len(actions), "passed_rpc_count": passed}


def validate_final_state(
    mission_result: Mapping[str, Any], session: Mapping[str, Any], errors: list[str],
) -> dict[str, Any]:
    mission = mission_result.get("mission")
    if not isinstance(mission, Mapping):
        errors.append("mission export is missing")
        return {}
    expected = len(session["workload"]["orders"])
    if (mission.get("completed") is not True or mission.get("failed") is not False
            or mission.get("censored") is not False or mission.get("replay_matches") is not True
            or mission.get("active_intents") != {} or mission.get("expected_order_count") != expected
            or mission.get("completed_order_count") != expected):
        errors.append("mission is not a complete replay-matching workload")
    snapshot = mission.get("snapshot", {})
    orders = snapshot.get("orders", {}) if isinstance(snapshot, Mapping) else {}
    if len(orders) != expected or any(row.get("state") != "complete" for row in orders.values() if isinstance(row, Mapping)):
        errors.append("ledger orders are not all COMPLETE")
    try:
        replay = Ledger.replay(list(mission["events"]))
        if replay.snapshot() != snapshot or replay.events != mission["events"]:
            errors.append("independent ledger replay differs")
    except (KeyError, TypeError, ValueError) as exc:
        errors.append(f"independent ledger replay failed: {exc}")
    route = mission_result.get("route_snapshot", {})
    holds = route.get("holds", {}) if isinstance(route, Mapping) else {}
    anchors = mission_result.get("physical_anchor_by_robot")
    initial = session["workload"]["initial_robots"]
    if anchors != initial or set(holds) != set(ROBOTS):
        errors.append("both robots are not retained at P1/P2")
    for robot in ROBOTS:
        hold = holds.get(robot, {})
        if hold.get("parked") is not True or not hold.get("tiles") or hold.get("goal_uuid") != hold.get("accepted_goal_uuid"):
            errors.append(f"{robot}: final parked hold is empty or unbound")
    return {"expected_orders": expected, "completed_orders": mission.get("completed_order_count"), "parked_holds": holds}


def validate_handoff_bindings(
    mission_result: Mapping[str, Any], actions: Sequence[Mapping[str, Any]], book: RouteBook,
    runtime: Mapping[str, Any], errors: list[str],
) -> dict[str, Any]:
    """Bind each physical business handoff to one navigation and ledger commit."""
    mission = mission_result.get("mission", {})
    ledger_events = [row for row in mission.get("events", []) if row.get("transition") in {"pickup", "drop"}]
    journal_rows = [row for row in mission.get("navigation_journal", []) if row.get("kind") == "result_accepted"]
    business = [row for row in actions if row.get("leg_kind") == "business" and row.get("chunk", {}).get("final") is True]
    seen: set[tuple[object, object]] = set()
    journal_bindings: set[tuple[object, object]] = set()
    for row in journal_rows:
        result = row.get("result", {})
        receipt = row.get("receipt", {})
        binding = (result.get("result_id"), receipt.get("ledger_seq"))
        if binding in journal_bindings:
            errors.append("duplicate navigation journal result/ledger binding")
        journal_bindings.add(binding)
    passed = 0
    for index, action in enumerate(business):
        try:
            result, receipt = action["navigation_result"], action["receipt"]
            binding = (result["result_id"], result["goal_id"])
            if binding in seen:
                raise ValueError("duplicate navigation result/goal binding")
            seen.add(binding)
            if (receipt.get("accepted") is not True or receipt.get("duplicate") is not False
                    or receipt.get("result_id") != result.get("result_id")
                    or receipt.get("goal_id") != result.get("goal_id")
                    or result.get("goal_id") != action.get("follow_action_uuid")
                    or result.get("status") != 4):
                raise ValueError("navigation result and receipt identity/status differ")
            matching_events = [row for row in ledger_events if row.get("ledger_seq") == receipt.get("ledger_seq")]
            if len(matching_events) != 1:
                raise ValueError("receipt does not select exactly one PICKUP/DROP ledger event")
            event = matching_events[0]
            if (event.get("robot_id") != action.get("robot_id")
                    or event.get("task_id") != result.get("task_id")
                    or event.get("station_id") != result.get("station")
                    or event.get("attempt") != result.get("attempt")
                    or event.get("transition") != receipt.get("committed_transition")):
                raise ValueError("ledger event differs from robot/task/station/attempt/transition binding")
            matching_journal = [row for row in journal_rows if row.get("robot_id") == action.get("robot_id")
                                and row.get("result") == result and row.get("receipt") == receipt]
            if len(matching_journal) != 1:
                raise ValueError("navigation journal does not contain exactly one matching accepted result")
            received = float(result["received_sim_time"])
            pose = result["pose"]
            pose_stamp = float(pose["stamp"])
            station = str(result["station"])
            station_xy = book.nodes[station]
            expected_yaw = float(runtime["anchor_yaw_rad"][station])
            if (pose.get("frame_id") != "map" or not 0 <= received - pose_stamp <= float(runtime["pose_ttl_s"])
                    or math.dist((float(pose["x"]), float(pose["y"])), station_xy) > float(runtime["station_xy_tolerance_m"])
                    or abs(math.atan2(math.sin(float(pose["yaw"]) - expected_yaw),
                                      math.cos(float(pose["yaw"]) - expected_yaw))) > float(runtime["station_yaw_tolerance_rad"])):
                raise ValueError("navigation result arrival pose/time is not fresh at the bound station")
            origin = float(mission_result["origin_sim_time_s"])
            if (not math.isclose(float(action["handoff_sim_time_s"]) - origin, received, abs_tol=1e-6)
                    or float(action["stopped_sim_time_s"]) - origin > received):
                raise ValueError("handoff/result time does not follow the measured stop on one clock")
            passed += 1
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append(f"business handoff {index}: {exc}")
    if len(business) != len(ledger_events) or len(journal_rows) != len(business) or passed != len(ledger_events):
        errors.append("PICKUP/DROP ledger commits and final business actions are not exact one-to-one sets")
    return {"business_action_count": len(business), "ledger_handoff_count": len(ledger_events),
            "passed_binding_count": passed}


def validate_final_parking(
    mission_result: Mapping[str, Any], route_events: Sequence[Mapping[str, Any]], book: RouteBook,
    runtime: Mapping[str, Any], errors: list[str],
) -> dict[str, Any]:
    route = mission_result.get("route_snapshot", {})
    holds = route.get("holds", {}) if isinstance(route, Mapping) else {}
    anchors = mission_result.get("physical_anchor_by_robot", {})
    radius = book.planning_radius_m + float(runtime["reservation_tracking_margin_m"])
    bounds = (book.layout.bounds.xmin, book.layout.bounds.ymin, book.layout.bounds.xmax, book.layout.bounds.ymax)
    checked = 0
    for robot, expected_anchor in zip(ROBOTS, ("P1", "P2")):
        hold = holds.get(robot, {})
        expected_tiles = sorted(swept_tiles([book.nodes[expected_anchor]], radius,
                                            tile_size_m=book.tile_size_m, bounds_xy=bounds))
        events = [row for row in route_events if row.get("robot_id") == robot
                  and row.get("event_type") == "parked_hold_shrunk"]
        final = events[-1] if events else {}
        if (anchors.get(robot) != expected_anchor or hold.get("parked") is not True
                or hold.get("tiles") != [list(tile) for tile in expected_tiles]
                or final.get("successor_generation") != hold.get("generation")
                or final.get("predecessor_generation") != hold.get("predecessor_generation")
                or final.get("tiles") != hold.get("tiles")
                or final.get("route_id") != hold.get("route_id")
                or final.get("goal_uuid") != hold.get("goal_uuid")
                or final.get("accepted_goal_uuid") != hold.get("accepted_goal_uuid")
                or final.get("parked") is not True):
            errors.append(f"{robot}: final private-pocket hold/shrink binding is invalid")
        else:
            checked += 1
    return {"parking_hold_count": checked, "reservation_radius_m": radius}


def validate_route_event_binding(
    route_events: Sequence[Mapping[str, Any]], decoded: object,
    mission_result: Mapping[str, Any], errors: list[str],
) -> dict[str, Any]:
    local = list(route_events)
    if decoded != local:
        errors.append("decoded G3 reservation stream differs from the local route journal")
    if mission_result.get("route_snapshot", {}).get("events") != local:
        errors.append("terminal route snapshot events differ from the local route journal")
    return {"route_event_count": len(local)}


def validate_terminal_recording(
    mission_result: Mapping[str, Any], scene: Mapping[str, Any],
    traces: Mapping[str, Sequence[Mapping[str, Any]]], bag: Mapping[str, Any],
    errors: list[str],
) -> dict[str, Any]:
    """Bind recorder shutdown to the final physics frame and actual bag odometry."""
    sealed = mission_result.get("terminal_recording")
    if not isinstance(sealed, Mapping):
        errors.append("terminal recording boundary is missing")
        return {"passed_robot_count": 0}
    received = sealed.get("received_odometry_ns")
    if not isinstance(received, Mapping) or set(received) != set(ROBOTS):
        errors.append("terminal recording odometry acknowledgment robot set is invalid")
        return {"passed_robot_count": 0}
    try:
        minimum = sealed.get("minimum_sim_time_ns")
        if type(minimum) is not int:
            raise ValueError("minimum terminal time must be integer nanoseconds")
        boundary = terminal_recording_boundary(
            scene, received, run_id=mission_result["run_id"], scenario_id=mission_result["scenario_id"],
            minimum_sim_time_ns=minimum,
        )
    except (ValueError, TypeError, KeyError) as error:
        errors.append(f"terminal recording boundary is invalid: {error}")
        return {"passed_robot_count": 0}
    if boundary is None or dict(sealed) != boundary:
        errors.append("terminal recording acknowledgment differs from the validated final scene")
        return {"passed_robot_count": 0}
    passed = 0
    odometry = bag.get("odometry", {})
    for robot in ROBOTS:
        rows = traces.get(robot, ())
        final_row = rows[-1] if rows else {}
        final_time = _finite_number(final_row.get("sim_time_s"))
        final_stamp = None
        if final_time is not None and final_time >= 0:
            sec, nanosec = _ros_stamp_parts(final_time)
            final_stamp = {"sec": sec, "nanosec": nanosec}
        samples = odometry.get(robot, ()) if isinstance(odometry, Mapping) else ()
        bag_stamps = {row.get("stamp_ns") for row in samples
                      if isinstance(row, Mapping) and type(row.get("stamp_ns")) is int}
        if (final_row.get("physics_frame") != boundary["final_physics_frame"]
                or final_stamp != boundary["final_stamp"]
                or boundary["final_sim_time_ns"] not in bag_stamps
                or received[robot] not in bag_stamps):
            errors.append(f"{robot}: terminal trajectory / acknowledged odometry is not bound to the closed bag")
            continue
        passed += 1
    return {"passed_robot_count": passed, **boundary}


def validate_initial_hold_barrier(
    mission_result: Mapping[str, Any], traces: Mapping[str, Sequence[Mapping[str, Any]]],
    authorizations: Sequence[Mapping[str, Any]], bag_events: Sequence[Mapping[str, Any]],
    errors: list[str],
) -> dict[str, Any]:
    """Bind the startup HOLD barrier to journaled leases and applied control samples."""
    run_id, scenario_id = mission_result.get("run_id"), mission_result.get("scenario_id")
    origin = _finite_number(mission_result.get("origin_sim_time_s"))
    barrier = mission_result.get("initial_hold_barrier")
    if (not isinstance(barrier, Mapping)
            or set(barrier) != {"expected_authorizations", "acknowledgments", "latched_sim_time_s"}):
        errors.append("initial HOLD barrier is missing or malformed")
        return {"passed_robot_count": 0, "origin_sim_time_s": origin}
    expected = barrier.get("expected_authorizations")
    acknowledgments = barrier.get("acknowledgments")
    latched = _finite_number(barrier.get("latched_sim_time_s"))
    if (not isinstance(expected, Mapping) or set(expected) != set(ROBOTS)
            or not isinstance(acknowledgments, Mapping) or set(acknowledgments) != set(ROBOTS)
            or latched is None or latched < 0):
        errors.append("initial HOLD barrier robot set or latch time is invalid")
        return {"passed_robot_count": 0, "origin_sim_time_s": origin}

    journal: dict[tuple[object, object], list[Mapping[str, Any]]] = defaultdict(list)
    for row in authorizations:
        journal[(row.get("robot_id"), row.get("authorization_seq"))].append(row)
    latest_sample = -math.inf
    passed = 0
    for robot in ROBOTS:
        payload, ack = expected[robot], acknowledgments[robot]
        if not isinstance(payload, Mapping) or not isinstance(ack, Mapping):
            errors.append(f"{robot}: initial HOLD payload or acknowledgment is malformed")
            continue
        scope = payload.get("scope")
        sequence = payload.get("authorization_seq")
        issued = _finite_number(payload.get("issued_sim_time_s"))
        until = _finite_number(payload.get("valid_until_sim_time_s"))
        evaluation = _finite_number(ack.get("evaluation_sim_time_s"))
        sample_time = _finite_number(ack.get("sample_sim_time_s"))
        frame = ack.get("physics_frame")
        command = ack.get("input_command")
        matching_frames = [row for row in traces.get(robot, ()) if row.get("physics_frame") == frame]
        sample = matching_frames[0] if len(matching_frames) == 1 else {}
        observed = sample.get("authorization", {}) if isinstance(sample, Mapping) else {}
        observed_command = sample.get("motion_profile", {}).get("input_command") if isinstance(sample, Mapping) else None
        lease_valid = False
        if issued is not None and until is not None and evaluation is not None and sample_time is not None:
            lease_valid = (0 <= issued <= evaluation < sample_time <= latched
                           and origin is not None and latched < origin < until
                           and 0 < until - issued <= .5 + 1e-9)
        identity_valid = (
            payload.get("schema_version") == 1 and payload.get("run_id") == run_id
            and payload.get("robot_id") == robot and payload.get("mode") == "hold"
            and type(sequence) is int and sequence > 0 and isinstance(scope, Mapping)
            and scope.get("scenario_id") == scenario_id
            and type(scope.get("reservation_id")) is int and scope["reservation_id"] > 0
            and ack.get("run_id") == run_id and ack.get("scenario_id") == scenario_id
            and ack.get("robot_id") == robot and ack.get("authorization_seq") == sequence
            and ack.get("scope") == scope and ack.get("mode") == "hold" and ack.get("valid") is True
            and type(frame) is int and frame > 0
        )
        command_valid = (
            isinstance(command, Mapping) and set(command) == {"linear_m_s", "angular_rad_s"}
            and all(_finite_number(value) == 0.0 for value in command.values())
            and observed_command == command
        )
        trace_valid = (
            len(matching_frames) == 1 and _finite_number(sample.get("sim_time_s")) == sample_time
            and isinstance(observed, Mapping) and observed.get("payload") == payload
            and observed.get("sequence") == sequence and observed.get("mode") == "hold"
            and observed.get("valid") is True and observed.get("reason") == "authorization_hold"
            and _finite_number(observed.get("evaluation_sim_time_s")) == evaluation
        )
        exact_journal = journal.get((robot, sequence), [])
        if (not identity_valid or not lease_valid or not command_valid or not trace_valid
                or len(exact_journal) != 1 or exact_journal[0] != payload):
            errors.append(f"{robot}: initial HOLD barrier is not bound to one fresh applied authorization")
            continue
        assert sample_time is not None
        latest_sample = max(latest_sample, sample_time)
        passed += 1

    ready = [row for row in bag_events if row.get("kind") == "navigation_ready"]
    if (len(ready) != 1 or ready[0].get("run_id") != run_id or ready[0].get("scenario_id") != scenario_id
            or _finite_number(ready[0].get("sim_time_s")) != origin
            or _finite_number(ready[0].get("origin_sim_time_s")) != origin
            or ready[0].get("initial_hold_barrier") != barrier):
        errors.append("navigation_ready does not exactly bind the initial HOLD barrier and origin")
    if origin is None or origin <= max(latched, latest_sample):
        errors.append("navigation origin is not strictly after the post-step HOLD barrier")
    return {"passed_robot_count": passed, "origin_sim_time_s": origin,
            "latched_sim_time_s": latched,
            "latest_ack_sample_sim_time_s": latest_sample if passed else None}


def validate_authorization_order(
    authorizations: Sequence[Mapping[str, Any]], route_events: Sequence[Mapping[str, Any]],
    traces: Mapping[str, Sequence[Mapping[str, Any]]], actions: Sequence[Mapping[str, Any]], errors: list[str],
    *, scenario_id: str | None = None,
) -> dict[str, Any]:
    """Bind drive permission to accepted UUID/generation/tile ownership evidence."""
    action_keys: dict[tuple[object, object, object], Mapping[str, Any]] = {}
    for row in actions:
        key = (row.get("robot_id"), row.get("generation"), row.get("follow_action_uuid"))
        if key in action_keys:
            errors.append("duplicate action reservation/goal binding")
        else:
            action_keys[key] = row
    accepted: dict[tuple[object, object, object], object] = {}
    route_by_sequence: dict[object, Mapping[str, Any]] = {}
    for row in route_events:
        sequence = row.get("event_sequence")
        if sequence in route_by_sequence:
            errors.append("duplicate route event sequence")
        else:
            route_by_sequence[sequence] = row
        if row.get("event_type") == "goal_accepted":
            key = (row.get("robot_id"), row.get("generation"), row.get("goal_uuid"))
            if key in accepted:
                errors.append("duplicate goal acceptance binding")
            else:
                accepted[key] = row.get("sim_time")
    navigate = 0
    for row in authorizations:
        if row.get("mode") != "navigate":
            continue
        scope = row.get("scope", {})
        key = (row.get("robot_id"), scope.get("reservation_id"), scope.get("goal_uuid"))
        action = action_keys.get(key)
        expected_hash = _hash_json(sorted(action["reservation_tiles"])) if action is not None else None
        issued = row.get("issued_sim_time_s")
        accepted_time = accepted.get(key)
        issued_number = _finite_number(issued)
        accepted_number = _finite_number(accepted_time)
        decision = route_by_sequence.get(scope.get("decision_event_seq"), {})
        decision_number = _finite_number(decision.get("sim_time"))
        journal_valid = False
        try:
            if "valid" in row:
                raise AuthorizationError("authorization journal contains a non-schema valid field")
            journal_state = LeaseGuard(str(row.get("run_id")), str(row.get("robot_id"))).evaluate(
                row, now_s=float(row["issued_sim_time_s"]),
            )
            journal_valid = journal_state.mode == "navigate" and journal_state.valid is True
        except (AuthorizationError, KeyError, TypeError, ValueError):
            journal_valid = False
        exact_decision = (decision.get("event_type") == "permission_queried"
                          and decision.get("robot_id") == key[0]
                          and decision.get("generation") == key[1]
                          and decision.get("goal_uuid") == key[2]
                          and decision.get("allowed") is True)
        ordered = (issued_number is not None and accepted_number is not None and decision_number is not None
                   and accepted_number <= decision_number <= issued_number)
        if (not journal_valid or action is None or not ordered or not exact_decision
                or scope.get("tile_hash") != expected_hash or scope.get("path_content_sha256") != action.get("path_content_sha256")
                or scope.get("route_id") != action.get("chunk", {}).get("route_id")
                or scope.get("leg_id") != action.get("leg_id")
                or scope.get("reservation_generation") != action.get("generation")
                or (scenario_id is not None and scope.get("scenario_id") != scenario_id)
                or scope.get("resources") != ["G3_FULL_LEG"] or scope.get("exit_id") != "G3_FULL_LEG:end"):
            errors.append("navigate authorization lacks accepted UUID/generation/tile permission binding")
            continue
        navigate += 1
    authorization_by_key: dict[tuple[object, object], Mapping[str, Any]] = {}
    for row in authorizations:
        key = (row.get("robot_id"), row.get("authorization_seq"))
        if key in authorization_by_key:
            errors.append("duplicate authorization robot/sequence binding")
        else:
            authorization_by_key[key] = row
    for robot, samples in traces.items():
        run_ids = {row.get("run_id") for row in authorizations
                   if row.get("robot_id") == robot and isinstance(row.get("run_id"), str) and row.get("run_id")}
        if len(run_ids) != 1:
            if samples:
                errors.append(f"{robot}: authorization journal does not provide one replay run identity")
            continue
        guard = LeaseGuard(str(next(iter(run_ids))), robot)
        for sample in samples:
            observed = sample.get("authorization", {})
            payload = observed.get("payload")
            if payload is not None and not isinstance(payload, Mapping):
                errors.append(f"{robot}: applied trajectory authorization payload is malformed")
                break
            if payload is not None and authorization_by_key.get((robot, observed.get("sequence"))) != payload:
                errors.append(f"{robot}: applied trajectory authorization differs from its journal payload")
                break
            try:
                if isinstance(payload, Mapping) and "valid" in payload:
                    raise AuthorizationError("authorization payload contains a non-schema valid field")
                state = guard.evaluate(payload, now_s=float(observed["evaluation_sim_time_s"]))
                if (observed.get("sequence") != state.sequence or observed.get("mode") != state.mode
                        or observed.get("valid") is not state.valid or observed.get("reason") != state.reason):
                    raise AuthorizationError("outer authorization state differs from LeaseGuard replay")
            except (AuthorizationError, KeyError, TypeError, ValueError) as exc:
                errors.append(f"{robot}: applied authorization evaluator replay failed: {exc}")
                break
            command = sample.get("motion_profile", {}).get("input_command", {})
            if any(float(value) != 0.0 for value in command.values()):
                authorization = sample.get("authorization", {})
                journal = authorization_by_key.get((robot, authorization.get("sequence")))
                if (payload is None or authorization.get("mode") != "navigate" or authorization.get("valid") is not True
                        or journal is None or journal.get("mode") != "navigate" or "valid" in journal):
                    errors.append(f"{robot}: nonzero drive input lacks navigate permission")
                    break
    for event in (row for row in route_events if row.get("event_type") == "parked_hold_shrunk"):
        predecessors = [row for row in actions if row.get("robot_id") == event.get("robot_id")
                        and row.get("generation") == event.get("predecessor_generation")]
        event_time = _finite_number(event.get("sim_time"))
        stopped_time = _finite_number(predecessors[0].get("stopped_sim_time_s")) if len(predecessors) == 1 else None
        if len(predecessors) != 1 or event_time is None or stopped_time is None or event_time < stopped_time:
            errors.append("parking hold was shrunk before its measured stop")
    return {"navigate_authorization_count": navigate, "route_event_count": len(route_events)}


def validate_scene_closure(
    output: Path, session: Mapping[str, Any], run_id: str, scenario_id: str, errors: list[str],
) -> dict[str, Any]:
    rows = _load_jsonl(output / "scene-events.jsonl", errors)
    closure = session.get("scene_closure")
    expected = [False] if closure is None else [True, False]
    states = [row.get("current_closed") for row in rows]
    if states != expected:
        errors.append("scene collider closure transitions differ from the authored schedule")
    for row in rows:
        if (row.get("run_id") != run_id or row.get("scenario_id") != scenario_id
                or row.get("collision_enabled_readback") is not row.get("current_closed")
                or "scene_active_interval_s" in row or "removal_time" in row):
            errors.append("scene closure event identity/readback/control boundary is invalid")
            break
    if closure is not None and len(rows) == 2:
        start = _load_json(output / "mission-start.json", errors)
        duration = float(rows[1]["sim_time_s"]) - float(start.get("sim_time_s", math.nan))
        authored = float(closure["scene_active_interval_s"][1] - closure["scene_active_interval_s"][0])
        if abs(duration - authored) > .1:
            errors.append("scene closure event timing differs from authored interval")
    return {"event_count": len(rows), "states": states}


def _validate_localization_reservations(
    bag: Mapping[str, Any], traces: Mapping[str, list[dict[str, Any]]], book: RouteBook,
    actions: Sequence[Mapping[str, Any]], route_events: Sequence[Mapping[str, Any]], errors: list[str],
    terminal_holds: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    by_generation: dict[tuple[object, object], set[tuple[int, int]]] = {}
    generation_owner: dict[int, str] = {}
    current_generation: dict[str, int] = {}
    prior_global = 0
    for row in route_events:
        generation = row.get("successor_generation")
        if generation is None:
            continue
        robot = row.get("robot_id")
        predecessor = row.get("predecessor_generation")
        tiles = row.get("tiles")
        key = (robot, generation)
        if (type(generation) is not int or generation <= prior_global or generation <= 0
                or robot not in ROBOTS or generation in generation_owner or key in by_generation
                or not isinstance(tiles, list)
                or predecessor != current_generation.get(str(robot))):
            errors.append("route reservation successor generation/owner/predecessor chain is invalid")
            continue
        try:
            parsed = {tuple(tile) for tile in tiles}
            if not parsed or any(len(tile) != 2 or any(type(value) is not int for value in tile) for tile in parsed):
                raise ValueError
        except (TypeError, ValueError):
            errors.append("route reservation tiles are malformed")
            continue
        by_generation[key] = parsed
        generation_owner[generation] = str(robot)
        current_generation[str(robot)] = generation
        prior_global = generation
    for row in actions:
        key = (row.get("robot_id"), row.get("generation"))
        action_tiles = {tuple(tile) for tile in row.get("reservation_tiles", [])}
        if key in by_generation and by_generation[key] != action_tiles:
            errors.append("action reservation tiles differ from the route generation journal")
        elif key not in by_generation:
            by_generation[key] = action_tiles
    if terminal_holds is not None:
        for robot in ROBOTS:
            hold = terminal_holds.get(robot, {})
            if current_generation.get(robot) != hold.get("generation"):
                errors.append(f"{robot}: terminal hold generation differs from the reservation event chain")
    bounds = (book.layout.bounds.xmin, book.layout.bounds.ymin, book.layout.bounds.xmax, book.layout.bounds.ymax)
    checked = 0
    startup_failsafe = 0
    ready_times = [float(row["sim_time_s"]) for row in bag.get("events", []) if row.get("kind") == "navigation_ready"]
    complete_times = [float(row["sim_time_s"]) for row in bag.get("events", []) if row.get("kind") == "scenario_complete"]
    ready_time = ready_times[0] if len(ready_times) == 1 else -math.inf
    complete_time = complete_times[0] if len(complete_times) == 1 else math.inf
    occupied_by_frame: dict[int, dict[str, frozenset[tuple[int, int]]]] = defaultdict(dict)
    for robot in ROBOTS:
        edges = bag.get("tf_samples", {}).get(robot, {})
        try:
            pose_index = index_decoded_tf(edges)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"{robot}: actual TF cannot build its trajectory pose index: {exc}")
            continue
        for sample in traces[robot]:
            stamp = float(sample["sim_time_s"])
            payload = sample.get("authorization", {}).get("payload") or {}
            scope = payload.get("scope", {}) if isinstance(payload, Mapping) else {}
            generation = scope.get("reservation_id")
            if type(generation) is not int:
                command = sample.get("motion_profile", {}).get("input_command", {})
                authorization = sample.get("authorization", {})
                zero = bool(command) and all(_finite_number(value) == 0.0 for value in command.values())
                if stamp < ready_time and zero and authorization.get("mode") != "navigate":
                    startup_failsafe += 1
                    continue
                if stamp > complete_time and zero and authorization.get("mode") != "navigate":
                    continue
                errors.append(f"{robot}: qualified trajectory sample lacks a reservation generation")
                continue
            held = by_generation.get((robot, generation))
            try:
                pose_xy = decoded_map_pose_indexed(pose_index, stamp)
            except (KeyError, TypeError, ValueError) as exc:
                errors.append(f"{robot}: actual TF cannot reconstruct the trajectory reservation pose: {exc}")
                continue
            occupied = swept_tiles([pose_xy], book.planning_radius_m, tile_size_m=book.tile_size_m, bounds_xy=bounds)
            if held is None or not occupied.issubset(held):
                errors.append(f"{robot}: localized planning circle leaves its held reservation")
            if type(sample.get("physics_frame")) is int:
                occupied_by_frame[sample["physics_frame"]][robot] = occupied
            checked += 1
    overlap_frames = 0
    for robots in occupied_by_frame.values():
        if set(robots) == set(ROBOTS) and robots[ROBOTS[0]].intersection(robots[ROBOTS[1]]):
            overlap_frames += 1
    if overlap_frames:
        errors.append("localized planning circles overlap between robots")
    return {"localized_reservation_checks": checked, "startup_failsafe_samples": startup_failsafe,
            "peer_overlap_frames": overlap_frames}


def validate_navigation_maps(
    output: Path, mission: Mapping[str, Any], book: RouteBook, errors: list[str],
) -> dict[str, Any]:
    """Bind runtime-readable map bytes and their provenance to the mission result."""
    from scripts.ros.g3_mission import verify_navigation_maps

    try:
        actual = verify_navigation_maps(output, book)
        provenance_path = output / "maps/provenance.json"
        provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
        recorded = mission.get("navigation_map_files")
        if recorded != actual or provenance.get("files") != actual:
            raise ValueError("navigation map hashes/provenance differ from source-derived bytes")
        pin = mission.get("inputs", {}).get("navigation_map_provenance", {})
        if pin.get("sha256") != sha(provenance_path):
            raise ValueError("navigation map provenance raw SHA differs from the mission input pin")
        return {"files": actual, "provenance_sha256": sha(provenance_path)}
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        errors.append(f"navigation map evidence: {exc}")
        return {}


def validate_g3_evidence(
    session_root: Path, output: Path, run_id: str, *, bag_observation: Mapping[str, Any] | None = None,
    result_name: str = RESULT,
) -> dict[str, Any]:
    errors: list[str] = []
    evaluation: dict[str, Any] = {}
    output = output.resolve()
    pair = _load_json(output / "pair-exit.json", errors)
    context = {key: pair.get(key) for key in ("profile", "policy", "condition", "seed")} if isinstance(pair, Mapping) else {}
    expected_session_sha = os.environ.get("FACTORY_OPS_G3_SESSION_SHA256")
    if not expected_session_sha:
        return {"schema_version": 1, "qualification": "g3_two_robot_orders", "run_id": run_id,
                "passed": False, "primary_evaluation": False, "scope": SCOPE,
                "errors": [*errors, "supervisor-bound session SHA environment is required"], "artifact_manifest": []}
    try:
        session = verify_session(session_root.resolve(strict=True), expected_context=context,
                                 expected_sha256=expected_session_sha)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {"schema_version": 1, "qualification": "g3_two_robot_orders", "run_id": run_id,
                "passed": False, "primary_evaluation": False, "scope": SCOPE,
                "errors": [*errors, f"session verification failed: {exc}"], "artifact_manifest": []}
    scenario_id = session["scenario_id"]
    ready = _load_json(output / "sim-ready.json", errors)
    scene = _load_json(output / "g3-scene.json", errors)
    mission = _load_json(output / "mission-result.json", errors)
    control = _load_json(output / "mission-control.json", errors)
    documents = ((ready, "ready"), (scene, "validated"), (mission, "passed"), (control, "passed"))
    for document, status in documents:
        if (not isinstance(document, Mapping) or document.get("run_id") != run_id
                or document.get("scenario_id") != scenario_id or document.get("status") != status):
            errors.append(f"worker document is not the matching {status} result")
    if (not re.fullmatch(r"[0-9a-f]{32}", run_id) or not isinstance(pair, Mapping)
            or pair.get("run_id") != run_id or pair.get("scenario_id") != scenario_id
            or pair.get("nav_exit_code") != 0 or pair.get("sim_exit_code") != 0
            or pair.get("mission-result.json") != mission or pair.get("g3-scene.json") != scene):
        errors.append("pair/run/session identity or worker exits differ")
    if scene.get("qualification") != "g3_two_robot_orders" or scene.get("cleanup") != "worker_exit_requested":
        errors.append("scene qualification/cleanup is invalid")
    if scene.get("mission_control") != control:
        errors.append("scene terminal control differs from the paired mission control")
    if scene.get("scene_extension", {}).get("ground_truth_control_input") is not False:
        errors.append("scene extension does not preserve the zero-ground-truth control boundary")
    if mission.get("primary_evaluation") is not False or mission.get("scope") != SCOPE or mission.get("ground_truth_input") is not False:
        errors.append("mission scope exceeds qualification/pilot evidence")
    if (mission.get("workload_sha256") != session.get("workload_sha256")
            or mission.get("session_sha256") != expected_session_sha
            or scene.get("inputs", {}).get("session", {}).get("sha256") != expected_session_sha
            or mission.get("inputs", {}).get("session", {}).get("sha256") != expected_session_sha):
        errors.append("scene/mission session SHA or workload binding differs")
    source_pins = (
        (scene, "source", PROJECT / "scripts/isaac/g2_scene.py"),
        (scene, "g3_scene_source", PROJECT / "scripts/isaac/g3_scene.py"),
        (mission, "mission_source", PROJECT / "scripts/ros/g3_mission.py"),
        (mission, "nav2_template", PROJECT / "configs/factory-v3/nav2-g2.yaml"),
    )
    for document, key, path in source_pins:
        if document.get("inputs", {}).get(key, {}).get("sha256") != sha(path):
            errors.append(f"runtime source pin differs: {path.relative_to(PROJECT).as_posix()}")
    for document, key, relative in (
        (scene, "scene", "scene/factory-v3.usda"), (scene, "layout", "layout.json"),
        (scene, "config", "config.json"), (scene, "scenarios", "scenarios.json"),
        (mission, "config", "config.json"),
    ):
        if document.get("inputs", {}).get(key, {}).get("sha256") != session["files"][relative]:
            errors.append(f"runtime session artifact pin differs: {relative}")
    traces: dict[str, list[dict[str, Any]]] = {}
    bag: dict[str, Any] = {}
    try:
        config = json.loads((session_root / "config.json").read_text(encoding="utf-8"))
        layout = Layout.from_file(session_root / "layout.json")
        book = RouteBook.from_files(PROJECT / "configs/factory-v3/layout.json", PROJECT / "configs/factory-v3/g3-routing.json")
        if layout.data != book.layout.data:
            errors.append("session layout semantics differ from the current route-book variant")
        for robot in ROBOTS:
            traces[robot], evaluation[robot] = _validate_robot(output, robot, ready, scene, layout, config, errors)
            samples = traces[robot]
            dt = float(config["physics_dt_s"])
            if not math.isclose(dt, 1 / 60, abs_tol=1e-12):
                errors.append("G3 physics rate is not 60 Hz")
            for previous, current in zip(samples, samples[1:]):
                if (current.get("physics_frame") != previous.get("physics_frame", -1) + 1
                        or abs(float(current["sim_time_s"]) - float(previous["sim_time_s"]) - dt) > 1e-4):
                    errors.append(f"{robot}: trajectory is not a complete 60 Hz frame stream")
                    break
            tilt_bad = 0
            for sample in samples:
                recorded = sample.get("ground_truth", {}).get("tilt_evaluation")
                derived = _tilt_evaluation(sample["ground_truth"]["orientation_wxyz"])
                if recorded != derived or derived["passed"] is not True:
                    tilt_bad += 1
            if tilt_bad:
                errors.append(f"{robot}: {tilt_bad} tilt evaluations fail independent recomputation")
        left, right = traces[ROBOTS[0]], traces[ROBOTS[1]]
        if len(left) != len(right) or any((a.get("physics_frame"), a.get("sim_time_s")) != (b.get("physics_frame"), b.get("sim_time_s")) for a, b in zip(left, right)):
            errors.append("robot full-rate physics frames do not match")
        radii = [ready["robots"][robot]["footprint"]["inflated_radius_m"] for robot in ROBOTS]
        distances = [math.dist(a["ground_truth"]["position_xyz_m"][:2], b["ground_truth"]["position_xyz_m"][:2]) for a, b in zip(left, right)]
        if not distances or min(distances) < sum(radii):
            errors.append("robot physical envelopes overlap")
        evaluation["separation"] = {"minimum_centre_distance_m": min(distances) if distances else None,
                                    "required_m": sum(radii)}
        for station in ("CI", "CO", "OUT"):
            if any(sum(math.dist(row["ground_truth"]["position_xyz_m"][:2], layout.stations[station]) <= .35 for row in pair_rows) > 1
                   for pair_rows in zip(left, right)):
                errors.append(f"physical dock overlap exceeds one robot at {station}")
        evaluation["media"] = _validate_media(output, scene, traces, errors)
        actions = mission.get("actions", [])
        if not isinstance(actions, list) or not actions:
            errors.append("mission action history is empty")
            actions = []
        sensor_bag = inspect_rosbag(output, traces, ready, event_prefix="/factory/g3")
        errors.extend(sensor_bag.get("errors", []))
        supplemental = _decode_g3_topics(output, run_id, scenario_id) if bag_observation is None else dict(bag_observation)
        bag = {**sensor_bag, **supplemental}
        errors.extend(supplemental.get("errors", []))
        local_events = _load_jsonl(output / "actions.jsonl", errors)
        local_auth = _load_jsonl(output / "authorizations.jsonl", errors)
        if supplemental.get("events") != local_events or supplemental.get("authorizations") != local_auth:
            errors.append("decoded G3 event/authorization streams differ from journals")
        if [row.get("event_seq") for row in local_events] != list(range(1, len(local_events) + 1)):
            errors.append("events sequence is missing, duplicated, or nonmonotonic")
        grouped_auth: dict[str, list[object]] = defaultdict(list)
        for row in local_auth:
            grouped_auth[str(row.get("robot_id"))].append(row.get("authorization_seq"))
        if any(values != list(range(1, len(values) + 1)) for values in grouped_auth.values()):
            errors.append("authorization sequence is missing, duplicated, or nonmonotonic")
        route_events = _load_jsonl(output / "route-events.jsonl", errors)
        if ([row.get("event_sequence") for row in route_events] != list(range(1, len(route_events) + 1))
                or any(row.get("run_id") != run_id for row in route_events)):
            errors.append("route reservation journal identity/sequence is invalid")
        evaluation["route_event_binding"] = validate_route_event_binding(
            route_events, supplemental.get("reservations"), mission, errors,
        )
        evaluation["actions"] = validate_actions(actions, book, bag, errors)
        evaluation["follow_rpc"] = validate_follow_rpc(actions, supplemental, errors)
        evaluation["stops"] = validate_stop_evidence(actions, traces, config["g3_runtime"], errors, bag)
        evaluation["initial_hold_barrier"] = validate_initial_hold_barrier(
            mission, traces, local_auth, local_events, errors,
        )
        evaluation["terminal_recording"] = validate_terminal_recording(mission, scene, traces, bag, errors)
        evaluation["authorization_order"] = validate_authorization_order(
            local_auth, route_events, traces, actions, errors, scenario_id=scenario_id,
        )
        evaluation["final"] = validate_final_state(mission, session, errors)
        evaluation["handoffs"] = validate_handoff_bindings(mission, actions, book, config["g3_runtime"], errors)
        evaluation["parking"] = validate_final_parking(mission, route_events, book, config["g3_runtime"], errors)
        evaluation["closure"] = validate_scene_closure(output, session, run_id, scenario_id, errors)
        evaluation["localization_reservations"] = _validate_localization_reservations(
            bag, traces, book, actions, route_events, errors,
            mission.get("route_snapshot", {}).get("holds", {}),
        )
        evaluation["navigation_maps"] = validate_navigation_maps(output, mission, book, errors)
    except (OSError, ValueError, TypeError, KeyError, IndexError, ImportError, sqlite3.Error) as exc:
        errors.append(f"G3 evidence inspection failed: {type(exc).__name__}: {exc}")
    safe = _safe_relative(result_name)
    if safe is None:
        errors.append("unsafe validation result name")
        safe = RESULT
    bounded_bag = {key: value for key, value in bag.items() if key not in {
        "events", "authorizations", "reservations", "amcl_poses", "tf_samples",
        "odometry", "plan_records", "follow_rpc",
    }}
    manifest = _manifest(output, {safe}, errors)
    return {"schema_version": 1, "qualification": "g3_two_robot_orders", "run_id": run_id,
            "scenario_id": scenario_id, "profile": session["profile"], "policy": session["policy"],
            "condition": session["condition"], "seed": session["seed"],
            "primary_evaluation": False, "scope": SCOPE, "passed": not errors, "status": "validated" if not errors else "failed",
            "errors": errors, "evaluation": evaluation, "bag": bounded_bag,
            "artifact_manifest_scope": "all stable post-worker files; this report excluded",
            "artifact_manifest": manifest,
            "claims": {"navigation_inputs": "decoded ROS CDR, estimated localization and authorization journals",
                       "ground_truth_use": "post-run clearance, docking, media and localization evaluation only",
                       "scope": "one qualification/pilot run; not primary comparison, repeatability, or production safety"}}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if not args.output.is_absolute():
        parser.error("--output must be absolute")
    output = args.output.resolve(strict=True)
    report = validate_g3_evidence(args.session, output, args.run_id)
    target = output / RESULT
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    temporary.replace(target)
    print("FACTORY_OPS_G3_VALIDATION=" + ("PASS" if report["passed"] else "FAIL"), flush=True)
    if report["errors"]:
        print(json.dumps(report["errors"], ensure_ascii=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
