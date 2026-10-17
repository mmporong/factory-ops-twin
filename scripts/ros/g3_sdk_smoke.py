#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Exercise actual Jazzy action RPC introspection with an isolated synthetic server."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import time
from typing import Any, Mapping
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.ros.action_services import IntrospectedActionClient
from scripts.ros.g2_mission import named_subscriber_topics_ready, stop_process
from scripts.ros.g3_mission import path_payload
from factory_ops.fleet_execution import path_content_sha256


def _canonical_message(message: Any) -> dict[str, Any]:
    from rosidl_runtime_py.convert import message_to_ordereddict

    value = message_to_ordereddict(message)
    return json.loads(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False))


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _decode_recorded_events(
    bag_dir: Path,
    topic_types: Mapping[str, str],
    live_events: dict[str, list[dict[str, Any]]],
    identity: str,
    expected_path_content_sha256: str,
) -> dict[str, Any]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    databases = sorted(bag_dir.glob("*.db3"))
    if not databases:
        raise RuntimeError("rosbag did not produce a SQLite database")
    topic_rows: dict[str, list[tuple[int, bytes, str]]] = {topic: [] for topic in topic_types}
    integrity: list[str] = []
    database_records: list[dict[str, Any]] = []
    for database in databases:
        with sqlite3.connect(f"file:{database}?mode=ro", uri=True) as connection:
            checks = [str(row[0]) for row in connection.execute("PRAGMA integrity_check")]
            integrity.extend(checks)
            topics = {
                int(row[0]): (str(row[1]), str(row[2]))
                for row in connection.execute("SELECT id,name,type FROM topics")
            }
            for timestamp, topic_id, raw in connection.execute(
                "SELECT timestamp,topic_id,data FROM messages ORDER BY timestamp,id"
            ):
                topic = topics[int(topic_id)][0]
                if topic in topic_rows:
                    topic_rows[topic].append((int(timestamp), bytes(raw), topics[int(topic_id)][1]))
        database_records.append({
            "path": database.name,
            "bytes": database.stat().st_size,
            "sha256": _sha(database),
            "integrity_check": checks,
        })
    if integrity != ["ok"] * len(databases):
        raise RuntimeError("rosbag SQLite integrity check failed")

    operations = dict(zip(("send_goal", "get_result", "cancel_goal"), topic_types))
    recorded: dict[str, list[dict[str, Any]]] = {}
    cdr_sha256: dict[str, list[str]] = {}
    topic_counts: dict[str, int] = {}
    summaries: dict[str, dict[str, Any]] = {}
    for operation, topic in operations.items():
        rows = sorted(topic_rows[topic], key=lambda row: row[0])
        topic_counts[topic] = len(rows)
        if len(rows) != 2:
            raise RuntimeError(f"{operation}: rosbag must contain exactly one request and one response")
        expected_type = topic_types[topic]
        if any(type_name != expected_type for _, _, type_name in rows):
            raise RuntimeError(f"{operation}: recorded service-event type mismatch")
        message_type = get_message(expected_type)
        decoded = [deserialize_message(raw, message_type) for _, raw, _ in rows]
        canonical = [_canonical_message(message) for message in decoded]
        recorded[operation] = canonical
        cdr_sha256[operation] = [hashlib.sha256(raw).hexdigest() for _, raw, _ in rows]
        if canonical != live_events[operation]:
            raise RuntimeError(f"{operation}: recorded CDR differs from the live introspection events")

        requests = [message for message in decoded if int(message.info.event_type) == 0]
        responses = [message for message in decoded if int(message.info.event_type) == 3]
        if (len(requests) != 1 or len(responses) != 1
                or len(requests[0].request) != 1 or len(responses[0].response) != 1):
            raise RuntimeError(f"{operation}: recorded request/response cardinality mismatch")
        request, response = requests[0], responses[0]
        request_transaction = (bytes(request.info.client_gid), int(request.info.sequence_number))
        response_transaction = (bytes(response.info.client_gid), int(response.info.sequence_number))
        if request_transaction != response_transaction:
            raise RuntimeError(f"{operation}: recorded transaction mismatch")
        body = request.request[0]
        request_uuid = body.goal_info.goal_id if operation == "cancel_goal" else body.goal_id
        if bytes(request_uuid.uuid).hex() != identity:
            raise RuntimeError(f"{operation}: recorded request UUID mismatch")
        if operation == "send_goal":
            content_sha256 = path_content_sha256(path_payload(body.goal.path))
            if content_sha256 != expected_path_content_sha256:
                raise RuntimeError("send_goal: recorded Path content mismatch")
        summaries[operation] = {
            "request_count": 1,
            "response_count": 1,
            "client_gid": request_transaction[0].hex(),
            "sequence_number": request_transaction[1],
        }
    return {
        "decoded_cdr": True,
        "integrity_check": "ok",
        "databases": database_records,
        "topic_counts": topic_counts,
        "cdr_sha256": cdr_sha256,
        "service_transactions": summaries,
        "events": recorded,
    }


def stop_and_decode_recorder(process: Any, cleanup: list[dict[str, Any]], bag_dir: Path,
                            topic_types: Mapping[str, str], live_events: dict[str, list[dict[str, Any]]],
                            identity: str, expected_path_content_sha256: str) -> dict[str, Any]:
    stopped = stop_process(process, "rosbag2_recorder")
    cleanup.append(stopped)
    if type(stopped.get("exit_code")) is not int or stopped["exit_code"] != 0:
        raise RuntimeError(f"rosbag recorder did not exit successfully: {stopped}")
    return _decode_recorded_events(bag_dir, topic_types, live_events, identity, expected_path_content_sha256)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    bag_dir = args.output.with_suffix(".rosbag")
    bag_log = args.output.with_suffix(".rosbag.log")
    if args.output.exists() or bag_dir.exists() or bag_log.exists():
        raise FileExistsError("SDK smoke report and rosbag must use new paths")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    import rclpy
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowPath
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.qos import qos_profile_services_default
    from rclpy.serialization import serialize_message
    from rclpy.service_introspection import ServiceIntrospectionState
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from unique_identifier_msgs.msg import UUID

    report: dict[str, Any] = {"status": "running", "scope": "synthetic_action_server_no_physics",
        "primary_evaluation": False, "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    nodes: list[Any] = []
    action_server: Any = None
    bag: subprocess.Popen[bytes] | None = None
    bag_log_stream: Any = None
    cleanup: list[dict[str, Any]] = []
    rclpy.init()
    try:
        server = rclpy.create_node("server", namespace="/factory_g3_sdk_smoke")
        client_node = rclpy.create_node("client", namespace="/factory_g3_sdk_smoke")
        nodes.extend((server, client_node))
        received: list[dict[str, Any]] = []
        received_paths: list[Any] = []
        received_serializations: list[str] = []
        def hash_path(path: Any) -> str:
            return path_content_sha256(path_payload(path))
        def accepted(goal: Any) -> Any:
            received.append({"path_content_sha256": hash_path(goal.path), "controller_id": goal.controller_id,
                "goal_checker_id": goal.goal_checker_id, "progress_checker_id": goal.progress_checker_id})
            received_paths.append(message_to_ordereddict(goal.path))
            received_serializations.append(hashlib.sha256(bytes(serialize_message(goal.path))).hexdigest())
            return GoalResponse.ACCEPT
        def execute(handle: Any) -> Any:
            handle.succeed()
            return FollowPath.Result()
        action_server = ActionServer(server, FollowPath, "follow_path", execute,
                                     goal_callback=accepted)
        client = IntrospectedActionClient(client_node, FollowPath, "follow_path", cancel_type=CancelGoal,
            event_qos=qos_profile_services_default, introspection_state=ServiceIntrospectionState.CONTENTS)
        events: dict[str, list[Any]] = {name: [] for name in ("send_goal", "get_result", "cancel_goal")}
        subscriptions = []
        for operation, topic, generated in zip(events, client.service_event_topics,
                ("SendGoal", "GetResult", "CancelGoal")):
            type_name = f"nav2_msgs/action/FollowPath_{generated}_Event" if operation != "cancel_goal" else "action_msgs/srv/CancelGoal_Event"
            subscriptions.append(client_node.create_subscription(get_message(type_name), topic,
                lambda message, name=operation: events[name].append(message), qos_profile_services_default))

        topic_types = dict(zip(client.service_event_topics, (
            "nav2_msgs/action/FollowPath_SendGoal_Event",
            "nav2_msgs/action/FollowPath_GetResult_Event",
            "action_msgs/srv/CancelGoal_Event",
        )))
        service_names = [topic.removesuffix("/_service_event") for topic in client.service_event_topics]
        bag_log_stream = bag_log.open("x", encoding="utf-8")
        bag = subprocess.Popen([
            "ros2", "bag", "record", "--include-hidden-topics", "--storage", "sqlite3",
            "--output", str(bag_dir), "--topics", *client.service_event_topics,
            "--services", *service_names,
        ], stdout=bag_log_stream, stderr=subprocess.STDOUT, start_new_session=True)
        report["rosbag_command"] = {
            "include_hidden_topics": True,
            "topics": list(client.service_event_topics),
            "services": service_names,
            "storage": "sqlite3",
            "output": bag_dir.name,
            "log": bag_log.name,
        }

        def wait(predicate: Any, label: str) -> None:
            deadline = time.monotonic() + 20
            while not predicate():
                if bag is not None and bag.poll() is not None:
                    raise RuntimeError(f"rosbag recorder exited before {label}: {bag.returncode}")
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                for node in nodes:
                    rclpy.spin_once(node, timeout_sec=.01)

        wait(lambda: client.server_is_ready() and all(client_node.count_subscribers(topic) >= 1
             for topic in client.service_event_topics)
             and named_subscriber_topics_ready(client_node, client.service_event_topics, "rosbag2_recorder"),
             "SDK services, local event subscribers or rosbag recorder subscriptions unavailable")
        goal = FollowPath.Goal()
        goal.path.header.frame_id = "map"
        goal.path.header.stamp = client_node.get_clock().now().to_msg()
        for x, y in ((1., 2.), (3., 2.)):
            pose = PoseStamped(); pose.header = goal.path.header
            pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.orientation.w = 1.
            goal.path.poses.append(pose)
        goal.controller_id = "FollowPath"; goal.goal_checker_id = "goal_checker"; goal.progress_checker_id = "progress_checker"
        identity = uuid4().hex
        sent = client.send_goal_async(goal, goal_uuid=UUID(uuid=list(bytes.fromhex(identity))))
        wait(sent.done, "SendGoal did not complete")
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError("synthetic SDK server rejected goal")
        result = handle.get_result_async()
        wait(result.done, "GetResult did not complete")
        if result.result().status != 4 or result.result().result.error_code != 0:
            raise RuntimeError("synthetic SDK result differs")
        # A terminal action has no motion; this also tests the real CancelGoal RPC schema.
        cancel = handle.cancel_goal_async()
        wait(cancel.done, "CancelGoal did not complete")
        wait(lambda: all({int(value.info.event_type) for value in rows} >= {0, 3}
                         for rows in events.values()), "client request/response bodies missing")
        summaries = {}
        live_events: dict[str, list[dict[str, Any]]] = {}
        for operation, rows in events.items():
            requests = [row for row in rows if int(row.info.event_type) == 0]
            responses = [row for row in rows if int(row.info.event_type) == 3]
            if len(requests) != 1 or len(responses) != 1 or len(requests[0].request) != 1 or len(responses[0].response) != 1:
                raise RuntimeError(f"{operation}: unexpected event cardinality")
            request, response = requests[0], responses[0]
            if (bytes(request.info.client_gid), request.info.sequence_number) != (bytes(response.info.client_gid), response.info.sequence_number):
                raise RuntimeError(f"{operation}: transaction mismatch")
            request_uuid = request.request[0].goal_info.goal_id if operation == "cancel_goal" else request.request[0].goal_id
            if bytes(request_uuid.uuid).hex() != identity:
                raise RuntimeError(f"{operation}: actual request UUID mismatch")
            summaries[operation] = {"request_count": 1, "response_count": 1,
                "client_gid": bytes(request.info.client_gid).hex(), "sequence_number": int(request.info.sequence_number)}
            live_events[operation] = [_canonical_message(row) for row in rows]
        actual = next(row.request[0].goal for row in events["send_goal"] if int(row.info.event_type) == 0)
        expected = {"path_content_sha256": hash_path(goal.path), "controller_id": "FollowPath",
                    "goal_checker_id": "goal_checker", "progress_checker_id": "progress_checker"}
        report["path_diagnostics"] = {"expected": expected, "server_received": received,
            "request_event_content_sha256": hash_path(actual.path), "expected_payload": message_to_ordereddict(goal.path),
            "expected_serialization_sha256": hashlib.sha256(bytes(serialize_message(goal.path))).hexdigest(),
            "request_event_serialization_sha256": hashlib.sha256(bytes(serialize_message(actual.path))).hexdigest(),
            "server_serialization_sha256s": received_serializations,
            "server_payloads": received_paths, "request_event_payload": message_to_ordereddict(actual.path)}
        if (received != [expected] or hash_path(actual.path) != expected["path_content_sha256"]
                or actual.controller_id != expected["controller_id"] or actual.goal_checker_id != expected["goal_checker_id"]
                or actual.progress_checker_id != expected["progress_checker_id"]):
            raise RuntimeError("server, real request event and planned Path differ")
        report["rosbag"] = stop_and_decode_recorder(
            bag, cleanup, bag_dir, topic_types, live_events, identity, expected["path_content_sha256"]
        )
        bag = None
        if report["rosbag"]["service_transactions"] != summaries:
            raise RuntimeError("recorded and live service transaction summaries differ")
        report.update({"status": "passed", "goal_uuid": identity, "path": expected,
                       "service_transactions": summaries, "cancel_return_code": int(cancel.result().return_code)})
    except Exception as error:
        report.update({"status": "failed", "error": f"{type(error).__name__}: {error}"})
    finally:
        if bag is not None:
            cleanup.append(stop_process(bag, "rosbag2_recorder"))
        if bag_log_stream is not None:
            bag_log_stream.close()
        report["process_cleanup"] = cleanup
        if action_server is not None:
            action_server.destroy()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
