#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Exercise actual Jazzy action RPC introspection with an isolated synthetic server."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.ros.action_services import IntrospectedActionClient
from scripts.ros.g3_mission import path_payload
from factory_ops.fleet_execution import path_content_sha256


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("SDK smoke report must use a new path")
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

        def wait(predicate: Any, label: str) -> None:
            deadline = time.monotonic() + 20
            while not predicate():
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                for node in nodes:
                    rclpy.spin_once(node, timeout_sec=.01)

        wait(lambda: client.server_is_ready() and all(client_node.count_subscribers(topic) >= 1
             for topic in client.service_event_topics), "SDK services or event subscribers unavailable")
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
        report.update({"status": "passed", "goal_uuid": identity, "path": expected,
                       "service_transactions": summaries, "cancel_return_code": int(cancel.result().return_code)})
    except Exception as error:
        report.update({"status": "failed", "error": f"{type(error).__name__}: {error}"})
    finally:
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
