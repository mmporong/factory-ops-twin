#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Run the real Jazzy action client, Nav2 lifecycle nodes and bounded ROS bag."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from typing import Any, Callable
from uuid import UUID, uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.layout import Layout
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult, SingleOrderMission


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


class LifecycleStatePoller:
    """Bound lifecycle GetState requests while retaining pending and ACTIVE state."""

    def __init__(self, names: list[str], *, minimum_interval_s: float = 0.5) -> None:
        if minimum_interval_s <= 0 or not math.isfinite(minimum_interval_s):
            raise ValueError("lifecycle request interval must be finite and positive")
        self.minimum_interval_s = minimum_interval_s
        self.states: dict[str, int | None] = {name: None for name in names}
        self._futures: dict[str, Any] = {}
        self._last_request_s: dict[str, float] = {name: -math.inf for name in names}

    def poll(
        self,
        wall_time_s: float,
        clients: dict[str, Any],
        request_factory: Callable[[], Any],
    ) -> dict[str, int | None]:
        if not math.isfinite(wall_time_s):
            raise ValueError("lifecycle polling wall time must be finite")
        if set(clients) != set(self.states):
            raise ValueError("lifecycle clients differ from configured nodes")
        for name, state_client in clients.items():
            future = self._futures.get(name)
            if future is not None:
                if not future.done():
                    continue
                response = future.result()
                self.states[name] = response.current_state.id if response is not None else None
                del self._futures[name]
            if self.states[name] == 3:
                continue
            if wall_time_s - self._last_request_s[name] < self.minimum_interval_s:
                continue
            if state_client.service_is_ready():
                self._futures[name] = state_client.call_async(request_factory())
                self._last_request_s[name] = wall_time_s
        return dict(self.states)


def prepare_params(
    project: Path, output: Path, ready: dict[str, Any], layout: Layout, config: dict[str, Any],
) -> Path:
    if not ready["footprint"]["passed"]:
        raise ValueError("scene collision footprint is not qualified")
    margin_value = config.get("localization_planning_margin_m")
    if (isinstance(margin_value, bool) or not isinstance(margin_value, (int, float))
            or not math.isfinite(float(margin_value)) or float(margin_value) <= 0):
        raise ValueError("localization_planning_margin_m must be finite and positive")
    footprint = ready["footprint"]
    collision_radius_m = float(footprint["collision_radius_m"])
    evaluation_clearance_m = float(footprint["clearance_m"])
    evaluation_radius_m = float(footprint["inflated_radius_m"])
    if (not all(math.isfinite(value) for value in (
            collision_radius_m, evaluation_clearance_m, evaluation_radius_m))
            or collision_radius_m <= 0 or evaluation_clearance_m != 0.2
            or not math.isclose(evaluation_radius_m, collision_radius_m + evaluation_clearance_m,
                                rel_tol=0.0, abs_tol=1e-9)):
        raise ValueError("scene footprint does not preserve the 0.2 m evaluation clearance contract")
    localization_margin_m = float(margin_value)
    planning_radius_m = evaluation_radius_m + localization_margin_m
    inflation_radius_m = planning_radius_m + 0.2
    import yaml

    maps = output / "maps"
    maps.mkdir()
    # Authored physical geometry + policy keepout. AMCL retains the physical-only map.
    occupied = bytearray(not free for free in layout._make_raw_mask())
    pgm = maps / "navigation.pgm"
    pgm.write_bytes(layout._pgm_bytes(occupied))
    (maps / "navigation.yaml").write_text(layout._yaml(pgm.name), encoding="utf-8")
    nav2_config = yaml.safe_load((project / "configs/factory-v3/nav2-g1.yaml").read_text(encoding="utf-8"))
    nav2_config["navigation_map_server"]["ros__parameters"]["yaml_filename"] = str(maps / "navigation.yaml")
    nav2_config["map_server"]["ros__parameters"]["yaml_filename"] = str(project / "runs/local/maps/map.yaml")
    for name in ("local_costmap", "global_costmap"):
        params = nav2_config[name][name]["ros__parameters"]
        params["robot_radius"] = planning_radius_m
        params["inflation_layer"]["inflation_radius"] = inflation_radius_m
    params_path = output / "nav2-params.yaml"
    params_path.write_text(yaml.safe_dump(nav2_config, sort_keys=False), encoding="utf-8")
    atomic_json(maps / "provenance.json", {
        "source_layout_sha256": layout.source_sha256,
        "navigation_map": "authored physical obstacles OR policy keepout; no online simulator truth",
        "amcl_map": "physical obstacles only",
        "collision_radius_m": collision_radius_m,
        "evaluation_clearance_m": evaluation_clearance_m,
        "evaluation_radius_m": evaluation_radius_m,
        "localization_planning_margin_m": localization_margin_m,
        "nav2_planning_radius_m": planning_radius_m,
        "inflation_radius_m": inflation_radius_m,
        "radius_basis": (
            "evaluation radius = measured collision radius + unchanged 0.2 m evaluator clearance; "
            "Nav2 planning radius adds the authored localization margin"
        ),
        "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in maps.iterdir() if p.is_file()},
    })
    return params_path


def stop_process(process: subprocess.Popen | None, label: str) -> dict[str, Any]:
    if process is None:
        return {"label": label, "started": False}
    if process.poll() is None:
        kill_group = getattr(os, "killpg")
        kill_group(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            kill_group(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                kill_group(process.pid, getattr(signal, "SIGKILL"))
                process.wait(timeout=10)
    return {"label": label, "pid": process.pid, "exit_code": process.returncode}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    output = args.output.resolve(strict=True)
    project = Path(__file__).resolve().parents[2]
    config = json.loads((project / "configs/factory-v3/g1.json").read_text(encoding="utf-8"))
    layout = Layout.from_file(project / "configs/factory-v3/layout.json")
    if layout.data["workcell"]["service_sim_s"] != 20:
        raise ValueError("single-order adapter service time differs from layout")
    deadline = time.monotonic() + config["max_wall_time_s"]
    launch = bag = None
    launch_log = bag_log = action_log = pose_log = None
    node = None
    rclpy = None
    mission = None
    result: dict[str, Any] = {"run_id": args.run_id, "status": "running", "scope": "g1 real ROS2/Nav2 single-order qualification"}
    cleanup: list[dict[str, Any]] = []
    try:
        ready_path = output / "sim-ready.json"
        ready_deadline = min(deadline, time.monotonic() + 300)
        while not ready_path.is_file():
            if time.monotonic() >= ready_deadline:
                raise TimeoutError("Isaac scene did not become ready")
            if (output / "g1-scene.json").is_file():
                scene_state = json.loads((output / "g1-scene.json").read_text())
                if scene_state.get("status") == "failed":
                    raise RuntimeError("Isaac scene failed before ready")
            time.sleep(0.2)
        ready = json.loads(ready_path.read_text())
        if ready.get("run_id") != args.run_id or ready.get("status") != "ready":
            raise ValueError("scene ready/run identity mismatch")
        params_path = prepare_params(project, output, ready, layout, config)
        os.environ["FACTORY_OPS_NAV2_PARAMS"] = str(params_path)
        import rclpy as ros
        from geometry_msgs.msg import Twist
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import Odometry
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        from unique_identifier_msgs.msg import UUID as GoalUUID

        rclpy = ros
        ros.init()
        node = ros.create_node("factory_g1_mission", parameter_overrides=[Parameter("use_sim_time", value=True)])
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        lifecycle_clients = {name: node.create_client(GetState, f"/{name}/get_state")
                             for name in ("planner_server", "controller_server", "bt_navigator")}
        lifecycle_poller = LifecycleStatePoller(list(lifecycle_clients))
        lifecycle_states = dict(lifecycle_poller.states)
        tf_buffer = Buffer(node=node)
        tf_listener = TransformListener(tf_buffer, node)
        control_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
        event_publisher = node.create_publisher(String, "/factory/mission_events", 10)
        counts: dict[str, int] = {"scan": 0, "odom": 0}
        latest_stamps: dict[str, float] = {}
        def count_message(name: str, msg: Any) -> None:
            counts[name] += 1
            latest_stamps[name] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        subscriptions = [
            node.create_subscription(LaserScan, "/scan", lambda msg: count_message("scan", msg), qos_profile_sensor_data),
            node.create_subscription(Odometry, "/odom", lambda msg: count_message("odom", msg), qos_profile_sensor_data),
        ]
        launch_log = (output / "nav2.log").open("w")
        bag_log = (output / "rosbag.log").open("w")
        action_log = (output / "actions.jsonl").open("w")
        pose_log = (output / "localization.jsonl").open("w")
        qos_path = output / "bag-qos.yaml"
        qos_path.write_text("/tf_static:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n/map:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n/navigation_map:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n")
        bag_topics = ["/clock", "/scan", "/odom", "/tf", "/tf_static", "/map", "/navigation_map", "/amcl_pose", "/plan", "/cmd_vel", "/factory/mission_events", "/navigate_to_pose/_action/feedback", "/navigate_to_pose/_action/status"]
        bag = subprocess.Popen(["ros2", "bag", "record", "--use-sim-time", "--include-hidden-topics", "--storage", "sqlite3", "--output", str(output / "rosbag"),
                                "--qos-profile-overrides-path", str(qos_path), "--topics", *bag_topics],
                               stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
        launch = subprocess.Popen(["ros2", "launch", str(project / "scripts/ros/g1.launch.py")],
                                  stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)

        def now() -> float:
            return node.get_clock().now().nanoseconds / 1e9

        def spin() -> None:
            if time.monotonic() >= deadline:
                raise TimeoutError("G1 mission wall timeout")
            if launch.poll() is not None:
                raise RuntimeError(f"Nav2 launch exited unexpectedly: {launch.returncode}")
            if bag.poll() is not None:
                raise RuntimeError(f"ROS recorder exited unexpectedly: {bag.returncode}")
            scene_path = output / "g1-scene.json"
            if scene_path.is_file() and json.loads(scene_path.read_text()).get("status") == "failed":
                raise RuntimeError("Isaac scene failed during the mission")
            ros.spin_once(node, timeout_sec=0.01)

        def localization() -> LocalizationPose:
            transform = tf_buffer.lookup_transform("map", "base_link", Time())
            translation = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            stamp_s = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            return LocalizationPose("map", translation.x, translation.y, yaw, stamp_s)

        def journal(kind: str, body: dict[str, Any]) -> None:
            item = {"run_id": args.run_id, "kind": kind, "sim_time": now(), **body}
            line = json.dumps(item, allow_nan=False)
            action_log.write(line + "\n")
            action_log.flush()
            msg = String()
            msg.data = line
            event_publisher.publish(msg)
            print("FACTORY_OPS_G1_ACTION=" + line, flush=True)

        startup_deadline = time.monotonic() + 240
        while True:
            spin()
            lifecycle_states = lifecycle_poller.poll(
                time.monotonic(), lifecycle_clients, GetState.Request,
            )
            try:
                pose = localization()
            except Exception:
                pose = None
            if (pose is not None and now() > 0 and counts["scan"] >= 5 and counts["odom"] >= 5
                    and client.server_is_ready() and all(state == 3 for state in lifecycle_states.values())
                    and 0 <= now() - pose.stamp <= 0.5):
                break
            if time.monotonic() > startup_deadline:
                raise TimeoutError("Nav2/TF/sensor graph did not become ready")
        journal("navigation_ready", {"lifecycle_states": lifecycle_states, "sensor_messages": dict(counts)})
        goals = {name: GoalPose("map", *layout.stations[name], yaw) for name, yaw in config["station_yaw_rad"].items()}
        mission = SingleOrderMission(args.run_id, robot_id=config["robot_id"], order_id=config["order_id"],
                                     payload_id=config["payload_id"], goal_timeout_s=240, station_goals=goals)
        mission.advance(now())
        mission.start()
        pending_goal = pending_result = goal_handle = None
        sent_goal_id = None
        last_pose_log_s = -1.0
        last_advance_s = -1.0
        last_progress_s = -1.0
        while not mission.completed:
            spin()
            sim_time = now()
            if sim_time > config["max_sim_time_s"]:
                raise TimeoutError("G1 mission simulation timeout")
            if sim_time - last_pose_log_s >= 0.1:
                pose = localization()
                pose_log.write(json.dumps(asdict(pose)) + "\n")
                pose_log.flush()
                last_pose_log_s = sim_time
            intent = mission.active_intent
            if intent is None:
                if sim_time - last_advance_s >= 0.05:
                    mission.advance(sim_time)
                    last_advance_s = sim_time
                if sim_time - last_progress_s >= 1.0:
                    atomic_json(output / "mission-progress.json", {"run_id": args.run_id, "sim_time": sim_time,
                                "phase": "logical process wait", "snapshot": mission.ledger.snapshot()})
                    last_progress_s = sim_time
                continue
            if intent.goal_id != sent_goal_id:
                msg = NavigateToPose.Goal()
                msg.pose.header.frame_id = "map"
                msg.pose.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = intent.goal.x, intent.goal.y
                msg.pose.pose.orientation.z = math.sin(intent.goal.yaw / 2)
                msg.pose.pose.orientation.w = math.cos(intent.goal.yaw / 2)
                goal_uuid = GoalUUID(uuid=list(UUID(intent.goal_id).bytes))
                pending_goal = client.send_goal_async(msg, goal_uuid=goal_uuid)
                pending_result = goal_handle = None
                sent_goal_id = intent.goal_id
                journal("goal_sent", {"intent": intent.to_dict()})
                atomic_json(output / "mission-progress.json", {"run_id": args.run_id, "sim_time": sim_time,
                            "phase": "navigation", "intent": intent.to_dict(), "snapshot": mission.ledger.snapshot()})
            if pending_goal is not None and pending_goal.done() and goal_handle is None:
                goal_handle = pending_goal.result()
                if goal_handle is None or not goal_handle.accepted:
                    raise RuntimeError("Nav2 rejected the active goal")
                accepted_uuid = bytes(goal_handle.goal_id.uuid).hex()
                if accepted_uuid != UUID(intent.goal_id).hex:
                    raise RuntimeError("Nav2 accepted a different goal UUID")
                journal("goal_accepted", {"goal_id": intent.goal_id, "server_goal_uuid": accepted_uuid})
                pending_result = goal_handle.get_result_async()
            if pending_result is not None and pending_result.done():
                terminal = pending_result.result()
                pose = localization()
                evidence = NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station,
                                            intent.attempt, int(terminal.status), now(), pose)
                journal("action_result", {"result": evidence.to_dict(), "nav2_error_code": int(terminal.result.error_code),
                                          "nav2_error_msg": str(terminal.result.error_msg)})
                if int(terminal.status) != 4:
                    # Preserve the real terminal result in the adapter's fail-stop journal.
                    mission.submit_result(evidence)
                if int(terminal.result.error_code) != 0:
                    mission.fail("Nav2 success status has a nonzero error code", sim_time=evidence.received_sim_time)
                    raise RuntimeError("Nav2 returned a nonzero result error code")
                receipt = mission.submit_result(evidence)
                journal("handoff_commit", {"receipt": receipt.to_dict(), "snapshot": mission.ledger.snapshot()})
                pending_goal = pending_result = goal_handle = None
            elif sim_time - last_advance_s >= 0.05:
                mission.advance(sim_time)
                last_advance_s = sim_time
        control_publisher.publish(Twist())
        result.update({"status": "passed", "mission": mission.export(), "sensor_messages": counts,
                       "latest_sensor_stamps": latest_stamps, "bag_topics_requested": bag_topics,
                       "localization_source": "TF map->base_link: wheel odometry + RTX LaserScan AMCL",
                       "ground_truth_input": False})
        atomic_json(output / "mission-control.json", {"run_id": args.run_id, "status": "passed"})
        print("FACTORY_OPS_G1_MISSION=PASS", flush=True)
        del tf_listener, subscriptions
    except BaseException as error:
        result.update({"status": "failed", "error": {"type": type(error).__name__, "message": str(error),
                      "traceback": traceback.format_exc(limit=20)}})
        if mission is not None:
            if not mission.completed and not mission.failed:
                mission.fail(f"external ROS runtime failure: {type(error).__name__}: {error}", sim_time=mission.sim_time)
            result["mission"] = mission.export()
        atomic_json(output / "mission-control.json", {"run_id": args.run_id, "status": "failed", "reason": str(error)})
        print(f"FACTORY_OPS_G1_MISSION=FAIL {type(error).__name__}: {error}", flush=True)
    finally:
        for process, label in ((bag, "rosbag"), (launch, "nav2")):
            try:
                cleanup.append(stop_process(process, label))
            except BaseException as error:
                cleanup.append({"label": label, "error": f"{type(error).__name__}: {error}"})
                result["status"] = "failed"
        for resource, method in ((node, "destroy_node"), (rclpy, "shutdown")):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except BaseException as error:
                    cleanup.append({"label": method, "error": f"{type(error).__name__}: {error}"})
                    result["status"] = "failed"
        for stream in (launch_log, bag_log, action_log, pose_log):
            if stream is not None:
                stream.close()
        result["cleanup"] = cleanup
        atomic_json(output / "mission-result.json", result)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
