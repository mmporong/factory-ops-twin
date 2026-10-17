#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""G3 qualification: actual planner paths, lease-gated edge actions and orders."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
from typing import Any, Callable, Mapping
from uuid import UUID, uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_execution import StoppedGate, make_chunk, qualify_computed_path, path_content_sha256, path_clock_ready, path_header_stamp_ns
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.g3_session import save, sha, verify_session
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from factory_ops.operations import LedgerConfig, Transition
from factory_ops.route_reservation import RouteReservationManager
from scripts.ros.g1_mission import LifecycleStatePoller
from scripts.ros.action_services import IntrospectedActionClient
from scripts.ros.g2_mission import (
    ROBOT_IDS, _load_ready, _planning_radius, atomic_json, build_namespaced_params,
    evidence_subscribers_ready, filtered_scan_is_recordable, make_authorization,
    mask_scan_ranges, named_subscriber_topics_ready, publish_scan_fanout,
    scan_recording_topics, stop_process,
)


def angle_error(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def failure_reason(error: BaseException) -> str:
    """Keep signal-driven failures usable by the mission fault-stop contract."""
    return str(error).strip() or type(error).__name__


def path_payload(path: Any) -> dict[str, Any]:
    """Preserve the complete Path value independently of the CDR byte hash."""
    def header(value: Any) -> dict[str, Any]:
        return {"frame_id": str(value.frame_id), "sec": int(value.stamp.sec), "nanosec": int(value.stamp.nanosec)}
    poses = []
    for item in path.poses:
        p, q = item.pose.position, item.pose.orientation
        xyz, xyzw = [float(p.x), float(p.y), float(p.z)], [float(q.x), float(q.y), float(q.z), float(q.w)]
        if item.header.frame_id != "map" or not all(math.isfinite(v) for v in xyz + xyzw) or abs(sum(v*v for v in xyzw)-1) > 1e-3:
            raise ValueError("computed Path contains an invalid frame/pose/quaternion")
        poses.append({"header": header(item.header), "position_xyz_m": xyz, "orientation_xyzw": xyzw})
    return {"header": header(path.header), "poses": poses}


def path_final_yaw(payload: Mapping[str, Any]) -> float:
    x, y, z, w = payload["poses"][-1]["orientation_xyzw"]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def valid_hold_ack(ack: Mapping[str, Any], *, run_id: str, scenario_id: str, robot_id: str,
                   minimum_sequence: int, generation: int, goal_uuid: str | None,
                   terminal_time_s: float, now_s: float) -> bool:
    """Admit a post-terminal applied hold, separate from measured wheel stop."""
    scope = ack.get("scope")
    stamp = ack.get("evaluation_sim_time_s")
    command = ack.get("input_command")
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        return False
    return bool(ack.get("run_id") == run_id and ack.get("scenario_id") == scenario_id
        and ack.get("robot_id") == robot_id and ack.get("mode") == "hold" and ack.get("valid") is True
        and type(ack.get("authorization_seq")) is int and ack["authorization_seq"] >= minimum_sequence
        and isinstance(scope, Mapping) and type(scope.get("reservation_id")) is int
        and scope.get("reservation_id") == generation and scope.get("goal_uuid") == goal_uuid
        and type(ack.get("physics_frame")) is int and ack["physics_frame"] > 0
        and math.isfinite(stamp) and terminal_time_s <= stamp <= now_s
        and isinstance(command, Mapping) and set(command) == {"linear_m_s", "angular_rad_s"}
        and all(type(v) in (int, float) and v == 0 for v in command.values()))


def valid_initial_hold_ack(ack: Mapping[str, Any], expected: Mapping[str, Any], *, now_s: float) -> bool:
    scope = expected.get("scope")
    issued, until, sample = (expected.get("issued_sim_time_s"), expected.get("valid_until_sim_time_s"),
                            ack.get("sample_sim_time_s"))
    if (expected.get("mode") != "hold" or not isinstance(scope, Mapping)
            or type(scope.get("reservation_id")) is not int or scope["reservation_id"] <= 0
            or type(expected.get("authorization_seq")) is not int or expected["authorization_seq"] <= 0):
        return False
    numeric = []
    for value in (issued, until, sample, now_s):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            return False
        numeric.append(float(value))
    issued_time, until_time, sample_time, current_time = numeric
    if not valid_hold_ack(ack, run_id=expected["run_id"], scenario_id=scope["scenario_id"],
            robot_id=expected["robot_id"], minimum_sequence=expected["authorization_seq"],
            generation=scope["reservation_id"], goal_uuid=scope.get("goal_uuid"),
            terminal_time_s=issued_time, now_s=current_time):
        return False
    return bool(ack["authorization_seq"] == expected["authorization_seq"] and ack.get("scope") == scope
                and issued_time <= ack["evaluation_sim_time_s"] < sample_time <= current_time < until_time
                and 0 < until_time-issued_time <= .5+1e-9)


def initial_hold_boundary_ready(acks: Mapping[str, Mapping[str, Any]], expected: Mapping[str, Mapping[str, Any]],
                                *, now_s: float) -> bool:
    return bool(set(acks) == set(expected) == set(ROBOT_IDS)
                and all(valid_initial_hold_ack(acks[robot], expected[robot], now_s=now_s)
                        and now_s > acks[robot]["sample_sim_time_s"] for robot in ROBOT_IDS))


def terminal_recording_boundary(scene: Mapping[str, Any] | None, odometry_ns: Mapping[str, Any], *,
                                run_id: str, scenario_id: str, minimum_sim_time_ns: int) -> dict[str, Any] | None:
    """Keep recording until the scene seals a final ROS-aligned physics frame."""
    if scene is None:
        return None
    if scene.get("run_id") != run_id or scene.get("scenario_id") != scenario_id:
        raise ValueError("terminal scene identity differs")
    status = scene.get("status")
    if status == "running":
        return None
    if status != "validated" or scene.get("cleanup") != "worker_exit_requested":
        raise ValueError("terminal scene did not validate its cleanup")
    timing = scene.get("timing")
    if not isinstance(timing, Mapping) or not isinstance(timing.get("final_stamp"), Mapping):
        raise ValueError("terminal scene has no canonical final stamp")
    final_ns = path_header_stamp_ns(timing["final_stamp"])
    frame = timing.get("final_physics_frame")
    final_seconds = timing.get("final_sim_time_s")
    if (type(minimum_sim_time_ns) is not int or minimum_sim_time_ns < 0
            or final_ns < minimum_sim_time_ns or type(frame) is not int or frame <= 0
            or isinstance(final_seconds, bool) or not isinstance(final_seconds, (int, float)) or not math.isfinite(final_seconds)
            or abs(final_seconds-final_ns/1e9) > 1e-9):
        raise ValueError("terminal scene final frame/time is missing, invalid or backward")
    if any(type(odometry_ns.get(robot)) is not int or odometry_ns[robot] < final_ns for robot in ROBOT_IDS):
        return None
    return {"final_stamp": dict(timing["final_stamp"]), "final_sim_time_ns": final_ns,
            "final_physics_frame": frame, "minimum_sim_time_ns": minimum_sim_time_ns,
            "received_odometry_ns": {robot: odometry_ns[robot] for robot in ROBOT_IDS}}


def wait_for_terminal_recording(*, tick: Callable[[], Any], read_scene: Callable[[], Mapping[str, Any] | None],
                                read_odometry_ns: Callable[[], Mapping[str, Any]], run_id: str, scenario_id: str,
                                minimum_sim_time_ns: int, wait_wall_s: float,
                                now_wall: Callable[[], float] = time.monotonic) -> dict[str, Any]:
    if type(wait_wall_s) not in (int, float) or not math.isfinite(wait_wall_s) or wait_wall_s <= 0:
        raise ValueError("terminal recording wait must have a finite positive wall bound")
    deadline = now_wall()+wait_wall_s
    while True:
        tick()
        boundary = terminal_recording_boundary(read_scene(), read_odometry_ns(), run_id=run_id,
            scenario_id=scenario_id, minimum_sim_time_ns=minimum_sim_time_ns)
        if boundary is not None:
            return boundary
        if now_wall() >= deadline:
            raise TimeoutError("terminal scene / final odometry recording boundary did not qualify")


class PoseNotFreshError(ValueError):
    """A valid transform exists but cannot provide current pose evidence."""


def localization_payload(transform: Any, *, now_s: float, ttl_s: float) -> dict[str, float]:
    p, q = transform.transform.translation, transform.transform.rotation
    values = [float(p.x), float(p.y), float(p.z), float(q.x), float(q.y), float(q.z), float(q.w)]
    if (transform.header.frame_id != "map" or transform.child_frame_id != "base_link"
            or not all(math.isfinite(value) for value in values) or abs(sum(v*v for v in values[3:])-1) > 1e-3):
        raise ValueError("map->base_link TF has an invalid frame, position or quaternion")
    stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
    if stamp < 0 or not 0 <= now_s-stamp <= ttl_s:
        raise PoseNotFreshError("map->base_link TF is stale or future")
    return {"x_m": values[0], "y_m": values[1], "yaw_rad": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), "stamp_s": stamp}


def validate_arrival(pose: Mapping[str, float], target: GoalPose, *, xy_limit: float, yaw_limit: float) -> None:
    if (not 0 < xy_limit <= .25 or not 0 < yaw_limit <= .25
            or math.dist((pose["x_m"], pose["y_m"]), (target.x, target.y)) > xy_limit
            or angle_error(pose["yaw_rad"], target.yaw) > yaw_limit):
        raise ValueError("FollowPath success failed configured estimated arrival pose/yaw limits")


def stopped_odometry_record(gate: StoppedGate, *, now_s: float) -> dict[str, Any]:
    """Seal the actual continuous stationary window and its measurement boundary."""
    samples = gate.samples
    if not gate.is_stopped(now_s=now_s) or not samples:
        raise ValueError("cannot seal an incomplete or stale stop window")
    return {"stopped_sim_time_s": now_s, "stopped_odometry": samples,
            "stop_measurement_start_stamp_s": samples[0]["stamp_s"]}


def verify_navigation_maps(output: Path, book: RouteBook) -> dict[str, str]:
    """Recompute every PGM/YAML byte that a navigation map server can read."""
    files: dict[str, str] = {}
    for closed in (False, True):
        state = "closed" if closed else "open"
        occupied = bytearray(not free for free in book.layout._make_raw_mask([book.layout.blocked_aisle] if closed else []))
        expected = {"navigation.pgm": book.layout._pgm_bytes(occupied),
                    "navigation.yaml": book.layout._yaml("navigation.pgm").encode("utf-8")}
        for name, content in expected.items():
            path = output / "maps" / state / name
            if path.is_symlink() or path.read_bytes() != content:
                raise ValueError(f"navigation map differs from authored source: {state}/{name}")
            files[path.relative_to(output).as_posix()] = sha(path)
    actual = {p.relative_to(output).as_posix() for p in (output / "maps").rglob("*") if p.is_file()}
    if actual not in (set(files), set(files) | {"maps/provenance.json"}):
        raise ValueError("navigation map exact file set differs")
    return files


def prepare_params(project: Path, output: Path, session_root: Path, session: Mapping[str, Any],
                   ready: Mapping[str, Any], book: RouteBook, config: Mapping[str, Any]) -> tuple[Path, dict[bool, Path]]:
    import yaml
    radii = {robot: _planning_radius(ready["robots"][robot], config["localization_planning_margin_m"])[0] for robot in ROBOT_IDS}
    if any(not math.isclose(value, book.planning_radius_m, abs_tol=1e-6) for value in radii.values()):
        raise ValueError("loaded collision radius differs from G3 routing provenance")
    maps: dict[bool, Path] = {}
    for closed in (False, True):
        folder = output / "maps" / ("closed" if closed else "open")
        folder.mkdir(parents=True)
        occupied = bytearray(not free for free in book.layout._make_raw_mask([book.layout.blocked_aisle] if closed else []))
        pgm = folder / "navigation.pgm"
        pgm.write_bytes(book.layout._pgm_bytes(occupied))
        path = folder / "navigation.yaml"
        path.write_text(book.layout._yaml(pgm.name), encoding="utf-8", newline="\n")
        maps[closed] = path
    scenario = json.loads((session_root / "scenarios.json").read_text(encoding="utf-8"))["scenarios"][session["scenario_id"]]
    template = yaml.safe_load((project / "configs/factory-v3/nav2-g2.yaml").read_text(encoding="utf-8"))
    params = build_namespaced_params(template, scenario, radii,
                                    physical_map=str(session_root / "maps/open/map.yaml"),
                                    navigation_map=str(maps[session["condition"] == "closure"]))
    path = output / "nav2-g3-params.yaml"
    path.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
    save(output / "maps/provenance.json", {"planning_radius_m": radii, "source_layout_sha256": book.layout.source_sha256,
         "loaded_radius_rechecked": True, "files": verify_navigation_maps(output, book),
         "scope": "authored navigation maps, current closure only; no simulator truth"})
    return path, maps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    session_root = args.session.resolve(strict=True)
    session = verify_session(session_root, expected_sha256=os.environ["FACTORY_OPS_G3_SESSION_SHA256"])
    config = json.loads((session_root / "config.json").read_text(encoding="utf-8"))
    runtime_config = config["g3_runtime"]
    scenario_id = session["scenario_id"]
    output = args.output.resolve(strict=True)
    book = RouteBook.from_files(project / "configs/factory-v3/layout.json", project / "configs/factory-v3/g3-routing.json")
    deadline = time.monotonic() + config["max_wall_time_s"]
    result: dict[str, Any] = {"run_id": args.run_id, "scenario_id": scenario_id, "status": "running", "stage": "wait_scene",
                              "primary_evaluation": False, "scope": session["scope"], "ground_truth_input": False,
                              "workload_sha256": session["workload_sha256"], "session_sha256": sha(session_root / "session.json")}
    launch = bag = ros = mission = None
    nodes: list[Any] = []
    streams: list[Any] = []
    cleanup: list[dict[str, Any]] = []
    origin: float | None = None
    robots: dict[str, dict[str, Any]] = {}
    history: list[dict[str, Any]] = []
    lease_sender: Any = None
    manager = RouteReservationManager(args.run_id, ROBOT_IDS, pose_ttl_s=runtime_config["pose_ttl_s"])
    try:
        while not (output / "sim-ready.json").is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError("G3 scene did not become ready")
            scene_file = output / "g3-scene.json"
            if scene_file.is_file() and json.loads(scene_file.read_text())["status"] == "failed":
                raise RuntimeError("G3 scene failed before ready")
            time.sleep(0.2)
        ready = _load_ready(output / "sim-ready.json", args.run_id, scenario_id)
        params_path, maps = prepare_params(project, output, session_root, session, ready, book, config)
        result["inputs"] = {name: {"path": str(path), "sha256": sha(path)} for name, path in {
            "mission_source": Path(__file__).resolve(), "session": session_root / "session.json",
            "generated_params": params_path, "config": session_root / "config.json",
            "nav2_template": project / "configs/factory-v3/nav2-g2.yaml"}.items()}
        result["navigation_map_files"] = verify_navigation_maps(output, book)
        result["inputs"]["navigation_map_provenance"] = {"path": str(output / "maps/provenance.json"), "sha256": sha(output / "maps/provenance.json")}
        os.environ["FACTORY_OPS_NAV2_PARAMS"] = str(params_path)
        os.environ["ROS_DOMAIN_ID"] = str(config["ros_domain_id"])
        import rclpy as ros
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import PoseStamped
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import ComputePathToPose, FollowPath
        from nav2_msgs.srv import LoadMap
        from nav_msgs.msg import Odometry
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
        from rclpy.service_introspection import ServiceIntrospectionState
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener, TransformException
        from unique_identifier_msgs.msg import UUID as GoalUUID
        ros.init()
        event_node = ros.create_node("factory_g3_mission", parameter_overrides=[Parameter("use_sim_time", value=True)])
        nodes.append(event_node)
        event_pub = event_node.create_publisher(String, "/factory/g3/events", 50)
        auth_pub = event_node.create_publisher(String, "/factory/g3/authorizations", 50)
        route_pub = event_node.create_publisher(String, "/factory/g3/reservations", 50)
        logs = {}
        for name in ("actions", "localization", "authorizations", "route-events", "nav2", "rosbag"):
            logs[name] = (output / (name + (".log" if name in {"nav2", "rosbag"} else ".jsonl"))).open("w", encoding="utf-8")
            streams.append(logs[name])
        event_seq = 0
        route_cursor = 0
        filters_ready = False
        scan_epoch_ns: int | None = None
        raw_stamp_peak = -1

        def now() -> float:
            return event_node.get_clock().now().nanoseconds / 1e9

        def journal(kind: str, body: Mapping[str, Any]) -> None:
            nonlocal event_seq
            event_seq += 1
            value = {"run_id": args.run_id, "scenario_id": scenario_id, "event_seq": event_seq,
                     "sim_time_s": now(), "kind": kind, **body}
            line = json.dumps(value, allow_nan=False)
            logs["actions"].write(line + "\n"); logs["actions"].flush()
            message = String(); message.data = line; event_pub.publish(message)

        for robot_id in ROBOT_IDS:
            node = ros.create_node(f"g3_adapter_{robot_id}", namespace=robot_id,
                 cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
                 parameter_overrides=[Parameter("use_sim_time", value=True)])
            nodes.append(node)
            buffer = Buffer(node=node)
            stop_gate = StoppedGate(window_s=runtime_config["stopped_window_s"],
                 max_linear_m_s=runtime_config["stopped_linear_m_s"], max_angular_rad_s=runtime_config["stopped_angular_rad_s"],
                 ttl_s=runtime_config["odom_ttl_s"])
            robots[robot_id] = {"node": node, "tf": buffer, "listener": TransformListener(buffer, node),
                "compute": ActionClient(node, ComputePathToPose, "compute_path_to_pose"),
                "follow": IntrospectedActionClient(node, FollowPath, "follow_path", cancel_type=CancelGoal,
                    event_qos=qos_profile_services_default, introspection_state=ServiceIntrospectionState.CONTENTS),
                "load_map": node.create_client(LoadMap, "navigation_map_server/load_map"),
                "lifecycle": {name: node.create_client(GetState, f"{name}/get_state") for name in ("planner_server", "controller_server", "bt_navigator")},
                "pose": None, "last_observation": -math.inf, "phase": "idle", "anchor": session["workload"]["initial_robots"][robot_id],
                "park": session["workload"]["initial_robots"][robot_id], "leg": None, "generation": None,
                "held_tiles": frozenset(), "sequence": 0, "last_lease": -math.inf, "last_request": -math.inf,
                "stop_gate": stop_gate, "stop_after": None, "stop_samples": [], "odom_count": 0,
                "scan_counts": {key: 0 for key in ("front_raw", "rear_raw", "front", "rear", "localization_front", "localization_rear")},
                "subscriptions": [], "map_future": None}
            rt = robots[robot_id]
            front = node.create_publisher(LaserScan, "scan_front", qos_profile_sensor_data)
            rear = node.create_publisher(LaserScan, "scan_rear", qos_profile_sensor_data)
            localization = node.create_publisher(LaserScan, "scan_localization", qos_profile_sensor_data)

            def scan(message: Any, robot: str, side: str, publisher: Any, local_pub: Any) -> None:
                nonlocal raw_stamp_peak
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                raw_stamp_peak = max(raw_stamp_peak, stamp)
                if not filtered_scan_is_recordable(filters_ready, scan_epoch_ns, stamp):
                    return
                filtered = copy.deepcopy(message)
                filtered.ranges = mask_scan_ranges(message.angle_min, message.angle_max, message.angle_increment, message.ranges)
                publish_scan_fanout(filtered, (publisher, local_pub))
                for key in (f"{side}_raw", side, f"localization_{side}"):
                    robots[robot]["scan_counts"][key] += 1

            for side, publisher in (("front", front), ("rear", rear)):
                rt["subscriptions"].append(node.create_subscription(LaserScan, f"scan_{side}_raw",
                    lambda message, r=robot_id, s=side, p=publisher, lp=localization: scan(message, r, s, p, lp), qos_profile_sensor_data))

            def odometry(message: Any, robot: str) -> None:
                state = robots[robot]
                state["odom_count"] += 1
                state["last_odom_stamp_ns"] = path_header_stamp_ns({
                    "sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec})
                stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                state["last_odom"] = {"stamp_s": stamp, "linear_m_s": float(message.twist.twist.linear.x),
                                      "angular_rad_s": float(message.twist.twist.angular.z)}
                if state["stop_after"] is not None and stamp > state["stop_after"]:
                    state["stop_gate"].update(state["last_odom"]["linear_m_s"], state["last_odom"]["angular_rad_s"], stamp_s=stamp, now_s=now())
                    state["stop_samples"] = state["stop_gate"].samples
            rt["subscriptions"].append(node.create_subscription(Odometry, "odom", lambda message, r=robot_id: odometry(message, r), qos_profile_sensor_data))

        qos = output / "bag-qos.yaml"
        qos.write_text("".join(f"/{robot}/{topic}:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n" for robot in ROBOT_IDS for topic in ("tf_static", "map", "navigation_map")), encoding="utf-8")
        bag_topics = ["/clock", "/factory/g3/events", "/factory/g3/authorizations", "/factory/g3/reservations"]
        for robot in ROBOT_IDS:
            bag_topics += [f"/{robot}/{name}" for name in ("scan_front_raw", "scan_rear_raw", "scan_front", "scan_rear", "scan_localization", "odom", "tf", "tf_static", "map", "navigation_map", "amcl_pose", "plan", "received_global_plan", "cmd_vel", "compute_path_to_pose/_action/status", "follow_path/_action/status", "follow_path/_action/feedback")]
            bag_topics.extend(robots[robot]["follow"].service_event_topics)
        service_event_topics = tuple(topic for rt in robots.values() for topic in rt["follow"].service_event_topics)
        service_names = [topic.removesuffix("/_service_event") for topic in service_event_topics]
        # Generated action events use /action/ types; list them explicitly as topics too.
        bag = subprocess.Popen(["ros2", "bag", "record", "--use-sim-time", "--include-hidden-topics", "--storage", "sqlite3", "--output", str(output / "rosbag"), "--qos-profile-overrides-path", str(qos), "--topics", *bag_topics, "--services", *service_names], stdout=logs["rosbag"], stderr=subprocess.STDOUT, start_new_session=True)
        launch = subprocess.Popen(["ros2", "launch", str(project / "scripts/ros/g2.launch.py")], stdout=logs["nav2"], stderr=subprocess.STDOUT, start_new_session=True)
        result["stage"] = "evidence_discovery"
        discovery_end = min(deadline, time.monotonic() + 300)
        while True:
            for node in nodes:
                ros.spin_once(node, timeout_sec=.005)
            if time.monotonic() > discovery_end or launch.poll() is not None or bag.poll() is not None:
                raise RuntimeError("G3 recorder/Nav2 evidence discovery failed")
            if now() > 0 and evidence_subscribers_ready((event_pub, auth_pub, route_pub)) and named_subscriber_topics_ready(event_node, (*scan_recording_topics(), *service_event_topics), "rosbag2_recorder"):
                break
        scan_epoch_ns = max(event_node.get_clock().now().nanoseconds, raw_stamp_peak)
        filters_ready = True
        bounds = (book.layout.bounds.xmin, book.layout.bounds.ymin, book.layout.bounds.xmax, book.layout.bounds.ymax)
        reservation_radius = book.planning_radius_m + runtime_config["reservation_tracking_margin_m"]

        def tiles_at(xy: tuple[float, float], radius: float) -> frozenset[tuple[int, int]]:
            return swept_tiles([xy], radius, tile_size_m=book.tile_size_m, bounds_xy=bounds)

        def drain_routes() -> None:
            nonlocal route_cursor
            for row in manager.events_since(route_cursor):
                line = json.dumps(row, allow_nan=False)
                logs["route-events"].write(line + "\n")
                message = String(); message.data = line; route_pub.publish(message)
                route_cursor = row["event_sequence"]
            logs["route-events"].flush()

        def lease(robot: str, *, forced_mode: str | None = None) -> int:
            rt = robots[robot]
            navigate = forced_mode is None and rt["phase"] == "following" and manager.permission(robot, rt["generation"], rt["follow_uuid"], now_s=now())
            drain_routes()
            mode = forced_mode or ("navigate" if navigate else "hold")
            rt["sequence"] += 1
            chunk = rt.get("chunk")
            payload = make_authorization(run_id=args.run_id, scenario_id=scenario_id, robot_id=robot,
                    sequence=rt["sequence"], now_s=now(), ttl_s=config["authorization_ttl_s"], mode=mode, decision_event_seq=route_cursor,
                    route_id=chunk.route_id if chunk is not None else None,
                    goal_uuid=rt.get("follow_uuid"), reservation_id=rt["generation"], resources=("G3_FULL_LEG",), exit_id="G3_FULL_LEG:end")
            payload["scope"].update({"reservation_generation": rt["generation"], "leg_id": rt["leg"]["plan"].route_id if rt["leg"] else None,
                "path_content_sha256": rt.get("path_hash"), "tile_hash": hashlib.sha256(json.dumps(sorted(rt["held_tiles"]), separators=(",", ":")).encode()).hexdigest()})
            atomic_json(output / "robots" / robot / "authorization.json", payload)
            line = json.dumps(payload, allow_nan=False)
            logs["authorizations"].write(line + "\n"); logs["authorizations"].flush()
            message = String(); message.data = line; auth_pub.publish(message)
            rt["last_lease"] = now()
            rt["last_authorization"] = payload
            return rt["sequence"]

        lease_sender = lease

        tick_clock_ns = 0

        def tick(*, require_fresh: bool, refresh_leases: bool = True) -> float:
            nonlocal tick_clock_ns
            if time.monotonic() > deadline or launch.poll() is not None or bag.poll() is not None:
                raise RuntimeError("G3 worker wall bound or dependent process failed")
            scene_path = output / "g3-scene.json"
            if scene_path.is_file() and json.loads(scene_path.read_text())["status"] == "failed":
                raise RuntimeError("G3 scene failed")
            for node in nodes:
                ros.spin_once(node, timeout_sec=.002)
            if not evidence_subscribers_ready((event_pub, auth_pub, route_pub)) or not named_subscriber_topics_ready(event_node, (*scan_recording_topics(), *service_event_topics), "rosbag2_recorder"):
                raise RuntimeError("G3 evidence/scan recording subscriptions were lost")
            tick_clock_ns = event_node.get_clock().now().nanoseconds
            stamp = tick_clock_ns / 1e9
            for robot, rt in robots.items():
                try:
                    transform = rt["tf"].lookup_transform("map", "base_link", Time())
                except TransformException as error:
                    cached = rt["pose"]
                    if require_fresh and (cached is None or not 0 <= stamp-cached["stamp_s"] <= runtime_config["pose_ttl_s"]):
                        raise RuntimeError(f"fresh map pose unavailable for {robot}") from error
                    transform = None
                if transform is not None:
                    try:
                        pose = localization_payload(transform, now_s=stamp, ttl_s=runtime_config["pose_ttl_s"])
                    except PoseNotFreshError as error:
                        cached = rt["pose"]
                        if require_fresh and (cached is None or not 0 <= stamp-cached["stamp_s"] <= runtime_config["pose_ttl_s"]):
                            raise RuntimeError(f"fresh map pose unavailable for {robot}") from error
                        pose = None
                else:
                    pose = None
                if pose is not None:
                    occupied = tiles_at((pose["x_m"], pose["y_m"]), book.planning_radius_m)
                    if rt["generation"] is not None and not occupied.issubset(rt["held_tiles"]):
                        raise RuntimeError(f"{robot} estimated planning footprint left its retained reservation")
                    rt["pose"] = pose
                    if stamp-rt["last_observation"] >= .1:
                        manager.observe(robot, occupied, pose_stamp_s=pose["stamp_s"], now_s=stamp)
                        logs["localization"].write(json.dumps({"run_id": args.run_id, "scenario_id": scenario_id, "robot_id": robot, "sim_time_s": stamp, **pose}, allow_nan=False) + "\n")
                        rt["last_observation"] = stamp
                if refresh_leases and stamp-rt["last_lease"] >= .1:
                    lease(robot)
            logs["localization"].flush()
            drain_routes()
            return stamp

        poller = LifecycleStatePoller([f"{robot}/{name}" for robot, rt in robots.items() for name in rt["lifecycle"]])
        lifecycle_clients = {f"{robot}/{name}": client for robot, rt in robots.items() for name, client in rt["lifecycle"].items()}
        startup_end = min(deadline, time.monotonic()+300)
        result["stage"] = "startup"
        while True:
            tick(require_fresh=False)
            states = poller.poll(time.monotonic(), lifecycle_clients, GetState.Request)
            if (all(value == 3 for value in states.values()) and all(rt["pose"] and rt["odom_count"] >= 5 and min(rt["scan_counts"].values()) >= 5 and rt["compute"].server_is_ready() and rt["follow"].server_is_ready() for rt in robots.values())):
                break
            if time.monotonic() > startup_end:
                raise TimeoutError("G3 planner/controller/TF/odometry/scan startup did not qualify")
        tick(require_fresh=True)
        for robot, rt in robots.items():
            held = tiles_at(book.nodes[rt["anchor"]], reservation_radius)
            decision = manager.reserve(robot, f"initial-{rt['anchor']}", str(uuid4()), held, now_s=now())
            if not decision.granted:
                raise RuntimeError("initial stationary footprint hold failed")
            rt["generation"], rt["held_tiles"] = decision.generation, held
        drain_routes()
        expected_initial: dict[str, dict[str, Any]] = {}
        for robot, rt in robots.items():
            lease(robot, forced_mode="hold")
            expected_initial[robot] = copy.deepcopy(rt["last_authorization"])
        result["stage"] = "initial_hold_barrier"
        initial_acks: dict[str, dict[str, Any]] = {}
        latched_at: float | None = None
        initial_deadline = min(deadline, time.monotonic()+30)
        while True:
            stamp = tick(require_fresh=True, refresh_leases=False)
            if time.monotonic() > initial_deadline or any(stamp >= payload["valid_until_sim_time_s"] for payload in expected_initial.values()):
                raise TimeoutError("initial reservation HOLD application did not qualify before lease expiry")
            for robot in ROBOT_IDS:
                path = output / "robots" / robot / "control-ack.json"
                if robot not in initial_acks and path.is_file():
                    ack = json.loads(path.read_text(encoding="utf-8"))
                    if valid_initial_hold_ack(ack, expected_initial[robot], now_s=stamp):
                        initial_acks[robot] = ack
            if latched_at is not None and stamp > latched_at and initial_hold_boundary_ready(initial_acks, expected_initial, now_s=stamp):
                break
            if latched_at is None and set(initial_acks) == set(ROBOT_IDS):
                latched_at = stamp
        initial_barrier = {"expected_authorizations": expected_initial, "acknowledgments": initial_acks,
                           "latched_sim_time_s": latched_at}
        result["initial_hold_barrier"] = initial_barrier
        origin = stamp
        result["origin_sim_time_s"] = origin
        journal("navigation_ready", {"lifecycle_states": states, "origin_sim_time_s": origin,
                                      "initial_hold_barrier": initial_barrier})
        atomic_json(output / "mission-start.json", {"run_id": args.run_id, "scenario_id": scenario_id, "sim_time_s": origin})
        load = session["workload"]["load"]
        goals = {name: GoalPose("map", *xy, runtime_config["anchor_yaw_rad"][name]) for name, xy in {**book.layout.stations, **book.layout.pockets}.items()}
        mission = FleetMission(args.run_id, [FleetOrder(**order) for order in session["workload"]["orders"]],
             robot_stations=session["workload"]["initial_robots"], policy=session["policy"],
             ledger_config=LedgerConfig(input_capacity=load["ci_capacity"], output_capacity=load["co_capacity"], workers=load["workers"], service_time=load["service_time_s"]),
             station_goals=goals, goal_timeout_s=session["workload"]["horizon_s"], eta_speed=load["nominal_speed_m_s"], eta_dwell=load["dwell_s"], horizon=session["workload"]["horizon_s"])
        journal("mission_started", {"origin_sim_time_s": origin, "workload_sha256": session["workload_sha256"],
                "strategy": runtime_config["reservation_strategy"], "reservation_tracking_margin_m": runtime_config["reservation_tracking_margin_m"]})
        current_closed = session["condition"] == "closure"
        last_update = last_progress = -math.inf
        completed_at: float | None = None

        def new_leg(robot: str, target: str, intent: Any = None) -> None:
            rt = robots[robot]
            plan = book.plan(rt["anchor"], target, closed=current_closed)
            full_tiles = swept_tiles(plan.points, reservation_radius, tile_size_m=book.tile_size_m, bounds_xy=bounds)
            rt["leg"] = {"plan": plan, "target": target, "intent": intent, "closed": current_closed, "index": 0,
                         "tiles": full_tiles, "kind": "business" if intent else "clearing"}
            rt["phase"] = "request"
            rt["chunk"] = None
            rt["last_request"] = -math.inf
            journal("leg_requested", {"robot_id": robot, "leg_id": plan.route_id, "kind_of_leg": rt["leg"]["kind"], "start": rt["anchor"], "target": target,
                    "nodes": plan.nodes, "tiles": sorted(full_tiles), "mission_goal_uuid": intent.goal_id if intent else None})

        result["stage"] = "orders"
        while True:
            stamp = tick(require_fresh=True)
            elapsed = stamp-origin
            scene_state = json.loads((output / "scene-state.json").read_text(encoding="utf-8"))
            if scene_state.get("run_id") != args.run_id or scene_state.get("scenario_id") != scenario_id or type(scene_state.get("current_closed")) is not bool:
                raise RuntimeError("G3 current scene-state identity differs")
            observed_closed = scene_state["current_closed"]
            if observed_closed != current_closed:
                if verify_navigation_maps(output, book) != result["navigation_map_files"]:
                    raise RuntimeError("navigation maps changed before LoadMap")
                current_closed = observed_closed
                for robot, rt in robots.items():
                    request = LoadMap.Request(); request.map_url = str(maps[current_closed])
                    rt["map_future"] = rt["load_map"].call_async(request)
                journal("current_closure_changed", {"current_closed": current_closed, "scene_event_sim_time_s": scene_state["sim_time_s"]})
            maps_ready = True
            for rt in robots.values():
                future = rt["map_future"]
                if future is not None:
                    if future.done():
                        response = future.result()
                        if response is None or response.result != 0:
                            raise RuntimeError("navigation map closure update failed")
                        rt["map_future"] = None
                    else:
                        maps_ready = False
            if elapsed > session["workload"]["horizon_s"]:
                if not mission.completed:
                    mission.finish_incomplete()
                raise TimeoutError("G3 qualification/pilot horizon reached; incomplete work retained")
            if elapsed-last_update >= .1:
                if not mission.completed:
                    mission.advance(elapsed)
                last_update = elapsed
            for robot, rt in robots.items():
                phase = rt["phase"]
                if phase == "request" and maps_ready and stamp-rt["last_request"] >= .2:
                    leg = rt["leg"]
                    if rt["chunk"] is None:
                        rt["chunk"] = make_chunk(book, rt["anchor"], leg["target"], closed=leg["closed"], final_goal=goals[leg["target"]])
                        rt["follow_uuid"] = leg["intent"].goal_id if rt["chunk"].final and leg["intent"] else str(uuid4())
                    decision = manager.transfer(robot, rt["generation"], rt["chunk"].route_id, rt["follow_uuid"], leg["tiles"], now_s=stamp)
                    rt["last_request"] = stamp
                    if not decision.granted:
                        continue
                    rt["generation"], rt["held_tiles"] = decision.generation, leg["tiles"]
                    chunk = rt["chunk"]
                    request = ComputePathToPose.Goal()
                    request.goal = PoseStamped(); request.goal.header.frame_id = "map"; request.goal.header.stamp = event_node.get_clock().now().to_msg()
                    request.goal.pose.position.x, request.goal.pose.position.y = chunk.goal_pose.x, chunk.goal_pose.y
                    request.goal.pose.orientation.z, request.goal.pose.orientation.w = math.sin(chunk.goal_pose.yaw/2), math.cos(chunk.goal_pose.yaw/2)
                    request.use_start = False; request.planner_id = "GridBased"
                    rt["compute_uuid"] = str(uuid4())
                    rt["send_future"] = rt["compute"].send_goal_async(request, goal_uuid=GoalUUID(uuid=list(UUID(rt["compute_uuid"]).bytes)))
                    rt["phase"], rt["phase_started"] = "compute_sent", stamp
                    rt["path_clock_first_seen_wall"] = None
                    rt["pending_path"] = None
                    rt["record"] = {"robot_id": robot, "chunk": chunk.to_dict(), "leg_id": leg["plan"].route_id,
                         "leg_target": leg["target"], "leg_kind": leg["kind"], "reservation_tiles": sorted(leg["tiles"]),
                         "generation": rt["generation"], "planner_action_uuid": rt["compute_uuid"], "follow_action_uuid": rt["follow_uuid"],
                         "mission_goal_uuid": leg["intent"].goal_id if leg["intent"] else None, "requested_sim_time_s": stamp}
                    history.append(rt["record"])
                    journal("planner_goal_sent", rt["record"])
                elif phase in {"compute_sent", "follow_sent"} and rt["send_future"].done():
                    handle = rt["send_future"].result()
                    expected_uuid = rt["compute_uuid"] if phase == "compute_sent" else rt["follow_uuid"]
                    if handle is None or not handle.accepted or bytes(handle.goal_id.uuid).hex() != UUID(expected_uuid).hex:
                        raise RuntimeError("Nav2 rejected or changed a requested action UUID")
                    rt["handle"], rt["result_future"] = handle, handle.get_result_async()
                    if phase == "compute_sent":
                        rt["phase"] = "computing"
                    else:
                        accepted = manager.mark_goal_accepted(robot, rt["generation"], rt["follow_uuid"], now_s=stamp)
                        if not accepted.granted:
                            raise RuntimeError("reservation rejected accepted FollowPath binding")
                        rt["phase"] = "following"
                        rt["record"]["accepted_sim_time_s"] = stamp
                        rt["record"]["server_accepted_stamp"] = {"sec": int(handle.accepted_stamp.sec), "nanosec": int(handle.accepted_stamp.nanosec)}
                    journal("action_accepted", {"robot_id": robot, "action": "compute" if phase == "compute_sent" else "follow", "goal_uuid": expected_uuid, "generation": rt["generation"]})
                elif phase == "computing" and rt["result_future"].done():
                    terminal = rt["result_future"].result()
                    if terminal is None or terminal.status != 4 or terminal.result.error_code != 0:
                        raise RuntimeError(f"ComputePathToPose failed for {robot}: {terminal}")
                    wall_now = time.monotonic()
                    if rt["path_clock_first_seen_wall"] is None:
                        rt["pending_path"] = copy.deepcopy(terminal.result.path)
                        rt["pending_path_payload"] = path_payload(rt["pending_path"])
                        rt["path_clock_first_seen_wall"] = wall_now
                        rt["record"]["path_clock"] = {
                            "path_stamp_ns": path_header_stamp_ns(rt["pending_path_payload"]["header"]),
                            "first_observed_clock_ns": tick_clock_ns}
                    path, payload = rt["pending_path"], rt["pending_path_payload"]
                    clock_wait_wall_s = wall_now-rt["path_clock_first_seen_wall"]
                    if not path_clock_ready(payload["header"], clock_ns=tick_clock_ns,
                            elapsed_wall_s=clock_wait_wall_s,
                            max_wait_wall_s=runtime_config["path_clock_wait_wall_s"]):
                        if "path_clock_wait" not in rt["record"]:
                            observation = {"path_stamp_ns": path_header_stamp_ns(payload["header"]),
                                           "observed_clock_ns": tick_clock_ns, "mode": "hold"}
                            rt["record"]["path_clock_wait"] = observation
                            journal("planner_clock_wait", {"robot_id": robot, **observation})
                        continue
                    rt["record"]["path_clock"].update({"admitted_clock_ns": tick_clock_ns,
                                                      "waited_wall_s": clock_wait_wall_s})
                    geometry = qualify_computed_path(book, rt["chunk"], [[item["position_xyz_m"][0], item["position_xyz_m"][1]] for item in payload["poses"]],
                        frame_id=payload["header"]["frame_id"], stamp_s=path_header_stamp_ns(payload["header"])/1e9,
                        now_s=stamp, start_xy=(rt["pose"]["x_m"], rt["pose"]["y_m"]), max_age_s=runtime_config["path_ttl_s"])
                    if angle_error(path_final_yaw(payload), rt["chunk"].goal_pose.yaw) > .05:
                        raise RuntimeError("computed Path terminal yaw differs from the authored goal")
                    if not {tuple(tile) for tile in geometry["swept_tiles"]}.issubset(rt["held_tiles"]):
                        raise RuntimeError("computed Path leaves full-leg reservation")
                    rt["path_hash"] = path_content_sha256(payload)
                    rt["record"].update({"path": payload, "path_geometry": geometry, "path_content_sha256": rt["path_hash"], "planner_status": 4, "planner_error_code": 0})
                    request = FollowPath.Goal(); request.path = path
                    request.controller_id = "FollowPath"; request.goal_checker_id = "goal_checker"; request.progress_checker_id = "progress_checker"
                    follow_path_hash = path_content_sha256(path_payload(request.path))
                    if follow_path_hash != rt["path_hash"]:
                        raise RuntimeError("FollowPath goal mutated the admitted planner Path")
                    rt["record"]["follow_goal_path_content_sha256"] = follow_path_hash
                    rt["send_future"] = rt["follow"].send_goal_async(request, goal_uuid=GoalUUID(uuid=list(UUID(rt["follow_uuid"]).bytes)))
                    rt["phase"] = "follow_sent"
                    journal("follow_goal_sent", rt["record"])
                elif phase == "following" and rt["result_future"].done():
                    terminal = rt["result_future"].result()
                    rt["record"].update({"status": int(terminal.status), "error_code": int(terminal.result.error_code), "error_msg": str(terminal.result.error_msg), "terminal_sim_time_s": stamp})
                    if terminal.status != 4 or terminal.result.error_code != 0:
                        raise RuntimeError(f"FollowPath failed: {rt['record']}")
                    rt["phase"] = "stopping"
                    rt["stop_after"] = None; rt["stop_gate"].reset(); rt["stop_samples"] = []
                    rt["hold_sequence"] = lease(robot, forced_mode="hold")
                    rt["stop_wait_started"] = stamp
                    journal("action_terminal", rt["record"])
                elif phase == "stopping":
                    ack_path = output / "robots" / robot / "control-ack.json"
                    if ack_path.is_file():
                        ack = json.loads(ack_path.read_text(encoding="utf-8"))
                        if valid_hold_ack(ack, run_id=args.run_id, scenario_id=scenario_id, robot_id=robot,
                                          minimum_sequence=rt["hold_sequence"], generation=rt["generation"],
                                          goal_uuid=rt["follow_uuid"], terminal_time_s=rt["record"]["terminal_sim_time_s"], now_s=stamp):
                            if rt["stop_after"] is None:
                                rt["stop_after"] = ack["evaluation_sim_time_s"]
                                rt["record"]["hold_ack"] = ack
                    if stamp-rt["stop_wait_started"] > 30:
                        raise TimeoutError("post-terminal hold acknowledgment / fresh odometry stop failed")
                    if rt["stop_after"] is not None and rt["stop_gate"].is_stopped(now_s=stamp):
                        pose = rt["pose"]; target = rt["chunk"].goal_pose
                        validate_arrival(pose, target, xy_limit=runtime_config["station_xy_tolerance_m"], yaw_limit=runtime_config["station_yaw_tolerance_rad"])
                        finished = manager.mark_goal_finished(robot, rt["generation"], rt["follow_uuid"], now_s=stamp)
                        if not finished.granted:
                            raise RuntimeError("finished goal reservation binding differs")
                        rt["record"].update(stopped_odometry_record(rt["stop_gate"], now_s=stamp))
                        rt["record"]["arrival_pose"] = dict(pose)
                        rt["stop_after"] = None
                        rt["anchor"] = rt["chunk"].goal
                        if not rt["chunk"].final:
                            rt["chunk"] = None; rt["phase"] = "request"; rt["last_request"] = -math.inf
                        elif rt["leg"]["kind"] == "clearing":
                            held = tiles_at(book.nodes[rt["anchor"]], reservation_radius)
                            shrunk = manager.shrink_hold(robot, rt["generation"], held, now_s=stamp, stopped=True)
                            if not shrunk.granted:
                                raise RuntimeError("parking hold shrink failed")
                            rt["generation"], rt["held_tiles"] = shrunk.generation, held
                            journal("parked", {"robot_id": robot, "anchor": rt["anchor"], "generation": rt["generation"], "tiles": sorted(held)})
                            rt["leg"] = None; rt["chunk"] = None; rt["phase"] = "idle"
                        else:
                            rt["phase"] = "dwell"; rt["dwell_until"] = stamp + load["dwell_s"]
                        journal("edge_stopped", rt["record"])
                elif phase == "dwell" and stamp >= rt["dwell_until"]:
                    intent = rt["leg"]["intent"]
                    pose = rt["pose"]
                    evidence = NavigationResult(str(uuid4()), rt["follow_uuid"], intent.task_id, intent.station, intent.attempt, 4, elapsed,
                        LocalizationPose("map", pose["x_m"], pose["y_m"], pose["yaw_rad"], pose["stamp_s"]-origin))
                    receipt = mission.submit_result(robot, evidence, error_code=0)
                    rt["record"].update({"handoff_sim_time_s": stamp, "receipt": receipt.to_dict(), "navigation_result": evidence.to_dict()})
                    journal("handoff_commit", {"robot_id": robot, "intent": intent.to_dict(), "result": evidence.to_dict(), "receipt": receipt.to_dict(), "ledger_seq": mission.ledger.ledger_seq})
                    rt["leg"] = None; rt["chunk"] = None; rt["phase"] = "idle"
                    if intent.transition is Transition.DROP:
                        new_leg(robot, rt["park"])
                if rt["phase"] in {"compute_sent", "computing", "follow_sent", "following"} and stamp-rt["phase_started"] > runtime_config["chunk_timeout_s"]:
                    raise TimeoutError(f"G3 chunk action deadline: {robot}")
            eligible = [robot for robot, rt in robots.items() if rt["phase"] == "idle" and rt["leg"] is None]
            if not mission.completed:
                def cost(robot: Any, task: Any) -> float:
                    distance = book.plan(robots[robot.robot_id]["anchor"], task.pickup_station, closed=current_closed).distance_m
                    if session["policy"] == "eta":
                        distance += book.plan(task.pickup_station, task.drop_station, closed=current_closed).distance_m
                    return distance
                mission.dispatch(elapsed, cost, eligible_robot_ids=eligible)
            for robot in eligible:
                intent = mission.active_intents.get(robot)
                if intent is not None:
                    new_leg(robot, intent.station, intent)
            if manager.detect_wait_cycle(now_s=stamp):
                raise RuntimeError("persistent full-leg wait cycle; preserve reservations and fail-stop")
            if mission.completed and completed_at is None:
                completed_at = elapsed
                journal("orders_completed", {"completed_orders": mission.completed_order_count, "elapsed_sim_s": completed_at})
            if mission.completed and all(rt["phase"] == "idle" and rt["anchor"] == rt["park"] for rt in robots.values()):
                break
            if elapsed-last_progress >= 1:
                atomic_json(output / "mission-progress.json", {"run_id": args.run_id, "scenario_id": scenario_id, "elapsed_sim_s": elapsed,
                     "completed_orders": mission.completed_order_count, "released_orders": mission.released_order_count,
                     "robots": {robot: {"phase": rt["phase"], "anchor": rt["anchor"], "target": rt["leg"]["target"] if rt["leg"] else None, "generation": rt["generation"]} for robot, rt in robots.items()}})
                last_progress = elapsed
        for robot in ROBOT_IDS:
            lease(robot, forced_mode="hold")
        journal("scenario_complete", {"completed_orders": mission.completed_order_count})
        flush_started = now()
        while now()-flush_started < .2:
            tick(require_fresh=True)
        if verify_navigation_maps(output, book) != result["navigation_map_files"]:
            raise RuntimeError("navigation map bytes changed before terminal verification")
        result["stage"] = "terminal_recording"
        for rt in robots.values():
            rt["stop_after"] = None
        atomic_json(output / "mission-control.json", {"run_id": args.run_id, "scenario_id": scenario_id, "status": "passed"})
        def read_terminal_scene() -> Mapping[str, Any] | None:
            path = output / "g3-scene.json"
            return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None
        terminal_boundary = wait_for_terminal_recording(
            tick=lambda: tick(require_fresh=False), read_scene=read_terminal_scene,
            read_odometry_ns=lambda: {robot: rt.get("last_odom_stamp_ns") for robot, rt in robots.items()},
            run_id=args.run_id, scenario_id=scenario_id, minimum_sim_time_ns=tick_clock_ns,
            wait_wall_s=runtime_config["terminal_recording_wait_wall_s"])
        result.update({"status": "passed", "stage": "complete", "origin_sim_time_s": origin,
            "terminal_recording": terminal_boundary,
            "completed_orders_sim_s": completed_at, "mission": mission.export(), "actions": history,
            "route_snapshot": manager.snapshot(now_s=now()), "physical_anchor_by_robot": {robot: rt["anchor"] for robot, rt in robots.items()},
            "scan_counts": {robot: rt["scan_counts"] for robot, rt in robots.items()}, "scan_epoch_ns": scan_epoch_ns,
            "odom_counts": {robot: rt["odom_count"] for robot, rt in robots.items()}, "bag_topics_requested": bag_topics,
            "planning_radius_m": book.planning_radius_m, "reservation_radius_m": reservation_radius})
        drain_routes()
        print("FACTORY_OPS_G3_MISSION=PASS", flush=True)
    except BaseException as error:
        reason = failure_reason(error)
        result.update({"status": "failed", "error": {"type": type(error).__name__, "message": reason, "traceback": traceback.format_exc(limit=30)}, "actions": history})
        if mission is not None:
            if not (mission.completed or mission.failed or mission.censored):
                mission.fail(reason, sim_time=mission.sim_time)
            result["mission"] = mission.export()
        for robot, rt in robots.items():
            try:
                if lease_sender is not None:
                    lease_sender(robot, forced_mode="emergency")
                if rt.get("handle") is not None and rt["phase"] == "following":
                    rt["handle"].cancel_goal_async()
            except Exception as cleanup_error:
                cleanup.append({"label": f"{robot}_fault_stop", "error": str(cleanup_error)})
        atomic_json(output / "mission-control.json", {"run_id": args.run_id, "scenario_id": scenario_id, "status": "failed", "reason": reason})
        print(f"FACTORY_OPS_G3_MISSION=FAIL {type(error).__name__}: {reason}", flush=True)
    finally:
        for process, label in ((bag, "rosbag"), (launch, "nav2")):
            try:
                cleanup.append(stop_process(process, label))
            except BaseException as error:
                cleanup.append({"label": label, "error": str(error)}); result["status"] = "failed"
        for node in reversed(nodes):
            try:
                node.destroy_node()
            except BaseException as error:
                cleanup.append({"label": "destroy_node", "error": str(error)}); result["status"] = "failed"
        if ros is not None:
            try:
                ros.shutdown()
            except BaseException as error:
                cleanup.append({"label": "rclpy.shutdown", "error": str(error)}); result["status"] = "failed"
        for stream in streams:
            try:
                stream.close()
            except BaseException as error:
                cleanup.append({"label": "close_stream", "error": str(error)}); result["status"] = "failed"
        result["cleanup"] = cleanup
        atomic_json(output / "mission-result.json", result)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
