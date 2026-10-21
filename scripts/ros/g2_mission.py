#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Run two namespaced Jazzy Nav2 stacks through the G2 traffic scenarios."""
from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
from typing import Any, Callable, Mapping, Sequence
from uuid import UUID

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.layout import Layout
from factory_ops.compat import replace_shared_file, write_shared_text
from factory_ops.fleet_registry import FleetRegistry
from factory_ops.traffic import Decision, TrafficController
from factory_ops.traffic_observation import project_observation

ROBOT_IDS = ("carter1", "carter2")
SUCCESS = 4
CANCELED = 5


def fleet_identity(robot_ids: Sequence[str] = ROBOT_IDS,
                   namespace_by_id: Mapping[str, str] | None = None) -> tuple[tuple[str, ...], dict[str, str]]:
    """Bind logical IDs to unique, single-segment ROS namespaces."""
    if isinstance(robot_ids, (str, bytes)) or not isinstance(robot_ids, Sequence) or not robot_ids:
        raise ValueError("fleet IDs must be a nonempty sequence")
    ids = tuple(robot_ids)
    if any(not isinstance(value, str) or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value) is None for value in ids):
        raise ValueError("fleet IDs must be valid identifiers")
    if len(set(ids)) != len(ids):
        raise ValueError("fleet IDs must be unique")
    namespaces = {robot: robot for robot in ids}
    if namespace_by_id is not None:
        if set(namespace_by_id) != set(ids):
            raise ValueError("namespace mapping must match fleet IDs exactly")
        namespaces = dict(namespace_by_id)
    if (any(not isinstance(value, str) or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value) is None
            for value in namespaces.values()) or len(set(namespaces.values())) != len(ids)):
        raise ValueError("fleet namespaces must be unique valid identifiers")
    return ids, namespaces


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    write_shared_text(temporary, json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")
    replace_shared_file(temporary, path)


def mask_scan_ranges(
    angle_min: float, angle_max: float, angle_increment: float, ranges: Sequence[float],
) -> list[float]:
    values = (angle_min, angle_max, angle_increment)
    if (not all(not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
                for value in values) or angle_increment <= 0 or angle_max < angle_min):
        raise ValueError("LaserScan angular metadata must be finite, increasing and positive-step")
    # ROS producers use both inclusive [min, max] and upper-exclusive [min, max)
    # angular topology. Bound float32 metadata drift without accepting arbitrary lengths.
    tolerance = 8 * 2 ** -23 * max(1.0, abs(angle_min), abs(angle_max), len(ranges) * abs(angle_increment))
    inclusive_error = abs(angle_min + (len(ranges) - 1) * angle_increment - angle_max)
    exclusive_error = abs(angle_min + len(ranges) * angle_increment - angle_max)
    if not ranges or min(inclusive_error, exclusive_error) > tolerance:
        raise ValueError("LaserScan ranges length disagrees with inclusive/exclusive angular metadata")
    result: list[float] = []
    for index, raw in enumerate(ranges):
        value = float(raw)
        angle = math.atan2(math.sin(angle_min + index * angle_increment),
                           math.cos(angle_min + index * angle_increment))
        result.append(value if abs(angle) <= math.pi / 2 + 1e-12 else math.inf)
    return result


def evidence_subscribers_ready(publishers: Sequence[Any]) -> bool:
    """Report DDS discovery only; decoded bag validation remains authoritative."""
    return bool(publishers) and all(publisher.get_subscription_count() >= 1 for publisher in publishers)


def named_subscriber_topics_ready(
    node: Any, topics: Sequence[str], subscriber_node_name: str,
) -> bool:
    expected = subscriber_node_name.lstrip("/")
    return bool(topics) and all(
        any(str(info.node_name).lstrip("/") == expected
            for info in node.get_subscriptions_info_by_topic(topic))
        for topic in topics
    )


def filtered_scan_is_recordable(recording_ready: bool, epoch_ns: int | None, stamp_ns: int) -> bool:
    return (recording_ready and type(epoch_ns) is int and type(stamp_ns) is int
            and epoch_ns >= 0 and stamp_ns > epoch_ns)


def publish_scan_fanout(message: Any, publishers: Sequence[Any]) -> None:
    if len(publishers) != 2:
        raise ValueError("filtered scan fan-in requires costmap and localization publishers")
    for publisher in publishers:
        publisher.publish(message)


def scan_recording_topics(robot_ids: Sequence[str] = ROBOT_IDS, *,
                          namespace_by_id: Mapping[str, str] | None = None) -> tuple[str, ...]:
    ids, namespaces = fleet_identity(robot_ids, namespace_by_id)
    suffixes = ("scan_front_raw", "scan_rear_raw", "scan_front", "scan_rear", "scan_localization")
    return tuple(f"/{namespaces[robot_id]}/{suffix}" for robot_id in ids for suffix in suffixes)


class PoseFreshnessGate:
    """Track only TF samples whose own timestamp is fresh in simulation time."""

    def __init__(self, robot_ids: Sequence[str], freshness_s: float) -> None:
        if freshness_s <= 0 or not math.isfinite(freshness_s):
            raise ValueError("pose freshness must be finite and positive")
        self.robot_ids = tuple(robot_ids)
        self.freshness_s = freshness_s
        self.last_success_s = {robot_id: -math.inf for robot_id in robot_ids}

    def accept(self, robot_id: str, *, pose_stamp_s: float, now_s: float) -> bool:
        if robot_id not in self.last_success_s:
            raise KeyError(robot_id)
        if (not math.isfinite(pose_stamp_s) or not math.isfinite(now_s)
                or pose_stamp_s > now_s + 1e-6 or now_s - pose_stamp_s > self.freshness_s):
            return False
        self.last_success_s[robot_id] = now_s
        return True

    def stale(self, now_s: float) -> tuple[str, ...]:
        return tuple(robot_id for robot_id in self.robot_ids
                     if now_s - self.last_success_s[robot_id] > self.freshness_s)


def _planning_radius(ready_robot: Mapping[str, Any], margin_m: float) -> tuple[float, float]:
    footprint = ready_robot.get("footprint")
    if not isinstance(footprint, Mapping) or not footprint.get("passed"):
        raise ValueError("each G2 robot needs a passed collision footprint")
    collision = float(footprint["collision_radius_m"])
    clearance = float(footprint["clearance_m"])
    evaluation = float(footprint["inflated_radius_m"])
    if (not math.isclose(clearance, 0.2, abs_tol=1e-9)
            or not math.isclose(evaluation, collision + clearance, abs_tol=1e-9)):
        raise ValueError("G2 footprint does not preserve the 0.2 m evaluation clearance")
    return evaluation + margin_m, evaluation


def prepare_params(
    project: Path, output: Path, ready: Mapping[str, Any], layout: Layout,
    config: Mapping[str, Any], scenario: Mapping[str, Any],
) -> tuple[Path, dict[str, float]]:
    import yaml

    margin = float(config["localization_planning_margin_m"])
    robots = ready.get("robots")
    if not isinstance(robots, Mapping) or set(robots) != set(ROBOT_IDS):
        raise ValueError("G2 ready file must contain exactly carter1 and carter2")
    radii: dict[str, float] = {}
    for robot_id in ROBOT_IDS:
        radii[robot_id], _ = _planning_radius(robots[robot_id], margin)
    if not math.isclose(radii["carter1"], radii["carter2"], abs_tol=1e-9):
        raise ValueError("G2 robots need one shared qualified planning radius")
    maps = output / "maps"
    maps.mkdir(exist_ok=False)
    occupied = bytearray(not free for free in layout._make_raw_mask())
    navigation_pgm = maps / "navigation.pgm"
    navigation_pgm.write_bytes(layout._pgm_bytes(occupied))
    navigation_yaml = maps / "navigation.yaml"
    navigation_yaml.write_text(layout._yaml(navigation_pgm.name), encoding="utf-8")
    template = yaml.safe_load((project / "configs/factory-v3/nav2-g2.yaml").read_text(encoding="utf-8"))
    result = build_namespaced_params(
        template, scenario, radii,
        physical_map=str(project / "runs/local/maps/map.yaml"),
        navigation_map=str(navigation_yaml),
    )
    params_path = output / "nav2-g2-params.yaml"
    params_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    atomic_json(maps / "provenance.json", {
        "source_layout_sha256": layout.source_sha256,
        "navigation_map": "authored physical obstacles and policy keepout; no simulator truth",
        "amcl_map": "authored physical obstacles only",
        "planning_radius_m": radii,
        "evaluation_clearance_m": 0.2,
        "localization_planning_margin_m": margin,
        "files": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in maps.iterdir() if path.is_file()},
    })
    return params_path, radii


def build_namespaced_params(
    template: Mapping[str, Any], scenario: Mapping[str, Any], radii: Mapping[str, float],
    *, physical_map: str, navigation_map: str, robot_ids: Sequence[str] = ROBOT_IDS,
    namespace_by_id: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Materialize one independent parameter tree per robot namespace."""
    ids, namespaces = fleet_identity(robot_ids, namespace_by_id)
    if set(scenario["spawns"]) != set(ids) or set(radii) != set(ids):
        raise ValueError("spawn and radius identities must match the fleet exactly")
    source = copy.deepcopy(dict(template))
    common = source.pop("/**")
    result: dict[str, Any] = {"/**": common}
    for robot_id in ids:
        ros_namespace = namespaces[robot_id]
        namespace = copy.deepcopy(source)
        namespace["map_server"]["ros__parameters"]["yaml_filename"] = physical_map
        namespace["navigation_map_server"]["ros__parameters"]["yaml_filename"] = navigation_map
        spawn = scenario["spawns"][robot_id]
        namespace["amcl"]["ros__parameters"]["initial_pose"] = {
            "x": float(spawn[0]), "y": float(spawn[1]), "z": 0.0, "yaw": float(spawn[2]),
        }
        for costmap in ("local_costmap", "global_costmap"):
            params = namespace[costmap][costmap]["ros__parameters"]
            params["robot_radius"] = radii[robot_id]
            params["inflation_layer"]["inflation_radius"] = radii[robot_id] + 0.2
            params["static_layer"]["map_topic"] = f"/{ros_namespace}/navigation_map"
            params["obstacle_layer"]["front"]["topic"] = f"/{ros_namespace}/scan_front"
            params["obstacle_layer"]["rear"]["topic"] = f"/{ros_namespace}/scan_rear"
        result[ros_namespace] = namespace
    return result


class ScenarioMachine:
    sequences = {
        "P0": ("initialized", "observing", "complete"),
        "F1": ("initialized", "c1_reserved", "c1_active", "c1_released", "c2_active", "complete"),
        "F2": ("initialized", "c2_claimed", "c2_active", "fault_active", "fault_cancelled",
               "c2_reissued", "c2_released", "c1_active", "complete"),
        "F3": ("initialized", "c1_reserved", "blocked_goal_active", "cycle_observed",
               "blocked_goal_cancelled", "yield_active", "yield_released", "ack_wait",
               "c1_released", "resume_active", "complete"),
    }

    def __init__(self, scenario_id: str) -> None:
        if scenario_id not in self.sequences:
            raise ValueError("scenario must be P0, F1, F2 or F3")
        self.scenario_id = scenario_id
        self.phase = "initialized"
        self.history = [self.phase]

    def transition(self, phase: str) -> None:
        sequence = self.sequences[self.scenario_id]
        current = sequence.index(self.phase)
        if current + 1 >= len(sequence) or sequence[current + 1] != phase:
            raise RuntimeError(f"invalid {self.scenario_id} transition: {self.phase} -> {phase}")
        self.phase = phase
        self.history.append(phase)


class LifecycleStatePoller:
    def __init__(self, clients: Mapping[str, Any], *, minimum_interval_s: float = 0.5) -> None:
        self.clients = dict(clients)
        self.states: dict[str, int | None] = {name: None for name in clients}
        self.futures: dict[str, Any] = {}
        self.last_request: dict[str, float] = {name: -math.inf for name in clients}
        self.minimum_interval_s = minimum_interval_s

    def poll(self, wall_time_s: float, request_factory: Callable[[], Any]) -> dict[str, int | None]:
        for name, client in self.clients.items():
            future = self.futures.get(name)
            if future is not None:
                if not future.done():
                    continue
                response = future.result()
                self.states[name] = response.current_state.id if response is not None else None
                del self.futures[name]
            if self.states[name] == 3 or wall_time_s - self.last_request[name] < self.minimum_interval_s:
                continue
            if client.service_is_ready():
                self.futures[name] = client.call_async(request_factory())
                self.last_request[name] = wall_time_s
        return dict(self.states)


class MovingChannelFault:
    """Fault injector owns duration; dispatch observes only link state and edges."""

    def __init__(self, robot_id: str, duration_s: float) -> None:
        if duration_s <= 0 or not math.isfinite(duration_s):
            raise ValueError("fault duration must be finite and positive")
        self.robot_id = robot_id
        self._duration_s = duration_s
        self._start_s: float | None = None
        self._end_s: float | None = None
        self._last_link_up = True

    def trigger(self, now_s: float) -> None:
        if self._start_s is not None:
            raise RuntimeError("fault can trigger only once")
        self._start_s = now_s
        self._end_s = now_s + self._duration_s

    def link_up(self, now_s: float) -> bool:
        return self._end_s is None or now_s >= self._end_s

    def edge(self, now_s: float) -> str | None:
        current = self.link_up(now_s)
        if current == self._last_link_up:
            return None
        self._last_link_up = current
        return "link_up" if current else "link_down"

    def evidence(self, now_s: float) -> dict[str, Any]:
        return {"robot_id": self.robot_id, "triggered": self._start_s is not None,
                "actual_start_sim_time_s": self._start_s,
                "actual_end_sim_time_s": now_s if self._start_s is not None and self.link_up(now_s) else None}


def authored_route_length(start_xy: Sequence[float], poses: Sequence[Sequence[float]]) -> float:
    if len(start_xy) < 2 or not poses:
        raise ValueError("route cost needs a start and at least one pose")
    previous = (float(start_xy[0]), float(start_xy[1]))
    total = 0.0
    for pose in poses:
        if len(pose) < 2:
            raise ValueError("route pose needs x and y")
        current = (float(pose[0]), float(pose[1]))
        if not all(math.isfinite(value) for value in current):
            raise ValueError("route pose must be finite")
        total += math.dist(previous, current)
        previous = current
    return total


@dataclass(slots=True)
class ActionEvidence:
    robot_id: str
    route_id: str
    action_type: str
    state: str = "sent"
    goal_uuid: str | None = None
    accepted_sim_time_s: float | None = None
    cancel_requested_sim_time_s: float | None = None
    terminal_sim_time_s: float | None = None
    terminal_status: int | None = None
    error_code: int | None = None

    def accepted(self, goal_uuid: str, sim_time_s: float) -> None:
        if self.state != "sent":
            raise RuntimeError("goal acceptance must follow sent")
        UUID(goal_uuid)
        self.goal_uuid = goal_uuid
        self.accepted_sim_time_s = sim_time_s
        self.state = "active"

    def request_cancel(self, sim_time_s: float) -> None:
        if self.state != "active":
            raise RuntimeError("only an active accepted goal can be cancelled")
        self.cancel_requested_sim_time_s = sim_time_s
        self.state = "cancel_requested"

    def terminal(self, status: int, error_code: int, sim_time_s: float) -> None:
        if self.state not in {"active", "cancel_requested"}:
            raise RuntimeError("terminal result requires an accepted goal")
        if self.state == "cancel_requested" and status != CANCELED:
            raise RuntimeError("cancel proof requires terminal GoalStatus.STATUS_CANCELED (5)")
        if self.state == "active" and (status != SUCCESS or error_code != 0):
            raise RuntimeError(f"navigation failed with status={status}, error_code={error_code}")
        self.terminal_status = status
        self.error_code = error_code
        self.terminal_sim_time_s = sim_time_s
        self.state = "canceled" if status == CANCELED else "succeeded"


def make_authorization(
    *, run_id: str, scenario_id: str, robot_id: str, sequence: int, now_s: float,
    ttl_s: float, mode: str, decision_event_seq: int, route_id: str | None = None,
    goal_uuid: str | None = None, reservation_id: int | None = None,
    recovery_id: int | None = None, resources: Sequence[str] = (), exit_id: str | None = None,
) -> dict[str, Any]:
    if mode not in {"navigate", "hold", "emergency"}:
        raise ValueError("invalid authorization mode")
    if ttl_s <= 0 or ttl_s > 0.5:
        raise ValueError("authorization TTL must be in (0, 0.5]")
    if mode == "navigate":
        if not route_id or not goal_uuid or not reservation_id or not resources or not exit_id:
            raise ValueError("navigate authorization requires accepted goal and reservation scope")
        UUID(goal_uuid)
    return {
        "schema_version": 1,
        "run_id": run_id,
        "robot_id": robot_id,
        "authorization_seq": sequence,
        "issued_sim_time_s": now_s,
        "valid_until_sim_time_s": now_s + ttl_s,
        "mode": mode,
        "scope": {
            "scenario_id": scenario_id,
            "route_id": route_id,
            "goal_uuid": goal_uuid,
            "decision_event_seq": decision_event_seq,
            "reservation_id": reservation_id,
            "recovery_id": recovery_id,
            "resources": list(resources),
            "exit_id": exit_id,
        },
    }


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


def _load_ready(path: Path, run_id: str, scenario_id: str, *, registry: FleetRegistry | None = None) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if (value.get("run_id") != run_id or value.get("scenario_id") != scenario_id
            or value.get("status") != "ready"):
        raise ValueError("G2 scene ready identity/status mismatch")
    robots = value.get("robots")
    if registry is not None:
        if not isinstance(robots, dict) or set(robots) != set(registry.robot_ids):
            raise ValueError("ready robot identities must match the registry exactly")
        for definition in registry.definitions:
            robot = robots[definition["robot_id"]]
            if (not isinstance(robot, Mapping) or robot.get("namespace") != definition["namespace"]
                    or robot.get("prim_path") != definition["prim_path"]):
                raise ValueError("ready namespace/prim differs from the registry")
            if not isinstance(robot.get("lidars"), (dict, list)) or not robot.get("ros"):
                raise ValueError("ready sensor/ROS metadata missing")
            footprint = robot.get("footprint")
            if (not isinstance(footprint, Mapping) or footprint.get("passed") is not True
                    or any(type(footprint.get(key)) not in (int, float)
                           or not math.isfinite(footprint[key]) or footprint[key] <= 0
                           for key in ("collision_radius_m", "clearance_m", "inflated_radius_m"))):
                raise ValueError("ready collision footprint did not qualify")
            radius, _ = _planning_radius(robot, 0.0)
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError("ready collision footprint did not qualify")
        return value
    if not isinstance(robots, dict) or set(robots) != set(ROBOT_IDS):
        raise ValueError("G2 scene ready must describe exactly two robots")
    for robot_id, namespace in (("carter1", "carter1"), ("carter2", "carter2")):
        robot = robots[robot_id]
        if robot.get("namespace") != namespace or robot.get("prim_path") != f"/World/{robot_id.title()}":
            raise ValueError(f"G2 ready namespace/prim mismatch for {robot_id}")
        if not isinstance(robot.get("lidars"), (dict, list)) or not robot.get("ros"):
            raise ValueError(f"G2 ready sensor/ROS metadata missing for {robot_id}")
    return value


def main() -> int:  # noqa: C901 - the explicit scenario phases are evidence boundaries
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario", required=True, choices=["P0", "F1", "F2", "F3"])
    parser.add_argument("--config", type=Path)
    parser.add_argument("--scenarios", type=Path)
    args = parser.parse_args()
    output = args.output.resolve(strict=True)
    project = Path(__file__).resolve().parents[2]
    config_path = (args.config or project / "configs/factory-v3/g2.json").resolve(strict=True)
    scenarios_path = (args.scenarios or project / "configs/factory-v3/g2-scenarios.json").resolve(strict=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    scenarios = json.loads(scenarios_path.read_text(encoding="utf-8"))["scenarios"]
    scenario = scenarios[args.scenario]
    machine = ScenarioMachine(args.scenario)
    deadline = time.monotonic() + float(config["max_wall_time_s"])
    result: dict[str, Any] = {
        "run_id": args.run_id, "scenario_id": args.scenario, "status": "running",
        "stage": "wait_scene", "ground_truth_input": False,
    }
    launch = bag = None
    streams: list[Any] = []
    nodes: list[Any] = []
    cleanup: list[dict[str, Any]] = []
    ros = None
    try:
        ready_path = output / "sim-ready.json"
        while not ready_path.is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError("G2 scene did not become ready")
            scene_path = output / "g2-scene.json"
            if scene_path.is_file() and json.loads(scene_path.read_text()).get("status") == "failed":
                raise RuntimeError("G2 Isaac scene failed before ready")
            time.sleep(0.2)
        ready = _load_ready(ready_path, args.run_id, args.scenario)
        layout = Layout.from_file(project / "configs/factory-v3/layout.json")
        params_path, planning_radii = prepare_params(project, output, ready, layout, config, scenario)
        planning_radius = planning_radii["carter1"]
        input_paths = {
            "mission_source": Path(__file__).resolve(),
            "launch_source": (project / "scripts/ros/g2.launch.py").resolve(strict=True),
            "config": config_path,
            "scenarios": scenarios_path,
            "nav2_template": (project / "configs/factory-v3/nav2-g2.yaml").resolve(strict=True),
            "generated_params": params_path,
        }
        result["inputs"] = {
            name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            for name, path in input_paths.items()
        }
        os.environ["FACTORY_OPS_NAV2_PARAMS"] = str(params_path)
        os.environ["ROS_DOMAIN_ID"] = str(config["ros_domain_id"])

        import rclpy as rclpy_module
        from geometry_msgs.msg import PoseStamped
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
        from nav_msgs.msg import Odometry
        from rclpy.action import ActionClient
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener

        ros = rclpy_module
        ros.init()
        event_node = ros.create_node("factory_g2_mission", parameter_overrides=[Parameter("use_sim_time", value=True)])
        nodes.append(event_node)
        event_pub = event_node.create_publisher(String, "/factory/g2/events", 10)
        authorization_pub = event_node.create_publisher(String, "/factory/g2/authorizations", 10)
        event_log = (output / "actions.jsonl").open("w", encoding="utf-8")
        traffic_log = (output / "traffic-events.jsonl").open("w", encoding="utf-8")
        pose_log = (output / "localization.jsonl").open("w", encoding="utf-8")
        auth_log = (output / "authorizations.jsonl").open("w", encoding="utf-8")
        launch_log = (output / "nav2.log").open("w", encoding="utf-8")
        bag_log = (output / "rosbag.log").open("w", encoding="utf-8")
        streams.extend([event_log, traffic_log, pose_log, auth_log, launch_log, bag_log])
        event_seq = 0

        def now() -> float:
            return event_node.get_clock().now().nanoseconds / 1e9

        def journal(kind: str, body: Mapping[str, Any]) -> int:
            nonlocal event_seq
            event_seq += 1
            item = {"run_id": args.run_id, "scenario_id": args.scenario,
                    "event_seq": event_seq, "sim_time_s": now(), "kind": kind, **body}
            line = json.dumps(item, allow_nan=False)
            event_log.write(line + "\n")
            event_log.flush()
            msg = String(); msg.data = line; event_pub.publish(msg)
            return event_seq

        robot_nodes: dict[str, Any] = {}
        tf_buffers: dict[str, Any] = {}
        tf_listeners: dict[str, Any] = {}
        clients: dict[str, dict[str, Any]] = {}
        lifecycle: dict[str, dict[str, Any]] = {}
        scan_counts = {robot: {name: 0 for name in (
            "front_raw", "rear_raw", "front", "rear", "localization_front", "localization_rear",
        )} for robot in ROBOT_IDS}
        scan_pre_record_drops = {robot: {side: 0 for side in ("front", "rear")} for robot in ROBOT_IDS}
        filters_ready = False
        scan_epoch_ns: int | None = None
        raw_stamp_peak_ns = -1
        odom_counts = {robot: 0 for robot in ROBOT_IDS}
        odom_speed = {robot: 0.0 for robot in ROBOT_IDS}
        last_moving = {robot: 0.0 for robot in ROBOT_IDS}
        subscriptions: list[Any] = []
        for robot_id in ROBOT_IDS:
            node = ros.create_node(
                f"g2_adapter_{robot_id}", namespace=robot_id,
                cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            nodes.append(node); robot_nodes[robot_id] = node
            buffer = Buffer(node=node); tf_buffers[robot_id] = buffer
            tf_listeners[robot_id] = TransformListener(buffer, node)
            clients[robot_id] = {
                "pose": ActionClient(node, NavigateToPose, "navigate_to_pose"),
                "through": ActionClient(node, NavigateThroughPoses, "navigate_through_poses"),
            }
            lifecycle[robot_id] = {
                name: node.create_client(GetState, f"{name}/get_state")
                for name in ("planner_server", "controller_server", "bt_navigator")
            }
            front_pub = node.create_publisher(LaserScan, "scan_front", qos_profile_sensor_data)
            rear_pub = node.create_publisher(LaserScan, "scan_rear", qos_profile_sensor_data)
            localization_pub = node.create_publisher(LaserScan, "scan_localization", qos_profile_sensor_data)

            def scan_callback(
                message: Any, *, robot: str, side: str,
                filtered_publisher: Any, localization_publisher: Any,
            ) -> None:
                nonlocal raw_stamp_peak_ns
                stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                raw_stamp_peak_ns = max(raw_stamp_peak_ns, stamp_ns)
                if not filtered_scan_is_recordable(filters_ready, scan_epoch_ns, stamp_ns):
                    scan_pre_record_drops[robot][side] += 1
                    return
                scan_counts[robot][f"{side}_raw"] += 1
                filtered = copy.deepcopy(message)
                filtered.ranges = mask_scan_ranges(
                    float(message.angle_min), float(message.angle_max), float(message.angle_increment), message.ranges,
                )
                publish_scan_fanout(filtered, (filtered_publisher, localization_publisher))
                scan_counts[robot][side] += 1
                scan_counts[robot][f"localization_{side}"] += 1

            subscriptions.extend([
                node.create_subscription(LaserScan, "scan_front_raw",
                    lambda msg, r=robot_id, p=front_pub, lp=localization_pub: scan_callback(
                        msg, robot=r, side="front", filtered_publisher=p, localization_publisher=lp,
                    ),
                    qos_profile_sensor_data),
                node.create_subscription(LaserScan, "scan_rear_raw",
                    lambda msg, r=robot_id, p=rear_pub, lp=localization_pub: scan_callback(
                        msg, robot=r, side="rear", filtered_publisher=p, localization_publisher=lp,
                    ),
                    qos_profile_sensor_data),
            ])

            def odom_callback(message: Any, *, robot: str) -> None:
                odom_counts[robot] += 1
                odom_speed[robot] = float(message.twist.twist.linear.x)
                if abs(odom_speed[robot]) > 0.02 or abs(float(message.twist.twist.angular.z)) > 0.02:
                    last_moving[robot] = now()

            subscriptions.append(node.create_subscription(
                Odometry, "odom", lambda msg, r=robot_id: odom_callback(msg, robot=r), qos_profile_sensor_data,
            ))

        qos_path = output / "bag-qos.yaml"
        qos_lines = []
        for robot_id in ROBOT_IDS:
            for topic in ("tf_static", "map", "navigation_map"):
                qos_lines.append(f"/{robot_id}/{topic}:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 1\n")
        qos_path.write_text("".join(qos_lines), encoding="utf-8")
        bag_topics = ["/clock", "/factory/g2/events", "/factory/g2/authorizations"]
        recorder_scan_topics = scan_recording_topics()
        for robot_id in ROBOT_IDS:
            prefix = f"/{robot_id}"
            bag_topics.extend(prefix + suffix for suffix in (
                "/scan_front_raw", "/scan_rear_raw", "/scan_front", "/scan_rear", "/scan_localization", "/odom",
                "/tf", "/tf_static", "/map", "/navigation_map", "/amcl_pose", "/plan", "/cmd_vel",
                "/navigate_to_pose/_action/status", "/navigate_to_pose/_action/feedback",
                "/navigate_through_poses/_action/status", "/navigate_through_poses/_action/feedback",
            ))
        bag = subprocess.Popen([
            "ros2", "bag", "record", "--use-sim-time", "--include-hidden-topics", "--storage", "sqlite3",
            "--output", str(output / "rosbag"), "--qos-profile-overrides-path", str(qos_path),
            "--topics", *bag_topics,
        ], stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
        launch = subprocess.Popen(
            ["ros2", "launch", str(project / "scripts/ros/g2.launch.py")],
            stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        evidence_publishers = (event_pub, authorization_pub)
        subscriber_deadline = min(deadline, time.monotonic() + 300.0)
        while True:
            if time.monotonic() >= subscriber_deadline:
                raise TimeoutError("rosbag did not discover G2 evidence and scan topics with live simulation time")
            if launch.poll() is not None or bag.poll() is not None:
                raise RuntimeError("G2 Nav2 launch or rosbag exited during evidence discovery")
            scene_path = output / "g2-scene.json"
            if scene_path.is_file() and json.loads(scene_path.read_text()).get("status") == "failed":
                raise RuntimeError("G2 Isaac scene failed during evidence discovery")
            for node in nodes:
                ros.spin_once(node, timeout_sec=0.01)
            if (evidence_subscribers_ready(evidence_publishers)
                    and named_subscriber_topics_ready(event_node, recorder_scan_topics, "rosbag2_recorder")
                    and now() > 0):
                break
        scan_epoch_ns = max(event_node.get_clock().now().nanoseconds, raw_stamp_peak_ns)
        scan_epoch_s = scan_epoch_ns / 1e9
        filters_ready = True
        traffic = TrafficController.from_file(project / config["traffic_config"], run_id=args.run_id)
        traffic_cursor = 0
        poses: dict[str, dict[str, float]] = {}
        channel_fault: MovingChannelFault | None = None
        active: dict[str, Any] = {}
        action_history: list[Any] = []
        lease_sequence = {robot: 0 for robot in ROBOT_IDS}
        authorization_event_seq = 0
        localization_event_seq = 0
        last_lease = {robot: -math.inf for robot in ROBOT_IDS}
        last_observation = -math.inf
        last_traffic_observation = {robot: -math.inf for robot in ROBOT_IDS}
        pose_freshness = PoseFreshnessGate(ROBOT_IDS, TrafficController.pose_freshness_s)
        navigation_ready = False

        @dataclass(slots=True)
        class RuntimeGoal:
            robot_id: str
            route_id: str
            route: Mapping[str, Any]
            reservation: Decision | None
            recovery_id: int | None
            navigate_allowed: bool
            evidence: ActionEvidence
            send_future: Any
            handle: Any = None
            result_future: Any = None
            cancel_future: Any = None

        def localization(robot_id: str) -> dict[str, float]:
            transform = tf_buffers[robot_id].lookup_transform("map", "base_link", Time())
            p = transform.transform.translation; q = transform.transform.rotation
            return {
                "x_m": float(p.x), "y_m": float(p.y),
                "yaw_rad": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                "stamp_s": transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9,
            }

        def pose_message(raw: Sequence[float]) -> Any:
            pose = PoseStamped(); pose.header.frame_id = "map"; pose.header.stamp = event_node.get_clock().now().to_msg()
            pose.pose.position.x = float(raw[0]); pose.pose.position.y = float(raw[1])
            pose.pose.orientation.z = math.sin(float(raw[2]) / 2); pose.pose.orientation.w = math.cos(float(raw[2]) / 2)
            return pose

        def send_route(route_id: str, reservation: Decision | None, *, navigate_allowed: bool,
                       recovery_id: int | None = None) -> RuntimeGoal:
            route = scenario["routes"][route_id]; robot_id = route["robot_id"]
            route_poses = route["poses"]
            if len(route_poses) == 1:
                goal = NavigateToPose.Goal(); goal.pose = pose_message(route_poses[0]); kind = "navigate_to_pose"
                client = clients[robot_id]["pose"]
            else:
                goal = NavigateThroughPoses.Goal(); goal.poses = [pose_message(raw) for raw in route_poses]
                kind = "navigate_through_poses"; client = clients[robot_id]["through"]
            evidence = ActionEvidence(robot_id, route_id, kind)

            def feedback(message: Any) -> None:
                journal("action_feedback", {"robot_id": robot_id, "route_id": route_id,
                    "action_type": kind, "feedback_type": type(message.feedback).__name__})

            runtime = RuntimeGoal(robot_id, route_id, route, reservation, recovery_id,
                                  navigate_allowed, evidence, client.send_goal_async(goal, feedback_callback=feedback))
            active[robot_id] = runtime
            action_history.append(runtime)
            journal("goal_sent", {"robot_id": robot_id, "route_id": route_id,
                                   "action_type": kind, "poses": route_poses,
                                   "navigate_authorized": navigate_allowed})
            return runtime

        def poll_goals(sim_time: float) -> None:
            for runtime in list(active.values()):
                if runtime.handle is None and runtime.send_future.done():
                    runtime.handle = runtime.send_future.result()
                    if runtime.handle is None or not runtime.handle.accepted:
                        raise RuntimeError(f"Nav2 rejected {runtime.route_id}")
                    goal_uuid = str(UUID(bytes=bytes(runtime.handle.goal_id.uuid)))
                    runtime.evidence.accepted(goal_uuid, sim_time)
                    runtime.result_future = runtime.handle.get_result_async()
                    journal("goal_accepted", {"robot_id": runtime.robot_id, "route_id": runtime.route_id,
                                               "goal_uuid": goal_uuid, "action_type": runtime.evidence.action_type})
                if runtime.result_future is not None and runtime.result_future.done() and runtime.evidence.state in {"active", "cancel_requested"}:
                    terminal = runtime.result_future.result()
                    error_code = int(getattr(terminal.result, "error_code", 0))
                    runtime.evidence.terminal(int(terminal.status), error_code, sim_time)
                    journal("action_terminal", {"robot_id": runtime.robot_id, "route_id": runtime.route_id,
                        "goal_uuid": runtime.evidence.goal_uuid, "status": int(terminal.status),
                        "error_code": error_code, "state": runtime.evidence.state})

        def cancel_goal(runtime: RuntimeGoal) -> None:
            if runtime.handle is None or runtime.evidence.state != "active":
                raise RuntimeError("cancel requires a currently accepted active goal")
            runtime.evidence.request_cancel(now())
            runtime.cancel_future = runtime.handle.cancel_goal_async()
            journal("cancel_requested", {"robot_id": runtime.robot_id, "route_id": runtime.route_id,
                                          "goal_uuid": runtime.evidence.goal_uuid})

        def drain_traffic() -> None:
            nonlocal traffic_cursor
            page = traffic.events_since(traffic_cursor)
            traffic_cursor = page["next_cursor"]
            for event in page["events"]:
                item = {**event, "scenario_id": args.scenario}
                line = json.dumps(item, allow_nan=False)
                traffic_log.write(line + "\n"); traffic_log.flush()
                msg = String(); msg.data = line; event_pub.publish(msg)

        def publish_lease(robot_id: str, sim_time: float) -> None:
            nonlocal authorization_event_seq
            if channel_fault is not None and channel_fault.robot_id == robot_id and not channel_fault.link_up(sim_time):
                return
            runtime = active.get(robot_id)
            mode = "hold"; route_id = goal_uuid = exit_id = None
            reservation_id = recovery_id = None; resources: Sequence[str] = ()
            if (runtime is not None and runtime.navigate_allowed and runtime.evidence.state == "active"
                    and runtime.reservation is not None and runtime.reservation.granted):
                mode = "navigate"; route_id = runtime.route_id; goal_uuid = runtime.evidence.goal_uuid
                reservation_id = runtime.reservation.reservation_id; recovery_id = runtime.recovery_id
                resources = runtime.route["resources"]; exit_id = runtime.route["exit_id"]
            lease_sequence[robot_id] += 1
            lease = make_authorization(
                run_id=args.run_id, scenario_id=args.scenario, robot_id=robot_id,
                sequence=lease_sequence[robot_id], now_s=sim_time,
                ttl_s=float(config["authorization_ttl_s"]), mode=mode,
                decision_event_seq=traffic.event_sequence,
                route_id=route_id, goal_uuid=goal_uuid,
                reservation_id=reservation_id, recovery_id=recovery_id,
                resources=resources, exit_id=exit_id,
            )
            robot_dir = output / "robots" / robot_id; robot_dir.mkdir(parents=True, exist_ok=True)
            atomic_json(robot_dir / "authorization.json", lease)
            msg = String(); msg.data = json.dumps(lease, sort_keys=True, separators=(",", ":")); authorization_pub.publish(msg)
            authorization_event_seq += 1
            record = {"run_id": args.run_id, "scenario_id": args.scenario, "event_seq": authorization_event_seq,
                      "sim_time_s": sim_time, "authorization": lease}
            auth_log.write(json.dumps(record, allow_nan=False) + "\n"); auth_log.flush()

        def tick() -> float:
            nonlocal last_observation, localization_event_seq
            if time.monotonic() >= deadline:
                raise TimeoutError("G2 mission wall timeout")
            if launch.poll() is not None or bag.poll() is not None:
                raise RuntimeError("G2 Nav2 launch or rosbag exited unexpectedly")
            scene_path = output / "g2-scene.json"
            if scene_path.is_file() and json.loads(scene_path.read_text()).get("status") == "failed":
                raise RuntimeError("G2 Isaac scene failed during mission")
            for node in nodes:
                ros.spin_once(node, timeout_sec=0.002)
            if not evidence_subscribers_ready(evidence_publishers):
                raise RuntimeError("rosbag evidence subscriptions were lost before lease publication")
            if not named_subscriber_topics_ready(event_node, recorder_scan_topics, "rosbag2_recorder"):
                raise RuntimeError("rosbag scan subscriptions were lost during scan filtering")
            sim_time = now()
            poll_goals(sim_time)
            if sim_time - last_observation >= 0.1:
                cycle_poses: dict[str, dict[str, float]] = {}
                for robot_id in ROBOT_IDS:
                    try:
                        candidate = localization(robot_id)
                    except Exception:
                        continue
                    if not pose_freshness.accept(
                        robot_id, pose_stamp_s=candidate["stamp_s"], now_s=sim_time,
                    ):
                        continue
                    poses[robot_id] = candidate
                    cycle_poses[robot_id] = candidate
                    if not (channel_fault is not None and channel_fault.robot_id == robot_id
                            and not channel_fault.link_up(sim_time)):
                        projection = project_observation(
                            traffic, (candidate["x_m"], candidate["y_m"]), planning_radius,
                        )
                        traffic.update_observation(
                            robot_id, pose_xy=(candidate["x_m"], candidate["y_m"]),
                            pose_time=candidate["stamp_s"], heartbeat_time=sim_time,
                            **projection, status="navigating" if robot_id in active else "available",
                            motion_allowed=True, now=sim_time,
                        )
                        last_traffic_observation[robot_id] = sim_time
                    localization_event_seq += 1
                    pose_log.write(json.dumps({"run_id": args.run_id, "scenario_id": args.scenario,
                        "event_seq": localization_event_seq, "sim_time_s": sim_time,
                        "robot_id": robot_id, **candidate}, allow_nan=False) + "\n")
                if cycle_poses:
                    pose_log.flush()
                last_observation = sim_time
            stale_robots = pose_freshness.stale(sim_time)
            if navigation_ready and stale_robots:
                raise RuntimeError(f"fresh map->base_link TF lost for: {', '.join(stale_robots)}")
            for robot_id in ROBOT_IDS:
                if sim_time - last_lease[robot_id] >= 1.0 / float(config["authorization_publish_hz"]):
                    publish_lease(robot_id, sim_time); last_lease[robot_id] = sim_time
            traffic.refresh_exit_waits(now=sim_time)
            drain_traffic()
            return sim_time

        def wait_until(predicate: Callable[[], bool], timeout_sim_s: float, label: str) -> float:
            start = now()
            while True:
                sim_time = tick()
                if predicate():
                    return sim_time
                if sim_time - start > timeout_sim_s:
                    raise TimeoutError(f"G2 timeout waiting for {label}")

        def wait_accepted(runtime: RuntimeGoal) -> None:
            wait_until(lambda: runtime.evidence.state in {"active", "succeeded", "canceled"}, 30, runtime.route_id + " acceptance")
            if runtime.evidence.state != "active":
                raise RuntimeError(f"{runtime.route_id} terminated before active observation")

        def wait_success(runtime: RuntimeGoal, timeout_s: float = 240) -> None:
            wait_until(lambda: runtime.evidence.state == "succeeded", timeout_s, runtime.route_id + " success")
            wait_until(lambda: abs(odom_speed[runtime.robot_id]) <= 0.02 and now() - last_moving[runtime.robot_id] >= 0.5,
                       20, runtime.route_id + " stationary hold")

        def request(route_id: str) -> Decision:
            route = scenario["routes"][route_id]
            return traffic.request(route["robot_id"], route["resources"], route["exit_id"], now=now())

        def wait_release(route_id: str) -> Decision:
            route = scenario["routes"][route_id]
            decision: Decision | None = None
            def released() -> bool:
                nonlocal decision
                decision = traffic.release(route["robot_id"], route["resources"], route["exit_id"], now=now())
                return decision.granted
            wait_until(released, 30, route_id + " release ACK")
            assert decision is not None
            return decision

        def drain_terminal_evidence() -> None:
            """Flush release ACKs before, and the completion marker after, terminal success."""
            tick()
            drain_traffic()
            journal("scenario_complete", {"phase": machine.phase})
            start_sim = now()
            wall_deadline = time.monotonic() + 3.0
            ticks = 0
            while ticks < 3 or now() - start_sim < 0.1:
                tick()
                ticks += 1
                if time.monotonic() > wall_deadline:
                    raise TimeoutError("clock did not advance while draining terminal G2 evidence")

        result["stage"] = "startup"
        lifecycle_poller = LifecycleStatePoller({
            f"{robot}/{name}": client
            for robot, robot_clients in lifecycle.items() for name, client in robot_clients.items()
        })
        startup_wall = time.monotonic() + 300
        while True:
            tick()
            lifecycle_states = lifecycle_poller.poll(time.monotonic(), GetState.Request)
            active_states = all(state == 3 for state in lifecycle_states.values())
            graph_ready = (set(poses) == set(ROBOT_IDS) and not pose_freshness.stale(now())
                and all(all(count >= 5 for count in scan_counts[robot].values()) and odom_counts[robot] >= 5 for robot in ROBOT_IDS)
                and all(all(client.server_is_ready() for client in clients[robot].values()) for robot in ROBOT_IDS))
            if active_states and graph_ready and now() > 0:
                break
            if time.monotonic() > startup_wall:
                raise TimeoutError("two Nav2 stacks, TF, odom and four scan streams did not become ready")
        navigation_ready = True
        journal("navigation_ready", {"scan_counts": scan_counts, "odom_counts": odom_counts,
                                      "planning_radius_m": planning_radius,
                                      "scan_epoch_s": scan_epoch_s,
                                      "scan_epoch_ns": scan_epoch_ns,
                                      "scan_pre_record_drops": scan_pre_record_drops})

        result["stage"] = "scenario"
        if args.scenario == "P0":
            machine.transition("observing")
            start = now()
            wait_until(lambda: now() - start >= float(scenario["stationary_observation_s"]),
                       float(scenario["stationary_observation_s"]) + 5, "stationary observation")
            if any(abs(odom_speed[robot]) > 0.02 for robot in ROBOT_IDS):
                raise RuntimeError("P0 robot moved during stationary preflight")
            machine.transition("complete")

        elif args.scenario == "F1":
            c1 = request("carter1_cross")
            if not c1.granted: raise RuntimeError(f"F1 carter1 reservation denied: {c1.reason}")
            machine.transition("c1_reserved")
            c2_wait = request("carter2_cross")
            if c2_wait.granted or c2_wait.reason not in {"occupied", "stale_occupancy", "queued_by_fairness"}:
                raise RuntimeError("F1 carter2 did not produce an observed reservation wait")
            c1_goal = send_route("carter1_cross", c1, navigate_allowed=True); wait_accepted(c1_goal)
            machine.transition("c1_active"); wait_success(c1_goal); wait_release("carter1_cross")
            machine.transition("c1_released")
            c2 = request("carter2_cross")
            if not c2.granted: raise RuntimeError("F1 waiting carter2 was not granted after release")
            c2_goal = send_route("carter2_cross", c2, navigate_allowed=True); wait_accepted(c2_goal)
            machine.transition("c2_active"); wait_success(c2_goal); wait_release("carter2_cross")
            machine.transition("complete")

        elif args.scenario == "F2":
            projection = project_observation(traffic, (poses["carter2"]["x_m"], poses["carter2"]["y_m"]), planning_radius)
            traffic.update_observation("carter2", pose_xy=(poses["carter2"]["x_m"], poses["carter2"]["y_m"]),
                pose_time=poses["carter2"]["stamp_s"], heartbeat_time=now(), **projection, now=now())
            c2 = traffic.claim_observed_occupancy("carter2", "Z_UPPER", "Z_UPPER:east", now=now())
            if not c2.granted: raise RuntimeError(f"F2 observed occupancy claim failed: {c2.reason}")
            machine.transition("c2_claimed")
            if request("carter1_cross").granted: raise RuntimeError("F2 carter1 entered an owned resource")
            c2_goal = send_route("carter2_exit", c2, navigate_allowed=True); wait_accepted(c2_goal)
            machine.transition("c2_active")
            wait_until(lambda: abs(odom_speed["carter2"]) > float(scenario["fault"]["trigger_encoder_speed_m_s"])
                       and "Z_UPPER" in project_observation(traffic, (poses["carter2"]["x_m"], poses["carter2"]["y_m"]), planning_radius)["occupied_resources"],
                       60, "moving owner fault trigger")
            channel_fault = MovingChannelFault("carter2", float(scenario["fault"]["duration_sim_s"]))
            channel_fault.trigger(now())
            if channel_fault.edge(now()) != "link_down": raise RuntimeError("F2 fault did not produce link-down edge")
            journal("fault_started", {**channel_fault.evidence(now()),
                "configured_duration_sim_s": float(scenario["fault"]["duration_sim_s"]),
                "dispatcher_observation": "link_down only", "scope": scenario["fault"]["scope"]})
            cancel_goal(c2_goal); machine.transition("fault_active")
            wait_until(lambda: c2_goal.evidence.state == "canceled", 30, "F2 actual cancel terminal status 5")
            machine.transition("fault_cancelled")
            stale_seen = False
            while not channel_fault.link_up(now()):
                tick()
                denial = request("carter1_cross")
                stale_seen = stale_seen or denial.reason == "stale_occupancy"
            if not stale_seen: raise RuntimeError("F2 did not retain stale ownership and deny carter1")
            if channel_fault.edge(now()) != "link_up": raise RuntimeError("F2 fault did not produce link-up edge")
            journal("fault_ended", {**channel_fault.evidence(now()), "dispatcher_observation": "link_up only"})
            link_up_sim_time = now()
            wait_until(lambda: last_traffic_observation["carter2"] >= link_up_sim_time,
                       2, "fresh carter2 pose and heartbeat after link restoration")
            c2_goal = send_route("carter2_exit", c2, navigate_allowed=True); wait_accepted(c2_goal)
            machine.transition("c2_reissued"); wait_success(c2_goal); wait_release("carter2_exit")
            machine.transition("c2_released")
            c1 = request("carter1_cross")
            if not c1.granted: raise RuntimeError("F2 carter1 not granted after fresh owner release")
            c1_goal = send_route("carter1_cross", c1, navigate_allowed=True); wait_accepted(c1_goal)
            machine.transition("c1_active"); wait_success(c1_goal); wait_release("carter1_cross")
            machine.transition("complete")

        else:  # F3
            c1 = request("carter1_inside")
            if not c1.granted: raise RuntimeError("F3 carter1 reservation denied")
            if request("carter2_blocked").granted: raise RuntimeError("F3 carter2 resource wait was not observed")
            machine.transition("c1_reserved")
            c1_goal = send_route("carter1_inside", c1, navigate_allowed=True); wait_accepted(c1_goal)
            wait_success(c1_goal)
            blocked = send_route("carter2_blocked", None, navigate_allowed=False); wait_accepted(blocked)
            machine.transition("blocked_goal_active")
            recovery: Decision | None = None
            yield_cost = authored_route_length(
                (poses["carter2"]["x_m"], poses["carter2"]["y_m"]),
                scenario["routes"]["carter2_yield"]["poses"],
            )
            journal("recovery_cost", {"robot_id": "carter2", "target": "P2", "cost_m": yield_cost,
                                      "basis": "authored route length, static preflight; not measured Nav2 path cost"})
            def cycle_ready() -> bool:
                nonlocal recovery
                if blocked.evidence.state != "active":
                    raise RuntimeError("F3 blocked goal terminated before persistent cycle cancellation")
                traffic.observe_exit_wait("carter1", "carter2", now=now(), planning_radius_m=planning_radius)
                recovery = traffic.recovery_decision(
                    now=now(), pocket_route_costs={"carter2": {"P1": None, "P2": yield_cost}},
                )
                return recovery is not None
            wait_until(cycle_ready, 30, "observed persistent exit/resource cycle")
            assert recovery is not None
            if not recovery.granted or recovery.robot_id != "carter2" or recovery.command != "cancel_and_yield":
                raise RuntimeError("F3 recovery did not select carter2 yield")
            machine.transition("cycle_observed"); cancel_goal(blocked)
            wait_until(lambda: blocked.evidence.state == "canceled", 30, "F3 blocked goal cancel status 5")
            traffic.cancel_pending("carter2", now=now(), reason="persistent_cycle_recovery")
            machine.transition("blocked_goal_cancelled")
            yielding = request("carter2_yield")
            if not yielding.granted: raise RuntimeError("F3 yield bundle was not granted")
            yield_goal = send_route("carter2_yield", yielding, navigate_allowed=True, recovery_id=recovery.recovery_id)
            wait_accepted(yield_goal); machine.transition("yield_active"); wait_success(yield_goal); wait_release("carter2_yield")
            machine.transition("yield_released")
            held = traffic.resume_after_release_ack("carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
                expected_reservation_id=int(c1.reservation_id), goal_xy=(20, 4), now=now())
            if held.granted or held.reason != "release_ack_required": raise RuntimeError("F3 ACK wait was not registered")
            machine.transition("ack_wait")
            c1_exit = send_route("carter1_exit", c1, navigate_allowed=True); wait_accepted(c1_exit)
            wait_success(c1_exit); wait_release("carter1_exit")
            machine.transition("c1_released")
            resume_ack = traffic.resume_after_release_ack("carter2", owner_robot_id="carter1", exit_id="Z_UPPER:east",
                expected_reservation_id=int(c1.reservation_id), goal_xy=(20, 4), now=now())
            if not resume_ack.granted: raise RuntimeError(f"F3 release ACK not consumed: {resume_ack.reason}")
            resumed = request("carter2_resume")
            if not resumed.granted: raise RuntimeError("F3 resume bundle was not granted")
            resume_goal = send_route("carter2_resume", resumed, navigate_allowed=True, recovery_id=recovery.recovery_id)
            wait_accepted(resume_goal); machine.transition("resume_active")
            wait_success(resume_goal, 300); wait_release("carter2_resume")
            machine.transition("complete")

        drain_terminal_evidence()
        result.update({
            "status": "passed", "stage": "complete", "scenario_phases": machine.history,
            "actions": [asdict(runtime.evidence) for runtime in action_history],
            "scan_counts": scan_counts, "odom_counts": odom_counts,
            "scan_epoch_s": scan_epoch_s, "scan_epoch_ns": scan_epoch_ns,
            "scan_pre_record_drops": scan_pre_record_drops,
            "bag_topics_requested": bag_topics, "planning_radius_m": planning_radius,
            "traffic_snapshot": traffic.snapshot(now=now()),
            "localization_source": "per-namespace TF map->base_link from wheel odometry plus front/rear filtered RTX scan AMCL",
            "localization_scan": {
                "topic": "per-robot scan_localization",
                "inputs": ["front_scan", "rear_scan"],
                "method": "unchanged filtered LaserScan fan-in; AMCL applies each frame extrinsic from TF",
                "ground_truth_input": False,
            },
            "fault_scope": scenario.get("fault"),
        })
        atomic_json(output / "mission-control.json", {
            "run_id": args.run_id, "scenario_id": args.scenario, "status": "passed",
        })
        print(f"FACTORY_OPS_G2_{args.scenario}=PASS", flush=True)
        del tf_listeners, subscriptions
    except BaseException as error:
        result.update({"status": "failed", "stage": result.get("stage", machine.phase),
                       "scenario_phases": machine.history,
                       "error": {"type": type(error).__name__, "message": str(error),
                                 "traceback": traceback.format_exc(limit=30)}})
        atomic_json(output / "mission-control.json", {
            "run_id": args.run_id, "scenario_id": args.scenario,
            "status": "failed", "reason": str(error),
        })
        print(f"FACTORY_OPS_G2_{args.scenario}=FAIL {type(error).__name__}: {error}", flush=True)
    finally:
        for process, label in ((bag, "rosbag"), (launch, "nav2")):
            try:
                cleanup.append(stop_process(process, label))
            except BaseException as error:
                cleanup.append({"label": label, "error": f"{type(error).__name__}: {error}"})
                result["status"] = "failed"
        for node in reversed(nodes):
            try:
                node.destroy_node()
            except BaseException as error:
                cleanup.append({"label": "destroy_node", "error": str(error)})
                result["status"] = "failed"
        if ros is not None:
            try:
                ros.shutdown()
            except BaseException as error:
                cleanup.append({"label": "rclpy.shutdown", "error": str(error)})
                result["status"] = "failed"
        for stream in streams:
            stream.close()
        result["cleanup"] = cleanup
        atomic_json(output / "mission-result.json", result)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
