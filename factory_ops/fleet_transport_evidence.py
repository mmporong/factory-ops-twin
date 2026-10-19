"""Validate closed four-AMR development records without opening bags or using ROS."""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import struct
from typing import Any, Callable, Iterator
from uuid import UUID

from .fleet_execution import StoppedGate
from .fleet_transport import SOURCE_FILES, load_bundle
from .material_evidence import validate_material_evidence
from .navigation import GoalPose
from .source_provenance import verify_source_archive


SCOPE = ("four-AMR development JSON evidence; not full bag/action qualification, "
         "physical loading, charging or policy comparison")
# SDK prepare_worker_paths pins these producers before creating the application.
REQUIRED_SOURCE_FILES = frozenset({
    "factory_ops/fleet_transport.py", "factory_ops/fleet_registry.py",
    "factory_ops/fleet_routing.py", "factory_ops/layout.py", "factory_ops/source_provenance.py",
    "scripts/isaac/fleet4_transport_scene.py", "scripts/isaac/g2_scene.py",
    "scripts/isaac/g3_scene.py", "scripts/isaac/factory_visuals.py",
    "scripts/ros/g2_mission.py", "scripts/ros/g3_mission.py",
    "scripts/ros/fleet4_mission.py", "scripts/ros/fleet4.launch.py",
    "factory_ops/material_flow.py", "factory_ops/operations/inventory.py",
    "factory_ops/energy.py", "factory_ops/energy_runtime.py", "factory_ops/energy_dispatch.py",
}) | {"configs/factory-fleet4/" + name for name in SOURCE_FILES}


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def _number(value: Any) -> float:
    _require(type(value) in (int, float) and math.isfinite(value), "non-finite numeric evidence")
    return float(value)


def _count(value: Any, minimum: int = 1) -> int:
    _require(type(value) is int and value >= minimum, "missing/invalid count")
    return value


def _digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def _regular(path: Path, *, directory: bool = False) -> Path:
    candidate = path.absolute()
    _require(not any(p.is_symlink() or p.is_junction() for p in (candidate, *candidate.parents)),
             f"symlink/junction path rejected: {path}")
    _require(candidate.is_dir() if directory else candidate.is_file(), f"missing regular path: {path}")
    return candidate.resolve(strict=True)


def _relative(root: Path, name: str) -> Path:
    _require(isinstance(name, str) and "\\" not in name and ":" not in name, "invalid record path")
    relative = PurePosixPath(name)
    _require(not relative.is_absolute() and bool(relative.parts)
             and all(p not in (".", "..") for p in relative.parts)
             and relative.as_posix() == name, "record path must be canonical and relative")
    return _regular(root.joinpath(*relative.parts))


def _object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        _require(key not in result, f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> Any:
    raise ValueError(f"non-finite JSON constant: {value}")


def _decode(value: str) -> dict[str, Any]:
    row = json.loads(value, object_pairs_hook=_object, parse_constant=_reject_constant)
    _require(isinstance(row, dict), "JSON record must be an object")
    return row


class _Records:
    def __init__(self) -> None:
        self.files: dict[str, str] = {}

    def read(self, path: Path) -> dict[str, Any]:
        path = _regular(path)
        data = path.read_bytes()
        self.files[str(path)] = hashlib.sha256(data).hexdigest()
        return _decode(data.decode("utf-8"))

    def lines(self, path: Path) -> Iterator[dict[str, Any]]:
        path = _regular(path)
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for line in stream:
                digest.update(line)
                _require(bool(line.strip()), f"empty JSONL row: {path}")
                yield _decode(line.decode("utf-8"))
        self.files[str(path)] = digest.hexdigest()


def _identity(row: dict[str, Any], run_id: str, scenario: str) -> None:
    _require(row.get("run_id") == run_id and row.get("scenario_id") == scenario,
             "run/scenario identity differs")


def _utc(value: str) -> datetime:
    stamp = datetime.fromisoformat(value.replace("Z", "+00:00"))
    _require(stamp.utcoffset() is not None, "closure timestamp requires timezone")
    return stamp


def _closure(records: _Records, stage: Path, run_id: str, source: str) -> dict[str, Any]:
    closure = records.read(stage / "closure-confirmation.json")
    runner = records.read(stage / "runner-exit.json")
    _require(closure.get("passed") is True and closure.get("run_id") == run_id
             and closure.get("run_label") == stage.name
             and re.fullmatch(r"fleet4-transport-normal-[0-9]{3}", stage.name) is not None
             and closure.get("remaining_owned_containers") == [], "fresh closed-run identity differs")
    _require(runner.get("run_id") == run_id and runner.get("source_commit") == source,
             "runner run/source differs")
    _require(runner.get("cleanup_passed") is True and runner.get("cleanup_errors") == []
             and type(runner.get("runner_exit_code")) is int and runner["runner_exit_code"] == 0
             and type(runner.get("original_runner_exit_code")) is int
             and runner["original_runner_exit_code"] == 0, "runner exit/cleanup failed")
    ids = runner["container_ids"]
    _require(set(ids) == {"nav", "sim"} and all(isinstance(v, str) and v for v in ids.values())
             and len(set(ids.values())) == 2, "two owned worker IDs required")
    for role in ("nav", "sim"):
        state = runner[role]
        _require(state.get("Running") is False and type(state.get("ExitCode")) is int
                 and state["ExitCode"] == 0 and not state.get("OOMKilled") and not state.get("Error"),
                 f"{role} worker did not exit successfully")
    for observed in closure["observed_containers"]:
        role = observed["name"].rsplit("-", 1)[-1]
        _require(role in ids and observed["id"] == ids[role]
                 and observed["name"] == f"/factory-ops-{stage.name}-{role}"
                 and observed["state"].get("Running") is False, "closure worker identity/state differs")
    _require(_utc(closure["observed_at_utc"]) >= _utc(runner["finished_at_utc"]),
             "closure predates runner exit")
    return {"run_label": stage.name, "workers": ids, "observed_at_utc": closure["observed_at_utc"]}


def _source(records: _Records, output: Path, bundle: Path, compiled: dict[str, Any],
            run_id: str, source: str) -> dict[str, Any]:
    metadata = records.read(output / "provenance/fleet-source.json")
    marker = records.read(output / "fleet-transport-verified.json")
    scenario = compiled["session"]["scenario_id"]
    for row in (metadata, marker):
        _identity(row, run_id, scenario)
        _require(row.get("source_commit") == source
                 and row.get("bundle_sha256") == compiled["bundle_sha256"], "source/bundle binding differs")
    registry = compiled["registry"]
    _require(marker.get("status") == "verified" and marker.get("profile") == "development"
             and marker.get("primary_evaluation") is False
             and marker.get("scope") == compiled["session"]["scope"]
             and marker.get("robot_ids") == list(registry.robot_ids)
             and marker.get("robots") == {r["robot_id"]: {k: r[k] for k in ("namespace", "prim_path")}
                                          for r in registry.definitions}, "transport marker differs")
    pins = metadata["source_files"]
    _require(isinstance(pins, dict) and REQUIRED_SOURCE_FILES.issubset(pins), "required source pins missing")
    for name in SOURCE_FILES:
        _require(_digest(_relative(bundle, name)) == pins["configs/factory-fleet4/" + name],
                 f"input config differs from source archive: {name}")
    for name in ("g3_mission.py", "fleet4_mission.py", "fleet4.launch.py"):
        _require(marker["source_files"].get(name) == pins["scripts/ros/" + name], "ROS source marker differs")
    archive = _relative(output, "provenance/source.tar")
    commit = _relative(output, "provenance/source-commit.raw")
    proof = verify_source_archive(archive, commit, source, pins)
    _require(proof.get("passed") is True, f"source archive proof failed: {proof.get('errors')}")
    _require(proof["source_archive_sha256"] == metadata["source_archive_sha256"], "source archive digest differs")
    records.files[str(archive)] = _digest(archive)
    records.files[str(commit)] = _digest(commit)
    return {"proof": proof, "pins": pins}


def _input(row: dict[str, Any], name: str, expected: str) -> None:
    _require(row["inputs"][name]["sha256"] == expected, f"producer input differs: {name}")


def _terminal(sdk: dict[str, Any], ros: dict[str, Any]) -> None:
    timing, boundary = sdk["timing"], ros["terminal_recording"]
    stamp = timing["final_stamp"]
    seconds, nanos = _count(stamp["sec"], 0), _count(stamp["nanosec"], 0)
    _require(nanos < 1_000_000_000, "noncanonical final ROS stamp")
    final_ns = seconds * 1_000_000_000 + nanos
    _require(abs(_number(timing["final_sim_time_s"]) - final_ns / 1e9) <= 1e-9
             and boundary["final_stamp"] == stamp and boundary["final_sim_time_ns"] == final_ns
             and _count(boundary["final_physics_frame"]) == _count(timing["final_physics_frame"])
             and _count(boundary["minimum_sim_time_ns"], 0) <= final_ns
             and set(boundary["received_odometry_ns"]) == set(sdk["robots"])
             and all(_count(v, 0) >= final_ns for v in boundary["received_odometry_ns"].values()),
             "SDK/ROS terminal recording boundary differs")


def _producers(records: _Records, output: Path, bundle: Path, compiled: dict[str, Any],
               pins: dict[str, str], run_id: str) -> tuple[dict[str, Any], dict[str, Any]]:
    sdk = records.read(output / "fleet4-transport-scene.json")
    ros = records.read(output / "mission-result.json")
    control = records.read(output / "mission-control.json")
    for row in (sdk, ros, control):
        _identity(row, run_id, compiled["session"]["scenario_id"])
    _require(sdk.get("status") == "validated" and sdk.get("cleanup") == "worker_exit_requested"
             and not sdk.get("cleanup_errors") and not sdk.get("close_error"), "SDK validation/cleanup failed")
    _require(ros.get("status") == "passed" and ros.get("stage") == "complete"
             and ros.get("trial_outcome") == "completed" and control.get("status") == "passed",
             "ROS mission/control incomplete")
    _require(isinstance(ros.get("cleanup"), list)
             and all(isinstance(r, dict) and not r.get("error") for r in ros["cleanup"]), "ROS cleanup error")
    _require(sdk.get("mission_control") == control, "SDK terminal mission control differs")
    _require(sdk.get("qualification") == compiled["session"]["scope"]
             and ros.get("scope") == compiled["session"]["scope"]
             and ros.get("primary_evaluation") is False and ros.get("ground_truth_input") is False,
             "development scope/ground-truth boundary differs")
    robots = set(compiled["registry"].robot_ids)
    _require(set(sdk["robots"]) == robots and ros.get("robot_ids") == list(compiled["registry"].robot_ids)
             and ros.get("namespace_by_id") == {r["robot_id"]: r["namespace"] for r in compiled["registry"].definitions},
             "four participating robot identities differ")
    for definition in compiled["registry"].definitions:
        robot = sdk["robots"][definition["robot_id"]]
        _require(all(robot.get(k) == definition[k] for k in ("namespace", "prim_path")), "SDK robot identity differs")
    for name, relative in (("scene", "scene/factory-v3.usda"), ("layout", "layout.json"), ("config", "config.json"),
                           ("scenarios", "scenarios.json"), ("fleet_bundle", "bundle.json")):
        _input(sdk, name, _digest(_relative(bundle, relative)))
    _input(sdk, "source", pins["scripts/isaac/g2_scene.py"])
    _input(sdk, "fleet_worker_source", pins["scripts/isaac/fleet4_transport_scene.py"])
    for name, relative in (("session", "session.json"), ("config", "config.json"), ("nav2_template", "nav2-g2.yaml")):
        _input(ros, name, _digest(_relative(bundle, relative)))
    _input(ros, "mission_source", pins["scripts/ros/g3_mission.py"])
    _input(ros, "fleet_transport_verification", _digest(output / "fleet-transport-verified.json"))
    _require(ros.get("bundle_sha256") == compiled["bundle_sha256"]
             and ros.get("session_sha256") == _digest(bundle / "session.json")
             and ros.get("workload_sha256") == compiled["session"]["workload_sha256"], "ROS input identity differs")
    _terminal(sdk, ros)
    return sdk, ros


def _trajectory(records: _Records, output: Path, robot: str, sdk: dict[str, Any],
                runtime: dict[str, Any]) -> dict[str, Any]:
    definition = sdk["robots"][robot]
    name = f"robots/{robot}/trajectory.jsonl"
    _require(definition.get("trajectory") == name, "trajectory robot path differs")
    last_time, last_frame = -1.0, -1
    first = previous = final = None
    distance = 0.0
    count = 0
    for row in records.lines(_relative(output, name)):
        stamp, frame = _number(row["sim_time_s"]), _count(row["physics_frame"], 0)
        _require(stamp > last_time and frame > last_frame, "trajectory time/frame did not advance")
        position = row["ground_truth"]["position_xyz_m"]
        _require(isinstance(position, list) and len(position) == 3, "ground-truth position missing")
        point = tuple(_number(v) for v in position)
        odom = row["odometry"]
        for key in ("x_m", "y_m", "yaw_rad", "linear_m_s", "angular_rad_s"):
            _number(odom[key])
        if previous is not None:
            distance += math.dist(previous[:2], point[:2])
        else:
            first = point
        previous, final = point, odom
        last_time, last_frame, count = stamp, frame, count + 1
    _require(count >= 2 and count == definition["trajectory_samples"] and distance > 0,
             f"{robot} has no sampled physical XY motion or sample count differs")
    _require(abs(last_time - sdk["timing"]["final_sim_time_s"]) <= 1e-9
             and last_frame == sdk["timing"]["final_physics_frame"], "trajectory does not reach sealed terminal frame")
    assert final is not None and first is not None and previous is not None
    _require(abs(final["linear_m_s"]) <= runtime["stopped_linear_m_s"]
             and abs(final["angular_rad_s"]) <= runtime["stopped_angular_rad_s"], "final encoder velocity is not stopped")
    return {"samples": count, "sampled_xy_distance_m": distance, "start_xyz_m": first,
            "final_xyz_m": previous, "final_sim_time_s": last_time}


def _stop(action: dict[str, Any], runtime: dict[str, Any]) -> None:
    samples = action["stopped_odometry"]
    _require(isinstance(samples, list) and len(samples) >= 2, "stop samples missing")
    gate = StoppedGate(window_s=runtime["stopped_window_s"], max_linear_m_s=runtime["stopped_linear_m_s"],
                       max_angular_rad_s=runtime["stopped_angular_rad_s"], ttl_s=runtime["odom_ttl_s"])
    previous = -1.0
    for sample in samples:
        stamp = _number(sample["stamp_s"])
        _require(stamp > previous and abs(_number(sample["linear_m_s"])) <= gate.max_linear_m_s
                 and abs(_number(sample["angular_rad_s"])) <= gate.max_angular_rad_s, "invalid stationary sample")
        gate.update(sample["linear_m_s"], sample["angular_rad_s"], stamp_s=stamp, now_s=stamp)
        previous = stamp
    _require(action["stop_measurement_start_stamp_s"] == samples[0]["stamp_s"]
             and samples[0]["stamp_s"] >= _number(action["terminal_sim_time_s"])
             and gate.is_stopped(now_s=_number(action["stopped_sim_time_s"])), "continuous post-action stop did not qualify")


def _arrival(pose: dict[str, Any], x: float, y: float, yaw: float, runtime: dict[str, Any]) -> None:
    distance = math.hypot(_number(pose["x_m"]) - x, _number(pose["y_m"]) - y)
    angle = abs(math.atan2(math.sin(_number(pose["yaw_rad"]) - yaw), math.cos(pose["yaw_rad"] - yaw)))
    _require(distance <= runtime["station_xy_tolerance_m"] and angle <= runtime["station_yaw_tolerance_rad"],
             "estimated arrival is outside configured tolerance")


def _actions(ros: dict[str, Any], compiled: dict[str, Any], final_time: float) -> dict[str, Any]:
    runtime, registry = compiled["config"]["g3_runtime"], compiled["registry"]
    parking = compiled["session"]["workload"]["initial_robots"]
    _require(ros.get("physical_anchor_by_robot") == parking, "final parking anchors differ")
    counts = {robot: 0 for robot in registry.robot_ids}
    last: dict[str, Any] = {}
    handoffs: dict[str, dict[str, Any]] = {}
    goals: set[str] = set()
    for action in ros["actions"]:
        robot = action["robot_id"]
        _require(robot in counts and type(action.get("status")) is int and action["status"] == 4
                 and type(action.get("error_code")) is int and action["error_code"] == 0, "FollowPath did not succeed")
        goal = str(UUID(action["follow_action_uuid"]))
        _require(goal not in goals, "FollowPath goal reused")
        goals.add(goal)
        _require(0 <= _number(action["follow_goal_sent_sim_time_s"]) <= _number(action["terminal_sim_time_s"])
                 <= _number(action["stopped_sim_time_s"]) <= final_time,
                 "FollowPath terminal precedes send")
        if robot in last:
            _require(action["follow_goal_sent_sim_time_s"] >= last[robot]["stopped_sim_time_s"],
                     "robot action order overlaps its previous stop")
        _stop(action, runtime)
        target = action["chunk"]["goal_pose"]
        _arrival(action["arrival_pose"], target["x"], target["y"], target["yaw"], runtime)
        counts[robot] += 1
        last[robot] = action
        if "receipt" in action:
            receipt, result = action["receipt"], action["navigation_result"]
            _require(receipt.get("accepted") is True and receipt.get("duplicate") is False
                     and receipt["result_id"] == result["result_id"] and receipt["goal_id"] == result["goal_id"] == goal
                     and result["status"] == 4 and result["station"] == action["leg_target"], "handoff result/receipt differs")
            _require(action["stopped_sim_time_s"] <= _number(action["handoff_sim_time_s"]) <= final_time
                     and abs(action["handoff_sim_time_s"] - _number(result["received_sim_time"])
                             - _number(ros["origin_sim_time_s"])) <= 1e-9, "handoff clock binding differs")
            _require(receipt["result_id"] not in handoffs, "handoff result reused")
            handoffs[receipt["result_id"]] = {"robot_id": robot, "error_code": 0, "result": result, "receipt": receipt}
    for robot, count in counts.items():
        _require(count > 0 and last[robot]["leg_target"] == parking[robot]
                 and last[robot]["chunk"]["goal"] == parking[robot], f"{robot} has no successful final parking action")
        x, y = compiled["book"].nodes[parking[robot]]
        _arrival(last[robot]["arrival_pose"], x, y, runtime["anchor_yaw_rad"][parking[robot]], runtime)
    return {"follow_path_counts": counts, "handoffs": handoffs}


def _orders(ros: dict[str, Any], compiled: dict[str, Any], run_id: str, handoffs: dict[str, Any]) -> dict[str, Any]:
    mission = ros["mission"]
    snapshot = mission["snapshot"]
    declarations = compiled["session"]["workload"]["orders"]
    _require(mission.get("completed") is True and mission.get("failed") is False and mission.get("censored") is False
             and mission.get("replay_matches") is True and not mission.get("active_intents")
             and snapshot.get("run_id") == run_id, "ledger mission incomplete/foreign")
    _require(mission["completed_order_count"] == mission["expected_order_count"] == len(declarations)
             and set(snapshot["orders"]) == {row["order_id"] for row in declarations}, "declared orders differ")
    _require(set(snapshot["robots"]) == set(compiled["registry"].robot_ids), "ledger robot identities differ")
    accepted: dict[str, Any] = {}
    transitions: set[tuple[str, str]] = set()
    for entry in mission["navigation_journal"]:
        if entry["kind"] != "result_accepted":
            continue
        key = entry["result"]["result_id"]
        expected = {k: entry[k] for k in ("robot_id", "error_code", "result", "receipt")}
        _require(key in handoffs and key not in accepted and handoffs[key] == expected, "accepted ledger handoff differs")
        accepted[key] = expected
        task = snapshot["tasks"][entry["result"]["task_id"]]
        _require(task["robot_id"] == entry["robot_id"] and task["status"] == "complete"
                 and entry["result"]["attempt"] == task["attempt"], "handoff task differs")
        transition = entry["receipt"]["committed_transition"]
        _require(transition in ("pickup", "drop") and entry["result"]["station"] == task[transition + "_station"],
                 "handoff station/transition differs")
        transitions.add((task["task_id"], transition))
    business_handoffs = {key: value for key, value in handoffs.items()
                         if value["result"]["task_id"] in snapshot["tasks"]}
    _require(set(accepted) == set(business_handoffs), "action handoff missing from accepted ledger journal")
    empty_ids = {"empty:" + row["order_id"] for row in compiled["session"]["workload"].get("material_flow", {}).get("materials", [])}
    _require(all(value["result"]["task_id"] in snapshot["tasks"] or value["result"]["task_id"] in empty_ids
                 for value in handoffs.values()), "action handoff has an undeclared task")
    participating = {entry["robot_id"] for entry in accepted.values()}
    _require(participating == set(compiled["registry"].robot_ids),
             "all four robots must participate in accepted business handoffs; parking alone is insufficient")
    for declared in declarations:
        order = snapshot["orders"][declared["order_id"]]
        _require(all(order.get(k) == declared[k] for k in ("order_id", "payload_id", "source_station", "ready_time"))
                 and order.get("state") == "complete" and order.get("owner") == "complete", "order not logically shipped")
        tasks = [t for t in snapshot["tasks"].values() if t["order_id"] == declared["order_id"]]
        _require(len(tasks) == 2 and {t["stage"] for t in tasks} == {"inbound", "outbound"}, "order transport stages missing")
        for task in tasks:
            _require(task["payload_id"] == declared["payload_id"] and task["status"] == "complete"
                     and all((task["task_id"], t) in transitions for t in ("pickup", "drop")), "order accepted handoffs missing")
    return {"completed_orders": len(declarations), "accepted_handoffs": len(accepted),
            "business_participants": sorted(participating), "shipment_kind": "logical ledger complete"}


def _lidars(records: _Records, output: Path, compiled: dict[str, Any], sdk: dict[str, Any],
            ros: dict[str, Any], run_id: str) -> dict[str, Any]:
    registry = compiled["registry"]
    expected = {(r["robot_id"], r["name"]): r["prim_path"] for r in registry.manifest()["lidars"]}
    seen: dict[tuple[str, str], list[tuple[int, int]]] = {key: [] for key in expected}
    sequence = -1
    for row in records.lines(_relative(output, "lidar-acquisition-events.jsonl")):
        _identity(row, run_id, compiled["session"]["scenario_id"])
        key = (row["robot_id"], row["side"])
        _require(key in expected and row["prim_path"] == expected[key]
                 and row.get("annotator") == "generic-model-output"
                 and row.get("registration_status") == "attached", "LiDAR identity differs")
        _require(0 <= _number(row["sim_time_s"]) <= sdk["timing"]["final_sim_time_s"] + 1e-9
                 and _count(row["physics_frame"], 0) <= sdk["timing"]["final_physics_frame"],
                 "LiDAR journal exceeds sealed terminal frame")
        _require(_count(row["sequence"], 0) > sequence, "LiDAR journal sequence did not advance")
        sequence = row["sequence"]
        _require(row.get("status") in ("warmup_none", "gmo_header"), "LiDAR acquisition error")
        if row["status"] == "gmo_header":
            gmo = row["gmo"]
            _require(row.get("registration_status") == "attached", "LiDAR registration failed")
            if _count(gmo["num_elements"], 0) == 0:
                continue
            current = (_count(gmo["frame_id"], 0), _count(gmo["timestamp_ns"], 0))
            if seen[key]:
                _require(all(a >= b for a, b in zip(current, seen[key][-1])), "LiDAR GMO regressed")
            seen[key].append(current)
    _require(set(ros["scan_counts"]) == set(registry.robot_ids)
             and set(ros["odom_counts"]) == set(registry.robot_ids), "ROS sensor robot identities differ")
    for (robot, side), samples in seen.items():
        _require(len(samples) >= 2 and all(a > b for a, b in zip(samples[-1], samples[0])), "LiDAR GMO did not advance")
        for name in (side + "_raw", side, "localization_" + side):
            _count(ros["scan_counts"][robot][name], 5)
        _count(ros["odom_counts"][robot], 5)
        _count(sdk["robots"][robot]["ros"]["scan_" + side + "_raw"])
    return {"sensor_count": len(seen), "gmo_samples": {r + "/" + s: len(v) for (r, s), v in seen.items()}}


def _media(records: _Records, output: Path, sdk: dict[str, Any]) -> dict[str, Any]:
    _require(sdk["media"]["index"] == "media/frames.json" and sdk["media"]["same_frame_sources"] == 3,
             "RGB index contract differs")
    index = records.read(_relative(output, "media/frames.json"))
    frames = index["frames"]
    _require(isinstance(frames, list) and len(frames) > 0 and len(frames) == sdk["media"]["frame_count"], "RGB frames missing")
    last_frame, last_time = -1, -1.0
    paths: set[Path] = set()
    for ordinal, frame in enumerate(frames):
        _require(frame["index"] == ordinal and _count(frame["physics_frame"], 0) > last_frame
                 and _number(frame["sim_time_s"]) > last_time, "RGB frame sequence differs")
        last_frame, last_time = frame["physics_frame"], frame["sim_time_s"]
        _require(last_frame <= sdk["timing"]["final_physics_frame"]
                 and last_time <= sdk["timing"]["final_sim_time_s"] + 1e-9, "RGB exceeds sealed terminal frame")
        _require(set(frame["sources"]) == {"overview", "carter1", "carter2"}
                 and set(frame["ground_truth"]) == set(sdk["robots"]), "same-frame RGB sources/robot identities differ")
        for name, record in frame["sources"].items():
            _require(record["path"] == f"media/{name}-{ordinal:05d}.png", "RGB source path differs")
            path = _relative(output, record["path"])
            digest = _digest(path)
            _require(path not in paths and digest == record["sha256"] and path.stat().st_size == record["bytes"], "RGB bytes/hash differs")
            with path.open("rb") as stream:
                header = stream.read(24)
            _require(len(header) == 24 and header[:8] == b"\x89PNG\r\n\x1a\n" and header[12:16] == b"IHDR"
                     and struct.unpack(">II", header[16:24]) == (record["width"], record["height"])
                     and record["width"] > 0 and record["height"] > 0, "RGB PNG header/dimensions differ")
            paths.add(path)
            records.files[str(path)] = digest
    return {"frames": len(frames), "hashed_png_files": len(paths), "sources_per_frame": 3}


def validate_closed_evidence(*, bundle: Path, stage: Path, expected_run_id: str,
                             expected_source_commit: str) -> dict[str, Any]:
    """Return a bounded, fail-closed report; never mutate the supplied evidence."""
    records = _Records()
    report: dict[str, Any] = {"schema_version": 1, "scope": SCOPE, "passed": False,
        "run_id": expected_run_id, "source_commit": expected_source_commit, "checks": {}, "errors": [],
        "limitations": ["No bag, action transport, TF or collision qualification is performed.",
            "Material checks reconcile operational receipts; payload mass, contact and lifting are unverified.",
            "Motion requires positive sampled ground-truth XY distance; no minimum-distance criterion is authored.",
            "Parking uses estimated arrival tolerances; ground-truth docking accuracy is not qualified.",
            "Closure is a recorded producer assertion, not a new VM/container observation.",
            "RGB checks cover indexed PNG bytes/headers, not visual quality or physical loading."]}
    def check(name: str, operation: Callable[[], Any]) -> Any:
        try:
            result = operation()
            report["checks"][name] = {"passed": True, "details": result}
            return result
        except (OSError, ValueError, TypeError, KeyError, IndexError, AttributeError, OverflowError) as error:
            report["checks"][name] = {"passed": False}
            report["errors"].append({"check": name, "reason": f"{type(error).__name__}: {error}"})
            return None
    def paths() -> tuple[Path, Path]:
        _require(re.fullmatch(r"[0-9a-f]{32}", expected_run_id) is not None
                 and re.fullmatch(r"[0-9a-f]{40}", expected_source_commit) is not None, "expected identity format differs")
        return _regular(bundle, directory=True), _regular(stage, directory=True)
    resolved = check("paths", paths)
    if resolved is not None:
        bundle, stage = resolved
        report["checks"]["paths"]["details"] = {"bundle": str(bundle), "stage": str(stage)}
        closed = check("closure", lambda: _closure(records, stage, expected_run_id, expected_source_commit))
        compiled = check("bundle", lambda: load_bundle(bundle)) if closed is not None else None
        # Compiled objects are not JSON data; retain only their verified identity in the report.
        if compiled is not None:
            report["checks"]["bundle"]["details"] = {"bundle_sha256": compiled["bundle_sha256"]}
            output = stage / "results/run"
            source = check("source", lambda: _source(records, output, bundle, compiled, expected_run_id, expected_source_commit))
            if source is not None:
                producer = check("producers", lambda: _producers(records, output, bundle, compiled, source["pins"], expected_run_id))
                if producer is not None:
                    sdk, ros = producer
                    report["checks"]["producers"]["details"] = {"sdk": sdk["status"], "ros": ros["status"]}
                    for robot in compiled["registry"].robot_ids:
                        check("trajectory/" + robot, lambda r=robot: _trajectory(records, output, r, sdk, compiled["config"]["g3_runtime"]))
                    actions = check("actions", lambda: _actions(ros, compiled, sdk["timing"]["final_sim_time_s"]))
                    if actions is not None:
                        check("orders", lambda: _orders(ros, compiled, expected_run_id, actions["handoffs"]))
                        material_profile = compiled["session"]["workload"].get("material_flow")
                        if material_profile is not None:
                            station_goals = {name: GoalPose("map", compiled["book"].nodes[name][0], compiled["book"].nodes[name][1], yaw)
                                for name, yaw in compiled["config"]["g3_runtime"]["anchor_yaw_rad"].items()}
                            check("material", lambda: validate_material_evidence(ros, material_profile,
                                expected_run_id, actions["handoffs"], station_goals))
                        report["checks"]["actions"]["details"] = {"follow_path_counts": actions["follow_path_counts"]}
                    check("lidars", lambda: _lidars(records, output, compiled, sdk, ros, expected_run_id))
                    check("media", lambda: _media(records, output, sdk))
    report["evidence_sha256"] = records.files
    report["passed"] = not report["errors"]
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("bundle", "stage", "report"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--expected-run-id", required=True)
    parser.add_argument("--expected-source-commit", required=True)
    args = parser.parse_args(argv)
    target = args.report.absolute()
    try:
        target = _regular(target.parent, directory=True) / target.name
        _require(not target.exists() and not target.is_symlink(), "report path must be new")
        for original in (args.bundle.resolve(), args.stage.resolve()):
            _require(not target.is_relative_to(original), "report must be outside original bundle/stage")
        report = validate_closed_evidence(bundle=args.bundle, stage=args.stage,
            expected_run_id=args.expected_run_id, expected_source_commit=args.expected_source_commit)
        serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        with target.open("x", encoding="utf-8") as stream:
            stream.write(serialized)
    except (OSError, ValueError) as error:
        print(json.dumps({"passed": False, "error": str(error), "scope": SCOPE}))
        return 1
    print(json.dumps({"passed": report["passed"], "report": str(target), "scope": SCOPE}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
