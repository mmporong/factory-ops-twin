"""Real Jazzy odometry-owner dispatch probe under bounded authored IO load.

This compares one isolated four-robot topology with odometry owned by the IO
node against the production priority node.  The 30 ms queued callbacks are an
authored contention fixture, not a claim about native callback execution time.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback
import uuid

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from factory_ops.callback_drain import CallbackDrain
from factory_ops.energy_runtime import EnergyRuntime
from factory_ops.fleet_execution import StoppedGate
from scripts.ros.g3_mission import (
    StopOdometryReceiver,
    committed_stopped_odometry_record,
    create_robot_dispatch_nodes,
    mission_odometry_qos,
)


ROBOT_IDS = ("carter1", "carter2", "carter3", "carter4")
SIM_EPOCH_S = 1000.0
POST_HOLD_STREAM_S = 1.05
QUEUED_WORK_PER_ROBOT = 16


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=195)
    parser.add_argument("--wall-limit-s", type=float, default=90.0)
    args = parser.parse_args()
    if args.domain_id != 195 or not 20.0 <= args.wall_limit_s <= 90.0:
        parser.error("this isolated probe requires domain 195 and a 20..90 second wall bound")

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException, TransformListener

    run_id = uuid.uuid4().hex
    started = time.monotonic()
    deadline = started + args.wall_limit_s
    profile_path = Path(__file__).resolve().parents[2] / "configs/factory-fleet4/energy-profile.json"
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    runtime_path = (Path(__file__).resolve().parents[2]
                    / "configs/factory-semiconductor-dual-lane/g3-runtime.json")
    runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    stop_window_s = float(runtime["stopped_window_s"])
    stopped_linear_m_s = float(runtime["stopped_linear_m_s"])
    stopped_angular_rad_s = float(runtime["stopped_angular_rad_s"])
    odom_ttl_s = float(runtime["odom_ttl_s"])
    tf_ttl_s = float(runtime["pose_ttl_s"])
    callback_max_rounds = int(runtime["callback_drain"]["max_rounds"])
    callback_wall_budget_s = float(runtime["callback_drain"]["wall_budget_s"])
    report = {
        "task": "odometry-owner-dispatch-probe",
        "run_id": run_id,
        "pid": os.getpid(),
        "domain_id": args.domain_id,
        "status": "running",
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_files": {
            "probe": str(Path(__file__).resolve()),
            "g3_mission_sha256": hashlib.sha256(
                (Path(__file__).with_name("g3_mission.py")).read_bytes()).hexdigest(),
            "callback_drain_sha256": hashlib.sha256(
                (Path(__file__).resolve().parents[2] / "factory_ops/callback_drain.py").read_bytes()).hexdigest(),
            "runtime_config": str(runtime_path),
            "runtime_config_sha256": hashlib.sha256(runtime_path.read_bytes()).hexdigest(),
        },
        "fixture_scope": (
            "actual Jazzy rclpy/DDS topology under authored finite 30 ms queued work; "
            "not native callback timing and not a complete073 replay"
        ),
        "limits": {
            "callback_max_rounds": callback_max_rounds,
            "callback_wall_budget_s": callback_wall_budget_s,
            "priority_prefix_fraction": 0.5,
            "consumer_pause_s": 0.65,
            "post_hold_stream_s": POST_HOLD_STREAM_S,
            "odom_ttl_s": odom_ttl_s,
            "stop_window_s": stop_window_s,
            "stopped_linear_m_s": stopped_linear_m_s,
            "stopped_angular_rad_s": stopped_angular_rad_s,
            "tf_ttl_s": tf_ttl_s,
            "queued_work_s": 0.03,
            "queued_work_per_robot": QUEUED_WORK_PER_ROBOT,
        },
        "rates_hz": {"clock": 60.0, "tf": 20.0, "odom": 20.0, "scan_front": 20.0, "scan_rear": 20.0},
        "qos": {"odom": "RELIABLE", "odom_depth": 50},
        "cases": [],
        "cleanup": {},
    }

    def bounded() -> None:
        if time.monotonic() > deadline:
            raise TimeoutError("probe wall deadline exceeded")

    def trial(owner_lane: str) -> dict:
        context = Context()
        nodes: list = []
        executors: list[tuple[object, object]] = []
        listeners: list = []
        worker = None
        stop = threading.Event()
        lock = threading.Lock()
        publisher_errors: list[str] = []
        case = {"odom_owner": owner_lane, "status": "running", "cleanup_errors": []}
        report["cases"].append(case)
        try:
            rclpy.init(context=context, domain_id=args.domain_id)
            case["actual_domain_id"] = context.get_domain_id()
            namespace_root = f"dispatch_probe_{run_id[:10]}_{owner_lane}"

            class OwnedRos:
                @staticmethod
                def create_node(name, **kwargs):
                    return rclpy.create_node(name, context=context, **kwargs)

            robots = {}
            critical_executors = []
            io_executors = []
            energy = EnergyRuntime(run_id + "_" + owner_lane, profile, {robot: 0.8 for robot in ROBOT_IDS})
            for robot in ROBOT_IDS:
                namespace = f"{namespace_root}/{robot}"
                node, io_node, critical, io_executor = create_robot_dispatch_nodes(
                    robot, namespace, isolated=True, ros=OwnedRos, parameter_type=Parameter,
                    executor_type=lambda: SingleThreadedExecutor(context=context),
                    owned_nodes=nodes, owned_executors=executors,
                )
                critical_executors.append(critical)
                io_executors.append(io_executor)
                buffer = Buffer(node=node)
                listener = TransformListener(buffer, node)
                listeners.append(listener)
                robots[robot] = {
                    "node": node,
                    "io_node": io_node,
                    "buffer": buffer,
                    "receiver": StopOdometryReceiver(StoppedGate(
                        window_s=stop_window_s, max_linear_m_s=stopped_linear_m_s,
                        max_angular_rad_s=stopped_angular_rad_s, ttl_s=odom_ttl_s)),
                    "stop_after_s": None,
                    "odom_count": 0,
                    "last_odom_stamp_s": None,
                    "scan_counts": {"front": 0, "rear": 0},
                    "work_callbacks": 0,
                }

            drain = CallbackDrain(io_executors, priority_executors=critical_executors,
                                  max_rounds=callback_max_rounds,
                                  wall_budget_s=callback_wall_budget_s)
            publishers = []
            for robot in ROBOT_IDS:
                state = robots[robot]
                owner = state["io_node"] if owner_lane == "io_node" else state["node"]

                def odometry(message, robot_id=robot, clock_node=state["node"]):
                    current = robots[robot_id]
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                    now_s = clock_node.get_clock().now().nanoseconds / 1e9
                    current["odom_count"] += 1
                    current["last_odom_stamp_s"] = stamp
                    linear_m_s = float(message.twist.twist.linear.x)
                    angular_rad_s = float(message.twist.twist.angular.z)
                    current["receiver"].observe(
                        linear_m_s, angular_rad_s,
                        stamp_s=stamp, stop_after_s=current["stop_after_s"], now_s=now_s)
                    try:
                        energy.observe(run_id + "_" + owner_lane, robot_id, stamp,
                                       linear_m_s, angular_rad_s, loaded=False)
                    except ValueError:
                        pass

                owner.create_subscription(Odometry, "odom", odometry, mission_odometry_qos())
                for side in ("front", "rear"):
                    state["io_node"].create_subscription(
                        LaserScan, f"scan_{side}_raw",
                        lambda _message, robot_id=robot, scan_side=side:
                            robots[robot_id]["scan_counts"].__setitem__(
                                scan_side, robots[robot_id]["scan_counts"][scan_side] + 1),
                        qos_profile_sensor_data)

                source = OwnedRos.create_node("source_" + robot, namespace=f"{namespace_root}/{robot}")
                nodes.append(source)
                publishers.append({
                    "clock": source.create_publisher(Clock, "/clock", qos_profile_sensor_data),
                    "tf": source.create_publisher(TFMessage, "tf", 100),
                    "odom": source.create_publisher(Odometry, "odom", mission_odometry_qos()),
                    "front": source.create_publisher(LaserScan, "scan_front_raw", qos_profile_sensor_data),
                    "rear": source.create_publisher(LaserScan, "scan_rear_raw", qos_profile_sensor_data),
                })

            published = {"clock": 0, "tf": 0, "odom": 0, "scan_front": 0, "scan_rear": 0}
            stream = {"latest_stamp_s": SIM_EPOCH_S, "post_hold_messages": 0}

            def publish_loop() -> None:
                origin = time.monotonic()
                next_clock = next_data = 0.0
                try:
                    while not stop.is_set():
                        bounded()
                        elapsed = time.monotonic() - origin
                        stamp_s = SIM_EPOCH_S + elapsed
                        sec = int(stamp_s)
                        nanosec = int((stamp_s-sec) * 1e9)
                        if elapsed >= next_clock:
                            clock = Clock()
                            clock.clock.sec, clock.clock.nanosec = sec, nanosec
                            for pubs in publishers:
                                pubs["clock"].publish(clock)
                            published["clock"] += len(publishers)
                            next_clock += 1.0 / 60.0
                        if elapsed >= next_data:
                            for index, pubs in enumerate(publishers):
                                robot = ROBOT_IDS[index]
                                odom = Odometry()
                                odom.header.stamp.sec, odom.header.stamp.nanosec = sec, nanosec
                                odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
                                pubs["odom"].publish(odom)
                                transforms = []
                                for parent, child in (("map", "odom"), ("odom", "base_link")):
                                    transform = TransformStamped()
                                    transform.header.stamp.sec, transform.header.stamp.nanosec = sec, nanosec
                                    transform.header.frame_id, transform.child_frame_id = parent, child
                                    transform.transform.translation.x = 0.1
                                    transform.transform.rotation.w = 1.0
                                    transforms.append(transform)
                                pubs["tf"].publish(TFMessage(transforms=transforms))
                                for side in ("front", "rear"):
                                    scan = LaserScan()
                                    scan.header.stamp.sec, scan.header.stamp.nanosec = sec, nanosec
                                    scan.header.frame_id = "laser_" + side
                                    scan.angle_min, scan.angle_max = -3.14159265, 3.14159265
                                    scan.angle_increment = 6.2831853 / 3199
                                    scan.range_min, scan.range_max = 0.05, 30.0
                                    scan.ranges = [2.0] * 3200
                                    pubs[side].publish(scan)
                                with lock:
                                    if robots[robot]["stop_after_s"] is not None:
                                        stream["post_hold_messages"] += 1
                            published["tf"] += len(publishers)
                            published["odom"] += len(publishers)
                            published["scan_front"] += len(publishers)
                            published["scan_rear"] += len(publishers)
                            with lock:
                                stream["latest_stamp_s"] = stamp_s
                            next_data += 0.05
                        stop.wait(0.002)
                except BaseException as error:
                    publisher_errors.append(f"{type(error).__name__}: {error}")
                    stop.set()

            worker = threading.Thread(target=publish_loop, name=f"odom_dispatch_{owner_lane}", daemon=False)
            worker.start()
            warmup_deadline = min(deadline, time.monotonic() + 12.0)
            while any(state["odom_count"] < 2 or min(state["scan_counts"].values()) < 1
                      for state in robots.values()):
                bounded()
                if publisher_errors:
                    raise RuntimeError(publisher_errors[0])
                if time.monotonic() > warmup_deadline:
                    raise TimeoutError("ROS discovery/receipt warmup timed out")
                drain.drain()
                time.sleep(0.005)

            pre_load = {robot: {"odom": state["odom_count"], "scans": dict(state["scan_counts"])}
                        for robot, state in robots.items()}

            def queued_work(robot_id: str) -> None:
                time.sleep(0.03)
                robots[robot_id]["work_callbacks"] += 1

            for robot, io_executor in zip(ROBOT_IDS, io_executors):
                for _ in range(QUEUED_WORK_PER_ROBOT):
                    io_executor.create_task(queued_work, robot)

            time.sleep(0.65)
            with lock:
                hold_boundary_s = stream["latest_stamp_s"]
                stream["post_hold_messages"] = 0
                for state in robots.values():
                    state["stop_after_s"] = hold_boundary_s
                    state["receiver"].reset()
            case["hold_boundary_s"] = hold_boundary_s

            dispatch_calls = []
            post_hold_started = time.monotonic()
            while time.monotonic() - post_hold_started < POST_HOLD_STREAM_S:
                bounded()
                dispatch_calls.append(drain.drain())
                if publisher_errors:
                    raise RuntimeError(publisher_errors[0])
                time.sleep(0.004)
            dispatch_calls.append(drain.drain())

            robot_results = {}
            event_now_s = max(state["node"].get_clock().now().nanoseconds / 1e9
                              for state in robots.values())
            for robot, state in robots.items():
                local_now_s = state["node"].get_clock().now().nanoseconds / 1e9
                last_stamp = state["last_odom_stamp_s"]
                stop_record = committed_stopped_odometry_record(
                    state["receiver"], local_now_s=local_now_s, event_now_s=event_now_s)
                try:
                    transform = state["buffer"].lookup_transform("map", "base_link", Time())
                    tf_stamp_s = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                    tf_age_s = local_now_s - tf_stamp_s
                except TransformException:
                    tf_stamp_s = tf_age_s = None
                projection = energy.projection(run_id + "_" + owner_lane, robot, local_now_s)
                robot_results[robot] = {
                    "published_odom_total": published["odom"] // len(ROBOT_IDS),
                    "received_odom_total": state["odom_count"],
                    "received_odom_since_load": state["odom_count"] - pre_load[robot]["odom"],
                    "last_odom_stamp_s": last_stamp,
                    "odom_age_s": None if last_stamp is None else local_now_s-last_stamp,
                    "local_now_s": local_now_s,
                    "stop_qualified": stop_record is not None,
                    "stop_record": stop_record,
                    "tf_stamp_s": tf_stamp_s,
                    "tf_age_s": tf_age_s,
                    "scan_counts_since_load": {
                        side: state["scan_counts"][side]-pre_load[robot]["scans"][side]
                        for side in ("front", "rear")},
                    "queued_work_callbacks": state["work_callbacks"],
                    "energy_freshness": projection["freshness"],
                    "energy_fault": projection["fault"],
                }
            case.update({
                "published_messages": dict(published),
                "post_hold_odom_messages": stream["post_hold_messages"],
                "post_hold_wall_s": time.monotonic()-post_hold_started,
                "robots": robot_results,
                "dispatch": drain.snapshot(),
                "dispatch_calls": len(dispatch_calls),
                "io_fair_round_received": all(state["work_callbacks"] > 0 for state in robots.values()),
                "energy_snapshot": energy.snapshot(),
                "publisher_errors": list(publisher_errors),
                "status": "completed",
            })
        finally:
            stop.set()
            if worker is not None:
                worker.join(timeout=3.0)
            case["publisher_thread_closed"] = worker is None or not worker.is_alive()
            for listener in listeners:
                try:
                    listener.unregister()
                except BaseException as error:
                    case["cleanup_errors"].append("listener: " + str(error))
            for node, executor in reversed(executors):
                try:
                    executor.remove_node(node)
                    if not executor.shutdown(timeout_sec=1.0):
                        raise RuntimeError("executor shutdown timeout")
                    if executor.get_nodes():
                        raise RuntimeError("executor still owns nodes")
                except BaseException as error:
                    case["cleanup_errors"].append("executor: " + str(error))
            for node in reversed(nodes):
                try:
                    node.destroy_node()
                except BaseException as error:
                    case["cleanup_errors"].append("node: " + str(error))
            try:
                if context.ok():
                    context.shutdown()
                case["context_closed"] = not context.ok()
            except BaseException as error:
                case["cleanup_errors"].append("context: " + str(error))
            case["owned_node_count"] = len(nodes)
            case["owned_executor_count"] = len(executors)
            case["owned_resources_closed"] = (
                case["publisher_thread_closed"] and case.get("context_closed", False)
                and not case["cleanup_errors"])
        return case

    try:
        baseline = trial("io_node")
        priority = trial("priority_node")
        baseline_stale_rejection_reproduced = any(
            row["odom_age_s"] is None or row["odom_age_s"] > odom_ttl_s or not row["stop_qualified"]
            for row in baseline["robots"].values())
        priority_acceptance = all(
            row["odom_age_s"] is not None and 0.0 <= row["odom_age_s"] <= odom_ttl_s
            and row["stop_qualified"]
            and row["tf_age_s"] is not None and 0.0 <= row["tf_age_s"] <= tf_ttl_s
            and row["energy_fault"] is None
            for row in priority["robots"].values())
        cleanup_ok = all(case["owned_resources_closed"] for case in report["cases"])
        report.update({
            "baseline_stale_stop_rejection_reproduced": baseline_stale_rejection_reproduced,
            "priority_acceptance": priority_acceptance,
            "io_fairness_verified": all(case["io_fair_round_received"] for case in report["cases"]),
            "cleanup": {"all_owned_resources_closed": cleanup_ok},
        })
        report["status"] = "passed" if (
            baseline_stale_rejection_reproduced and priority_acceptance
            and report["io_fairness_verified"] and cleanup_ok) else "failed"
    except BaseException as error:
        report.update(status="failed", error={
            "type": type(error).__name__, "message": str(error),
            "traceback": traceback.format_exc(limit=10),
        })
    report["wall_elapsed_s"] = time.monotonic() - started
    print(json.dumps(report, allow_nan=False, sort_keys=True), flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
