"""Isolated real Jazzy TF/clock scheduling coupon; no SDK, Nav2 or bag replay.

Real ROS Clock/TF/LaserScan messages run against production dispatch wiring.
Authored finite slow executor tasks stand in for queued action/scan CPU work.
This is a component contention test, not a reproduction of the complete041 run.
Only stdout JSON is written; every owned context/node/executor/thread is closed.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import traceback
import uuid

from factory_ops.callback_drain import CallbackDrain
from scripts.ros.g3_mission import create_robot_dispatch_nodes, mask_scan_ranges, sample_localization_batch


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, required=True)
    parser.add_argument("--wall-limit-s", type=float, default=90.)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232 or args.domain_id == 73 or not 10 <= args.wall_limit_s <= 110:
        parser.error("isolated domain (not factory73) and 10..110 second bound required")
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener, TransformException

    run = uuid.uuid4().hex
    started = time.monotonic()
    deadline = started+args.wall_limit_s
    report = {"task": "localization-critical-dispatch-120", "run_id": run,
        "pid": os.getpid(), "status": "running", "domain_id": args.domain_id,
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "actual Jazzy component dispatch under authored queued work; not full041/native reproduction",
        "cases": [], "cleanup": [], "sdk_or_nav2_started": False,
        "ttl_s": .5, "max_attempts": 3, "max_rounds": 64, "wall_budget_s": .05}

    def bounded():
        if time.monotonic() > deadline:
            raise TimeoutError("component wall deadline exceeded")

    def trial(isolated):
        context = Context()
        nodes, executors = [], []
        listener = worker = None
        stop = threading.Event()
        tf_enabled = threading.Event()
        tf_enabled.set()
        publisher_errors = []
        case = {"isolated": isolated, "status": "running", "cleanup": []}
        report["cases"].append(case)
        try:
            rclpy.init(context=context, domain_id=args.domain_id)
            case["actual_domain_id"] = context.get_domain_id()
            namespace = "dispatch120_"+run[:12]+("_isolated" if isolated else "_shared")

            class OwnedRos:
                @staticmethod
                def create_node(name, **kwargs):
                    return rclpy.create_node(name, context=context, **kwargs)

            event = OwnedRos.create_node("event", namespace=namespace,
                parameter_overrides=[Parameter("use_sim_time", value=True)])
            nodes.append(event)
            event_ex = SingleThreadedExecutor(context=context)
            executors.append((event, event_ex))
            event_ex.add_node(event)
            tf_node, io, tf_ex, io_ex = create_robot_dispatch_nodes("robot", namespace, isolated=isolated,
                ros=OwnedRos, parameter_type=Parameter,
                executor_type=lambda: SingleThreadedExecutor(context=context),
                owned_nodes=nodes, owned_executors=executors)
            buffer = Buffer(node=tf_node)
            listener = TransformListener(buffer, tf_node)
            publisher = OwnedRos.create_node("source", namespace=namespace)
            nodes.append(publisher)
            clock_pub = publisher.create_publisher(Clock, "/clock", qos_profile_sensor_data)
            tf_pub = publisher.create_publisher(TFMessage, "tf", 100)
            scan_pub = publisher.create_publisher(LaserScan, "scan_front_raw", qos_profile_sensor_data)
            counts = {"scan_callbacks": 0, "work_callbacks": 0, "published_tf": 0}
            source_stamps = []

            def scan_callback(message):
                filtered = copy.deepcopy(message)
                filtered.ranges = mask_scan_ranges(message.angle_min, message.angle_max,
                                                   message.angle_increment, message.ranges)
                counts["scan_callbacks"] += 1

            io.create_subscription(LaserScan, "scan_front_raw", scan_callback, qos_profile_sensor_data)
            drain = CallbackDrain((io_ex,), priority_executors=(event_ex, tf_ex),
                max_rounds=64, wall_budget_s=.05) if isolated else CallbackDrain(
                (event_ex, tf_ex), max_rounds=64, wall_budget_s=.05)

            def lookup(_):
                try:
                    return buffer.lookup_transform("map", "base_link", Time())
                except TransformException:
                    return None

            def publish():
                origin = time.monotonic()
                last_tf = -1.
                try:
                    while not stop.is_set():
                        bounded()
                        elapsed = time.monotonic()-origin
                        ns = 1000_000_000_000+int(elapsed*1e9)
                        sec, nano = divmod(ns, 10**9)
                        clock = Clock()
                        clock.clock.sec, clock.clock.nanosec = sec, nano
                        clock_pub.publish(clock)
                        if tf_enabled.is_set() and elapsed-last_tf >= .05:
                            transforms = []
                            for parent, child in (("map", "odom"), ("odom", "base_link")):
                                t = TransformStamped()
                                t.header.stamp.sec, t.header.stamp.nanosec = sec, nano
                                t.header.frame_id, t.child_frame_id = parent, child
                                t.transform.translation.x = .1
                                t.transform.rotation.w = 1.
                                transforms.append(t)
                            tf_pub.publish(TFMessage(transforms=transforms))
                            counts["published_tf"] += 1
                            source_stamps.append(ns)
                            last_tf = elapsed
                        scan = LaserScan()
                        scan.header.stamp.sec, scan.header.stamp.nanosec = sec, nano
                        scan.header.frame_id = "laser_front"
                        scan.angle_min, scan.angle_max, scan.angle_increment = -3.14159265, 3.14159265, 6.2831853/3199
                        scan.range_min, scan.range_max = .05, 30.
                        scan.ranges = [2.]*3200
                        scan_pub.publish(scan)
                        stop.wait(.01)
                except BaseException as error:
                    publisher_errors.append(str(error))
                    stop.set()

            worker = threading.Thread(target=publish, name="dispatch120_source_"+run[:8], daemon=False)
            worker.start()
            warmup_deadline = min(deadline, time.monotonic()+10.)
            while (lookup("robot") is None or event.get_clock().now().nanoseconds == 0
                    or counts["scan_callbacks"] == 0):
                bounded()
                if publisher_errors: raise RuntimeError(publisher_errors[0])
                if time.monotonic() > warmup_deadline: raise TimeoutError("isolated ROS discovery/TF warmup")
                drain.drain()
                time.sleep(.005)
            seed = lookup("robot")
            case["seed_tf_stamp_ns"] = seed.header.stamp.sec*10**9+seed.header.stamp.nanosec

            def queued_work():
                time.sleep(.03)
                counts["work_callbacks"] += 1

            for _ in range(24):
                io_ex.create_task(queued_work)
            # Authored consumer pause causes real DDS/TF callback backlog while
            # the actual Clock/TF publishers continue. No buffer is filled by hand.
            time.sleep(.65)
            traces = []
            try:
                clock_ns, poses = sample_localization_batch(("robot",), drain=drain.drain,
                    read_clock_ns=lambda: event.get_clock().now().nanoseconds,
                    lookup=lookup, cached={"robot": None}, ttl_s=.5, require_fresh=True,
                    max_attempts=3, checkpoint=traces.append)
                pose = poses["robot"]
                case["loaded_sample"] = {"accepted": True, "clock_ns": clock_ns,
                    "tf_stamp_s": pose["stamp_s"], "age_s": clock_ns/1e9-pose["stamp_s"],
                    "stamp_is_from_published_tf": any(abs(s/1e9-pose["stamp_s"]) < 1e-9 for s in source_stamps)}
            except RuntimeError as error:
                if not str(error).startswith("fresh map pose unavailable"):
                    raise
                case["loaded_sample"] = {"accepted": False, "reason": str(error)}
            case["loaded_attempt_diagnostics"] = traces
            case["loaded_counts"] = dict(counts)
            if isolated:
                tf_enabled.clear()
                time.sleep(.65)  # True TF loss; Clock and scans continue.
                loss_traces = []
                try:
                    sample_localization_batch(("robot",), drain=drain.drain,
                        read_clock_ns=lambda: event.get_clock().now().nanoseconds,
                        lookup=lookup, cached={"robot": None}, ttl_s=.5, require_fresh=True,
                        max_attempts=3, checkpoint=loss_traces.append)
                    case["persistent_loss_rejected"] = False
                except RuntimeError as error:
                    if not str(error).startswith("fresh map pose unavailable"):
                        raise
                    case["persistent_loss_rejected"] = True
                case["loss_attempt_diagnostics"] = loss_traces
            case["dispatch"] = drain.snapshot()
            case["nodes"] = [n.get_fully_qualified_name() for n in nodes]
            if publisher_errors: raise RuntimeError(publisher_errors[0])
            case["status"] = "completed"
        finally:
            stop.set()
            if worker is not None:
                worker.join(timeout=2.)
                case["publisher_thread_closed"] = not worker.is_alive()
            else:
                case["publisher_thread_closed"] = True
            if listener is not None:
                try: listener.unregister()
                except BaseException as error: case["cleanup"].append("listener: "+str(error))
            for node, executor in reversed(executors):
                try:
                    executor.remove_node(node)
                    if not executor.shutdown(timeout_sec=1.):
                        raise RuntimeError("executor shutdown timeout")
                    if executor.get_nodes(): raise RuntimeError("executor still owns nodes")
                except BaseException as error: case["cleanup"].append("executor: "+str(error))
            for node in reversed(nodes):
                try: node.destroy_node()
                except BaseException as error: case["cleanup"].append("node: "+str(error))
            try:
                if context.ok(): context.shutdown()
                case["context_closed"] = not context.ok()
            except BaseException as error: case["cleanup"].append("context: "+str(error))
            case["owned_node_count"] = len(nodes)
            case["owned_executor_count"] = len(executors)
            case["owned_resources_closed"] = (not case["cleanup"] and case["publisher_thread_closed"]
                                               and case.get("context_closed", False))
        bounded()

    try:
        trial(False)
        trial(True)
        shared, isolated = report["cases"]
        passed = (not shared["loaded_sample"]["accepted"] and isolated["loaded_sample"]["accepted"]
            and isolated["loaded_sample"]["stamp_is_from_published_tf"]
            and isolated["persistent_loss_rejected"] and all(c["owned_resources_closed"] for c in report["cases"]))
        report["status"] = "passed" if passed else "failed"
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
            "traceback": traceback.format_exc(limit=8)})
    report["wall_elapsed_s"] = time.monotonic()-started
    print("DISPATCH120_RESULT "+json.dumps(report, allow_nan=False), flush=True)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
