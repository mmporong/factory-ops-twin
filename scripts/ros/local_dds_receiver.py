#!/usr/bin/env python3
"""Bound an isolated WSL receiver; never subscribe to or publish robot commands."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time


def save(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def scan_observation(message):
    """Check a real scan against the authored square room, not a truth topic."""
    angles = [float(message.angle_min), float(message.angle_max), float(message.angle_increment)]
    if not all(math.isfinite(x) for x in angles) or angles[2] <= 0 or len(message.ranges) < 100:
        raise ValueError("invalid scan angle metadata")
    if abs(angles[0] + (len(message.ranges) - 1) * angles[2] - angles[1]) > 2 * angles[2]:
        raise ValueError("scan size and angle metadata disagree")
    errors = []
    valid_angles = []
    raw = []
    for index, value in enumerate(message.ranges):
        value = float(value)
        if math.isnan(value) or (value < 0 and value != -1.):
            raise ValueError("NaN or negative range")
        raw.append(value if math.isfinite(value) else "inf")
        if math.isfinite(value) and message.range_min <= value <= message.range_max:
            angle = angles[0] + index * angles[2]
            valid_angles.append(angle)
            expected = 4. / max(abs(math.cos(angle)), abs(math.sin(angle)))
            errors.append(abs(value - expected))
    sectors = {int(((angle + math.pi) % (2 * math.pi)) / (math.pi / 360))
               for angle in valid_angles}
    if len(sectors) != 720:
        raise ValueError("square room has an uncovered half-degree sector")
    gaps = [after - before for before, after in
            zip(valid_angles, valid_angles[1:] + [valid_angles[0] + 2 * math.pi])]
    if max(gaps) > math.pi / 360 + 1e-7:
        raise ValueError("valid-return angular gap exceeds half a degree")
    errors.sort()
    p95 = errors[min(len(errors) - 1, int(.95 * len(errors)))]
    if p95 > .12:
        raise ValueError("RTX ranges do not match the authored 4m wall faces: p95=" + str(p95))
    return {"stamp_s": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
            "frame_id": message.header.frame_id, "angle_min": angles[0], "angle_max": angles[1],
            "angle_increment": angles[2], "range_min": float(message.range_min),
            "range_max": float(message.range_max), "ranges": raw,
            "finite_returns": len(errors), "invalid_returns": len(raw) - len(errors),
            "covered_half_degree_sectors": len(sectors),
            "maximum_valid_return_gap_deg": math.degrees(max(gaps)),
            "raw_fill_95_percent_passed": len(errors) >= .95 * len(raw),
            "quality_scope": "angular coverage and authored wall ranges only; raw dropout preserved",
            "room_range_error_p95_m": p95}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if not args.output.is_dir() or len(args.run_id) != 32:
        parser.error("an existing producer output and run identity are required")
    os.environ["ROS_DOMAIN_ID"] = "143"
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    import rclpy
    from std_msgs.msg import String
    producer = json.loads((args.output / "result.json").read_text(encoding="utf-8"))
    cfg = producer["config"]
    lidar_mode = cfg.get("mode") == "rtx_scan"
    rclpy.init(args=[])
    node = rclpy.create_node("wsl_receiver_" + args.run_id[:8])
    report = {"run_id": args.run_id, "status": "waiting", "messages": [], "rclpy_source": rclpy.__file__,
              "pid": os.getpid(), "domain_id": 143, "mode": cfg.get("mode", "string"),
              "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "config_sha256": producer["config_sha256"],
              "clocks_s": [], "rejected_scans": []}
    def receive(message):
        try:
            value = json.loads(message.data)
            if (value.get("run_id") == args.run_id and type(value.get("seq")) is int
                    and (not report["messages"] or value["seq"] > report["messages"][-1]["seq"])):
                report["messages"].append(value)
        except (ValueError, TypeError, AttributeError):
            pass
    if lidar_mode:
        from sensor_msgs.msg import LaserScan
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import qos_profile_sensor_data
        def receive_scan(message):
            if len(report["messages"]) >= 3:
                return
            try:
                value = scan_observation(message)
                if value["frame_id"] != "probe_lidar_" + args.run_id or value["stamp_s"] <= 0:
                    raise ValueError("wrong sensor identity/time")
                if report["messages"] and value["stamp_s"] <= report["messages"][-1]["stamp_s"]:
                    return
                report["messages"].append(value)
            except ValueError as error:
                if len(report["rejected_scans"]) < 10:
                    report["rejected_scans"].append(str(error))
                rejected = args.output / "first-rejected-scan.json"
                if not rejected.exists():
                    values = [float(x) for x in message.ranges]
                    save(rejected, {"run_id": args.run_id, "error": str(error),
                        "frame_id": message.header.frame_id,
                        "stamp_s": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                        "angle_min": float(message.angle_min), "angle_max": float(message.angle_max),
                        "angle_increment": float(message.angle_increment),
                        "range_min": float(message.range_min), "range_max": float(message.range_max),
                        "ranges": [x if math.isfinite(x) else str(x) for x in values]})
        def receive_clock(message):
            stamp = message.clock.sec + message.clock.nanosec / 1e9
            if stamp > 0 and (not report["clocks_s"] or stamp > report["clocks_s"][-1]):
                report["clocks_s"].append(stamp)
        subscription = node.create_subscription(LaserScan, "/factory_ops_local45/scan_" + args.run_id,
                                                receive_scan, qos_profile_sensor_data)
        clock_sub = node.create_subscription(Clock, "/factory_ops_local45/clock_" + args.run_id,
                                             receive_clock, 20)
    else:
        clock_sub = None
        subscription = node.create_subscription(String, "/factory_ops_local45/probe_" + args.run_id, receive, 10)
    save(args.output / "receiver-ready.json", {"run_id": args.run_id, "pid": os.getpid()})
    try:
        if lidar_mode:
            ready = args.output / "sensor-stream-ready.json"
            startup_deadline = time.monotonic() + 120
            while not ready.is_file():
                if time.monotonic() >= startup_deadline:
                    raise TimeoutError("native sensor initialization deadline expired")
                rclpy.spin_once(node, timeout_sec=.05)
            if json.loads(ready.read_text(encoding="utf-8"))["run_id"] != args.run_id:
                raise ValueError("sensor stream readiness belongs to another run")
            save(args.output / "sensor-receiver-armed.json", {"run_id": args.run_id, "pid": os.getpid()})
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(report["messages"]) >= 3:
                if not lidar_mode:
                    break
                clocks = report["clocks_s"]
                if clocks:
                    errors = [min(abs(scan["stamp_s"] - stamp) for stamp in clocks)
                              for scan in report["messages"]]
                    if max(errors) <= .05:
                        report["scan_clock_nearest_errors_s"] = errors
                        break
        report["status"] = ("passed" if len(report["messages"]) >= 3
                            and (not lidar_mode or "scan_clock_nearest_errors_s" in report) else "failed")
        save(args.output / "receiver-result.json", report)
    finally:
        node.destroy_subscription(subscription)
        if clock_sub is not None:
            node.destroy_subscription(clock_sub)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"status": report["status"], "received": len(report["messages"])}))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
