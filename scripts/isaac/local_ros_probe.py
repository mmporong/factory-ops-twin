#!/usr/bin/env python3
"""Test the existing native SDK ROS bridge without robot command topics."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time
import traceback


def save(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if args.output.exists() or not args.output.is_absolute() or len(args.run_id) != 32:
        parser.error("use a new absolute output and run identity")
    cfg = json.loads(args.config.read_text(encoding="utf-8-sig"))
    if cfg["domain_id"] != 143 or cfg["minimum_messages"] != 3 or cfg["stream_wall_s"] != 30:
        parser.error("unsupported isolated probe settings")
    args.output.mkdir(parents=True)
    report = {"run_id": args.run_id, "status": "starting", "stage": "sdk_startup", "config": cfg,
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
        "isaac_version": (Path(os.environ["ISAAC_PATH"]) / "VERSION").read_text().strip(),
        "topic": "/factory_ops_local45/probe_" + args.run_id,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "vehicle_commands": False, "physical_fleet_verified": False, "requires_host_closure": True}
    save(args.output / "result.json", report)
    app = node = ros = None
    code = 1
    try:
        os.environ["ROS_DOMAIN_ID"] = str(cfg["domain_id"])
        os.environ["ROS_LOCALHOST_ONLY"] = "0"
        os.environ["ROS_DISTRO"] = "humble"
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        native_ros_lib = Path(os.environ["ISAAC_PATH"]) / "exts/isaacsim.ros2.bridge/humble/lib"
        if not (native_ros_lib / "rmw_fastrtps_cpp.dll").is_file():
            raise RuntimeError("existing bundled ROS middleware DLL is unavailable")
        os.environ["PATH"] = str(native_ros_lib) + os.pathsep + os.environ.get("PATH", "")
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "multi_gpu": False})
        from pxr import Usd, Pcp
        report["installed_usd_error_apis"] = {
            "Stage": [name for name in dir(Usd.Stage) if "error" in name.lower()],
            "PrimIndex": list(dir(Pcp.PrimIndex))}
        save(args.output / "result.json", report)
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        app.update()
        import rclpy
        from std_msgs.msg import String
        ros = rclpy
        ros.init(args=[])
        node = ros.create_node("native_sdk_probe_" + args.run_id[:8])
        publisher = node.create_publisher(String, report["topic"], 10)
        report.update(stage="await_receiver", rclpy_source=rclpy.__file__)
        save(args.output / "result.json", report)
        deadline = time.monotonic() + cfg["wait_receiver_wall_s"]
        ready = args.output / "receiver-ready.json"
        while not ready.is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError("receiver did not enter its bounded subscription window")
            app.update()
            time.sleep(.05)
        if json.loads(ready.read_text(encoding="utf-8"))["run_id"] != args.run_id:
            raise ValueError("receiver readiness belongs to another run")
        if cfg.get("mode") == "rtx_scan":
            from local_lidar_bridge import run
            run(app, ros, node, args.output, args.run_id, cfg, report, save)
        else:
            report["stage"] = "publishing"
            save(args.output / "result.json", report)
            started = time.monotonic()
            acknowledgement = args.output / "receiver-result.json"
            count = 0
            with (args.output / "published.jsonl").open("x", encoding="utf-8") as stream:
                while time.monotonic() - started < cfg["stream_wall_s"]:
                    payload = {"run_id": args.run_id, "seq": count, "sdk_version": report["isaac_version"],
                               "publisher_elapsed_wall_s": time.monotonic() - started}
                    message = String()
                    message.data = json.dumps(payload, allow_nan=False)
                    publisher.publish(message)
                    stream.write(message.data + "\n")
                    stream.flush()
                    count += 1
                    ros.spin_once(node, timeout_sec=0.)
                    app.update()
                    if acknowledgement.is_file():
                        ack = json.loads(acknowledgement.read_text(encoding="utf-8"))
                        if ack["run_id"] == args.run_id and ack["status"] == "passed" and len(ack["messages"]) >= cfg["minimum_messages"]:
                            report.update(status="passed", receiver_count=len(ack["messages"]))
                            break
                    time.sleep(cfg["publish_interval_s"])
            report["published_count"] = count
            if report["status"] != "passed":
                raise TimeoutError("WSL did not acknowledge three matching messages")
        code = 0
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
                                             "traceback": traceback.format_exc(limit=14)})
    finally:
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        save(args.output / "result.json", report)
        print("LOCAL_ROS_TRANSPORT_RESULT", report["status"], flush=True)
        if app is not None:
            app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
