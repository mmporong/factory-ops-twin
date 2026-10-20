"""One bounded domain74 Jazzy static-map planner comparison with owned Linux closure."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from factory_ops.planner_guidance import authored_guides


def save(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def host(args):
    from factory_ops.wsl_process_supervisor import WslProcessSupervisor
    from scripts.isaac.run_local45_fleet_pair import linux
    if not args.output.is_absolute() or args.output.exists() or args.output.is_symlink():
        raise ValueError("new absolute CPU probe output required")
    args.output.mkdir(parents=True)
    owner = WslProcessSupervisor(args.run_id)
    report = {"run_id": args.run_id, "scope": "CPU planner comparison only", "status": "running"}
    started = time.monotonic()
    try:
        command = "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=74 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1; "
        command += "export ROS_LOG_DIR="+shlex.quote(linux(args.output / "roslogs"))+"; exec "+shlex.join([
            "/usr/bin/python3", "-B", linux(Path(__file__).resolve()), "--linux-worker", "--run-id", args.run_id,
            "--output", linux(args.output), "--frozen-stage", linux(args.frozen_stage), "--case", linux(args.case)])
        report["start"] = owner.start(["/bin/bash", "-lc", command], working_directory=linux(ROOT),
            log_path=linux(args.output/"unit.log"), runtime_bound_s=240)
        report["observed"] = owner.inspect()
        save(args.output/"host.json", report)
        while owner.client_state()["running"]:
            if time.monotonic()-started > 240:
                raise TimeoutError("CPU fixture exceeded 240 wall seconds")
            time.sleep(.5)
        result = json.loads((args.output/"result.json").read_text(encoding="utf-8"))
        report["status"] = result["status"]
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
    finally:
        try:
            closure = owner.stop()
            save(args.output/"closure.json", closure)
            report["closure_verified"] = closure["closure_verified"]
            if not closure["closure_verified"]:
                report["status"] = "failed"
        except BaseException as error:
            report.update(status="failed", cleanup_error=str(error))
        report["wall_s"] = time.monotonic()-started
        save(args.output/"host.json", report)
    print(json.dumps({key: report.get(key) for key in ("run_id", "status", "closure_verified", "wall_s", "error")}))
    return 0 if report["status"] == "passed" else 1


def worker(args):
    import yaml
    import rclpy
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses
    from tf2_ros import StaticTransformBroadcaster
    from factory_ops.fleet_execution import make_chunk, qualify_computed_path
    from factory_ops.fleet_routing import RouteBook, swept_tiles
    from factory_ops.navigation import GoalPose
    from scripts.ros.g3_mission import path_payload, path_content_sha256, path_header_stamp_ns, path_final_yaw, angle_error

    deadline = time.monotonic()+210
    frozen = args.frozen_stage/"results/run"
    case = json.loads(args.case.read_text(encoding="utf-8"))
    raw = yaml.safe_load((frozen/"nav2-g3-params.yaml").read_text())
    params = {"carter3": {key: deepcopy(raw["carter3"][key]) for key in
        ("navigation_map_server", "planner_server", "global_costmap")}, "/**": {"ros__parameters": {"use_sim_time": False}}}
    for name in ("navigation_map_server", "planner_server"):
        params["carter3"][name]["ros__parameters"]["use_sim_time"] = False
    params["carter3"]["global_costmap"]["global_costmap"]["ros__parameters"]["use_sim_time"] = False
    (args.output/"params.yaml").write_text(yaml.safe_dump(params), encoding="utf-8")
    book = RouteBook.from_files(args.frozen_stage/"inputs/layout.json", args.frozen_stage/"inputs/g3-routing.json")
    expected = case["failed_action"]["chunk"]
    chunk = make_chunk(book, expected["start"], expected["target"], closed=expected["closed"],
        final_goal=GoalPose(**expected["final_goal"]))
    if chunk.to_dict() != expected:
        raise ValueError("frozen authored chunk differs from original failed action")
    start = case["previous_qualified_arrival"]
    report = {"run_id": args.run_id, "status": "running", "source_case_sha256": sha(args.case),
        "source_params_sha256": sha(frozen/"nav2-g3-params.yaml"), "fixture_params_sha256": sha(args.output/"params.yaml"),
        "map_inputs": {name: sha(frozen/"maps/open"/name) for name in ("navigation.yaml", "navigation.pgm")},
        "deviations": ["domain74/localhost CPU only", "use_sim_time=false/system clock, no SDK clock",
            "one static map->base_link TF at supplied previous qualified arrival, not the exact later TF sample",
            "original obstacle plugins/parameters retained, but no live scans/dynamic obstacles supplied",
            "only map_server/planner_server lifecycle nodes; no controller/AMCL/robots/actuation"],
        "chunk": chunk.to_dict(), "start": start, "cases": {}}
    processes, streams = [], []
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node("reserved_planner_077_probe")
        report["actual_context_domain"] = node.context.get_domain_id()
        for package, executable, name in (("nav2_map_server", "map_server", "navigation_map_server"),
                                          ("nav2_planner", "planner_server", "planner_server")):
            stream = (args.output/(name+".log")).open("xb")
            streams.append(stream)
            process = subprocess.Popen([f"/opt/ros/jazzy/lib/{package}/{executable}", "--ros-args", "-r",
                "__ns:=/carter3", "-r", "__node:="+name, "--params-file", str(args.output/"params.yaml")],
                stdout=stream, stderr=subprocess.STDOUT)
            processes.append(process)
        report["child_pids"] = [p.pid for p in processes]
        save(args.output/"result.json", report)

        def wait(future):
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., min(35., deadline-time.monotonic())))
            if not future.done():
                raise TimeoutError("bounded planner/service future did not complete")
            return future.result()

        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_link"
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y = start["x_m"], start["y_m"]
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(start["yaw_rad"]/2), math.cos(start["yaw_rad"]/2)
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform)
        for name in ("navigation_map_server", "planner_server"):
            client = node.create_client(ChangeState, f"/carter3/{name}/change_state")
            if not client.wait_for_service(timeout_sec=20.):
                raise TimeoutError("lifecycle service unavailable: "+name)
            for transition in (1, 3):
                request = ChangeState.Request(); request.transition.id = transition
                if not wait(client.call_async(request)).success:
                    raise RuntimeError("lifecycle transition failed: "+name)

        def pose(x, y, yaw, stamp):
            value = PoseStamped(); value.header.frame_id = "map"; value.header.stamp = stamp
            value.pose.position.x, value.pose.position.y = float(x), float(y)
            value.pose.orientation.z, value.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            return value

        for method, action, topic in (("to_pose", ComputePathToPose, "compute_path_to_pose"),
                                      ("through_poses", ComputePathThroughPoses, "compute_path_through_poses")):
            client = ActionClient(node, action, "/carter3/"+topic)
            if not client.wait_for_server(timeout_sec=20.):
                raise TimeoutError("planner action unavailable: "+topic)
            request = action.Goal()
            stamp = node.get_clock().now().to_msg()
            request.start = pose(start["x_m"], start["y_m"], start["yaw_rad"], stamp)
            request.use_start, request.planner_id = True, "GridBased"
            if method == "to_pose": request.goal = pose(chunk.goal_pose.x, chunk.goal_pose.y, chunk.goal_pose.yaw, stamp)
            else: request.goals = [pose(goal.x, goal.y, goal.yaw, stamp) for goal in authored_guides(chunk)]
            begun = time.monotonic()
            handle = wait(client.send_goal_async(request))
            if not handle.accepted:
                raise RuntimeError("planner request rejected: "+method)
            terminal = wait(handle.get_result_async())
            payload = path_payload(terminal.result.path)
            save(args.output/(method+"-path.json"), payload)
            points = [[p["position_xyz_m"][0], p["position_xyz_m"][1]] for p in payload["poses"]]
            bounds = book.layout.bounds
            tiles = swept_tiles(points, book.evaluation_radius_m, tile_size_m=book.tile_size_m,
                bounds_xy=(bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax)) if points else frozenset()
            outside = sorted(tiles-chunk.tiles)
            row = {"status": int(terminal.status), "error_code": int(terminal.result.error_code),
                "error_msg": terminal.result.error_msg, "wall_s": time.monotonic()-begun, "point_count": len(points),
                "raw_header": payload["header"], "path_sha256": path_content_sha256(payload), "outside_tiles": outside,
                "x_range_m": [min(p[0] for p in points), max(p[0] for p in points)] if points else None,
                "guide_count": len(request.goals) if method == "through_poses" else 0}
            row["geometry_passed"] = bool(terminal.status == 4 and terminal.result.error_code == 0 and len(points) >= 2
                and payload["header"]["frame_id"] == "map" and not outside
                and math.dist(points[0], (start["x_m"], start["y_m"])) <= .15
                and math.dist(points[-1], (chunk.goal_pose.x, chunk.goal_pose.y)) <= .15
                and angle_error(path_final_yaw(payload), chunk.goal_pose.yaw) <= .05)
            try:
                qualify_computed_path(book, chunk, points, frame_id=payload["header"]["frame_id"],
                    stamp_s=path_header_stamp_ns(payload["header"])/1e9, now_s=node.get_clock().now().nanoseconds/1e9,
                    start_xy=(start["x_m"], start["y_m"]))
                row["actual_clock_admission"] = "passed"
            except ValueError as error:
                row["actual_clock_admission"] = str(error)
            report["cases"][method] = row
            save(args.output/"result.json", report)
        report["status"] = "passed" if report["cases"]["through_poses"]["geometry_passed"] else "failed"
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
        report["child_exit_codes"] = []
        for process in reversed(processes):
            if process.poll() is None: process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
            report["child_exit_codes"].append({"pid": process.pid, "returncode": process.returncode})
        for stream in streams: stream.close()
        save(args.output/"result.json", report)
    return 0 if report["status"] == "passed" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen-stage", type=Path, required=True)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--linux-worker", action="store_true")
    args = parser.parse_args()
    return worker(args) if args.linux_worker else host(args)


if __name__ == "__main__":
    raise SystemExit(main())
