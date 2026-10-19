"""Bounded SDK4.5 RTX LaserScan bridge component, without vehicle controls."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time


def validate_scan_ack(ack, run_id, cfg, config_sha256, receiver_sha256):
    if (ack.get("run_id") != run_id or ack.get("status") != "passed"
            or ack.get("config_sha256") != config_sha256
            or ack.get("source_sha256") != receiver_sha256):
        raise ValueError("receiver identity/source/config mismatch")
    messages = ack.get("messages")
    errors = ack.get("scan_clock_nearest_errors_s")
    if (not isinstance(messages, list) or len(messages) < cfg["minimum_messages"]
            or not isinstance(errors, list) or len(errors) != len(messages)):
        raise ValueError("insufficient scans or missing clock checks")
    import math
    previous = 0.
    for sample, error in zip(messages, errors):
        stamp = sample.get("stamp_s")
        if (type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= previous
                or sample.get("frame_id") != "probe_lidar_" + run_id
                or sample.get("covered_half_degree_sectors") != 720
                or type(error) not in (int, float) or not math.isfinite(error) or not 0 <= error <= .05):
            raise ValueError("scan identity, chronology, coverage or clock check failed")
        previous = stamp


def run(app, ros, node, output, run_id, cfg, report, save):
    # Called only after SimulationApp and the bundled ROS extension are ready.
    import numpy as np
    import omni.kit.commands
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.core.api import World
    from pxr import Gf, UsdGeom, UsdPhysics
    from rosgraph_msgs.msg import Clock

    world = product = writer = annotator = None
    source = Path(__file__).resolve()
    receiver_path = source.parents[2] / "scripts/ros/local_dds_receiver.py"
    receiver_sha256 = hashlib.sha256(receiver_path.read_bytes()).hexdigest()
    report["project_source_sha256"] = {
        "scripts/isaac/local_lidar_bridge.py": hashlib.sha256(source.read_bytes()).hexdigest(),
        "scripts/ros/local_dds_receiver.py": receiver_sha256}
    report.update(stage="rtx_scan_setup", lidar_verified=False,
                  scope="SDK4.5 RTX scan and sim clock to WSL; static authored room, no vehicle")
    save(output / "result.json", report)
    try:
        if cfg["render_hz"] != 10 or cfg["physics_hz"] != 60:
            raise ValueError("scan admission requires 10Hz render and 60Hz physics")
        world = World(stage_units_in_meters=1., physics_dt=1./cfg["physics_hz"],
                      rendering_dt=1./cfg["render_hz"],
                      backend="numpy", device="cpu")
        world.get_physics_context().enable_gpu_dynamics(False)
        world.scene.add_ground_plane(size=12., color=np.array([.6, .6, .6]))
        stage = omni.usd.get_context().get_stage()
        # Four authored planar wall faces at x/y +/-4m; sensor at room centre.
        for name, xyz, scale in [
                ("east", (4.1, 0., 1.), (.2, 8.4, 2.)),
                ("west", (-4.1, 0., 1.), (.2, 8.4, 2.)),
                ("north", (0., 4.1, 1.), (8.4, .2, 2.)),
                ("south", (0., -4.1, 1.), (8.4, .2, 2.))]:
            cube = UsdGeom.Cube.Define(stage, "/World/" + name)
            cube.CreateSizeAttr(1.)
            cube.AddScaleOp().Set(Gf.Vec3f(*scale))
            cube.AddTranslateOp().Set(Gf.Vec3d(*xyz))
            # Translation must precede scale in the composed transform.
            ops = cube.GetOrderedXformOps()
            cube.SetXformOpOrder([ops[1], ops[0]])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        ok, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar", path="/World/ProbeLidar", parent=None,
            config=cfg["lidar_profile"], translation=(0., 0., 1.),
            orientation=Gf.Quatd(1., 0., 0., 0.))
        if not ok or sensor is None or not sensor.IsValid():
            raise RuntimeError("RTX sensor creation failed")
        product = rep.create.render_product(sensor.GetPath(), (1, 1), name="Local45Scan")
        writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        topic = "/factory_ops_local45/scan_" + run_id
        frame = "probe_lidar_" + run_id
        writer.initialize(topicName=topic, frameId=frame)
        writer.attach([product])
        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacComputeRTXLidarFlatScanSimulationTime")
        annotator.attach([product])
        clock_topic = "/factory_ops_local45/clock_" + run_id
        clock_publisher = node.create_publisher(Clock, clock_topic, 20)
        report.update(stage="rtx_scan_stream", scan_topic=topic, frame_id=frame,
                      clock_topic=clock_topic, profile=cfg["lidar_profile"],
                      render_hz=cfg["render_hz"], physics_hz=cfg["physics_hz"],
                      geometry={"authored_inner_wall_m": 4., "sensor_position_m": [0., 0., 1.]},
                      writer="RtxLidarROS2PublishLaserScan (simulation time)")
        sdk = Path(__import__("os").environ["ISAAC_PATH"])
        profile_paths = [sdk / "exts/isaacsim.sensors.rtx/data/lidar_configs/SLAMTEC/RPLIDAR_S2E.json"]
        if cfg["lidar_profile"] != "RPLIDAR_S2E" or not profile_paths[0].is_file():
            raise RuntimeError("installed profile provenance is ambiguous")
        report["installed_profile"] = {"path": str(profile_paths[0]),
            "sha256": hashlib.sha256(profile_paths[0].read_bytes()).hexdigest()}
        save(output / "result.json", report)
        world.reset()
        # RTX startup may outlast discovery. Start the receiver's sample window
        # only after the native world is ready, then wait for its explicit arm.
        save(output / "sensor-stream-ready.json", {"run_id": run_id})
        arm_deadline = time.monotonic() + 30
        armed = output / "sensor-receiver-armed.json"
        while not armed.is_file():
            if time.monotonic() >= arm_deadline:
                raise TimeoutError("sensor receiver did not arm after world initialization")
            time.sleep(.02)
        if json.loads(armed.read_text(encoding="utf-8"))["run_id"] != run_id:
            raise ValueError("sensor receiver armed another run")
        wall_deadline = time.monotonic() + cfg["stream_wall_s"]
        count = 0
        best_valid_count = -1
        with (output / "physics-clock.jsonl").open("x", encoding="utf-8") as stream:
            while count < 100 and time.monotonic() < wall_deadline:
                if not app.is_running():
                    raise RuntimeError("SDK closed during sensor stream")
                world.step(render=True)
                stamp = float(world.current_time)
                data = annotator.get_data()
                depths = np.asarray(data.get("linearDepthData", [])).reshape(-1)
                if depths.size:
                    valid_count = int(np.count_nonzero(np.isfinite(depths) & (depths >= .05) & (depths <= 30.)))
                    diagnostic = {"run_id": run_id, "read_at_sim_time_s": stamp,
                        "render_updates": count + 1, "buffer_fields": sorted(data),
                        "valid_count": valid_count, "size": int(depths.size),
                        "raw_ranges": [float(x) if np.isfinite(x) else str(float(x)) for x in depths]}
                    save(output / "last-native-scan.json", diagnostic)
                    if valid_count > best_valid_count:
                        best_valid_count = valid_count
                        save(output / "best-native-scan.json", diagnostic)
                message = Clock()
                message.clock.sec = int(stamp)
                message.clock.nanosec = round((stamp - int(stamp)) * 1_000_000_000)
                if message.clock.nanosec == 1_000_000_000:
                    message.clock.sec += 1
                    message.clock.nanosec = 0
                clock_publisher.publish(message)
                stream.write(json.dumps({"run_id": run_id, "step": count,
                                         "sim_time_s": stamp}) + "\n")
                stream.flush()
                count += 1
                ros.spin_once(node, timeout_sec=0.)
                acknowledgement = output / "receiver-result.json"
                if acknowledgement.is_file():
                    ack = json.loads(acknowledgement.read_text(encoding="utf-8"))
                    validate_scan_ack(ack, run_id, cfg, report["config_sha256"], receiver_sha256)
                    report.update(status="passed", lidar_verified=False, lidar_transport_verified=True,
                                  raw_fill_95_percent_passed=all(s["raw_fill_95_percent_passed"] for s in ack["messages"]),
                                  quality_scope="angular coverage and wall-distance/clock transport component; raw dropout unresolved",
                                  receiver_count=len(ack["messages"]), render_updates=count,
                                  physics_steps=int(world.current_time_step_index),
                                  last_sim_time_s=stamp)
                    return
                # Leave wall time for the independent WSL subscriber to run.
                time.sleep(.005)
        raise TimeoutError("three valid scans and clock alignment were not acknowledged")
    finally:
        cleanup = []
        if writer is not None:
            try:
                writer.detach()
            except BaseException as error:
                cleanup.append("writer.detach: " + repr(error))
        if annotator is not None:
            try:
                annotator.detach()
            except BaseException as error:
                cleanup.append("annotator.detach: " + repr(error))
        if product is not None:
            try:
                product.destroy()
            except BaseException as error:
                cleanup.append("product.destroy: " + repr(error))
        if world is not None:
            try:
                world.stop()
            except BaseException as error:
                cleanup.append("world.stop: " + repr(error))
        report["sensor_cleanup_errors"] = cleanup
        if cleanup:
            raise RuntimeError("sensor cleanup failed: " + repr(cleanup))
