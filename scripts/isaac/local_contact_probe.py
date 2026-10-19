#!/usr/bin/env python3
"""Bounded native contact-sensor experiment; never a four-AMR success claim."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback


def save(path: Path, value) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if args.output.exists() or not args.output.is_absolute():
        parser.error("output must be a new absolute directory")
    cfg = json.loads(args.config.read_text(encoding="utf-8-sig"))
    if cfg["schema_version"] != 1 or len(args.run_id) != 32:
        parser.error("invalid component probe identity")
    args.output.mkdir(parents=True)
    report = {"run_id": args.run_id, "status": "starting", "stage": "native_sdk_startup",
        "started_at_utc": datetime.now(timezone.utc).isoformat(), "config": cfg,
        "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": cfg["scope"], "python": sys.version,
        "amr_docking_verified": False, "four_amr_verified": False, "charging_power_verified": False,
        "navigation_arrival": "not used; this is a vertical rigid-body contact coupon",
        "isaac_version": (Path(os.environ["ISAAC_PATH"]) / "VERSION").read_text().strip(),
        "media": [], "transitions": [], "requires_host_closure": True}
    save(args.output / "result.json", report)
    app = None
    code = 1
    try:
        from isaacsim import SimulationApp
        width, height = cfg["image_size_px"]
        app = SimulationApp({"headless": True, "width": width, "height": height,
                             "multi_gpu": False, "renderer": "RayTracedLighting"})
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.sensors.physics import ContactSensor
        from pxr import UsdLux, UsdGeom
        import omni.usd

        hz = cfg["physics_hz"]
        dt = 1. / hz
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt)
        body_path, pad_path = "/World/ProbeBody", "/World/ContactPad"
        world.scene.add(FixedCuboid(prim_path=pad_path, name="contact_pad", size=1.,
            scale=np.array(cfg["pad_dimensions_m"]), position=np.array(cfg["pad_position_m"]),
            color=np.array([.2, .55, .4])))
        body = world.scene.add(DynamicCuboid(prim_path=body_path, name="probe_body", size=1.,
            scale=np.array(cfg["body_dimensions_m"]), position=np.array(cfg["body_initial_position_m"]),
            mass=cfg["body_mass_kg"], color=np.array([.1, .35, .8])))
        sensor = world.scene.add(ContactSensor(prim_path=body_path + "/ContactSensor", name="contact_sensor",
            dt=dt, min_threshold=cfg["minimum_contact_force_n"], max_threshold=1000000., radius=-1.))
        sensor.add_raw_contact_data_to_frame()
        stage = omni.usd.get_context().get_stage()
        light = UsdLux.DomeLight.Define(stage, "/World/Light")
        light.CreateIntensityAttr(1000.)
        ground = UsdGeom.Cube.Define(stage, "/World/DisplayFloor")
        ground.CreateSizeAttr(1.)
        ground.AddScaleOp().Set((3., 3., .02))
        ground.AddTranslateOp().Set((0., 0., -.03))
        camera = rep.create.camera(position=(1.5, 1.5, 1.15), look_at=(0., 0., .25))
        product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        world.reset()
        report.update(status="running", stage="contact_acquisition")
        save(args.output / "result.json", report)
        stage.GetRootLayer().Export(str(args.output / "scene.usda"))
        last_time, last_step = -1., -1
        contact_since = separated_since = None
        released = confirmed = False
        captures = set()
        samples = 0
        with (args.output / "observations.jsonl").open("x", encoding="utf-8") as stream:
            for index in range(int(hz * cfg["maximum_sim_time_s"])):
                world.step(render=True)
                stamp = float(world.current_time)
                frame = sensor.get_current_frame()
                sensor_time, sensor_step = float(frame["time"]), int(frame["physics_step"])
                fresh = (sensor_time > last_time and sensor_step > last_step
                         and -1e-9 <= stamp - sensor_time <= 2. * dt)
                pairs = [[str(c["body0"]), str(c["body1"])] for c in frame.get("contacts", [])]
                paired = any(set(pair) == {body_path, pad_path} for pair in pairs)
                linear = float(np.linalg.norm(body.get_linear_velocity()))
                angular = float(np.linalg.norm(body.get_angular_velocity()))
                force = float(frame.get("force", 0.))
                stationary = linear <= cfg["stationary_linear_m_s"] and angular <= cfg["stationary_angular_rad_s"]
                contact = fresh and paired and bool(frame.get("in_contact")) and force >= cfg["minimum_contact_force_n"]
                transition = None
                if not released:
                    if contact and stationary:
                        if contact_since is None:
                            contact_since = sensor_time
                        if not confirmed and sensor_time - contact_since >= cfg["contact_dwell_s"]:
                            confirmed = True
                            transition = "stationary_pair_contact_confirmed"
                    else:
                        contact_since = None
                    if confirmed and contact_since is not None and sensor_time - contact_since >= cfg["contact_dwell_s"] + .25:
                        body.set_linear_velocity(np.array([0., 0., cfg["release_velocity_z_m_s"]]))
                        released = True
                        transition = "authored_release_velocity_applied"
                elif fresh and not paired and not frame.get("in_contact"):
                    if separated_since is None:
                        separated_since = sensor_time
                    if sensor_time - separated_since >= cfg["separation_dwell_s"]:
                        transition = "separation_observed"
                else:
                    separated_since = None
                sample = {"run_id": args.run_id, "physics_frame": index + 1, "sim_time_s": stamp,
                    "sensor_time_s": sensor_time, "sensor_step": sensor_step, "fresh": fresh,
                    "in_contact": bool(frame.get("in_contact")), "expected_pair": paired, "pairs": pairs,
                    "force_n": force, "linear_m_s": linear, "angular_rad_s": angular,
                    "position_m": body.get_world_pose()[0].tolist(), "transition": transition}
                if index % 12 == 0 or transition:
                    stream.write(json.dumps(sample, allow_nan=False) + "\n")
                    stream.flush()
                    samples += 1
                if transition:
                    report["transitions"].append(sample)
                capture = "initial" if index == 15 else "contact" if transition == "stationary_pair_contact_confirmed" else "separated" if transition == "separation_observed" else None
                if capture and capture not in captures:
                    data = np.asarray(rgb.get_data())
                    if data.size:
                        image_path = args.output / (capture + ".png")
                        Image.fromarray(data[:, :, :3].astype(np.uint8)).save(image_path)
                        report["media"].append({"path": image_path.name, "sim_time_s": stamp,
                            "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest()})
                        captures.add(capture)
                last_time, last_step = sensor_time, sensor_step
                if transition == "separation_observed":
                    break
        passed = confirmed and any(t["transition"] == "separation_observed" for t in report["transitions"])
        report.update(status="passed" if passed else "failed", stage="component_complete", samples=samples,
            contact_sensor_source="native PhysX ContactSensor raw actor pair, force, time and physics step",
            body_velocity_source="PhysX rigid-body velocity; not wheel odometry", final_sim_time_s=stamp)
        code = 0 if passed else 1
    except BaseException as error:
        report.update(status="failed", stage="exception", error={"type": type(error).__name__,
            "message": str(error), "traceback": traceback.format_exc(limit=16)})
    finally:
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        save(args.output / "result.json", report)
        print("LOCAL_CONTACT_COMPONENT_RESULT", report["status"], flush=True)
        if app is not None:
            app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
