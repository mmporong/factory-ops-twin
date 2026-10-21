"""SDK4.5 dynamic carrier/tote on fixed stands; not a robot loading admission."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
import traceback


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if (not args.output.is_absolute() or args.output.exists() or args.output.is_symlink()
            or not re.fullmatch("[0-9a-f]{32}", args.run_id)):
        parser.error("new absolute output and 32 lowercase hex run-id required")
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    from factory_ops.carrier_module import build_manifest, load_profile, manifest_json, manifest_sha256

    manifest = build_manifest(load_profile(args.config))
    cfg, contact = manifest["probe"], manifest["contact"]
    args.output.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    report = {"schema_version": 1, "run_id": args.run_id, "status": "starting", "stage": "startup",
        "source_sha256": sha(Path(__file__)), "config_sha256": sha(args.config),
        "project_source_sha256": {name: sha(root/name) for name in
            ("factory_ops/__init__.py", "factory_ops/carrier_module.py")},
        "manifest_sha256": manifest_sha256(manifest), "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "simplified fixed-stand fixture with two dynamic compound bodies; no Nova Carter loaded",
        "robot_integration_verified": False, "transport_verified": False, "charging_verified": False,
        "industrial_payload_rating_verified": False, "native_measurements": {}, "transitions": [],
        "media": [], "cleanup_errors": [], "requires_host_closure": True}
    (args.output/"manifest.json").write_text(manifest_json(manifest), encoding="utf-8")
    save(args.output/"result.json", report)
    started = time.monotonic()
    deadline = started+cfg["maximum_wall_time_s"]
    app = world = rgb = product = None
    code = 1

    def bounded() -> None:
        if time.monotonic() >= deadline:
            raise TimeoutError("carrier fixture wall deadline exceeded; host guard still required")

    try:
        from isaacsim import SimulationApp
        width, height = cfg["image_size_px"]
        app = SimulationApp({"headless": True, "width": width, "height": height,
                             "multi_gpu": False, "renderer": "RayTracedLighting"})
        bounded()
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleRigidPrim
        from isaacsim.sensors.physics import ContactSensor
        from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade

        report["isaac_version"] = (Path(os.environ["ISAAC_PATH"])/"VERSION").read_text().strip()
        dt = cfg["physics_dt_s"]
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt, backend="numpy", device="cpu")
        stage = omni.usd.get_context().get_stage()
        material = UsdShade.Material.Define(stage, "/World/CarrierMaterial")
        physical = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physical.CreateStaticFrictionAttr(contact["static_friction"])
        physical.CreateDynamicFrictionAttr(contact["dynamic_friction"])
        physical.CreateRestitutionAttr(contact["restitution"])
        owners = {}

        def cube(path, size, center, body, color):
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1.)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center))
            shape.AddScaleOp().Set(Gf.Vec3d(*size))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            prim = shape.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(contact["contact_offset_m"])
            collision.CreateRestOffsetAttr(contact["rest_offset_m"])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
            owners[path] = body
            return prim

        bodies, paths, sensors = {}, {"carrier": "/World/Carrier", "tote": "/World/Tote"}, {}
        for key, path in paths.items():
            transform = UsdGeom.Xform.Define(stage, path)
            UsdPhysics.RigidBodyAPI.Apply(transform.GetPrim()).CreateRigidBodyEnabledAttr(True)
            props = manifest["mass_properties"][key]
            inertia = props["inertia_matrix_kg_m2"]
            if any(abs(inertia[i][j]) > 1e-10 for i in range(3) for j in range(3) if i != j):
                raise ValueError("fixture requires axis-aligned principal inertia")
            mass = UsdPhysics.MassAPI.Apply(transform.GetPrim())
            mass.CreateMassAttr(props["mass_kg"])
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*props["center_of_mass_m"]))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*(inertia[i][i] for i in range(3))))
            mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
            for box in (b for b in manifest["geometry"] if b["body"] == key):
                cube(path+"/"+box["name"], box["size_m"], box["center_m"], key,
                     (.12, .32, .65) if key == "carrier" else (.22, .65, .48))
            position = [0., 0., cfg["initial_gap_m"]] if key == "carrier" else list(manifest["tote_rest_origin_m"])
            if key == "tote":
                position[2] += 2*cfg["initial_gap_m"]
            bodies[key] = world.scene.add(SingleRigidPrim(prim_path=path, name=key, position=np.array(position)))
            owners[path] = key
            # ContactSensor reports raw actor paths, which may be the compound root.
            sensors[key] = world.scene.add(ContactSensor(prim_path=path+"/ContactSensor", name=key+"_contact",
                dt=dt, radius=-1., min_threshold=cfg["minimum_force_n"], max_threshold=1e6))
            sensors[key].add_raw_contact_data_to_frame()
        stand_ids = set()
        for i, post in enumerate(b for b in manifest["geometry"] if b["name"].startswith("post_")):
            path = "/World/Stand_%d" % i
            z = manifest["mount_plane_z_m"]
            stand = cube(path, [post["size_m"][0]*2, post["size_m"][1]*2, z],
                 [post["center_m"][0], post["center_m"][1], z/2], "stand_%d" % i, (.55, .59, .66))
            support_body = UsdPhysics.RigidBodyAPI.Apply(stand)
            support_body.CreateRigidBodyEnabledAttr(True)
            support_body.CreateKinematicEnabledAttr(True)
            key = "stand_%d" % i
            stand_ids.add(key)
            sensors[key] = world.scene.add(ContactSensor(prim_path=path+"/ContactSensor", name=key+"_contact",
                dt=dt, radius=-1., min_threshold=cfg["minimum_force_n"], max_threshold=1e6))
            sensors[key].add_raw_contact_data_to_frame()
        UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1300.)
        # Initialize physics before creating persistent render products.
        world.reset()
        bounded()
        camera = rep.create.camera(position=(1.5, 1.5, 1.35), look_at=(-.18, 0., .6))
        product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        stage.GetRootLayer().Export(str(args.output/"scene.usda"))
        report.update(status="running", stage="settling")
        save(args.output/"result.json", report)
        previous_time, previous_frame = float(world.current_time), int(world.current_time_step_index)
        last_sensor = {k: (-1., -1) for k in sensors}
        contact_since = separate_since = None
        settled = released = separated = False
        sample_count = 0

        def capture(label, stamp, physics_frame):
            data = np.asarray(rgb.get_data())
            if (data.ndim != 3 or data.shape[:2] != (height, width) or data.shape[2] < 3
                    or data.dtype != np.uint8 or float(data[:, :, :3].std()) < 1.):
                raise RuntimeError("RGB buffer unavailable or uniform; no synthetic image saved")
            path = args.output/(label+".png")
            Image.fromarray(data[:, :, :3]).save(path)
            report["media"].append({"path": path.name, "sha256": sha(path), "world_checkpoint_s": stamp,
                "world_checkpoint_frame": physics_frame, "timing_scope": "read after rendered physics sample; exposure clock not independently validated"})

        with (args.output/"observations.jsonl").open("x", encoding="utf-8") as stream:
            for _ in range(math.ceil(cfg["maximum_sim_time_s"]/dt)):
                bounded()
                world.step(render=True)
                bounded()
                stamp, frame = float(world.current_time), int(world.current_time_step_index)
                if (not math.isfinite(stamp) or frame != previous_frame+1 or stamp <= previous_time
                        or abs(stamp-previous_time-dt) > 1e-6):
                    raise RuntimeError("native World clock/frame progression mismatch")
                states, contacts = {}, {}
                for key, body in bodies.items():
                    position, rotation = body.get_world_pose()
                    linear, angular = body.get_linear_velocity(), body.get_angular_velocity()
                    values = [*position, *rotation, *linear, *angular]
                    if not all(math.isfinite(float(v)) for v in values):
                        raise ValueError("nonfinite native body observation")
                    states[key] = {"position_m": position.tolist(), "orientation_wxyz": rotation.tolist(),
                        "linear_velocity_m_s": linear.tolist(), "angular_velocity_rad_s": angular.tolist()}
                for key, sensor in sensors.items():
                    raw = sensor.get_current_frame()
                    sensor_time, sensor_frame = float(raw["time"]), int(raw["physics_step"])
                    if (not math.isfinite(sensor_time) or sensor_time < last_sensor[key][0]
                            or sensor_frame < last_sensor[key][1]):
                        raise RuntimeError("native contact clock regressed or is invalid")
                    fresh = (sensor_time > last_sensor[key][0] and sensor_frame > last_sensor[key][1]
                             and 0. <= stamp-sensor_time <= cfg["sensor_freshness_s"])
                    force = float(raw.get("force", 0.))
                    if not math.isfinite(force) or force < 0:
                        raise ValueError("invalid contact force")
                    pairs = [[str(p["body0"]), str(p["body1"])] for p in raw.get("contacts", [])]
                    mapped = [sorted([owners.get(a, "UNKNOWN:"+a), owners.get(b, "UNKNOWN:"+b)]) for a, b in pairs]
                    if any(any(v.startswith("UNKNOWN:") for v in pair) for pair in mapped):
                        raise RuntimeError("unexpected actor in isolated contact fixture")
                    contacts[key] = {"sensor_time_s": sensor_time, "sensor_physics_step": sensor_frame,
                        "fresh": fresh, "force_n": force, "in_contact": bool(raw.get("in_contact", False)),
                        "raw_actor_pairs": pairs, "body_pairs": mapped}
                    last_sensor[key] = (sensor_time, sensor_frame)
                supported = {key for key in stand_ids
                    if sorted(["carrier", key]) in contacts[key]["body_pairs"]
                    and contacts[key]["fresh"] and contacts[key]["in_contact"]
                    and contacts[key]["force_n"] >= cfg["minimum_force_n"]}
                tote_pair = ["carrier", "tote"] in contacts["tote"]["body_pairs"]
                stationary = all(np.linalg.norm(s["linear_velocity_m_s"]) <= cfg["stationary_linear_m_s"]
                    and np.linalg.norm(s["angular_velocity_rad_s"]) <= cfg["stationary_angular_rad_s"] for s in states.values())
                surface_gap = states["tote"]["position_m"][2]-states["carrier"]["position_m"][2]-manifest["tote_rest_origin_m"][2]
                error = cfg["maximum_settle_error_m"]
                aligned = (abs(states["carrier"]["position_m"][2]) <= error and abs(surface_gap) <= error
                    and all(np.linalg.norm(np.array(states[k]["position_m"][:2])-np.array(target)) <= error
                            for k, target in (("carrier", [0., 0.]), ("tote", manifest["tote_rest_origin_m"][:2])))
                    and all(2*math.atan2(np.linalg.norm(s["orientation_wxyz"][1:]), abs(s["orientation_wxyz"][0])) <= .05 for s in states.values()))
                qualified_contact = (len(supported) == 4 and tote_pair and stationary and aligned
                    and all(c["fresh"] and c["in_contact"] and c["force_n"] >= cfg["minimum_force_n"] for c in contacts.values()))
                transition = None
                if not released:
                    contact_since = (stamp if contact_since is None else contact_since) if qualified_contact else None
                    if contact_since is not None and stamp-contact_since >= cfg["contact_dwell_s"]:
                        settled = True
                        transition = "supported_stationary_contact_confirmed"
                elif contacts["tote"]["fresh"] and not tote_pair and not contacts["tote"]["in_contact"] and surface_gap > .02:
                    separate_since = stamp if separate_since is None else separate_since
                    if stamp-separate_since >= cfg["separation_dwell_s"]:
                        separated = True
                        transition = "authored_release_separation_observed"
                else:
                    separate_since = None
                sample = {"run_id": args.run_id, "world_time_s": stamp, "world_physics_frame": frame,
                    "phase": "release" if released else "settling", "bodies": states, "contacts": contacts,
                    "support_count": len(supported), "tote_surface_gap_m": surface_gap,
                    "stationary": bool(stationary), "aligned": bool(aligned), "transition": transition}
                stream.write(json.dumps(sample, allow_nan=False)+"\n")
                stream.flush()
                sample_count += 1
                if transition:
                    report["transitions"].append(sample)
                    capture("settled" if settled and not released else "separated", stamp, frame)
                if settled and not released:
                    bodies["tote"].set_linear_velocity(np.array([0., 0., cfg["release_velocity_z_m_s"]]))
                    released = True
                    report["transitions"].append({"transition": "authored_release_velocity_applied",
                        "world_time_s": stamp, "world_physics_frame": frame,
                        "velocity_z_m_s": cfg["release_velocity_z_m_s"]})
                previous_time, previous_frame = stamp, frame
                if separated:
                    break
        passed = settled and separated and len(report["media"]) == 2
        report.update(status="passed" if passed else "failed", stage="fixture_complete",
            native_measurements={"samples": sample_count, "settled_contact_observed": settled,
                "separation_observed": separated, "last_world_time_s": previous_time, "last_world_frame": previous_frame},
            contact_source="PhysX ContactSensor raw actor pairs/force/time/physics step",
            velocity_source="PhysX rigid-body velocity, not encoder odometry")
        code = 0 if passed else 1
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
                      "traceback": traceback.format_exc(limit=12)})
    finally:
        for label, resource, operation in (("rgb", rgb, "detach"), ("render_product", product, "destroy")):
            if resource is not None:
                try:
                    getattr(resource, operation)()
                except BaseException as error:
                    report["cleanup_errors"].append(label+": "+str(error))
        if report["cleanup_errors"]:
            report["status"], code = "failed", 1
        report.update(finished_at_utc=datetime.now(timezone.utc).isoformat(),
                      elapsed_wall_s=time.monotonic()-started, app_close_requested=app is not None)
        try:
            save(args.output/"result.json", report)
            print("TOTE_CARRIER_COMPONENT_RESULT", report["status"], flush=True)
        finally:
            if app is not None:
                app.close(wait_for_replicator=False)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
