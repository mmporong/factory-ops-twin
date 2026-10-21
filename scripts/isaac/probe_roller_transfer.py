"""Bounded SDK4.5 roller/stopper coupon, not an AMR or ledger admission.

No pose/velocity setters are used. Motor control is limited to native angular
joint drive targets. All body ground truth is evaluation-only.
"""
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


def safe_value(value):
    """Preserve invalid evidence explicitly, never replace it with valid data."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_numeric_repr": repr(value)}
    if isinstance(value, dict):
        return {str(k): safe_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_value(v) for v in value]
    return value


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
    from factory_ops.carrier_module import manifest_json, manifest_sha256
    from factory_ops.roller_transfer import TransferController, build_transfer_manifest, motor_degrees

    manifest = build_transfer_manifest(json.loads(args.config.read_text(encoding="utf-8-sig")))
    cfg, g, drive_cfg = (manifest[k] for k in ("control", "geometry", "drive"))
    args.output.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    report = {"schema_version": 1, "run_id": args.run_id, "status": "starting", "stage": "startup",
        "source_sha256": sha(Path(__file__)), "config_sha256": sha(args.config),
        "project_source_sha256": {name: sha(root/name) for name in
            ("factory_ops/__init__.py", "factory_ops/carrier_module.py", "factory_ops/roller_transfer.py")},
        "manifest_sha256": manifest_sha256(manifest), "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": manifest["scope"], "native_transfer_verified": False,
        "robot_mount_verified": False, "inventory_or_ownership_changed": False,
        "industrial_rating_verified": False, "stage2_complete": False,
        "actuator": "UsdPhysics.RevoluteJoint angular force drive; velocity target deg/s, torque N*m",
        "contact_source": "PhysX ContactSensor force/raw actor pairs/time/physics_step, per roller and stopper",
        "ground_truth_scope": "evaluation and failure diagnosis only, excluded from controller input",
        "transitions": [], "transfer_events": [], "media": [], "cleanup_errors": [],
        "requires_host_closure": True}
    (args.output/"manifest.json").write_text(manifest_json(manifest), encoding="utf-8")
    save(args.output/"result.json", report)
    started = time.monotonic()
    deadline = started+cfg["maximum_wall_time_s"]
    app = world = product = rgb = controller = None
    drives, roller_bodies = {}, {}
    last_sample = None
    code = 1

    def bounded():
        if time.monotonic() >= deadline:
            raise TimeoutError("roller fixture wall deadline exceeded; host Job still required")

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
        physics = world.get_physics_context()
        physics.enable_gpu_dynamics(False)
        physics.set_broadphase_type("MBP")
        physics.set_gravity(-9.81)
        stage = omni.usd.get_context().get_stage()
        material = UsdShade.Material.Define(stage, "/World/RollerMaterial")
        friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        friction.CreateStaticFrictionAttr(manifest["contact"]["static_friction"])
        friction.CreateDynamicFrictionAttr(manifest["contact"]["dynamic_friction"])
        friction.CreateRestitutionAttr(manifest["contact"]["restitution"])
        owners, sensors, stopper_prims = {}, {}, {}

        def collision(prim, actor):
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(manifest["contact"]["contact_offset_m"])
            api.CreateRestOffsetAttr(manifest["contact"]["rest_offset_m"])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
            owners[str(prim.GetPath())] = actor

        def cube(path, size, center, actor, color):
            shape = UsdGeom.Cube.Define(stage, path)
            shape.CreateSizeAttr(1.)
            shape.AddTranslateOp().Set(Gf.Vec3d(*center))
            shape.AddScaleOp().Set(Gf.Vec3d(*size))
            shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            collision(shape.GetPrim(), actor)

        def body(actor, position, mass=None, diagonal=None, com=(0., 0., 0.), fixed=False):
            path = "/World/"+actor
            xf = UsdGeom.Xform.Define(stage, path)
            xf.AddTranslateOp().Set(Gf.Vec3d(*position))
            api = UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
            api.CreateRigidBodyEnabledAttr(True)
            api.CreateKinematicEnabledAttr(fixed)
            if mass is not None:
                props = UsdPhysics.MassAPI.Apply(xf.GetPrim())
                props.CreateMassAttr(mass)
                props.CreateCenterOfMassAttr(Gf.Vec3f(*com))
                props.CreateDiagonalInertiaAttr(Gf.Vec3f(*diagonal))
                props.CreatePrincipalAxesAttr(Gf.Quatf(1.))
            owners[path] = actor
            return path, xf.GetPrim()

        def sensor(actor, path):
            obj = world.scene.add(ContactSensor(prim_path=path+"/ContactSensor", name=actor+"_contact",
                dt=dt, radius=-1., min_threshold=cfg["minimum_force_n"], max_threshold=1e6))
            obj.add_raw_contact_data_to_frame()
            sensors[actor] = obj

        for row in manifest["rollers"]:
            actor = row["actor_id"]
            path, _ = body(actor, row["center_m"], row["mass_kg"], row["diagonal_inertia_kg_m2"])
            cylinder = UsdGeom.Cylinder.Define(stage, path+"/Cylinder")
            cylinder.CreateAxisAttr("Y")
            cylinder.CreateRadiusAttr(row["radius_m"])
            cylinder.CreateHeightAttr(row["length_m"])
            cylinder.CreateDisplayColorAttr([Gf.Vec3f(.25, .40, .60) if row["zone"] == "carrier" else Gf.Vec3f(.45, .50, .55)])
            collision(cylinder.GetPrim(), actor)
            joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint_"+actor)
            # Empty body0 means world. This is a stationary coupon, not a
            # mobile attachment; no such world axle is installed on an AMR.
            joint.CreateBody1Rel().SetTargets([path])
            joint.CreateAxisAttr("Y")
            joint.CreateLocalPos0Attr(Gf.Vec3f(*row["center_m"]))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0., 0., 0.))
            joint.CreateLocalRot0Attr(Gf.Quatf(1.))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.))
            joint.CreateCollisionEnabledAttr(False)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(0.)
            drive.CreateDampingAttr(drive_cfg["usd_damping_n_m_per_deg_s"])
            drive.CreateMaxForceAttr(drive_cfg["torque_limit_n_m"])
            drive.CreateTargetVelocityAttr(0.)
            drives[actor] = drive
            roller_bodies[actor] = world.scene.add(SingleRigidPrim(prim_path=path, name=actor))
            sensor(actor, path)
        for row in manifest["stoppers"]:
            actor = row["actor_id"]
            path, prim = body(actor, row["center_m"], fixed=True)
            cube(path+"/Collider", row["size_m"], [0., 0., 0.], actor, (.85, .55, .16))
            stopper_prims[actor] = prim
            sensor(actor, path)
        # Side rails begin ABOVE the roller tops, never intersecting the axles.
        for sign, actor in ((-1, "rail_m"), (1, "rail_p")):
            path, _ = body(actor, [g["transfer_gap_m"]/2, sign*(g["rail_inner_y_m"]+.01),
                                  g["roller_top_z_m"]+.09], fixed=True)
            cube(path+"/Collider", [2*g["zone_length_m"]+g["transfer_gap_m"], .02, .15],
                 [0., 0., 0.], actor, (.60, .65, .7))
        props = manifest["tote_mass_properties"]
        inertia = props["inertia_matrix_kg_m2"]
        if any(abs(inertia[i][j]) > 1e-10 for i in range(3) for j in range(3) if i != j):
            raise ValueError("non-axis-aligned tote inertia unsupported by this fixture")
        path, _ = body(manifest["tote_id"], manifest["initial_tote_origin_m"], props["mass_kg"],
                        [inertia[i][i] for i in range(3)], props["center_of_mass_m"])
        for box in manifest["tote_geometry"]:
            cube(path+"/"+box["name"], box["size_m"], box["center_m"], manifest["tote_id"],
                 (.76, .48, .22) if box["name"] == "rigid_ballast" else (.18, .65, .46))
        tote = world.scene.add(SingleRigidPrim(prim_path=path, name=manifest["tote_id"]))
        UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(1400.)
        world.reset()  # Initialize World BEFORE persistent RGB render products.
        bounded()
        camera = rep.create.camera(position=(1.3, 1.5, 1.4), look_at=(.01, 0., .69))
        product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        stage.GetRootLayer().Export(str(args.output/"scene.usda"))
        controller = TransferController(manifest, args.run_id)
        previous_time, previous_frame = float(world.current_time), int(world.current_time_step_index)
        motor_targets = {a: 0. for a in drives}
        last_written = -1.
        prior_phase = None
        samples = 0
        armed = False
        sensor_clocks = {}

        def apply_targets(targets):
            for actor, target in targets.items():
                if not drives[actor].GetTargetVelocityAttr().Set(motor_degrees(target, drive_cfg["maximum_rad_s"])):
                    raise RuntimeError("joint target write failed: "+actor)

        def capture(label, stamp, frame):
            # Continuous RGB; no pause/resume or Replicator step/GT motion.
            data = np.asarray(rgb.get_data())
            if (data.dtype != np.uint8 or data.ndim != 3 or data.shape[:2] != (height, width)
                    or data.shape[2] < 3 or float(data[:, :, :3].std()) < 1.):
                raise RuntimeError("actual RGB buffer missing or uniform")
            media = args.output/(label+".png")
            Image.fromarray(data[:, :, :3]).save(media)
            report["media"].append({"path": media.name, "sha256": sha(media),
                "world_checkpoint_s": stamp, "world_physics_frame": frame,
                "timing_scope": "continuous RGB read at confirmation checkpoint; exposure clock not independently qualified"})

        report.update(status="running", stage="sensor_initialization")
        save(args.output/"result.json", report)
        with (args.output/"observations.jsonl").open("x", encoding="utf-8") as stream:
            while previous_time < cfg["maximum_sim_time_s"]:
                bounded()
                world.step(render=True)
                bounded()
                stamp, frame = float(world.current_time), int(world.current_time_step_index)
                if (not math.isfinite(stamp) or frame != previous_frame+1 or stamp <= previous_time
                        or abs(stamp-previous_time-dt) > 1e-6):
                    raise RuntimeError("actual World clock/frame progression invalid")
                previous_time, previous_frame = stamp, frame
                sensor_rows, raw_rows = {}, {}
                last_sample = {"run_id": args.run_id, "world_time_s": stamp,
                    "world_physics_frame": frame, "raw_sensors": raw_rows,
                    "controller_sensors": sensor_rows}
                for actor, obj in sensors.items():
                    raw = obj.get_current_frame()
                    # Preserve every examined field, including invalid actor paths.
                    pairs = [[str(p["body0"]), str(p["body1"])] for p in raw.get("contacts", [])]
                    row = {"time_s": float(raw["time"]), "frame": float(raw["physics_step"]),
                           "raw_actor_pairs": pairs}
                    if "force" in raw:
                        if isinstance(raw["force"], (bool, np.bool_)):
                            raise ValueError("boolean native force")
                        row["force_n"] = float(raw["force"])
                    if "in_contact" in raw: row["in_contact"] = bool(raw["in_contact"])
                    raw_rows[actor] = row
                    sf = row["frame"]
                    if not math.isfinite(sf) or sf != int(sf):
                        raise ValueError("invalid native sensor frame")
                    old = sensor_clocks.get(actor, (-1., -1))
                    if (not math.isfinite(row["time_s"]) or row["time_s"] < old[0]
                            or sf < old[1] or sf > frame or row["time_s"] > stamp
                            or stamp-row["time_s"] > cfg["sensor_freshness_s"]):
                        raise ValueError("invalid/stale/regressing native sensor clock: "+actor)
                    sensor_clocks[actor] = (row["time_s"], sf)
                    if "force_n" in row and (not math.isfinite(row["force_n"]) or row["force_n"] < 0):
                        raise ValueError("invalid native contact force: "+actor)
                    if any({owners.get(a), owners.get(b)} != {actor, manifest["tote_id"]} for a, b in pairs):
                        raise ValueError("unknown native sensor actor: "+actor)
                    sensor_rows[actor] = {k: v for k, v in row.items() if k != "raw_actor_pairs"}
                    sensor_rows[actor]["frame"] = int(sf)
                    sensor_rows[actor]["actor_pairs"] = [[owners.get(a, "UNKNOWN:"+a),
                                                            owners.get(b, "UNKNOWN:"+b)] for a, b in pairs]
                velocities, readbacks = {}, {}
                last_sample["motors"] = readbacks
                for actor, obj in roller_bodies.items():
                    angular = [float(v) for v in obj.get_angular_velocity()]
                    readbacks[actor] = {"angular_velocity_rad_s_xyz": angular,
                        "target_rad_s": motor_targets[actor],
                        "usd_target_deg_s": float(drives[actor].GetTargetVelocityAttr().Get()),
                        "usd_max_torque_n_m": float(drives[actor].GetMaxForceAttr().Get())}
                    if not all(math.isfinite(v) for v in angular):
                        raise ValueError("nonfinite roller readback")
                    velocities[actor] = angular[1]  # Revolute axis is world Y.
                    if (not all(math.isfinite(readbacks[actor][k]) for k in ("usd_target_deg_s", "usd_max_torque_n_m"))
                            or abs(readbacks[actor]["usd_target_deg_s"]-math.degrees(motor_targets[actor])) > 1e-3
                            or abs(readbacks[actor]["usd_max_torque_n_m"]-drive_cfg["torque_limit_n_m"]) > 1e-6):
                        raise ValueError("drive target readback mismatch")
                fixed = {actor: (UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get() is True
                    and UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get() is True
                    and UsdPhysics.CollisionAPI(stage.GetPrimAtPath(str(prim.GetPath())+"/Collider")).GetCollisionEnabledAttr().Get() is True)
                    for actor, prim in stopper_prims.items()}
                # Evaluation data is deliberately kept out of update()/request().
                position, rotation = tote.get_world_pose()
                linear, angular = tote.get_linear_velocity(), tote.get_angular_velocity()
                values = [float(v) for v in (*position, *rotation, *linear, *angular)]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("nonfinite tote evaluation data")
                evaluation = {"position_m": position.tolist(), "orientation_wxyz": rotation.tolist(),
                    "linear_velocity_m_s": linear.tolist(), "angular_velocity_rad_s": angular.tolist()}
                last_sample = {"run_id": args.run_id, "world_time_s": stamp, "world_physics_frame": frame,
                    "raw_sensors": raw_rows, "controller_sensors": sensor_rows, "motors": readbacks,
                    "stoppers_fixed": fixed, "evaluation_only": evaluation}
                half_x, half_y = (v/2 for v in manifest["tote_outer_size_m"][:2])
                if (position[0]-half_x < -g["zone_length_m"]-.02
                        or position[0]+half_x > g["transfer_gap_m"]+g["zone_length_m"]+.02
                        or abs(position[1])+half_y > g["rail_inner_y_m"]+.015
                        or abs(position[2]-g["roller_top_z_m"]) > .04):
                    raise RuntimeError("evaluation: tote escaped authored fixture bounds")
                # Bounded sensor startup, with motors ZERO. Invalid startup data
                # is recorded, not used as contact/force. No moving transfer is
                # admitted until all required fields exist.
                initialized = all("force_n" in s and "in_contact" in s for s in sensor_rows.values())
                if not initialized and not armed:
                    if stamp > .25:
                        raise RuntimeError("contact sensors not initialized before startup deadline")
                    out = controller.output()
                else:
                    armed = True
                    out = controller.update(now_s=stamp, frame=frame, sensors=sensor_rows,
                        motor_rad_s=velocities, stoppers_fixed=fixed)
                last_sample["controller"] = out
                changed = prior_phase != out["phase"]
                if changed or stamp-last_written >= 1/cfg["observation_hz"] or out["failure"]:
                    stream.write(json.dumps(safe_value(last_sample), allow_nan=False)+"\n")
                    stream.flush()
                    samples += 1
                    last_written = stamp
                if changed:
                    report["transitions"].append({"phase": out["phase"], "world_time_s": stamp,
                        "world_physics_frame": frame, "command_id": out["command_id"]})
                    prior_phase = out["phase"]
                motor_targets = out["motor_targets_rad_s"]
                apply_targets(motor_targets)
                if out["failure"]:
                    raise RuntimeError(out["failure"])
                if out["phase"] in ("loaded", "returned"):
                    # An event cannot turn a moving/unsupported native tote into
                    # a passed physical result. This check never steers it.
                    if (np.linalg.norm(linear) > cfg["stationary_linear_m_s"]
                            or np.linalg.norm(angular) > cfg["stationary_angular_rad_s"]):
                        raise RuntimeError("evaluation: contact-confirmed tote is not stationary")
                    target_zone = "carrier" if out["phase"] == "loaded" else "source"
                    low = g["transfer_gap_m"] if target_zone == "carrier" else -g["zone_length_m"]
                    high = g["transfer_gap_m"]+g["zone_length_m"] if target_zone == "carrier" else 0.
                    if position[0]-half_x < low-.005 or position[0]+half_x > high+.005:
                        raise RuntimeError("evaluation: tote not fully supported inside confirmed bank")
                    if not any(m["path"] == out["phase"]+".png" for m in report["media"]):
                        capture(out["phase"], stamp, frame)
                    if out["phase"] == "returned":
                        break
                    motor_targets = controller.request("unload", "return_"+args.run_id)["motor_targets_rad_s"]
                    apply_targets(motor_targets)
                elif out["phase"] == "source_ready" and out["ready"]:
                    motor_targets = controller.request("load", "load_"+args.run_id)["motor_targets_rad_s"]
                    apply_targets(motor_targets)
        events = controller.events
        passed = (controller.phase == "returned" and [e["event"] for e in events]
            == ["transfer_loaded", "transfer_returned"] and len(report["media"]) == 2)
        if not passed:
            controller.fail("simulation budget exhausted without confirmed round trip")
        report.update(status="passed" if passed else "failed", stage="fixture_complete",
            native_transfer_verified=passed, transfer_events=events,
            native_measurements={"written_samples": samples, "last_world_time_s": previous_time,
                "last_world_frame": previous_frame}, failure=None if passed else "simulation budget exhausted")
        code = 0 if passed else 1
    except BaseException as error:
        if controller is not None:
            controller.fail(str(error))
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
            "traceback": traceback.format_exc(limit=12)}, last_sample=safe_value(last_sample))
    finally:
        for actor, drive in drives.items():
            try:
                if not drive.GetTargetVelocityAttr().Set(0.):
                    raise RuntimeError("drive zero write returned false")
            except BaseException as error:
                report["cleanup_errors"].append("motor_zero "+actor+": "+str(error))
        report["motor_zero_requested"] = bool(drives)
        if controller is not None:
            report["controller_final"] = controller.output()
            report["transfer_events"] = controller.events
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
            save(args.output/"result.json", safe_value(report))
            print("ROLLER_TRANSFER_RESULT", report["status"], flush=True)
        finally:
            if app is not None:
                app.close(wait_for_replicator=False)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
