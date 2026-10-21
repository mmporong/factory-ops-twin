"""Bounded SDK4.5 mounted-carrier admission, optional time-only loaded motion."""
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
    from factory_ops.carrier_robot import (ASSET_PATH, CHASSIS_PATH, ROBOT_PATH, assess_mount,
        inverse_rigid, pose_matrix, rigid_matrix, transform_point)

    profile = load_profile(args.config)
    manifest = build_manifest(profile)
    cfg, contact = manifest["probe"], manifest["contact"]
    motion_cfg = None
    project_sources = ["factory_ops/__init__.py", "factory_ops/carrier_module.py", "factory_ops/carrier_robot.py"]
    if "loaded_motion" in profile:
        from factory_ops.carrier_mobility import (MotionEvidence, MotionPlan, RgbCadence,
            native_drive_geometry, validate_motion)
        from factory_ops.motion import DriveGeometry
        motion_cfg = validate_motion(profile["loaded_motion"], cfg)
        cfg = {**cfg, "maximum_sim_time_s": motion_cfg["maximum_sim_time_s"],
               "maximum_wall_time_s": motion_cfg["maximum_wall_time_s"]}
        project_sources += ["factory_ops/carrier_mobility.py", "factory_ops/drive_profile.py", "factory_ops/motion.py"]
    args.output.mkdir(parents=True, exist_ok=False)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    report = {"schema_version": 1, "run_id": args.run_id, "status": "starting", "stage": "preflight",
        "source_sha256": sha(Path(__file__)), "config_sha256": sha(args.config),
        "project_source_sha256": {name: sha(root/name) for name in project_sources},
        "manifest_sha256": manifest_sha256(manifest), "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "one free Nova Carter, chassis-fixed carrier and separate free dynamic loaded tote; static admission only",
        "transport_verified": False, "sensor_fov_verified": False, "mount_strength_verified": False,
        "industrial_rating_verified": False, "stage2_complete": False, "physical_attachment_verified": False,
        "media": [], "cleanup_errors": [], "requires_host_closure": True,
        "profile_assumptions_verbatim": manifest["assumptions"],
        "fixture_scope_difference": "Original profile stand/release wording describes 114 only. This fixture uses a free Nova Carter and chassis fixed joint, no stands and no authored separation. Geometry/mass/material assumptions remain unchanged."}
    if motion_cfg is not None:
        report.update(scope="one Nova Carter with chassis-fixed authored carrier and free6kg tote; bounded time-command motion coupon only",
            loaded_motion_verified=False, loaded_motion_config=motion_cfg,
            loaded_motion_config_sha256=manifest_sha256(motion_cfg),
            motion_rgb={"planned_fps": motion_cfg["rgb_fps"], "maximum_frames": motion_cfg["maximum_rgb_frames"], "captured_frames": 0,
                "media_clock_sync_verified": False, "source": "continuous persistent640x480 RGB; actual physics-frame cadence from loaded_motion_config"})
    (args.output/"manifest.json").write_text(manifest_json(manifest), encoding="utf-8")
    started = time.monotonic()
    deadline = started+cfg["maximum_wall_time_s"]
    app = rgb = product = robot = None
    plan = evidence = cadence = None
    code = 1

    def phase(name):
        report["stage"] = name
        save(args.output/"result.json", report)

    def bounded():
        if time.monotonic() >= deadline:
            raise TimeoutError("loaded carrier wall deadline exceeded; native blocking calls require host Job guard")

    try:
        report["isaac_version"] = (Path(os.environ["ISAAC_PATH"])/"VERSION").read_text().strip()
        if not report["isaac_version"].startswith("4.5.0"):
            raise ValueError("this probe requires the installed SDK4.5.0 baseline")
        phase("native_startup")
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
        from isaacsim.core.utils.stage import is_stage_loading
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.sensors.physics import ContactSensor
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

        dt = cfg["physics_dt_s"]
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt, backend="numpy", device="cpu")
        physics = world.get_physics_context()
        physics.enable_gpu_dynamics(False)
        physics.set_broadphase_type("MBP")
        physics.set_gravity(-9.81)
        world.scene.add_ground_plane(size=5., restitution=0., static_friction=.8, dynamic_friction=.6)
        asset_root = get_assets_root_path()
        if not isinstance(asset_root, str) or not asset_root:
            raise RuntimeError("existing asset resolver returned no root; no alternate asset attempted")
        asset_url = asset_root.rstrip("/")+ASSET_PATH
        report["asset"] = {"relative_path": ASSET_PATH, "resolved_endpoint": asset_url,
            "resolver": "installed isaacsim.storage.native.get_assets_root_path", "alternate_assets_attempted": 0}
        wheels = ["joint_wheel_left", "joint_wheel_right"]
        robot = world.scene.add(WheeledRobot(prim_path=ROBOT_PATH, name="loaded_nova_carter",
            wheel_dof_names=wheels, create_robot=True, usd_path=asset_url, position=np.array([0., 0., .15])))
        phase("asset_loading")
        load_deadline, quiet = min(deadline, time.monotonic()+90.), 0
        for _ in range(20000):
            bounded()
            if time.monotonic() >= load_deadline or not app.is_running():
                raise TimeoutError("Nova Carter composition/load deadline exceeded")
            app.update()
            quiet = 0 if is_stage_loading() else quiet+1
            if quiet >= 3:
                break
        else:
            raise TimeoutError("asset update budget exhausted")
        phase("native_geometry_inspection")
        stage = omni.usd.get_context().get_stage()
        chassis_prim = stage.GetPrimAtPath(CHASSIS_PATH)
        root_prim = stage.GetPrimAtPath(ROBOT_PATH)
        if not root_prim.IsValid():
            raise ValueError("expected Nova Carter root is absent")
        transforms = UsdGeom.XformCache(Usd.TimeCode.Default())

        def matrix(prim):
            value = transforms.GetLocalToWorldTransform(prim)
            # USD Gf uses row vectors; carrier_robot uses column vectors.
            result = [[float(value[j][i]) for j in range(4)] for i in range(4)]
            try:
                return rigid_matrix(result)
            except ValueError:
                report["invalid_native_transform"] = {"prim": str(prim.GetPath()), "raw_repr": repr(result)}
                raise

        units = float(UsdGeom.GetStageMetersPerUnit(stage))
        if not math.isfinite(units):
            raise ValueError("nonfinite native meters-per-unit: "+repr(units))
        measured = {"schema_version": 1, "run_id": args.run_id, "robot_path": ROBOT_PATH,
            "chassis_path": CHASSIS_PATH, "asset_relative_path": ASSET_PATH,
            "meters_per_unit": units, "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "robot_world_from_local": matrix(root_prim),
            "chassis_world_from_local": matrix(chassis_prim) if chassis_prim.IsValid() else None,
            "rigid_bodies": [], "articulation_roots": [], "colliders": [], "sensors": [], "composition_errors": [],
            "source": "geometry-computed local extents transformed by native XformCache before stepping; not cooked hull, ray visibility or mount strength",
            "sensor_keepout_source": "authored 0.16m cube around measured camera/lidar frame candidates; lidar name matches are conservative anchors, not proof of a functioning sensor or FOV"}
        report["native_geometry"] = measured
        report["usd_version"] = list(Usd.GetVersion())

        def collision_bound(collider):
            # Authored extent is a cached value, not the shape parameters.
            # Nova Carter's caster Cylinder has a stale 100m cached extent.
            # Recompute every geometry leaf and preserve both values without
            # modifying the asset or silently trusting a malformed cache.
            combined = Gf.Range3d()
            leaves = []
            for leaf in Usd.PrimRange(collider, Usd.TraverseInstanceProxies()):
                boundable = UsdGeom.Boundable(leaf)
                if not boundable:
                    continue
                computed = UsdGeom.Boundable.ComputeExtentFromPlugins(boundable, Usd.TimeCode.Default())
                if computed is None or len(computed) != 2:
                    raise ValueError("no geometry extent computation for collider leaf: "+str(leaf.GetPath()))
                lo, hi = ([float(v) for v in point] for point in computed)
                if (not all(math.isfinite(v) for v in lo+hi)
                        or any(b < a for a, b in zip(lo, hi)) or all(b == a for a, b in zip(lo, hi))):
                    raise ValueError("invalid geometry-computed collider extent: "+str(leaf.GetPath()))
                local_range = Gf.Range3d(Gf.Vec3d(*lo), Gf.Vec3d(*hi))
                leaf_box = Gf.BBox3d(local_range, transforms.GetLocalToWorldTransform(leaf)).ComputeAlignedRange()
                combined.UnionWith(leaf_box)
                authored = boundable.GetExtentAttr().Get()
                leaves.append({"path": str(leaf.GetPath()), "type": leaf.GetTypeName(),
                    "authored_extent": None if authored is None else [[float(v) for v in p] for p in authored],
                    "computed_extent": [lo, hi],
                    "world_aabb_m": {"center_m": list(leaf_box.GetMidpoint()), "size_m": list(leaf_box.GetSize())}})
            if not leaves or combined.IsEmpty():
                raise ValueError("enabled collider has no computable geometry leaves: "+str(collider.GetPath()))
            return combined, leaves

        prim_count = 0
        for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
            bounded()
            prim_count += 1
            if prim_count > 50000:
                raise RuntimeError("asset inspection prim budget exceeded")
            path = str(prim.GetPath())
            measured["composition_errors"].extend(path+": "+str(e) for e in prim.GetPrimIndex().localErrors)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                measured["articulation_roots"].append(path)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                body = UsdPhysics.RigidBodyAPI(prim)
                measured["rigid_bodies"].append({"path": path, "enabled": bool(body.GetRigidBodyEnabledAttr().Get()),
                    "kinematic": bool(body.GetKinematicEnabledAttr().Get()), "world_from_local": matrix(prim)})
            if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                box, extent_sources = collision_bound(prim)
                if box.IsEmpty():
                    raise ValueError("empty enabled collision bound: "+path)
                parent = prim
                while parent.IsValid() and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
                    parent = parent.GetParent()
                center, size = list(box.GetMidpoint()), list(box.GetSize())
                if not all(math.isfinite(float(v)) for v in center+size) or not all(v > 0 for v in size):
                    report["invalid_native_bound"] = {"prim": path, "raw_repr": repr((center, size))}
                    raise ValueError("invalid enabled native collider bound: "+path)
                measured["colliders"].append({"path": path, "body_path": str(parent.GetPath()) if parent.IsValid() else "",
                    "shape_type": prim.GetTypeName(), "world_aabb_m": {"center_m": center, "size_m": size},
                    "extent_sources": extent_sources})
            if (prim.IsA(UsdGeom.Camera) or (prim.IsA(UsdGeom.Xformable)
                    and "lidar" in (prim.GetName()+" "+prim.GetTypeName()+" "+" ".join(prim.GetAppliedSchemas())).lower())):
                measured["sensors"].append({"path": path, "type": prim.GetTypeName(),
                    "applied_schemas": list(prim.GetAppliedSchemas()),
                    "classification": "typed camera" if prim.IsA(UsdGeom.Camera) else "lidar type/schema/name clearance candidate",
                    "world_from_local": matrix(prim)})
        measured["prims_inspected"] = prim_count
        layers = []
        for layer in stage.GetUsedLayers():
            local = layer.realPath
            layers.append({"identifier": layer.identifier, "resolved_path": local,
                "local_file_sha256": sha(Path(local)) if local and "://" not in local and Path(local).is_file() else None})
        report["asset"]["used_layers"] = layers
        save(args.output/"native-geometry.json", measured)
        phase("mount_admission")
        if measured["composition_errors"]:
            raise ValueError("asset composition errors; raw diagnostics preserved")
        admission = assess_mount(manifest, measured, args.run_id, clearance_m=profile["mount"]["clearance_m"])
        report["admission"] = admission
        save(args.output/"admission.json", admission)
        if not admission["admitted"]:
            report.update(status="failed", reason="measured_mount_candidate_rejected")
            return 1
        if motion_cfg is not None:
            report["drive_geometry"] = native_drive_geometry(measured)

        phase("author_physical_attachment")
        material = UsdShade.Material.Define(stage, "/World/LoadedCarrierMaterial")
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr(contact["static_friction"])
        material_api.CreateDynamicFrictionAttr(contact["dynamic_friction"])
        material_api.CreateRestitutionAttr(contact["restitution"])
        chassis_usd_matrix = transforms.GetLocalToWorldTransform(chassis_prim)
        body_paths = {"carrier": "/World/LoadedCarrier", "tote": "/World/LoadedTote"}
        bodies, actor_owners = {}, {}
        for key, path in body_paths.items():
            prim = UsdGeom.Xform.Define(stage, path)
            initial = Gf.Matrix4d(chassis_usd_matrix)
            if key == "tote":
                offset = list(manifest["tote_rest_origin_m"])
                offset[2] += cfg["initial_gap_m"]
                translation = Gf.Matrix4d(1.)
                translation.SetTranslate(Gf.Vec3d(*offset))
                initial = translation*chassis_usd_matrix
            prim.AddTransformOp().Set(initial)
            UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim()).CreateRigidBodyEnabledAttr(True)
            props = manifest["mass_properties"][key]
            inertia = props["inertia_matrix_kg_m2"]
            if any(abs(inertia[i][j]) > 1e-10 for i in range(3) for j in range(3) if i != j):
                raise ValueError("axis-aligned principal inertia required")
            mass = UsdPhysics.MassAPI.Apply(prim.GetPrim())
            mass.CreateMassAttr(props["mass_kg"])
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*props["center_of_mass_m"]))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*(inertia[i][i] for i in range(3))))
            mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
            for box in (b for b in manifest["geometry"] if b["body"] == key):
                shape = UsdGeom.Cube.Define(stage, path+"/"+box["name"])
                shape.CreateSizeAttr(1.)
                shape.AddTranslateOp().Set(Gf.Vec3d(*box["center_m"]))
                shape.AddScaleOp().Set(Gf.Vec3d(*box["size_m"]))
                shape.CreateDisplayColorAttr([Gf.Vec3f(*((.12, .32, .65) if key == "carrier" else (.22, .65, .48)))])
                UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
                collision = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
                collision.CreateContactOffsetAttr(contact["contact_offset_m"])
                collision.CreateRestOffsetAttr(contact["rest_offset_m"])
                UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose="physics")
                actor_owners[str(shape.GetPath())] = key
            actor_owners[path] = key
            bodies[key] = world.scene.add(SingleRigidPrim(prim_path=path, name="loaded_"+key))
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/CarrierChassisJoint")
        joint.GetBody0Rel().SetTargets([CHASSIS_PATH])
        joint.GetBody1Rel().SetTargets([body_paths["carrier"]])
        for index in (0, 1):
            joint.GetPrim().GetAttribute("physics:localPos%d" % index).Set(Gf.Vec3f(0.))
            joint.GetPrim().GetAttribute("physics:localRot%d" % index).Set(Gf.Quatf(1.))
        joint.GetExcludeFromArticulationAttr().Set(True)
        joint.GetCollisionEnabledAttr().Set(False)
        report["attachment"] = {"joint_path": str(joint.GetPath()), "body0": [str(p) for p in joint.GetBody0Rel().GetTargets()],
            "body1": [str(p) for p in joint.GetBody1Rel().GetTargets()],
            "joint_enabled": bool(joint.GetJointEnabledAttr().Get()),
            "exclude_from_articulation": bool(joint.GetExcludeFromArticulationAttr().Get()),
            "connected_body_pair_collision_enabled": bool(joint.GetCollisionEnabledAttr().Get()),
            "local_positions_m": [list(joint.GetPrim().GetAttribute("physics:localPos%d" % i).Get()) for i in (0, 1)],
            "policy": "external fixed constraint as installed RobotAssembler; no global collision mask, no world anchor or articulation topology change"}
        if (report["attachment"]["body0"] != [CHASSIS_PATH] or report["attachment"]["body1"] != [body_paths["carrier"]]
                or report["attachment"]["exclude_from_articulation"] is not True
                or report["attachment"]["joint_enabled"] is not True
                or any(abs(v) > 1e-8 for p in report["attachment"]["local_positions_m"] for v in p)):
            raise ValueError("fixed joint identity/semantics readback failed")
        bodies["chassis"] = world.scene.add(SingleRigidPrim(prim_path=CHASSIS_PATH, name="observed_chassis"))
        sensor = world.scene.add(ContactSensor(prim_path=body_paths["tote"]+"/ContactSensor", name="loaded_tote_contact",
            dt=dt, radius=-1., min_threshold=cfg["minimum_force_n"], max_threshold=1e6))
        sensor.add_raw_contact_data_to_frame()
        UsdLux.DomeLight.Define(stage, "/World/LoadedLight").CreateIntensityAttr(1300.)
        phase("native_physics_initialization")
        world.reset()
        bounded()
        if not robot.handles_initialized or not set(wheels).issubset(robot.dof_names):
            raise RuntimeError("native articulation/wheel handles unavailable")
        if physics.is_gpu_dynamics_enabled():
            raise RuntimeError("fixture requires CPU PhysX, GPU rendering only")
        wheel_indices = np.array(robot.wheel_dof_indices, dtype=np.int64)
        if motion_cfg is not None:
            if (len(wheel_indices) != 2 or [robot.dof_names[int(i)] for i in wheel_indices] != wheels):
                raise ValueError("native wheel identity/order mismatch")
            max_efforts = [float(v) for v in robot.get_articulation_controller().get_max_efforts()[wheel_indices]]
            if not all(math.isfinite(v) and v > 0 for v in max_efforts):
                raise ValueError("native wheel effort limits unavailable/invalid")
            report["wheel_drive_readback"] = {"names": wheels, "indices": wheel_indices.tolist(),
                "native_maximum_effort_n_m": max_efforts,
                "geometry_source": "pre-step measured collider bounds; wheel DOF readback before commands; original asset drives unchanged"}
        camera = rep.create.camera(position=(2.6, 2.8, 2.) if motion_cfg else (2., 2., 1.7),
                                   look_at=(.35, .1, .7) if motion_cfg else (0., 0., .7))
        product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        stage.GetRootLayer().Export(str(args.output/"scene.usda"))
        phase("static_loaded_contact")
        previous_time, previous_frame = float(world.current_time), int(world.current_time_step_index)
        previous_sensor_time, previous_sensor_frame = -1., -1
        dwell_since = None
        success = False
        sample_count = 0
        physics_samples = 0
        plan = evidence = cadence = motion_origin = None
        last_written_frame = -1
        last_motion_phase = None

        def angle_between(a, b):
            return math.acos(max(-1., min(1., (sum(a[k][i]*b[k][i] for k in range(3) for i in range(3))-1)/2)))

        def capture(name, stamp, frame, *, motion_frame=False):
            data = np.asarray(rgb.get_data())
            if (data.ndim != 3 or data.shape[:2] != (height, width) or data.shape[2] not in (3, 4)
                    or data.dtype != np.uint8 or float(data[:, :, :3].std()) < 1.):
                raise RuntimeError("genuine RGB unavailable; no synthetic substitute")
            image = args.output/name
            Image.fromarray(data[:, :, :3]).save(image)
            entry = {"path": image.name, "sha256": sha(image), "world_checkpoint_s": stamp,
                "world_checkpoint_frame": frame, "media_clock_sync_verified": False}
            if motion_cfg is not None:
                entry["role"] = "motion_frame" if motion_frame else "keyframe"
            report["media"].append(entry)
            if motion_frame:
                report["motion_rgb"]["captured_frames"] += 1

        with (args.output/"observations.jsonl").open("x", encoding="utf-8") as stream:
            for _ in range(math.ceil(cfg["maximum_sim_time_s"]/dt)):
                bounded()
                command = plan.command(previous_time, dt) if plan is not None else None
                targets = command["target"]["wheel_targets_rad_s"] if command is not None else [0., 0.]
                robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.array(targets)))
                world.step(render=True)
                bounded()
                stamp, frame = float(world.current_time), int(world.current_time_step_index)
                if not math.isfinite(stamp) or frame != previous_frame+1 or stamp <= previous_time or abs(stamp-previous_time-dt) > 1e-6:
                    raise RuntimeError("native World clock/frame mismatch")
                physics_samples += 1
                states, matrices = {}, {}
                for key, body in bodies.items():
                    p, q = body.get_world_pose()
                    v, w = body.get_linear_velocity(), body.get_angular_velocity()
                    if not all(math.isfinite(float(n)) for n in [*p, *q, *v, *w]):
                        raise ValueError("nonfinite native body observation")
                    matrices[key] = pose_matrix(p.tolist(), q.tolist())
                    states[key] = {"position_m": p.tolist(), "orientation_wxyz": q.tolist(),
                        "linear_velocity_m_s": v.tolist(), "angular_velocity_rad_s": w.tolist()}
                raw = sensor.get_current_frame()
                sensor_time, sensor_frame = float(raw["time"]), int(raw["physics_step"])
                force = float(raw.get("force", 0.))
                if not math.isfinite(sensor_time) or not math.isfinite(force) or force < 0 or sensor_time < previous_sensor_time or sensor_frame < previous_sensor_frame:
                    raise ValueError("contact clock/force invalid or regressed")
                fresh = (sensor_time > previous_sensor_time and sensor_frame > previous_sensor_frame
                    and 0. <= stamp-sensor_time <= cfg["sensor_freshness_s"])
                pairs = [[str(c["body0"]), str(c["body1"])] for c in raw.get("contacts", [])]
                mapped = [sorted([actor_owners.get(a, a), actor_owners.get(b, b)]) for a, b in pairs]
                inverse_chassis = inverse_rigid(matrices["chassis"])
                module_offset = transform_point(inverse_chassis, states["carrier"]["position_m"])
                tote_offset = transform_point(inverse_chassis, states["tote"]["position_m"])
                expected = manifest["tote_rest_origin_m"]
                drift = math.dist(states["chassis"]["position_m"][:2], [measured["chassis_world_from_local"][i][3] for i in (0, 1)])
                base_angle = angle_between(matrices["chassis"], measured["chassis_world_from_local"])
                module_error = float(np.linalg.norm(module_offset))
                tote_error = math.dist(tote_offset, expected)
                stationary = all(np.linalg.norm(s["linear_velocity_m_s"]) <= cfg["stationary_linear_m_s"]
                    and np.linalg.norm(s["angular_velocity_rad_s"]) <= cfg["stationary_angular_rad_s"] for s in states.values())
                load_supported = (fresh and bool(raw.get("in_contact", False)) and force >= cfg["minimum_force_n"]
                    and ["carrier", "tote"] in mapped
                    and module_error <= cfg["maximum_settle_error_m"] and tote_error <= cfg["maximum_settle_error_m"]
                    and angle_between(matrices["carrier"], matrices["chassis"]) <= .05
                    and angle_between(matrices["tote"], matrices["chassis"]) <= .05)
                qualified = load_supported and stationary
                sample = {"run_id": args.run_id, "world_time_s": stamp, "world_physics_frame": frame, "bodies": states,
                    "wheel_position_rad": robot.get_wheel_positions(),
                    "wheel_velocity_rad_s": robot.get_joint_velocities()[robot.wheel_dof_indices],
                    "contact": {"sensor_time_s": sensor_time, "sensor_physics_step": sensor_frame, "fresh": fresh,
                        "force_n": force, "in_contact": bool(raw.get("in_contact", False)), "raw_actor_pairs": pairs, "mapped_pairs": mapped},
                    "chassis_xy_drift_m": drift, "chassis_rotation_drift_rad": base_angle,
                    "module_chassis_position_error_m": module_error, "tote_rest_position_error_m": tote_error,
                    "stationary": bool(stationary), "qualified": bool(qualified)}
                report["last_observation"] = sample
                # WheeledRobot returns numpy scalar lists; retain finite SI values in JSON.
                for field in ("wheel_position_rad", "wheel_velocity_rad_s"):
                    sample[field] = [float(v) for v in sample[field]]
                    if len(sample[field]) != 2 or not all(math.isfinite(v) for v in sample[field]):
                        raise ValueError("native wheel readback invalid")
                if command is None and (drift > cfg["maximum_settle_error_m"] or base_angle > .05):
                    raise RuntimeError("free chassis moved outside static admission bounds; no pose correction applied")
                if any(pair != ["carrier", "tote"] for pair in mapped):
                    raise RuntimeError("unexpected tote actor contact; diagnostics preserved")
                transition = False
                if command is not None:
                    wheel_data = plan.observe_wheels(stamp, sample["wheel_position_rad"], sample["wheel_velocity_rad_s"],
                        [float(v) for v in robot.get_measured_joint_efforts(joint_indices=wheel_indices)], max_efforts)
                    delta = transform_point(inverse_rigid(motion_origin), states["chassis"]["position_m"])
                    relative_rotation = [[sum(motion_origin[k][i]*matrices["chassis"][k][j] for k in range(3))
                        for j in range(3)] for i in range(3)]
                    yaw = math.atan2(relative_rotation[1][0], relative_rotation[0][0])
                    tilt = math.acos(max(-1., min(1., relative_rotation[2][2])))
                    encoder = wheel_data["encoder"]
                    encoder_stopped = (abs(encoder["linear_m_s"]) <= cfg["stationary_linear_m_s"]
                        and abs(encoder["angular_rad_s"]) <= cfg["stationary_angular_rad_s"])
                    done = evidence.observe(command["phase"], stamp, position_m=delta, yaw_rad=yaw, tilt_rad=tilt,
                        encoder=encoder, wheel_velocity=sample["wheel_velocity_rad_s"],
                        load_supported=bool(load_supported), stationary=bool(stationary and encoder_stopped),
                        target_zero=all(v == 0. for v in targets))
                    sample.update(motion=command, motion_readback=wheel_data, load_supported=bool(load_supported),
                        evaluation_only={"initial_chassis_relative_position_m": delta, "yaw_rad": yaw, "tilt_rad": tilt})
                    transition = last_motion_phase != command["phase"]
                    if transition:
                        report.setdefault("motion_transitions", []).append({"phase": command["phase"],
                            "world_time_s": stamp, "world_physics_frame": frame, "drive_phase": command["target"]["phase"]})
                        last_motion_phase = command["phase"]
                    if cadence.due(frame):
                        capture("motion-%04d.png" % (cadence.count-1), stamp, frame, motion_frame=True)
                    if done:
                        report["motion_evaluation"] = evidence.result()
                        if not report["motion_evaluation"]["passed"]:
                            raise RuntimeError("loaded motion evidence failed: "+repr(report["motion_evaluation"]["failures"]))
                        report["motion_rgb"]["sequence_readback"] = cadence.validate_records(
                            [m for m in report["media"] if m.get("role") == "motion_frame"])
                        capture("mounted-motion-final.png", stamp, frame)
                        success = True
                else:
                    dwell_since = (stamp if dwell_since is None else dwell_since) if qualified else None
                if (motion_cfg is None or last_written_frame < 0
                        or frame-last_written_frame >= motion_cfg["rgb_period_physics_frames"] or transition or success):
                    stream.write(json.dumps(sample, allow_nan=False)+"\n")
                    stream.flush()
                    sample_count += 1
                    last_written_frame = frame
                previous_time, previous_frame = stamp, frame
                previous_sensor_time, previous_sensor_frame = sensor_time, sensor_frame
                if success:
                    break
                if command is None and dwell_since is not None and stamp-dwell_since >= cfg["contact_dwell_s"]:
                    capture("mounted-settled.png", stamp, frame)
                    if motion_cfg is None:
                        success = True
                        break
                    phase("loaded_motion")
                    report["initial_static_dwell_s"] = stamp-dwell_since
                    geometry = report["drive_geometry"]
                    plan = MotionPlan(motion_cfg, DriveGeometry(geometry["wheel_radius_m"], geometry["wheel_separation_m"]),
                        stamp, sample["wheel_position_rad"])
                    evidence = MotionEvidence(motion_cfg, cfg)
                    cadence = RgbCadence(frame, motion_cfg["rgb_period_physics_frames"], motion_cfg["maximum_rgb_frames"])
                    motion_origin = matrices["chassis"]
                    report["motion_origin"] = {"world_time_s": stamp, "world_physics_frame": frame,
                        "evaluation_only_chassis_world_from_local": motion_origin}
        report.update(status="passed" if success else "failed", stage="loaded_motion_complete" if motion_cfg else "static_fixture_complete",
            physical_attachment_verified=success, native_samples=sample_count,
            static_dwell_s=report.get("initial_static_dwell_s", 0. if dwell_since is None else previous_time-dwell_since),
            velocity_source="native PhysX rigid-body and wheel joint reads, not ROS odometry",
            control_policy="zero wheel velocity targets only; no position/velocity state setters, world anchor, GT driving or tolerance changes")
        if motion_cfg is not None:
            report.update(loaded_motion_verified=success, native_control_samples=physics_samples,
                control_policy="fixed-time commands through existing DriveProfile; native wheel targets only; GT is evaluation-only; relative load bounds unchanged")
            if evidence is not None:
                report["motion_evaluation"] = evidence.result()
        code = 0 if success else 1
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
                      "traceback": traceback.format_exc(limit=12)})
        if evidence is not None:
            report["motion_evaluation"] = evidence.result()
    finally:
        if motion_cfg is not None and robot is not None:
            try:
                robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.zeros(2)))
                report["fault_or_final_zero_requested"] = True
            except BaseException as error:
                report["cleanup_errors"].append("wheel_zero: "+str(error))
        for label, resource, operation in (("rgb", rgb, "detach"), ("render_product", product, "destroy")):
            if resource is not None:
                try:
                    getattr(resource, operation)()
                except BaseException as error:
                    report["cleanup_errors"].append(label+": "+str(error))
        if report["cleanup_errors"]:
            report["status"], code = "failed", 1
            if motion_cfg is not None:
                report["loaded_motion_verified"] = False
        report.update(finished_at_utc=datetime.now(timezone.utc).isoformat(), elapsed_wall_s=time.monotonic()-started,
                      app_close_requested=app is not None)
        try:
            save(args.output/"result.json", report)
            print("LOADED_CARRIER_STATIC_RESULT", report["status"], flush=True)
        finally:
            if app is not None:
                app.close(wait_for_replicator=False)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
