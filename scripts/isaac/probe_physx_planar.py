"""One SDK4.5 PhysX planar candidate; no ROS, driving, or transport qualification."""
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


OWNED = "/World/PhysXPlanar054"
ROBOT = OWNED + "/Robot"
PARAMS = dict(min_range=0.1, max_range=20.0, draw_points=False, draw_lines=False,
              horizontal_fov=360.0, vertical_fov=2.0, horizontal_resolution=0.5,
              vertical_resolution=1.0, rotation_rate=0.0, high_lod=False,
              yaw_offset=0.0, enable_semantics=False)
DEPENDENCIES = ("factory_ops/__init__.py", "factory_ops/isaac45_runtime.py")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def save(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def vector(value, length):
    if (not isinstance(value, list) or len(value) != length
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in value)):
        raise ValueError("invalid finite vector")
    return value


def ray_box(origin, direction, center, size):
    enter, leave = 0.0, float("inf")
    for p, d, c, width in zip(origin, direction, center, size):
        lo, hi = c - width / 2, c + width / 2
        if abs(d) < 1e-12:
            if not lo <= p <= hi:
                return None
        else:
            near, far = sorted(((lo - p) / d, (hi - p) / d))
            enter, leave = max(enter, near), min(leave, far)
    return enter if leave >= enter and enter >= PARAMS["min_range"] else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True,
                        help="existing factory runtime config; reads only physics_dt_s and front/rear mounts")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    if (not args.output.is_absolute() or args.output.exists() or args.output.is_symlink()
            or os.path.normcase(str(args.output)) != os.path.normcase(str(args.output.resolve()))):
        parser.error("output must be new, absolute, canonical, and not a link")
    if not re.fullmatch(r"[0-9a-fA-F]{32}", args.run_id):
        parser.error("run-id must be 32 hexadecimal characters")
    if not args.config.is_absolute() or not args.config.is_file() or args.config.is_symlink():
        parser.error("config must be an existing absolute regular file")
    cfg_bytes = args.config.read_bytes()
    cfg = json.loads(cfg_bytes.decode("utf-8-sig"))
    mounts = cfg["lidars"]
    if len(mounts) != 2 or {m["name"] for m in mounts} != {"front", "rear"}:
        parser.error("the existing front and rear mounts are required")
    for mount in mounts:
        vector(mount["translation_xyz_m"], 3)
        vector([mount["yaw_rad"]], 1)
    dt = vector([cfg["physics_dt_s"]], 1)[0]
    if not 0 < dt <= 0.1:
        parser.error("physics_dt_s outside the bounded candidate")
    source_bytes = Path(__file__).read_bytes()
    dependency_hashes = {name: digest((root / name).read_bytes()) for name in DEPENDENCIES}
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = args.output / "inputs"
    inputs.mkdir()
    (inputs / "probe_physx_planar.py").write_bytes(source_bytes)
    (inputs / "config.json").write_bytes(cfg_bytes)
    (args.output / "snapshots").mkdir()
    started = time.monotonic()
    report = dict(schema_name="local45-physx-planar-admission", schema_version=1,
                  run_id=args.run_id, status="starting", stage="preflight",
                  source_sha256=digest(source_bytes), config_sha256=digest(cfg_bytes),
                  project_source_sha256=dependency_hashes, config_path=str(args.config),
                  started_at_utc=datetime.now(timezone.utc).isoformat(),
                  limits={"maximum_wall_s": 180, "maximum_physics_steps": 600},
                  parameters=PARAMS, mounts=mounts, physics_dt_s=dt, snapshots=[],
                  checks={}, pending_gaps=[], cleanup_errors=[], cleanup_operations=[],
                  scope="one static asset collision-ray candidate; no ROS/RTX/RPLIDAR/GMO/transport claim",
                  native_timestamp=None, time_source="authored_World_observation",
                  native_current_step_sequence_verified=False, requires_host_closure=True,
                  wall_bound_note="cooperative checks around SDK calls; host Job timeout also required",
                  sdk_close_requested=False, sdk_close_returned=False)
    app = world = stage = interface = None
    sensors, retained, targets = {}, {}, {}
    steps, code, owned_created = 0, 1, False

    def bound():
        if time.monotonic() - started >= 180:
            raise TimeoutError("180s cooperative wall limit")

    def phase(name):
        bound()
        report["stage"] = name
        report["elapsed_wall_s"] = time.monotonic() - started
        save(args.output / "result.json", report)

    def clock():
        stamp, frame = float(world.current_time), int(world.current_time_step_index)
        if not math.isfinite(stamp) or stamp < 0 or frame < 0:
            raise ValueError("invalid actual World clock")
        if frame > 600:
            raise TimeoutError("actual World physics-frame cap")
        return {"world_time_s": stamp, "world_physics_frame": frame}

    def step():
        nonlocal steps
        bound()
        if steps >= 600:
            raise TimeoutError("600 physics-call cap")
        before = clock()
        world.step(render=True)
        steps += 1
        after = clock()
        if (after["world_physics_frame"] != before["world_physics_frame"] + 1
                or after["world_time_s"] <= before["world_time_s"]):
            raise RuntimeError("World did not advance exactly one physics frame")
        bound()

    def analyze(arrays, matrix):
        depth, hit = arrays["linear_depth"], arrays["intensity"]
        az, zen = arrays["azimuth_rad"], arrays["zenith_rad"]
        if (az.ndim != 1 or zen.ndim != 1 or len(az) != 720 or not len(zen)
                or not np.isfinite(az).all() or not np.isfinite(zen).all()
                or hit.shape != depth.shape or arrays["encoded_depth"].shape != depth.shape):
            raise ValueError("angle lengths or raw depth/hit dimensions do not agree")
        layouts = []
        if depth.shape == (len(az), len(zen)):
            layouts.append((depth, hit, "azimuth,zenith"))
        if depth.shape == (len(zen), len(az)):
            layouts.append((depth.T, hit.T, "zenith,azimuth"))
        if depth.ndim == 1 and len(zen) == 1 and len(depth) == len(az):
            layouts.append((depth[:, None], hit[:, None], "azimuth; one zenith"))
        if len(layouts) != 1:
            raise ValueError("no unique raw axis mapping; no reshape assumption permitted")
        ranges, mask, layout = layouts[0]
        horizontal = np.flatnonzero(np.abs(zen.astype(float) - math.pi / 2) <= 1e-6)
        if len(horizontal) != 1:
            raise ValueError("exact horizontal zenith row absent or ambiguous")
        column = int(horizontal[0])
        ranges, mask = ranges[:, column], mask[:, column]
        if not np.isin(mask, [0, 255]).all() or not np.isfinite(ranges[mask == 255]).all():
            raise ValueError("malformed hit mask or hit distance")
        ordered = np.sort(np.mod(az.astype(float), 2 * math.pi))
        gaps = np.diff(np.r_[ordered, ordered[0] + 2 * math.pi])
        if not np.all(np.abs(gaps - math.radians(0.5)) <= 2e-5):
            raise ValueError("actual angles do not cover 720 distinct uniform directions")
        origin = list(matrix.ExtractTranslation())
        directions = []
        for angle in az:
            d = matrix.TransformDir(Gf.Vec3d(math.cos(float(angle)), math.sin(float(angle)), 0.0))
            norm = math.sqrt(sum(float(v) ** 2 for v in d))
            if not math.isfinite(norm) or abs(norm - 1) > 1e-6:
                raise ValueError("sensor transform changes ray scale")
            directions.append([float(v) / norm for v in d])
        if any(abs(d[2]) > 1e-6 for d in directions):
            raise ValueError("observed sensor mount is not world-horizontal")
        prim = arrays.get("prim_hits")
        prim_plane = None
        if prim is not None and prim.shape == depth.shape:
            prim_plane = (prim if depth.ndim == 1 else
                          (prim if layout == "azimuth,zenith" else prim.T)[:, column])
        elif prim is not None and prim.ndim == 1 and len(zen) == 1 and len(prim) == len(az):
            prim_plane = prim
        summary = {"raw_axes": layout, "horizontal_row": column,
                   "observed_zenith_rad": float(zen[column]), "beam_count": len(az),
                   "azimuth_order": "increasing" if bool(np.all(np.diff(az) > 0)) else "native_other_order",
                   "minimum_gap_rad": float(min(gaps)), "maximum_gap_rad": float(max(gaps)),
                   "origin_world_m": origin, "prim_beam_correspondence": prim_plane is not None,
                   "self_hits": None, "cardinal_targets": [], "no_hit_sector_world_deg": [35, 55]}
        if prim_plane is not None:
            summary["self_hits"] = [{"beam_index": i, "prim": str(p), "range_m": float(ranges[i])}
                for i, p in enumerate(prim_plane) if int(mask[i]) == 255
                and (str(p) == ROBOT or str(p).startswith(ROBOT + "/"))]
        for name, target in targets.items():
            bearing = math.atan2(target["center"][1] - origin[1], target["center"][0] - origin[0])
            errors = [abs(math.atan2(math.sin(math.atan2(d[1], d[0]) - bearing),
                                    math.cos(math.atan2(d[1], d[0]) - bearing))) for d in directions]
            i = min(range(len(errors)), key=errors.__getitem__)
            expected = ray_box(origin, directions[i], target["center"], target["size"])
            observed = float(ranges[i])
            distance_ok = expected is not None and int(mask[i]) == 255 and math.isfinite(observed) and abs(observed - expected) <= 0.08
            prim_path = str(prim_plane[i]) if prim_plane is not None else None
            summary["cardinal_targets"].append(dict(name=name, beam_index=i,
                actual_azimuth_rad=float(az[i]), direction_world=directions[i],
                expected_range_m=expected, observed_range_m=observed if math.isfinite(observed) else None,
                intensity=int(mask[i]), prim_hit=prim_path, distance_passed=bool(distance_ok),
                prim_passed=None if prim_path is None else prim_path == target["path"]))
        sector = [i for i, d in enumerate(directions) if 35 <= math.degrees(math.atan2(d[1], d[0])) <= 55]
        summary["no_hit_sector"] = {"beam_indices": sector, "hit_count": sum(int(mask[i]) != 0 for i in sector),
                                    "passed": bool(sector) and all(int(mask[i]) == 0 for i in sector)}
        return summary

    def snapshot(label):
        phase(label)
        group = {}
        for side, path in sensors.items():
            bound()
            before = clock()
            row = {"label": label, "side": side, "sensor_path": path,
                   "time_source": "authored_World_observation_not_native_stamp", "before": before,
                   "arrays": {}, "read_errors": []}
            owned, views = {}, {}
            for field, method in (("linear_depth", "get_linear_depth_data"), ("encoded_depth", "get_depth_data"),
                                  ("intensity", "get_intensity_data"), ("azimuth_rad", "get_azimuth_data"),
                                  ("zenith_rad", "get_zenith_data"), ("prim_hits", "get_prim_data")):
                try:
                    view = np.asarray(getattr(interface, method)(path))
                    if view.dtype.hasobject:
                        raise ValueError("object buffer cannot be preserved without assuming element conversion")
                    array = view.copy()
                    views[field], owned[field] = view, array
                    file = args.output / "snapshots" / (label + "-" + side + "-" + field + ".npy")
                    np.save(str(file), array, allow_pickle=False)
                    item = dict(path=str(file.relative_to(args.output)), dtype=str(view.dtype), shape=list(view.shape),
                                strides=list(view.strides), raw_c_bytes_sha256=digest(array.tobytes()),
                                file_sha256=digest(file.read_bytes()))
                    if field in ("azimuth_rad", "zenith_rad", "intensity", "prim_hits"):
                        item["values"] = array.tolist() if field in ("intensity", "prim_hits") else [float(v) if math.isfinite(float(v)) else None for v in array.flat]
                    row["arrays"][field] = item
                except Exception as error:
                    row["read_errors"].append({"field": field, "error": str(error)})
            row["sdk_counts"] = {name: int(getattr(interface, name)(path))
                                  for name in ("get_num_rows", "get_num_cols", "get_num_cols_ticked")}
            row["after"] = clock()
            if row["after"] != before:
                raise RuntimeError("World advanced while copying one acquisition")
            matrix = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            row["sensor_world_transform"] = [list(v) for v in matrix]
            try:
                row["plane"] = analyze(owned, matrix)
            except Exception as error:
                row["plane_error"] = str(error)
            report["snapshots"].append(row)
            group[side] = {"row": row, "owned": owned, "views": views}
            save(args.output / "result.json", report)
        retained[label] = group
        return group

    save(args.output / "result.json", report)
    try:
        sdk = Path(os.environ["ISAAC_PATH"])
        version_bytes = (sdk / "VERSION").read_bytes()
        report["isaac_version"] = version_bytes.decode().strip()
        report["sdk_version_sha256"] = digest(version_bytes)
        if not report["isaac_version"].startswith("4.5"):
            raise ValueError("only the existing SDK4.5 installation is permitted")
        base = sdk / "exts/isaacsim.sensors.physx"
        report["installed_source_sha256"] = {str(p.relative_to(sdk)): digest(p.read_bytes()) for p in (
            base / "config/extension.toml", base / "isaacsim/sensors/physx/scripts/commands.py",
            base / "isaacsim/sensors/physx/_range_sensor.cp310-win_amd64.pyd")}
        sys.path.insert(0, str(root))
        from factory_ops.isaac45_runtime import ASSET_PATH
        from isaacsim import SimulationApp
        phase("sdk_startup")
        app = SimulationApp({"headless": True, "multi_gpu": False, "width": 640, "height": 480})
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.sensors.physx")
        import numpy as np
        import omni.kit.commands
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
        from isaacsim.storage.native import get_assets_root_path
        from isaacsim.sensors.physx import _range_sensor
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=dt, device="cpu")
        world.pause()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
            raise ValueError("candidate requires meter stage units")
        if stage.GetPrimAtPath(OWNED).IsValid():
            raise RuntimeError("owned calibration root already exists")
        UsdGeom.Xform.Define(stage, OWNED)
        owned_created = True
        assets_root = get_assets_root_path()
        if not assets_root:
            raise RuntimeError("existing asset root unavailable")
        report["asset_url"] = assets_root + ASSET_PATH
        phase("asset_load_static_geometry")
        add_reference_to_stage(usd_path=report["asset_url"], prim_path=ROBOT)
        quiet = 0
        while quiet < 3:
            bound()
            app.update()
            quiet = 0 if is_stage_loading() else quiet + 1
        robot = stage.GetPrimAtPath(ROBOT)
        chassis_path = ROBOT + "/chassis_link"
        if not robot.IsValid() or not stage.GetPrimAtPath(chassis_path).IsValid():
            raise ValueError("actual NovaCarter/chassis is absent")
        UsdGeom.Xformable(robot).AddTranslateOp(opSuffix="AdmissionPlacement").Set(Gf.Vec3d(0, 0, 0.1))
        report["static_calibration"] = {"method": "disable existing rigid bodies; preserve colliders and all joints",
                                         "rigid_bodies": [], "collision_paths": [], "joint_edits": []}
        for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
            bound()
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = UsdPhysics.RigidBodyAPI(prim)
                old = api.GetRigidBodyEnabledAttr().Get()
                if not api.CreateRigidBodyEnabledAttr().Set(False):
                    raise RuntimeError("could not freeze reference body " + str(prim.GetPath()))
                report["static_calibration"]["rigid_bodies"].append({"path": str(prim.GetPath()), "original_enabled": old})
            if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.GetAttribute("physics:collisionEnabled").Get() is not False:
                report["static_calibration"]["collision_paths"].append(str(prim.GetPath()))
        if not report["static_calibration"]["rigid_bodies"] or not report["static_calibration"]["collision_paths"]:
            raise ValueError("reference lacks measurable rigid bodies/collision geometry")
        world.reset()
        world.pause()
        interface = _range_sensor.acquire_lidar_sensor_interface()
        report["available_sequence_api_names"] = [name for name in dir(interface) if "sequence" in name.lower()]
        report["sequence_api_called"] = None
        for mount in mounts:
            path = chassis_path + "/physx_admission_" + mount["name"]
            if stage.GetPrimAtPath(path).IsValid():
                raise ValueError("owned sensor prim already exists")
            ok, lidar = omni.kit.commands.execute("RangeSensorCreateLidar", path=path, parent=None, **PARAMS)
            if not ok or not lidar or str(lidar.GetPath()) != path or not lidar.GetPrim().IsValid():
                raise RuntimeError("PhysX command did not create the exact sensor")
            sensors[mount["name"]] = path
            transform = UsdGeom.Xformable(lidar.GetPrim())
            transform.ClearXformOpOrder()
            matrix = Gf.Matrix4d(1.0)
            matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(mount["yaw_rad"])))
            matrix.SetTranslateOnly(Gf.Vec3d(*mount["translation_xyz_m"]))
            transform.AddTransformOp(opSuffix="AdmissionMount").Set(matrix)
        origins = [UsdGeom.Xformable(stage.GetPrimAtPath(p)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation() for p in sensors.values()]
        z = sum(float(o[2]) for o in origins) / len(origins)
        for name, center, size in (("east", [4., 0., z], [.2, 2., 2.]), ("west", [-5., 0., z], [.2, 2.4, 2.]),
                                   ("north", [0., 6., z], [2.6, .2, 2.]), ("south", [0., -7., z], [3., .2, 2.])):
            path = OWNED + "/Targets/" + name
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            translate = cube.AddTranslateOp()
            translate.Set(Gf.Vec3d(*center))
            cube.AddScaleOp().Set(Gf.Vec3d(*size))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            targets[name] = {"path": path, "center": center, "size": size, "translate": translate}
        report["targets_before"] = {n: {k: v for k, v in t.items() if k != "translate"} for n, t in targets.items()}
        phase("warmup")
        initial_transform = UsdGeom.Xformable(stage.GetPrimAtPath(chassis_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world.play()
        for _ in range(12):
            step()
        before = snapshot("before_target_move")
        phase("move_owned_target_between_physics_steps")
        report["target_edit_world"] = clock()
        targets["east"]["center"] = [5., 0., z]
        targets["east"]["translate"].Set(Gf.Vec3d(5., 0., z))
        report["targets_after"] = {n: {k: v for k, v in t.items() if k != "translate"} for n, t in targets.items()}
        step()
        after_one = snapshot("after_one_physics_step")
        for _ in range(5):
            step()
        after_six = snapshot("after_six_physics_steps")
        report["target_response"] = {}
        for side in sensors:
            rows = [g[side]["row"].get("plane", {}).get("cardinal_targets", []) for g in (before, after_one, after_six)]
            east = [next((r for r in rs if r["name"] == "east"), None) for rs in rows]
            responses = []
            for item in east[1:]:
                a, b = east[0], item
                responses.append(bool(a and b and a["distance_passed"] and b["distance_passed"]
                    and abs(b["observed_range_m"] - a["observed_range_m"]) > .8))
            report["target_response"][side] = {"after_one_step": responses[0], "within_six_steps": responses[1],
                "before_depth_bytes_sha256": before[side]["row"]["arrays"].get("linear_depth", {}).get("raw_c_bytes_sha256"),
                "after_one_depth_bytes_sha256": after_one[side]["row"]["arrays"].get("linear_depth", {}).get("raw_c_bytes_sha256"),
                "after_six_depth_bytes_sha256": after_six[side]["row"]["arrays"].get("linear_depth", {}).get("raw_c_bytes_sha256")}
        world.pause()
        paused_before = snapshot("paused_before_four_renders")
        report["paused_render_boundary"] = {"before": clock(), "render_calls": 4}
        for _ in range(4):
            bound()
            world.render()
        report["paused_render_boundary"]["after"] = clock()
        paused_after = snapshot("paused_after_four_renders")
        changed = {side: [f for f, a in paused_before[side]["owned"].items()
                         if f not in paused_after[side]["owned"] or a.tobytes() != paused_after[side]["owned"][f].tobytes()]
                   for side in sensors}
        report["paused_render_boundary"]["changed_raw_fields"] = changed
        report["retained_copies"] = {side: {f: {"still_matches_original_bytes": digest(a.tobytes()) == before[side]["row"]["arrays"][f]["raw_c_bytes_sha256"],
                                                     "native_view_changed": a.tobytes() != before[side]["views"][f].tobytes()}
                                                   for f, a in before[side]["owned"].items()} for side in sensors}
        final_transform = UsdGeom.Xformable(stage.GetPrimAtPath(chassis_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        report["static_calibration"]["chassis_transform_before"] = [list(v) for v in initial_transform]
        report["static_calibration"]["chassis_transform_after"] = [list(v) for v in final_transform]
        checks = report["checks"]
        checks["static_chassis"] = all(abs(initial_transform[i][j] - final_transform[i][j]) <= 1e-7 for i in range(4) for j in range(4))
        checks["all_raw_planes_identified"] = all("plane" in s for s in report["snapshots"])
        checks["cardinal_distance_checks"] = all("plane" in s and all(r["distance_passed"] for r in s["plane"]["cardinal_targets"]) for s in report["snapshots"])
        checks["no_hit_sector"] = all(s.get("plane", {}).get("no_hit_sector", {}).get("passed", False) for s in report["snapshots"])
        checks["moving_target_response_within_six_steps"] = all(r["within_six_steps"] for r in report["target_response"].values())
        checks["paused_world_time_frame_unchanged"] = report["paused_render_boundary"]["before"] == report["paused_render_boundary"]["after"]
        checks["owned_copies_stable"] = all(v["still_matches_original_bytes"] for fields in report["retained_copies"].values() for v in fields.values())
        for s in report["snapshots"]:
            plane = s.get("plane")
            if plane and not plane["prim_beam_correspondence"]:
                report["pending_gaps"].append(s["label"] + "/" + s["side"] + ": prim-to-beam correspondence/self-hit unknown")
            elif plane and any(r["prim_passed"] is False for r in plane["cardinal_targets"]):
                checks["cardinal_prim_identity"] = False
        if any(changed.values()):
            report["pending_gaps"].append("raw data changed across paused render; acquisition binding unresolved")
        if not all(r["after_one_step"] for r in report["target_response"].values()):
            report["pending_gaps"].append("same-current-step target response not demonstrated on both sensors")
        report["acquisition_limit"] = "World-bounded observation and moving-target response only; no native absolute timestamp/current-step sequence or10Hz sustained qualification"
        report["status"] = "failed" if not all(checks.values()) else ("pending" if report["pending_gaps"] else "passed")
        code = {"passed": 0, "failed": 1, "pending": 2}[report["status"]]
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error), "traceback": traceback.format_exc(limit=15)})
    finally:
        if world is not None:
            try:
                world.pause()
                report["final_world"] = clock()
            except Exception as error:
                report["cleanup_errors"].append("world.pause/clock: " + str(error))
        if stage is not None:
            for path in [*sensors.values(), *([OWNED] if owned_created else [])]:
                try:
                    if stage.GetPrimAtPath(path).IsValid() and not stage.RemovePrim(path):
                        raise RuntimeError("RemovePrim returned false")
                    absent = not stage.GetPrimAtPath(path).IsValid()
                    report["cleanup_operations"].append({"path": path, "absent": absent})
                    if not absent:
                        raise RuntimeError("owned prim remains")
                except Exception as error:
                    report["cleanup_errors"].append(path + ": " + str(error))
        interface = None
        report.update(physics_step_calls=steps, elapsed_wall_s=time.monotonic() - started,
                      finished_at_utc=datetime.now(timezone.utc).isoformat(), sdk_close_requested=app is not None)
        if report["cleanup_errors"]:
            report["status"], code = "failed", 1
        save(args.output / "result.json", report)
        if app is not None:
            try:
                app.close()
                report["sdk_close_returned"] = True
            except BaseException as error:
                report["cleanup_errors"].append("app.close: " + str(error))
                report["status"], code = "failed", 1
            save(args.output / "result.json", report)
        print("PHYSX_PLANAR_ADMISSION", report["status"], flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
