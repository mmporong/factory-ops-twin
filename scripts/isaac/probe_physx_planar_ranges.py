"""Bounded SDK4.5 range/scene-envelope diagnosis; no robot motion or transport claim."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.output.is_absolute() or args.output.exists() or args.output.is_symlink():
        parser.error("output must be a new absolute directory")
    args.output.mkdir(parents=True)
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    from factory_ops.physx_planar import LIDAR_PARAMETERS
    minimum, maximum = float(LIDAR_PARAMETERS["min_range"]), float(LIDAR_PARAMETERS["max_range"])

    started = time.monotonic()
    report = dict(status="starting", scope=__doc__, cases=[], cleanup_errors=[],
        maximum_wall_s=180, maximum_physics_steps=600, physics_steps=0,
        parameters=dict(LIDAR_PARAMETERS), sdk_close_requested=False, sdk_close_returned=False,
        source_sha256={name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                       for name in ("scripts/isaac/probe_physx_planar_ranges.py", "factory_ops/physx_planar.py")})

    def save():
        report["elapsed_wall_s"] = time.monotonic() - started
        temporary = args.output / "result.tmp"
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(args.output / "result.json")

    def bound():
        if time.monotonic() - started > 180 or report["physics_steps"] >= 600:
            raise TimeoutError("native range diagnosis bound")

    app = world = stage = None
    owned = "/World/RangeBoundaryProbe"
    code = 1
    save()
    try:
        sdk = Path(os.environ["ISAAC_PATH"])
        report["sdk_version"] = (sdk / "VERSION").read_text().strip()
        if not report["sdk_version"].startswith("4.5"):
            raise ValueError("only existing SDK4.5 is supported")
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "multi_gpu": False, "width": 640, "height": 480})
        import numpy as np
        import omni.kit.commands
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.sensors.physx")
        from isaacsim.sensors.physx import _range_sensor
        from pxr import Gf, UsdGeom, UsdPhysics

        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60, device="cpu")
        world.pause()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, owned)
        sensor_path = owned + "/Lidar"
        ok, sensor = omni.kit.commands.execute("RangeSensorCreateLidar", path=sensor_path,
                                               parent=None, **dict(LIDAR_PARAMETERS))
        if not ok or not sensor:
            raise RuntimeError("owned lidar creation failed")
        transform = UsdGeom.Xformable(sensor.GetPrim()).AddTransformOp()
        target = UsdGeom.Cube.Define(stage, owned + "/Target")
        target.CreateSizeAttr(1.0)
        target_transform = target.AddTransformOp()
        target.AddScaleOp().Set(Gf.Vec3d(.02, 2., 2.))
        UsdPhysics.CollisionAPI.Apply(target.GetPrim())
        interface = _range_sensor.acquire_lidar_sensor_interface()
        world.reset()
        world.play()

        # These are authored diagnostic placements, not reconstructed robot poses.
        origins = ((0., 0., 1.), (3.6093505457687947, 4.564174283606593, 1.))
        distances = (.05, .099, .1, .10000001, .101, .25, 1., 29.99, 30., 30.01, 45.61, 46.)
        first_rejection = False
        for origin in origins:
            for yaw in (0., .5339742290318825, -2.6076184245579106):
                matrix = Gf.Matrix4d(1.)
                matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
                matrix.SetTranslateOnly(Gf.Vec3d(*origin))
                transform.Set(matrix)
                for distance in distances:
                    bound()
                    target_matrix = Gf.Matrix4d(1.)
                    target_matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
                    target_matrix.SetTranslateOnly(Gf.Vec3d(
                        origin[0] + (distance + .01) * math.cos(yaw),
                        origin[1] + (distance + .01) * math.sin(yaw), origin[2]))
                    target_transform.Set(target_matrix)
                    for _ in range(4):
                        bound()
                        world.step(render=True)
                        report["physics_steps"] += 1
                    arrays = {name: np.asarray(getattr(interface, getter)(sensor_path)).copy()
                              for name, getter in (("depth", "get_linear_depth_data"),
                                  ("intensity", "get_intensity_data"), ("azimuth", "get_azimuth_data"),
                                  ("zenith", "get_zenith_data"))}
                    depth, intensity = arrays["depth"], arrays["intensity"]
                    if depth.shape != (720, 1) or intensity.shape != depth.shape:
                        raise ValueError("unexpected native range dimensions")
                    hits = depth[intensity == 255]
                    # Python compares actual float32 values to exact authored limits.
                    invalid = [(i, j) for i in range(720) for j in (0,)
                               if int(intensity[i, j]) == 255 and
                               (not math.isfinite(float(depth[i, j])) or not minimum <= float(depth[i, j]) <= maximum)]
                    row = dict(origin_m=origin, yaw_rad=yaw, target_face_distance_m=distance,
                        world_time_s=float(world.current_time), physics_frame=int(world.current_time_step_index),
                        shapes={k: list(v.shape) for k, v in arrays.items()},
                        dtypes={k: str(v.dtype) for k, v in arrays.items()}, hit_count=int(hits.size),
                        hit_min_m=float(hits.min()) if hits.size and np.isfinite(hits).all() else None,
                        hit_max_m=float(hits.max()) if hits.size and np.isfinite(hits).all() else None,
                        invalid_count=len(invalid), invalid_samples=[dict(beam_index=i,
                            value_repr=repr(float(depth[i, j])), float64_hex=float(depth[i, j]).hex())
                            for i, j in invalid[:8]])
                    report["cases"].append(row)
                    if invalid and not first_rejection:
                        first_rejection = True
                        row["raw_files"] = {}
                        for name, array in arrays.items():
                            path = args.output / ("first-invalid-" + name + ".npy")
                            np.save(path, array, allow_pickle=False)
                            row["raw_files"][name] = dict(file=path.name,
                                sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                    save()
        report["status"] = "observed_out_of_range" if first_rejection else "no_out_of_range_in_authored_cases"
        code = 0
    except BaseException as error:
        report.update(status="failed", error={"type": type(error).__name__, "message": str(error),
                                               "traceback": traceback.format_exc(limit=12)})
    finally:
        if world is not None:
            try:
                world.pause()
            except Exception as error:
                report["cleanup_errors"].append("pause: " + str(error))
        if stage is not None:
            try:
                stage.RemovePrim(owned)
                if stage.GetPrimAtPath(owned).IsValid():
                    raise RuntimeError("owned probe remains")
            except Exception as error:
                report["cleanup_errors"].append("remove: " + str(error))
        if app is not None:
            report["sdk_close_requested"] = True
            save()
            try:
                app.close()
                report["sdk_close_returned"] = True
            except BaseException as error:
                report["cleanup_errors"].append("close: " + str(error))
        if report["cleanup_errors"]:
            code = 1
        save()
    print(json.dumps({key: report[key] for key in ("status", "elapsed_wall_s", "physics_steps", "cleanup_errors")}))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
