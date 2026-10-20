"""Isolated moving-wall PhysX range/cadence fixture; never a fleet runner."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from numbers import Integral
import os
from pathlib import Path
import re
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from factory_ops.local45_stepping import Local45Stepper, physics_state
from factory_ops.physx_planar import LIDAR_PARAMETERS, PROFILE_SHA256


DT = 1 / 60
MAX_WALL_S = 210
WARMUP_STEPS = 60
BLOCK_STEPS = 6
WALL_X = (6., 4., 7., 6., 4., 7.)
WALL_SIZE = (.2, 4., 2.)
SENSOR_PATH = "/World/PhysxCadence067/Lidar"
WALL_PATH = "/World/PhysxCadence067/Wall"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def save(path, report):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    output = args.output
    if (not output.is_absolute() or output.exists() or output.is_symlink()
            or any(parent.is_symlink() for parent in output.parents)
            or os.path.normcase(str(output)) != os.path.normcase(str(output.resolve()))):
        parser.error("output must be new, absolute, canonical and nonsymlink")
    if not re.fullmatch(r"[0-9a-f]{32}", args.run_id):
        parser.error("run-id must contain32 lowercase hex characters")
    fixture = {"physics_dt_s": DT, "rendering_dt_s": DT, "warmup_steps": WARMUP_STEPS,
        "block_steps": BLOCK_STEPS, "wall_center_x_m": list(WALL_X), "wall_size_m": list(WALL_SIZE),
        "lidar_origin_m": [0., 0., 1.], "candidate_render_every_steps": 6,
        "maximum_wall_s": MAX_WALL_S, "lidar_parameters": dict(LIDAR_PARAMETERS)}
    source_bytes = Path(__file__).read_bytes()
    dependencies = ("factory_ops/__init__.py", "factory_ops/physx_planar.py", "factory_ops/local45_stepping.py")
    output.mkdir(parents=True, exist_ok=False)
    (output / "arrays").mkdir()
    (output / "probe-source.py").write_bytes(source_bytes)
    save(output / "fixture.json", fixture)
    report = {"schema_name": "local45-physx-cadence-probe", "schema_version": 1,
        "run_id": args.run_id, "status": "starting", "stage": "startup",
        "probe_sha256": sha(source_bytes), "project_source_sha256": {
            name: sha((ROOT / name).read_bytes()) for name in dependencies},
        "api_reference_sha256": {"factory_ops/isaac45_runtime.py": sha((ROOT / "factory_ops/isaac45_runtime.py").read_bytes())},
        "configuration": fixture, "configuration_sha256": sha((output / "fixture.json").read_bytes()),
        "lidar_profile_sha256": PROFILE_SHA256, "phases": {}, "primary_error": None,
        "cleanup_errors": [], "native_timestamp": None,
        "observation_time_source": "actual World observation, not native acquisition timestamp",
        "current_frame_native_timestamp_verified": False, "requires_host_closure": True,
        "sdk_close_requested": False, "sdk_close_returned": False,
        "scope": "one static lidar and relocated collision wall; fixture performance only; no AMR/ROS/factory speed or transport claim"}
    for name in ("baseline_app_update", "candidate_stepper6"):
        report["phases"][name] = {"status": "not_run", "blocks": []}
    started = time.monotonic()
    app = world = None
    code = 1

    def bound():
        if time.monotonic() - started >= MAX_WALL_S:
            raise TimeoutError("210s cooperative probe bound; host Job containment also required")

    def checkpoint():
        report["elapsed_wall_s"] = time.monotonic() - started
        save(output / "result.json", report)

    try:
        checkpoint()
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "sync_loads": True, "multi_gpu": False,
                             "width": 640, "height": 480})
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.sensors.physx")
        from isaacsim.core.api import World
        from isaacsim.sensors.physx import _range_sensor
        import numpy as np
        import omni.kit.commands
        import omni.usd
        from pxr import Gf, UsdGeom, UsdPhysics

        bound()
        world = World(stage_units_in_meters=1., physics_dt=DT, rendering_dt=DT, device="cpu")
        world.get_physics_context().enable_gpu_dynamics(False)
        world.pause()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        if UsdGeom.GetStageMetersPerUnit(stage) != 1.:
            raise RuntimeError("probe requires meter stage units")
        cube = UsdGeom.Cube.Define(stage, WALL_PATH)
        cube.CreateSizeAttr(1.)
        translate = cube.AddTranslateOp()
        translate.Set(Gf.Vec3d(WALL_X[0], 0., 1.))
        cube.AddScaleOp().Set(Gf.Vec3d(*WALL_SIZE))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        world.reset()
        world.pause()
        interface = _range_sensor.acquire_lidar_sensor_interface()
        ok, lidar = omni.kit.commands.execute("RangeSensorCreateLidar", path=SENSOR_PATH,
                                             parent=None, **dict(LIDAR_PARAMETERS))
        if not ok or not lidar or str(lidar.GetPath()) != SENSOR_PATH or not lidar.GetPrim().IsValid():
            raise RuntimeError("PhysX command did not create the exact owned lidar")
        sensor_xf = UsdGeom.Xformable(lidar.GetPrim())
        sensor_xf.ClearXformOpOrder()
        sensor_xf.AddTranslateOp(opSuffix="Cadence067").Set(Gf.Vec3d(0., 0., 1.))
        world.play()

        def baseline_step():
            bound()
            before = physics_state(world)
            app.update()
            after = physics_state(world)
            if (after[0] != before[0]+1
                    or not math.isclose(after[1]-before[1], DT, rel_tol=1e-7, abs_tol=1e-9)):
                raise RuntimeError("app.update did not advance exactly one actual physics frame")
            bound()

        report["stage"] = "warmup"
        warm_before = physics_state(world)
        for _ in range(WARMUP_STEPS):
            baseline_step()
        warm_after = physics_state(world)
        report["warmup"] = {"before": list(warm_before), "after": list(warm_after)}
        checkpoint()
        previous = None

        def snapshot(phase, block, center):
            nonlocal previous
            bound()
            observed_before = physics_state(world)
            row = {"block": block, "authored_wall_center_x_m": center,
                "world_observed_before": list(observed_before), "arrays": {}, "checks": {}}
            report["phases"][phase]["blocks"].append(row)
            arrays = {}
            for field, method in (("depth", "get_linear_depth_data"), ("encoded_depth", "get_depth_data"),
                                  ("intensity", "get_intensity_data"), ("azimuth", "get_azimuth_data"),
                                  ("zenith", "get_zenith_data")):
                buffer = np.asarray(getattr(interface, method)(SENSOR_PATH)).copy()
                if buffer.dtype.hasobject:
                    raise RuntimeError("object-valued native array cannot be preserved as numeric evidence")
                arrays[field] = buffer
                row["arrays"][field] = {"shape": [int(n) for n in buffer.shape],
                    "dtype": str(buffer.dtype), "raw_bytes_sha256": sha(buffer.tobytes())}
            file = output / "arrays" / f"{phase}-{block:02}.npz"
            np.savez(file, **arrays)
            row["array_file"] = str(file.relative_to(output))
            row["array_file_sha256"] = sha(file.read_bytes())
            observed_after = physics_state(world)
            row["world_observed_after"] = list(observed_after)
            checkpoint()  # Preserve native buffers before any range-quality rejection.
            if observed_after != observed_before:
                raise RuntimeError("World advanced during native buffer read")
            ticks = interface.get_num_cols_ticked(SENSOR_PATH)
            if isinstance(ticks, bool) or not isinstance(ticks, Integral) or ticks != 720:
                raise RuntimeError("native populated column count is not720")
            row["num_cols_ticked"] = int(ticks)
            if (arrays["depth"].shape != (720, 1) or arrays["intensity"].shape != (720, 1)
                    or arrays["encoded_depth"].shape != (720, 1) or arrays["azimuth"].shape != (720,)
                    or arrays["zenith"].shape != (1,) or arrays["encoded_depth"].dtype != np.dtype("uint16")):
                raise RuntimeError("native arrays differ from720x1/uint16 depth design")
            azimuth, zenith = arrays["azimuth"].astype(float), float(arrays["zenith"][0])
            if not np.isfinite(azimuth).all() or not math.isfinite(zenith) or abs(zenith) > 1e-6:
                raise RuntimeError("native azimuth/zenith is not a finite horizontal scan")
            beam = int(np.argmin(np.abs(np.arctan2(np.sin(azimuth), np.cos(azimuth)))))
            angle = float(azimuth[beam])
            half_bin = math.radians(float(LIDAR_PARAMETERS["horizontal_resolution"]) / 2)
            if abs(math.atan2(math.sin(angle), math.cos(angle))) > half_bin + 2e-5:
                raise RuntimeError("closest actual forward ray lies outside the half-bin geometry")
            front = center-WALL_SIZE[0]/2
            expected = front / (math.cos(angle)*math.cos(zenith))
            if abs(expected*math.cos(zenith)*math.sin(angle)) > WALL_SIZE[1]/2:
                raise RuntimeError("selected ray does not intersect the wall front")
            quantum = (float(LIDAR_PARAMETERS["max_range"])-float(LIDAR_PARAMETERS["min_range"])) / 65535
            # Authored oracle budget: four uint16 depth quanta plus half-bin projection and float margin.
            tolerance = 4*quantum + front*(1/math.cos(half_bin)-1) + 1e-4
            measured = float(arrays["depth"][beam, 0])
            intensity = int(arrays["intensity"][beam, 0])
            changed = previous is not None and previous["wall_x"] != center
            responds = not changed or (math.isfinite(measured) and measured != previous["range"]
                and row["arrays"]["depth"]["raw_bytes_sha256"] != previous["depth_sha256"])
            row.update(forward_beam=beam, actual_azimuth_rad=angle, actual_zenith_rad=zenith,
                expected_front_range_m=expected, observed_forward_range_m=measured if math.isfinite(measured) else None,
                observed_intensity=intensity, depth_quantum_design_m=quantum, tolerance_design_m=tolerance,
                tolerance_scope="authored uint16/beam geometry oracle budget, not measured factory accuracy")
            row["checks"] = {"front_range": math.isfinite(measured) and abs(measured-expected) <= tolerance,
                "hit": intensity == 255, "changed_wall_changed_native_depth": responds}
            checkpoint()
            if not all(row["checks"].values()):
                raise RuntimeError("native range did not respond to the changed wall within6actual steps")
            previous = {"wall_x": center, "range": measured,
                        "depth_sha256": row["arrays"]["depth"]["raw_bytes_sha256"]}

        for name in report["phases"]:
            report["stage"] = name
            phase = report["phases"][name]
            phase.update(status="running", before=list(physics_state(world)), stepping_wall_s=0.,
                         app_update_calls=0, stepper_render_calls=0)
            phase_started = time.monotonic()
            stepper = Local45Stepper(world, DT, render_every_steps=6) if name == "candidate_stepper6" else None
            try:
                for block, center in enumerate(WALL_X):
                    bound()
                    if not translate.Set(Gf.Vec3d(center, 0., 1.)):
                        raise RuntimeError("wall geometry write failed")
                    if tuple(float(v) for v in translate.Get()) != (center, 0., 1.):
                        raise RuntimeError("wall transform readback differs from authored center")
                    before = physics_state(world)
                    stepping_started = time.monotonic()
                    for _ in range(BLOCK_STEPS):
                        bound()
                        if stepper is None:
                            baseline_step()
                            phase["app_update_calls"] += 1
                        else:
                            tick = stepper.step()
                            phase["stepper_render_calls"] += int(tick["rendered"])
                    phase["stepping_wall_s"] += time.monotonic()-stepping_started
                    if physics_state(world)[0]-before[0] != BLOCK_STEPS:
                        raise RuntimeError("wall block did not span6actual physics frames")
                    snapshot(name, block, center)
                phase["status"] = "passed"
            finally:
                phase["wall_s_including_evidence_io"] = time.monotonic()-phase_started
                phase["after"] = list(physics_state(world))
                phase["actual_steps"] = phase["after"][0]-phase["before"][0]
                phase["actual_elapsed_sim_s"] = phase["after"][1]-phase["before"][1]
                if phase["status"] != "passed":
                    phase["status"] = "failed"
                checkpoint()
        report["status"] = "passed_range_change_fixture"
        report["range_change_verified_within_six_steps"] = True
        report["fixture_stepping_wall_ratio_baseline_over_candidate"] = (
            report["phases"]["baseline_app_update"]["stepping_wall_s"] /
            report["phases"]["candidate_stepper6"]["stepping_wall_s"])
        code = 0
    except BaseException as error:
        report["status"] = "failed"
        report["primary_error"] = {"type": type(error).__name__, "message": str(error),
                                   "traceback": traceback.format_exc()}
    finally:
        report["sdk_close_requested"] = app is not None
        try:
            checkpoint()  # Primary result is persisted before potentially blocking native teardown.
        finally:
            if app is not None:
                try:
                    app.close()
                except BaseException as error:
                    report["cleanup_errors"].append(f"{type(error).__name__}: {error}")
                    code = 1
                else:
                    report["sdk_close_returned"] = True
                checkpoint()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
