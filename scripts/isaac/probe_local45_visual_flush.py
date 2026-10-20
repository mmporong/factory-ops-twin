"""Authored static visibility A/B fixture; root must run under a <=240s Windows Job."""
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# Fixed authored image criteria, not hardware calibration; never tuned after a run.
CRITERIA = {"roi_xyxy_fraction": [.30, .30, .70, .70], "minimum_reference_blue_pixels": 128,
    "minimum_amber_pixels": 64, "maximum_reference_blue_retained_fraction": .10,
    "blue_rgb_rule": {"b_minus_r_min": 30, "b_minus_g_min": 15, "b_min": 60},
    "amber_rgb_rule": {"r_minus_b_min": 50, "g_minus_b_min": 15, "r_minus_g_min": 15, "r_min": 90}}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if (not re.fullmatch(r"[0-9a-f]{32}", args.run_id) or not args.output.is_absolute()
            or args.output.exists() or args.output.is_symlink() or any(p.is_symlink() for p in args.output.parents)):
        raise ValueError("new absolute nonsymlink output and distinct fixture run hex required")
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    factory = Path(cfg["factory_directory"]).resolve(strict=True)
    scene = Path(cfg.get("scene_path", factory/"scene/factory-v3.usda")).resolve(strict=True)
    profile = json.loads((factory/"transport-profile.json").read_text(encoding="utf-8"))
    robot_id = cfg.get("robot_id", next(iter(profile["initial_robots"])))
    station = profile["initial_robots"][robot_id]
    args.output.mkdir(parents=True)
    report = {"run_id": args.run_id, "fixture_run_id": args.run_id, "render_run_id": args.run_id, "source_run_id": None,
        "source_event_sim_time_s": None, "status": "starting", "requires_host_closure": True,
        "scope": "authored static visibility transition; sequential A/B at one frozen physics state, not source034 replay",
        "physical_loading_validated": False, "live_mission_quality_validated": False,
        "source_sha256": sha(__file__), "config_sha256": sha(args.config),
        "project_source_sha256": {name: sha(ROOT/name) for name in (
            "scripts/isaac/local45_fleet4_transport_scene.py", "scripts/isaac/factory_visuals.py",
            "scripts/isaac/local_asset_probe.py", "factory_ops/layout.py", "factory_ops/visual_state.py")},
        "factory_input_sha256": {"layout.json": sha(factory/"layout.json"),
            "transport-profile.json": sha(factory/"transport-profile.json"), "scene": sha(scene)},
        "criteria": CRITERIA, "robot_id": robot_id, "authored_station": station,
        "internal_wall_limit_s": 210, "required_host_wall_limit_s": 240, "captures": {},
        "baseline_lag_observed": None, "candidate_current_state_verified": False,
        "quality_effect_verified": False}
    app = rgb = product = None
    code = 1
    deadline = time.monotonic()+210

    def save():
        (args.output/"result.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")

    def bounded():
        if time.monotonic() > deadline:
            raise TimeoutError("visibility fixture exceeded internal wall bound")

    save()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "width": 640, "height": 480, "multi_gpu": False,
                             "renderer": "RayTracedLighting"})
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.storage.native import get_assets_root_path
        from pxr import UsdGeom, UsdPhysics, Gf
        from PIL import Image
        import numpy as np
        from factory_ops.layout import Layout
        from scripts.isaac.local_asset_probe import ASSET_PATH
        from scripts.isaac.factory_visuals import FactoryVisualRuntime, TOTE_SIZE_M, _session_edit_target
        from scripts.isaac.local45_fleet4_transport_scene import EmptyToteGeometry

        bounded()
        context = omni.usd.get_context()
        context.open_stage(str(scene))
        while context.get_stage_loading_status()[2] > 0:
            bounded()
            app.update()
        stage = context.get_stage()
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        layout = Layout.from_file(factory/"layout.json")
        visuals = FactoryVisualRuntime(layout_path=factory/"layout.json", base_scene_path=scene,
            output=args.output, variant_seed=0, run_id=args.run_id,
            scenario_id="authored_visibility_fixture", config_directory=factory)
        visuals.setup(stage, UsdGeom, UsdPhysics, Gf)
        asset = get_assets_root_path().rstrip("/")+ASSET_PATH
        robot_path = "/World/VisibilityFixture/"+robot_id
        report["asset_relative_path"], report["resolved_asset_url"] = ASSET_PATH, asset
        add_reference_to_stage(usd_path=asset, prim_path=robot_path)
        while context.get_stage_loading_status()[2] > 0:
            bounded()
            app.update()
        xy = {**layout.stations, **layout.pockets}[station]
        UsdGeom.Xformable(stage.GetPrimAtPath(robot_path)).AddTranslateOp(opSuffix="VisibilityFixture").Set(Gf.Vec3d(*xy, .1))
        world.reset()
        world.pause()
        visuals.after_robots_loaded(stage, {robot_id: {"robot_id": robot_id, "prim_path": robot_path}})
        cargo_parent = visuals.robot_paths[robot_id]+"/FactoryVisualCargo"
        placement = {"parent_path": cargo_parent, "translate_xyz_m": [0., 0., .52], "size_xyz_m": list(TOTE_SIZE_M)}
        with _session_edit_target(stage):
            blue_path = visuals._ensure_payload_copy({"payload_id": "FIXTURE_BLUE", "location": robot_id,
                "slot": 0, "state": "fixture_visible_blue"}, placement)
        blue = UsdGeom.Imageable(stage.GetPrimAtPath(blue_path))
        empty = EmptyToteGeometry(visuals)
        empty_rows = [{"tote_id": "FIXTURE_EMPTY", "robot_id": robot_id, "location": robot_id,
                       "slot": 0, "state": "returning", "task_id": "fixture_only"}]
        placements = empty.apply(empty_rows)
        if placements is None:
            raise ValueError("fixture-only empty tote support was unavailable")
        empty.apply([])
        amber_path = placements[0]["prim_path"]
        center = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(blue_path)).ExtractTranslation()
        camera = rep.create.camera(position=tuple(float(center[i])+v for i, v in enumerate((1.2, -1.2, 1.5))),
                                   look_at=tuple(float(v) for v in center))
        product = rep.create.render_product(camera, (640, 480))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        rep.orchestrator.set_capture_on_play(False)

        def state():
            cache = UsdGeom.XformCache()
            matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(visuals.robot_paths[robot_id]))
            values = [[float(matrix[i][j]) for j in range(4)] for i in range(4)]
            stamp = float(world.current_time)
            if not all(math.isfinite(v) for row in values for v in row) or not math.isfinite(stamp):
                raise ValueError("fixture pose/time contains nonfinite native data")
            return {"physics_frame": int(world.current_time_step_index), "world_time_s": stamp,
                    "robot_chassis_world_transform": values}

        frozen = state()
        report["frozen_state"] = frozen

        def usd_state():
            cache = UsdGeom.XformCache()
            result = {}
            for label, path in (("blue", blue_path), ("amber_hollow", amber_path)):
                prim = stage.GetPrimAtPath(path)
                imageable = UsdGeom.Imageable(prim)
                matrix = cache.GetLocalToWorldTransform(prim)
                result[label] = {"prim_path": path, "authored_visibility": str(imageable.GetVisibilityAttr().Get()),
                    "computed_visibility": str(imageable.ComputeVisibility()),
                    "captured_world_transform": [[float(matrix[i][j]) for j in range(4)] for i in range(4)]}
            return result

        def flush():
            bounded()
            rep.orchestrator.step(delta_time=0.0, rt_subframes=8, pause_timeline=True)
            rep.orchestrator.wait_until_complete()
            bounded()

        def readback(name):
            bounded()
            data = np.asarray(rgb.get_data()).copy()
            after = state()
            row = {"shape": list(data.shape), "dtype": str(data.dtype), "finite": bool(np.isfinite(data).all()),
                "nonzero": bool(np.any(data)), "before": frozen, "after": after,
                "frozen_state_unchanged": after == frozen, "usd": usd_state(),
                "raw_buffer_sha256": hashlib.sha256(data.tobytes()).hexdigest()}
            report["captures"][name] = row
            if after != frozen:
                raise RuntimeError("visibility capture changed exact frozen World frame/time/pose")
            row["valid_rgb"] = (data.ndim == 3 and data.shape[:2] == (480, 640)
                and data.shape[2] >= 3 and row["finite"] and bool(np.any(data[:, :, :3])))
            if not row["valid_rgb"]:
                return None, None
            color = data[:, :, :3].astype(np.int32)
            r, g, b = (color[:, :, i] for i in range(3))
            left, top, right, bottom = CRITERIA["roi_xyxy_fraction"]
            roi = np.zeros((480, 640), dtype=bool)
            roi[int(top*480):int(bottom*480), int(left*640):int(right*640)] = True
            bc, ac = CRITERIA["blue_rgb_rule"], CRITERIA["amber_rgb_rule"]
            blue_mask = roi & (b-r >= bc["b_minus_r_min"]) & (b-g >= bc["b_minus_g_min"]) & (b >= bc["b_min"])
            amber_mask = roi & (r-b >= ac["r_minus_b_min"]) & (g-b >= ac["g_minus_b_min"]) & (r-g >= ac["r_minus_g_min"]) & (r >= ac["r_min"])
            row.update(roi_pixel_count=int(roi.sum()), blue_pixels=int(blue_mask.sum()), amber_pixels=int(amber_mask.sum()),
                       rgb_min=int(color.min()), rgb_max=int(color.max()))
            path = args.output/(name+".png")
            Image.fromarray(data[:, :, :3].astype(np.uint8)).save(path)
            row.update(path=path.name, sha256=sha(path))
            return blue_mask, amber_mask

        flush()
        reference_blue, _ = readback("reference_blue")
        if (reference_blue is None or int(reference_blue.sum()) < CRITERIA["minimum_reference_blue_pixels"]
                or usd_state()["blue"]["computed_visibility"] != "inherited"
                or usd_state()["amber_hollow"]["computed_visibility"] != "invisible"):
            raise RuntimeError("reference blue cargo did not meet fixed authored visibility/image criteria")
        with _session_edit_target(stage):
            blue.CreateVisibilityAttr().Set("invisible")
        if empty.apply(empty_rows) is None:
            raise ValueError("fixture empty tote toggle failed")
        report["toggle"] = {"scope": "authored fixture-only visibility edit; no source event time",
            "state": state(), "usd": usd_state(), "placement": empty.capture_placements({"applied": {"placements": placements}})}
        for _ in range(4):
            bounded()
            world.render()
        baseline_blue, baseline_amber = readback("baseline_world_render4")
        flush()
        candidate_blue, candidate_amber = readback("candidate_rep_step0_wait")

        def current(name, blue_mask, amber_mask):
            if blue_mask is None:
                return False
            retained = int((blue_mask & reference_blue).sum())
            fraction = retained/int(reference_blue.sum())
            row = report["captures"][name]
            row.update(reference_blue_pixels_retained=retained, reference_blue_retained_fraction=fraction)
            return (fraction <= CRITERIA["maximum_reference_blue_retained_fraction"]
                and int(amber_mask.sum()) >= CRITERIA["minimum_amber_pixels"]
                and row["usd"]["blue"]["computed_visibility"] == "invisible"
                and row["usd"]["amber_hollow"]["computed_visibility"] == "inherited")

        baseline_current = current("baseline_world_render4", baseline_blue, baseline_amber)
        candidate_current = current("candidate_rep_step0_wait", candidate_blue, candidate_amber)
        baseline_valid = report["captures"]["baseline_world_render4"]["valid_rgb"]
        report.update(baseline_current_state_verified=baseline_current,
            baseline_lag_observed=bool(baseline_valid and not baseline_current),
            candidate_current_state_verified=candidate_current,
            quality_effect_verified=bool(baseline_valid and not baseline_current and candidate_current))
        if not candidate_current:
            raise RuntimeError("candidate did not meet fixed authored transition criteria")
        report["status"] = "passed" if report["quality_effect_verified"] else "limited"
        report["conclusion"] = ("Baseline lag and candidate current image observed at identical frozen state."
            if report["quality_effect_verified"] else
            "Candidate image current; baseline lag not established. No comparative fix claim.")
        code = 0 if report["quality_effect_verified"] else 2
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
    finally:
        errors = []
        for label, resource, method in (("rgb", rgb, "detach"), ("product", product, "destroy")):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except BaseException as error:
                    errors.append(f"{label}: {type(error).__name__}: {error}")
        if errors:
            report.update(status="failed", cleanup_errors=errors)
            code = 1
        report["intended_exit_code"] = code
        report["final_owned_app_close_attempted"] = app is not None
        try:
            save()
        finally:
            if app is not None:
                app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
