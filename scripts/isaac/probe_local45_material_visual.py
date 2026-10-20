"""Render a captured material snapshot after its source pair closes; not a replay."""
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from factory_ops.material_visual import project_empty_totes
from factory_ops.visual_state import canonical_sha256
from scripts.isaac.local_asset_probe import ASSET_PATH


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_inputs(config, output, render_run_id):
    if (not re.fullmatch(r"[0-9a-f]{32}", render_run_id) or not output.is_absolute()
            or output.exists() or output.is_symlink() or any(p.is_symlink() for p in output.parents)):
        raise ValueError("distinct render run ID and new absolute nonsymlink output required")
    cfg = json.loads(config.read_text(encoding="utf-8"))
    observation_path, closure_path = Path(cfg["observation_path"]), Path(cfg["closed_pair_host_path"])
    if (sha(observation_path) != cfg["observation_sha256"] or sha(closure_path) != cfg["closed_pair_host_sha256"]):
        raise ValueError("captured observation or closed pair SHA differs")
    observed = json.loads(observation_path.read_text(encoding="utf-8"))
    snapshot = observed["snapshot"]
    closed = json.loads(closure_path.read_text(encoding="utf-8"))
    source_run_id = observed["source_run_id"]
    if (snapshot["run_id"] != source_run_id or render_run_id == source_run_id or closed["run_id"] != source_run_id
            or closed.get("windows_closed") is not True or closed.get("linux_closed") is not True
            or closed.get("cleanup_errors") != []):
        raise ValueError("source pair must have verified closure and a distinct source/render identity")
    factory = Path(cfg["factory_directory"]).resolve(strict=True)
    cfg["resolved_scene_path"] = str(Path(cfg.get("scene_path", factory/"scene/factory-v3.usda")).resolve(strict=True))
    profile = json.loads((factory / "transport-profile.json").read_text(encoding="utf-8"))
    rows = project_empty_totes(snapshot, profile["material_flow"]["materials"],
        tuple(profile["initial_robots"]), ci_capacity=profile["load"]["ci_capacity"])
    return cfg, observed, factory, profile, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    cfg, observed, factory, profile, rows = load_inputs(args.config, args.output, args.run_id)
    args.output.mkdir(parents=True)
    report = {"run_id": args.run_id, "render_run_id": args.run_id, "source_run_id": observed["source_run_id"], "status": "starting",
        "source_sha256": sha(__file__), "config_sha256": sha(args.config), "observation_sha256": cfg["observation_sha256"],
        "source_snapshot_sha256": canonical_sha256(observed["snapshot"]), "source_snapshot": observed["snapshot"],
        "project_source_sha256": {name: sha(ROOT/name) for name in (
            "factory_ops/material_visual.py", "factory_ops/visual_state.py", "scripts/isaac/factory_visuals.py",
            "scripts/isaac/local45_fleet4_transport_scene.py", "scripts/isaac/local_asset_probe.py")},
        "factory_input_sha256": {**{name: sha(factory/name) for name in ("layout.json", "transport-profile.json")},
            "scene": sha(cfg["resolved_scene_path"])}, "scene_path": cfg["resolved_scene_path"],
        "asset_relative_path": ASSET_PATH,
        "observed_source_bytes_sha256": observed.get("source_bytes_sha256"), "source_event_sim_time_s": None,
        "scope": "static render-only fixture; periodic clock checkpoints are not exact event/capture timing; authored robot positions",
        "physical_loading_validated": False, "requires_host_closure": True, "captures": []}
    app = None
    code = 1

    def save():
        (args.output / "result.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")

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
        from scripts.isaac.factory_visuals import FactoryVisualRuntime
        from scripts.isaac.local45_fleet4_transport_scene import EmptyToteGeometry

        deadline = time.monotonic() + 120
        context = omni.usd.get_context()
        context.open_stage(cfg["resolved_scene_path"])
        while context.get_stage_loading_status()[2] > 0:
            if time.monotonic() > deadline:
                raise TimeoutError("render-only fixture stage load exceeded bound")
            app.update()
        stage = context.get_stage()
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        asset = get_assets_root_path().rstrip("/") + ASSET_PATH
        report["resolved_asset_url"] = asset
        layout = Layout.from_file(factory / "layout.json")
        visuals = FactoryVisualRuntime(layout_path=factory/"layout.json", base_scene_path=Path(cfg["resolved_scene_path"]),
            output=args.output, variant_seed=0, run_id=args.run_id, scenario_id="static_material_fixture", config_directory=factory)
        visuals.setup(stage, UsdGeom, UsdPhysics, Gf)
        definitions = {}
        for robot, station in profile["initial_robots"].items():
            path = "/World/MaterialFixture/" + robot
            add_reference_to_stage(usd_path=asset, prim_path=path)
            definitions[robot] = {"robot_id": robot, "prim_path": path}
        while context.get_stage_loading_status()[2] > 0:
            if time.monotonic() > deadline:
                raise TimeoutError("existing NovaCarter reference load exceeded bound")
            app.update()
        for robot, station in profile["initial_robots"].items():
            prim = stage.GetPrimAtPath(definitions[robot]["prim_path"])
            xy = {**layout.stations, **layout.pockets}[station]
            UsdGeom.Xformable(prim).AddTranslateOp(opSuffix="MaterialFixture").Set(Gf.Vec3d(*xy, .1))
        world.reset()
        world.pause()
        visuals.after_robots_loaded(stage, definitions)
        geometry = EmptyToteGeometry(visuals)
        placements = geometry.apply(rows)
        if placements is None:
            raise ValueError("closed material fixture exceeds existing visual support capacity")
        frozen = (int(world.current_time_step_index), float(world.current_time))
        camera = rep.create.camera(position=(29., -4., 22.), look_at=(16., 10., 0.))
        product = rep.create.render_product(camera, (640, 480))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        rep.orchestrator.set_capture_on_play(False)
        rep.orchestrator.step(delta_time=0.0, rt_subframes=8)
        rep.orchestrator.wait_until_complete()
        data = np.asarray(rgb.get_data())
        report["buffer_observation"] = {"shape": list(data.shape), "finite": bool(np.isfinite(data).all()),
            "nonzero": bool(np.any(data)), "before": list(frozen),
            "after": [int(world.current_time_step_index), float(world.current_time)]}
        if (data.ndim != 3 or data.shape[:2] != (480, 640) or data.shape[2] < 3
                or not np.isfinite(data).all() or not np.any(data[:, :, :3])
                or frozen != (int(world.current_time_step_index), float(world.current_time))):
            raise RuntimeError("render-only buffer or frozen physics validation failed")
        Image.fromarray(data[:, :, :3].astype(np.uint8)).save(args.output / "material.png")
        placed = geometry.capture_placements({"applied": {"placements": placements}})
        report.update(status="passed", frozen_physics_frame=frozen[0], frozen_world_time_s=frozen[1],
            placements=placed["applied"]["placements"], captures=[{"path": "material.png", "sha256": sha(args.output / "material.png")}])
        rgb.detach()
        product.destroy()
        code = 0
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
    finally:
        try:
            save()
        finally:
            if app is not None:
                app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
