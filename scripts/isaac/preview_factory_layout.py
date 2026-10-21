"""Bounded native RGB layout preview; no ROS, fleet mission or production claim."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if args.output.exists() or len(args.run_id) != 32:
        raise ValueError("new output and 32-character preview identity required")
    args.output.mkdir(parents=True)
    sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
    report = {"run_id": args.run_id, "status": "starting", "scope": "authored layout RGB preview; no robot mission or factory throughput proof",
        "source_sha256": sha(__file__), "input_sha256": {n: sha(args.bundle/n) for n in
            ("layout.json", "visual-assets.json", "render-profile.json", "scene/factory-v3.usda")},
        "captures": [], "ROS_started": False, "bag_SQL_opened": False, "cleanup_errors": []}
    app = None
    annotator = product = None

    def save():
        (args.output/"result.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")

    save()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "width": 1280, "height": 720,
            "renderer": "RayTracedLighting", "multi_gpu": False})
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from pxr import UsdGeom, UsdPhysics, Gf
        import numpy as np
        from PIL import Image
        from scripts.isaac.factory_visuals import FactoryVisualRuntime
        context = omni.usd.get_context()
        context.open_stage(str(args.bundle/"scene/factory-v3.usda"))
        deadline = time.monotonic()+150
        while context.get_stage_loading_status()[2]:
            if time.monotonic() > deadline:
                raise TimeoutError("preview stage load exceeded bound")
            app.update()
        stage = context.get_stage()
        world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/60)
        visual = FactoryVisualRuntime(args.bundle/"layout.json", args.bundle/"scene/factory-v3.usda",
            args.output, 0, args.run_id, "semiconductor_layout_preview", config_directory=args.bundle)
        report["visual_setup"] = visual.setup(stage, UsdGeom, UsdPhysics, Gf)
        world.reset()
        world.pause()
        frozen = (int(world.current_time_step_index), float(world.current_time))
        for name, eye, target, focal in (
                ("overview", (16., -13., 24.), (16., 11., .7), 17.),
                ("test-handler", (26., 1., 12.), (19., 10., 1.), 26.)):
            camera = rep.create.camera(position=eye, look_at=target, focal_length=focal)
            product = rep.create.render_product(camera, (1280, 720))
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([product])
            rep.orchestrator.set_capture_on_play(False)
            rep.orchestrator.step(delta_time=0., rt_subframes=8)
            rep.orchestrator.wait_until_complete()
            data = np.asarray(annotator.get_data())
            if (data.ndim != 3 or data.shape[:2] != (720, 1280) or data.shape[2] < 3
                    or not np.isfinite(data).all() or not np.any(data[:, :, :3])
                    or frozen != (int(world.current_time_step_index), float(world.current_time))):
                raise RuntimeError("native RGB/frozen physics validation failed")
            path = args.output/(name+".png")
            Image.fromarray(data[:, :, :3].astype(np.uint8)).save(path)
            report["captures"].append({"path": path.name, "sha256": sha(path), "width": 1280, "height": 720,
                "camera_eye_xyz_m": eye, "camera_target_xyz_m": target, "focal_length_mm": focal,
                "physics_frame": frozen[0], "sim_time_s": frozen[1], "source": "native Replicator RGB; no generated image or interpolation"})
            annotator.detach(); annotator = None
            product.destroy(); product = None
            save()
        report["status"] = "passed"
    except BaseException as error:
        report.update(status="failed", error=type(error).__name__+": "+str(error), traceback=traceback.format_exc())
    finally:
        try:
            if annotator is not None and product is not None:
                annotator.detach()
            if product is not None:
                product.destroy()
            report["app_close_requested"] = app is not None
            # SDK shutdown can terminate the interpreter before Python resumes.
            # The outer Job receipt proves process closure after this report.
            save()
            if app is not None:
                app.close(wait_for_replicator=False)
        except BaseException as error:
            report["cleanup_errors"].append(str(error)); report["status"] = "failed"
        save()
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
