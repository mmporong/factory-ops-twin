"""Capture bounded stills of an existing USD composition with a stopped timeline."""
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


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    scene = args.scene.resolve(strict=True)
    config = args.config.resolve(strict=True)
    if not args.output.is_absolute() or args.output.exists():
        parser.error('a new absolute output directory is required')
    profile = json.loads((config / 'render-profile.json').read_text(encoding='utf-8'))
    cameras = [dict(next(c for c in profile['cameras'] if c['id'] == name))
               for name in ('portfolio_overview', 'cell_process', 'west_material_flow')]
    cameras[0]['focal_length_mm'] = 18
    args.output.mkdir(parents=True)
    report = {
        'status': 'starting', 'execution_mode': 'static_scene_render',
        'scope': 'Authored factory stills, stopped timeline; no robot spawn, transport or physical validation',
        'source_sha256': sha(Path(__file__)),
        'input_sha256': {p.name: sha(p) for p in (
            scene, scene.parent / 'factory-v3.usda', scene.parent / 'factory-v3.visual.usda',
            config / 'layout.json', config / 'render-profile.json')},
        'sdk_renderer': 'RayTracedLighting', 'image_size_px': [1280, 720],
        'camera_override': 'Overview focal18mm to retain full campus margins; profile file unchanged',
        'captures': [], 'cleanup_errors': [], 'requires_host_closure': True,
    }
    started = time.monotonic()
    app = product = rgb = None

    def save():
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')

    def bound():
        if time.monotonic() - started > 180:
            raise TimeoutError('static capture exceeded180s internal bound')

    save()
    code = 1
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'width': 1280, 'height': 720,
                             'multi_gpu': False, 'renderer': 'RayTracedLighting'})
        import numpy as np
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdLux, Gf
        from PIL import Image
        from factory_ops.layout import Layout
        from scripts.isaac.factory_visuals import light_positions

        context = omni.usd.get_context()
        context.open_stage(str(scene))
        while context.get_stage_loading_status()[2] > 0:
            bound()
            app.update()
        stage = context.get_stage()
        if not stage or not stage.GetDefaultPrim():
            raise RuntimeError('factory stage is unavailable')
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        frozen_time = timeline.get_current_time()
        layout = Layout.from_file(config / 'layout.json')
        for index, (x, y) in enumerate(light_positions(layout)):
            light = UsdLux.RectLight.Define(stage, f'/World/CaptureLights/Panel_{index}')
            light.CreateWidthAttr(5.)
            light.CreateHeightAttr(.8)
            light.CreateIntensityAttr(6000.)
            light.CreateColorTemperatureAttr(5200.)
            light.CreateEnableColorTemperatureAttr(True)
            UsdGeom.Xformable(light.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(x, y, 5.35))
        UsdLux.DomeLight.Define(stage, '/World/CaptureLights/Ambient').CreateIntensityAttr(1000.)
        # USD owns the pose: a Replicator-created camera's graph can overwrite
        # external xform edits when the next orchestrator step executes.
        camera_schema = UsdGeom.Camera.Define(stage, '/World/StaticCapture/Camera')
        camera_schema.CreateHorizontalApertureAttr(24.)
        camera_schema.CreateVerticalApertureAttr(13.5)
        camera_schema.CreateClippingRangeAttr(Gf.Vec2f(.05, 200.))
        camera_xform = UsdGeom.Xformable(camera_schema.GetPrim())
        camera_pose = camera_xform.AddTransformOp()
        product = rep.create.render_product(str(camera_schema.GetPath()), (1280, 720), name='StaticCapture')
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach([product])
        rep.orchestrator.set_capture_on_play(False)
        for camera_config in cameras:
            bound()
            view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*camera_config['eye_xyz_m']),
                                         Gf.Vec3d(*camera_config['target_xyz_m']), Gf.Vec3d(0., 0., 1.))
            camera_pose.Set(view.GetInverse())
            camera_schema.GetFocalLengthAttr().Set(camera_config['focal_length_mm'])
            for attempt in range(3):
                bound()
                rep.orchestrator.step(delta_time=0., rt_subframes=8, pause_timeline=True)
                rep.orchestrator.wait_until_complete()
                pixels = np.asarray(rgb.get_data())
                valid = (pixels.shape[:2] == (720, 1280) and pixels.ndim == 3
                         and pixels.shape[2] >= 3 and pixels[:, :, :3].std() >= 1)
                if valid:
                    break
            if not valid:
                report['invalid_buffer'] = {'camera': camera_config['id'], 'shape': list(pixels.shape),
                                            'attempts': attempt + 1}
                raise RuntimeError('RGB buffer unavailable or blank after bounded warmup')
            if timeline.is_playing() or abs(timeline.get_current_time() - frozen_time) > 1e-8:
                raise RuntimeError('static capture advanced the physics timeline')
            image_path = args.output / (camera_config['id'] + '.png')
            Image.fromarray(pixels[:, :, :3].astype(np.uint8)).save(image_path)
            world = camera_xform.ComputeLocalToWorldTransform(0.)
            actual_eye = [float(v) for v in world.ExtractTranslation()]
            if any(abs(a - b) > 1e-6 for a, b in zip(actual_eye, camera_config['eye_xyz_m'])):
                raise RuntimeError('capture camera world pose differs from requested pose')
            report['captures'].append({'path': image_path.name, 'sha256': sha(image_path), 'camera': camera_config,
                                       'camera_world_eye_m': actual_eye, 'buffer_attempts': attempt + 1})
            save()
        report.update(status='passed', frozen_timeline_s=frozen_time)
        code = 0
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    finally:
        for label, action in (('annotator', lambda: rgb.detach() if rgb else None),
                              ('product', lambda: product.destroy() if product else None)):
            try:
                action()
            except BaseException as error:
                report['cleanup_errors'].append(f'{label}: {error}')
        report['elapsed_wall_s'] = time.monotonic() - started
        save()
        if app is not None:
            app.close(wait_for_replicator=False)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
