"""Bounded static three-RGB cleanup probe; requires an external Windows Job."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--generation-state', choices=('stepped', 'paused'), default='paused')
    args = parser.parse_args()
    if not args.output.is_absolute() or args.output.exists():
        raise ValueError('new absolute output required')
    args.output.mkdir(parents=True)
    report = {'run_id': args.run_id, 'status': 'starting',
              'scope': 'Static three-product capture shutdown only; not transport or036 retrospective pass.',
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    app = session = None
    code = 1

    def save(phase):
        report['phase'] = phase
        (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')

    save('starting')
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'width': 640, 'height': 360,
                             'multi_gpu': False, 'renderer': 'RayTracedLighting'})
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from pxr import UsdGeom, UsdLux, Gf
        from PIL import Image
        import numpy as np
        from types import SimpleNamespace
        from scripts.isaac.robot_qualification import CaptureSession, CameraSpec, _camera
        world = World(physics_dt=1/60, rendering_dt=1/60, stage_units_in_meters=1.)
        stage = omni.usd.get_context().get_stage()
        cube = UsdGeom.Cube.Define(stage, '/World/ProbeCube')
        cube.CreateSizeAttr(.5)
        cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, .5))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.1, .5, .8)])

        def light():
            UsdLux.DomeLight.Define(stage, '/World/ProbeLight').CreateIntensityAttr(1000.)

        def save_rgb(data, path):
            data = np.asarray(data)
            if data.ndim != 3 or data.shape[:2] != (360, 640) or not np.isfinite(data).all():
                raise ValueError(f'invalid RGB shape={data.shape}, dtype={data.dtype}')
            Image.fromarray(data[:, :, :3].astype(np.uint8)).save(path)

        manager = SimpleNamespace(get_simulation_time=lambda: float(world.current_time))
        session = CaptureSession(args.output, app, stage, rep, UsdGeom, Gf, manager, save_rgb,
            medium_camera=CameraSpec('/World/ProbeCamera', (2., -2., 2.), (0., 0., .5), 'world'),
            dome_light_factory=light, resolution=(640, 360), stop_before_drain=True)
        session.capture_load_preview()
        session.capture_factory_overview()
        camera = _camera(stage, Gf, UsdGeom, '/World/ProbeThird', (-2., -2., 2.), (0., 0., .5), (640, 360))
        product = session._product(str(camera.GetPath()), (640, 360))
        third = session._annotator('rgb', product)
        session._frozen_step('third RGB priming')
        world.reset()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(6):
            world.step(render=True)
        timeline.pause()
        for _ in range(4):
            world.render()
        save_rgb(third.get_data(), args.output/'third.png')
        if args.generation_state == 'paused':
            rep.orchestrator.run(start_timeline=False)
            rep.orchestrator.pause()
            app.update()
            if not rep.orchestrator.get_is_paused():
                raise ValueError('authored public paused generation state was not reached')
        report['authored_generation_state'] = args.generation_state
        report['reproduction_scope'] = 'Public API authored state, not retained036 orchestrator state.'
        before = {'world_s': float(world.current_time), 'physics_frame': int(world.current_time_step_index),
                  'orchestrator_status': str(rep.orchestrator.get_status())}
        report.update(before=before, products=len(session.products), annotators=len(session.annotators))
        save('before_session_close')
        started = time.monotonic()
        session.close()
        elapsed = time.monotonic()-started
        session = None
        after = {'world_s': float(world.current_time), 'physics_frame': int(world.current_time_step_index),
                 'orchestrator_status': str(rep.orchestrator.get_status())}
        report.update(after=after, close_wall_s=elapsed, frozen_state_unchanged=(before['world_s']==after['world_s']
                      and before['physics_frame']==after['physics_frame']),
                      generation_stopped=rep.orchestrator.get_status()==rep.orchestrator.Status.STOPPED)
        if not report['frozen_state_unchanged'] or not report['generation_stopped']:
            raise ValueError('shutdown changed frozen state or failed to stop generation')
        report['status']='passed'
        code=0
        save('closed')
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
        save('failed')
    finally:
        if session is not None:
            report['unfinished_session_owned_by_external_job']=True
        report['intended_exit_code']=code
        save(report['phase'])
        if app is not None:
            app.close(wait_for_replicator=False)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
