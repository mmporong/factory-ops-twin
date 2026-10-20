"""One two-Carter physical parking/drive comparison; no ROS, rendering or GT steering."""
from pathlib import Path
import argparse, hashlib, json, math, sys, time, traceback

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source-root', type=Path, required=True)
p.add_argument('--scene', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--run-id', required=True)
a = p.parse_args()
assert a.source_root.is_absolute() and a.scene.is_absolute() and a.output.is_absolute()
assert not a.output.exists()
a.output.mkdir(parents=True)
started = time.monotonic()
report = dict(run_id=a.run_id, status='starting', scope='2 identical Carter models, same floor/yaw; no ROS/render/sensor/transport qualification',
    gt_use='evaluation only; never fed to wheel control', physics_dt_s=1/60,
    primary_error=None, cleanup_errors=[], sdk_close_requested=False, sdk_close_returned=False,
    requires_host_closure=True, phases=[], source_files={})

def save():
    tmp=a.output/'report.tmp'
    tmp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    tmp.replace(a.output/'report.json')

save()
sys.path.insert(0,str(a.source_root))
for rel in ('factory_ops/isaac45_runtime.py','factory_ops/wheel_parking_hold.py'):
    report['source_files'][rel]=hashlib.sha256((a.source_root/rel).read_bytes()).hexdigest()
report['probe_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['scene_sha256']=hashlib.sha256(a.scene.read_bytes()).hexdigest()
app=None
try:
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'sync_loads':True})
    import numpy as np
    from pxr import Gf,UsdGeom,UsdPhysics
    from factory_ops.isaac45_runtime import create_runtime
    rt=create_runtime(app)
    ok,stage=rt.stage_utils.open_stage(str(a.scene))
    if not ok:raise RuntimeError('factory stage did not open')
    def bound():
        if time.monotonic()-started>210:raise TimeoutError('bounded parking comparison exceeded210wall seconds')
    while rt.stage_utils.is_stage_loading():
        bound();app.update()
    asset='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd'
    robots={}
    for name,x,enabled in [('baseline',12.,False),('hold',17.,True)]:
        path='/World/ParkingHold063/'+name
        rt.stage_utils.add_reference_to_stage(usd_path=asset,path=path)
        while rt.stage_utils.is_stage_loading():bound();app.update()
        root=stage.GetPrimAtPath(path)
        xf=UsdGeom.Xformable(root)
        xf.AddTranslateOp(opSuffix='probeSpawn').Set(Gf.Vec3d(x,1.5,.1))
        xf.AddRotateZOp(opSuffix='probeYaw').Set(90.)
        robots[name]=rt.WheeledRobot(path,wheel_dof_names=['joint_wheel_left','joint_wheel_right'],wheel_parking_hold=enabled)
    scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:UsdPhysics.Scene.Define(stage,'/World/Local45PhysicsScene')
    elif len(scenes)!=1:raise RuntimeError('expected one PhysicsScene')
    world=rt.SimulationManager.setup_simulation(1/60,device='cpu')
    import omni.timeline
    omni.timeline.get_timeline_interface().play()
    zero=np.zeros(2,dtype=np.float32)
    def snap(robot):
        pos,quat=robot.get_world_poses()
        native=robot._ready()
        ctl=native.get_articulation_controller()
        kp,kd=ctl.get_gains()
        xyz=[float(v) for v in pos[0]]
        orientation=[float(v) for v in quat[0]]
        encoders=[float(v) for v in native.get_wheel_positions()]
        if not all(math.isfinite(v) for v in xyz+orientation+encoders):
            raise RuntimeError('nonfinite physical state in parking comparison')
        limits=[float(v) for v in ctl.get_max_efforts()]
        return {'sim_time_s':float(world.current_time),'physics_frame':int(world.current_time_step_index),
            'gt_xyz_m':xyz,'gt_wxyz':orientation,'encoder_rad':encoders,
            'all_kp':[float(v) for v in kp],'all_kd':[float(v) for v in kd],
            'max_efforts':[v if math.isfinite(v) else str(v) for v in limits],
            'effort_modes':list(ctl.get_effort_modes()),'brake':robot.parking_brake_report()}
    def phase(name,seconds,targets):
        before={k:snap(v) for k,v in robots.items()}
        maximum={k:0. for k in robots}
        samples=[]
        for i in range(round(seconds*60)):
            bound()
            for robot in robots.values():robot.apply_wheel_actions(np.asarray(targets,dtype=np.float32))
            world.step(render=False)
            if i%60==59:
                row={k:snap(v) for k,v in robots.items()}
                for key in row:
                    maximum[key]=max(maximum[key],math.dist(row[key]['gt_xyz_m'][:2],before[key]['gt_xyz_m'][:2]))
                samples.append({key:{field:value[field] for field in ('sim_time_s','physics_frame','gt_xyz_m','encoder_rad')} for key,value in row.items()})
        after={k:snap(v) for k,v in robots.items()}
        item={'phase':name,'seconds':seconds,'wheel_targets_rad_s':[float(v) for v in targets],
            'before':before,'after':after,'maximum_sampled_xy_displacement_m':maximum,
            'physics_steps':after['hold']['physics_frame']-before['hold']['physics_frame'],
            'measured_elapsed_sim_s':after['hold']['sim_time_s']-before['hold']['sim_time_s']}
        assert item['physics_steps']==round(seconds*60)
        assert math.isfinite(item['measured_elapsed_sim_s']) and item['measured_elapsed_sim_s']>0
        report['phases'].append(item)
        (a.output/(name+'.json')).write_text(json.dumps(samples,indent=2,allow_nan=False)+'\n')
        save()
        return item
    # The baseline is the original facade behavior; no joint/pose reset occurs between phases.
    report['status']='running'
    phase('settle',2,zero)
    still=phase('stationary320',320,zero)
    # Exact authored geometry from the original run config is passed by the parent beside the scene.
    cfg=json.loads((a.scene.parent.parent/'config.json').read_text(encoding='utf-8'))
    radius=float(cfg['drive_geometry']['wheel_radius_m']);track=float(cfg['drive_geometry']['wheel_separation_m'])
    straight=phase('drive6',6,[.2/radius,.2/radius])
    held=phase('rehold30',30,zero)
    turn=phase('turn4',4,[-.4*track/(2*radius),.4*track/(2*radius)])
    held2=phase('turn_rehold30',30,zero)
    moved=math.dist(straight['before']['hold']['gt_xyz_m'][:2],straight['after']['hold']['gt_xyz_m'][:2])
    def yaw(q):
        w,x,y,z=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    turned=abs(math.atan2(math.sin(yaw(turn['after']['hold']['gt_wxyz'])-yaw(turn['before']['hold']['gt_wxyz'])),math.cos(yaw(turn['after']['hold']['gt_wxyz'])-yaw(turn['before']['hold']['gt_wxyz']))))
    checks={'stationary320_hold_under_2cm':still['maximum_sampled_xy_displacement_m']['hold']<.02,
        'resume_drive_over_50cm':moved>.5,'resume_turn_over_half_radian':turned>.5,
        'rehold30_under_2cm':held['maximum_sampled_xy_displacement_m']['hold']<.02,
        'turn_rehold30_under_2cm':held2['maximum_sampled_xy_displacement_m']['hold']<.02}
    report.update(checks=checks,baseline_start_gate_exceeded=still['maximum_sampled_xy_displacement_m']['baseline']>.15,
        baseline_scope='small floor/2-body comparison, not the original4-robot full scene',
        status='passed_functional_hold_and_release' if all(checks.values()) else 'failed_functional_hold_or_release')
except BaseException as e:
    report['status']='failed'
    report['primary_error']={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()}
finally:
    report['wall_s_before_close']=time.monotonic()-started
    if app is not None:report['sdk_close_requested']=True
    try:
        save()
        print(json.dumps({'status':report['status'],'checks':report.get('checks'),'error':report['primary_error']}),flush=True)
    finally:
        if app is not None:
            try:app.close()
            except BaseException as e:report['cleanup_errors'].append(str(e))
            else:report['sdk_close_returned']=True
            save()
