"""CPU command/evaluation fixtures only, no native motion or RGB claim."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from factory_ops.carrier_mobility import (MotionEvidence, MotionPlan, RgbCadence,
    native_drive_geometry, validate_motion)
from factory_ops.carrier_module import build_manifest
from factory_ops.carrier_robot import ROBOT_PATH
from factory_ops.drive_profile import DriveProfile
from factory_ops.motion import DriveGeometry


ROOT = Path(__file__).resolve().parents[1]


class CarrierMobilityTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT/'configs/vehicles/tote-carrier-loaded-motion-v1.json').read_text())
        self.static = self.profile['probe']
        self.cfg = validate_motion(self.profile['loaded_motion'], self.static)

    def test_static_copy_unchanged_and_motion_bounds_reject(self):
        original = json.loads((ROOT/'configs/vehicles/tote-carrier-carter-v2.json').read_text())
        self.assertEqual({k:v for k,v in self.profile.items() if k != 'loaded_motion'}, original)
        self.assertEqual(build_manifest(self.profile), build_manifest(original))
        self.assertEqual(self.static['maximum_sim_time_s'], 6.)
        self.assertEqual(self.cfg['schedule_duration_s'], 9.)
        self.assertEqual(self.cfg['rgb_period_physics_frames'], 6)
        for key, value in [('maximum_sim_time_s', 21.), ('maximum_wall_time_s', 361.),
                ('maximum_tilt_rad', .051), ('rgb_fps', 30), ('maximum_rgb_frames', 241),
                ('maximum_torque_n_m', math.nan), ('schema_version', True)]:
            with self.subTest(key=key):
                bad = deepcopy(self.profile['loaded_motion']); bad[key] = value
                with self.assertRaises(ValueError): validate_motion(bad, self.static)
        for mutation in ('order', 'negative', 'speed', 'duration'):
            bad = deepcopy(self.profile['loaded_motion'])
            if mutation == 'order': bad['phases'].reverse()
            if mutation == 'negative': bad['phases'][0]['linear_m_s'] = -.1
            if mutation == 'speed': bad['phases'][0]['linear_m_s'] = 1.
            if mutation == 'duration': bad['phases'][0]['duration_s'] = 10.
            with self.assertRaises(ValueError): validate_motion(bad, self.static)

    def test_fresh_wheel_geometry_readback_and_identity(self):
        measured = {'robot_path':ROBOT_PATH,'meters_per_unit':1.,'up_axis':'Z',
            'robot_world_from_local':[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]],'colliders':[]}
        for side, sign in [('left',1),('right',-1)]:
            for i, diameter in enumerate([.25,.2,.28]):
                measured['colliders'].append({'path':ROBOT_PATH+'/wheel_'+side+'/Colliders/Cylinder_'+str(i),
                    'world_aabb_m':{'center_m':[0.,sign*.2066,.14],'size_m':[diameter,.02,diameter]}})
        geometry = native_drive_geometry(measured)
        self.assertAlmostEqual(geometry['wheel_radius_m'], .14)
        self.assertAlmostEqual(geometry['wheel_separation_m'], .4132)
        measured['colliders'][1] = deepcopy(measured['colliders'][0])
        with self.assertRaises(ValueError): native_drive_geometry(measured)

    def test_only_existing_driveprofile_shapes_time_commands(self):
        plan = MotionPlan(self.cfg, DriveGeometry(.14,.4132), 10., [0.,0.])
        self.assertIsInstance(plan.drive, DriveProfile)
        prior = [0.,0.]
        positions = [0.,0.]
        seen = set()
        opposite = False
        for i in range(1152):
            now = 10.+i/120
            cmd = plan.command(now, 1/120)
            seen.add(cmd['phase'])
            targets = cmd['target']['wheel_targets_rad_s']
            self.assertTrue(all(abs(b-a) <= 1/120+1e-10 for a,b in zip(prior,targets)))
            if cmd['phase'] == 'in_place' and targets[0] < 0 < targets[1]: opposite = True
            positions = [p+w/120 for p,w in zip(positions,targets)]
            row = plan.observe_wheels(now+1/120, positions, list(targets), [0.,0.], [100.,100.])
            self.assertGreaterEqual(row['encoder']['stamp_s'], now)
            prior = targets
        self.assertEqual(seen, {'forward','curve','brake_before_turn','in_place','final_brake','final_settle'})
        self.assertTrue(opposite)
        self.assertEqual(prior, (0.,0.))
        for values in ([math.nan,0.], [81.,0.]):
            with self.assertRaises(ValueError):
                plan.observe_wheels(21., positions, [0.,0.], values, [100.,100.])

    def test_evidence_expected_direction_final_dwell_and_load_loss(self):
        evidence = MotionEvidence(self.cfg, self.static)
        def add(phase,t,x,yaw,stationary=False,wheels=(0.,0.),load=True):
            return evidence.observe(phase,t,position_m=[x,0.,0.],yaw_rad=yaw,tilt_rad=0.,
                encoder={'x_m':x,'yaw_rad':yaw},wheel_velocity=list(wheels),load_supported=load,
                stationary=stationary,target_zero=stationary)
        add('forward',1.,0.,0.); add('forward',3.,.2,0.)
        add('curve',3.1,.2,0.); add('curve',5.,.4,.3)
        add('brake_before_turn',6.,.4,.3,True)
        add('in_place',6.1,.4,.3,wheels=(-.2,.2)); add('in_place',8.,.4,.6,wheels=(-.2,.2))
        add('final_brake',9.,.4,.6,True)
        self.assertFalse(add('final_settle',9.1,.4,.6,True))
        self.assertTrue(add('final_settle',9.6,.4,.6,True))
        self.assertTrue(evidence.result()['passed'])
        with self.assertRaisesRegex(ValueError,'support lost'):
            add('final_settle',9.7,.4,.6,True,load=False)
        wrong = MotionEvidence(self.cfg,self.static)
        wrong.observe('forward',1.,position_m=[0.,0.,0.],yaw_rad=0.,tilt_rad=0.,
            encoder={'x_m':0.,'yaw_rad':0.},wheel_velocity=[0.,0.],load_supported=True,stationary=True,target_zero=True)
        self.assertFalse(wrong.result()['passed'])

    def test_brake_stationarity_and_tilt_are_not_relaxed(self):
        evidence = MotionEvidence(self.cfg, self.static)
        kwargs = dict(position_m=[0.,0.,0.],yaw_rad=0.,tilt_rad=0.,encoder={'x_m':0.,'yaw_rad':0.},
            wheel_velocity=[0.,0.],load_supported=True,stationary=False,target_zero=True)
        evidence.observe('brake_before_turn',1.,**kwargs)
        with self.assertRaisesRegex(ValueError,'not stationary'):
            evidence.observe('in_place',2.,**kwargs)
        kwargs['tilt_rad'] = .051
        with self.assertRaises(ValueError):
            MotionEvidence(self.cfg,self.static).observe('forward',1.,**kwargs)

    def test_actual_frame_rgb20fps_and_hard_count_limit(self):
        cadence = RgbCadence(72,6)
        frames = [frame for frame in range(73,1273) if cadence.due(frame)]
        self.assertEqual(frames, list(range(78,1273,6)))
        self.assertEqual(cadence.count,200)
        rows = [{'world_checkpoint_frame': f,'sha256': ('a' if i < 100 else 'b')*64}
                for i,f in enumerate(frames)]
        self.assertEqual(cadence.validate_records(rows)['distinct_image_sha256'],2)
        with self.assertRaisesRegex(ValueError,'missing capture'):
            cadence.validate_records(rows[:-1])
        with self.assertRaisesRegex(ValueError,'stayed identical'):
            cadence.validate_records([dict(r,sha256='a'*64) for r in rows])
        with self.assertRaises(ValueError): cadence.due(1272)
        maximum = RgbCadence(0,6)
        with self.assertRaisesRegex(ValueError,'maximum real RGB'):
            for frame in range(1,1447): maximum.due(frame)
        self.assertEqual(maximum.count,240)

    def test_native60fps_profile_and_frame_budget_preserve_20fps_limits(self):
        from copy import deepcopy
        sixty = deepcopy(self.cfg)
        sixty.update(rgb_fps=60, maximum_rgb_frames=660)
        validated = validate_motion(sixty, self.static)
        self.assertEqual(validated['rgb_period_physics_frames'],2)
        self.assertEqual(validate_motion(self.cfg,self.static)['maximum_rgb_frames'],240)
        with self.assertRaisesRegex(ValueError,'bounded real'):
            validate_motion(dict(sixty,maximum_rgb_frames=240),self.static)
        cadence=RgbCadence(0,2,660)
        self.assertEqual(sum(cadence.due(i) for i in range(1,1201)),600)
        with self.assertRaisesRegex(ValueError,'maximum real RGB'):
            for i in range(1201,1323):cadence.due(i)


if __name__ == '__main__':
    unittest.main()
