"""CPU planning/measurement checks; synthetic fixtures never count as SDK proof."""
from __future__ import annotations

import ast
from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import asdict
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from factory_ops.drive_profile import DriveLimits
from factory_ops.motion import DriveGeometry, WheelOdometry
from scripts.isaac import local_asset_probe as probe
from scripts.isaac.local_asset_probe import (
    MotionPlan, drive_geometry_from_records, evaluate_motion, planned_phase, validate_config,
)

ROOT = Path(__file__).resolve().parents[1]


def configuration() -> dict:
    return json.loads((ROOT / "configs/local-motion-probe.json").read_text(encoding="utf-8"))


def wheel_records() -> list[dict]:
    # Synthetic three-slice wheels: radius 0.1m, width 0.04m, center separation 0.4m.
    return [{"path": f"/Robot/wheel_{side}/slice_{i}",
             "min": [-0.1, center-0.02 + i*0.04/3., 0.],
             "max": [0.1, center-0.02 + (i+1)*0.04/3., 0.2]}
            for side, center in (("left", 0.2), ("right", -0.2)) for i in range(3)]


def synthetic_observations(*, frozen_body: bool = False) -> tuple[list[dict], dict]:
    """Perfect synthetic response, or frozen body despite valid target/encoder ramps."""
    cfg = configuration()
    geometry = DriveGeometry(0.1, 0.4)
    plan, encoder = MotionPlan(cfg["motion_profile"], geometry, 0.), WheelOdometry(geometry)
    initial = {"position_m": [0., 0., 0.05], "yaw_rad": 0., "encoder": asdict(encoder.update(0., 0., 0.))}
    rows = []
    x = y = yaw = left = right = 0.
    dt = 1. / 60.
    for index in range(1020):
        command = plan.step(index*dt, dt)
        target = command["target"]
        v, w = (0., 0.) if frozen_body else (target["linear_m_s"], target["angular_rad_s"])
        x += v*math.cos(yaw)*dt
        y += v*math.sin(yaw)*dt
        yaw += w*dt
        wheels = target["wheel_targets_rad_s"]
        left += wheels[0]*dt
        right += wheels[1]*dt
        stamp = (index+1)*dt
        rows.append({"sim_time_s": stamp, "position_m": [x, y, 0.05], "yaw_rad": yaw,
            "roll_rad": 0., "pitch_rad": 0., "linear_speed_m_s": abs(v), "angular_speed_rad_s": abs(w),
            "forward_velocity_m_s": v, "angular_velocity_rad_s": [0., 0., w],
            "motion": command, "encoder": asdict(encoder.update(stamp,left,right)),
            "measured_wheel_velocity_rad_s": list(wheels)})
    return rows, initial


class LocalMotionPlanTests(unittest.TestCase):
    def test_project_hashes_are_saved_before_sdk_startup_failure(self) -> None:
        saved = []
        args = types.SimpleNamespace(config=ROOT / "configs/local-motion-probe.json",
                                     output=ROOT / "unused-test-output", run_id="a"*32)
        sdk = types.ModuleType("isaacsim")
        def reject_startup(settings):
            raise RuntimeError("injected startup failure; no native SDK launched")
        sdk.SimulationApp = reject_startup
        original_read = Path.read_text
        def read_text(path, *args, **kwargs):
            return "4.5.0-fixture" if path.name == "VERSION" else original_read(path,*args,**kwargs)
        with patch.object(probe.argparse.ArgumentParser,"parse_args",return_value=args), \
                patch.object(Path,"exists",return_value=False), patch.object(Path,"mkdir"), \
                patch.object(Path,"read_text",read_text), \
                patch.object(probe,"save",side_effect=lambda path,value:saved.append(deepcopy(value))), \
                patch.dict(probe.os.environ,{"ISAAC_PATH":"E:/unused-sdk-fixture"}), \
                patch.dict(sys.modules,{"isaacsim":sdk}), redirect_stdout(io.StringIO()):
            self.assertEqual(probe.main(),1)
        self.assertEqual(saved[-1]["status"],"failed")
        self.assertEqual(saved[-1]["failed_phase"],"native_sdk_startup")
        startup = next(row for row in saved if row["stage"]=="native_sdk_startup")
        for name in ("factory_ops/__init__.py","factory_ops/motion.py","factory_ops/drive_profile.py"):
            self.assertEqual(startup["project_source_sha256"][name],hashlib.sha256((ROOT/name).read_bytes()).hexdigest())
        self.assertEqual(startup["config_sha256"],hashlib.sha256(args.config.read_bytes()).hexdigest())
        self.assertFalse(saved[-1]["sdk_close_attempted"])

    def test_idle_configuration_and_python310_syntax_remain_supported(self) -> None:
        validate_config(json.loads((ROOT / "configs/local-asset-probe.json").read_text()))
        for name in ("scripts/isaac/local_asset_probe.py", "factory_ops/motion.py", "factory_ops/drive_profile.py"):
            ast.parse((ROOT / name).read_text(encoding="utf-8"), feature_version=(3,10))

    def test_schedule_is_bounded_with_explicit_watchdog_phase(self) -> None:
        cfg = configuration()
        validate_config(cfg)
        self.assertEqual(sum(p["duration_s"] for p in cfg["motion_profile"]["phases"]),17.)
        for time_s, name in ((0.,"initial_stop"),(1.,"forward"),(4.,"normal_stop"),
                             (6.,"turn"),(9.,"turn_stop"),(11.,"curve"),(14.,"watchdog_stop")):
            self.assertEqual(planned_phase(cfg["motion_profile"],time_s)[0]["name"],name)
        for invalid in (-1.,17.,float("nan")):
            with self.assertRaises(ValueError): planned_phase(cfg["motion_profile"],invalid)

    def test_unsafe_schedule_and_limits_are_rejected(self) -> None:
        cfg = configuration()
        variants = []
        bad = deepcopy(cfg); bad["motion_profile"]["phases"][-1]["refresh_commands"] = True; variants.append(bad)
        bad = deepcopy(cfg); bad["motion_profile"]["phases"][1]["duration_s"] = 10.; variants.append(bad)
        bad = deepcopy(cfg); bad["motion_profile"]["phases"][1]["duration_s"] = 1.001; variants.append(bad)
        bad = deepcopy(cfg); bad["motion_profile"]["drive_limits"]["wheel_accel_rad_s2"] = float("nan"); variants.append(bad)
        bad = deepcopy(cfg); bad["motion_profile"]["evaluation"]["xy_bounds_m"] = [3.,-3.,-3.,3.]; variants.append(bad)
        for bad in variants:
            with self.subTest(config=bad), self.assertRaises(ValueError): validate_config(bad)

    def test_fresh_collider_geometry_and_invalid_structures(self) -> None:
        measured = drive_geometry_from_records(wheel_records(),"/Robot")
        self.assertAlmostEqual(measured["wheel_radius_m"],0.1)
        self.assertAlmostEqual(measured["wheel_separation_m"],0.4)
        for mutation in ("count","nonfinite","asymmetric","wrong_axis","side_swap"):
            rows = wheel_records()
            if mutation == "count": rows.pop()
            elif mutation == "nonfinite": rows[0]["min"][0] = float("nan")
            elif mutation == "asymmetric": rows[0]["min"][0] = -0.11
            elif mutation == "wrong_axis": rows[0]["max"][1] = 1.
            else:
                for row in rows:
                    row["min"][1],row["max"][1] = -row["max"][1],-row["min"][1]
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                drive_geometry_from_records(rows,"/Robot")

    def test_normal_body_and_wheel_target_acceleration_and_timeout(self) -> None:
        cfg = configuration()
        plan = MotionPlan(cfg["motion_profile"],DriveGeometry(0.1,0.4),0.)
        limits = DriveLimits(**cfg["motion_profile"]["drive_limits"])
        previous = (0.,0.,(0.,0.))
        expired = []
        for index in range(1020):
            result = plan.step(index/60.,1./60.)
            target = result["target"]
            v,w,wheels = target["linear_m_s"],target["angular_rad_s"],target["wheel_targets_rad_s"]
            if not target["emergency_stop"]:
                self.assertLessEqual(abs(v-previous[0]),max(limits.linear_accel_m_s2,limits.linear_decel_m_s2)/60.+1e-10)
                self.assertLessEqual(abs(w-previous[1]),max(limits.angular_accel_rad_s2,limits.angular_decel_rad_s2)/60.+1e-10)
                self.assertTrue(all(abs(a-b)<=limits.wheel_accel_rad_s2/60.+1e-10 for a,b in zip(wheels,previous[2])))
            if result["phase"] == "watchdog_stop": self.assertFalse(result["command_refreshed"])
            if result["watchdog_expired"]:
                expired.append(result)
                self.assertEqual(tuple(wheels),(0.,0.))
            previous = v,w,wheels
        age = expired[0]["issued_at_sim_time_s"]-expired[0]["command_stamp_s"]
        self.assertGreaterEqual(age,0.3-1e-9)
        self.assertLessEqual(age,0.3+1./60.+1e-9)

    def test_evaluation_distinguishes_target_ramps_from_physics(self) -> None:
        rows,initial = synthetic_observations()
        # JSON roundtrip matches native JSONL tuple/list representation.
        result = evaluate_motion(json.loads(json.dumps(rows)),initial,configuration())
        self.assertTrue(result["passed"],result)
        frozen,initial = synthetic_observations(frozen_body=True)
        result = evaluate_motion(frozen,initial,configuration())
        self.assertFalse(result["passed"])
        self.assertTrue(all(not p["passed"] for p in result["phases"] if p["phase"] in ("forward","turn","curve")))

    def test_planar_escape_and_incomplete_phase_are_not_accepted(self) -> None:
        rows,initial = synthetic_observations()
        rows[70]["position_m"][0] = 3.01
        self.assertFalse(evaluate_motion(rows,initial,configuration())["planar_bounds_passed"])
        rows,initial = synthetic_observations()
        result = evaluate_motion(rows[:-1],initial,configuration())
        self.assertFalse(result["passed"])
        self.assertFalse(result["phases"][-1]["checks"]["complete_phase"])


if __name__ == "__main__":
    unittest.main()
