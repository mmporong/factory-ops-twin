from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from factory_ops.docking_baseline import EncoderDockingController, evaluate_docking_truth, validate_config, wrap_angle
from factory_ops.motion import DriveGeometry, EncoderSample


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/local45-docking-alignment.json"


class DockingBaselineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = validate_config(json.loads(CONFIG.read_text(encoding="utf-8")))
        self.geometry = DriveGeometry(**self.config["drive_geometry"])

    def test_misaligned_encoder_pose_converges_with_drive_limits(self) -> None:
        controller = EncoderDockingController(self.config, self.geometry)
        dt = float(self.config["physics_dt_s"])
        x, y, yaw = -0.05, -0.03, -0.06
        linear = angular = 0.0
        previous_linear = previous_angular = 0.0
        phases: set[str] = set()
        for index in range(1, 2701):
            stamp = index*dt
            output = controller.step(stamp, EncoderSample(stamp, x, y, yaw, linear, angular), dt)
            phases.add(output["phase"])
            command = output["command"]
            linear, angular = float(command["linear_m_s"]), float(command["angular_rad_s"])
            limits = self.config["drive_limits"]
            self.assertLessEqual(abs(linear), limits["max_linear_m_s"]+1e-12)
            self.assertLessEqual(abs(angular), limits["max_angular_rad_s"]+1e-12)
            linear_rate = (limits["linear_decel_m_s2"] if abs(linear) < abs(previous_linear)
                           else limits["linear_accel_m_s2"])
            angular_rate = (limits["angular_decel_rad_s2"] if abs(angular) < abs(previous_angular)
                            else limits["angular_accel_rad_s2"])
            self.assertLessEqual(abs(linear-previous_linear), linear_rate*dt+1e-12)
            self.assertLessEqual(abs(angular-previous_angular), angular_rate*dt+1e-12)
            self.assertLessEqual(max(abs(value) for value in command["wheel_targets_rad_s"]),
                                 limits["max_wheel_rad_s"]+1e-12)
            yaw_mid = yaw+angular*dt/2.0
            x += linear*math.cos(yaw_mid)*dt
            y += linear*math.sin(yaw_mid)*dt
            yaw = wrap_angle(yaw+angular*dt)
            previous_linear, previous_angular = linear, angular
            if controller.complete:
                break
        self.assertTrue(controller.complete, output)
        self.assertIn("approach", phases)
        self.assertIn("yaw_alignment", phases)
        self.assertIn("stationary_dwell", phases)
        target = self.config["target_pose_odom"]
        self.assertLessEqual(math.hypot(x-target["x_m"], y-target["y_m"]),
                             self.config["controller"]["position_tolerance_m"])
        self.assertLessEqual(abs(wrap_angle(yaw-target["yaw_rad"])),
                             self.config["controller"]["yaw_tolerance_rad"])

    def test_observed_motion_or_off_pose_resets_stationary_dwell(self) -> None:
        controller = EncoderDockingController(self.config, self.geometry)
        dt = float(self.config["physics_dt_s"])
        target = self.config["target_pose_odom"]
        stamp = 0.0
        for _ in range(40):
            stamp += dt
            output = controller.step(stamp, EncoderSample(stamp, target["x_m"], target["y_m"],
                target["yaw_rad"], 0.0, 0.0), dt)
        self.assertIsNotNone(output["stationary_since_s"])
        stamp += dt
        moved = controller.step(stamp, EncoderSample(stamp, target["x_m"]+0.03, target["y_m"],
            target["yaw_rad"], 0.02, 0.0), dt)
        self.assertIsNone(moved["stationary_since_s"])
        self.assertFalse(controller.complete)
        for _ in range(90):
            stamp += dt
            output = controller.step(stamp, EncoderSample(stamp, target["x_m"], target["y_m"],
                target["yaw_rad"], 0.0, 0.0), dt)
            if controller.complete:
                break
        self.assertTrue(controller.complete)

    def test_ground_truth_failure_is_independent_of_encoder_acceptance(self) -> None:
        target = self.config["target_pose_odom"]
        samples = []
        for index in range(3):
            samples.append({
                "controller": {"phase": "complete" if index == 1 else "stationary_dwell"},
                "ground_truth": {"x_m": target["x_m"]+(0.09 if index >= 1 else 0.0),
                    "y_m": target["y_m"], "yaw_rad": target["yaw_rad"]+0.1,
                    "roll_rad": 0.0, "pitch_rad": 0.0,
                    "forward_speed_m_s": 0.0, "yaw_rate_rad_s": 0.0},
            })
        result = evaluate_docking_truth(samples, self.config)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["endpoint"])
        self.assertFalse(result["checks"]["yaw"])
        self.assertEqual(result["truth_usage"],
                         "independent post-run evaluation only; never controller or dwell input")
        self.assertFalse(result["physical_contact_verified"])
        self.assertFalse(result["electrical_contact_verified"])

    def test_config_rejects_unbounded_or_false_contact_claims(self) -> None:
        for mutation in (
            lambda value: value["drive_limits"].update(max_linear_m_s=0.151),
            lambda value: value["controller"].update(stationary_dwell_s=0.9),
            lambda value: value["evaluation"].update(maximum_endpoint_error_m=0.061),
            lambda value: value.update(maximum_sim_time_s=45.1),
            lambda value: value.update(unloaded=False),
        ):
            raw = deepcopy(self.config)
            mutation(raw)
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_config(raw)


if __name__ == "__main__":
    unittest.main()
