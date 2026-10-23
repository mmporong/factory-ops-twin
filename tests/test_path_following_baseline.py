from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from factory_ops.motion import DriveGeometry, EncoderSample
from factory_ops.path_following_baseline import (
    EncoderPathController,
    build_reference_path,
    evaluate_ground_truth,
    validate_config,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "local45-path-following-baseline.json"


class PathFollowingBaselineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = validate_config(json.loads(CONFIG.read_text(encoding="utf-8")))
        raw = self.config["drive_geometry"]
        self.geometry = DriveGeometry(float(raw["wheel_radius_m"]), float(raw["wheel_separation_m"]))

    def test_reference_path_is_tangent_straight_then_quarter_arc(self) -> None:
        points = build_reference_path(self.config["path"])
        self.assertEqual((points[0].x_m, points[0].y_m, points[0].yaw_rad), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(points[-1].x_m, 4.0)
        self.assertAlmostEqual(points[-1].y_m, 2.0)
        self.assertAlmostEqual(points[-1].yaw_rad, math.pi / 2.0)
        self.assertAlmostEqual(points[-1].s_m, 2.0 + math.pi)
        self.assertTrue(all(a.s_m < b.s_m for a, b in zip(points, points[1:])))

    def test_encoder_only_controller_converges_and_obeys_g2_envelope(self) -> None:
        controller = EncoderPathController(self.config, self.geometry)
        dt = float(self.config["physics_dt_s"])
        x = y = yaw = linear = angular = 0.0
        phases: set[str] = set()
        maximum_linear = maximum_angular = maximum_wheel = 0.0
        for index in range(1, 6001):
            stamp = index * dt
            pose = EncoderSample(stamp, x, y, yaw, linear, angular)
            output = controller.step(stamp, pose, dt)
            phases.add(output["phase"])
            command = output["command"]
            linear, angular = float(command["linear_m_s"]), float(command["angular_rad_s"])
            maximum_linear = max(maximum_linear, abs(linear))
            maximum_angular = max(maximum_angular, abs(angular))
            maximum_wheel = max(maximum_wheel, *(abs(value) for value in command["wheel_targets_rad_s"]))
            yaw_mid = yaw + angular * dt / 2.0
            x += linear * math.cos(yaw_mid) * dt
            y += linear * math.sin(yaw_mid) * dt
            yaw += angular * dt
            if controller.complete:
                break
        self.assertTrue(controller.complete, output)
        self.assertIn("path_tracking", phases)
        self.assertIn("terminal_brake", phases)
        self.assertIn("stationary_dwell", phases)
        goal = build_reference_path(self.config["path"])[-1]
        self.assertLessEqual(math.hypot(x-goal.x_m, y-goal.y_m), self.config["controller"]["position_tolerance_m"])
        self.assertLessEqual(abs(math.atan2(math.sin(yaw-goal.yaw_rad), math.cos(yaw-goal.yaw_rad))),
                             self.config["controller"]["yaw_tolerance_rad"])
        limits = self.config["drive_limits"]
        self.assertLessEqual(maximum_linear, limits["max_linear_m_s"])
        self.assertLessEqual(maximum_angular, limits["max_angular_rad_s"])
        self.assertLessEqual(maximum_wheel, limits["max_wheel_rad_s"])

    def test_ground_truth_is_an_independent_evaluation_input(self) -> None:
        controller = EncoderPathController(self.config, self.geometry)
        pose = EncoderSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        output = controller.step(0.0, pose, float(self.config["physics_dt_s"]))
        self.assertEqual(output["control_input"], "wheel_encoder_odometry_only")
        self.assertNotIn("ground_truth", output)
        path = build_reference_path(self.config["path"])
        samples = [{
            "controller": {"phase": "path_tracking" if index < len(path)-1 else "complete"},
            "ground_truth": {"x_m": point.x_m, "y_m": point.y_m, "yaw_rad": point.yaw_rad,
                             "roll_rad": 0.0, "pitch_rad": 0.0,
                             "forward_speed_m_s": 0.5, "yaw_rate_rad_s": 0.5},
        } for index, point in enumerate(path)]
        result = evaluate_ground_truth(samples, self.config)
        self.assertTrue(result["passed"], result)
        changed = deepcopy(samples)
        changed[len(changed)//2]["ground_truth"]["y_m"] += 1.0
        failed = evaluate_ground_truth(changed, self.config)
        self.assertFalse(failed["passed"])
        self.assertFalse(failed["checks"]["maximum_cross_track"])

    def test_config_rejects_resource_and_speed_expansion(self) -> None:
        mutations = []
        for field, value in (("maximum_sim_time_s", 121.0), ("maximum_wall_time_s", 601.0),
                             ("capture_every_physics_steps", 1)):
            changed = deepcopy(self.config)
            changed.pop("computed", None)
            changed[field] = value
            mutations.append(changed)
        changed = deepcopy(self.config); changed.pop("computed", None)
        changed["drive_limits"]["max_linear_m_s"] = 0.61; mutations.append(changed)
        for changed in mutations:
            with self.assertRaises(ValueError):
                validate_config(changed)


if __name__ == "__main__":
    unittest.main()
