from __future__ import annotations

import math
import unittest

from factory_ops.motion_transients import measure_transients


class MotionTransientTests(unittest.TestCase):
    @staticmethod
    def _config() -> dict:
        return {
            "transient_measurement": {
                "smoothing_window_s": 0.1,
                "stop_speed_m_s": 0.02,
                "stop_hold_s": 0.5,
                "reverse_confirm_s": 0.2,
            }
        }

    @staticmethod
    def _trapezoid_trace() -> list[dict]:
        dt = 0.1
        x = 0.0
        encoder_x = 0.0
        samples: list[dict] = []
        for step in range(1, 121):
            stamp = round(step * dt, 10)
            if stamp <= 4.0:
                segment = "straight_accel_cruise"
                speed = min(stamp / 3.0, 1.0)
                requested = 1.0
                applied = speed
            elif stamp <= 7.0:
                segment = "normal_brake"
                speed = max(0.0, 1.0 - (stamp - 4.0))
                requested = 0.0
                applied = speed
            elif stamp <= 9.0:
                segment = "reverse_setup"
                speed = 0.5
                requested = 0.5
                applied = 0.5
            else:
                segment = "reverse_through_zero"
                speed = max(-0.5, 0.5 - (stamp - 9.0))
                requested = -0.5
                applied = speed
            x += speed * dt
            encoder_x += speed * dt
            samples.append({
                "sim_time_s": stamp,
                "schedule_segment": segment,
                "requested_command": {"linear_m_s": requested, "angular_rad_s": 0.0},
                "command": {"linear_m_s": applied, "angular_rad_s": 0.0},
                "odometry": {"x_m": encoder_x, "y_m": 0.0, "linear_m_s": speed},
                "ground_truth": {
                    "position_xyz_m": [x, 0.0, 0.0],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
            })
        return samples

    def test_known_trapezoid_measures_rise_brake_distance_and_reverse_sign_change(self) -> None:
        result = measure_transients(self._trapezoid_trace(), self._config())
        self.assertTrue(result["measured"], result["errors"])
        rise = result["metrics"]["straight_rise"]
        self.assertAlmostEqual(rise["ground_truth"]["steady_speed_m_s"], 1.0, places=6)
        self.assertAlmostEqual(rise["ground_truth"]["rise_10_to_90_s"], 2.4, delta=0.12)
        self.assertAlmostEqual(rise["encoder"]["rise_10_to_90_s"], 2.4, delta=0.12)
        brake = result["metrics"]["normal_brake"]
        self.assertAlmostEqual(brake["ground_truth"]["braking_time_s"], 1.0, delta=0.12)
        self.assertAlmostEqual(brake["ground_truth"]["braking_path_distance_m"], 0.45, delta=0.12)
        self.assertAlmostEqual(brake["encoder"]["braking_path_distance_m"], 0.45, delta=0.12)
        reverse = result["metrics"]["reverse_through_zero"]
        self.assertAlmostEqual(reverse["ground_truth"]["transition_s"], 9.5, delta=0.12)
        self.assertLess(reverse["ground_truth"]["zero_bracket"]["nonpositive"]["speed_m_s"], 1e-9)
        self.assertTrue(reverse["command_zero"]["applied"]["observed"])
        self.assertFalse(reverse["command_zero"]["requested"]["observed"])
        self.assertEqual(result["units"]["path_distance"], "m")
        self.assertIn("not hardware-calibrated", result["criteria"]["status"])

    def test_constant_motion_is_descriptive_failure_not_a_false_measurement(self) -> None:
        samples = self._trapezoid_trace()
        for index, sample in enumerate(samples):
            sample["odometry"].update({"x_m": (index + 1) * 0.1, "linear_m_s": 1.0})
            sample["ground_truth"]["position_xyz_m"] = [(index + 1) * 0.1, 0.0, 0.0]
        result = measure_transients(samples, self._config())
        self.assertFalse(result["measured"])
        self.assertTrue(any("rise" in error for error in result["errors"]))
        self.assertTrue(any("normal brake" in error for error in result["errors"]))

    def test_stop_after_normal_brake_segment_is_not_attributed_to_braking(self) -> None:
        samples = self._trapezoid_trace()
        x = 0.0
        for sample in samples:
            speed = float(sample["odometry"]["linear_m_s"])
            if sample["schedule_segment"] == "normal_brake":
                speed = 1.0
            elif sample["schedule_segment"] in {"reverse_setup", "reverse_through_zero"}:
                speed = 0.0
            x += speed * 0.1
            sample["odometry"].update({"x_m": x, "linear_m_s": speed})
            sample["ground_truth"]["position_xyz_m"] = [x, 0.0, 0.0]
        result = measure_transients(samples, self._config())
        brake = result["metrics"]["normal_brake"]
        self.assertFalse(brake["ground_truth"]["observed"])
        self.assertFalse(brake["encoder"]["observed"])
        self.assertIn("hold duration", brake["ground_truth"]["reason"])

    def test_rise_in_a_later_segment_is_not_attributed_to_straight_acceleration(self) -> None:
        samples = self._trapezoid_trace()
        x = 0.0
        for sample in samples:
            if sample["schedule_segment"] == "straight_accel_cruise":
                speed = 1.0
            elif sample["schedule_segment"] == "normal_brake":
                speed = 0.0
            elif sample["schedule_segment"] == "reverse_setup":
                speed = min(1.0, max(0.0, (float(sample["sim_time_s"]) - 7.0) / 1.0))
            else:
                speed = -0.5
            x += speed * 0.1
            sample["odometry"].update({"x_m": x, "linear_m_s": speed})
            sample["ground_truth"]["position_xyz_m"] = [x, 0.0, 0.0]
        result = measure_transients(samples, self._config())
        rise = result["metrics"]["straight_rise"]
        self.assertFalse(rise["ground_truth"]["observed"])
        self.assertFalse(rise["encoder"]["observed"])
        self.assertIn("not both observed", rise["encoder"]["reason"])

    def test_negative_motion_after_reverse_segment_is_not_a_reverse_measurement(self) -> None:
        samples = self._trapezoid_trace()
        x = 0.0
        for sample in samples:
            speed = float(sample["odometry"]["linear_m_s"])
            if sample["schedule_segment"] == "reverse_through_zero":
                speed = 0.5
            x += speed * 0.1
            sample["odometry"].update({"x_m": x, "linear_m_s": speed})
            sample["ground_truth"]["position_xyz_m"] = [x, 0.0, 0.0]
        encoder_x = x
        for offset in range(1, 6):
            stamp = 12.0 + offset * 0.1
            x -= 0.05
            encoder_x -= 0.05
            samples.append({
                "sim_time_s": stamp,
                "schedule_segment": "after_reverse",
                "requested_command": {"linear_m_s": -0.5, "angular_rad_s": 0.0},
                "command": {"linear_m_s": -0.5, "angular_rad_s": 0.0},
                "odometry": {"x_m": encoder_x, "y_m": 0.0, "linear_m_s": -0.5},
                "ground_truth": {
                    "position_xyz_m": [x, 0.0, 0.0],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
            })
        result = measure_transients(samples, self._config())
        reverse = result["metrics"]["reverse_through_zero"]
        self.assertFalse(reverse["ground_truth"]["observed"])
        self.assertFalse(reverse["encoder"]["observed"])
        self.assertIn("confirmed negative", reverse["encoder"]["reason"])

    def test_midpoint_yaw_projection_separates_forward_motion_from_lateral_slip(self) -> None:
        samples = self._trapezoid_trace()
        quarter_turn = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
        for sample in samples:
            sample["ground_truth"]["orientation_wxyz"] = quarter_turn
        result = measure_transients(samples, self._config())
        self.assertFalse(result["measured"])
        self.assertIn("not positive", result["metrics"]["straight_rise"]["ground_truth"]["reason"])


if __name__ == "__main__":
    unittest.main()
