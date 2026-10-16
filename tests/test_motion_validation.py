from __future__ import annotations

from copy import deepcopy
import unittest

from factory_ops.drive_profile import DriveLimits
from factory_ops.motion import DriveGeometry
from factory_ops.motion_validation import validate_drive_trace


class MotionValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.geometry = DriveGeometry(0.14, 0.4132)
        self.limits = DriveLimits()
        self.dt = 1 / 60

    def sample(self, index: int, v: float = 0.0, w: float = 0.0) -> dict:
        return {"physics_frame": index, "sim_time_s": index * self.dt,
                "command": {"linear_m_s": v, "angular_rad_s": w},
                "wheel_target_rad_s": list(self.geometry.wheel_targets(v, w)),
                "motion_profile": {"phase": "translate", "emergency_stop": False,
                                   "curvature_limited": False, "wheel_speed_limited": False,
                                   "input_command": {"linear_m_s": v, "angular_rad_s": w},
                                   "stop_reason": "none"},
                "odometry": {"linear_m_s": v, "angular_rad_s": w},
                "ground_truth": {"position_xyz_m": [v * index * self.dt, 0.0, 0.14],
                                 "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}}

    def test_authored_ramp_and_separate_fault_stop_are_valid(self) -> None:
        samples = [self.sample(i, 0.25 * i * self.dt) for i in range(60)]
        fault = self.sample(60)
        fault["motion_profile"].update(phase="fault_stop", emergency_stop=True, stop_reason="watchdog_stale")
        result = validate_drive_trace([*samples, fault], self.geometry, self.limits, self.dt)
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(result["fault_stop_samples"], 1)
        self.assertAlmostEqual(result["command_maxima"]["linear_accel_m_s2"], 0.25)

    def test_ordinary_stop_cannot_hide_a_velocity_jump(self) -> None:
        result = validate_drive_trace([self.sample(0, 0.4), self.sample(1)],
                                      self.geometry, self.limits, self.dt)
        self.assertFalse(result["passed"])
        self.assertTrue(any("linear_decel" in error for error in result["errors"]))

    def test_dropped_frames_bad_actuators_and_nonfinite_response_are_rejected(self) -> None:
        baseline = [self.sample(0), self.sample(1)]
        for field, value in (("physics_frame", 3), ("wheel_target_rad_s", [0.0, 1.0]),
                             ("odometry", {"linear_m_s": float("nan"), "angular_rad_s": 0.0}),
                             ("ground_truth", {})):
            samples = deepcopy(baseline)
            samples[1][field] = value
            self.assertFalse(validate_drive_trace(samples, self.geometry, self.limits, self.dt)["passed"])

    def test_absolute_speed_and_curvature_are_checked_even_in_first_sample(self) -> None:
        result = validate_drive_trace([self.sample(0, 0.6, 1.2)], self.geometry, self.limits, self.dt)
        self.assertFalse(result["passed"])
        self.assertTrue(any("lateral" in error for error in result["errors"]))

    def test_phase_and_stop_reason_cannot_disguise_an_emergency_as_normal_stop(self) -> None:
        baseline = [self.sample(0), self.sample(1)]
        for fields in ({"phase": "fabricated"}, {"phase": "fault_stop"},
                       {"phase": "fault_stop", "emergency_stop": True, "stop_reason": "normal_mission_complete"},
                       {"phase": "normal_stop", "input_command": {"linear_m_s": 0.1, "angular_rad_s": 0.0}},
                       {"curvature_limited": "true"}, {"stop_reason": "unknown"}):
            samples = deepcopy(baseline)
            samples[1]["motion_profile"].update(fields)
            self.assertFalse(validate_drive_trace(samples, self.geometry, self.limits, self.dt)["passed"])
        samples = deepcopy(baseline)
        samples[0]["sim_time_s"] = -1.0
        self.assertFalse(validate_drive_trace(samples, self.geometry, self.limits, self.dt)["passed"])


if __name__ == "__main__":
    unittest.main()
