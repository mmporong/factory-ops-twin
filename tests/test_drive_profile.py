from __future__ import annotations

import math
import unittest

from factory_ops.drive_profile import DriveLimits, DriveProfile
from factory_ops.motion import DriveGeometry


class DriveProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.geometry = DriveGeometry(0.14, 0.4132)
        self.limits = DriveLimits()
        self.profile = DriveProfile(self.geometry, self.limits)
        self.dt = 1.0 / 60.0

    def test_normal_acceleration_and_braking_are_continuous(self) -> None:
        speeds = [self.profile.step(0.4, 0.0, self.dt).linear_m_s for _ in range(120)]
        self.assertAlmostEqual(speeds[-1], 0.4)
        self.assertAlmostEqual(speeds[0], self.limits.linear_accel_m_s2 * self.dt)
        before = speeds[-1]
        stopped = []
        for _ in range(90):
            command = self.profile.step(0.0, 0.0, self.dt)
            self.assertLessEqual(abs(command.linear_m_s - before), self.limits.linear_decel_m_s2 * self.dt + 1e-12)
            self.assertFalse(command.emergency_stop)
            stopped.append(command.linear_m_s)
            before = command.linear_m_s
        self.assertGreater(stopped[0], 0.0)
        self.assertEqual(stopped[-1], 0.0)

    def test_in_place_turn_brakes_translation_then_departure_settles_yaw(self) -> None:
        for _ in range(120):
            self.profile.step(0.4, 0.0, self.dt)
        commands = [self.profile.step(0.0, 0.5, self.dt) for _ in range(180)]
        for command in commands:
            if command.linear_m_s != 0.0:
                self.assertEqual(command.angular_rad_s, 0.0)
        self.assertEqual(commands[-1].linear_m_s, 0.0)
        self.assertAlmostEqual(commands[-1].angular_rad_s, 0.5)
        commands = [self.profile.step(0.4, 0.0, self.dt) for _ in range(180)]
        for command in commands:
            if command.angular_rad_s != 0.0:
                self.assertEqual(command.linear_m_s, 0.0)
        self.assertAlmostEqual(commands[-1].linear_m_s, 0.4)
        self.assertEqual(commands[-1].angular_rad_s, 0.0)

    def test_curve_ratio_transitions_make_progress_on_the_lateral_boundary(self) -> None:
        for target in ((0.6, 1.0 / 3.0), (1.0 / 3.0, 0.6), (0.6, 1.0 / 3.0)):
            for _ in range(600):
                before = (self.profile.linear_m_s, self.profile.angular_rad_s)
                before_wheels = self.geometry.wheel_targets(*before)
                command = self.profile.step(*target, self.dt)
                self.assertLessEqual(abs(command.linear_m_s * command.angular_rad_s), 0.2 + 1e-12)
                self.assertLessEqual(abs(command.linear_m_s - before[0]) / self.dt, 0.35 + 1e-12)
                self.assertLessEqual(abs(command.angular_rad_s - before[1]) / self.dt, 0.7 + 1e-12)
                self.assertLessEqual(max(abs(a - b) for a, b in zip(command.wheel_targets_rad_s, before_wheels)) / self.dt, 4.0 + 1e-12)
            self.assertAlmostEqual(self.profile.linear_m_s, target[0])
            self.assertAlmostEqual(self.profile.angular_rad_s, target[1])

    def test_wheel_speed_cap_preserves_requested_curvature(self) -> None:
        profile = DriveProfile(self.geometry, DriveLimits(max_wheel_rad_s=3.0))
        commands = [profile.step(0.6, 0.3, self.dt) for _ in range(600)]
        self.assertTrue(commands[-1].wheel_speed_limited)
        self.assertAlmostEqual(max(abs(value) for value in commands[-1].wheel_targets_rad_s), 3.0)
        self.assertAlmostEqual(commands[-1].angular_rad_s / commands[-1].linear_m_s, 0.5)

    def test_body_ramps_do_not_exceed_wheel_speed_and_keep_progress_at_boundary(self) -> None:
        limits = DriveLimits(max_linear_m_s=0.7, max_angular_rad_s=1.0,
                             linear_accel_m_s2=1.0, linear_decel_m_s2=1.0,
                             angular_accel_rad_s2=1.0, angular_decel_rad_s2=0.1,
                             lateral_accel_m_s2=0.5, max_wheel_rad_s=5.0,
                             wheel_accel_rad_s2=100.0)
        profile = DriveProfile(self.geometry, limits)
        for _ in range(20):
            profile.step(-0.5, -0.5, 0.1)
        commands = [profile.step(-0.7, 0.0, 0.1) for _ in range(120)]
        self.assertTrue(commands[0].wheel_speed_limited)
        for command in commands:
            self.assertLessEqual(max(abs(value) for value in command.wheel_targets_rad_s), 5.0 + 1e-12)
        self.assertAlmostEqual(commands[-1].linear_m_s, -0.7)
        self.assertEqual(commands[-1].angular_rad_s, 0.0)

    def test_curve_speed_scaling_and_every_wheel_step_respect_bounds(self) -> None:
        limits = DriveLimits(wheel_accel_rad_s2=1.0, lateral_accel_m_s2=0.1)
        profile = DriveProfile(self.geometry, limits)
        previous_wheels = (0.0, 0.0)
        commands = []
        for _ in range(1200):
            command = profile.step(0.6, 1.2, self.dt)
            self.assertLessEqual(abs(command.linear_m_s * command.angular_rad_s), 0.1 + 1e-12)
            self.assertLessEqual(max(abs(a - b) for a, b in zip(command.wheel_targets_rad_s, previous_wheels)), self.dt + 1e-12)
            previous_wheels = command.wheel_targets_rad_s
            commands.append(command)
        self.assertTrue(commands[-1].curvature_limited)
        self.assertAlmostEqual(commands[-1].angular_rad_s / commands[-1].linear_m_s, 2.0)

    def test_repeated_target_at_rounded_wheel_boundary_does_not_divide_by_zero(self) -> None:
        limits = DriveLimits(max_linear_m_s=1.4091782047947023,
                             max_angular_rad_s=0.31095792600663275,
                             linear_accel_m_s2=0.42140504283565505,
                             linear_decel_m_s2=0.14471977112596696,
                             angular_accel_rad_s2=0.6527694964692536,
                             angular_decel_rad_s2=1.0455791754646222,
                             lateral_accel_m_s2=0.9270950736576163,
                             max_wheel_rad_s=4.1282972532615085,
                             wheel_accel_rad_s2=17.34274457140063)
        profile = DriveProfile(self.geometry, limits)
        commands = [profile.step(0.5939927409678645, 0.22615842225505012, 0.2) for _ in range(100)]
        for command in commands:
            self.assertLessEqual(max(abs(value) for value in command.wheel_targets_rad_s),
                                 limits.max_wheel_rad_s + 1e-12)

    def test_reversal_reaches_zero_before_opposite_direction(self) -> None:
        for _ in range(150):
            self.profile.step(0.4, 0.0, self.dt)
        values = [self.profile.step(-0.3, 0.0, self.dt).linear_m_s for _ in range(200)]
        zero_index = values.index(0.0)
        self.assertTrue(all(value >= 0 for value in values[:zero_index]))
        self.assertTrue(any(value < 0 for value in values[zero_index + 1:]))

    def test_fault_stop_is_explicit_and_invalid_commands_fail_closed(self) -> None:
        for _ in range(100):
            self.profile.step(0.4, 0.5, self.dt)
        command = self.profile.step(0.0, 0.0, self.dt, emergency_stop=True)
        self.assertEqual(command.wheel_targets_rad_s, (0.0, 0.0))
        self.assertTrue(command.emergency_stop)
        command = self.profile.step(math.nan, math.inf, math.nan, emergency_stop=True)
        self.assertEqual(command.wheel_targets_rad_s, (0.0, 0.0))
        for value in (math.nan, math.inf, 0.7):
            self.profile.step(0.2, 0.0, self.dt)
            with self.assertRaises(ValueError):
                self.profile.step(value, 0.0, self.dt)
            self.assertEqual(self.profile.linear_m_s, 0.0)
        with self.assertRaises(ValueError):
            DriveLimits(linear_decel_m_s2=0.0)


if __name__ == "__main__":
    unittest.main()
