from math import nan, pi
import unittest

from factory_ops.motion import DriveGeometry, VelocityWatchdog, WheelOdometry


class MotionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.geometry = DriveGeometry(0.1, 0.4)

    def test_command_inverse_and_straight_encoders(self) -> None:
        self.assertEqual(self.geometry.wheel_targets(0.2, 0), (2, 2))
        odom = WheelOdometry(self.geometry)
        self.assertEqual(odom.update(2, 7, 8).x_m, 0)
        sample = odom.update(3, 9, 10)
        self.assertAlmostEqual(sample.x_m, 0.2)
        self.assertAlmostEqual(sample.y_m, 0)
        self.assertAlmostEqual(sample.linear_m_s, 0.2)

    def test_rotation_and_forward_in_rotated_frame(self) -> None:
        odom = WheelOdometry(self.geometry)
        odom.update(0, 0, 0)
        # Opposing wheel distances rotate about the axle center.
        sample = odom.update(1, -pi / 2, pi / 2)
        sample = odom.update(2, -pi, pi)
        self.assertAlmostEqual(sample.yaw_rad, pi / 2)
        self.assertAlmostEqual(sample.x_m, 0)
        sample = odom.update(3, -pi + 2, pi + 2)
        self.assertAlmostEqual(sample.x_m, 0)
        self.assertAlmostEqual(sample.y_m, 0.2)

    def test_curved_arc_matches_circle(self) -> None:
        odom = WheelOdometry(self.geometry)
        sample = odom.update(0, 0, 0)
        for index in range(1, 5):
            sample = odom.update(index / 4, 0, index * pi / 2)
        self.assertAlmostEqual(sample.yaw_rad, pi / 2)
        self.assertAlmostEqual(sample.x_m, 0.2)
        self.assertAlmostEqual(sample.y_m, 0.2)

    def test_joint_representation_wrap_does_not_reverse_odometry(self) -> None:
        odom = WheelOdometry(self.geometry)
        odom.update(0, 3.0, 3.0)
        forward = odom.update(0.1, -3.0, -3.0)
        self.assertAlmostEqual(forward.x_m, (2 * pi - 6.0) * 0.1)
        self.assertAlmostEqual(forward.yaw_rad, 0)
        backward = odom.update(0.2, 3.0, 3.0)
        self.assertAlmostEqual(backward.x_m, 0)
        self.assertAlmostEqual(backward.yaw_rad, 0)
        four_pi = WheelOdometry(self.geometry)
        four_pi.update(0, 6.0, 6.0)
        wrapped = four_pi.update(0.1, -6.0, -6.0)
        self.assertAlmostEqual(wrapped.x_m, (4 * pi - 12.0) * 0.1)

    def test_asymmetric_wrap_keeps_small_turn_instead_of_full_revolution(self) -> None:
        odom = WheelOdometry(self.geometry)
        odom.update(0, 3.1, 1.0)
        sample = odom.update(0.1, 3.1 + 0.1 - 2 * pi, 1.2)
        self.assertAlmostEqual(sample.yaw_rad, 0.1 * 0.1 / 0.4)

    def test_invalid_encoder_does_not_mutate(self) -> None:
        odom = WheelOdometry(self.geometry)
        before = odom.update(1, 0, 0)
        for values in [(1, 2, 2), (0, 2, 2), (2, nan, 2)]:
            with self.assertRaises(ValueError):
                odom.update(*values)
            self.assertEqual(odom.sample, before)
        self.assertAlmostEqual(odom.update(2, 1, 1).x_m, 0.1)

    def test_watchdog_stale_future_and_invalid_stop(self) -> None:
        watchdog = VelocityWatchdog(max_age_s=0.5, max_linear_m_s=1, max_angular_rad_s=2)
        self.assertEqual(watchdog.command(0), (0, 0))
        watchdog.receive(1, 0.5, -1)
        self.assertEqual(watchdog.command(1.2), (0.5, -1))
        self.assertEqual(watchdog.command(0.9), (0, 0))
        self.assertEqual(watchdog.command(1.1), (0, 0))
        watchdog.receive(1, 0.5, -1)
        self.assertEqual(watchdog.command(1.6), (0, 0))
        self.assertEqual(watchdog.command(1.2), (0, 0))
        watchdog.receive(1, 0.5, -1)
        with self.assertRaises(ValueError):
            watchdog.command(nan)
        self.assertEqual(watchdog.command(1.2), (0, 0))
        for values in [(1, nan, 0), (1, 2, 0), (1, 0, -3)]:
            watchdog.receive(1, 0.5, 0)
            with self.assertRaises(ValueError):
                watchdog.receive(*values)
            self.assertEqual(watchdog.command(1), (0, 0))

    def test_invalid_geometry_and_limits(self) -> None:
        for dimensions in [(0, 1), (1, -1), (nan, 1)]:
            with self.assertRaises(ValueError):
                DriveGeometry(*dimensions)
        with self.assertRaises(ValueError):
            self.geometry.wheel_targets(nan, 0)
        with self.assertRaises(ValueError):
            VelocityWatchdog(max_age_s=0, max_linear_m_s=1, max_angular_rad_s=1)


if __name__ == "__main__":
    unittest.main()
