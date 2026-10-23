from __future__ import annotations

from collections import deque
import math
import unittest

from factory_ops.drive_profile import DriveLimits
from factory_ops.motion import DriveGeometry
from factory_ops.operations_motion import PathMotion


class PathMotionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = DriveLimits()
        self.geometry = DriveGeometry(0.14, 0.4132)

    def test_dock_alignment_keeps_position_and_respects_angular_envelope(self):
        motion = PathMotion(self.limits, self.geometry)
        pose = {"x_m": 12.0, "y_m": 1.5, "yaw_rad": 0.0}
        previous = 0.0
        dt = 0.001
        for _ in range(20000):
            done = motion.rotate_to(pose, -math.pi / 2, dt)
            self.assertEqual((pose["x_m"], pose["y_m"]), (12.0, 1.5))
            self.assertLessEqual(abs(motion.angular_rad_s), self.limits.max_angular_rad_s + 1e-9)
            self.assertLessEqual(abs(motion.angular_rad_s - previous) / dt,
                                 min(self.limits.angular_accel_rad_s2, self.limits.angular_decel_rad_s2) + 1e-6)
            previous = motion.angular_rad_s
            if done:
                break
        self.assertTrue(done)
        self.assertAlmostEqual(pose["yaw_rad"], -math.pi / 2)
        self.assertEqual(motion.angular_rad_s, 0)
        points = deque([(12.0, 4.5)])
        motion.advance(pose, points, 0.1)
        with self.assertRaises(ValueError):
            motion.rotate_to(pose, 0, 0.1)

    def test_alignment_target_is_fixed_and_time_estimate_covers_waypoint_motion(self):
        motion = PathMotion(self.limits, self.geometry)
        pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        motion.rotate_to(pose, math.pi / 2, 0.1)
        with self.assertRaises(ValueError):
            motion.rotate_to(pose, 0, 0.1)
        with self.assertRaises(ValueError):
            motion.advance(pose, deque(), 0.1)
        path = ((0.0, 0.0), (0.08, 0.0), (0.08, 0.07))
        motion = PathMotion(self.limits, self.geometry)
        estimate = motion.estimated_duration(path, 0)
        pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        points = deque(path)
        elapsed = 0.0
        while points:
            motion.advance(pose, points, 0.001)
            elapsed += 0.001
        self.assertLessEqual(elapsed, estimate + 0.001)
        self.assertGreaterEqual(elapsed, estimate - 0.001)

    def run_path(self, dt_s: float) -> tuple[dict[str, float], PathMotion]:
        pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        points = deque([(0.0, 0.0), (0.08, 0.0), (0.08, 0.07)])
        motion = PathMotion(self.limits, self.geometry)
        for _ in range(100_000):
            motion.advance(pose, points, dt_s)
            if motion.stopped and not points:
                break
        else:
            self.fail("path did not finish")
        return pose, motion

    def test_short_segment_and_right_angle_corner_finish_at_rest(self) -> None:
        pose, motion = self.run_path(1.0 / 60.0)
        self.assertAlmostEqual(pose["x_m"], 0.08)
        self.assertAlmostEqual(pose["y_m"], 0.07)
        self.assertAlmostEqual(pose["yaw_rad"], math.pi / 2)
        self.assertEqual(motion.linear_m_s, 0.0)
        self.assertEqual(motion.angular_rad_s, 0.0)
        self.assertEqual(motion.phase, "stopped")

    def test_body_and_wheel_speed_and_acceleration_limits_hold(self) -> None:
        limits = DriveLimits(
            max_linear_m_s=2.0,
            max_angular_rad_s=5.0,
            linear_accel_m_s2=5.0,
            linear_decel_m_s2=4.0,
            angular_accel_rad_s2=6.0,
            angular_decel_rad_s2=5.0,
            lateral_accel_m_s2=2.0,
            max_wheel_rad_s=1.0,
            wheel_accel_rad_s2=0.8,
        )
        pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": math.pi}
        points = deque([(1.0, 0.0)])
        motion = PathMotion(limits, self.geometry)
        dt_s = 1.0 / 1000.0
        before_linear = before_angular = 0.0
        before_wheels = (0.0, 0.0)
        phases: set[str] = set()
        for _ in range(100_000):
            motion.advance(pose, points, dt_s)
            phases.add(motion.phase)
            wheels = self.geometry.wheel_targets(motion.linear_m_s, motion.angular_rad_s)
            self.assertLessEqual(abs(motion.linear_m_s), limits.max_linear_m_s + 1e-10)
            self.assertLessEqual(abs(motion.angular_rad_s), limits.max_angular_rad_s + 1e-10)
            self.assertLessEqual(max(map(abs, wheels)), limits.max_wheel_rad_s + 1e-10)
            self.assertLessEqual(
                abs(motion.linear_m_s - before_linear) / dt_s,
                min(limits.linear_accel_m_s2, limits.linear_decel_m_s2) + 2e-4,
            )
            self.assertLessEqual(
                abs(motion.angular_rad_s - before_angular) / dt_s,
                min(limits.angular_accel_rad_s2, limits.angular_decel_rad_s2) + 2e-4,
            )
            self.assertLessEqual(
                max(abs(after - before) for before, after in zip(before_wheels, wheels)) / dt_s,
                limits.wheel_accel_rad_s2 + 2e-3,
            )
            before_linear, before_angular, before_wheels = motion.linear_m_s, motion.angular_rad_s, wheels
            if motion.stopped and not points:
                break
        self.assertIn("turn", phases)
        self.assertIn("translate", phases)

    def test_result_is_invariant_to_dt_chunking(self) -> None:
        def state_after(chunks: list[float]) -> tuple[dict[str, float], PathMotion]:
            pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
            points = deque([(0.2, 0.0), (0.2, 0.2)])
            motion = PathMotion(self.limits, self.geometry)
            for chunk in chunks:
                motion.advance(pose, points, chunk)
            return pose, motion

        fine_pose, fine_motion = state_after([0.01] * 500)
        coarse_pose, coarse_motion = state_after([1.7, 2.4, 0.9])
        for key in ("x_m", "y_m", "yaw_rad"):
            self.assertAlmostEqual(fine_pose[key], coarse_pose[key], places=11)
        self.assertAlmostEqual(fine_motion.linear_m_s, coarse_motion.linear_m_s, places=11)
        self.assertAlmostEqual(fine_motion.angular_rad_s, coarse_motion.angular_rad_s, places=11)
        self.assertEqual(fine_motion.phase, coarse_motion.phase)

    def test_varied_dt_paths_remain_finite_and_finish(self) -> None:
        for dt_s in (1.0 / 240.0, 1.0 / 60.0, 0.17, 2.0):
            with self.subTest(dt_s=dt_s):
                pose, motion = self.run_path(dt_s)
                self.assertTrue(all(math.isfinite(value) for value in pose.values()))
                self.assertTrue(math.isfinite(motion.linear_m_s))
                self.assertTrue(math.isfinite(motion.angular_rad_s))

    def test_empty_route_can_wait_then_accept_a_new_route(self) -> None:
        pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        points: deque[tuple[float, float]] = deque()
        motion = PathMotion(self.limits, self.geometry)
        motion.advance(pose, points, 0.1)
        self.assertTrue(motion.stopped)
        points.append((0.1, 0.0))
        motion.advance(pose, points, 0.1)
        self.assertFalse(motion.stopped)
        self.assertGreater(pose["x_m"], 0.0)

    def test_route_change_while_moving_is_rejected(self) -> None:
        pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        points = deque([(1.0, 0.0)])
        motion = PathMotion(self.limits, self.geometry)
        motion.advance(pose, points, 0.1)
        points.append((2.0, 0.0))
        with self.assertRaisesRegex(ValueError, "cannot change"):
            motion.advance(pose, points, 0.1)
        with self.assertRaisesRegex(ValueError, "cannot change"):
            motion.advance(pose, deque([(1.0, 0.0)]), 0.1)

    def test_invalid_inputs_do_not_start_motion(self) -> None:
        valid_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        for pose, points, dt in (
            ({"x_m": math.nan, "y_m": 0.0, "yaw_rad": 0.0}, deque(), 0.1),
            (valid_pose.copy(), deque([(math.inf, 0.0)]), 0.1),
            (valid_pose.copy(), deque([(1.0, 0.0)]), 0.0),
        ):
            motion = PathMotion(self.limits, self.geometry)
            with self.assertRaises(ValueError):
                motion.advance(pose, points, dt)
            self.assertTrue(motion.stopped)
        motion = PathMotion(self.limits, self.geometry)
        with self.assertRaises(TypeError):
            motion.advance(valid_pose.copy(), [(1.0, 0.0)], 0.1)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
