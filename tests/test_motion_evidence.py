from __future__ import annotations

import math
from copy import deepcopy
import unittest

from scripts.isaac.motion_evidence import steady_response, validate_capture_alignment


class MotionEvidenceTests(unittest.TestCase):
    def test_chassis_response_uses_pose_differences_and_unwraps_yaw(self) -> None:
        samples = []
        # A 0.4 m radius CCW arc crosses the quaternion yaw branch at pi.
        for index in range(61):
            elapsed = index / 60
            yaw = 3.0 + 0.5 * elapsed
            samples.append({"sim_time_s": elapsed,
                            "odometry": {"linear_m_s": 0.21, "angular_rad_s": 0.51},
                            "ground_truth": {"position_xyz_m": [0.4 * math.sin(yaw), -0.4 * math.cos(yaw), 0.14],
                                             "orientation_wxyz": [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]}})
        actual = steady_response(samples)
        self.assertAlmostEqual(actual["encoder_linear_m_s"], 0.21)
        self.assertAlmostEqual(actual["chassis_linear_m_s"], 0.2, places=5)
        self.assertAlmostEqual(actual["chassis_angular_rad_s"], 0.5)
        self.assertAlmostEqual(actual["encoder_angular_rad_s"], 0.51)

    def test_steady_response_does_not_accept_an_empty_or_single_sample(self) -> None:
        for samples in ([], [{}]):
            with self.assertRaises(ValueError):
                steady_response(samples)

    def test_capture_alignment_rejects_duplicated_and_missing_maneuver_frames(self) -> None:
        samples = [{"physics_frame": i + 120, "sim_time_s": (i + 120) / 60,
                    "schedule_segment": "straight" if i < 6 else "turn"} for i in range(12)]
        frames = [dict(samples[step], index=index) for index, step in enumerate((0, 5, 11))]
        self.assertEqual(validate_capture_alignment(frames, samples, total_count=12, period=6), [])
        duplicated = deepcopy(frames)
        duplicated[2] = dict(duplicated[1], index=2)
        self.assertTrue(validate_capture_alignment(duplicated, samples, total_count=12, period=6))
        self.assertTrue(validate_capture_alignment(frames[:-1], samples, total_count=12, period=6))
        reordered = [frames[0], dict(frames[2], index=1), dict(frames[1], index=2)]
        self.assertTrue(validate_capture_alignment(reordered, samples, total_count=12, period=6))


if __name__ == "__main__":
    unittest.main()
