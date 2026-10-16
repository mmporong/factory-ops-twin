from pathlib import Path
import unittest

from factory_ops.traffic import TrafficController
from factory_ops.traffic_observation import circle_intersects_polygon, project_observation

ROOT = Path(__file__).resolve().parents[1]


class EstimatedTrafficTests(unittest.TestCase):
    def controller(self):
        return TrafficController.from_file(ROOT / "configs/factory-v3/traffic.json")

    def observe(self, traffic, robot, xy, now):
        traffic.update_observation(robot, pose_xy=xy, pose_time=now, heartbeat_time=now,
                                   **project_observation(traffic, xy, 1.137109), now=now)

    def test_intersection_uses_circle_boundary_and_root_offset(self):
        square = [[0, 0], [2, 0], [2, 2], [0, 2]]
        self.assertTrue(circle_intersects_polygon([1, 1], .1, square))
        self.assertTrue(circle_intersects_polygon([3, 1], 1, square))
        self.assertFalse(circle_intersects_polygon([3.01, 1], 1, square))
        self.assertFalse(circle_intersects_polygon([3, 3], 1, square))

    def test_projection_requires_full_footprint_clearance(self):
        traffic = self.controller()
        self.assertNotIn("Z_UPPER", project_observation(traffic, (22, 4), 1.137109)["occupied_resources"])
        self.assertIn("Z_UPPER", project_observation(traffic, (22.5, 4), 1.137109)["occupied_resources"])
        self.assertNotIn("Z_UPPER:east", project_observation(traffic, (30.5, 4), 1.137109)["clear_exits"])

    def test_exit_wait_must_be_observed_inside_owned_resource(self):
        traffic = self.controller()
        self.observe(traffic, "carter1", (22, 4), 0)
        self.observe(traffic, "carter2", (30, 4), 0)
        traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        self.assertFalse(traffic.observe_exit_wait("carter1", "carter2", now=0, planning_radius_m=1.137109))
        self.observe(traffic, "carter1", (26.8, 4), .1)
        self.assertTrue(traffic.observe_exit_wait("carter1", "carter2", now=.1, planning_radius_m=1.137109))
        self.assertFalse(traffic.observe_exit_wait("carter2", "carter1", now=.1, planning_radius_m=1.137109))

    def test_stale_or_departed_peer_cannot_sustain_a_cycle(self):
        traffic = self.controller()
        for time_s in (0, 1, 2, 3, 4, 5):
            self.observe(traffic, "carter1", (26.8, 4), time_s)
            self.observe(traffic, "carter2", (30, 4), time_s)
            traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=time_s)
            traffic.request("carter2", ["Z_UPPER"], "Z_UPPER:west", now=time_s)
            self.assertTrue(traffic.observe_exit_wait("carter1", "carter2", now=time_s, planning_radius_m=1.137109))
        self.assertEqual(traffic.detect_cycle(now=5), ("carter1", "carter2"))
        self.assertIsNone(traffic.detect_cycle(now=6.01))
        self.observe(traffic, "carter1", (26.8, 4), 6.1)
        self.observe(traffic, "carter2", (32, 8), 6.1)
        self.assertFalse(traffic.observe_exit_wait("carter1", "carter2", now=6.1, planning_radius_m=1.137109))
        self.assertIsNone(traffic.detect_cycle(now=6.1))

    def test_departure_and_return_between_cycle_queries_reset_persistence(self):
        traffic = self.controller()
        self.observe(traffic, "carter1", (26.8, 4), 0)
        self.observe(traffic, "carter2", (30, 4), 0)
        traffic.request("carter1", ["Z_UPPER"], "Z_UPPER:east", now=0)
        traffic.request("carter2", ["Z_UPPER"], "Z_UPPER:west", now=0)
        traffic.observe_exit_wait("carter1", "carter2", now=0, planning_radius_m=1.137109)
        self.observe(traffic, "carter2", (32, 8), 1)
        for time_s in (2, 3, 4, 5):
            self.observe(traffic, "carter1", (26.8, 4), time_s)
            self.observe(traffic, "carter2", (30, 4), time_s)
            traffic.observe_exit_wait("carter1", "carter2", now=time_s, planning_radius_m=1.137109)
        self.assertIsNone(traffic.detect_cycle(now=5))


if __name__ == "__main__":
    unittest.main()
