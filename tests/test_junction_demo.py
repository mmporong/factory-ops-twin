import math
import unittest
from factory_ops.junction_demo import JunctionDemo


class JunctionDemoTests(unittest.TestCase):
    def test_wait_requires_actual_estimated_exit_clearance(self):
        d = JunctionDemo("test")
        poses = {k: r.spawn for k, r in d.routes.items()}
        poses.update(carter1=(12.5, 17., 0.), carter2=(15., 14.6, math.pi/2))
        d.observe(poses, 1.)
        d.command("carter1", poses["carter1"], 1., 30.)
        self.assertEqual(d.command("carter2", poses["carter2"], 1., 30.)[:2], (0., 0.))
        self.assertEqual(d.states["carter2"], "yield_to_cross_traffic")
        poses["carter1"] = (17., 17., 0.)
        d.observe(poses, 2.)
        d.command("carter1", poses["carter1"], 2., 30.)
        d.command("carter2", poses["carter2"], 2., 30.)
        self.assertFalse(d.routes["carter2"].reserved)
        poses["carter1"] = (17.2, 17., 0.)
        d.observe(poses, 3.)
        d.command("carter1", poses["carter1"], 3., 30.)
        d.command("carter2", poses["carter2"], 3., 30.)
        self.assertTrue(d.result()["yield_clear_resume_observed"])

    def test_bad_range_stops_and_pose_uses_encoder_only(self):
        d = JunctionDemo("test")
        self.assertEqual(d.command("carter1", (11., 17., 0.), 1., float("nan")), (0., 0., True))
        self.assertEqual(d.command("carter1", (11., 17., 0.), 2., .8), (0., 0., True))

    def test_approach_reserves_downstream_space_before_north_entry(self):
        d = JunctionDemo("test")
        poses = {k: r.spawn for k, r in d.routes.items()}
        d.observe(poses, 0.)
        d.command("carter1", poses["carter1"], 0., 30.)
        d.command("carter2", poses["carter2"], 0., 30.)
        self.assertTrue(d.routes["carter1"].reserved)
        self.assertFalse(d.routes["carter2"].reserved)
        self.assertNotIn("J:west", d.clear_exits["carter2"])


if __name__ == "__main__":
    unittest.main()
