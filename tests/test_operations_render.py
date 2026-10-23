from copy import deepcopy
import math
import unittest

from factory_ops.operations_render import RecordedOperations


def frame(stamp, x, yaw, payload):
    return {"run_id": "render-test", "execution_mode": "operations", "dispatch_policy": "eta",
            "sim_time_s": stamp, "kpis": {"completed_orders": int(stamp)},
            "robots": [{"id": f"amr-{i}", "pose": {"x_m": x, "y_m": i, "yaw_rad": yaw},
                        "payload_id": payload} for i in range(1, 5)]}


class OperationsRenderTest(unittest.TestCase):
    def test_pose_interpolation_does_not_anticipate_material_handover(self):
        original = [frame(0, 0, math.radians(179), None), frame(1, .4, math.radians(-179), "lot")]
        before = deepcopy(original)
        replay = RecordedOperations(original)
        state, meta = replay.sample(.5)
        self.assertAlmostEqual(state["robots"][0]["pose"]["x_m"], .2)
        self.assertAlmostEqual(state["robots"][0]["pose"]["yaw_rad"], math.pi)
        self.assertIsNone(state["robots"][0]["payload_id"])
        self.assertEqual(state["kpis"]["completed_orders"], 0)
        self.assertEqual(meta["state_observed_time_s"], 0)
        self.assertEqual(original, before)
        state, _ = replay.sample(1)
        self.assertEqual(state["robots"][0]["payload_id"], "lot")

    def test_no_extrapolation_or_mixed_run(self):
        frames = [frame(0, 0, 0, None), frame(1, .4, 0, None)]
        replay = RecordedOperations(frames)
        with self.assertRaises(ValueError):
            replay.sample(1.1)
        frames[1]["run_id"] = "other"
        with self.assertRaises(ValueError):
            RecordedOperations(frames)


if __name__ == "__main__":
    unittest.main()
