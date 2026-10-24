import copy
import unittest

from factory_ops.operations_render import RecordedOperations
from scripts.media.select_operations_excerpt import completion_contract, rank_excerpts


class OperationsExcerptTests(unittest.TestCase):
    def complete_report(self):
        return {"status": "complete", "error": None, "source_stable_during_run": True,
                "workload": {"lots": 48}, "final_snapshot": {
                    "robots": [{"id": f"amr-{i}"} for i in range(1, 17)],
                    "production_line": {"completed_lots": 48, "returned": 48},
                    "traffic": {"quiescent": True, "peak_transport_active": 16,
                                "deliveries": {f"amr-{i}": 2 for i in range(1, 17)}}}}

    def test_closed_completion_alone_does_not_prove_material_fleet_goal(self):
        report = self.complete_report()
        self.assertTrue(completion_contract(report, fleet=16, lots=48)["passed"])
        for change in ("material_peak", "unused_robot", "returns", "fleet", "source", "case"):
            with self.subTest(change=change):
                candidate = copy.deepcopy(report)
                state = candidate["final_snapshot"]
                if change == "material_peak":
                    state["traffic"].update(peak_transport_active=13, peak_moving_robots=16)
                elif change == "unused_robot":
                    state["traffic"]["deliveries"]["amr-16"] = 1
                elif change == "returns":
                    state["production_line"]["returned"] = 47
                elif change == "fleet":
                    state["robots"].pop()
                elif change == "source":
                    candidate["source_stable_during_run"] = False
                else:
                    candidate["status"] = "incomplete"
                self.assertFalse(completion_contract(candidate, fleet=16, lots=48)["passed"])

    def test_excerpt_fits_source_and_prefers_shipment_return_over_parking_motion(self):
        rows = []
        for stamp in range(5):
            rows.append({
                "run_id": "excerpt-test", "execution_mode": "operations", "dispatch_policy": "fifo",
                "sim_time_s": float(stamp),
                "production_line": {"completed_lots": int(stamp >= 2), "returned": int(stamp >= 3)},
                "traffic": {"deliveries": {"amr-1": stamp, "amr-2": 0}},
                "robots": [
                    {"id": "amr-1", "task_id": "material", "task_type": "product_move",
                     "pose": {"x_m": stamp, "y_m": 0, "yaw_rad": 0}},
                    {"id": "amr-2", "task_id": "home", "task_type": "parking",
                     "pose": {"x_m": stamp * 100, "y_m": 0, "yaw_rad": 0}},
                ]})
        candidates = rank_excerpts(RecordedOperations(rows), 2.8)
        self.assertEqual(2, len(candidates))
        self.assertEqual(1.0, candidates[0]["start_s"])
        self.assertEqual(3.8, candidates[0]["last_frame_source_s"])
        self.assertEqual(3.0, candidates[0]["last_observed_sample_s"])
        score = candidates[0]["selection"]["score"]
        self.assertTrue(score["shipment_and_return_both_increased"])
        self.assertEqual(2.0, score["logistics_pose_distance_m"])
        self.assertEqual(1, score["distinct_moving_logistics_robots"])


if __name__ == "__main__":
    unittest.main()
