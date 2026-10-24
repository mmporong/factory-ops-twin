import unittest

from factory_ops.operations_clip_selection import ClipSelectionError, OperationsClipSelector


def snapshot(stamp, *, shipped=0, returned=0, deliveries=0, robots=()):
    return {
        "sim_time_s": float(stamp),
        "production_line": {"completed_lots": shipped, "returned": returned},
        "traffic": {"deliveries": {"amr-1": deliveries}},
        "robots": [
            {
                "id": robot_id,
                "task_id": task_id,
                "task_type": task_type,
                "payload_id": payload_id,
                "pose": {"x_m": x_m, "y_m": y_m},
            }
            for robot_id, task_id, task_type, payload_id, x_m, y_m in robots
        ],
    }


class OperationsClipSelectorTests(unittest.TestCase):
    def test_window_with_both_shipment_and_return_wins(self):
        selector = OperationsClipSelector(sample_count=3)
        rows = [
            snapshot(0, robots=(("amr-1", "move-1", "product_move", "T1", 0, 0),)),
            snapshot(1, shipped=1, deliveries=1,
                     robots=(("amr-1", "move-1", "product_move", "T1", 1, 0),)),
            snapshot(2, shipped=1, returned=1, deliveries=2,
                     robots=(("amr-1", "move-1", "product_move", "T1", 2, 0),)),
            snapshot(3, shipped=1, returned=1, deliveries=9,
                     robots=(("amr-1", "move-1", "product_move", "T1", 3, 0),)),
        ]
        for row in rows:
            selector.offer(row)
        evidence = selector.evidence()
        self.assertEqual((0.0, 2.0), (evidence["first"]["sim_time_s"], evidence["last"]["sim_time_s"]))
        self.assertTrue(evidence["score"]["shipment_and_return_both_increased"])
        self.assertEqual(1, evidence["score"]["minimum_shipment_return_delta"])

    def test_parking_motion_is_excluded_but_payload_free_pickup_approach_counts(self):
        selector = OperationsClipSelector(sample_count=3)
        selector.offer(snapshot(0, robots=(
            ("amr-1", "park-1", "parking", None, 0, 0),
            ("amr-2", "pickup-1", "process_input", None, 0, 0),
        )))
        selector.offer(snapshot(1, robots=(
            ("amr-1", "park-1", "parking", None, 100, 0),
            ("amr-2", "pickup-1", "process_input", None, 3, 4),
        )))
        score = selector.evidence()["score"]
        self.assertEqual(5.0, score["logistics_pose_distance_m"])
        self.assertEqual(1, score["distinct_moving_logistics_robots"])

    def test_task_change_edge_is_not_counted_as_logistics_motion(self):
        selector = OperationsClipSelector(sample_count=2)
        selector.offer(snapshot(0, robots=(("amr-1", "task-a", "product_move", "T1", 0, 0),)))
        selector.offer(snapshot(1, robots=(("amr-1", "task-b", "product_move", "T2", 40, 30),)))
        score = selector.evidence()["score"]
        self.assertEqual(0.0, score["logistics_pose_distance_m"])
        self.assertEqual(0, score["distinct_moving_logistics_robots"])

    def test_buffer_and_best_window_never_exceed_300_samples(self):
        selector = OperationsClipSelector(sample_count=300)
        for stamp in range(301):
            selector.offer(snapshot(stamp, deliveries=stamp))
        self.assertEqual(300, len(selector.best_snapshots()))
        # Both full windows have the same delta, so the earlier candidate wins the tie.
        self.assertEqual((0.0, 299.0), (
            selector.evidence()["first"]["sim_time_s"],
            selector.evidence()["last"]["sim_time_s"],
        ))

    def test_time_and_cumulative_counter_errors_fail_closed(self):
        selector = OperationsClipSelector(sample_count=3)
        selector.offer(snapshot(0, shipped=1, returned=1, deliveries=1))
        with self.assertRaisesRegex(ClipSelectionError, "monotonically"):
            selector.offer(snapshot(0, shipped=1, returned=1, deliveries=1))

        skipped = OperationsClipSelector(sample_count=3)
        skipped.offer(snapshot(0))
        with self.assertRaisesRegex(ClipSelectionError, "skipped the next 1 Hz sample"):
            skipped.offer(snapshot(2))

        regressed = OperationsClipSelector(sample_count=3)
        regressed.offer(snapshot(0, shipped=1, returned=1, deliveries=1))
        with self.assertRaisesRegex(ClipSelectionError, "must not decrease"):
            regressed.offer(snapshot(1, shipped=0, returned=1, deliveries=1))

    def test_partial_window_has_unpadded_boundaries_and_metadata(self):
        selector = OperationsClipSelector(sample_count=5)
        first = snapshot(10, shipped=2, returned=1, deliveries=4)
        second = snapshot(11, shipped=3, returned=2, deliveries=6)
        selector.offer(first)
        selector.offer(second)
        evidence = selector.evidence()
        self.assertEqual((first, second), selector.best_snapshots())
        self.assertEqual(2, evidence["selected_sample_count"])
        self.assertEqual(
            {"sim_time_s": 10.0, "shipments_completed": 2, "returns_completed": 1,
             "transports_completed": 4},
            evidence["first"],
        )
        self.assertEqual(2, evidence["score"]["completed_transport_delta"])

    def test_subsecond_offers_are_downsampled_without_copying(self):
        selector = OperationsClipSelector(sample_count=2)
        first = snapshot(0)
        ignored = snapshot(0.5)
        second = snapshot(1)
        self.assertTrue(selector.offer(first))
        self.assertFalse(selector.offer(ignored))
        self.assertTrue(selector.offer(second))
        self.assertEqual(2, len(selector.best_snapshots()))
        self.assertIs(first, selector.best_snapshots()[0])


if __name__ == "__main__":
    unittest.main()
