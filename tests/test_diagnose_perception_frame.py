import unittest

import numpy as np

from scripts.isaac.diagnose_perception_frame import (
    FRAME_ID,
    MAX_OUTPUT_BYTES,
    SCENE_INDEX,
    TARGET_OBJECT_ID,
    VIEW_ID,
    build_diagnostic,
)


class DiagnosePerceptionFrameTests(unittest.TestCase):
    def test_contract_is_fixed_to_the_single_failed_frame(self):
        self.assertEqual(102, FRAME_ID)
        self.assertEqual(25, SCENE_INDEX)
        self.assertEqual("aisle_low", VIEW_ID)
        self.assertEqual("small_load_carrier-026-03", TARGET_OBJECT_ID)
        self.assertEqual(32 * 1024 * 1024, MAX_OUTPUT_BYTES)

    def test_keeps_annotator_object_sets_separate_and_counts_pixels(self):
        expected = {"pallet-026-01": "pallet", TARGET_OBJECT_ID: "small_load_carrier"}
        semantic = np.array([[0, 2, 2], [0, 0, 0]], dtype=np.uint32)
        instance = np.array([[0, 8, 8], [0, 0, 0]], dtype=np.uint32)
        semantic_info = {"idToLabels": {
            "0": {"class": "BACKGROUND"},
            "2": {"class": "pallet", "object_id": "pallet-026-01"},
        }}
        instance_info = {
            "idToLabels": {"0": "BACKGROUND", "8": "/World/Objects/pallet_026_01"},
            "idToSemantics": {
                "0": {"class": "BACKGROUND"},
                "8": {"class": "pallet", "object_id": "pallet-026-01"},
            },
        }
        tight_rows = [
            {"semanticId": 4, "x_min": 0, "y_min": 1, "x_max": 0, "y_max": 1, "occlusionRatio": 0.0}
        ]
        tight_info = {"idToLabels": {
            "4": {"class": "small_load_carrier", "object_id": TARGET_OBJECT_ID}
        }}
        loose_rows = [
            {"semanticId": 4, "x_min": -20, "y_min": 0, "x_max": 0, "y_max": 2, "occlusionRatio": 0.0}
        ]
        result = build_diagnostic(
            expected=expected,
            semantic=semantic,
            semantic_info=semantic_info,
            instance=instance,
            instance_info=instance_info,
            tight_rows=tight_rows,
            tight_info=tight_info,
            loose_rows=loose_rows,
            loose_info=tight_info,
        )
        target = result["target_observation"]
        self.assertTrue(target["tight_bbox_present"])
        self.assertTrue(target["tight_bbox_touches_image_border"])
        self.assertTrue(target["loose_bbox_present"])
        self.assertEqual(0, target["semantic_pixel_count"])
        self.assertEqual(0, target["instance_pixel_count"])
        pallet = result["objects"]["pallet-026-01"]
        self.assertEqual(2, pallet["semantic_masks"][0]["pixel_count"])
        self.assertEqual(2, pallet["instance_masks"][0]["pixel_count"])
        self.assertEqual("/World/Objects/pallet_026_01", pallet["instance_masks"][0]["prim_path"])
        self.assertFalse(result["cause_confirmed"])

    def test_diagnostic_summary_does_not_promote_a_capture(self):
        expected = {TARGET_OBJECT_ID: "small_load_carrier"}
        result = build_diagnostic(
            expected=expected,
            semantic=np.zeros((2, 3), dtype=np.uint32),
            semantic_info={"idToLabels": {"0": {"class": "BACKGROUND"}}},
            instance=np.zeros((2, 3), dtype=np.uint32),
            instance_info={"idToLabels": {"0": "BACKGROUND"}, "idToSemantics": {"0": {"class": "BACKGROUND"}}},
            tight_rows=[],
            tight_info={"idToLabels": {}},
            loose_rows=[],
            loose_info={"idToLabels": {}},
        )
        self.assertEqual([], result["objects"][TARGET_OBJECT_ID]["semantic_masks"])
        self.assertEqual([], result["objects"][TARGET_OBJECT_ID]["instance_masks"])
        self.assertFalse(result["cause_confirmed"])


if __name__ == "__main__":
    unittest.main()
