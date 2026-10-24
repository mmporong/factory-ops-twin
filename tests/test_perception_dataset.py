import unittest

import numpy as np

from factory_ops.perception_dataset import PerceptionDatasetError, usd_safe_identifier, validate_capture
from scripts.isaac.capture_perception_probe import (
    LEGACY_ASSETS,
    PROBE_SEMANTIC_TYPES,
    ProbeError,
    class_asset_ids,
    scene_plan,
    source_receipt_sha256,
)


class PerceptionDatasetTests(unittest.TestCase):
    def setUp(self) -> None:
        self.semantic = np.array([
            [0, 1, 1, 0, 0],
            [0, 1, 1, 0, 2],
            [0, 0, 0, 2, 2],
            [0, 0, 0, 2, 2],
        ], dtype=np.uint32)
        self.semantic_info = {"idToLabels": {
            "0": {"class": "BACKGROUND"},
            "1": {"class": "pallet"},
            "2": {"class": "small_load_carrier"},
        }}
        self.bbox_info = {"idToLabels": {
            "11": {"class": "pallet", "object_id": "pallet-01"},
            "12": {"class": "small_load_carrier", "object_id": "crate-01"},
        }}
        self.rows = [
            {"semanticId": 11, "x_min": 1, "y_min": 0, "x_max": 2, "y_max": 1},
            {"semanticId": 12, "x_min": 3, "y_min": 1, "x_max": 4, "y_max": 3},
        ]
        self.expected = {"pallet-01": "pallet", "crate-01": "small_load_carrier"}

    def test_valid_capture_binds_every_bbox_to_object_and_class(self) -> None:
        result = validate_capture(
            width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
            bbox_rows=self.rows, bbox_info=self.bbox_info, expected_objects=self.expected,
        )
        self.assertEqual(result["bbox_object_count"], 2)
        self.assertEqual(result["class_pixel_counts"], {"pallet": 4, "small_load_carrier": 5})
        self.assertEqual(result["annotations"][0]["object_id"], "crate-01")
        pallet = next(item for item in result["annotations"] if item["object_id"] == "pallet-01")
        self.assertEqual(pallet["bbox"], [1, 0, 2, 2])
        self.assertEqual(pallet["area"], 4)

    def test_usd_identifier_is_safe_without_changing_semantic_source(self) -> None:
        object_id = "pallet-01-01"
        self.assertEqual(usd_safe_identifier(object_id), "pallet_01_01")
        self.assertEqual(object_id, "pallet-01-01")
        self.assertEqual(usd_safe_identifier("7-small.load/carrier"), "_7_small_load_carrier")

    def test_probe_requests_class_and_object_identity_from_every_label_annotator(self) -> None:
        self.assertEqual(PROBE_SEMANTIC_TYPES, ("class", "object_id"))

    def test_probe_uses_pinned_class_asset_contract(self) -> None:
        prepared = {
            "status": "passed_candidate_probe_input_prepared",
            "probe_contract": {
                "class_asset_ids": {
                    "pallet": "nvidia:pallet_block_a01",
                    "small_load_carrier": "nvidia:crate_blue_a02",
                },
                "maximum_capture_count": 16,
                "candidate_probe_only": True,
                "model_training_performed": False,
            },
        }
        assets = class_asset_ids(prepared)
        self.assertEqual(assets["small_load_carrier"], "nvidia:crate_blue_a02")
        self.assertEqual({item["asset_id"] for item in scene_plan(101, 0, assets)}, set(assets.values()))
        self.assertEqual(class_asset_ids({"status": "passed"}), LEGACY_ASSETS)

    def test_probe_rejects_unbounded_or_incomplete_asset_contract(self) -> None:
        base = {
            "status": "passed_candidate_probe_input_prepared",
            "probe_contract": {
                "class_asset_ids": {"pallet": "nvidia:pallet_block_a01"},
                "maximum_capture_count": 16,
                "candidate_probe_only": True,
                "model_training_performed": False,
            },
        }
        with self.assertRaisesRegex(ProbeError, "exactly the primary classes"):
            class_asset_ids(base)
        base["probe_contract"]["class_asset_ids"]["small_load_carrier"] = "external:anything"
        with self.assertRaisesRegex(ProbeError, "invalid asset ID"):
            class_asset_ids(base)
        base["probe_contract"]["class_asset_ids"]["small_load_carrier"] = "nvidia:crate_blue_a02"
        base["probe_contract"]["maximum_capture_count"] = 512
        with self.assertRaisesRegex(ProbeError, "bounded candidate recipe"):
            class_asset_ids(base)

    def test_probe_accepts_only_pinned_source_receipt_hashes(self) -> None:
        digest = "a" * 64
        self.assertEqual(source_receipt_sha256({"source_receipt_sha256": digest}), digest)
        self.assertEqual(
            source_receipt_sha256({"inputs": {"source_receipt": {"sha256": digest}}}),
            digest,
        )
        with self.assertRaisesRegex(ProbeError, "pinned source receipt"):
            source_receipt_sha256({"inputs": {"source_receipt": {"sha256": "not-a-hash"}}})

    def test_missing_bbox_fails_closed(self) -> None:
        with self.assertRaisesRegex(PerceptionDatasetError, "coverage differs"):
            validate_capture(
                width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
                bbox_rows=self.rows[:1], bbox_info=self.bbox_info, expected_objects=self.expected,
            )

    def test_bbox_class_mismatch_and_outside_extent_fail(self) -> None:
        wrong = {"idToLabels": {**self.bbox_info["idToLabels"], "11": {"class": "small_load_carrier", "object_id": "pallet-01"}}}
        with self.assertRaisesRegex(PerceptionDatasetError, "differs from provenance"):
            validate_capture(
                width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
                bbox_rows=self.rows, bbox_info=wrong, expected_objects=self.expected,
            )
        outside = [dict(self.rows[0], x_max=5), self.rows[1]]
        with self.assertRaisesRegex(PerceptionDatasetError, "outside"):
            validate_capture(
                width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
                bbox_rows=outside, bbox_info=self.bbox_info, expected_objects=self.expected,
            )


if __name__ == "__main__":
    unittest.main()
