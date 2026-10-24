import unittest

import numpy as np

from factory_ops.perception_dataset import (
    PerceptionDatasetError,
    usd_safe_identifier,
    validate_capture,
    validate_distance_to_camera,
)
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
            "1": {"class": "pallet", "object_id": "pallet-01"},
            "2": {"class": "small_load_carrier", "object_id": "crate-01"},
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
        self.assertEqual(result["object_mask_checks"]["matched_object_count"], 2)
        self.assertEqual(
            result["object_mask_checks"]["objects"]["crate-01"]["extent_inclusive"],
            [3, 1, 4, 3],
        )

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

    def test_bbox_must_equal_same_object_semantic_mask_extent(self) -> None:
        wider = [dict(self.rows[0], x_min=0), self.rows[1]]
        with self.assertRaisesRegex(PerceptionDatasetError, "same object's semantic mask"):
            validate_capture(
                width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
                bbox_rows=wider, bbox_info=self.bbox_info, expected_objects=self.expected,
            )

    def test_fully_hidden_object_fails_closed(self) -> None:
        hidden = self.semantic.copy()
        hidden[hidden == 2] = 0
        with self.assertRaisesRegex(PerceptionDatasetError, "no visible pixels for object: crate-01"):
            validate_capture(
                width=5, height=4, semantic=hidden, semantic_info=self.semantic_info,
                bbox_rows=self.rows, bbox_info=self.bbox_info, expected_objects=self.expected,
            )

    def test_clipped_frame_edge_uses_inclusive_min_and_max(self) -> None:
        semantic = np.array([
            [1, 1, 0],
            [0, 0, 0],
            [0, 0, 0],
        ], dtype=np.uint32)
        result = validate_capture(
            width=3,
            height=3,
            semantic=semantic,
            semantic_info={"idToLabels": {
                "0": {"class": "BACKGROUND"},
                "1": {"class": "pallet", "object_id": "edge-pallet"},
            }},
            bbox_rows=[{"semanticId": 7, "x_min": 0, "y_min": 0, "x_max": 1, "y_max": 0}],
            bbox_info={"idToLabels": {
                "7": {"class": "pallet", "object_id": "edge-pallet"},
            }},
            expected_objects={"edge-pallet": "pallet"},
        )
        self.assertEqual(result["annotations"][0]["bbox"], [0, 0, 2, 1])
        self.assertEqual(result["object_mask_checks"]["endpoint_convention"], "inclusive_min_max")

    def test_class_only_semantic_label_no_longer_passes_object_export(self) -> None:
        class_only = {"idToLabels": {
            "0": {"class": "BACKGROUND"},
            "1": {"class": "pallet"},
            "2": {"class": "small_load_carrier"},
        }}
        with self.assertRaisesRegex(PerceptionDatasetError, "semantic identity count differs"):
            validate_capture(
                width=5, height=4, semantic=self.semantic, semantic_info=class_only,
                bbox_rows=self.rows, bbox_info=self.bbox_info, expected_objects=self.expected,
            )

    def test_distance_allows_infinite_background_but_requires_finite_object_ranges(self) -> None:
        capture = validate_capture(
            width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
            bbox_rows=self.rows, bbox_info=self.bbox_info, expected_objects=self.expected,
        )
        distance = np.full((4, 5), np.inf, dtype=np.float32)
        distance[self.semantic == 1] = 2.5
        distance[self.semantic == 2] = 3.5
        result = validate_distance_to_camera(
            width=5, height=4, distance=distance, semantic=self.semantic,
            capture_validation=capture,
        )
        self.assertEqual(result["shape"], [4, 5])
        self.assertEqual(result["positive_infinity_background_pixel_count"], 11)
        self.assertEqual(result["objects"]["pallet-01"]["min_m"], 2.5)

    def test_distance_rejects_wrong_shape_and_nonfinite_object_pixels(self) -> None:
        capture = validate_capture(
            width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
            bbox_rows=self.rows, bbox_info=self.bbox_info, expected_objects=self.expected,
        )
        with self.assertRaisesRegex(PerceptionDatasetError, "shape differs"):
            validate_distance_to_camera(
                width=5, height=4, distance=np.ones((4, 5, 1), dtype=np.float32),
                semantic=self.semantic, capture_validation=capture,
            )
        distance = np.ones((4, 5), dtype=np.float32)
        distance[0, 1] = np.inf
        with self.assertRaisesRegex(PerceptionDatasetError, "visible object"):
            validate_distance_to_camera(
                width=5, height=4, distance=distance, semantic=self.semantic,
                capture_validation=capture,
            )

    def test_distance_rejects_invalid_background_ranges_and_accepts_zero_background(self) -> None:
        capture = validate_capture(
            width=5, height=4, semantic=self.semantic, semantic_info=self.semantic_info,
            bbox_rows=self.rows, bbox_info=self.bbox_info, expected_objects=self.expected,
        )
        for invalid in (np.nan, -np.inf, -1.0):
            with self.subTest(invalid=invalid):
                distance = np.ones((4, 5), dtype=np.float32)
                distance[0, 0] = invalid
                with self.assertRaisesRegex(PerceptionDatasetError, "negative ranges"):
                    validate_distance_to_camera(
                        width=5, height=4, distance=distance, semantic=self.semantic,
                        capture_validation=capture,
                    )
        distance[0, 0] = 0
        result = validate_distance_to_camera(
            width=5, height=4, distance=distance, semantic=self.semantic,
            capture_validation=capture,
        )
        self.assertEqual(20, result["finite_pixel_count"])


if __name__ == "__main__":
    unittest.main()
