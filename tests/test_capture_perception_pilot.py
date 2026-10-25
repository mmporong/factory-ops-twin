import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from scripts.isaac.capture_perception_pilot import (
    PilotCaptureError,
    canonical_digest,
    failure_document,
    frame_camera_candidates,
    preserve_failure_diagnostics,
    rejection_record,
    reserve_additional_capture_attempt,
    retryable_visibility_error,
    validate_manifest_recipe,
)
from factory_ops.perception_dataset import PerceptionDatasetError


class CapturePerceptionPilotTests(unittest.TestCase):
    def make_manifest(self):
        assets = {
            "nvidia:pallet_block_a01": {
                "class": "pallet", "sha256": "a" * 64,
                "relative_path": "prepared/pallet.usda", "bounds_xyz_m": [-0.5, -0.5, 0, 0.5, 0.5, 0.2],
                "license": "CC-BY-4.0", "scale_range": [0.95, 1.05],
            },
            "nvidia:crate_gray_a20": {
                "class": "small_load_carrier", "sha256": "b" * 64,
                "relative_path": "prepared/gray.usda", "bounds_xyz_m": [-0.3, -0.2, 0, 0.3, 0.2, 0.1],
                "license": "CC-BY-4.0", "scale_range": [0.9, 1.1],
            },
            "nvidia:crate_blue_a02": {
                "class": "small_load_carrier", "sha256": "c" * 64,
                "relative_path": "prepared/blue.usda", "bounds_xyz_m": [-0.3, -0.2, 0, 0.3, 0.2, 0.3],
                "license": "CC-BY-4.0", "scale_range": [0.9, 1.1],
            },
        }
        frames = []
        scenes = []
        seeds = list(range(1000, 1128))
        for scene_index, seed in enumerate(seeds):
            frame_ids = []
            for view_index, view_id in enumerate(("overview", "aisle_low", "side", "oblique")):
                frame_id = scene_index * 4 + view_index + 1
                frame_ids.append(frame_id)
                frames.append({
                    "frame_id": frame_id, "scene_index": scene_index, "view_index": view_index,
                    "view_id": view_id, "eye_xyz_m": [5.0, -5.0, 4.0],
                    "target_xyz_m": [0.0, 0.0, 0.3], "focal_length_mm": 35.0,
                })
            crate_asset = "nvidia:crate_gray_a20" if scene_index % 2 == 0 else "nvidia:crate_blue_a02"
            scenes.append({
                "scene_index": scene_index, "seed": seed, "layout_variant": "open_staging",
                "background_variant": "neutral_floor", "occlusion_target": "clear", "stacking_mode": "none",
                "floor_rgb": [0.3, 0.3, 0.3], "dome_intensity": 450.0, "key_intensity": 1800.0,
                "context_primitives": [], "context_semantics": "unlabelled",
                "objects": [
                    {
                        "object_id": f"pallet-{scene_index:03d}", "class": "pallet",
                        "asset_id": "nvidia:pallet_block_a01", "translation_xyz_m": [-1.0, 0.0, 0.0],
                        "uniform_scale": 1.0, "rotation_z_deg": 0.0,
                    },
                    {
                        "object_id": f"crate-{scene_index:03d}", "class": "small_load_carrier",
                        "asset_id": crate_asset, "translation_xyz_m": [1.0, 0.0, 0.0],
                        "uniform_scale": 1.0, "rotation_z_deg": 0.0,
                    },
                ],
                "frame_ids": frame_ids,
            })
        manifest = {
            "schema_version": 1,
            "status": "pilot_manifest_prepared_exporter_extension_required",
            "profile_id": "loco-pallet-rigid-crate-pilot-v1",
            "profile_sha256": "d" * 64,
            "generator": {"path": "scripts/perception/prepare_pilot_manifest.py", "sha256": "e" * 64},
            "prepared_inputs": [],
            "cache_root": "E:/cache",
            "assets": assets,
            "scene_count": 128,
            "frame_count": 512,
            "scene_seeds": seeds,
            "scenes": scenes,
            "frames": frames,
            "capture_contract": {
                "resolution": [768, 512],
                "modalities": ["rgb", "semantic_segmentation", "bounding_box_2d_tight", "distance_to_camera"],
                "semantic_types": ["class", "object_id"],
                "label_contract": {
                    "root_semantics_only": True,
                    "bbox_endpoint_convention": "inclusive_min_max",
                    "per_object_bbox_equals_same_object_id_semantic_tight_extent": True,
                    "reject_fully_hidden_objects": True,
                    "clipped_frame_edges_allowed": True,
                },
            },
            "scope": {
                "training_ready": False, "mapping_approved": False, "model_training_performed": False,
                "Cosmos_used": False, "heldout_LOCO_RGB_read": False, "GPU_capture_performed": False,
                "existing_probe_repeated": False, "baseline_limit": "test",
            },
            "exporter_boundary": {
                "current_probe_supports_this_manifest": False,
                "missing_support": [],
                "required_validation": "per-object",
            },
        }
        manifest["recipe_sha256"] = canonical_digest({"scenes": scenes, "frames": frames, "assets": assets})
        return manifest

    def test_accepts_exact_128_scene_512_frame_recipe(self):
        validate_manifest_recipe(self.make_manifest())

    def test_rejects_unconsumed_recipe_field_and_recipe_tampering(self):
        manifest = self.make_manifest()
        manifest["scenes"][0]["unused_recipe"] = "must not be ignored"
        with self.assertRaisesRegex(PilotCaptureError, "scene recipe fields differ"):
            validate_manifest_recipe(manifest)
        manifest = self.make_manifest()
        manifest["frames"][0]["focal_length_mm"] = 38.0
        with self.assertRaisesRegex(PilotCaptureError, "recipe hash differs"):
            validate_manifest_recipe(manifest)

    def test_rejects_scope_promotion_and_scale_outside_pinned_asset_range(self):
        manifest = self.make_manifest()
        manifest["scope"]["training_ready"] = True
        with self.assertRaisesRegex(PilotCaptureError, "candidate-only scope"):
            validate_manifest_recipe(manifest)
        manifest = self.make_manifest()
        manifest["scenes"][0]["objects"][0]["uniform_scale"] = 1.2
        manifest["recipe_sha256"] = canonical_digest({
            "scenes": manifest["scenes"], "frames": manifest["frames"], "assets": manifest["assets"]
        })
        with self.assertRaisesRegex(PilotCaptureError, "scale differs"):
            validate_manifest_recipe(manifest)

    def test_preserves_one_invalid_frame_without_admitting_it_as_capture(self):
        with tempfile.TemporaryDirectory(prefix="factoryops-pilot-diagnostic-") as temporary:
            output = Path(temporary)
            result = preserve_failure_diagnostics(
                output=output,
                scene={"scene_index": 2, "seed": 17, "objects": []},
                frame={"frame_id": 9, "view_index": 0, "view_id": "overview"},
                expected_objects={"crate-003-02": "small_load_carrier"},
                rgb=np.zeros((4, 5, 4), dtype=np.uint8),
                semantic=np.zeros((4, 5), dtype=np.uint32),
                semantic_info={"idToLabels": {"0": {"class": "BACKGROUND"}}},
                bbox_rows=[],
                bbox_info={"idToLabels": {}},
                distance=np.ones((4, 5), dtype=np.float32),
                camera={"width": 5, "height": 4, "raw_camera_params": {}},
                source_pin={"source_commit": "a" * 40, "manifest_sha256": "b" * 64},
                error=ValueError("missing object identity"),
            )
            root = output / "diagnostics" / "frame-009"
            diagnostic = json.loads((root / "diagnostic.json").read_text(encoding="utf-8"))
            self.assertEqual(result["path"], "diagnostics/frame-009")
            self.assertFalse(result["counts_as_valid_capture"])
            self.assertFalse(diagnostic["counts_toward_coco"])
            self.assertEqual(diagnostic["source_pin"]["manifest_sha256"], "b" * 64)
            self.assertEqual(np.load(root / "rgb.raw.npy", allow_pickle=False).shape, (4, 5, 4))
            self.assertEqual(np.load(root / "semantic.raw.npy", allow_pickle=False).dtype, np.uint32)
            self.assertEqual(np.load(root / "distance-to-camera.raw.npy", allow_pickle=False).dtype, np.float32)
            self.assertEqual(
                json.loads((root / "expected-objects.json").read_text(encoding="utf-8")),
                {"crate-003-02": "small_load_carrier"},
            )
            failure = failure_document(ValueError("missing object identity"), [], result, 8)
            self.assertEqual(failure["scene_index"], 2)
            self.assertEqual(failure["frame_id"], 9)
            self.assertEqual(failure["view_id"], "overview")
            self.assertEqual(failure["completed_valid_frames"], 8)
            self.assertFalse(failure["diagnostics"]["counts_as_valid_capture"])

    def test_v6_candidate_recipe_is_strict_and_deterministic(self):
        manifest = self.make_manifest()
        manifest["profile_id"] = "loco-pallet-rigid-crate-pilot-v6"
        manifest["capture_contract"]["valid_view_resampling"] = {
            "candidate_count_per_frame": 3,
            "maximum_additional_capture_attempts": 128,
            "candidate_derivation": "ordered_camera_guard_attempts_v6",
            "retry_visibility_validation_only": True,
            "preserve_full_diagnostics_for_first_rejections": 3,
            "accepted_captures_only_in_dataset_counts": True,
        }
        for frame in manifest["frames"]:
            candidates = []
            for index in range(3):
                candidate = {
                    "candidate_index": index,
                    "eye_xyz_m": [frame["eye_xyz_m"][0], frame["eye_xyz_m"][1] - index * 0.2,
                                  frame["eye_xyz_m"][2] + index * 0.1],
                    "target_xyz_m": list(frame["target_xyz_m"]),
                    "focal_length_mm": frame["focal_length_mm"],
                }
                candidate["camera_recipe_sha256"] = canonical_digest(candidate)
                candidates.append(candidate)
            frame["camera_candidates"] = candidates
        manifest["recipe_sha256"] = canonical_digest({
            "scenes": manifest["scenes"], "frames": manifest["frames"], "assets": manifest["assets"]
        })
        validate_manifest_recipe(manifest)
        self.assertEqual(frame_camera_candidates(manifest["frames"][0]), manifest["frames"][0]["camera_candidates"])

        manifest["frames"][0]["camera_candidates"][1]["unexpected"] = True
        manifest["recipe_sha256"] = canonical_digest({
            "scenes": manifest["scenes"], "frames": manifest["frames"], "assets": manifest["assets"]
        })
        with self.assertRaisesRegex(PilotCaptureError, "camera candidate fields differ"):
            validate_manifest_recipe(manifest)

    def test_only_visibility_loss_is_retryable_and_rejections_never_count(self):
        retryable = PerceptionDatasetError("semantic identity count differs for object: crate-003: 0")
        duplicate = PerceptionDatasetError("semantic identity count differs for object: crate-003: 2")
        wrong_class = PerceptionDatasetError(
            "semantic class/object binding differs from provenance: crate-003 -> pallet"
        )
        depth = PerceptionDatasetError("distance_to_camera contains no finite positive ranges")
        self.assertTrue(retryable_visibility_error(retryable))
        self.assertFalse(retryable_visibility_error(duplicate))
        self.assertFalse(retryable_visibility_error(wrong_class))
        self.assertFalse(retryable_visibility_error(depth))
        candidate = {
            "candidate_index": 0, "camera_recipe_sha256": "a" * 64,
            "eye_xyz_m": [0, 0, 1], "target_xyz_m": [0, 0, 0], "focal_length_mm": 35,
        }
        record = rejection_record({"frame_id": 9}, candidate, retryable)
        self.assertFalse(record["counts_as_valid_capture"])
        self.assertFalse(record["counts_toward_coco"])

    def test_additional_attempt_limit_fails_closed_at_128(self):
        self.assertEqual(128, reserve_additional_capture_attempt(127, 128))
        with self.assertRaisesRegex(PilotCaptureError, "exceeded 128"):
            reserve_additional_capture_attempt(128, 128)


if __name__ == "__main__":
    unittest.main()
