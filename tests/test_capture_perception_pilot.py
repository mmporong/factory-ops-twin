import unittest

from scripts.isaac.capture_perception_pilot import (
    PilotCaptureError,
    canonical_digest,
    validate_manifest_recipe,
)


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


if __name__ == "__main__":
    unittest.main()
