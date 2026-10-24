import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.perception.prepare_pilot_manifest import (
    PilotManifestError,
    _guard_camera_frames,
    _guard_context,
    build_manifest,
    occluder_conflicts,
    source_digest,
)


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/perception/loco-pallet-rigid-crate-pilot-v1.json"
PROFILE_V4 = ROOT / "configs/perception/loco-pallet-rigid-crate-pilot-v4.json"


class PreparePerceptionPilotTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="factoryops-perception-pilot-")
        self.root = Path(self.temporary.name)
        self.cache = self.root / "cache"
        self.profile = json.loads(PROFILE.read_text(encoding="utf-8"))
        for source in self.profile["prepared_inputs"]:
            assets = {}
            for asset_id in source["asset_ids"]:
                contract = self.profile["assets"][asset_id]
                relative = f"prepared/{asset_id.split(':', 1)[1]}.usda"
                path = self.cache / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                payload = asset_id.encode()
                path.write_bytes(payload)
                contract["sha256"] = hashlib.sha256(payload).hexdigest()
                assets[asset_id] = {
                    "sha256": contract["sha256"], "relative_path": relative,
                    "bounds_xyz_m": [-0.5, -0.5, 0, 0.5, 0.5, 0.5],
                    "license": "CC-BY-4.0", "mesh_count": 1,
                    "semantic_state_remaining_count": 0, "removed_physics_typed_prims": [],
                }
            document = {
                "status": source["status"], "training_ready": False, "mapping_approved": False,
                "source_receipt_sha256": "a" * 64, "assets": assets,
            }
            path = self.root / source["path"]
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(document), encoding="utf-8")
            source["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.profile_path = self.root / "profile.json"
        self.profile_path.write_text(json.dumps(self.profile), encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def test_builds_128_unique_scenes_and_512_bounded_frames(self):
        manifest = build_manifest(self.profile_path, self.root, self.cache)
        self.assertEqual((128, 512), (manifest["scene_count"], manifest["frame_count"]))
        self.assertEqual(128, len(set(manifest["scene_seeds"])))
        self.assertEqual(list(range(1, 513)), [row["frame_id"] for row in manifest["frames"]])
        self.assertEqual(64, sum(
            scene["objects"][0]["asset_id"] == "nvidia:pallet_block_a01"
            and any(item["asset_id"] == "nvidia:crate_gray_a20" for item in scene["objects"])
            for scene in manifest["scenes"]
        ))
        self.assertFalse(manifest["scope"]["training_ready"])
        self.assertFalse(manifest["scope"]["mapping_approved"])
        self.assertFalse(manifest["exporter_boundary"]["current_probe_supports_this_manifest"])
        self.assertEqual(
            "inclusive_min_max",
            manifest["capture_contract"]["label_contract"]["bbox_endpoint_convention"],
        )
        self.assertTrue(any(scene["context_primitives"] for scene in manifest["scenes"]))
        self.assertTrue(any(
            item.get("support_object_id")
            for scene in manifest["scenes"]
            for item in scene["objects"]
        ))
        self.assertTrue(any(
            frame["view_id"] == "overview" and frame["target_xyz_m"][0] > 1.0
            for scene in manifest["scenes"] if scene["occlusion_target"] == "edge_crop"
            for frame in manifest["frames"] if frame["scene_index"] == scene["scene_index"]
        ))

    def test_rejects_tampered_prepared_input_and_scope_promotion(self):
        first = self.root / self.profile["prepared_inputs"][0]["path"]
        original = first.read_bytes()
        first.write_text("{}", encoding="utf-8")
        with self.assertRaisesRegex(PilotManifestError, "missing or changed"):
            build_manifest(self.profile_path, self.root, self.cache)

        first.write_bytes(original)
        promoted = json.loads(self.profile_path.read_text())
        promoted["scope_flags"]["training_ready"] = True
        self.profile_path.write_text(json.dumps(promoted), encoding="utf-8")
        with self.assertRaisesRegex(PilotManifestError, "candidate-only"):
            build_manifest(self.profile_path, self.root, self.cache)

    def test_frame_009_occluder_proxy_rejects_original_and_accepts_seeded_resample(self):
        profile = json.loads(PROFILE_V4.read_text(encoding="utf-8"))
        target = {
            "asset_id": "nvidia:crate_gray_a20", "class": "small_load_carrier",
            "object_id": "small_load_carrier-003-02",
            "rotation_z_deg": 9.753641814818714,
            "translation_xyz_m": [-2.176223415256911, 0.18632263742472838, 0.0],
            "uniform_scale": 1.0950239519029386,
        }
        assets = {"nvidia:crate_gray_a20": {"bounds_xyz_m": [
            -0.30583518743515015, -0.20579081773757935, 0.0,
            0.30583518743515015, 0.20579081773757935, 0.09034392349713016,
        ]}}
        frame = {
            "eye_xyz_m": [5.840359046041419, -7.07835881635652, 6.176416810110922],
            "focal_length_mm": 32.30203031732612, "frame_id": 9,
            "target_xyz_m": [0.0522773562445005, 0.0698466528497668, 0.3753347168011776],
        }
        occluder = {
            "center_xyz_m": [-1.4, -0.2, 0.65], "id": "moderate-occluder", "primitive": "cube",
            "rgb": [0.38, 0.42, 0.46], "size_xyz_m": [0.5, 1.4, 1.3],
        }
        original = occluder_conflicts([target], [frame], assets, occluder, [768, 512], 24.0)
        self.assertEqual(1, len(original))
        self.assertTrue(original[0]["full_projection_cover"])
        self.assertTrue(original[0]["center_sight_intersection"])

        first, counts = _guard_context(
            2, 1234, [target], [frame], assets, [occluder], {"moderate-occluder"}, profile,
        )
        second, _ = _guard_context(
            2, 1234, [target], [frame], assets, [occluder], {"moderate-occluder"}, profile,
        )
        self.assertEqual(first, second)
        self.assertNotEqual(occluder["center_xyz_m"], first[0]["center_xyz_m"])
        self.assertEqual(1, counts["resampled_occluders"])
        self.assertEqual([], occluder_conflicts([target], [frame], assets, first[0], [768, 512], 24.0))

    def test_v4_fails_closed_when_a_target_projection_is_outside_the_image(self):
        profile = json.loads(PROFILE_V4.read_text(encoding="utf-8"))
        target = {
            "asset_id": "asset", "class": "pallet", "object_id": "outside",
            "rotation_z_deg": 0.0, "translation_xyz_m": [100.0, 100.0, 0.0], "uniform_scale": 1.0,
        }
        frame = {"eye_xyz_m": [0.0, -7.0, 3.0], "target_xyz_m": [0.0, 0.0, 0.0],
                 "focal_length_mm": 35.0, "frame_id": 1}
        with self.assertRaisesRegex(PilotManifestError, "target projection is outside"):
            _guard_context(0, 1, [target], [frame], {"asset": {"bounds_xyz_m": [-.5, -.5, 0, .5, .5, 1]}},
                           [], set(), profile)

    def test_camera_guard_keeps_partial_clipping_but_recovers_fully_offscreen_target(self):
        profile = json.loads(PROFILE_V4.read_text(encoding="utf-8"))
        target = {
            "asset_id": "asset", "class": "pallet", "object_id": "edge-target",
            "rotation_z_deg": 0.0, "translation_xyz_m": [3.3, 0.0, 0.0], "uniform_scale": 1.0,
        }
        frame = {"eye_xyz_m": [0.0, -4.0, 2.0], "target_xyz_m": [0.0, 0.0, 0.0],
                 "focal_length_mm": 35.0, "frame_id": 1}
        assets = {"asset": {"bounds_xyz_m": [-.5, -.5, 0, .5, .5, 1]}}
        guarded, counts = _guard_camera_frames(
            0, [target], [frame], [[0.0, 0.0, 0.0]], assets, profile, False,
        )
        self.assertEqual(1, counts["adjusted_frames"])
        self.assertGreater(guarded[0]["eye_xyz_m"][1] * -1, 4.0)
        _guard_context(0, 1, [target], guarded, assets, [], set(), profile)

    def test_rejects_crlf_in_pinned_profile_and_generator_sources(self):
        for name in ("profile.json", "generator.py"):
            path = self.root / name
            path.write_bytes(b"first\r\nsecond\r\n")
            with self.subTest(name=name), self.assertRaisesRegex(PilotManifestError, "LF line endings"):
                source_digest(path, name)


if __name__ == "__main__":
    unittest.main()
