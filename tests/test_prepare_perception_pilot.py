import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.perception.prepare_pilot_manifest import PilotManifestError, build_manifest


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "configs/perception/loco-pallet-rigid-crate-pilot-v1.json"


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


if __name__ == "__main__":
    unittest.main()
