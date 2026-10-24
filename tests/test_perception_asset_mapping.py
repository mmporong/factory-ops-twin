from hashlib import sha256
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.perception_asset_mapping import evaluate_asset_mapping


class PerceptionAssetMappingTests(unittest.TestCase):
    def test_rejects_semantic_shortcuts_and_keeps_candidates_unapproved(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            geometry = folder / "geometry.json"
            geometry.write_text(json.dumps({"instances": [
                {"stable_id": "FG_HIGH", "prim_path": "/World/FG_HIGH", "semantic_class": "pallet", "asset": "esd_tray_loadport"},
                {"stable_id": "MAG", "prim_path": "/World/MAG", "semantic_class": "tote", "asset": "esd_tray_loadport/TrayMagazine"},
                {"stable_id": "TOTE", "prim_path": "/World/TOTE", "semantic_class": "tote", "asset": "finished_goods_tote"},
                {"stable_id": "UNKNOWN", "prim_path": "/World/UNKNOWN", "semantic_class": "tote", "asset": "generic_box"},
            ]}), encoding="utf-8")
            geometry_hash = sha256(geometry.read_bytes()).hexdigest()
            config = folder / "config.json"
            config.write_text(json.dumps({
                "schema_version": 1, "profile_id": "test", "status": "candidate_mapping_pending",
                "source_geometry_manifest": {"path": str(geometry), "sha256": geometry_hash},
                "training_readiness_gates": {
                    "approved_asset_required_for_each_primary_class": True,
                    "root_leaf_semantic_boundary_validation": None,
                    "bbox_export_validation": None,
                },
                "target_classes": {
                    "pallet": {
                        "source_semantic_classes": ["pallet"], "approved_asset_ids": [],
                        "candidate_asset_ids": ["finished_goods_pallet"],
                        "rejected_asset_prefixes": {"esd_tray_loadport": "not a pallet"},
                    },
                    "small_load_carrier": {
                        "source_semantic_classes": ["tote"], "approved_asset_ids": [],
                        "candidate_asset_ids": ["finished_goods_tote"],
                        "rejected_asset_prefixes": {"esd_tray_loadport": "not a KLT"},
                    },
                },
                "decision_policy": {
                    "unknown_assets": "reject", "candidate_assets_are_training_labels": False,
                    "semantic_class_alone_is_sufficient": False, "automatic_class_mapping": False,
                },
            }), encoding="utf-8")
            report = evaluate_asset_mapping(config, geometry)
            self.assertFalse(report["training_ready"])
            self.assertEqual(report["class_summary"]["pallet"]["rejected_instances"], 1)
            self.assertEqual(report["class_summary"]["small_load_carrier"]["candidate_instances"], 1)
            self.assertEqual(report["class_summary"]["small_load_carrier"]["rejected_instances"], 2)
            self.assertFalse(report["validation_gates"]["root_leaf_semantic_boundary_validation"])
            self.assertFalse(report["validation_gates"]["bbox_export_validation"])

    def test_manifest_hash_must_match_profile(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            geometry = folder / "geometry.json"
            geometry.write_text('{"instances": []}', encoding="utf-8")
            config = folder / "config.json"
            config.write_text(json.dumps({
                "schema_version": 1, "profile_id": "test",
                "source_geometry_manifest": {"sha256": "0" * 64},
                "training_readiness_gates": {},
                "target_classes": {
                    "pallet": {"source_semantic_classes": ["pallet"], "approved_asset_ids": [], "candidate_asset_ids": [], "rejected_asset_prefixes": {}},
                    "small_load_carrier": {"source_semantic_classes": ["tote"], "approved_asset_ids": [], "candidate_asset_ids": [], "rejected_asset_prefixes": {}},
                },
                "decision_policy": {},
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                evaluate_asset_mapping(config, geometry)


if __name__ == "__main__":
    unittest.main()
