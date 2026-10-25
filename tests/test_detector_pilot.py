import gzip
import json
from hashlib import sha256
from pathlib import Path
import tempfile
import unittest

from scripts.perception.run_detector_pilot import (
    Budget,
    DeadlineExceeded,
    PilotError,
    build_real_registry,
    category_mapping,
    validate_coco,
    validate_condition_symmetry,
    validate_synthetic_release,
    verify_disjoint_sets,
    require_finite_loss,
    write_prediction_artifact,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "configs/perception/detector-pilot-v1.json").read_text(encoding="utf-8"))


def coco(file_name: str, category_names=("pallet", "small load carrier"), image_count=1):
    return {
        "images": [{"id": index, "file_name": f"{file_name}-{index}.png", "width": 32, "height": 24}
                   for index in range(1, image_count + 1)],
        "categories": [{"id": 7, "name": category_names[0]}, {"id": 9, "name": category_names[1]}],
        "annotations": [],
    }


def valid_coco(file_name: str, image_count: int):
    payload = coco(file_name, image_count=image_count)
    payload["annotations"] = []
    for index in range(1, image_count + 1):
        payload["annotations"].extend([
            {"id": 2 * index - 1, "image_id": index, "category_id": 7, "bbox": [1, 2, 3, 4]},
            {"id": 2 * index, "image_id": index, "category_id": 9, "bbox": [5, 6, 7, 8]},
        ])
    return payload


class DetectorPilotTests(unittest.TestCase):
    def test_unapproved_synthetic_release_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "release.json"
            path.write_text(json.dumps({"schema_version": 1, "release_type": "approved_detector_training_dataset",
                                        "status": "candidate", "training_ready": False,
                                        "mapping_approved": False}), encoding="utf-8")
            with self.assertRaisesRegex(PilotError, "not approved"):
                validate_synthetic_release(path)

    def test_synthetic_inventory_must_match_coco_ids_and_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            images = root / "images"
            images.mkdir()
            coco_path = root / "training.json"
            coco_payload = {"images": [{"id": index, "file_name": f"{index}.png"}
                                        for index in range(1, 513)]}
            coco_path.write_text(json.dumps(coco_payload), encoding="utf-8")
            inventory_path = root / "inventory.json"
            inventory_path.write_text(json.dumps({
                "schema_version": 1,
                "files": [{"image_id": index, "path": "wrong.png" if index == 512 else f"{index}.png",
                           "sha256": "a" * 64, "bytes": 1} for index in range(1, 513)],
            }), encoding="utf-8")
            bounds = root / "bounds.json"
            bounds.write_text("{}", encoding="utf-8")
            digest = lambda path: sha256(path.read_bytes()).hexdigest()
            release = root / "release.json"
            release.write_text(json.dumps({
                "schema_version": 1, "release_type": "approved_detector_training_dataset",
                "scope": "bounded_pallet_and_rigid_crate_detector_pilot", "status": "approved",
                "training_ready": True, "mapping_approved": True, "image_count": 512,
                "producer_source_commit": "a" * 40,
                "category_mapping": {"pallet": 1, "small_load_carrier": 2},
                "coco": {"path": coco_path.name, "sha256": digest(coco_path)},
                "images": {"root": images.name, "inventory": {
                    "path": inventory_path.name, "sha256": digest(inventory_path)}},
                "bounds_source": {"path": bounds.name, "sha256": digest(bounds)},
            }), encoding="utf-8")
            with self.assertRaisesRegex(PilotError, "does not match COCO"):
                validate_synthetic_release(release)

    def test_split_leakage_is_rejected(self):
        with self.assertRaisesRegex(PilotError, "split leakage"):
            verify_disjoint_sets({"train": {"same.png"}, "final_fixed_test": {"same.png"}})

    def test_r50_and_sr50_have_equal_real_budget(self):
        validate_condition_symmetry(CONFIG)
        changed = json.loads(json.dumps(CONFIG))
        changed["conditions"]["SR50"]["updates"] = 255
        with self.assertRaisesRegex(PilotError, "differ in updates"):
            validate_condition_symmetry(changed)

    def test_coco_boxes_and_category_mapping_are_validated(self):
        payload = valid_coco("sample", 2)
        self.assertEqual({7: 1, 9: 2}, validate_coco(payload, CONFIG["category_aliases"]))
        payload["annotations"][0]["bbox"][2] = 0
        with self.assertRaisesRegex(PilotError, "bbox"):
            validate_coco(payload, CONFIG["category_aliases"])
        with self.assertRaises(PilotError):
            category_mapping(valid_coco("x", 1), {"pallet": ["pallet"], "small_load_carrier": ["absent"]})

    def test_real_registry_uses_fixed_role_budgets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for role in CONFIG["real_split"].values():
                if not isinstance(role, dict):
                    continue
                for filename, count in role.items():
                    (root / filename).write_text(json.dumps(valid_coco(filename, count)), encoding="utf-8")
            registry, _ = build_real_registry(root, CONFIG)
            self.assertEqual(50, sum(row["image_count"] for row in registry["train"]))
            self.assertEqual(200, sum(row["image_count"] for row in registry["tuning_validation"]))
            self.assertEqual(1075, sum(row["image_count"] for row in registry["final_fixed_test"]))

    def test_budget_checks_exact_wall_and_deadline_boundaries(self):
        exhausted = Budget(10.0, 5.0, None, monotonic=lambda: 15.0)
        with self.assertRaises(DeadlineExceeded):
            exhausted.check("train")

    def test_nonfinite_loss_is_rejected_before_completion(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value), self.assertRaisesRegex(PilotError, "non-finite loss"):
                require_finite_loss(value, "R50", 17)
        require_finite_loss(1.25, "R50", 17)

    def test_fixed_test_predictions_are_gzipped_and_bound(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "S-predictions.json.gz"
            ground_truth = {"images": [{"id": 4}, {"id": 2}], "annotations": [], "categories": []}
            artifact = write_prediction_artifact(path, ground_truth, [
                {"image_id": 2, "category_id": 1, "bbox": [1, 2, 3, 4], "score": 0.5},
            ], {"condition": "S", "config_sha256": "a" * 64, "preregistration_sha256": "b" * 64,
                "score_threshold": 0.001, "max_detections_per_image": 100})
            payload = json.loads(gzip.decompress(path.read_bytes()))
            self.assertEqual([2, 4], payload["image_ids"])
            self.assertEqual("b" * 64, payload["preregistration_sha256"])
            self.assertEqual((1, path.stat().st_size), (artifact["rows"], artifact["bytes"]))


if __name__ == "__main__":
    unittest.main()
