import csv
from hashlib import sha256
import json
from pathlib import Path
import tempfile
import unittest

from scripts.data.prepare_loco_split import prepare_split


def _hash(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


class PrepareLocoSplitTests(unittest.TestCase):
    def test_role_override_and_class_support_use_existing_index(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            metadata_dir = folder / "metadata"
            metadata_dir.mkdir()
            metadata = metadata_dir / "part.json"
            metadata.write_text(json.dumps({
                "images": [{"id": 1, "file_name": "a.jpg", "width": 2, "height": 2}],
                "categories": [{"id": 7, "name": "pallet"}, {"id": 3, "name": "small_load_carrier"}],
                "annotations": [
                    {"id": 1, "image_id": 1, "category_id": 7},
                    {"id": 2, "image_id": 1, "category_id": 3},
                ],
            }), encoding="utf-8")
            index = folder / "index.csv"
            fields = ["metadata_file", "split_role", "image_id", "file_name", "file_sha256", "pixel_sha256", "phash64"]
            with index.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerow({"metadata_file": "part.json", "split_role": "old", "image_id": 1, "file_name": "a.jpg",
                                 "file_sha256": "a" * 64, "pixel_sha256": "b" * 64, "phash64": "0" * 16})
            report = folder / "report.json"
            report.write_text("{}", encoding="utf-8")
            report_payload = {"status": "valid", "outputs": {"image_index_sha256": _hash(index)}}
            report.write_text(json.dumps(report_payload), encoding="utf-8")
            config = folder / "config.json"
            config.write_text(json.dumps({
                "schema_version": 1, "profile_id": "test", "status": "candidate_mapping_pending",
                "role_override_contract": "override", "primary_classes": ["pallet", "small_load_carrier"],
                "inputs": {
                    "rgb_audit_report": {"path": str(report), "sha256": _hash(report)},
                    "rgb_image_index": {"path": str(index), "sha256": _hash(index)},
                    "metadata_dir": str(metadata_dir), "metadata_sha256": {"part.json": _hash(metadata)},
                },
                "split_roles": {"tuning_validation": ["part.json"]},
                "asset_mapping": {"status": "pending"},
                "planned_training_candidates": {"real_label_image_budgets": [10], "seeds": [17], "frozen_for_training": False},
                "leakage_candidate_policy": {"phash64_hamming_threshold": 6},
            }), encoding="utf-8")
            manifest = prepare_split(config, folder / "out")
            support = manifest["roles"]["tuning_validation"]["class_support"]
            self.assertTrue(manifest["roles"]["tuning_validation"]["primary_classes_supported"])
            self.assertEqual(support["pallet"], {"images_with_class": 1, "annotations": 1})
            self.assertEqual(support["small_load_carrier"], {"images_with_class": 1, "annotations": 1})
            with (folder / "out" / "loco-eval-v1-index.csv").open(encoding="utf-8", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["original_split_role"], "old")
            self.assertEqual(row["effective_split_role"], "tuning_validation")


if __name__ == "__main__":
    unittest.main()
