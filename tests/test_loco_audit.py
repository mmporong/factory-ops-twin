import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.loco_audit import audit_annotations


def _write(path: Path, *, image_id: int, file_name: str, annotation_id: int = 1,
           bbox: list[float] | None = None, category_id: int = 1) -> None:
    payload = {
        "images": [{"id": image_id, "file_name": file_name, "width": 100, "height": 80}],
        "categories": [{"id": 1, "name": "pallet"}],
        "annotations": [{
            "id": annotation_id, "image_id": image_id, "category_id": category_id,
            "bbox": bbox if bbox is not None else [10, 12, 20, 15], "area": 300,
        }],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


class LocoAuditTests(unittest.TestCase):
    def test_valid_coco_partition_reports_counts_dimensions_and_classes(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "sample.json"
            _write(path, image_id=7, file_name="frame.png")
            report = audit_annotations([path])
            self.assertEqual(report["status"], "valid")
            self.assertEqual(report["totals"]["images"], 1)
            self.assertEqual(report["published_reference"], {
                "total_images": 37_988,
                "manually_annotated_images": 5_593,
                "annotations": 152_421,
            })
            self.assertEqual(
                report["published_reference_comparison"]["manually_annotated_image_count_delta"],
                1 - 5_593,
            )
            partition = report["partitions"][0]
            self.assertEqual(partition["class_counts"], [{"category_id": 1, "name": "pallet", "annotations": 1}])
            self.assertEqual(partition["image_dimensions"]["unique_width_height_pairs"], 1)
            self.assertEqual(partition["bbox"]["outside_image_count"], 0)

    def test_overlap_unknown_reference_and_invalid_box_are_reported(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            left = folder / "loco-sub2-v1-train.json"
            right = folder / "loco-sub1-v1-val.json"
            _write(left, image_id=1, file_name="shared.png")
            _write(right, image_id=1, file_name="shared.png", bbox=[0, 0, -2, 3], category_id=99)
            payload = json.loads(right.read_text(encoding="utf-8"))
            payload["annotations"][0]["image_id"] = 404
            right.write_text(json.dumps(payload), encoding="utf-8")
            report = audit_annotations([left, right])
            self.assertEqual(report["status"], "invalid")
            self.assertEqual(report["candidate_split_leakage"]["status"], "detected")
            bad = next(row for row in report["partitions"] if row["file"] == right.name)
            self.assertEqual(bad["references"]["unknown_image_count"], 1)
            self.assertEqual(bad["references"]["unknown_category_count"], 1)
            self.assertEqual(bad["bbox"]["invalid_count"], 1)


if __name__ == "__main__":
    unittest.main()
