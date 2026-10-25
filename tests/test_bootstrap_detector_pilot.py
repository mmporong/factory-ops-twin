import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from scripts.perception.bootstrap_detector_pilot import (
    build_evaluator,
    cached_sample_ap,
    duplicate_sample,
    equivalence_fixture,
    fresh_ap,
    run_bootstrap,
    runtime_imports,
)


HAS_RUNTIME = importlib.util.find_spec("numpy") is not None and importlib.util.find_spec("pycocotools") is not None


def fixture():
    ground_truth = {
        "info": {}, "licenses": [],
        "images": [{"id": 1, "width": 64, "height": 64, "file_name": "one.png"},
                   {"id": 2, "width": 64, "height": 64, "file_name": "two.png"}],
        "categories": [{"id": 1, "name": "pallet"}, {"id": 2, "name": "small_load_carrier"}],
        "annotations": [
            {"id": 1, "image_id": 1, "category_id": 1, "bbox": [4, 4, 20, 20], "area": 400, "iscrowd": 0},
            {"id": 2, "image_id": 1, "category_id": 2, "bbox": [30, 5, 15, 15], "area": 225, "iscrowd": 1},
            {"id": 3, "image_id": 2, "category_id": 1, "bbox": [8, 8, 18, 18], "area": 324, "iscrowd": 0},
        ],
    }
    predictions = [
        {"image_id": 1, "category_id": 1, "bbox": [4, 4, 20, 20], "score": 0.5},
        {"image_id": 1, "category_id": 2, "bbox": [30, 5, 15, 15], "score": 0.5},
        {"image_id": 2, "category_id": 1, "bbox": [9, 9, 18, 18], "score": 0.5},
    ]
    return ground_truth, predictions


class BootstrapBlockedResultTests(unittest.TestCase):
    def test_invalid_analysis_writes_blocked_result_without_ci(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            analysis = root / "analysis.json"
            summary = root / "summary.json"
            analysis.write_text("{}", encoding="utf-8")
            summary.write_text("{}", encoding="utf-8")
            output = root / "out"
            with self.assertRaisesRegex(Exception, "analysis preregistration differs"):
                run_bootstrap(summary, analysis, output, 30)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(("blocked", False), (report["status"], report["confidence_interval_published"]))


@unittest.skipUnless(HAS_RUNTIME, "pycocotools/numpy runtime is not installed")
class BootstrapDetectorPilotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = runtime_imports()

    def test_identity_sample_matches_fresh_cocoeval(self):
        ground_truth, predictions = fixture()
        evaluator, base, _ = build_evaluator(self.runtime, ground_truth, predictions)
        cached = cached_sample_ap(self.runtime, evaluator, base, [0, 1])
        fresh = fresh_ap(self.runtime, ground_truth, predictions)
        self.assertAlmostEqual(fresh, cached, places=12)

    def test_duplicate_image_sample_matches_fresh_cocoeval_with_ties_and_ignore(self):
        ground_truth, predictions = fixture()
        evaluator, base, _ = build_evaluator(self.runtime, ground_truth, predictions)
        sample = [0, 0]
        cached = cached_sample_ap(self.runtime, evaluator, base, sample)
        duplicated_gt, duplicated_predictions = duplicate_sample(ground_truth, predictions, sample)
        fresh = fresh_ap(self.runtime, duplicated_gt, duplicated_predictions)
        self.assertAlmostEqual(fresh, cached, places=12)
        evidence = equivalence_fixture(self.runtime)
        self.assertEqual(evidence["duplicate_fresh"], evidence["duplicate_cached"])


if __name__ == "__main__":
    unittest.main()
