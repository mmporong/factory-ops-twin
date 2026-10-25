import gzip
from hashlib import sha256
import json
from pathlib import Path
import tempfile
import unittest

from scripts.perception.summarize_detector_pilot import SummaryError, summarize


def digest(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


class SummarizeDetectorPilotTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.analysis = self.root / "analysis-preregistration.json"
        self.analysis.write_text(json.dumps({
            "schema_version": 1, "frozen_before_first_training_result": True,
            "primary_contrast": "SR50 minus R50", "registered_seeds": [1701, 1702, 1703],
            "selection": "no hyperparameter or threshold changes using final fixed test",
            "optional_interval": {"draws": 500, "seed": 7301, "implementation_optional": True,
                                  "no_interval_if_not_validated": True},
        }), encoding="utf-8")

    def tearDown(self):
        self.temporary.cleanup()

    def make_run(self, seed: int, *, config_change: bool = False, bad_prediction_hash: bool = False,
                 status: str = "complete") -> Path:
        run = self.root / f"seed-{seed}"
        prediction_root = run / "evaluation" / "final_fixed_test"
        prediction_root.mkdir(parents=True)
        config = {
            "profile_id": "detector-pilot-v1",
            "seeds": {"pilot": seed, "registered_candidates": [1701, 1702, 1703], "real_subset": 5701},
            "model": {"score_threshold": 0.001 if not config_change else 0.01, "max_detections_per_image": 100},
        }
        config_path = run / "config.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        config_sha = digest(config_path)
        conditions = {
            "S": {"updates": 1024, "initial_checkpoint": "common_coco_head_init"},
            "R50": {"updates": 256, "initial_checkpoint": "common_coco_head_init"},
            "SR50": {"updates": 256, "initial_checkpoint": "S_final"},
        }
        preregistration = {
            "status": "frozen_before_ml_runtime_import", "source_commit": "a" * 40,
            "config": {"path": str(config_path), "sha256": config_sha},
            "seeds": config["seeds"], "conditions": conditions,
        }
        prereg_path = run / "preregistration.json"
        prereg_path.write_text(json.dumps(preregistration), encoding="utf-8")
        prereg_sha = digest(prereg_path)
        result_conditions = {}
        fixed = {}
        for index, condition in enumerate(("S", "R50", "SR50")):
            prediction_path = prediction_root / f"{condition}-predictions.json.gz"
            rows = [{"image_id": 1, "category_id": 1, "bbox": [1, 2, 3, 4], "score": 0.5}]
            payload = {
                "artifact_type": "fixed_test_coco_detection_predictions", "split": "final_fixed_test",
                "condition": condition, "ground_truth_sha256": "c" * 64,
                "config_sha256": config_sha, "preregistration_sha256": prereg_sha,
                "image_ids": [1, 2], "score_threshold": config["model"]["score_threshold"],
                "max_detections_per_image": 100, "rows": rows,
            }
            prediction_path.write_bytes(gzip.compress(json.dumps(payload).encode(), mtime=0))
            prediction_sha = "f" * 64 if bad_prediction_hash and condition == "S" else digest(prediction_path)
            score = 0.20 + index * 0.10 + (seed - 1701) * 0.01
            fixed[condition] = {
                "AP": score, "AP50": score + 0.10, "small_object_AP": score - 0.05,
                "class_AP": {"pallet": score + 0.02, "small_load_carrier": score - 0.02},
                "prediction_artifact": {
                    "path": prediction_path.relative_to(run).as_posix(), "sha256": prediction_sha,
                    "rows": 1, "bytes": prediction_path.stat().st_size,
                    "ground_truth_sha256": "c" * 64, "config_sha256": config_sha,
                    "preregistration_sha256": prereg_sha,
                },
            }
            result_conditions[condition] = {
                "starting_checkpoint": "S-final.pt" if condition == "SR50" else "common-coco-head-init.pt",
                "checkpoint": f"{condition}-final.pt", "checkpoint_sha256": chr(100 + index) * 64,
                "training": {"updates": conditions[condition]["updates"], "wall_seconds": 10.0 + index},
            }
        result = {
            "status": status, "source_commit": "a" * 40, "preregistration_sha256": prereg_sha,
            "initial_checkpoint": {"path": "common-coco-head-init.pt", "sha256": "9" * 64},
            "conditions": result_conditions,
            "evaluation": {"tuning_validation": {key: {
                "AP": value["AP"], "AP50": value["AP50"], "small_object_AP": value["small_object_AP"],
                "class_AP": value["class_AP"],
            } for key, value in fixed.items()}, "final_fixed_test": fixed},
        }
        result_path = run / "result.json"
        result_path.write_text(json.dumps(result), encoding="utf-8")
        return result_path

    def test_writes_compact_report_csv_and_white_svg(self):
        paths = [self.make_run(seed) for seed in (1701, 1702, 1703)]
        report = summarize(paths, self.analysis, self.root / "summary")
        self.assertEqual("complete", report["status"])
        self.assertAlmostEqual(0.10, report["paired_SR50_minus_R50"]["aggregate"]["AP"]["mean"])
        self.assertEqual("not_implemented", report["paired_image_bootstrap_ci"]["status"])
        self.assertTrue((self.root / "summary" / "detector-pilot-summary.csv").is_file())
        svg = (self.root / "summary" / "detector-pilot-summary.svg").read_text(encoding="utf-8")
        self.assertIn('fill="#ffffff"', svg)
        self.assertIn("No bootstrap CI", svg)

    def test_missing_registered_seed_fails_closed(self):
        paths = [self.make_run(seed) for seed in (1701, 1702)]
        with self.assertRaisesRegex(SummaryError, "completed seeds differ"):
            summarize(paths, self.analysis, self.root / "summary")

    def test_prediction_hash_mismatch_fails_closed(self):
        paths = [self.make_run(1701, bad_prediction_hash=True)]
        with self.assertRaisesRegex(SummaryError, "prediction SHA-256 differs"):
            summarize(paths, self.analysis, self.root / "summary")

    def test_config_difference_beyond_pilot_seed_fails_closed(self):
        paths = [self.make_run(1701), self.make_run(1702, config_change=True), self.make_run(1703)]
        with self.assertRaisesRegex(SummaryError, "configs differ"):
            summarize(paths, self.analysis, self.root / "summary")

    def test_incomplete_seed_fails_closed(self):
        paths = [self.make_run(1701, status="failed")]
        with self.assertRaisesRegex(SummaryError, "incomplete"):
            summarize(paths, self.analysis, self.root / "summary")

    def test_changed_analysis_preregistration_fails_closed(self):
        changed = json.loads(self.analysis.read_text(encoding="utf-8"))
        changed["optional_interval"]["draws"] = 499
        self.analysis.write_text(json.dumps(changed), encoding="utf-8")
        with self.assertRaisesRegex(SummaryError, "analysis preregistration contract differs"):
            summarize([self.make_run(1701)], self.analysis, self.root / "summary")


if __name__ == "__main__":
    unittest.main()
