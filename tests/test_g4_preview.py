"""Source-report binding tests for the G4 diagnostic preview exporter."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.media.export_g4_preview import _bind_source_report, main


COMMIT = "a" * 40
RUN_ID = "g4-probe-test"


def save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")


class PreviewSourceBindingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "scenes/scene-000/captures/f000/camera/rgb.png"
        self.source.parent.mkdir(parents=True)
        self.source.write_bytes(b"bound-rgb-bytes")
        self.environment: dict[str, object] = {
            "run_id": RUN_ID, "source_commit": COMMIT, "scope": "diagnostic_unreviewed",
        }

    def dataset(self) -> dict[str, object]:
        return {
            "schema_version": 1, "run_id": RUN_ID, "status": "diagnostic_unreviewed",
            "primary_evaluation": False, "training_eligible": False,
            "source": {"commit": COMMIT},
            "artifact_manifest": [{
                "path": self.source.relative_to(self.root).as_posix(),
                "bytes": self.source.stat().st_size,
                "sha256": hashlib.sha256(self.source.read_bytes()).hexdigest(),
            }],
        }

    def prepare_main(self) -> tuple[list[str], Path, bytes]:
        environment = {**self.environment, "source_files": {}, "isaac_image": "isaac:test"}
        provenance = self.root / "provenance"
        save(provenance / "source-environment.json", environment)
        (provenance / "source.tar").write_bytes(b"fixture archive")
        (provenance / "source-commit.raw").write_bytes(b"fixture commit")
        save(self.root / "dataset.json", self.dataset())
        save(self.source.with_name("instance-id.raw-info.json"), {
            "scene_id": "scene-000", "capture_id": "capture-0", "camera_id": "camera",
            "physics_frame": 12, "sim_time_ns": 200_000_000,
        })
        self.source.with_name("camera.raw-params.json").write_bytes(b"camera params")
        review_path = self.root / "review.json"
        save(review_path, {
            "classification": "static_media_visual_review",
            "source_rgb_sha256": hashlib.sha256(self.source.read_bytes()).hexdigest(),
            "render_source_commit": COMMIT, "run_id": RUN_ID, "camera_id": "camera",
            "verdict": "limited_preview_pass", "reviewer": "fixture-reviewer",
            "scope": "unit fixture only", "score_out_of_10": 8,
        })
        output = self.root / "export/preview.png"
        arguments = [
            "export_g4_preview.py", "--diagnostic", str(self.root),
            "--rgb", self.source.relative_to(self.root).as_posix(),
            "--expected-commit", COMMIT, "--output", str(output), "--review", str(review_path),
        ]
        return arguments, output, self.source.read_bytes()

    def test_successful_dataset_and_failed_diagnostic_are_preserved(self) -> None:
        dataset = self.dataset()
        save(self.root / "dataset.json", dataset)
        bound = _bind_source_report(self.root, self.source, COMMIT, self.environment)
        self.assertEqual((bound["kind"], bound["path"]), ("dataset", "dataset.json"))
        self.assertEqual(bound["sha256"], hashlib.sha256((self.root / "dataset.json").read_bytes()).hexdigest())
        manifest = dataset["artifact_manifest"]
        assert isinstance(manifest, list)
        self.assertEqual(bound["artifact_manifest_entry"], manifest[0])

        (self.root / "dataset.json").unlink()
        save(self.root / "failed.json", {"schema_version": 1, "run_id": RUN_ID, "status": "failed"})
        bound = _bind_source_report(self.root, self.source, COMMIT, self.environment)
        self.assertEqual((bound["kind"], bound["path"]), ("failed", "failed.json"))
        self.assertRegex(str(bound["sha256"]), r"^[0-9a-f]{64}$")

    def test_missing_ambiguous_or_wrong_identity_reports_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "exactly one"):
            _bind_source_report(self.root, self.source, COMMIT, self.environment)
        save(self.root / "dataset.json", self.dataset())
        save(self.root / "failed.json", {"run_id": RUN_ID, "status": "failed"})
        with self.assertRaisesRegex(ValueError, "exactly one"):
            _bind_source_report(self.root, self.source, COMMIT, self.environment)
        (self.root / "failed.json").unlink()

        mutations = (
            ("run", lambda value: value.__setitem__("run_id", "other"), "run identity"),
            ("commit", lambda value: value["source"].__setitem__("commit", "b" * 40), "scope or source"),
            ("status", lambda value: value.__setitem__("status", "passed"), "scope or source"),
            ("primary", lambda value: value.__setitem__("primary_evaluation", True), "scope or source"),
            ("training", lambda value: value.__setitem__("training_eligible", True), "scope or source"),
        )
        for label, mutate, message in mutations:
            with self.subTest(label=label):
                value = deepcopy(self.dataset()); mutate(value); save(self.root / "dataset.json", value)
                with self.assertRaisesRegex(ValueError, message):
                    _bind_source_report(self.root, self.source, COMMIT, self.environment)

    def test_selected_rgb_must_have_one_exact_manifest_entry(self) -> None:
        mutations = (
            ("missing", lambda value: value.__setitem__("artifact_manifest", []), "uniquely declared"),
            ("duplicate", lambda value: value["artifact_manifest"].append(deepcopy(value["artifact_manifest"][0])), "uniquely declared"),
            ("sha", lambda value: value["artifact_manifest"][0].__setitem__("sha256", "0" * 64), "bytes or SHA"),
            ("bytes", lambda value: value["artifact_manifest"][0].__setitem__("bytes", 1), "bytes or SHA"),
        )
        for label, mutate, message in mutations:
            with self.subTest(label=label):
                value = self.dataset(); mutate(value); save(self.root / "dataset.json", value)
                with self.assertRaisesRegex(ValueError, message):
                    _bind_source_report(self.root, self.source, COMMIT, self.environment)

    def test_main_exports_successful_diagnostic_to_new_temp_paths(self) -> None:
        arguments, output, original = self.prepare_main()
        with (patch("sys.argv", arguments),
              patch("scripts.media.export_g4_preview.verify_source_archive", return_value={"passed": True}),
              patch("scripts.media.export_g4_preview._png_dimensions", return_value=(1280, 720))):
            main()
        record = json.loads(output.with_name("preview-provenance.json").read_text(encoding="utf-8"))
        self.assertEqual(output.read_bytes(), original)
        self.assertEqual(record["source_report"]["kind"], "dataset")
        self.assertEqual(record["source_dataset_report_sha256"], record["source_report"]["sha256"])
        self.assertNotIn("source_failure_report_sha256", record)

    def test_copy_time_source_or_output_mutation_leaves_no_partial_export(self) -> None:
        def mutate_source(source: str | Path, destination: str | Path):
            Path(source).write_bytes(b"MUTATED")
            return Path(destination).write_bytes(b"MUTATED")

        def mutate_output(source: str | Path, destination: str | Path):
            return Path(destination).write_bytes(b"MUTATED")

        for label, copy_effect in (("source", mutate_source), ("output", mutate_output)):
            with self.subTest(label=label):
                arguments, output, _ = self.prepare_main()
                record = output.with_name("preview-provenance.json")
                with (patch("sys.argv", arguments),
                      patch("scripts.media.export_g4_preview.verify_source_archive", return_value={"passed": True}),
                      patch("scripts.media.export_g4_preview._png_dimensions", return_value=(1280, 720)),
                      patch("scripts.media.export_g4_preview.shutil.copyfile", side_effect=copy_effect)):
                    with self.assertRaisesRegex(ValueError, "changed during export"):
                        main()
                self.assertFalse(output.exists())
                self.assertFalse(record.exists())
                for path in output.parent.glob(".g4-preview-*"):
                    self.fail(f"temporary export artifact remained: {path}")
                if label == "source":
                    self.source.write_bytes(b"bound-rgb-bytes")

    def test_link_time_rgb_or_record_mutation_leaves_no_partial_export(self) -> None:
        real_link = os.link

        def corrupting_link(kind: str):
            def link(source: str | Path, destination: str | Path) -> None:
                real_link(source, destination)
                path = Path(destination)
                if ((kind == "rgb" and path.suffix == ".png")
                        or (kind == "record" and path.name == "preview-provenance.json")):
                    path.write_bytes(b"MUTATED AFTER LINK")
            return link

        for label in ("rgb", "record"):
            with self.subTest(label=label):
                arguments, output, _ = self.prepare_main()
                record = output.with_name("preview-provenance.json")
                with (patch("sys.argv", arguments),
                      patch("scripts.media.export_g4_preview.verify_source_archive", return_value={"passed": True}),
                      patch("scripts.media.export_g4_preview._png_dimensions", return_value=(1280, 720)),
                      patch("scripts.media.export_g4_preview.os.link", side_effect=corrupting_link(label))):
                    with self.assertRaisesRegex(ValueError, "changed during publication"):
                        main()
                self.assertFalse(output.exists())
                self.assertFalse(record.exists())
                self.assertEqual(self.source.read_bytes(), b"bound-rgb-bytes")
                for path in output.parent.glob(".g4-preview-*"):
                    self.fail(f"temporary export artifact remained: {path}")


if __name__ == "__main__":
    unittest.main()
