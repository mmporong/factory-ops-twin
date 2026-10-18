from __future__ import annotations

import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest

from factory_ops.g3_archive_replay import _regular, load_archive_record, materialize_archive_slot


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class G3ArchiveReplayTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.run_label = "g3-m-normal-fifo-001"
        self.run_id = "a" * 32
        self.resources = self.root / "resources" / self.run_label
        self.resources.mkdir(parents=True)
        self.target = self.root / "results" / self.run_label
        self.target.parent.mkdir()
        self.session = self.root / "source-snapshots" / self.run_label / "runs" / f"{self.run_label}-session"
        self.session.mkdir(parents=True)
        (self.session / "session.json").write_text('{"stable":true}\n', encoding="utf-8")
        self.trial_path = self.root / "trial.json"
        self.trial_path.write_text('{"trial":true}\n', encoding="utf-8")
        self.calibration_path = self.root / "calibration.json"
        self.calibration_path.write_text('{"calibration":true}\n', encoding="utf-8")
        pilots = {}
        for condition in ("normal", "closure", "burst"):
            for policy in ("fifo", "eta"):
                key = f"{condition}:{policy}"
                pilot_root = self.root / "pilots" / f"{condition}-{policy}"
                pilot_session = self.root / "pilot-sessions" / f"{condition}-{policy}"
                pilot_root.mkdir(parents=True)
                pilot_session.mkdir(parents=True)
                report = pilot_root / "g3-evidence.json"
                mission = pilot_root / "mission-result.json"
                report.write_text('{"pilot":true}\n', encoding="utf-8")
                mission.write_text('{"pilot":true}\n', encoding="utf-8")
                (pilot_session / "session.json").write_text('{"pilot":true}\n', encoding="utf-8")
                pilots[key] = {"report_path": str(report), "mission_path": str(mission),
                               "session_root": str(pilot_session), "artifact_root": str(pilot_root)}
        resource_root = self.root / "resource-evidence"
        resource_root.mkdir()
        billing = resource_root / "billing.json"
        billing.write_text('{"billing":true}\n', encoding="utf-8")
        supervisor = resource_root / "supervisor.json"
        supervisor.write_text(json.dumps({"runs": [], "billing": {
            "path": "billing.json", "sha256": digest(billing)}}), encoding="utf-8")
        self.primary = self.root / "primary-inputs.json"
        self.primary.write_text(json.dumps({
            "trial_path": str(self.trial_path), "calibration_path": str(self.calibration_path),
            "pilot_slots": pilots,
            "resource_input": {"evidence_root": str(resource_root),
                               "supervisor_path": "supervisor.json",
                               "supervisor_sha256": digest(supervisor),
                               "requested_primary_runs": 60, "reserve_fraction": 0.2},
            "expected_source_commit": "b" * 40,
        }), encoding="utf-8")
        self.files = {
            "g3-evidence.json": json.dumps({"run_id": self.run_id,
                "trial_id": "normal-fifo-seed-0", "condition": "normal", "policy": "fifo",
                "seed": 0, "trial_input_sha256": "2" * 64,
                "workload_sha256": "3" * 64}).encode(),
            "mission-result.json": json.dumps({"run_id": self.run_id,
                "trial_id": "normal-fifo-seed-0", "trial_input_sha256": "2" * 64,
                "workload_sha256": "3" * 64}).encode(),
            "rosbag/data.db3": b"bag-bytes",
        }
        self.manifest = self.resources / "output-manifest.json"
        rows = [{"path": name, "bytes": len(value),
                 "sha256": hashlib.sha256(value).hexdigest()}
                for name, value in sorted(self.files.items())]
        self.manifest.write_text(json.dumps({"schema_version": 1, "files": rows}, sort_keys=True),
                                 encoding="utf-8")
        self.archive = self.resources / "output.tar.gz"
        with tarfile.open(self.archive, "w:gz", compresslevel=3) as packed:
            for name, value in sorted(self.files.items()):
                info = tarfile.TarInfo(name)
                info.size = len(value)
                packed.addfile(info, io.BytesIO(value))
        self.trial = {
            "trial_id": "normal-fifo-seed-0", "condition": "normal", "policy": "fifo", "seed": 0,
            "source_config_sha256": "1" * 64, "trial_input_sha256": "2" * 64,
            "workload_sha256": "3" * 64,
        }
        self.record = self.resources / "archive-record.json"
        bindings = {"source_commit": "b" * 40, "experiment_config_sha256": "1" * 64,
                    "trial_input_sha256": "2" * 64, "workload_sha256": "3" * 64,
                    "session_sha256": digest(self.session / "session.json"),
                    "trial_json_sha256": digest(self.trial_path),
                    "gate_sha256": digest(self.calibration_path),
                    "primary_inputs_sha256": digest(self.primary)}
        identity = {"profile": "primary", "trial_id": self.trial["trial_id"],
                    "condition": "normal", "policy": "fifo", "seed": 0}
        receipt = self.resources / "attempt-receipt.json"
        receipt.write_text(json.dumps({"run_id": self.run_id, "identity": identity,
                                       "bindings": bindings, "references": {}}), encoding="utf-8")
        value = {
            "schema_version": 2, "classification": "g3_closed_output_backup_not_validation",
            "run_id": self.run_id, "run_label": self.run_label,
            "identity": identity, "bindings": bindings,
            "original_output_root": str(self.target),
            "literal_paths": {
                "artifact_root": str(self.target), "session_root": str(self.session),
                "primary_inputs_path": str(self.primary), "evidence_root": str(self.root),
                "attempt_receipt_path": str(self.resources / "attempt-receipt.json"),
                "archive_path": str(self.archive), "manifest_path": str(self.manifest),
                "archive_record_path": str(self.record),
            },
            "started_at_utc": "2026-01-01T00:00:00Z",
            "workers_finished_at_utc": "2026-01-01T00:01:00Z",
            "validation_finished_at_utc": "2026-01-01T00:02:00Z",
            "archive_finished_at_utc": "2026-01-01T00:03:00Z",
            "file_count": len(rows), "raw_bytes": sum(row["bytes"] for row in rows),
            "archive": {"path": self.archive.name, "bytes": self.archive.stat().st_size,
                        "sha256": digest(self.archive)},
            "output_manifest": {"path": self.manifest.name, "sha256": digest(self.manifest)},
            "output_tree_sha256": hashlib.sha256(json.dumps(
                rows, sort_keys=True, separators=(",", ":"), allow_nan=False
            ).encode()).hexdigest(),
            "exact_archive_content_verified": True, "stable_output_rechecked": True,
        }
        self.record.write_text(json.dumps(value), encoding="utf-8")

    def slot(self) -> dict[str, object]:
        return {
            "kind": "archive_only", "replay_kind": "validated_trial",
            "archive_path": str(self.archive), "manifest_path": str(self.manifest),
            "archive_record_path": str(self.record), "original_output_root": str(self.target),
            "report_path": str(self.target / "g3-evidence.json"),
            "mission_path": str(self.target / "mission-result.json"),
            "artifact_root": str(self.target), "session_root": str(self.session),
            "primary_inputs_path": str(self.primary), "primary_inputs_sha256": digest(self.primary),
        }

    def external_record_slot(self) -> tuple[dict[str, object], Path]:
        canonical = self.root / "canonical-record.json"
        shutil.copyfile(self.record, canonical)
        slot = self.slot()
        slot["archive_record_path"] = str(canonical)
        return slot, canonical

    def test_materializes_literal_root_and_cleans_only_owned_tree(self) -> None:
        with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40) as replay:
            self.assertEqual("validated_trial", replay["kind"])
            self.assertEqual(self.files["rosbag/data.db3"], (self.target / "rosbag/data.db3").read_bytes())
        self.assertFalse(self.target.exists())
        self.assertTrue(self.archive.is_file())
        self.assertTrue(self.session.is_dir())

    def test_foreign_binding_existing_target_and_mid_replay_mutation_fail(self) -> None:
        foreign = json.loads(self.record.read_text(encoding="utf-8"))
        foreign["bindings"]["trial_input_sha256"] = "f" * 64
        self.record.write_text(json.dumps(foreign), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "bindings differ"):
            with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40):
                pass
        foreign["bindings"]["trial_input_sha256"] = "2" * 64
        self.record.write_text(json.dumps(foreign), encoding="utf-8")
        self.target.mkdir()
        with self.assertRaisesRegex(ValueError, "already exists"):
            with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40):
                pass
        self.target.rmdir()
        with self.assertRaisesRegex(ValueError, "support closure node changed"):
            with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40):
                self.primary.write_text("changed\n", encoding="utf-8")
        self.assertFalse(self.target.exists())

    def test_external_replacement_is_preserved(self) -> None:
        foreign = self.target.parent / "foreign"
        foreign.mkdir()
        marker = foreign / "keep.txt"
        marker.write_text("keep", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "external replacement was preserved"):
            with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40):
                moved = self.target.parent / "owned-moved"
                os.replace(self.target, moved)
                os.replace(foreign, self.target)
        self.assertEqual("keep", (self.target / "keep.txt").read_text(encoding="utf-8"))
        shutil.rmtree(self.target.parent / "owned-moved")

    def test_foreign_run_layout_is_rejected_even_when_record_and_slot_are_rewritten_together(self) -> None:
        record = json.loads(self.record.read_text(encoding="utf-8"))
        record["run_label"] = "g3-m-closure-eta-999"
        foreign = self.root / "results" / record["run_label"]
        record["original_output_root"] = str(foreign)
        record["literal_paths"]["artifact_root"] = str(foreign)
        self.record.write_text(json.dumps(record), encoding="utf-8")
        slot = self.slot()
        slot["original_output_root"] = str(foreign)
        slot["artifact_root"] = str(foreign)
        slot["report_path"] = str(foreign / "g3-evidence.json")
        slot["mission_path"] = str(foreign / "mission-result.json")
        with self.assertRaisesRegex(ValueError, "run label"):
            with materialize_archive_slot(slot, self.trial, expected_source_commit="b" * 40):
                pass

    def test_session_exact_set_is_checked_when_builder_raises(self) -> None:
        caught = None
        try:
            with materialize_archive_slot(self.slot(), self.trial,
                                          expected_source_commit="b" * 40):
                (self.session / "injected.txt").write_text("changed", encoding="utf-8")
                raise RuntimeError("builder failed")
        except ValueError as exc:
            caught = exc
        self.assertIsNotNone(caught)
        assert caught is not None
        self.assertIn("exact node set changed", str(caught))
        self.assertIsInstance(caught.__context__, RuntimeError)

    @unittest.skipUnless(os.name == "nt", "Windows junction regression")
    def test_read_path_rejects_junction_ancestor(self) -> None:
        real = self.root / "junction-real"
        real.mkdir()
        (real / "value.json").write_text("{}", encoding="utf-8")
        link = self.root / "junction-link"
        result = subprocess.run(["cmd.exe", "/c", "mklink", "/J", str(link), str(real)],
                                capture_output=True, text=True, check=False)
        if result.returncode != 0:
            self.skipTest("junction creation is unavailable")
        try:
            with self.assertRaisesRegex(ValueError, "junction ancestor"):
                _regular(link / "value.json", "junction fixture")
        finally:
            link.rmdir()

    def test_broken_symlink_replacement_is_preserved(self) -> None:
        moved = self.target.parent / "owned-moved-broken-link"
        try:
            with self.assertRaisesRegex(ValueError, "external replacement was preserved"):
                with materialize_archive_slot(self.slot(), self.trial,
                                              expected_source_commit="b" * 40):
                    os.replace(self.target, moved)
                    missing = self.target.parent / "missing-target"
                    if os.name == "nt":
                        result = subprocess.run(
                            ["cmd.exe", "/c", "mklink", "/J", str(self.target), str(missing)],
                            capture_output=True, text=True, check=False,
                        )
                        if result.returncode != 0:
                            raise OSError(result.stderr or result.stdout)
                    else:
                        os.symlink(missing, self.target, target_is_directory=True)
        except OSError as exc:
            if moved.exists() and not self.target.exists() and not self.target.is_symlink():
                os.replace(moved, self.target)
            self.skipTest(f"symlink creation is unavailable: {exc}")
        self.assertTrue(self.target.is_symlink() or os.name == "nt")
        if self.target.is_symlink():
            self.target.unlink()
        else:
            self.target.rmdir()
        shutil.rmtree(moved)

    def test_malicious_existing_literal_record_is_rejected(self) -> None:
        slot, _canonical = self.external_record_slot()
        self.record.write_text("{}\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "existing literal archive resource bytes differ"):
            with materialize_archive_slot(slot, self.trial, expected_source_commit="b" * 40):
                pass
        self.assertEqual({}, json.loads(self.record.read_text(encoding="utf-8")))

    def test_absent_literal_record_is_owned_then_cleaned(self) -> None:
        slot, canonical = self.external_record_slot()
        expected = canonical.read_bytes()
        self.record.unlink()
        with materialize_archive_slot(slot, self.trial, expected_source_commit="b" * 40):
            self.assertEqual(expected, self.record.read_bytes())
        self.assertFalse(self.record.exists())
        self.assertEqual(expected, canonical.read_bytes())

    def test_literal_record_mutation_on_builder_error_is_rechecked_and_cleaned(self) -> None:
        slot, _canonical = self.external_record_slot()
        self.record.unlink()
        caught = None
        try:
            with materialize_archive_slot(slot, self.trial, expected_source_commit="b" * 40):
                self.record.write_text("mutated\n", encoding="utf-8")
                raise RuntimeError("builder failed")
        except ValueError as exc:
            caught = exc
        self.assertIsNotNone(caught)
        assert caught is not None
        self.assertIn("archive replay input changed", str(caught))
        self.assertFalse(self.record.exists())

    def test_literal_record_external_replacement_is_preserved(self) -> None:
        slot, _canonical = self.external_record_slot()
        self.record.unlink()
        moved = self.resources / "owned-record-moved.json"
        with self.assertRaisesRegex(ValueError, "external replacement was preserved"):
            with materialize_archive_slot(slot, self.trial, expected_source_commit="b" * 40):
                os.replace(self.record, moved)
                self.record.write_text('{"foreign":true}\n', encoding="utf-8")
        self.assertEqual('{"foreign":true}\n', self.record.read_text(encoding="utf-8"))
        self.record.unlink()
        moved.unlink()

    def test_v1_is_readable_but_cannot_be_used_as_self_authored_replay_proof(self) -> None:
        value = json.loads(self.record.read_text(encoding="utf-8"))
        for name in ("identity", "bindings", "original_output_root", "literal_paths", "output_tree_sha256"):
            value.pop(name)
        value["schema_version"] = 1
        self.record.write_text(json.dumps(value), encoding="utf-8")
        self.assertEqual(1, load_archive_record(self.record)["schema_version"])
        with self.assertRaisesRegex(ValueError, "requires a v2"):
            with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40):
                pass

    def test_collection_failure_materialization_reuses_immutable_resource_files(self) -> None:
        start = self.resources / "attempt-start.json"
        start.write_text(json.dumps({"references": {}}), encoding="utf-8")
        receipt = self.resources / "attempt-receipt.json"
        record_before = json.loads(self.record.read_text(encoding="utf-8"))
        receipt.write_text(json.dumps({"run_id": self.run_id,
            "identity": record_before["identity"], "bindings": record_before["bindings"],
            "references": {
            "start": {"path": start.relative_to(self.root).as_posix(), "sha256": digest(start)},
            "archive": {"path": self.archive.relative_to(self.root).as_posix(),
                        "sha256": digest(self.archive)},
            "output_manifest": {"path": self.manifest.relative_to(self.root).as_posix(),
                                "sha256": digest(self.manifest)},
        }}), encoding="utf-8")
        record = record_before
        record["literal_paths"]["attempt_receipt_path"] = str(receipt)
        self.record.write_text(json.dumps(record), encoding="utf-8")
        slot = {
            "kind": "archive_only", "replay_kind": "collection_failure",
            "archive_path": str(self.archive), "manifest_path": str(self.manifest),
            "archive_record_path": str(self.record), "original_output_root": str(self.target),
            "receipt_path": str(receipt), "evidence_root": str(self.root),
        }
        archive_before = digest(self.archive)
        with materialize_archive_slot(slot, self.trial, expected_source_commit="b" * 40) as replay:
            self.assertEqual("collection_failure", replay["kind"])
            self.assertTrue(self.target.is_dir())
        self.assertFalse(self.target.exists())
        self.assertEqual(archive_before, digest(self.archive))
        self.assertTrue(receipt.is_file())

    def test_link_case_collision_and_path_traversal_archives_fail(self) -> None:
        rows = json.loads(self.manifest.read_text(encoding="utf-8"))["files"]
        rows.append(dict(rows[0], path=rows[0]["path"].upper()))
        self.manifest.write_text(json.dumps({"schema_version": 1, "files": rows}), encoding="utf-8")
        record = json.loads(self.record.read_text(encoding="utf-8"))
        record["output_manifest"]["sha256"] = digest(self.manifest)
        self.record.write_text(json.dumps(record), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "collide"):
            with materialize_archive_slot(self.slot(), self.trial, expected_source_commit="b" * 40):
                pass


if __name__ == "__main__":
    unittest.main()
