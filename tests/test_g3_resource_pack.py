from __future__ import annotations

import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from scripts.brev.g3_resource_pack import archive_output, checkpoint, sha, verify_archive
from factory_ops.g3_supervisor import create_attempt_start, finalize_attempt, validate_attempt_receipt


class G3ResourcePackTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.run_label = "g3-p-normal-fifo-001"
        self.run_id = "a" * 32
        self.output = self.root / "results" / self.run_label
        self.output.mkdir(parents=True)
        (self.output / "failed.json").write_text('{"status":"failed"}\n', encoding="utf-8")
        (self.output / "rosbag").mkdir()
        (self.output / "rosbag/data.db3").write_bytes(b"explicit_non_evidence_test_bytes")

    def record(self):
        for phase in ("start", "workers-finished", "validation-finished"):
            checkpoint(self.root, self.run_label, self.run_id, phase)

    def test_closed_failed_output_can_be_restored_exactly_without_becoming_qa(self):
        self.record()
        record = archive_output(self.root, self.run_label, self.run_id)
        resource = self.root / "resources" / self.run_label
        self.assertEqual("g3_closed_output_backup_not_validation", record["classification"])
        self.assertNotIn("passed", record)
        self.assertEqual(2, record["file_count"])
        restore = self.root / "restored"
        with tarfile.open(resource / "output.tar.gz") as packed:
            packed.extractall(restore, filter="data")
        expected = {path.relative_to(self.output).as_posix(): sha(path)
                    for path in self.output.rglob("*") if path.is_file()}
        actual = {path.relative_to(restore).as_posix(): sha(path)
                  for path in restore.rglob("*") if path.is_file()}
        self.assertEqual(expected, actual)
        with self.assertRaises(ValueError):
            archive_output(self.root, self.run_label, self.run_id)
        with self.assertRaises(FileExistsError):
            checkpoint(self.root, self.run_label, self.run_id, "start")

    def test_checkpoint_identity_time_and_tar_digest_mutations_fail(self):
        self.record()
        resource = self.root / "resources" / self.run_label
        path = resource / "workers-finished.json"
        original = path.read_bytes()
        value = json.loads(original)
        value["run_id"] = "b" * 32
        path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaises(ValueError):
            archive_output(self.root, self.run_label, self.run_id)
        path.write_bytes(original)
        archive_output(self.root, self.run_label, self.run_id)
        manifest = json.loads((resource / "output-manifest.json").read_text(encoding="utf-8"))
        manifest["files"][0]["sha256"] = "0" * 64
        with self.assertRaises(ValueError):
            verify_archive(resource / "output.tar.gz", manifest)

    def test_unfinished_or_wrong_attempt_cannot_archive(self):
        checkpoint(self.root, self.run_label, self.run_id, "start")
        with self.assertRaises(FileNotFoundError):
            archive_output(self.root, self.run_label, self.run_id)
        with self.assertRaises(ValueError):
            checkpoint(self.root, self.run_label, "b" * 32, "workers-finished")
        with self.assertRaises(ValueError):
            checkpoint(self.root, "../old", self.run_id, "start")

    def test_actual_checkpoint_format_flows_through_supervisor_finalize(self):
        # Only serialization/archiving integration: no physical trial or valid bag claim.
        with patch("scripts.brev.g3_resource_pack.shutil.disk_usage", return_value=SimpleNamespace(free=1_000_000)):
            start = checkpoint(self.root, self.run_label, self.run_id, "start")
        resource = self.root / "resources" / self.run_label
        config = self.root / "config.json"
        config.write_text('{}', encoding="utf-8")
        session = self.root / "session"
        session.mkdir()
        (session / "session.json").write_text(json.dumps({
            "profile": "pilot", "condition": "normal", "policy": "fifo", "seed": 100,
            "workload_sha256": "f" * 64,
        }), encoding="utf-8")
        billing = self.root / "billing.json"
        billing.write_text(json.dumps({
            "observed_at": start["observed_at_utc"], "source": "explicit_non_evidence_test_fixture",
            "confirmation_method": "serialization test only", "currency": "USD",
            "confirmed_hourly_rate_usd": 3.36, "budget_remaining_usd": 20,
        }), encoding="utf-8")
        create_attempt_start(resource / "attempt-start.json", self.root, run_id=self.run_id,
                             profile="pilot", condition="normal", policy="fifo", seed=100,
                             source_commit="b" * 40, config_path=config, session_root=session,
                             billing_path=billing, free_disk_bytes_before=start["free_disk_bytes"],
                             started_at_utc=start["observed_at_utc"])
        with patch("scripts.brev.g3_resource_pack.shutil.disk_usage", return_value=SimpleNamespace(free=900_000)):
            workers = checkpoint(self.root, self.run_label, self.run_id, "workers-finished")
            validation = checkpoint(self.root, self.run_label, self.run_id, "validation-finished")
            packed = archive_output(self.root, self.run_label, self.run_id)
        receipt_path = resource / "attempt-receipt.json"
        finalize_attempt(resource / "attempt-start.json", receipt_path, self.root,
                         workers_finished_at_utc=workers["observed_at_utc"],
                         validation_finished_at_utc=validation["observed_at_utc"],
                         archive_finished_at_utc=packed["archive_finished_at_utc"],
                         gpu_csv_path=None, scene_summary_path=None,
                         output_manifest_path=resource / "output-manifest.json",
                         archive_path=resource / "output.tar.gz", disk_path=resource / "disk.json",
                         collection_failure_reason="explicit_non_evidence serialization fixture")
        result = validate_attempt_receipt(receipt_path, self.root, None, expected_source_commit="b" * 40,
                                          expected_profile="pilot", expected_condition="normal",
                                          expected_policy="fifo", expected_seed=100)
        self.assertEqual("collection_failed", result["attempt_status"])
        self.assertTrue(all(value.endswith("Z") for value in result["timestamps"].values()))


if __name__ == "__main__":
    unittest.main()
