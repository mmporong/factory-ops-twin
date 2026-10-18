from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.g3_qualification_gate import (
    PRODUCER_SOURCE_COMMIT,
    _host_source_commit,
    validate_qualification_predecessor,
)


ROOT = Path(__file__).parents[1]
RUN_LABEL = "g3-q-normal-fifo-013"
RUN_ID = "1" * 32


def save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class QualificationGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.host_repository = self.root / "host"
        for relative in (
            "factory_ops/g3_qualification_gate.py",
            "scripts/brev/g3_qualification_gate.py",
            "tests/test_g3_qualification_gate.py",
        ):
            target = self.host_repository / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, target)
        subprocess.run(("git", "init", "-q"), cwd=self.host_repository, check=True)
        subprocess.run(("git", "config", "user.email", "fixture@example.invalid"),
                       cwd=self.host_repository, check=True)
        subprocess.run(("git", "config", "user.name", "Fixture"),
                       cwd=self.host_repository, check=True)
        subprocess.run(("git", "add", "factory_ops/g3_qualification_gate.py",
                        "scripts/brev/g3_qualification_gate.py", "tests/test_g3_qualification_gate.py"),
                       cwd=self.host_repository, check=True)
        subprocess.run(("git", "commit", "-qm", "fixture host"), cwd=self.host_repository, check=True)
        self.host_source = subprocess.check_output(
            ("git", "rev-parse", "HEAD"), cwd=self.host_repository, text=True,
        ).strip()
        host = patch("factory_ops.g3_qualification_gate._host_source_commit",
                     return_value=self.host_source)
        host.start()
        self.addCleanup(host.stop)

        self.rows = []
        self.backups = {}
        self.sources = {}
        for store_id in ("D", "E"):
            restore = self.root / f"restore-{store_id}"
            artifact = restore / "artifact"
            session = restore / "session"
            session.mkdir(parents=True)
            save(session / "session.json", {"profile": "qualification"})
            report = {
                "run_id": RUN_ID, "profile": "qualification", "condition": "normal",
                "policy": "fifo", "seed": 0, "passed": True, "status": "validated",
                "errors": [], "closed_bag": True, "actual_validator_passed": True,
                "collection_qa_passed": True, "safety_qa_passed": True,
                "terminal_qa_passed": True,
                "source": {"commit": PRODUCER_SOURCE_COMMIT, "environment_sha256": "e" * 64},
                "fixture_boundary": "explicit_nonphysical_validator_double",
            }
            save(artifact / "g3-evidence.json", report)
            raw_manifest = self.root / f"raw-{store_id}.json"
            backup_record = self.root / f"backup-{store_id}.json"
            verification = self.root / f"verification-{store_id}.json"
            save(raw_manifest, {"store": store_id})
            save(backup_record, {"store": store_id})
            save(verification, {"store": store_id, "exact_restore": True})
            row = {
                "store_id": store_id, "restore_root": str(restore),
                "raw_manifest_path": str(raw_manifest), "backup_record_path": str(backup_record),
                "verification_path": str(verification), "session_root": str(session),
                "artifact_root": str(artifact),
            }
            self.rows.append(row)
            common_record = {
                "archive_sha256": "a" * 64, "manifest_sha256": "b" * 64,
                "source_bundle_sha256": "c" * 64, "file_count": 12,
                "total_bytes": 3456, "source_commit": PRODUCER_SOURCE_COMMIT,
            }
            self.backups[str(restore.resolve())] = {
                "root": restore.resolve(), "record": common_record,
                "verification": {"exact_restore": True},
            }
            self.sources[str(restore.resolve())] = {
                "archives": [
                    {"path": "D:\\q013.tar", "bytes": 100, "sha256": "a" * 64},
                    {"path": "E:\\q013.tar", "bytes": 100, "sha256": "a" * 64},
                ],
                "bundle": {"sha256": "c" * 64, "main_commit": PRODUCER_SOURCE_COMMIT},
                "verified_source_pin_count": 17,
            }
        self.inventory = self.root / "inventory.json"
        self.supplement = self.root / "supplement.json"
        save(self.inventory, {"sealed": True})
        save(self.supplement, {"sealed": True})

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def backup(self, root, *_args, **_kwargs):
        return deepcopy(self.backups[str(Path(root).resolve())])

    def source(self, backup, *_args, **_kwargs):
        return deepcopy(self.sources[str(Path(backup["root"]).resolve())])

    def separation(self, *_args, **_kwargs):
        return {
            "inventory": {"path": str(self.inventory), "sha256": digest(self.inventory)},
            "supplement_manifest": {"path": str(self.supplement), "sha256": digest(self.supplement)},
            "archive_drives": ["D:", "E:"],
            "physical_disks": [
                [{"index": 1, "device_id": "disk-D", "serial_sha256_prefix": "A" * 12}],
                [{"index": 2, "device_id": "disk-E", "serial_sha256_prefix": "B" * 12}],
            ],
        }

    def invoke(self, name: str = "gate", rows=None) -> Path:
        with patch("factory_ops.g3_qualification_gate.verify_backup_restore", side_effect=self.backup), \
                patch("factory_ops.g3_qualification_gate.verify_backup_archives_and_source",
                      side_effect=self.source), \
                patch("factory_ops.g3_qualification_gate.verify_storage_separation",
                      side_effect=self.separation), \
                patch("factory_ops.g3_qualification_gate.revalidate_physical_report",
                      side_effect=lambda path, *_args, **_kwargs: json.loads(Path(path).read_text())):
            return validate_qualification_predecessor(
                self.root / name, run_label=RUN_LABEL, run_id=RUN_ID,
                restore_rows=self.rows if rows is None else rows,
                storage_inventory_path=self.inventory,
                storage_supplement_manifest_path=self.supplement,
                storage_supplement_manifest_sha256=digest(self.supplement),
                host_repository=self.host_repository,
            )

    def test_closes_two_restore_predecessor_without_opening_approval(self) -> None:
        receipt_path = self.invoke()
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        manifest = json.loads((receipt_path.parent / "output-manifest.json").read_text())
        self.assertEqual({"D", "E"}, set(receipt["stores"]))
        self.assertEqual(self.host_source, receipt["host_source_commit"])
        self.assertTrue(receipt["semantic_revalidation_passed"])
        for name in ("actual_six_approved", "sixty_primary_trials_approved", "launch_authorized",
                     "final_approved", "cleanup_authorized"):
            self.assertFalse(receipt[name])
        self.assertEqual(digest(receipt_path), manifest["files"][0]["sha256"])
        with self.assertRaises(FileExistsError):
            self.invoke()

    def test_missing_duplicate_and_reused_restore_inputs_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "exact D and E"):
            self.invoke("missing", self.rows[:1])
        duplicate = deepcopy(self.rows)
        duplicate[1]["store_id"] = "D"
        with self.assertRaisesRegex(ValueError, "duplicated"):
            self.invoke("duplicate", duplicate)
        reused = deepcopy(self.rows)
        reused[1]["verification_path"] = reused[0]["verification_path"]
        with self.assertRaisesRegex(ValueError, "reuse"):
            self.invoke("reused", reused)

    def test_restore_content_mismatch_and_failed_restore_fail_closed(self) -> None:
        changed = deepcopy(self.backups[str(Path(self.rows[1]["restore_root"]).resolve())])
        changed["record"]["total_bytes"] += 1
        self.backups[str(Path(self.rows[1]["restore_root"]).resolve())] = changed
        with self.assertRaisesRegex(ValueError, "contents differ"):
            self.invoke("mismatch")
        self.backups[str(Path(self.rows[1]["restore_root"]).resolve())]["record"]["total_bytes"] -= 1
        with patch("factory_ops.g3_qualification_gate.verify_backup_restore",
                   side_effect=ValueError("failed exact restore")), self.assertRaisesRegex(
                       ValueError, "failed exact restore"
                   ):
            validate_qualification_predecessor(
                self.root / "failed", run_label=RUN_LABEL, run_id=RUN_ID, restore_rows=self.rows,
                storage_inventory_path=self.inventory,
                storage_supplement_manifest_path=self.supplement,
                storage_supplement_manifest_sha256=digest(self.supplement),
                host_repository=self.host_repository,
            )

    def test_report_failure_or_one_restore_disk_spoof_is_rejected(self) -> None:
        report = Path(self.rows[1]["artifact_root"]) / "g3-evidence.json"
        value = json.loads(report.read_text())
        value["terminal_qa_passed"] = False
        save(report, value)
        with self.assertRaisesRegex(ValueError, "identity or QA"):
            self.invoke("bad-report")
        value["terminal_qa_passed"] = True
        save(report, value)

        def same_disk(*_args, **_kwargs):
            result = self.separation()
            result["physical_disks"][1] = deepcopy(result["physical_disks"][0])
            return result

        with patch("factory_ops.g3_qualification_gate.verify_backup_restore", side_effect=self.backup), \
                patch("factory_ops.g3_qualification_gate.verify_backup_archives_and_source",
                      side_effect=self.source), \
                patch("factory_ops.g3_qualification_gate.verify_storage_separation", side_effect=same_disk), \
                patch("factory_ops.g3_qualification_gate.revalidate_physical_report",
                      side_effect=lambda path, *_args, **_kwargs: json.loads(Path(path).read_text())), \
                self.assertRaisesRegex(ValueError, "independently sealed"):
            validate_qualification_predecessor(
                self.root / "same-disk", run_label=RUN_LABEL, run_id=RUN_ID,
                restore_rows=self.rows, storage_inventory_path=self.inventory,
                storage_supplement_manifest_path=self.supplement,
                storage_supplement_manifest_sha256=digest(self.supplement),
                host_repository=self.host_repository,
            )

    def test_host_source_requires_active_clean_committed_checkout(self) -> None:
        fake_module = self.host_repository / "factory_ops/g3_qualification_gate.py"
        with patch("factory_ops.g3_qualification_gate.__file__", str(fake_module)):
            self.assertEqual(self.host_source, _host_source_commit(self.host_repository))
            fake_module.write_bytes(fake_module.read_bytes() + b"\n")
            with self.assertRaisesRegex(ValueError, "clean"):
                _host_source_commit(self.host_repository)
            subprocess.run(("git", "checkout", "--", "factory_ops/g3_qualification_gate.py"),
                           cwd=self.host_repository, check=True)
        with self.assertRaisesRegex(ValueError, "active execution checkout"):
            _host_source_commit(self.host_repository)


if __name__ == "__main__":
    unittest.main()
