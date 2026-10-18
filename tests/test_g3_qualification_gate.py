from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path, PureWindowsPath
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.g3_qualification_gate import (
    PRODUCER_SOURCE_COMMIT,
    _canonical_sha256,
    _host_source_commit,
    _wsl_restore_root,
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


def reference(path: Path) -> dict[str, object]:
    path = path.resolve(strict=True)
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": digest(path)}


class QualificationGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.host_repository = self.root / "host"
        for relative in (
            "factory_ops/g3_qualification_gate.py",
            "scripts/brev/g3_qualification_gate.py",
            "tests/test_g3_qualification_gate.py",
            "configs/calibrations/g3-qualification-013-predecessor.json",
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
                        "scripts/brev/g3_qualification_gate.py", "tests/test_g3_qualification_gate.py",
                        "configs/calibrations/g3-qualification-013-predecessor.json"),
                       cwd=self.host_repository, check=True)
        subprocess.run(("git", "commit", "-qm", "fixture host"), cwd=self.host_repository, check=True)
        self.host_source = subprocess.check_output(
            ("git", "rev-parse", "HEAD"), cwd=self.host_repository, text=True,
        ).strip()
        host = patch("factory_ops.g3_qualification_gate._host_source_commit",
                     return_value=self.host_source)
        host.start()
        self.addCleanup(host.stop)

        self.operation_wrapper = self.root / "run-q013-full-restore-operation-001.py"
        self.semantic_helper = self.root / "replay-q013-restored-evidence-001.py"
        self.operation_helpers = {
            "D": self.root / "verify-industrial-g3-015-d-source.py",
            "E": self.root / "preserve-industrial-g3-015.py",
        }
        self.decoder_checkout = self.root / "decoder-source-s"
        self.decoder_checkout.mkdir()
        for path, content in (
            (self.operation_wrapper, "operation wrapper fixture\n"),
            (self.semantic_helper, "semantic helper fixture\n"),
            (self.operation_helpers["D"], "D helper fixture\n"),
            (self.operation_helpers["E"], "E helper fixture\n"),
        ):
            path.write_text(content, encoding="utf-8")

        self.rows = []
        self.backups = {}
        self.sources = {}
        for store_id in ("D", "E"):
            restore = self.root / store_id / RUN_ID / "full-restore"
            artifact = restore / "results" / RUN_LABEL
            session = restore / "source-snapshots" / RUN_LABEL / "runs" / f"{RUN_LABEL}-session"
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
            operation_receipt = self.root / f"operation-{store_id}.json"
            semantic_receipt = self.root / f"semantic-{store_id}.json"
            archive = f"{store_id}:\\q013.tar"
            save(raw_manifest, {"store": store_id})
            save(backup_record, {"store": store_id})
            save(verification, {"store": store_id, "exact_restore": True})
            row = {
                "store_id": store_id, "restore_root": str(restore),
                "raw_manifest_path": str(raw_manifest), "backup_record_path": str(backup_record),
                "verification_path": str(verification), "session_root": str(session),
                "operation_receipt_path": str(operation_receipt),
                "semantic_replay_receipt_path": str(semantic_receipt),
                "path_mapping_path": str(self.root / f"mapping-{store_id}.json"),
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
                "verification": {"exact_restore": True, "runtime_source_pins_verified": 17},
            }
            self.sources[str(restore.resolve())] = {
                "archives": [
                    {"path": archive, "bytes": 7, "sha256": "a" * 64},
                ],
                "bundle": {"sha256": "c" * 64, "main_commit": PRODUCER_SOURCE_COMMIT},
                "verified_source_pin_count": 17,
            }
        self.inventory = self.root / "inventory.json"
        self.supplement = self.root / "supplement.json"
        self.prestart = self.root / "prestart.json"
        self.preservation = self.root / "preservation.json"
        save(self.inventory, {"observed_at_utc": "inventory-time"})
        save(self.supplement, {"sealed": True})
        snapshot = {"bytes": self.inventory.stat().st_size, "device": 1, "inode": 2,
                    "links": 1, "sha256": digest(self.inventory)}
        save(self.preservation, {
            "schema_version": 1, "status": "passed", "source_commit": PRODUCER_SOURCE_COMMIT,
            "verified_at_utc": "preservation-time", "manifest_sha256": digest(self.supplement),
            "disk_identity_inventory": "artifacts/brev/inventory.json",
            "input_snapshots": [{"path": "artifacts/brev/inventory.json",
                                 "before": snapshot, "after": snapshot}],
        })
        save(self.prestart, {
            "schema_version": 1, "status": "passed", "source_commit": PRODUCER_SOURCE_COMMIT,
            "run_label": RUN_LABEL, "checked_at_utc": "prestart-time",
            "source_preservation": {"path": self.preservation.name,
                                    "bytes": self.preservation.stat().st_size,
                                    "sha256": digest(self.preservation)},
            "actual_q013_executed": False, "worker_launch_gate_opened": False,
            "six_pilot_gate_opened": False, "primary_matrix_gate_opened": False,
        })
        self.devices = {
            "D": {"index": 3, "device_id": r"\\.\PHYSICALDRIVE3", "serial_sha256_prefix": "A" * 12},
            "E": {"index": 2, "device_id": r"\\.\PHYSICALDRIVE2", "serial_sha256_prefix": "B" * 12},
        }
        self.trusted = {
            "schema_version": 1, "source_commit": PRODUCER_SOURCE_COMMIT, "run_label": RUN_LABEL,
            "prestart_receipt": {"name": self.prestart.name, "bytes": self.prestart.stat().st_size,
                                 "sha256": digest(self.prestart), "checked_at_utc": "prestart-time"},
            "source_preservation_receipt": {
                "name": self.preservation.name, "bytes": self.preservation.stat().st_size,
                "sha256": digest(self.preservation), "verified_at_utc": "preservation-time"},
            "sealed_supplement_manifest": {
                "name": self.supplement.name, "bytes": self.supplement.stat().st_size,
                "sha256": digest(self.supplement)},
            "storage_inventory": {
                "path": "artifacts/brev/inventory.json", "name": self.inventory.name,
                "bytes": self.inventory.stat().st_size, "sha256": digest(self.inventory),
                "observed_at_utc": "inventory-time", "snapshot": snapshot},
            "restore_layout": {
                "artifact_relative": f"results/{RUN_LABEL}",
                "session_relative": f"source-snapshots/{RUN_LABEL}/runs/{RUN_LABEL}-session",
            },
            "operation_producers": {
                "wrapper": {"name": self.operation_wrapper.name,
                            "sha256": digest(self.operation_wrapper)},
                "prepared_manifest_sha256": "9" * 64,
                "stores": {
                    name: {"name": path.name, "sha256": digest(path)}
                    for name, path in self.operation_helpers.items()
                },
            },
            "semantic_producer": {"name": self.semantic_helper.name,
                                  "sha256": digest(self.semantic_helper)},
            "stores": {},
        }
        for row in self.rows:
            store_id = row["store_id"]
            restore = Path(row["restore_root"])
            canonical_restore = str(PureWindowsPath(str(restore)))
            archive = self.sources[str(restore.resolve())]["archives"][0]["path"]
            self.trusted["stores"][store_id] = {
                "archive_drive": f"{store_id}:", "restore_root": canonical_restore,
                "device_identity": self.devices[store_id],
            }
            common = self.backups[str(restore.resolve())]["record"]
            operation_log = self.root / f"operation-{store_id}.log"
            operation_log.write_text(f"closed operation {store_id}\n", encoding="utf-8")
            live_cim = self.root / f"live-cim-{store_id}.json"
            save(live_cim, {
                "schema_version": 1,
                "source": "live CIM associations before independent full restore",
                "observed_at_utc": "2026-10-18T06:59:00+00:00",
                "stores": [{"store_id": name, "physical_disks": [self.devices[name]]}
                           for name in ("D", "E")],
            })
            helper = self.operation_helpers[store_id]
            if store_id == "E":
                command = [str(self.root / "python.exe"), "-B", str(helper.resolve()),
                           "--expected-source", PRODUCER_SOURCE_COMMIT]
            else:
                command = [
                    str(self.root / "python.exe"), "-B", str(helper.resolve()),
                    "--expected-source", PRODUCER_SOURCE_COMMIT,
                    "--archive", archive, "--backup", "E:\\q013.tar",
                    "--record", str(backup_record.resolve()),
                    "--restore-parent", str(PureWindowsPath(canonical_restore).parent),
                    "--download-receipt", str((self.root / "download.json").resolve()),
                    "--capacity-record", str((self.root / "capacity.json").resolve()),
                ]
            save(Path(row["operation_receipt_path"]), {
                "schema_version": 1, "classification": "g3_qualification_full_restore_operation_receipt",
                "operation": "full_restore_completed_and_closed", "store_id": store_id,
                "source_device_identity": self.devices[store_id],
                "destination_device_identity": self.devices[store_id],
                "selected_source_archive": {"path": archive, "bytes": 7,
                                            "sha256": "a" * 64},
                "destination_restore_root": canonical_restore, "run_label": RUN_LABEL, "run_id": RUN_ID,
                "source_commit": PRODUCER_SOURCE_COMMIT, "archive_sha256": "a" * 64,
                "restore_manifest_sha256": "b" * 64, "source_bundle_sha256": "c" * 64,
                "source_bundle_restore_fsck": True, "runtime_source_pins_verified": 17,
                "started_at_utc": "2026-10-18T07:00:00+00:00",
                "finished_at_utc": "2026-10-18T07:01:00+00:00",
                "execution": {
                    "helper": reference(helper), "wrapper": reference(self.operation_wrapper),
                    "log": reference(operation_log), "live_cim": reference(live_cim),
                    "command": command, "helper_sha256": digest(helper),
                    "log_sha256": digest(operation_log), "exit_code": 0,
                },
            })
            operation_sha = digest(Path(row["operation_receipt_path"]))
            wsl_root = f"/mnt/{store_id.lower()}/fixture/{RUN_ID}/full-restore"
            save(Path(row["path_mapping_path"]), {
                "schema_version": 1, "classification": "g3_qualification_restore_path_mapping",
                "store_id": store_id, "windows_restore_root": canonical_restore,
                "wsl_restore_root": wsl_root, "restore_manifest_sha256": common["manifest_sha256"],
                "operation_receipt_sha256": operation_sha,
            })
            fresh_report = self.root / f"fresh-{store_id}-g3-evidence.json"
            shutil.copyfile(Path(row["artifact_root"]) / "g3-evidence.json", fresh_report)
            semantic_log = self.root / f"semantic-{store_id}.log"
            preserved_report = json.loads((Path(row["artifact_root"]) / "g3-evidence.json").read_text())
            semantic_rows = [
                {"observed_at_utc": "2026-10-18T07:01:10+00:00", "phase": "replay_started",
                 "store_id": store_id, "run_id": RUN_ID, "source_commit": PRODUCER_SOURCE_COMMIT,
                 "operation_receipt_sha256": operation_sha,
                 "path_mapping_sha256": digest(Path(row["path_mapping_path"])),
                 "decoder_checkout": str(self.decoder_checkout.resolve()),
                 "helper_sha256": digest(self.semantic_helper)},
                {"observed_at_utc": "2026-10-18T07:01:20+00:00",
                 "phase": "closed_bag_revalidation_started",
                 "report_sha256": digest(Path(row["artifact_root"]) / "g3-evidence.json"),
                 "session_json_sha256": digest(Path(row["session_root"]) / "session.json"),
                 "environment_sha256": "e" * 64,
                 "restore_manifest_sha256": common["manifest_sha256"]},
                {"observed_at_utc": "2026-10-18T07:01:30+00:00",
                 "phase": "closed_bag_revalidation_completed", "passed": True,
                 "semantic_equal": True, "fresh_report_sha256": digest(fresh_report),
                 "fresh_report_canonical_sha256": _canonical_sha256(preserved_report),
                 "preserved_report_canonical_sha256": _canonical_sha256(preserved_report),
                 "exit_code": 0},
            ]
            semantic_log.write_text(
                "".join(json.dumps(value, sort_keys=True) + "\n" for value in semantic_rows),
                encoding="utf-8",
            )
            runtime_manifest = self.root / f"runtime-manifest-{store_id}.json"
            runtime_files = sorted((fresh_report, semantic_log), key=lambda path: path.name)
            save(runtime_manifest, {
                "schema_version": 1,
                "classification": "g3_qualification_semantic_replay_output_manifest",
                "files": [{"path": path.name, "bytes": path.stat().st_size,
                           "sha256": digest(path)} for path in runtime_files],
            })
            save(Path(row["semantic_replay_receipt_path"]), {
                "schema_version": 1,
                "classification": "g3_qualification_independent_semantic_replay_receipt",
                "status": "passed", "errors": [], "store_id": store_id, "run_label": RUN_LABEL,
                "run_id": RUN_ID, "source_commit": PRODUCER_SOURCE_COMMIT,
                "destination_restore_root": canonical_restore, "wsl_restore_root": wsl_root,
                "artifact_root": f"{wsl_root}/results/{RUN_LABEL}",
                "session_root": f"{wsl_root}/source-snapshots/{RUN_LABEL}/runs/{RUN_LABEL}-session",
                "operation_receipt_sha256": operation_sha,
                "path_mapping_sha256": digest(Path(row["path_mapping_path"])),
                "report_sha256": digest(restore / "results" / RUN_LABEL / "g3-evidence.json"),
                "session_json_sha256": digest(
                    restore / "source-snapshots" / RUN_LABEL / "runs" / f"{RUN_LABEL}-session" / "session.json"),
                "preserved_report_canonical_sha256": _canonical_sha256(
                    json.loads((restore / "results" / RUN_LABEL / "g3-evidence.json").read_text())),
                "fresh_report_canonical_sha256": _canonical_sha256(
                    json.loads((restore / "results" / RUN_LABEL / "g3-evidence.json").read_text())),
                "semantic_equal": True,
                "environment_sha256": "e" * 64, "archive_sha256": common["archive_sha256"],
                "restore_manifest_sha256": common["manifest_sha256"],
                "source_bundle_sha256": common["source_bundle_sha256"],
                "source_bundle_restore_fsck": True, "runtime_source_pins_verified": 17,
                "cleanup_authorized": False,
                "observed_at_utc": "2026-10-18T07:02:00+00:00",
                "decoder_source": {"checkout_root": str(self.decoder_checkout.resolve()),
                                   "head": PRODUCER_SOURCE_COMMIT, "clean": True},
                "execution": {
                    "helper": reference(self.semantic_helper), "log": reference(semantic_log),
                    "fresh_report": reference(fresh_report),
                    "output_manifest": reference(runtime_manifest), "exit_code": 0,
                },
            })
        trusted = patch("factory_ops.g3_qualification_gate._trusted_provenance",
                        return_value=self.trusted)
        trusted.start()
        self.addCleanup(trusted.stop)
        wsl = patch("factory_ops.g3_qualification_gate._wsl_restore_root",
                    side_effect=lambda path: f"/mnt/{path.parts[-3].lower()}/fixture/{RUN_ID}/full-restore")
        wsl.start()
        self.addCleanup(wsl.stop)
        def fixture_drive(path: object) -> str:
            parsed = PureWindowsPath(str(path))
            if parsed.drive.upper() in {"D:", "E:"}:
                return parsed.drive.upper()
            return f"{parsed.parts[-3].upper()}:"

        drive = patch("factory_ops.g3_qualification_gate._drive", side_effect=fixture_drive)
        drive.start()
        self.addCleanup(drive.stop)
        decoder = patch("factory_ops.g3_qualification_gate._verify_decoder_checkout")
        decoder.start()
        self.addCleanup(decoder.stop)

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
                [self.devices["D"]],
                [self.devices["E"]],
            ],
        }

    def invoke(self, name: str = "gate", rows=None) -> Path:
        with patch("factory_ops.g3_qualification_gate.verify_backup_restore", side_effect=self.backup), \
                patch("factory_ops.g3_qualification_gate.verify_backup_archives_and_source",
                      side_effect=self.source), \
                patch("factory_ops.g3_qualification_gate.verify_storage_separation",
                      side_effect=self.separation):
            return validate_qualification_predecessor(
                self.root / name, run_label=RUN_LABEL, run_id=RUN_ID,
                restore_rows=self.rows if rows is None else rows,
                prestart_receipt_path=self.prestart,
                source_preservation_receipt_path=self.preservation,
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
                prestart_receipt_path=self.prestart,
                source_preservation_receipt_path=self.preservation,
                storage_inventory_path=self.inventory,
                storage_supplement_manifest_path=self.supplement,
                storage_supplement_manifest_sha256=digest(self.supplement),
                host_repository=self.host_repository,
            )

    def test_swapped_restore_root_source_and_mapping_fail_closed(self) -> None:
        swapped_roots = deepcopy(self.rows)
        for field in ("restore_root", "session_root", "artifact_root"):
            swapped_roots[0][field], swapped_roots[1][field] = (
                swapped_roots[1][field], swapped_roots[0][field])
        with self.assertRaisesRegex(ValueError, "operation|canonical"):
            self.invoke("swapped-roots", swapped_roots)

        operation_path = Path(self.rows[0]["operation_receipt_path"])
        operation = json.loads(operation_path.read_text())
        operation["selected_source_archive"]["path"] = "E:\\q013.tar"
        save(operation_path, operation)
        with self.assertRaisesRegex(ValueError, "select one exact source archive"):
            self.invoke("swapped-source")

        operation["selected_source_archive"]["path"] = "D:\\q013.tar"
        save(operation_path, operation)
        mapping_path = Path(self.rows[0]["path_mapping_path"])
        mapping = json.loads(mapping_path.read_text())
        mapping["wsl_restore_root"] = mapping["wsl_restore_root"].replace("/mnt/d/", "/mnt/e/")
        save(mapping_path, mapping)
        with self.assertRaisesRegex(ValueError, "path mapping differs"):
            self.invoke("swapped-mapping")

    def test_runtime_pin_count_and_trusted_prestart_are_exact(self) -> None:
        restore = str(Path(self.rows[0]["restore_root"]).resolve())
        self.backups[restore]["verification"]["runtime_source_pins_verified"] = 16
        with self.assertRaisesRegex(ValueError, "source pin count"):
            self.invoke("pin-mismatch")
        self.backups[restore]["verification"]["runtime_source_pins_verified"] = 17

        stale = json.loads(self.prestart.read_text())
        stale["checked_at_utc"] = "2026-10-17T00:00:00+00:00"
        save(self.prestart, stale)
        self.trusted["prestart_receipt"].update(
            bytes=self.prestart.stat().st_size, sha256=digest(self.prestart))
        with self.assertRaisesRegex(ValueError, "prestart receipt identity"):
            self.invoke("stale-prestart")

    def test_operation_execution_files_are_rehashed_and_command_is_bound(self) -> None:
        operation_path = Path(self.rows[0]["operation_receipt_path"])
        operation = json.loads(operation_path.read_text())
        operation["execution"]["helper_sha256"] = "1" * 64
        save(operation_path, operation)
        with self.assertRaisesRegex(ValueError, "producer, log or exit"):
            self.invoke("invented-helper-hash")

        operation["execution"]["helper_sha256"] = operation["execution"]["helper"]["sha256"]
        operation["execution"]["command"][2] = str(self.operation_wrapper.resolve())
        save(operation_path, operation)
        with self.assertRaisesRegex(ValueError, "execution command"):
            self.invoke("wrong-operation-command")

        operation["execution"]["command"][2] = operation["execution"]["helper"]["path"]
        operation["execution"]["command"][0] = str(self.root / "invented-runner.exe")
        save(operation_path, operation)
        with self.assertRaisesRegex(ValueError, "execution command"):
            self.invoke("wrong-operation-runner")

    def test_operation_log_content_must_match_its_receipt_reference(self) -> None:
        operation_path = Path(self.rows[0]["operation_receipt_path"])
        operation = json.loads(operation_path.read_text())
        log_path = Path(operation["execution"]["log"]["path"])
        log_path.write_text("replacement log\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "log reference bytes or SHA differ"):
            self.invoke("changed-operation-log")

    def test_semantic_fresh_result_must_exist_and_must_not_alias_preserved_report(self) -> None:
        semantic_path = Path(self.rows[0]["semantic_replay_receipt_path"])
        semantic = json.loads(semantic_path.read_text())
        fresh_path = Path(semantic["execution"]["fresh_report"]["path"])
        fresh_path.unlink()
        with self.assertRaisesRegex(ValueError, "fresh_report reference must be a regular file"):
            self.invoke("missing-fresh")

        shutil.copyfile(Path(self.rows[0]["artifact_root"]) / "g3-evidence.json", fresh_path)
        semantic["execution"]["fresh_report"] = reference(
            Path(self.rows[0]["artifact_root"]) / "g3-evidence.json")
        save(semantic_path, semantic)
        with self.assertRaisesRegex(ValueError, "alias preserved evidence"):
            self.invoke("aliased-fresh")

    def test_semantic_log_bindings_are_checked_after_rehashing_manifest(self) -> None:
        semantic_path = Path(self.rows[0]["semantic_replay_receipt_path"])
        semantic = json.loads(semantic_path.read_text())
        log_path = Path(semantic["execution"]["log"]["path"])
        log_rows = [json.loads(line) for line in log_path.read_text().splitlines()]
        log_rows[-1]["passed"] = False
        log_path.write_text(
            "".join(json.dumps(value, sort_keys=True) + "\n" for value in log_rows),
            encoding="utf-8",
        )
        semantic["execution"]["log"] = reference(log_path)
        manifest_path = Path(semantic["execution"]["output_manifest"]["path"])
        manifest = json.loads(manifest_path.read_text())
        manifest["files"] = [
            {"path": log_path.name, "bytes": log_path.stat().st_size, "sha256": digest(log_path)}
            if row["path"] == log_path.name else row
            for row in manifest["files"]
        ]
        save(manifest_path, manifest)
        semantic["execution"]["output_manifest"] = reference(manifest_path)
        save(semantic_path, semantic)
        with self.assertRaisesRegex(ValueError, "semantic replay log binding differs"):
            self.invoke("fabricated-semantic-log")

    def test_self_consistent_stale_inventory_chain_is_not_trusted(self) -> None:
        save(self.inventory, {"observed_at_utc": "older-inventory-time"})
        save(self.supplement, {"sealed": "older"})
        stale_snapshot = {"bytes": self.inventory.stat().st_size, "device": 9, "inode": 9,
                          "links": 1, "sha256": digest(self.inventory)}
        preservation = json.loads(self.preservation.read_text())
        preservation["manifest_sha256"] = digest(self.supplement)
        preservation["input_snapshots"] = [{
            "path": "artifacts/brev/inventory.json",
            "before": stale_snapshot, "after": stale_snapshot,
        }]
        save(self.preservation, preservation)
        prestart = json.loads(self.prestart.read_text())
        prestart["source_preservation"].update(
            bytes=self.preservation.stat().st_size, sha256=digest(self.preservation))
        save(self.prestart, prestart)
        with self.assertRaisesRegex(ValueError, "trusted predecessor file measurement"):
            self.invoke("stale-chain")

    def test_windows_restore_roots_have_fixed_wsl_mount_mapping(self) -> None:
        self.assertEqual(
            "/mnt/d/FactoryOpsArtifacts/industrial-diagnostics/restore-industrial-015/payload",
            _wsl_restore_root(PureWindowsPath(
                r"D:\FactoryOpsArtifacts\industrial-diagnostics\restore-industrial-015\payload")),
        )
        with self.assertRaisesRegex(ValueError, "canonical D or E"):
            _wsl_restore_root(PureWindowsPath(r"C:\temp\full-restore"))

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
                self.assertRaisesRegex(ValueError, "independently sealed"):
            validate_qualification_predecessor(
                self.root / "same-disk", run_label=RUN_LABEL, run_id=RUN_ID,
                restore_rows=self.rows, storage_inventory_path=self.inventory,
                prestart_receipt_path=self.prestart,
                source_preservation_receipt_path=self.preservation,
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
