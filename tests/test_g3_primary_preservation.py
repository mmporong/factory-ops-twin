from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.experiments import build_matrix, load_experiment_config
from factory_ops.g3_primary_preservation import (
    _default_semantic, _restore_source_checkout, close_primary_run, import_primary_run,
    preserve_primary_run,
)
from factory_ops.g3_raw_storage import _shared_support_pack_sha


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path: Path, value: object) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


class PrimaryPreservationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.evidence = self.root / "evidence"
        self.evidence.mkdir()
        self.store_d = self.root / "D"
        self.store_e = self.root / "E"
        self.store_d.mkdir()
        self.store_e.mkdir()
        self.run_id = "a" * 32
        self.source = "b" * 40
        self.host = "c" * 40
        self.matrix = build_matrix(load_experiment_config(
            Path(__file__).parents[1] / "configs/factory-v3/g3-experiment.json"))
        self.trial_id = self.matrix[0]["trial_id"]
        self.trial = save(self.root / "trial.json", self.matrix[0])
        self.primary = save(self.root / "primary.json", {
            "trial_path": str(self.trial), "calibration_path": str(self.root / "calibration.json"),
            "pilot_slots": {"fixture": {}}, "resource_input": {"fixture": True},
            "expected_source_commit": self.source,
        })
        self.result = save(self.evidence / "trial-result.json", {
            "run_id": self.run_id, "trial_id": self.trial_id,
            "trial_outcome": "failed",
        })
        self.envelope_path = save(self.evidence / "raw" / "g3-raw-content-envelope-v1.json",
                                  {"fixture": True})
        self.package = self.envelope_path.parent / "g3-raw-content-package-v1.tar"
        self.package.write_bytes(b"sealed-package")
        self.envelope = {
            "run_id": self.run_id, "trial_id": self.trial_id, "profile": "primary",
            "source_commit": self.source, "attempt_status": "collected",
            "package": {"bytes": self.package.stat().st_size, "sha256": sha(self.package)},
        }
        self.prepared = save(self.evidence / "primary-prepared" / self.run_id /
                             "prepared-primary-run.json", {
            "schema_version": 1, "classification": "g3_prepared_primary_preservation",
            "run_id": self.run_id, "trial_id": self.trial_id, "source_commit": self.source,
            "trial_outcome": "failed", "primary_inputs_path": str(self.primary),
            "primary_inputs_sha256": sha(self.primary),
            "trial_result": {"path": self.result.relative_to(self.evidence).as_posix(),
                             "sha256": sha(self.result)},
            "raw_envelope": {"path": self.envelope_path.relative_to(self.evidence).as_posix(),
                             "sha256": sha(self.envelope_path)},
            "package": {"path": str(self.package), "bytes": self.package.stat().st_size,
                        "sha256": sha(self.package)},
            "attempt_status": "collected", "cleanup_authorized": False,
        })

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _live(self, path: Path, *, ordinal: int, trial_id: str,
              primary: Path, previous_receipts: list[dict[str, object]] | None = None) -> Path:
        return save(path, {
            "schema_version": 1, "classification": "g3_live_primary_launch_recheck",
            "decision": "eligible_for_host_coordinator",
            "producer_source_commit": self.source, "host_source_commit": self.host,
            "primary_inputs_sha256": sha(primary), "trial_id": trial_id,
            "ordinal": ordinal, "previous_receipts": previous_receipts or [],
            "stores": {
                "D": {"device_identity": "device-D", "free_bytes": 1000,
                      "required_bytes": 1, "one_primary_replay_workspace_bytes": 1},
                "E": {"device_identity": "device-E", "free_bytes": 1000,
                      "required_bytes": 1, "one_primary_replay_workspace_bytes": 1},
            },
            "launch_executed": False, "cleanup_authorized": False,
            "external_expected_anchor_is_authorization": False,
            "primary_run_approved": False, "final_approved": False,
        })

    def _preserve(self, store: str, root: Path, *, fail: bool = False) -> Path:
        def extract(_archive: Path, destination: Path) -> None:
            destination.mkdir(parents=True)
            if destination.name == "package":
                (destination / "original").mkdir()
                (destination / "original/output.tar.gz").write_bytes(b"output")
                (destination / "original/output-manifest.json").write_text("{}")
                (destination / "support").mkdir()
                (destination / "support/source.bundle").write_bytes(b"bundle")

        def restore(_package: Path, scratch: Path) -> Path:
            session = scratch / "session"
            session.mkdir()
            return session

        def semantic(*_args: object) -> dict[str, object]:
            if fail:
                raise RuntimeError("semantic failure")
            return {"status": "passed_existing_g3_semantic_validator", "errors": []}

        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope), \
             patch("factory_ops.g3_primary_preservation._extract_tar", side_effect=extract), \
             patch("factory_ops.g3_primary_preservation._verify_manifest"), \
             patch("factory_ops.g3_primary_preservation._restore_session", side_effect=restore):
            return preserve_primary_run(
                prepared_path=self.prepared, evidence_root=self.evidence,
                store_root=root, store_id=store,
                storage_inventory=self.primary, storage_supplement_manifest=self.primary,
                storage_supplement_sha256=sha(self.primary), store_drive=f"{store}:",
                identity_resolver=lambda **_kwargs: f"device-{store}",
                source_validator=lambda *_args: None, semantic_validator=semantic,
            )

    def test_primary_import_does_not_require_actual_six_membership(self) -> None:
        alternate_run = "d" * 32
        archive = save(self.root / "archive.json", {})
        receipt = save(self.root / "receipt.json", {})
        bundle = self.root / "source.bundle"
        bundle.write_bytes(b"bundle")
        result = save(self.evidence / "alternate-trial-result.json", {
            "run_id": alternate_run, "trial_id": self.trial_id, "trial_outcome": "failed",
        })

        def prepare(**kwargs: object) -> Path:
            self.assertEqual([result], kwargs["primary_shared_support_files"])
            output = Path(str(kwargs["output"]))
            output.mkdir(parents=True)
            (output / "g3-raw-content-package-v1.tar").write_bytes(b"sealed-package")
            return save(output / "g3-raw-content-envelope-v1.json", {"fixture": True})

        record = {"run_id": alternate_run, "identity": {"profile": "primary",
                  "trial_id": self.trial_id}, "bindings": {"source_commit": self.source}}
        envelope = {**self.envelope, "run_id": alternate_run}
        with patch("factory_ops.g3_primary_preservation.load_archive_record", return_value=record), \
             patch("factory_ops.g3_primary_preservation.resolve_primary_shared_support",
                   return_value=([], [])), \
             patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=envelope):
            prepared = import_primary_run(
                archive_record=archive, attempt_receipt=receipt, source_bundle=bundle,
                primary_inputs_path=self.primary, primary_inputs_sha256=sha(self.primary),
                trial_result_path=result, evidence_root=self.evidence,
                package_preparer=prepare,
            )
        value = json.loads(prepared.read_text())
        self.assertEqual((alternate_run, self.trial_id), (value["run_id"], value["trial_id"]))
        self.assertNotIn("actual_six_run_ids", value)

    def test_primary_only_shared_support_digest_binds_all_emitted_rows(self) -> None:
        rows = [{"sha256": "1" * 64}, {"sha256": "2" * 64}]
        expected = hashlib.sha256(("1" * 64 + "2" * 64).encode()).hexdigest()
        self.assertEqual(expected, _shared_support_pack_sha(rows))
        self.assertIsNone(_shared_support_pack_sha([]))

    def test_import_rechecks_frozen_inputs_after_package_publication(self) -> None:
        archive = save(self.root / "drift-archive.json", {})
        receipt = save(self.root / "drift-receipt.json", {})
        bundle = self.root / "drift-source.bundle"
        bundle.write_bytes(b"bundle")

        def prepare(**kwargs: object) -> Path:
            self.primary.write_text('{"changed":true}', encoding="utf-8")
            output = Path(str(kwargs["output"]))
            output.mkdir(parents=True)
            (output / "g3-raw-content-package-v1.tar").write_bytes(b"sealed-package")
            return save(output / "g3-raw-content-envelope-v1.json", {"fixture": True})

        record = {"run_id": "d" * 32, "identity": {"profile": "primary",
                  "trial_id": self.trial_id}, "bindings": {"source_commit": self.source}}
        result = save(self.evidence / "drift-result.json", {
            "run_id": "d" * 32, "trial_id": self.trial_id, "trial_outcome": "failed"})
        envelope = {**self.envelope, "run_id": "d" * 32}
        with patch("factory_ops.g3_primary_preservation.load_archive_record", return_value=record), \
             patch("factory_ops.g3_primary_preservation.resolve_primary_shared_support",
                   return_value=([], [])), \
             patch("factory_ops.g3_primary_preservation.load_raw_envelope", return_value=envelope), \
             self.assertRaisesRegex(ValueError, "changed during publication"):
            import_primary_run(
                archive_record=archive, attempt_receipt=receipt, source_bundle=bundle,
                primary_inputs_path=self.primary,
                primary_inputs_sha256=hashlib.sha256(b'{"canonical": true}').hexdigest(),
                trial_result_path=result, evidence_root=self.evidence,
                package_preparer=prepare,
            )

    def test_success_removes_only_owned_workspace_and_retains_package(self) -> None:
        result = self._preserve("D", self.store_d)
        value = json.loads(result.read_text())
        namespace = Path(value["persistent_namespace"])
        self.assertTrue((namespace / "g3-raw-content-package-v1.tar").is_file())
        self.assertFalse((namespace / "owned-primary-replay-workspace").exists())
        self.assertEqual("removed_after_verified_close", value["transient_disposition"])
        self.assertFalse(value["cleanup_authorized"])

    def test_collection_failure_uses_distinct_replay_contract(self) -> None:
        prepared = json.loads(self.prepared.read_text())
        prepared["attempt_status"] = "collection_failed"
        save(self.prepared, prepared)
        self.envelope["attempt_status"] = "collection_failed"

        def extract(_archive: Path, destination: Path) -> None:
            destination.mkdir(parents=True)
            if destination.name == "package":
                (destination / "original").mkdir()
                (destination / "original/output.tar.gz").write_bytes(b"output")
                (destination / "original/output-manifest.json").write_text("{}")
                (destination / "support").mkdir()
                (destination / "support/source.bundle").write_bytes(b"bundle")

        def restore(_package: Path, scratch: Path) -> Path:
            session = scratch / "session"
            session.mkdir()
            return session

        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope), \
             patch("factory_ops.g3_primary_preservation._extract_tar", side_effect=extract), \
             patch("factory_ops.g3_primary_preservation._verify_manifest"), \
             patch("factory_ops.g3_primary_preservation._restore_session", side_effect=restore):
            result = preserve_primary_run(
                prepared_path=self.prepared, evidence_root=self.evidence,
                store_root=self.store_d, store_id="D",
                storage_inventory=self.primary, storage_supplement_manifest=self.primary,
                storage_supplement_sha256=sha(self.primary), store_drive="D:",
                identity_resolver=lambda **_kwargs: "device-D",
                source_validator=lambda *_args: None,
                semantic_validator=lambda *_args: {
                    "status": "passed_collection_failure_contract", "errors": []},
            )
        value = json.loads(result.read_text())
        replay_ref = value["semantic_replay_report"]
        replay = json.loads((self.evidence / replay_ref["path"]).read_text())
        self.assertEqual("collection_failure", replay["replay_kind"])
        self.assertEqual("passed_collection_failure_contract", replay["status"])

    def test_failure_keeps_owned_workspace_and_writes_no_success_result(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "semantic failure"):
            self._preserve("D", self.store_d, fail=True)
        namespaces = list((self.store_d / "factory-ops-g3-primary" / self.run_id).iterdir())
        self.assertEqual(1, len(namespaces))
        self.assertTrue((namespaces[0] / "owned-primary-replay-workspace").is_dir())
        evidence_run = self.evidence / "primary-stores" / "D" / self.run_id
        self.assertEqual([], list(evidence_run.glob("*/store-run-result.json")))
        failures = list(evidence_run.glob("*/preserve-failure-*.json"))
        self.assertEqual(1, len(failures))
        self.assertEqual([], list(evidence_run.glob("*/verified-close-receipt.json")))
        result_e = self._preserve("E", self.store_e)
        self.assertTrue(result_e.is_file())
        retry_d = self._preserve("D", self.store_d)
        self.assertTrue(retry_d.is_file())
        reused_d = self._preserve("D", self.store_d)
        self.assertEqual(retry_d, reused_d)

    def test_reuse_rehashes_envelope_replay_and_verified_close_receipt(self) -> None:
        result_path = self._preserve("D", self.store_d)
        value = json.loads(result_path.read_text())
        envelope = Path(value["raw_envelope"]["path"])
        envelope.unlink()
        with self.assertRaises((FileNotFoundError, ValueError)):
            self._preserve("D", self.store_d)

        # Restoring the exact envelope then proves a rehashed reference cannot
        # turn an empty semantic report into a valid closed attempt.
        save(envelope, {"fixture": True})
        self.assertEqual(value["raw_envelope"]["sha256"], sha(envelope))
        value = json.loads(result_path.read_text())
        replay = self.evidence / value["semantic_replay_report"]["path"]
        replay.write_text("{}", encoding="utf-8")
        value["semantic_replay_report"]["sha256"] = sha(replay)
        result_path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "semantic replay"):
            self._preserve("D", self.store_d)

    @unittest.skipUnless(__import__("os").name == "posix", "mount namespace requires POSIX")
    def test_isolated_replay_mounts_literal_root_and_uses_selected_source_origin(self) -> None:
        literal_root = self.root / "literal-root"
        artifact_literal = literal_root / "results/run"
        session_literal = literal_root / "sessions/run"
        artifact_literal.parent.mkdir(parents=True)
        session_literal.parent.mkdir(parents=True)
        trial = literal_root / "inputs/trial.json"
        config = literal_root / "inputs/config.json"
        start = literal_root / "resources/attempt-start.json"
        receipt = literal_root / "resources/attempt-receipt.json"
        expected_result = {"run_id": self.run_id, "trial_outcome": "failed",
                           "metrics_state": "unknown"}
        save(trial, {"trial_id": self.trial_id})
        save(config, {"config": True})
        save(start, {"references": {"config": {"path": config.relative_to(literal_root).as_posix()}}})
        save(receipt, {"references": {"start": {
            "path": start.relative_to(literal_root).as_posix()}},
            "expected_result": expected_result})
        primary = literal_root / "inputs/primary.json"
        save(primary, {"trial_path": str(trial)})
        submitted_result = save(literal_root / "primary-results/submitted-result.json",
                                expected_result)
        package_root = self.root / "restored-package"
        rows = []
        for index, original in enumerate((primary, trial, config, start, receipt,
                                          submitted_result)):
            member = package_root / f"support/per-trial/{index:06d}-{original.name}"
            member.parent.mkdir(parents=True, exist_ok=True)
            member.write_bytes(original.read_bytes())
            rows.append({"path": str(original),
                         "member": member.relative_to(package_root).as_posix(),
                         "bytes": original.stat().st_size, "sha256": sha(original)})
        save(package_root / "support/support-manifest.json", {
            "categories": {"per-trial": rows},
        })
        save(package_root / "original/archive-record.json", {"literal_paths": {
            "evidence_root": str(literal_root), "artifact_root": str(artifact_literal),
            "session_root": str(session_literal),
            "attempt_receipt_path": str(receipt),
        }})
        artifact = self.root / "artifact"
        session = self.root / "session"
        scratch = self.root / "scratch"
        artifact.mkdir()
        session.mkdir()
        scratch.mkdir()
        repository = self.root / "native-source"
        repository.mkdir()
        shutil.copytree(Path(__file__).parents[1] / "factory_ops", repository / "factory_ops")
        (repository / "factory_ops/g3_supervisor.py").write_text(
            "import json\n"
            "def build_collection_failure_result(trial, receipt_path, evidence_root, **kwargs):\n"
            "    return json.loads(receipt_path.read_text())['expected_result']\n",
            encoding="utf-8",
        )
        (repository / "factory_ops/g3_trial_result.py").write_text(
            "def build_trial_result(*args, **kwargs):\n"
            "    raise RuntimeError('collected builder was not selected')\n",
            encoding="utf-8",
        )
        subprocess.run(("git", "init"), cwd=repository, check=True,
                       capture_output=True, text=True)
        subprocess.run(("git", "config", "user.email", "fixture@example.invalid"),
                       cwd=repository, check=True)
        subprocess.run(("git", "config", "user.name", "fixture"),
                       cwd=repository, check=True)
        subprocess.run(("git", "add", "factory_ops"), cwd=repository, check=True)
        subprocess.run(("git", "commit", "-m", "fixture"), cwd=repository, check=True,
                       capture_output=True, text=True)
        commit = subprocess.run(("git", "rev-parse", "HEAD"), cwd=repository,
                                check=True, capture_output=True, text=True).stdout.strip()
        bundle = self.root / "source.bundle"
        subprocess.run(("git", "bundle", "create", str(bundle), "HEAD"), cwd=repository,
                       check=True, capture_output=True, text=True)
        checkout = self.root / "restored-source"
        _restore_source_checkout(bundle, checkout, commit)
        result = _default_semantic(
            artifact, session, self.run_id, commit, primary, sha(primary),
            "collection_failure", package_root, checkout, scratch, submitted_result,
        )
        self.assertEqual("passed_collection_failure_contract", result["status"])
        self.assertEqual({"trial_path": str(trial)}, json.loads(primary.read_text()))
        support_path = package_root / "support/support-manifest.json"
        support_doc = json.loads(support_path.read_text())
        support_doc["categories"]["per-trial"] = [
            row for row in support_doc["categories"]["per-trial"]
            if row["path"] != str(submitted_result)
        ]
        save(support_path, support_doc)
        negative_scratch = self.root / "negative-scratch"
        negative_scratch.mkdir()
        with self.assertRaises((FileNotFoundError, ValueError)):
            _default_semantic(
                artifact, session, self.run_id, commit, primary, sha(primary),
                "collection_failure", package_root, checkout, negative_scratch,
                submitted_result,
            )

    def test_close_binds_matching_d_e_and_chain_then_rejects_corruption(self) -> None:
        result_d = self._preserve("D", self.store_d)
        result_e = self._preserve("E", self.store_e)
        live = self._live(self.root / "live.json", ordinal=1,
                          trial_id=self.trial_id, primary=self.primary)
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope):
            closed = close_primary_run(
                output=self.evidence / "closed-001.json", evidence_root=self.evidence,
                ordinal=1, trial_id=self.trial_id, primary_inputs_path=self.primary,
                primary_inputs_sha256=sha(self.primary), expected_source_commit=self.source,
                expected_host_commit=self.host, live_eligibility_path=live,
                live_eligibility_sha256=sha(live), store_result_paths=[result_d, result_e],
                store_roots={"D": self.store_d, "E": self.store_e},
                stopped_storage_hours=0.5,
            )
        value = json.loads(closed.read_text())
        self.assertEqual(["D", "E"], [row["store_id"] for row in value["preservations"]])
        self.assertEqual("removed_after_verified_close", value["transient_disposition"])
        tampered = json.loads(result_e.read_text())
        tampered["trial_outcome"] = "completed"
        result_e.write_text(json.dumps(tampered), encoding="utf-8")
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope), self.assertRaises(ValueError):
            close_primary_run(
                output=self.evidence / "closed-002.json", evidence_root=self.evidence,
                ordinal=1, trial_id=self.trial_id, primary_inputs_path=self.primary,
                primary_inputs_sha256=sha(self.primary), expected_source_commit=self.source,
                expected_host_commit=self.host, live_eligibility_path=live,
                live_eligibility_sha256=sha(live), store_result_paths=[result_d, result_e],
                store_roots={"D": self.store_d, "E": self.store_e},
                stopped_storage_hours=0.5,
            )

    def test_close_rejects_current_results_with_forged_live_device_identities(self) -> None:
        result_d = self._preserve("D", self.store_d)
        result_e = self._preserve("E", self.store_e)
        for path, forged in ((result_d, "forged-device-D"),
                             (result_e, "forged-device-E")):
            value = json.loads(path.read_text())
            receipt = self.evidence / value["preservation_receipt"]["path"]
            receipt_value = json.loads(receipt.read_text())
            receipt_value["device_identity"] = forged
            save(receipt, receipt_value)
            value["device_identity"] = forged
            value["preservation_receipt"]["sha256"] = sha(receipt)
            save(path, value)
        live = self._live(self.root / "identity-live.json", ordinal=1,
                          trial_id=self.trial_id, primary=self.primary)
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope), \
             self.assertRaisesRegex(ValueError, "expected binding"):
            close_primary_run(
                output=self.evidence / "closed-forged-identity.json",
                evidence_root=self.evidence, ordinal=1, trial_id=self.trial_id,
                primary_inputs_path=self.primary, primary_inputs_sha256=sha(self.primary),
                expected_source_commit=self.source, expected_host_commit=self.host,
                live_eligibility_path=live, live_eligibility_sha256=sha(live),
                store_result_paths=[result_d, result_e],
                store_roots={"D": self.store_d, "E": self.store_e},
                stopped_storage_hours=0.0,
            )

    def test_close_rejects_foreign_previous_and_first_receipt_sha(self) -> None:
        trial_two = save(self.root / "trial-two.json", self.matrix[1])
        primary_two = save(self.root / "primary-two.json", {
            "trial_path": str(trial_two),
            "calibration_path": str(self.root / "calibration.json"),
            "pilot_slots": {"fixture": {}}, "resource_input": {"fixture": True},
            "expected_source_commit": self.source,
        })
        foreign = save(self.evidence / "foreign-previous.json", {
            "schema_version": 2, "classification": "g3_closed_primary_preservation_receipt",
            "ordinal": 1, "trial_id": self.matrix[0]["trial_id"],
            "source_commit": "d" * 40, "host_source_commit": "e" * 40,
            "previous_receipt_sha256": "f" * 64,
        })
        live = self._live(self.root / "live.json", ordinal=2,
                          trial_id=self.matrix[1]["trial_id"], primary=primary_two,
                          previous_receipts=[{
                              "path": str(foreign), "sha256": sha(foreign), "ordinal": 1,
                              "trial_id": self.matrix[0]["trial_id"], "run_id": self.run_id,
                              "primary_inputs_sha256": sha(self.primary),
                          }])
        with self.assertRaisesRegex(ValueError, "chained primary receipt closure"):
            close_primary_run(
                output=self.evidence / "closed-foreign.json", evidence_root=self.evidence,
                ordinal=2, trial_id=self.matrix[1]["trial_id"],
                primary_inputs_path=primary_two, primary_inputs_sha256=sha(primary_two),
                expected_source_commit=self.source, expected_host_commit=self.host,
                live_eligibility_path=live, live_eligibility_sha256=sha(live),
                store_result_paths=[], stopped_storage_hours=0.0,
                store_roots={"D": self.store_d, "E": self.store_e},
                previous_receipt=foreign, previous_receipt_sha256=sha(foreign),
            )
        with self.assertRaisesRegex(ValueError, "first primary closure"):
            live_first = self._live(self.root / "live-first.json", ordinal=1,
                                    trial_id=self.trial_id, primary=self.primary)
            close_primary_run(
                output=self.evidence / "closed-first-with-sha.json", evidence_root=self.evidence,
                ordinal=1, trial_id=self.trial_id, primary_inputs_path=self.primary,
                primary_inputs_sha256=sha(self.primary), expected_source_commit=self.source,
                expected_host_commit=self.host, live_eligibility_path=live_first,
                live_eligibility_sha256=sha(live_first), store_result_paths=[],
                store_roots={"D": self.store_d, "E": self.store_e},
                stopped_storage_hours=0.0, previous_receipt_sha256="f" * 64,
            )

    def test_close_accepts_complete_previous_chain_before_current_store_gate(self) -> None:
        result_d = self._preserve("D", self.store_d)
        result_e = self._preserve("E", self.store_e)
        live_one = self._live(self.root / "live-one.json", ordinal=1,
                              trial_id=self.trial_id, primary=self.primary)
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope):
            closed_one = close_primary_run(
                output=self.evidence / "closed-chain-001.json", evidence_root=self.evidence,
                ordinal=1, trial_id=self.trial_id, primary_inputs_path=self.primary,
                primary_inputs_sha256=sha(self.primary), expected_source_commit=self.source,
                expected_host_commit=self.host, live_eligibility_path=live_one,
                live_eligibility_sha256=sha(live_one),
                store_result_paths=[result_d, result_e], stopped_storage_hours=0.5,
                store_roots={"D": self.store_d, "E": self.store_e},
            )
        trial_two = save(self.root / "chain-trial-two.json", self.matrix[1])
        primary_two = save(self.root / "chain-primary-two.json", {
            "trial_path": str(trial_two),
            "calibration_path": str(self.root / "calibration.json"),
            "pilot_slots": {"fixture": {}}, "resource_input": {"fixture": True},
            "expected_source_commit": self.source,
        })
        chain_reference = {
            "path": str(closed_one), "sha256": sha(closed_one), "ordinal": 1,
            "trial_id": self.trial_id, "run_id": self.run_id,
            "primary_inputs_sha256": sha(self.primary),
        }
        live_two = self._live(
            self.root / "live-two.json", ordinal=2,
            trial_id=self.matrix[1]["trial_id"], primary=primary_two,
            previous_receipts=[chain_reference],
        )
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope), \
             self.assertRaisesRegex(ValueError, "requires D and E"):
            close_primary_run(
                output=self.evidence / "closed-chain-002.json", evidence_root=self.evidence,
                ordinal=2, trial_id=self.matrix[1]["trial_id"],
                primary_inputs_path=primary_two, primary_inputs_sha256=sha(primary_two),
                expected_source_commit=self.source, expected_host_commit=self.host,
                live_eligibility_path=live_two, live_eligibility_sha256=sha(live_two),
                store_result_paths=[], stopped_storage_hours=0.0,
                store_roots={"D": self.store_d, "E": self.store_e},
                previous_receipt=closed_one, previous_receipt_sha256=sha(closed_one),
            )

        spliced = dict(chain_reference)
        spliced["primary_inputs_sha256"] = "f" * 64
        spliced_live = self._live(
            self.root / "live-two-spliced.json", ordinal=2,
            trial_id=self.matrix[1]["trial_id"], primary=primary_two,
            previous_receipts=[spliced],
        )
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   return_value=self.envelope), \
             self.assertRaisesRegex(ValueError, "chained primary receipt reference differs"):
            close_primary_run(
                output=self.evidence / "closed-chain-spliced.json", evidence_root=self.evidence,
                ordinal=2, trial_id=self.matrix[1]["trial_id"],
                primary_inputs_path=primary_two, primary_inputs_sha256=sha(primary_two),
                expected_source_commit=self.source, expected_host_commit=self.host,
                live_eligibility_path=spliced_live,
                live_eligibility_sha256=sha(spliced_live), store_result_paths=[],
                store_roots={"D": self.store_d, "E": self.store_e},
                stopped_storage_hours=0.0, previous_receipt=closed_one,
                previous_receipt_sha256=sha(closed_one),
            )

    def test_close_revalidates_complete_ordinal_three_chain(self) -> None:
        envelopes = {self.run_id: self.envelope}

        def reference(path: Path) -> dict[str, object]:
            value = json.loads(path.read_text())
            return {
                "path": str(path), "sha256": sha(path), "ordinal": value["ordinal"],
                "trial_id": value["trial_id"], "run_id": value["run_id"],
                "primary_inputs_sha256": value["primary_inputs_sha256"],
            }

        def reset(index: int, character: str) -> None:
            self.run_id = character * 32
            self.trial_id = self.matrix[index]["trial_id"]
            self.trial = save(self.root / f"deep-trial-{index + 1}.json", self.matrix[index])
            self.primary = save(self.root / f"deep-primary-{index + 1}.json", {
                "trial_path": str(self.trial),
                "calibration_path": str(self.root / "calibration.json"),
                "pilot_slots": {"fixture": {}}, "resource_input": {"fixture": True},
                "expected_source_commit": self.source,
            })
            self.result = save(self.evidence / f"deep-trial-result-{index + 1}.json", {
                "run_id": self.run_id, "trial_id": self.trial_id,
                "trial_outcome": "failed",
            })
            self.envelope_path = save(
                self.evidence / f"deep-raw-{index + 1}" / "g3-raw-content-envelope-v1.json",
                {"fixture": True},
            )
            self.package = self.envelope_path.parent / "g3-raw-content-package-v1.tar"
            self.package.write_bytes(f"sealed-package-{index + 1}".encode())
            self.envelope = {
                "run_id": self.run_id, "trial_id": self.trial_id, "profile": "primary",
                "source_commit": self.source, "attempt_status": "collected",
                "package": {"bytes": self.package.stat().st_size,
                            "sha256": sha(self.package)},
            }
            envelopes[self.run_id] = self.envelope
            self.prepared = save(
                self.evidence / "primary-prepared" / self.run_id / "prepared-primary-run.json",
                {
                    "schema_version": 1,
                    "classification": "g3_prepared_primary_preservation",
                    "run_id": self.run_id, "trial_id": self.trial_id,
                    "source_commit": self.source, "trial_outcome": "failed",
                    "primary_inputs_path": str(self.primary),
                    "primary_inputs_sha256": sha(self.primary),
                    "trial_result": {
                        "path": self.result.relative_to(self.evidence).as_posix(),
                        "sha256": sha(self.result),
                    },
                    "raw_envelope": {
                        "path": self.envelope_path.relative_to(self.evidence).as_posix(),
                        "sha256": sha(self.envelope_path),
                    },
                    "package": {"path": str(self.package),
                                "bytes": self.package.stat().st_size,
                                "sha256": sha(self.package)},
                    "attempt_status": "collected", "cleanup_authorized": False,
                },
            )

        def envelope_loader(path: Path) -> dict[str, object]:
            for run_id, envelope in envelopes.items():
                if run_id in Path(path).parts:
                    return envelope
            raise ValueError("fixture envelope run is absent")

        def close(index: int, history: list[Path], result_d: Path,
                  result_e: Path) -> Path:
            live = self._live(
                self.root / f"deep-live-{index + 1}.json", ordinal=index + 1,
                trial_id=self.trial_id, primary=self.primary,
                previous_receipts=[reference(path) for path in history],
            )
            with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                       side_effect=envelope_loader):
                return close_primary_run(
                    output=self.evidence / f"deep-closed-{index + 1}.json",
                    evidence_root=self.evidence, ordinal=index + 1,
                    trial_id=self.trial_id, primary_inputs_path=self.primary,
                    primary_inputs_sha256=sha(self.primary),
                    expected_source_commit=self.source, expected_host_commit=self.host,
                    live_eligibility_path=live, live_eligibility_sha256=sha(live),
                    store_result_paths=[result_d, result_e],
                    store_roots={"D": self.store_d, "E": self.store_e},
                    stopped_storage_hours=0.0,
                    previous_receipt=history[-1] if history else None,
                    previous_receipt_sha256=sha(history[-1]) if history else None,
                )

        result_d1 = self._preserve("D", self.store_d)
        result_e1 = self._preserve("E", self.store_e)
        closed_one = close(0, [], result_d1, result_e1)
        reset(1, "d")
        result_d2 = self._preserve("D", self.store_d)
        result_e2 = self._preserve("E", self.store_e)
        closed_two = close(1, [closed_one], result_d2, result_e2)
        reset(2, "e")
        result_d3 = self._preserve("D", self.store_d)
        result_e3 = self._preserve("E", self.store_e)
        close(2, [closed_one, closed_two], result_d3, result_e3)

        oldest = json.loads(closed_one.read_text())
        oldest_d = Path(oldest["preservations"][0]["store_result_path"])
        oldest_result = json.loads(oldest_d.read_text())
        Path(oldest_result["raw_envelope"]["path"]).unlink()
        corrupt_live = self._live(
            self.root / "deep-live-3-corrupt.json", ordinal=3,
            trial_id=self.trial_id, primary=self.primary,
            previous_receipts=[reference(closed_one), reference(closed_two)],
        )
        with patch("factory_ops.g3_primary_preservation.load_raw_envelope",
                   side_effect=envelope_loader), \
             self.assertRaisesRegex((FileNotFoundError, ValueError), "envelope"):
            close_primary_run(
                output=self.evidence / "deep-closed-3-corrupt.json",
                evidence_root=self.evidence, ordinal=3, trial_id=self.trial_id,
                primary_inputs_path=self.primary, primary_inputs_sha256=sha(self.primary),
                expected_source_commit=self.source, expected_host_commit=self.host,
                live_eligibility_path=corrupt_live,
                live_eligibility_sha256=sha(corrupt_live), store_result_paths=[],
                store_roots={"D": self.store_d, "E": self.store_e},
                stopped_storage_hours=0.0, previous_receipt=closed_two,
                previous_receipt_sha256=sha(closed_two),
            )


if __name__ == "__main__":
    unittest.main()
