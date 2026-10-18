from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.g3_primary_preservation import (
    _default_semantic, _restore_source_checkout, close_primary_run, import_primary_run,
    preserve_primary_run,
)


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
        self.trial_id = "normal-fifo-seed-0"
        self.primary = save(self.root / "primary.json", {"canonical": True})
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
            self.assertEqual([], kwargs["primary_shared_support_files"])
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
        result_e = self._preserve("E", self.store_e)
        self.assertTrue(result_e.is_file())
        retry_d = self._preserve("D", self.store_d)
        self.assertTrue(retry_d.is_file())
        reused_d = self._preserve("D", self.store_d)
        self.assertEqual(retry_d, reused_d)

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
        submitted_result = save(self.root / "submitted-result.json", expected_result)
        package_root = self.root / "restored-package"
        rows = []
        for index, original in enumerate((primary, trial, config, start, receipt)):
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

    def test_close_binds_matching_d_e_and_chain_then_rejects_corruption(self) -> None:
        result_d = self._preserve("D", self.store_d)
        result_e = self._preserve("E", self.store_e)
        live = save(self.root / "live.json", {"eligible": True})
        closed = close_primary_run(
            output=self.evidence / "closed-001.json", evidence_root=self.evidence,
            ordinal=1, trial_id=self.trial_id, primary_inputs_path=self.primary,
            primary_inputs_sha256=sha(self.primary), expected_source_commit=self.source,
            expected_host_commit=self.host, live_eligibility_path=live,
            live_eligibility_sha256=sha(live), store_result_paths=[result_d, result_e],
            stopped_storage_hours=0.5,
        )
        value = json.loads(closed.read_text())
        self.assertEqual(["D", "E"], [row["store_id"] for row in value["preservations"]])
        self.assertEqual("removed_after_verified_close", value["transient_disposition"])
        tampered = json.loads(result_e.read_text())
        tampered["trial_outcome"] = "completed"
        result_e.write_text(json.dumps(tampered), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "outcome or package"):
            close_primary_run(
                output=self.evidence / "closed-002.json", evidence_root=self.evidence,
                ordinal=1, trial_id=self.trial_id, primary_inputs_path=self.primary,
                primary_inputs_sha256=sha(self.primary), expected_source_commit=self.source,
                expected_host_commit=self.host, live_eligibility_path=live,
                live_eligibility_sha256=sha(live), store_result_paths=[result_d, result_e],
                stopped_storage_hours=0.5,
            )


if __name__ == "__main__":
    unittest.main()
