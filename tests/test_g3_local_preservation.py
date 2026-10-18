from __future__ import annotations

import hashlib
from io import BytesIO
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import time
import unittest

from factory_ops.g3_local_preservation import (_measure_shared_paths, _verify_source_bundle,
                                                finalize_plan, finalize_store, prepare_run,
                                                preserve_run)
from factory_ops.g3_raw_storage import (
    ENVELOPE_NAME,
    PACKAGE_NAME,
    SCHEMA,
    SUPPORT_SCHEMA,
    _LIMITATIONS,
    _PROOF_FLAGS,
    _json_bytes,
    load_raw_envelope,
)


def digest_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def add_bytes(packed: tarfile.TarFile, name: str, payload: bytes, *, link: str | None = None) -> None:
    info = tarfile.TarInfo(name)
    info.size = len(payload)
    info.mode = 0o600
    info.mtime = 0
    if link is not None:
        info.type = tarfile.SYMTYPE
        info.linkname = link
        info.size = 0
    packed.addfile(info, BytesIO(payload) if link is None else None)


def make_package(root: Path, *, run_id: str, source_commit: str, index: int = 0,
                 output_link: bool = False) -> Path:
    package_root = root / f"package-{index}"
    package_root.mkdir()
    artifact_files = {
        "g3-evidence.json": _json_bytes({"run_id": run_id, "passed": True}),
        "mission-result.json": _json_bytes({"run_id": run_id, "completed": True}),
    }
    manifest = {"schema_version": 1, "files": [
        {"path": name, "bytes": len(payload), "sha256": digest_bytes(payload)}
        for name, payload in sorted(artifact_files.items())
    ]}
    manifest_payload = _json_bytes(manifest)
    archive_buffer = BytesIO()
    with tarfile.open(fileobj=archive_buffer, mode="w:") as packed:
        for name, payload in artifact_files.items():
            add_bytes(packed, name, payload, link="target" if output_link and name == "mission-result.json" else None)
    archive_payload = archive_buffer.getvalue()
    original_session = root / f"original-session-{index}"
    original_session.mkdir()
    session_path = original_session / "session.json"
    session_payload = _json_bytes({"run_id": run_id, "profile": "pilot"})
    session_path.write_bytes(session_payload)
    record_payload = _json_bytes({
        "schema_version": 2,
        "literal_paths": {"session_root": str(original_session)},
    })
    source_payload = f"source-{index}".encode()
    receipt_payload = f"receipt-{index}".encode()
    storage_attempt_id = f"{index + 200:032x}"
    session_member = "support/session/000000-session.json"
    session_row = {"path": str(session_path), "bytes": len(session_payload),
                   "sha256": digest_bytes(session_payload), "device": 1, "inode": 1,
                   "links": 1, "member": session_member}
    support = {"schema": SUPPORT_SCHEMA, "storage_attempt_id": storage_attempt_id,
               "categories": {"source": [], "session": [session_row], "per-trial": [],
                              "shared-six-pilot": []}}
    support_payload = _json_bytes(support)
    per_trial_sha = digest_bytes(_json_bytes({
        "schema": SUPPORT_SCHEMA, "category": "per-trial", "files": []}))
    internal = {
        "schema": SCHEMA, "kind": "raw_content_package", "storage_attempt_id": storage_attempt_id,
        "source_commit": source_commit, "run_label": "g3-p-normal-fifo-001", "run_id": run_id,
        "profile": "pilot", "condition": "normal", "policy": "fifo", "seed": 100,
        "trial_id": None, "archive_record_sha256": digest_bytes(record_payload),
        "original_archive_bytes": len(archive_payload),
        "original_archive_sha256": digest_bytes(archive_payload),
        "output_manifest_bytes": len(manifest_payload),
        "output_manifest_sha256": digest_bytes(manifest_payload),
        "output_tree_sha256": "1" * 64, "file_count": len(artifact_files),
        "raw_bytes": sum(len(payload) for payload in artifact_files.values()),
        "attempt_receipt_sha256": digest_bytes(receipt_payload),
        "support_manifest_sha256": digest_bytes(support_payload),
        "per_trial_support_manifest_sha256": per_trial_sha,
        "shared_six_pilot_support_pack_sha256": digest_bytes(b""),
        "source_bundle_sha256": digest_bytes(source_payload), "attempt_status": "collected",
        "limitations": list(_LIMITATIONS), "proof_flags": dict(_PROOF_FLAGS),
        "qualification_no_billing_attempt_receipt_absent": False,
    }
    members = {
        "original/output.tar.gz": archive_payload,
        "original/output-manifest.json": manifest_payload,
        "original/archive-record.json": record_payload,
        "support/source.bundle": source_payload,
        "support/attempt-receipt.json": receipt_payload,
        "support/support-manifest.json": support_payload,
        session_member: session_payload,
        "envelope/g3-raw-content-envelope-v1.json": _json_bytes(internal),
    }
    package_path = package_root / PACKAGE_NAME
    with tarfile.open(package_path, "w:") as packed:
        for name, payload in members.items():
            add_bytes(packed, name, payload)
    envelope = {**internal, "package": {"name": PACKAGE_NAME,
                "bytes": package_path.stat().st_size, "sha256": digest(package_path),
                "member_count": len(members)}}
    envelope_path = package_root / ENVELOPE_NAME
    envelope_path.write_bytes(_json_bytes(envelope))
    return envelope_path


@unittest.skipUnless(os.name == "posix", "local preservation uses POSIX pinned-file semantics")
class LocalPreservationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.evidence = self.root / "evidence"
        self.evidence.mkdir()
        self.store = self.root / "store"
        self.store.mkdir()
        self.commit = "a" * 40
        self.config = "b" * 64
        self.run_ids = [f"{index + 1:032x}" for index in range(6)]
        self.identity_inputs = {
            "storage_inventory": self.root / "fixture-inventory.json",
            "storage_supplement_manifest": self.root / "fixture-supplement.json",
            "storage_supplement_sha256": "f" * 64,
            "store_drive": "D:",
            "identity_resolver": lambda **_: "fixture-only-device-D",
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def prepare(self, index: int = 0, *, output_link: bool = False,
                package_delay_s: float = 0.0) -> Path:
        envelope = make_package(self.root, run_id=self.run_ids[index], source_commit=self.commit,
                                index=index, output_link=output_link)
        self.assertEqual(self.run_ids[index], load_raw_envelope(envelope)["run_id"])

        def fixture_preparer(**kwargs) -> Path:
            if package_delay_s:
                time.sleep(package_delay_s)
            output = Path(kwargs["output"])
            shutil.copytree(envelope.parent, output)
            return output / ENVELOPE_NAME

        return prepare_run(
            archive_record=envelope, attempt_receipt=envelope, source_bundle=envelope,
            shared_support_files=[], evidence_root=self.evidence, source_commit=self.commit,
            experiment_config_sha256=self.config, actual_six_run_ids=self.run_ids,
            package_preparer=fixture_preparer,
        )

    def test_actual_package_is_copied_fully_restored_and_semantically_checked(self) -> None:
        prepared = self.prepare(package_delay_s=.02)
        self.assertGreaterEqual(json.loads(prepared.read_text())["package_phase_wall_time_s"], .015)
        calls = []

        def semantic(artifact: Path, session: Path, run_id: str, commit: str) -> None:
            calls.append((run_id, commit))
            self.assertTrue((artifact / "g3-evidence.json").is_file())
            self.assertEqual(self.run_ids[0], json.loads((session / "session.json").read_text())["run_id"])

        def delayed_loader(path: Path) -> dict:
            if self.store in path.parents:
                time.sleep(.02)
            return load_raw_envelope(path)

        result = preserve_run(
            prepared_path=prepared, evidence_root=self.evidence, store_root=self.store,
            store_id="D", semantic_validator=semantic, source_validator=lambda *_: None,
            envelope_loader=delayed_loader, **self.identity_inputs,
        )
        value = json.loads(result.read_text())
        self.assertEqual([(self.run_ids[0], self.commit)], calls)
        self.assertFalse(value["cleanup_authorized"])
        self.assertGreaterEqual(value["offload_phase_wall_time_s"], .015)
        self.assertTrue((self.store / "factory-ops-g3" / self.run_ids[0] / "full-restore"
                         / "artifact" / "mission-result.json").is_file())
        semantic_path = self.evidence / value["semantic_replay_report"]["path"]
        self.assertEqual("passed_existing_g3_semantic_validator",
                         json.loads(semantic_path.read_text())["status"])

    def test_prepared_evidence_tree_remains_relocatable_after_vm_handoff(self) -> None:
        prepared = self.prepare(5)
        relative_prepared = prepared.relative_to(self.evidence)
        transported = self.root / "transported-evidence"
        shutil.copytree(self.evidence, transported)
        old_evidence = self.root / "vm-evidence-offline"
        self.evidence.rename(old_evidence)
        result = preserve_run(
            prepared_path=transported / relative_prepared, evidence_root=transported,
            store_root=self.store, store_id="D", semantic_validator=lambda *_: None,
            source_validator=lambda *_: None, **self.identity_inputs)
        self.assertTrue(result.is_file())
        value = json.loads(result.read_text())
        self.assertNotIn(str(old_evidence), json.dumps(value))

    def test_forged_prepared_identity_creates_no_store_or_receipt_namespace(self) -> None:
        prepared = self.prepare()
        original_prepared_bytes = prepared.read_bytes()
        original = json.loads(original_prepared_bytes)
        record_path = self.evidence / original["package_record"]["path"]
        closure_path = self.evidence / original["source_support_closure"]["path"]
        original_record_bytes = record_path.read_bytes()
        original_closure_bytes = closure_path.read_bytes()
        original_record = json.loads(original_record_bytes)
        original_closure = json.loads(original_closure_bytes)
        manifest_path = self.evidence / original_record["package_manifest"]["path"]
        original_manifest_bytes = manifest_path.read_bytes()
        original_manifest = json.loads(original_manifest_bytes)

        def identity_must_not_run(**_kwargs) -> str:
            self.fail("storage identity resolution ran before prepared evidence validation")

        def assert_rejected() -> None:
            with self.assertRaisesRegex(ValueError, "prepared (run|closure)"):
                preserve_run(
                    prepared_path=prepared, evidence_root=self.evidence,
                    store_root=self.store, store_id="D",
                    semantic_validator=lambda *_: None, source_validator=lambda *_: None,
                    **{**self.identity_inputs, "identity_resolver": identity_must_not_run})
            self.assertFalse((self.store / "factory-ops-g3").exists())
            self.assertFalse((self.evidence / "stores").exists())

        for field, forged in (
            ("run_id", "../outside"),
            ("run_id", "f" * 32),
            ("source_commit", "f" * 40),
        ):
            with self.subTest(field=field, forged=forged):
                prepared.write_text(json.dumps({**original, field: forged}), encoding="utf-8")
                assert_rejected()
        prepared.write_bytes(original_prepared_bytes)

        with self.subTest(forged="record source"):
            record_path.write_text(
                json.dumps({**original_record, "source_commit": "f" * 40}), encoding="utf-8")
            forged_prepared = {**original, "package_record": {
                **original["package_record"], "sha256": digest(record_path)}}
            prepared.write_text(json.dumps(forged_prepared), encoding="utf-8")
            assert_rejected()
        record_path.write_bytes(original_record_bytes)
        prepared.write_bytes(original_prepared_bytes)

        with self.subTest(forged="closure source"):
            closure_path.write_text(
                json.dumps({**original_closure, "source_commit": "f" * 40}), encoding="utf-8")
            forged_prepared = {**original, "source_support_closure": {
                **original["source_support_closure"], "sha256": digest(closure_path)}}
            prepared.write_text(json.dumps(forged_prepared), encoding="utf-8")
            assert_rejected()
        closure_path.write_bytes(original_closure_bytes)
        prepared.write_bytes(original_prepared_bytes)

        with self.subTest(forged="manifest source"):
            manifest_path.write_text(
                json.dumps({**original_manifest, "source_commit": "f" * 40}), encoding="utf-8")
            forged_record = {**original_record, "package_manifest": {
                **original_record["package_manifest"], "sha256": digest(manifest_path)}}
            record_path.write_text(json.dumps(forged_record), encoding="utf-8")
            forged_prepared = {**original, "package_record": {
                **original["package_record"], "sha256": digest(record_path)}}
            prepared.write_text(json.dumps(forged_prepared), encoding="utf-8")
            assert_rejected()
        manifest_path.write_bytes(original_manifest_bytes)
        record_path.write_bytes(original_record_bytes)
        prepared.write_bytes(original_prepared_bytes)

    def test_damage_link_and_counterfactual_semantic_failure_fail_closed(self) -> None:
        prepared = self.prepare()
        prepared_value = json.loads(prepared.read_text())
        copied_envelope = self.evidence / prepared_value["raw_envelope"]["path"]
        copied_package = copied_envelope.parent / PACKAGE_NAME
        copied_package.write_bytes(copied_package.read_bytes() + b"damage")
        with self.assertRaises(ValueError):
            preserve_run(prepared_path=prepared, evidence_root=self.evidence,
                         store_root=self.store, store_id="D", source_validator=lambda *_: None,
                         **self.identity_inputs)

        linked = self.root / "linked-envelope.json"
        original = make_package(self.root, run_id=self.run_ids[1], source_commit=self.commit, index=1)
        linked.symlink_to(original)
        with self.assertRaises(ValueError):
            prepare_run(
                archive_record=linked, attempt_receipt=linked, source_bundle=linked,
                shared_support_files=[], evidence_root=self.evidence,
                source_commit=self.commit, experiment_config_sha256=self.config,
                actual_six_run_ids=self.run_ids, package_preparer=lambda **_: linked)

        other_evidence = self.root / "counter-evidence"
        other_evidence.mkdir()
        other_store = self.root / "counter-store"
        other_store.mkdir()
        counter_source = make_package(
            self.root, run_id=self.run_ids[2], source_commit=self.commit, index=2)
        def copy_counter(**kwargs) -> Path:
            output = Path(kwargs["output"])
            shutil.copytree(counter_source.parent, output)
            return output / ENVELOPE_NAME
        counter = prepare_run(
            archive_record=counter_source, attempt_receipt=counter_source,
            source_bundle=counter_source, shared_support_files=[], evidence_root=other_evidence,
            source_commit=self.commit, experiment_config_sha256=self.config,
            actual_six_run_ids=self.run_ids, package_preparer=copy_counter)
        with self.assertRaisesRegex(RuntimeError, "counterfactual"):
            preserve_run(prepared_path=counter, evidence_root=other_evidence,
                         store_root=other_store, store_id="E",
                         semantic_validator=lambda *_: (_ for _ in ()).throw(RuntimeError("counterfactual")),
                         source_validator=lambda *_: None,
                         **{**self.identity_inputs, "store_drive": "E:",
                            "identity_resolver": lambda **_: "fixture-only-device-E"})
        failure_root = other_evidence / "stores" / "E" / self.run_ids[2]
        self.assertTrue(any(failure_root.glob("preserve-failure-*.json")))
        self.assertFalse((failure_root / "semantic-replay-report.json").exists())
        self.assertTrue((other_store / "factory-ops-g3" / self.run_ids[2]).is_dir())

        linked_evidence = self.root / "linked-output-evidence"
        linked_evidence.mkdir()
        unsafe_source = make_package(self.root, run_id=self.run_ids[3], source_commit=self.commit,
                                     index=3, output_link=True)
        def copy_unsafe(**kwargs) -> Path:
            output = Path(kwargs["output"])
            shutil.copytree(unsafe_source.parent, output)
            return output / ENVELOPE_NAME
        unsafe = prepare_run(
            archive_record=unsafe_source, attempt_receipt=unsafe_source,
            source_bundle=unsafe_source, shared_support_files=[], evidence_root=linked_evidence,
            source_commit=self.commit, experiment_config_sha256=self.config,
            actual_six_run_ids=self.run_ids, package_preparer=copy_unsafe)
        unsafe_store = self.root / "unsafe-store"
        unsafe_store.mkdir()
        with self.assertRaisesRegex(ValueError, "link"):
            preserve_run(prepared_path=unsafe, evidence_root=linked_evidence,
                         store_root=unsafe_store, store_id="D",
                         semantic_validator=lambda *_: None, source_validator=lambda *_: None,
                         **self.identity_inputs)

    def test_finalize_plan_rejects_same_physical_device_before_success_output(self) -> None:
        receipts = []
        for store in ("D", "E"):
            path = self.evidence / f"coordinator-{store}.json"
            path.write_text(json.dumps({"store_id": store, "device_identity": "posix-st_dev:7"}),
                            encoding="utf-8")
            receipts.append(path)
        with self.assertRaisesRegex(ValueError, "must be distinct"):
            finalize_plan(
                evidence_root=self.evidence, coordinator_receipts=receipts, measurements=[], billing={},
                source_commit=self.commit, experiment_config_sha256=self.config,
                supervisor_sha256="c" * 64, requested_primary_runs=60, reserve_fraction=.2,
                stopped_storage_hourly_rate_usd=.05, planned_stopped_hours=1,
            )
        self.assertFalse((self.evidence / "sequential-capacity-plan.json").exists())

    def test_full_source_bundle_clone_and_fsck_boundary_is_real(self) -> None:
        repository = self.root / "source-repository"
        repository.mkdir()
        subprocess.run(("git", "init", "-q"), cwd=repository, check=True)
        subprocess.run(("git", "config", "user.email", "fixture@example.invalid"),
                       cwd=repository, check=True)
        subprocess.run(("git", "config", "user.name", "Fixture"), cwd=repository, check=True)
        (repository / "source.txt").write_text("source\n", encoding="utf-8")
        subprocess.run(("git", "add", "source.txt"), cwd=repository, check=True)
        subprocess.run(("git", "commit", "-qm", "source"), cwd=repository, check=True)
        commit = subprocess.check_output(("git", "rev-parse", "HEAD"), cwd=repository,
                                         text=True).strip()
        bundle = self.root / "source.bundle"
        subprocess.run(("git", "bundle", "create", str(bundle), "--all"),
                       cwd=repository, check=True)
        restored = self.root / "restored-source.git"
        _verify_source_bundle(bundle, restored, commit)
        self.assertTrue((restored / "HEAD").is_file())
        with self.assertRaises(subprocess.CalledProcessError):
            _verify_source_bundle(bundle, self.root / "wrong-source.git", "f" * 40)

    def test_source_validation_failure_keeps_restore_but_publishes_no_success_receipts(self) -> None:
        prepared = self.prepare(4)
        semantic_called = False

        def semantic(*_args) -> None:
            nonlocal semantic_called
            semantic_called = True

        def reject_source(*_args) -> None:
            raise subprocess.CalledProcessError(1, ("git", "fsck", "--full", "--strict"))

        with self.assertRaises(subprocess.CalledProcessError):
            preserve_run(
                prepared_path=prepared, evidence_root=self.evidence, store_root=self.store,
                store_id="D", semantic_validator=semantic, source_validator=reject_source,
                **self.identity_inputs)
        receipt_root = self.evidence / "stores" / "D" / self.run_ids[4]
        self.assertFalse(semantic_called)
        self.assertFalse((receipt_root / "full-restore-receipt.json").exists())
        self.assertFalse((receipt_root / "semantic-replay-report.json").exists())
        self.assertFalse((receipt_root / "store-run-result.json").exists())
        self.assertTrue(any(receipt_root.glob("preserve-failure-*.json")))
        self.assertTrue((self.store / "factory-ops-g3" / self.run_ids[4]
                         / "full-restore" / "artifact" / "g3-evidence.json").is_file())

    def test_finalize_store_measures_real_files_and_rejects_inode_overlap(self) -> None:
        referenced = self.evidence / "referenced.json"
        referenced.write_text("{}\n", encoding="utf-8")
        reference = {"path": referenced.relative_to(self.evidence).as_posix(),
                     "sha256": digest(referenced)}
        results = []
        restore_roots = []
        for index, run_id in enumerate(self.run_ids):
            namespace = self.store / "factory-ops-g3" / run_id
            restore = namespace / "full-restore"
            restore.mkdir(parents=True)
            (restore / "payload.bin").write_bytes(bytes([index]) * (index + 1))
            restore_roots.append(str(restore))
            result = (self.evidence / "stores" / "D" / run_id
                      / "store-run-result.json")
            result.parent.mkdir(parents=True)
            result.write_text(json.dumps({
                "schema_version": 1, "classification": "g3_local_store_run_result",
                "store_id": "D", "device_identity": "fixture-only-device-D", "run_id": run_id,
                "package_record": reference, "preservation_receipt": reference,
                "full_restore_receipt": reference, "semantic_replay_report": reference,
                "source_support_closure": reference, "raw_envelope": reference,
                "package_phase_wall_time_s": 1.0, "offload_phase_wall_time_s": 1.0,
                "restore_phase_wall_time_s": 1.0, "semantic_phase_wall_time_s": 1.0,
                "store_namespace": str(namespace), "cleanup_authorized": False,
            }), encoding="utf-8")
            results.append(result)
        category_files = {}
        for category in ("retained_six_support_bytes", "primary_specific_support_bytes",
                         "source_bytes", "metadata_bytes", "vm_applicable_scratch_bytes"):
            path = self.root / f"{category}.bin"
            path.write_bytes(category.encode())
            category_files[category] = [str(path)]
        measurements = {**category_files, "owned_restore_bytes": restore_roots}

        original_first = json.loads(results[0].read_text())
        outside = self.root / "outside-store" / self.run_ids[0]
        (outside / "full-restore").mkdir(parents=True)
        hostile_outside = {**original_first, "store_namespace": str(outside)}
        results[0].write_text(json.dumps(hostile_outside), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "outside its canonical"):
            finalize_store(
                result_paths=results, evidence_root=self.evidence, store_root=self.store,
                store_id="D", shared_measurements=measurements, **self.identity_inputs)
        results[0].write_text(json.dumps(original_first), encoding="utf-8")

        noncanonical_result = self.evidence / "noncanonical-result.json"
        noncanonical_result.write_text(json.dumps(original_first), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "path is not canonical"):
            finalize_store(
                result_paths=[noncanonical_result, *results[1:]],
                evidence_root=self.evidence, store_root=self.store,
                store_id="D", shared_measurements=measurements, **self.identity_inputs)

        original_second = json.loads(results[1].read_text())
        shared_namespace = {**original_second,
                            "store_namespace": original_first["store_namespace"]}
        results[1].write_text(json.dumps(shared_namespace), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "outside its canonical"):
            finalize_store(
                result_paths=results, evidence_root=self.evidence, store_root=self.store,
                store_id="D", shared_measurements=measurements, **self.identity_inputs)
        results[1].write_text(json.dumps(original_second), encoding="utf-8")

        receipt = finalize_store(
            result_paths=results, evidence_root=self.evidence, store_root=self.store,
            store_id="D", shared_measurements=measurements, **self.identity_inputs)
        value = json.loads(receipt.read_text())
        self.assertEqual(2, value["schema_version"])
        self.assertEqual(6, value["shared_bytes"]["owned_restore_bytes"])
        self.assertEqual(6, value["shared_file_counts"]["owned_restore_bytes"])
        self.assertEqual(6, len(value["runs"]))

        alias = self.root / "metadata-alias.bin"
        os.link(Path(category_files["source_bytes"][0]), alias)
        hostile = {**measurements, "metadata_bytes": [str(alias)]}
        with self.assertRaisesRegex(ValueError, "hard links"):
            _measure_shared_paths(hostile, {Path(path).resolve() for path in restore_roots})


if __name__ == "__main__":
    unittest.main()
