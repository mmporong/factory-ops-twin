from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from io import BytesIO

from factory_ops.g3_raw_storage import (
    ENVELOPE_NAME,
    PACKAGE_NAME,
    load_raw_envelope,
    main,
    prepare_raw_package,
)
from factory_ops.g3_raw_storage import _verify_bundle_commit, _verify_package
from factory_ops.g3_supervisor import create_attempt_start, finalize_attempt
from factory_ops.g3_session import prepare_session
from scripts.brev.g3_resource_pack import archive_output, checkpoint


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@unittest.skipUnless(os.name == "posix", "raw package mutation is intentionally POSIX/WSL-only")
class G3RawStoragePosixTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).absolute()
        self.source_bundle = self.root / "source.bundle"
        repository = self.root / "tiny-repository"
        repository.mkdir()
        subprocess.run(("git", "init", "-q"), cwd=repository, check=True)
        subprocess.run(("git", "config", "user.email", "fixture@example.invalid"), cwd=repository, check=True)
        subprocess.run(("git", "config", "user.name", "Fixture"), cwd=repository, check=True)
        (repository / "tracked.txt").write_text("source bytes\n", encoding="utf-8")
        shutil.copytree(Path(__file__).resolve().parents[1] / "configs" / "factory-v3",
                        repository / "configs" / "factory-v3")
        subprocess.run(("git", "add", "tracked.txt", "configs/factory-v3"), cwd=repository, check=True)
        subprocess.run(("git", "commit", "-q", "-m", "fixture"), cwd=repository, check=True)
        self.commit = subprocess.run(("git", "rev-parse", "HEAD"), cwd=repository, check=True,
                                     capture_output=True, text=True).stdout.strip()
        subprocess.run(("git", "bundle", "create", str(self.source_bundle), "--all"),
                       cwd=repository, check=True)
        self.run_label = "g3-p-normal-fifo-001"
        self.run_id = "a" * 32
        self.resources = self.root / "resources" / self.run_label
        checkpoint(self.root, self.run_label, self.run_id, "start")
        self.config = self.root / "config.json"
        self.config.write_text('{}\n', encoding="utf-8")
        self.session = self.root / "source-snapshots" / self.run_label / "runs" / f"{self.run_label}-session"
        prepare_session(self.session, profile="pilot", condition="normal", policy="fifo", seed=100)
        self.billing = self.root / "billing.json"
        self.billing.write_text(json.dumps({
            "observed_at": "2026-10-18T00:00:00Z", "source": "test_fixture",
            "confirmation_method": "local fixture", "currency": "USD",
            "confirmed_hourly_rate_usd": 3.36, "budget_remaining_usd": 20,
        }), encoding="utf-8")
        start = checkpoint(self.root, self.run_label, self.run_id, "workers-finished")
        self.output = self.root / "results" / self.run_label
        self.output.mkdir(parents=True)
        (self.output / "rosbag.db3").write_bytes(b"actual-cdr-fixture")
        create_attempt_start(
            self.resources / "attempt-start.json", self.root, run_id=self.run_id,
            profile="pilot", condition="normal", policy="fifo", seed=100,
            source_commit=self.commit, config_path=self.config, session_root=self.session,
            billing_path=self.billing, free_disk_bytes_before=10_000_000,
            started_at_utc=start["observed_at_utc"],
        )
        checkpoint(self.root, self.run_label, self.run_id, "validation-finished")
        record = archive_output(self.root, self.run_label, self.run_id)
        disk = self.resources / "disk.json"
        disk_doc = json.loads(disk.read_text(encoding="utf-8"))
        disk_doc["free_disk_bytes_before"] = 10_000_000
        disk_doc["free_disk_bytes_after"] = 9_000_000
        disk.write_text(json.dumps(disk_doc), encoding="utf-8")
        self.receipt = self.resources / "attempt-receipt.json"
        finalize_attempt(
            self.resources / "attempt-start.json", self.receipt, self.root,
            workers_finished_at_utc=record["workers_finished_at_utc"],
            validation_finished_at_utc=record["validation_finished_at_utc"],
            archive_finished_at_utc=record["archive_finished_at_utc"],
            gpu_csv_path=None, scene_summary_path=None,
            output_manifest_path=self.resources / "output-manifest.json",
            archive_path=self.resources / "output.tar.gz", disk_path=disk,
            collection_failure_reason="qualification fixture deliberately has no GPU sample",
        )
        self.record = self.resources / "archive-record.json"

    def prepare(self, name: str = "package") -> Path:
        return prepare_raw_package(
            archive_record=self.record, attempt_receipt=self.receipt,
            source_bundle=self.source_bundle, output=self.root / name,
        )

    def test_prepares_uncompressed_byte_exact_package_with_false_approval_flags(self) -> None:
        envelope_path = self.prepare()
        envelope = load_raw_envelope(envelope_path)
        self.assertEqual("pilot", envelope["profile"])
        self.assertIsNone(envelope["trial_id"])
        self.assertTrue(all(value is False for value in envelope["proof_flags"].values()))
        package = envelope_path.parent / PACKAGE_NAME
        self.assertEqual(sha(package), envelope["package"]["sha256"])
        with package.open("rb") as stream:
            self.assertNotEqual(b"\x1f\x8b", stream.read(2))
        with tarfile.open(package, "r:") as packed:
            names = packed.getnames()
            self.assertEqual(len(names), len(set(names)))
            archive = packed.extractfile("original/output.tar.gz")
            manifest = packed.extractfile("original/output-manifest.json")
            record = packed.extractfile("original/archive-record.json")
            source = packed.extractfile("support/source.bundle")
            self.assertIsNotNone(archive)
            self.assertIsNotNone(manifest)
            self.assertIsNotNone(record)
            self.assertIsNotNone(source)
            assert archive and manifest and record and source
            self.assertEqual((self.resources / "output.tar.gz").read_bytes(), archive.read())
            self.assertEqual((self.resources / "output-manifest.json").read_bytes(), manifest.read())
            self.assertEqual(self.record.read_bytes(), record.read())
            self.assertEqual(self.source_bundle.read_bytes(), source.read())
            support_stream = packed.extractfile("support/support-manifest.json")
            assert support_stream
            support = json.load(support_stream)
            self.assertTrue(support["categories"]["source"])
            self.assertEqual(len([path for path in self.session.rglob("*") if path.is_file()]),
                             len(support["categories"]["session"]))
            self.assertEqual([], support["categories"]["per-trial"])

    def test_qualification_preserves_no_billing_absent_receipt_and_checkpoint_closure(self) -> None:
        root = self.root / "qualification"
        root.mkdir()
        run = "g3-q-closure-eta-011"
        run_id = "c" * 32
        checkpoint(root, run, run_id, "start")
        session = root / "source-snapshots" / run / "runs" / f"{run}-session"
        prepare_session(session, profile="qualification", condition="closure", policy="eta", seed=0)
        output = root / "results" / run
        output.mkdir(parents=True)
        (output / "rosbag.db3").write_bytes(b"qualification-cdr")
        checkpoint(root, run, run_id, "workers-finished")
        checkpoint(root, run, run_id, "validation-finished")
        record = archive_output(root, run, run_id)
        record_path = root / "resources" / run / "archive-record.json"
        receipt_path = root / "resources" / run / "attempt-receipt.json"
        self.assertFalse(receipt_path.exists())
        envelope_path = prepare_raw_package(
            archive_record=record_path, attempt_receipt=None, source_bundle=self.source_bundle,
            output=root / "package", qualification_source_commit=self.commit,
            qualification_seed=0, qualification_session_root=session,
        )
        envelope = load_raw_envelope(envelope_path)
        self.assertTrue(envelope["qualification_no_billing_attempt_receipt_absent"])
        self.assertIsNone(envelope["attempt_receipt_sha256"])
        self.assertEqual("qualification", envelope["profile"])
        self.assertEqual(record["output_tree_sha256"], envelope["output_tree_sha256"])
        with tarfile.open(envelope_path.parent / PACKAGE_NAME, "r:") as packed:
            self.assertNotIn("support/attempt-receipt.json", packed.getnames())
            support_stream = packed.extractfile("support/support-manifest.json")
            assert support_stream
            paths = {Path(row["path"]).name
                     for row in json.load(support_stream)["categories"]["source"]}
            self.assertTrue({"start.json", "workers-finished.json", "validation-finished.json"} <= paths)

        hostile_root = self.root / "qualification-hostile"
        hostile_root.mkdir()
        checkpoint(hostile_root, run, run_id, "start")
        hostile_session = hostile_root / "source-snapshots" / run / "runs" / f"{run}-session"
        shutil.copytree(self.session, hostile_session)
        hostile_output = hostile_root / "results" / run
        hostile_output.mkdir(parents=True)
        (hostile_output / "rosbag.db3").write_bytes(b"qualification-cdr")
        checkpoint(hostile_root, run, run_id, "workers-finished")
        checkpoint(hostile_root, run, run_id, "validation-finished")
        archive_output(hostile_root, run, run_id)
        with self.assertRaises(ValueError):
            prepare_raw_package(
                archive_record=hostile_root / "resources" / run / "archive-record.json",
                attempt_receipt=None, source_bundle=self.source_bundle,
                output=hostile_root / "package", qualification_source_commit=self.commit,
                qualification_seed=0, qualification_session_root=hostile_session,
            )

    def test_rejects_source_bundle_schema_and_record_receipt_identity_forgery(self) -> None:
        envelope_path = self.prepare("valid")
        hostile = self.root / "hostile.json"
        hostile.write_text(json.dumps({"schema": "g3-storage-source-bundle-envelope-v1",
                                       "kind": "source_bundle_probe"}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "shape"):
            load_raw_envelope(hostile)
        valid = json.loads(envelope_path.read_text(encoding="utf-8"))
        mutations = (
            {"condition": "burst"}, {"policy": "eta"}, {"attempt_status": "fabricated"},
            {"raw_bytes": -9}, {"file_count": -1}, {"limitations": []},
        )
        for index, mutation in enumerate(mutations):
            document = {**valid, **mutation}
            changed = self.root / f"hostile-envelope-{index}.json"
            changed.write_text(json.dumps(document), encoding="utf-8")
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                load_raw_envelope(changed)
        receipt = json.loads(self.receipt.read_text(encoding="utf-8"))
        receipt["identity"]["seed"] = 99
        forged = self.resources / "forged-receipt.json"
        forged.write_text(json.dumps(receipt), encoding="utf-8")
        record = json.loads(self.record.read_text(encoding="utf-8"))
        record["literal_paths"]["attempt_receipt_path"] = str(forged)
        forged_record = self.resources / "forged-record.json"
        record["literal_paths"]["archive_record_path"] = str(forged_record)
        forged_record.write_text(json.dumps(record), encoding="utf-8")
        with self.assertRaises(ValueError):
            prepare_raw_package(archive_record=forged_record, attempt_receipt=forged,
                                source_bundle=self.source_bundle, output=self.root / "rejected")
        self.assertTrue(any((self.root / "rejected").glob("prepare-error-*.json")))
        self.assertTrue(envelope_path.is_file())

    def test_rejects_support_links_case_collisions_and_existing_namespace(self) -> None:
        linked = self.root / "shared-link.json"
        linked.symlink_to(self.billing)
        with self.assertRaises((OSError, ValueError)):
            prepare_raw_package(archive_record=self.record, attempt_receipt=self.receipt,
                                source_bundle=self.source_bundle, output=self.root / "link-rejected",
                                shared_support_files=[linked])
        collision_a = self.root / "Case.json"
        collision_b = self.root / "case.json"
        collision_a.write_text("{}", encoding="utf-8")
        collision_b.write_text("{}", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "multiple categories|collide"):
            prepare_raw_package(archive_record=self.record, attempt_receipt=self.receipt,
                                source_bundle=self.source_bundle, output=self.root / "case-rejected",
                                shared_support_files=[collision_a, collision_b])
        self.prepare("once")
        with self.assertRaises(FileExistsError):
            self.prepare("once")

    def test_source_mutation_during_stream_is_rejected_without_published_package(self) -> None:
        from factory_ops import g3_raw_storage
        original = g3_raw_storage._add_stream
        fired = False

        def mutate(packed, member, stream, expected):
            nonlocal fired
            result = original(packed, member, stream, expected)
            if member == "support/source.bundle" and not fired:
                fired = True
                self.source_bundle.write_bytes(b"foreign replacement")
            return result

        with patch("factory_ops.g3_raw_storage._add_stream", side_effect=mutate):
            with self.assertRaises(g3_raw_storage.PreparationIntegrityError) as caught:
                self.prepare("mutated")
        self.assertIn("pinned_input_exit_integrity", caught.exception.integrity_codes)
        self.assertIsInstance(caught.exception.__cause__, ValueError)
        self.assertFalse((self.root / "mutated" / PACKAGE_NAME).exists())
        self.assertFalse((self.root / "mutated" / ENVELOPE_NAME).exists())
        self.assertTrue(any((self.root / "mutated").glob("prepare-error-*.json")))

    def test_same_bytes_new_source_inode_and_late_session_file_are_rejected(self) -> None:
        from factory_ops import g3_raw_storage
        original_rows = g3_raw_storage._support_rows
        source_payload = self.source_bundle.read_bytes()
        changed = False

        def replace_source(category, paths):
            nonlocal changed
            if category == "source" and not changed:
                changed = True
                replacement = self.source_bundle.with_suffix(".replacement")
                replacement.write_bytes(source_payload)
                os.replace(replacement, self.source_bundle)
            return original_rows(category, paths)

        with patch("factory_ops.g3_raw_storage._support_rows", side_effect=replace_source):
            with self.assertRaises(ValueError):
                self.prepare("same-byte-inode")
        self.assertFalse((self.root / "same-byte-inode" / PACKAGE_NAME).exists())

        added = False
        def add_late(category, paths):
            nonlocal added
            if category == "source" and not added:
                added = True
                (self.session / "late-added.bin").write_bytes(b"late")
            return original_rows(category, paths)

        with patch("factory_ops.g3_raw_storage._support_rows", side_effect=add_late):
            with self.assertRaisesRegex(ValueError, "exact node set|session"):
                self.prepare("late-session")
        self.assertFalse((self.root / "late-session" / PACKAGE_NAME).exists())

    def test_publish_swap_and_second_link_failure_never_leave_owned_partial_success(self) -> None:
        from factory_ops import g3_raw_storage
        original_unlink = g3_raw_storage.os.unlink
        swapped = False

        def swap_after_partial_unlink(path, *args, **kwargs):
            nonlocal swapped
            result = original_unlink(path, *args, **kwargs)
            descriptor = kwargs.get("dir_fd")
            if isinstance(path, str) and path.startswith("package-") and path.endswith(".partial") and not swapped:
                swapped = True
                os.rename(PACKAGE_NAME, "attacker-preserved-package.tar",
                          src_dir_fd=descriptor, dst_dir_fd=descriptor)
                attacker = os.open(PACKAGE_NAME, os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                                   0o600, dir_fd=descriptor)
                os.write(attacker, b"attacker")
                os.close(attacker)
            return result

        with patch("factory_ops.g3_raw_storage.os.unlink", side_effect=swap_after_partial_unlink):
            with self.assertRaises(ValueError):
                self.prepare("publish-swap")
        swapped_root = self.root / "publish-swap"
        self.assertEqual(b"attacker", (swapped_root / PACKAGE_NAME).read_bytes())
        self.assertFalse((swapped_root / ENVELOPE_NAME).exists())

        original_link = g3_raw_storage.os.link
        calls = 0
        def fail_second_link(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise FileExistsError("synthetic second publication failure")
            return original_link(*args, **kwargs)

        with patch("factory_ops.g3_raw_storage.os.link", side_effect=fail_second_link):
            with self.assertRaises(FileExistsError):
                self.prepare("second-link-failure")
        failed_root = self.root / "second-link-failure"
        self.assertFalse((failed_root / PACKAGE_NAME).exists())
        self.assertFalse((failed_root / ENVELOPE_NAME).exists())
        self.assertTrue(any(failed_root.glob("prepare-error-*.json")))

    def test_thin_bundle_and_sensitive_error_filename_are_rejected(self) -> None:
        repository = self.root / "thin-repository"
        repository.mkdir()
        subprocess.run(("git", "init", "-q"), cwd=repository, check=True)
        subprocess.run(("git", "config", "user.email", "fixture@example.invalid"), cwd=repository, check=True)
        subprocess.run(("git", "config", "user.name", "Fixture"), cwd=repository, check=True)
        (repository / "a").write_text("one", encoding="utf-8")
        subprocess.run(("git", "add", "a"), cwd=repository, check=True)
        subprocess.run(("git", "commit", "-qm", "one"), cwd=repository, check=True)
        (repository / "a").write_text("two", encoding="utf-8")
        subprocess.run(("git", "commit", "-qam", "two"), cwd=repository, check=True)
        head = subprocess.run(("git", "rev-parse", "HEAD"), cwd=repository, check=True,
                              capture_output=True, text=True).stdout.strip()
        thin = self.root / "thin.bundle"
        subprocess.run(("git", "bundle", "create", str(thin), "HEAD", "^HEAD~1"),
                       cwd=repository, check=True)
        with thin.open("rb") as stream, self.assertRaises(subprocess.CalledProcessError):
            _verify_bundle_commit(stream, head)

        secret = self.root / "token-SECRET123.bundle"
        output = self.root / "secret-error"
        with self.assertRaises((OSError, ValueError)):
            prepare_raw_package(archive_record=self.record, attempt_receipt=self.receipt,
                                source_bundle=secret, output=output)
        receipt = next(output.glob("prepare-error-*.json")).read_text(encoding="utf-8")
        self.assertNotIn("SECRET123", receipt)
        self.assertNotIn("message", json.loads(receipt))

    def test_loader_binds_internal_envelope_support_members_and_actual_count(self) -> None:
        valid_envelope = self.prepare("loader-contract")
        valid_root = valid_envelope.parent

        identity_root = self.root / "loader-identity"
        shutil.copytree(valid_root, identity_root)
        identity_path = identity_root / ENVELOPE_NAME
        identity = json.loads(identity_path.read_text(encoding="utf-8"))
        identity.update(run_label="g3-p-burst-eta-001", condition="burst", policy="eta")
        identity_path.write_text(json.dumps(identity), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "internal envelope"):
            load_raw_envelope(identity_path)

        count_root = self.root / "loader-count"
        shutil.copytree(valid_root, count_root)
        count_path = count_root / ENVELOPE_NAME
        count = json.loads(count_path.read_text(encoding="utf-8"))
        count["package"]["member_count"] += 1
        count_path.write_text(json.dumps(count), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "count"):
            load_raw_envelope(count_path)

        missing_root = self.root / "loader-missing"
        shutil.copytree(valid_root, missing_root)
        source_package = missing_root / PACKAGE_NAME
        replacement = missing_root / "replacement.tar"
        members: list[tuple[tarfile.TarInfo, bytes]] = []
        with tarfile.open(source_package, "r:") as packed:
            for member in packed:
                extracted = packed.extractfile(member)
                assert extracted is not None
                members.append((member, extracted.read()))
        removed = next(name for name, _payload in ((member.name, payload) for member, payload in members)
                       if name.startswith("support/session/"))
        with tarfile.open(replacement, "w:") as packed:
            for member, payload in members:
                if member.name != removed:
                    packed.addfile(member, BytesIO(payload))
        os.replace(replacement, source_package)
        missing_path = missing_root / ENVELOPE_NAME
        missing = json.loads(missing_path.read_text(encoding="utf-8"))
        missing["package"]["bytes"] = source_package.stat().st_size
        missing["package"]["sha256"] = sha(source_package)
        missing["package"]["member_count"] -= 1
        missing_path.write_text(json.dumps(missing), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "member"):
            load_raw_envelope(missing_path)

    def test_validated_reference_replacement_and_double_integrity_failure_are_preserved(self) -> None:
        from factory_ops import g3_raw_storage
        original_validate = g3_raw_storage.validate_attempt_receipt
        billing_payload = self.billing.read_bytes()
        replaced = False

        def replace_reference(*args, **kwargs):
            nonlocal replaced
            result = original_validate(*args, **kwargs)
            if not replaced:
                replaced = True
                foreign = self.billing.with_suffix(".foreign")
                foreign.write_text('{"foreign":true}\n', encoding="utf-8")
                os.replace(foreign, self.billing)
            return result

        with patch("factory_ops.g3_raw_storage.validate_attempt_receipt", side_effect=replace_reference):
            with self.assertRaises(ValueError):
                self.prepare("reference-replaced")
        self.assertFalse((self.root / "reference-replaced" / PACKAGE_NAME).exists())
        self.billing.write_bytes(billing_payload)

        original_rows = g3_raw_storage._support_rows
        fired = False
        def mutate_and_fail(category, pins):
            nonlocal fired
            if category == "source" and not fired:
                fired = True
                (self.session / "double-failure.bin").write_bytes(b"changed")
                bound = self.session / "config.json"
                replacement = self.session / "config.replacement"
                replacement.write_bytes(bound.read_bytes())
                os.replace(replacement, bound)
                raise RuntimeError("token-SECRET123.bundle")
            return original_rows(category, pins)

        with patch("factory_ops.g3_raw_storage._support_rows", side_effect=mutate_and_fail):
            with self.assertRaises(g3_raw_storage.PreparationIntegrityError) as caught:
                self.prepare("double-integrity")
        self.assertIsInstance(caught.exception.__cause__, RuntimeError)
        error_path = next((self.root / "double-integrity").glob("prepare-error-*.json"))
        error = json.loads(error_path.read_text(encoding="utf-8"))
        self.assertIn("pinned_input_exit_integrity", error["integrity_error_codes"])
        self.assertIn("session_tree_exit_integrity", error["integrity_error_codes"])
        self.assertNotIn("SECRET123", error_path.read_text(encoding="utf-8"))

    def test_package_verifier_rejects_unsafe_duplicate_extra_missing_link_and_sparse_members(self) -> None:
        expected = {"safe.bin": {"bytes": 4, "sha256": hashlib.sha256(b"safe").hexdigest()}}

        def payload(members):
            stream = BytesIO()
            with tarfile.open(fileobj=stream, mode="w:") as packed:
                for info, value in members:
                    packed.addfile(info, BytesIO(value) if info.isreg() else None)
            stream.seek(0)
            return stream

        safe = tarfile.TarInfo("safe.bin")
        safe.size = 4
        hostile_cases = []
        traversal = tarfile.TarInfo("../safe.bin")
        traversal.size = 4
        hostile_cases.append([(traversal, b"safe")])
        duplicate_a = tarfile.TarInfo("safe.bin")
        duplicate_a.size = 4
        duplicate_b = tarfile.TarInfo("safe.bin")
        duplicate_b.size = 4
        hostile_cases.append([(duplicate_a, b"safe"), (duplicate_b, b"safe")])
        extra = tarfile.TarInfo("extra.bin")
        extra.size = 4
        hostile_cases.append([(safe, b"safe"), (extra, b"safe")])
        hostile_cases.append([])
        symlink = tarfile.TarInfo("safe.bin")
        symlink.type = tarfile.SYMTYPE
        symlink.linkname = "target"
        hostile_cases.append([(symlink, b"")])
        hardlink = tarfile.TarInfo("safe.bin")
        hardlink.type = tarfile.LNKTYPE
        hardlink.linkname = "target"
        hostile_cases.append([(hardlink, b"")])
        case_a = tarfile.TarInfo("safe.bin")
        case_a.size = 4
        case_b = tarfile.TarInfo("SAFE.BIN")
        case_b.size = 4
        hostile_cases.append([(case_a, b"safe"), (case_b, b"safe")])
        sparse = tarfile.TarInfo("safe.bin")
        sparse.size = 4
        sparse.pax_headers = {"GNU.sparse.map": "0,4"}
        hostile_cases.append([(sparse, b"safe")])
        for members in hostile_cases:
            with self.subTest(members=[info.name for info, _value in members]):
                with self.assertRaises(ValueError):
                    _verify_package(payload(members), expected)


class G3RawStorageWindowsGuardTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "nt", "Windows guard is verified on Windows")
    def test_windows_cli_fails_before_creating_output(self) -> None:
        root = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: __import__("shutil").rmtree(root))
        output = root / "output"
        with self.assertRaisesRegex(RuntimeError, "Linux or WSL"):
            main(["--archive-record", str(root / "record.json"),
                  "--attempt-receipt", str(root / "receipt.json"),
                  "--source-bundle", str(root / "source.bundle"),
                  "--output", str(output)])
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
