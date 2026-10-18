from __future__ import annotations

from io import StringIO
from contextlib import redirect_stderr
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import unittest
from unittest.mock import patch
from collections.abc import Callable
from typing import BinaryIO, cast

from factory_ops.g3_storage import (
    GhReleaseApi,
    _absolute_lexical,
    load_envelope,
    main,
    prepare_envelope,
    readback_envelope,
    upload_envelope,
    verify_readback,
)


COMMIT = "a" * 40


class FakeReleaseApi:
    def __init__(self) -> None:
        self.assets: list[dict[str, object]] = []
        self.payloads: dict[int, bytes] = {}
        self.repository_calls = 0
        self.release_calls = 0
        self.upload_calls = 0
        self.download_calls = 0
        self.private_sequence: list[bool] = []
        self.fail_upload_at: int | None = None
        self.on_upload: Callable[[], None] | None = None
        self.on_repository: Callable[[], None] | None = None

    def repository(self, owner: str, repository: str) -> dict[str, object]:
        self.repository_calls += 1
        if self.on_repository is not None:
            self.on_repository()
        private = self.private_sequence.pop(0) if self.private_sequence else True
        return {"id": 1421505101, "name": "factory-ops-twin",
                "full_name": "mmporong/factory-ops-twin", "private": private,
                "html_url": "https://should-not-be-preserved.invalid/token"}

    def release(self, owner: str, repository: str, release_id: int) -> dict[str, object]:
        self.release_calls += 1
        return {"id": 991, "tag_name": "storage-probe-001", "target_commitish": COMMIT,
                "draft": True, "assets": list(self.assets),
                "upload_url": "https://temporary.invalid/signed-secret"}

    def upload(self, owner: str, repository: str, release_id: int,
               name: str, stream) -> dict[str, object]:
        self.upload_calls += 1
        if self.on_upload is not None:
            self.on_upload()
        if self.fail_upload_at == self.upload_calls:
            raise RuntimeError("synthetic upload failure with token secret")
        payload = stream.read()
        asset_id = 10_000 + len(self.assets)
        row: dict[str, object] = {
            "id": asset_id, "name": name, "size": len(payload), "state": "uploaded",
            "digest": "sha256:" + hashlib.sha256(payload).hexdigest(),
            "browser_download_url": "https://temporary.invalid/private",
        }
        self.assets.append(row)
        self.payloads[asset_id] = payload
        return row

    def download(self, owner: str, repository: str, asset_id: int, stream) -> None:
        self.download_calls += 1
        stream.write(self.payloads[asset_id])


@unittest.skipUnless(os.name == "posix", "mutation contract is intentionally POSIX/WSL-only")
class StorageEnvelopePosixTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).absolute()
        self.archive = self.root / "source.bundle"
        self.payload = bytes(range(251)) * 5 + b"sealed-source-bundle"
        self.archive.write_bytes(self.payload)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def prepare(self, name: str = "envelope", chunk_bytes: int = 317) -> Path:
        return prepare_envelope(
            archive=self.archive, output=self.root / name, source_commit=COMMIT,
            repository_id=1421505101, owner="mmporong", repository="factory-ops-twin",
            release_id=991, tag="storage-probe-001", target_commit=COMMIT,
            draft=True, chunk_bytes=chunk_bytes)

    def uploaded(self, chunk_bytes: int = 317) -> tuple[Path, FakeReleaseApi]:
        envelope = self.prepare(chunk_bytes=chunk_bytes)
        api = FakeReleaseApi()
        return upload_envelope(envelope, api), api

    def test_roundtrip_requeries_remote_and_preserves_false_proof_flags(self) -> None:
        uploaded, api = self.uploaded()
        readback = readback_envelope(uploaded, self.root / "readback", api)
        receipt = verify_readback(uploaded, readback, self.root / "reassembled.bundle", api)
        self.assertEqual(self.payload, (self.root / "reassembled.bundle").read_bytes())
        result = json.loads(receipt.read_text(encoding="utf-8"))
        self.assertEqual(hashlib.sha256(self.payload).hexdigest(), result["archive_sha256"])
        self.assertEqual("point_checks_cannot_exclude_between_check_visibility",
                         result["privacy_observation_limit"])
        self.assertFalse(any(result["proof_flags"].values()))
        self.assertGreaterEqual(api.repository_calls, 25)

    def test_source_bundle_only_schema_rejects_raw_mode_and_raw_fields(self) -> None:
        envelope = self.prepare()
        document = json.loads(envelope.read_text(encoding="utf-8"))
        document["kind"] = "raw_archive"
        hostile = self.root / "hostile.json"
        hostile.write_text(json.dumps(document), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "shape"):
            load_envelope(hostile)
        document["kind"] = "source_bundle_probe"
        document["raw_manifest"] = {"sha256": "0" * 64}
        hostile.write_text(json.dumps(document), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "shape"):
            load_envelope(hostile)
        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):
                main(["prepare", "--kind", "raw_archive"])

    def test_cli_does_not_resolve_archive_or_output_parent_symlinks(self) -> None:
        archive_link = self.root / "source-link.bundle"
        archive_link.symlink_to(self.archive)
        arguments = ["prepare", "--archive", str(archive_link), "--output", str(self.root / "out"),
                     "--source-commit", COMMIT, "--repository-id", "1421505101",
                     "--owner", "mmporong", "--repository", "factory-ops-twin",
                     "--release-id", "991", "--tag", "storage-probe-001",
                     "--target-commit", COMMIT, "--draft", "--chunk-bytes", "300"]
        with self.assertRaises((OSError, ValueError)):
            main(arguments)
        self.assertFalse((self.root / "out").exists())
        real_parent = self.root / "real-parent"
        real_parent.mkdir()
        linked_parent = self.root / "linked-parent"
        linked_parent.symlink_to(real_parent, target_is_directory=True)
        arguments[arguments.index(str(self.root / "out"))] = str(linked_parent / "out")
        arguments[arguments.index(str(archive_link))] = str(self.archive)
        with self.assertRaises((OSError, ValueError)):
            main(arguments)
        self.assertFalse((real_parent / "out").exists())

    def test_cli_envelope_and_readback_symlinks_are_not_hidden(self) -> None:
        uploaded, api = self.uploaded(chunk_bytes=10_000)
        envelope_link = self.root / "envelope-link.json"
        envelope_link.symlink_to(uploaded)
        with patch("factory_ops.g3_storage.GhReleaseApi", return_value=api):
            with self.assertRaises((OSError, ValueError)):
                main(["readback", "--envelope", str(envelope_link),
                      "--destination", str(self.root / "never"), "--gh-executable", "mock"])
        readback = readback_envelope(uploaded, self.root / "readback", api)
        readback_link = self.root / "readback-link"
        readback_link.symlink_to(readback, target_is_directory=True)
        with patch("factory_ops.g3_storage.GhReleaseApi", return_value=api):
            with self.assertRaises((OSError, ValueError)):
                main(["verify", "--envelope", str(uploaded), "--readback", str(readback_link),
                      "--output-archive", str(self.root / "never.bundle"),
                      "--gh-executable", "mock"])

    def test_upload_stream_is_same_verified_fd_when_path_is_replaced(self) -> None:
        envelope = self.prepare(chunk_bytes=10_000)
        planned = load_envelope(envelope)["assets"][0]
        chunk = envelope.parent / "chunks" / planned["name"]
        original = chunk.read_bytes()
        api = FakeReleaseApi()
        fired = False

        def replace_path() -> None:
            nonlocal fired
            if fired:
                return
            fired = True
            moved = chunk.with_name("preserved-original.part")
            chunk.rename(moved)
            chunk.write_bytes(b"replacement-21-bytes"[:21])

        api.on_upload = replace_path
        with self.assertRaisesRegex(ValueError, "pathname or identity changed"):
            upload_envelope(envelope, api)
        self.assertEqual(original, api.payloads[10_000])
        self.assertEqual(b"replacement-21-bytes"[:21], chunk.read_bytes())
        receipt = json.loads(next(envelope.parent.glob("upload-error-*.json")).read_text())
        self.assertEqual(1, len(receipt["completed_assets"]))

    def test_prepare_namespace_swap_writes_nothing_into_replacement(self) -> None:
        output = self.root / "swap"
        moved = self.root / "preserved-original"
        from factory_ops import g3_storage
        original_copy = g3_storage._copy_segment
        fired = False

        def swap(source, destination, size, expected):
            nonlocal fired
            if not fired:
                fired = True
                output.rename(moved)
                output.mkdir()
                (output / "chunks").mkdir()
            return original_copy(source, destination, size, expected)

        with patch("factory_ops.g3_storage._copy_segment", side_effect=swap):
            with self.assertRaisesRegex(ValueError, "moved or replaced"):
                self.prepare(name="swap", chunk_bytes=300)
        self.assertTrue(moved.is_dir())
        self.assertEqual([], list((output / "chunks").iterdir()))
        self.assertEqual(1, len(list(moved.glob("prepare-error-*.json"))))

    def test_component_swap_after_open_never_enters_attacker_tree(self) -> None:
        nested = self.root / "guard" / "a" / "b"
        nested.mkdir(parents=True)
        source = nested / "source.bundle"
        source.write_bytes(self.payload)
        attacker = self.root / "attacker"
        (attacker / "b").mkdir(parents=True)
        (attacker / "b" / "source.bundle").write_bytes(b"attacker")
        original_open = os.open
        fired = False

        def racing_open(path, flags, mode=0o777, *, dir_fd=None):
            nonlocal fired
            descriptor = original_open(path, flags, mode, dir_fd=dir_fd)
            if path == "a" and dir_fd is not None and not fired:
                fired = True
                (self.root / "guard" / "a").rename(self.root / "guard" / "a-original")
                (self.root / "guard" / "a").symlink_to(attacker, target_is_directory=True)
            return descriptor

        with patch("factory_ops.g3_storage.os.open", side_effect=racing_open):
            with self.assertRaisesRegex(ValueError, "moved or replaced"):
                prepare_envelope(
                    archive=source, output=self.root / "never", source_commit=COMMIT,
                    repository_id=1421505101, owner="mmporong",
                    repository="factory-ops-twin", release_id=991,
                    tag="storage-probe-001", target_commit=COMMIT, draft=True,
                    chunk_bytes=300)
        self.assertEqual(b"attacker", (attacker / "b" / "source.bundle").read_bytes())
        self.assertFalse((self.root / "never").exists())

    def test_created_leaf_identity_is_checked_for_every_artifact_role(self) -> None:
        from factory_ops.g3_storage import _Directory
        for index, label in enumerate(("storage chunk", "storage envelope", "readback asset",
                                       "reassembly output", "verification receipt")):
            directory_path = self.root / f"leaf-{index}"
            with _Directory.create(directory_path, label) as directory:
                leaf = directory_path / "value.bin"
                moved = directory_path / "original.bin"
                with self.subTest(label=label), self.assertRaisesRegex(
                        ValueError, "pathname or identity changed"):
                    with directory.open_new("value.bin", label) as stream:
                        stream.write(b"verified")
                        leaf.rename(moved)
                        leaf.write_bytes(b"evil")
                self.assertEqual(b"verified", moved.read_bytes())
                self.assertEqual(b"evil", leaf.read_bytes())

    def test_same_inode_same_size_envelope_mutation_records_failure_and_blocks_reuse(self) -> None:
        envelope = self.prepare(chunk_bytes=10_000)
        api = FakeReleaseApi()
        fired = False

        def mutate() -> None:
            nonlocal fired
            if fired:
                return
            fired = True
            with envelope.open("r+b") as stream:
                stream.seek(-1, os.SEEK_END)
                stream.write(b" ")

        api.on_repository = mutate
        with self.assertRaisesRegex(ValueError, "bytes changed"):
            upload_envelope(envelope, api)
        self.assertTrue(list(envelope.parent.glob("upload-error-*.json")))
        with self.assertRaisesRegex(ValueError, "failure receipt"):
            upload_envelope(envelope, FakeReleaseApi())

    def test_source_mutation_at_context_exit_leaves_failure_namespace_without_envelope(self) -> None:
        from factory_ops import g3_storage
        original_sha = g3_storage._sha_stream
        calls = 0

        def mutate_after_hash(stream):
            nonlocal calls
            result = original_sha(stream)
            calls += 1
            if calls == 4:
                with self.archive.open("r+b") as target:
                    first = target.read(1)
                    target.seek(0)
                    target.write(bytes([first[0] ^ 1]))
            return result

        with patch("factory_ops.g3_storage._sha_stream", side_effect=mutate_after_hash):
            with self.assertRaisesRegex(ValueError, "bytes changed"):
                self.prepare(name="source-mutated", chunk_bytes=300)
        namespace = self.root / "source-mutated"
        self.assertFalse((namespace / "envelope.json").exists())
        self.assertEqual(1, len(list(namespace.glob("prepare-error-*.json"))))

    def test_repository_privacy_change_fails_before_success_receipt(self) -> None:
        envelope = self.prepare(chunk_bytes=10_000)
        api = FakeReleaseApi()
        api.private_sequence = [True, False]
        with self.assertRaisesRegex(ValueError, "privacy"):
            upload_envelope(envelope, api)
        self.assertFalse((envelope.parent / "uploaded-envelope.json").exists())
        receipts = list(envelope.parent.glob("upload-error-*.json"))
        self.assertEqual(1, len(receipts))

    def test_readback_and_verify_recheck_privacy_before_success_receipts(self) -> None:
        uploaded, api = self.uploaded(chunk_bytes=10_000)
        api.private_sequence = [True, True, True, True, False]
        with self.assertRaisesRegex(ValueError, "privacy"):
            readback_envelope(uploaded, self.root / "privacy-readback", api)
        self.assertFalse((self.root / "privacy-readback" / "readback-receipt.json").exists())
        self.assertTrue(list((self.root / "privacy-readback").glob("readback-error-*.json")))

        api.private_sequence = []
        readback = readback_envelope(uploaded, self.root / "clean-readback", api)
        api.private_sequence = [True, True, False]
        output = self.root / "privacy-output.bundle"
        with self.assertRaisesRegex(ValueError, "privacy"):
            verify_readback(uploaded, readback, output, api)
        self.assertTrue(output.is_file())
        self.assertFalse(output.with_name(output.name + ".verification.json").exists())
        self.assertTrue(list(self.root.glob("verify-error-*.json")))

    def test_partial_upload_receipt_is_bound_sanitized_and_nonretrying(self) -> None:
        envelope = self.prepare(chunk_bytes=500)
        envelope_sha = hashlib.sha256(envelope.read_bytes()).hexdigest()
        api = FakeReleaseApi()
        api.fail_upload_at = 2
        with self.assertRaisesRegex(RuntimeError, "synthetic"):
            upload_envelope(envelope, api)
        receipt_path = next(envelope.parent.glob("upload-error-*.json"))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        self.assertEqual(envelope_sha, receipt["envelope_sha256"])
        self.assertEqual(1, len(receipt["completed_assets"]))
        self.assertEqual({"id", "name", "size", "server_digest"},
                         set(receipt["completed_assets"][0]))
        serialized = json.dumps(receipt)
        self.assertNotIn("http", serialized)
        self.assertNotIn("token", serialized)
        with self.assertRaisesRegex(ValueError, "failure receipt"):
            upload_envelope(envelope, api)
        self.assertEqual(1, len(api.assets))

    def test_post_upload_privacy_failure_preserves_attempted_remote_binding(self) -> None:
        envelope = self.prepare(chunk_bytes=10_000)
        api = FakeReleaseApi()
        api.private_sequence = [True, True, True, False, False]
        with self.assertRaisesRegex(ValueError, "after remote operation"):
            upload_envelope(envelope, api)
        receipt = json.loads(next(envelope.parent.glob("upload-error-*.json")).read_text())
        self.assertEqual([], receipt["completed_assets"])
        self.assertEqual(1, len(receipt["attempted_assets"]))
        self.assertEqual(10_000, receipt["attempted_assets"][0]["id"])
        self.assertEqual({"error_code"}, set(receipt["live_release_snapshot"]))
        self.assertEqual(1, len(api.assets))

    def test_hostile_remote_asset_name_is_rejected_and_never_persisted(self) -> None:
        envelope = self.prepare(chunk_bytes=10_000)

        class HostileNameApi(FakeReleaseApi):
            def upload(self, owner, repository, release_id, name, stream):
                row = super().upload(owner, repository, release_id, name, stream)
                row["name"] = "https://attacker.invalid/secret-token"
                return row

        api = HostileNameApi()
        with self.assertRaisesRegex(ValueError, "identity/state/digest"):
            upload_envelope(envelope, api)
        receipt = json.loads(next(envelope.parent.glob("upload-error-*.json")).read_text())
        serialized = json.dumps(receipt)
        self.assertNotIn("attacker.invalid", serialized)
        self.assertNotIn("secret-token", serialized)
        self.assertEqual([], receipt["attempted_assets"])
        self.assertEqual({"error_code"}, set(receipt["live_release_snapshot"]))

    def test_error_receipt_leaf_swap_preserves_fallback_for_every_phase(self) -> None:
        from factory_ops.g3_storage import _Directory, _write_error

        for phase in ("prepare", "upload", "readback", "verify"):
            directory_path = self.root / f"error-{phase}"
            with self.subTest(phase=phase), _Directory.create(
                    directory_path, f"{phase} error directory") as directory:
                original_fsync = os.fsync
                moved: list[Path] = []

                def racing_fsync(descriptor: int) -> None:
                    original_fsync(descriptor)
                    if moved:
                        return
                    target = Path(os.readlink(f"/proc/self/fd/{descriptor}"))
                    if target.name.startswith(f"{phase}-error-"):
                        preserved = target.with_name("attacker-preserved-primary.json")
                        target.rename(preserved)
                        moved.append(preserved)

                with patch("factory_ops.g3_storage.os.fsync", side_effect=racing_fsync):
                    _write_error(directory, phase, RuntimeError("secret payload"))
                self.assertEqual(1, len(moved))
                fallback = list(directory_path.glob("storage-error-fallback-*.json"))
                self.assertEqual(1, len(fallback))
                document = json.loads(fallback[0].read_text())
                self.assertEqual(phase, document["phase"])
                self.assertEqual("RuntimeError", document["error_code"])
                self.assertEqual("FileNotFoundError", document["receipt_write_error_code"])
                self.assertNotIn("secret payload", json.dumps(document))

    def test_remote_digest_extra_asset_download_tamper_and_output_collision_fail(self) -> None:
        uploaded, api = self.uploaded(chunk_bytes=10_000)
        api.assets[0]["digest"] = "sha256:" + "f" * 64
        with self.assertRaisesRegex(ValueError, "exact asset set"):
            readback_envelope(uploaded, self.root / "bad", api)
        api.assets[0]["digest"] = "sha256:" + load_envelope(uploaded)["assets"][0]["sha256"]
        api.payloads[10_000] += b"tamper"
        with self.assertRaisesRegex(ValueError, "downloaded asset"):
            readback_envelope(uploaded, self.root / "tampered", api)
        self.assertTrue(list((self.root / "tampered").glob("readback-error-*.json")))
        api.payloads[10_000] = self.payload
        readback = readback_envelope(uploaded, self.root / "clean", api)
        output = self.root / "existing.bundle"
        output.write_bytes(b"foreign")
        with self.assertRaises(FileExistsError):
            verify_readback(uploaded, readback, output, api)
        self.assertEqual(b"foreign", output.read_bytes())

    def test_hardlink_and_symlink_inputs_are_rejected(self) -> None:
        hardlink = self.root / "hard.bundle"
        os.link(self.archive, hardlink)
        with self.assertRaisesRegex(ValueError, "single-link"):
            prepare_envelope(
                archive=hardlink, output=self.root / "hard-out", source_commit=COMMIT,
                repository_id=1421505101, owner="mmporong", repository="factory-ops-twin",
                release_id=991, tag="storage-probe-001", target_commit=COMMIT,
                draft=True, chunk_bytes=300)
        hardlink.unlink()
        link = self.root / "link.bundle"
        link.symlink_to(self.archive)
        with self.assertRaises((OSError, ValueError)):
            prepare_envelope(
                archive=link, output=self.root / "link-out", source_commit=COMMIT,
                repository_id=1421505101, owner="mmporong", repository="factory-ops-twin",
                release_id=991, tag="storage-probe-001", target_commit=COMMIT,
                draft=True, chunk_bytes=300)

    def test_gap_duplicate_unsafe_name_and_false_approval_are_rejected(self) -> None:
        source = self.prepare()
        original = json.loads(source.read_text(encoding="utf-8"))
        variants = []
        for mutation in ("gap", "duplicate", "unsafe", "approval"):
            value = json.loads(json.dumps(original))
            if mutation == "gap":
                value["assets"][1]["offset"] += 1
            elif mutation == "duplicate":
                value["assets"][1]["name"] = value["assets"][0]["name"]
            elif mutation == "unsafe":
                value["assets"][0]["name"] = "../escape.part"
            else:
                value["proof_flags"]["g008_approved"] = True
            variants.append(value)
        for index, value in enumerate(variants):
            path = self.root / f"hostile-{index}.json"
            path.write_text(json.dumps(value), encoding="utf-8")
            with self.subTest(index=index), self.assertRaises(ValueError):
                load_envelope(path)


class PlatformAndGhTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "nt", "Windows-only fail-closed CLI regression")
    def test_windows_cli_rejects_before_mutation(self) -> None:
        with tempfile.TemporaryDirectory() as value:
            root = Path(value)
            archive = root / "source.bundle"
            archive.write_bytes(b"source")
            with self.assertRaisesRegex(RuntimeError, "Linux or WSL"):
                main(["prepare", "--archive", str(archive), "--output", str(root / "out"),
                      "--source-commit", COMMIT, "--repository-id", "1421505101",
                      "--owner", "mmporong", "--repository", "factory-ops-twin",
                      "--release-id", "991", "--tag", "storage-probe-001",
                      "--target-commit", COMMIT, "--draft"])
            self.assertFalse((root / "out").exists())

    def test_lexical_absolute_does_not_resolve_symlink(self) -> None:
        path = Path("relative") / "link"
        self.assertEqual(Path.cwd() / path, _absolute_lexical(path))

    def test_gh_upload_uses_absolute_uploads_url_and_same_stdin_handle(self) -> None:
        api = GhReleaseApi("gh-test")
        response = subprocess.CompletedProcess([], 0, json.dumps({
            "id": 1, "name": "g3se-v1-000001-of-000001-" + "a" * 64 + ".part",
            "size": 14, "state": "uploaded", "digest": "sha256:" + "a" * 64,
        }).encode(), b"")
        name = "g3se-v1-000001-of-000001-" + "a" * 64 + ".part"
        with tempfile.TemporaryFile() as payload:
            payload.write(b"verified-bytes")
            payload.flush()
            with patch("factory_ops.g3_storage.subprocess.run", return_value=response) as called:
                api.upload("mmporong", "factory-ops-twin", 991, name,
                           cast(BinaryIO, payload))
        argv = called.call_args.args[0]
        self.assertIn("--input", argv)
        self.assertEqual("-", argv[argv.index("--input") + 1])
        self.assertIn("Content-Length: 14", argv)
        self.assertIn(f"https://uploads.github.com/repos/mmporong/factory-ops-twin/releases/991/assets?name={name}", argv)
        self.assertNotIn("--hostname", argv)
        self.assertIs(payload, called.call_args.kwargs["stdin"])

    def test_real_gh_sends_fixed_content_length_to_local_server(self) -> None:
        executable = shutil.which("gh")
        if executable is None:
            self.skipTest("gh executable is unavailable")
        observed: dict[str, object] = {}

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def do_POST(self) -> None:
                observed["content_length"] = self.headers.get("Content-Length")
                observed["transfer_encoding"] = self.headers.get("Transfer-Encoding")
                length = int(self.headers.get("Content-Length", "0"))
                observed["body"] = self.rfile.read(length)
                payload = b'{"ok":true}'
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, format, *args) -> None:
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, name="gh-wire-probe")
        thread.start()
        body = b"verified-local-wire"
        try:
            host = f"127.0.0.1:{server.server_port}"
            with tempfile.TemporaryDirectory() as config, tempfile.TemporaryFile() as source:
                source.write(body)
                source.seek(0)
                environment = os.environ.copy()
                for key in ("GH_TOKEN", "GITHUB_TOKEN", "GH_ENTERPRISE_TOKEN"):
                    environment.pop(key, None)
                environment.update({"GH_CONFIG_DIR": config, "GH_HOST": "github.com",
                                    "GH_TOKEN": "dummy-local-only"})
                completed = subprocess.run([
                    executable, "api", "--method", "POST",
                    "-H", "Content-Type: application/octet-stream",
                    "-H", f"Content-Length: {len(body)}", "--input", "-",
                    f"http://{host}/upload"], stdin=source, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, env=environment, check=False, timeout=10)
            self.assertEqual(0, completed.returncode)
            self.assertEqual(str(len(body)), observed.get("content_length"))
            self.assertIsNone(observed.get("transfer_encoding"))
            self.assertEqual(body, observed.get("body"))
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)
        self.assertFalse(thread.is_alive())

    def test_gh_errors_hide_stderr(self) -> None:
        api = GhReleaseApi("gh-test")
        failed = subprocess.CompletedProcess([], 17, b"", b"secret-token-value")
        with patch("factory_ops.g3_storage.subprocess.run", return_value=failed):
            with self.assertRaisesRegex(RuntimeError, "status 17") as caught:
                api.repository("mmporong", "factory-ops-twin")
        self.assertNotIn("secret-token-value", str(caught.exception))


if __name__ == "__main__":
    unittest.main()
