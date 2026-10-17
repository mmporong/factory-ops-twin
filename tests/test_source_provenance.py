from __future__ import annotations

from copy import copy
import hashlib
from io import BytesIO
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from typing import Any
import unittest

from factory_ops.source_provenance import verify_source_archive


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_object_sha1(kind: bytes, payload: bytes) -> str:
    return hashlib.sha1(kind + b" " + str(len(payload)).encode("ascii") + b"\0" + payload).hexdigest()


class SourceProvenanceTests(unittest.TestCase):
    def setUp(self) -> None:
        if shutil.which("git") is None:
            self.skipTest("git is required for source provenance tests")
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.repo = self.base / "repo"
        self.repo.mkdir()
        self._git("init", "--initial-branch=main")
        self._git("config", "user.name", "Source Test")
        self._git("config", "user.email", "source@example.invalid")
        self._git("config", "core.autocrlf", "false")
        (self.repo / "pkg").mkdir()
        (self.repo / "README.md").write_bytes(b"factory source\n")
        (self.repo / "pkg/module.py").write_bytes(b"VALUE = 7\n")
        (self.repo / "pkg.txt").write_bytes(b"sorts before pkg tree under Git byte ordering\n")
        (self.repo / "script.sh").write_bytes(b"#!/bin/sh\nprintf 'ok\\n'\n")
        self._git("add", "README.md", "pkg/module.py", "pkg.txt", "script.sh")
        self._git("update-index", "--chmod=+x", "script.sh")
        self._git("commit", "-m", "fixture")
        self.commit = self._git("rev-parse", "HEAD").decode("ascii").strip()
        self.archive = self.base / "source.tar"
        self._git("archive", "--format=tar", f"--output={self.archive}", self.commit)
        self.commit_object = self.base / "commit.raw"
        self.commit_object.write_bytes(self._git("cat-file", "commit", self.commit))
        self.pins = {
            path: _sha256(self.repo / path)
            for path in ("README.md", "pkg/module.py", "script.sh")
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_actual_git_archive_reconstructs_commit_and_pins_snapshot(self) -> None:
        report = self._verify()

        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["commit_sha1"], self.commit)
        self.assertEqual(report["commit_tree_sha1"], report["archive_tree_sha1"])
        self.assertEqual(report["source_archive_sha256"], _sha256(self.archive))
        self.assertEqual(report["source_pin_count"], 3)
        modes = {item["path"]: item["mode"] for item in report["source_files"]}
        self.assertEqual(modes["script.sh"], "100755")
        self.assertEqual(modes["README.md"], "100644")
        self.assertTrue(all(item["snapshot_sha256"] == item["sha256"] for item in report["source_files"]))

    def test_archive_member_order_does_not_change_canonical_git_tree(self) -> None:
        reordered = self._rewrite_archive("reordered.tar", reverse=True)

        report = self._verify(archive=reordered)

        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["archive_tree_sha1"], report["commit_tree_sha1"])

    def test_arbitrary_valid_commit_is_rejected_for_the_original_archive(self) -> None:
        (self.repo / "README.md").write_bytes(b"different commit\n")
        self._git("add", "README.md")
        self._git("commit", "-m", "second")
        other_commit = self._git("rev-parse", "HEAD").decode("ascii").strip()
        other_object = self.base / "other-commit.raw"
        other_object.write_bytes(self._git("cat-file", "commit", other_commit))

        report = self._verify(commit=other_commit, commit_object=other_object)

        self.assertFalse(report["passed"])
        self.assertIn("archive files and modes do not reconstruct the commit tree", report["errors"])

    def test_commit_object_byte_mutation_is_rejected(self) -> None:
        changed = self.base / "changed-commit.raw"
        changed.write_bytes(self.commit_object.read_bytes() + b"tamper")

        report = self._verify(commit_object=changed)

        self.assertFalse(report["passed"])
        self.assertIn("raw commit object SHA-1 does not match expected_commit", report["errors"])

    def test_forged_commit_with_noncanonical_reordered_tree_is_rejected(self) -> None:
        entries = [
            (b"100644", b"README.md", self._git("rev-parse", f"{self.commit}:README.md").strip()),
            (b"40000", b"pkg", self._git("rev-parse", f"{self.commit}:pkg").strip()),
            (b"100644", b"pkg.txt", self._git("rev-parse", f"{self.commit}:pkg.txt").strip()),
            (b"100755", b"script.sh", self._git("rev-parse", f"{self.commit}:script.sh").strip()),
        ]
        wrong_body = b"".join(mode + b" " + name + b"\0" + bytes.fromhex(object_id.decode())
                              for mode, name, object_id in reversed(entries))
        wrong_tree = _git_object_sha1(b"tree", wrong_body).encode("ascii")
        original = self.commit_object.read_bytes()
        _, newline, remainder = original.partition(b"\n")
        forged = b"tree " + wrong_tree + newline + remainder
        forged_path = self.base / "forged-commit.raw"
        forged_path.write_bytes(forged)
        forged_commit = _git_object_sha1(b"commit", forged)

        report = self._verify(commit=forged_commit, commit_object=forged_path)

        self.assertFalse(report["passed"])
        self.assertEqual(report["commit_sha1"], forged_commit)
        self.assertIn("archive files and modes do not reconstruct the commit tree", report["errors"])

    def test_file_content_mode_and_name_mutations_are_rejected(self) -> None:
        cases = {
            "content": self._rewrite_archive("content.tar", content_path="pkg/module.py", content=b"VALUE = 8\n"),
            "mode": self._rewrite_archive("mode.tar", mode_path="script.sh", mode=0o664),
            "name": self._rewrite_archive("name.tar", rename=("README.md", "README-renamed.md")),
        }
        for label, archive in cases.items():
            with self.subTest(label=label):
                report = self._verify(archive=archive)
                self.assertFalse(report["passed"])
                self.assertIn("archive files and modes do not reconstruct the commit tree", report["errors"])

    def test_unsafe_duplicate_and_link_members_fail_closed(self) -> None:
        archives = {
            "unsafe": self._rewrite_archive("unsafe.tar", extra=("../escape", b"bad", tarfile.REGTYPE)),
            "duplicate": self._rewrite_archive("duplicate.tar", extra=("README.md", b"again", tarfile.REGTYPE)),
            "symlink": self._rewrite_archive("symlink.tar", extra=("alias", b"", tarfile.SYMTYPE)),
        }
        for label, archive in archives.items():
            with self.subTest(label=label):
                report = self._verify(archive=archive)
                self.assertFalse(report["passed"])
                self.assertTrue(any(word in " ".join(report["errors"]) for word in ("unsafe", "duplicated", "supported regular")))

    def test_missing_pin_and_pin_hash_mutation_are_rejected(self) -> None:
        cases = {
            "missing": {**self.pins, "missing.py": "0" * 64},
            "hash": {**self.pins, "pkg/module.py": "f" * 64},
        }
        for label, pins in cases.items():
            with self.subTest(label=label):
                report = self._verify(pins=pins)
                self.assertFalse(report["passed"])
                self.assertTrue(any("source pin" in error for error in report["errors"]))

    def test_snapshot_byte_mutation_is_rejected_even_when_archive_is_valid(self) -> None:
        (self.repo / "pkg/module.py").write_bytes(b"VALUE = 99\n")

        report = self._verify()

        self.assertFalse(report["passed"])
        self.assertIn("source pin SHA-256 does not match snapshot bytes: pkg/module.py", report["errors"])
        self.assertEqual(report["archive_tree_sha1"], report["commit_tree_sha1"])

    def _verify(
        self,
        *,
        archive: Path | None = None,
        commit: str | None = None,
        commit_object: Path | None = None,
        pins: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        return verify_source_archive(
            archive or self.archive,
            commit_object or self.commit_object,
            commit or self.commit,
            pins or self.pins,
            self.repo,
        )

    def _git(self, *arguments: str) -> bytes:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=self.repo,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return completed.stdout

    def _rewrite_archive(
        self,
        filename: str,
        *,
        reverse: bool = False,
        content_path: str | None = None,
        content: bytes | None = None,
        mode_path: str | None = None,
        mode: int | None = None,
        rename: tuple[str, str] | None = None,
        extra: tuple[str, bytes, bytes] | None = None,
    ) -> Path:
        output = self.base / filename
        with tarfile.open(self.archive, "r:") as source:
            members = source.getmembers()
            records: list[tuple[tarfile.TarInfo, bytes | None]] = []
            for original in members:
                member = copy(original)
                extracted = source.extractfile(original) if original.isfile() else None
                if original.isfile() and extracted is None:
                    self.fail(f"fixture archive member has no payload: {original.name}")
                payload = extracted.read() if extracted is not None else None
                if original.name == content_path:
                    payload = content
                    member.size = len(content or b"")
                if original.name == mode_path:
                    member.mode = mode or 0
                if rename is not None and original.name == rename[0]:
                    member.name = rename[1]
                records.append((member, payload))
        if reverse:
            records.reverse()
        with tarfile.open(output, "w:") as target:
            for member, payload in records:
                target.addfile(member, BytesIO(payload) if payload is not None else None)
            if extra is not None:
                name, payload, type_ = extra
                member = tarfile.TarInfo(name)
                member.type = type_
                if type_ == tarfile.SYMTYPE:
                    member.linkname = "README.md"
                else:
                    member.size = len(payload)
                target.addfile(member, BytesIO(payload) if member.isfile() else None)
        return output


if __name__ == "__main__":
    unittest.main()
