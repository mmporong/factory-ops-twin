"""Actual Git object fixtures test source binding, not physical execution."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import tarfile
import unittest
from unittest.mock import patch

from factory_ops.g3_session import prepare_session, sha
from scripts.ros.g3_evidence import validate_fixed_source, validate_g3_evidence
from tests import test_source_provenance as source_fixture


class FixedSourceEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = source_fixture.SourceProvenanceTests()
        self.fixture.setUp()
        self.base = self.fixture.base
        self.output = self.base / "run"
        self.snapshot = self.base / "snapshot"
        self.snapshot.mkdir()
        self.commit = self.fixture.commit
        archive, commit_object, pins = self.fixture.archive, self.fixture.commit_object, self.fixture.pins
        proof = self.fixture._verify()
        provenance = self.output / "provenance"
        provenance.mkdir(parents=True)
        shutil.copyfile(archive, provenance / "source.tar")
        shutil.copyfile(commit_object, provenance / "source-commit.raw")
        with tarfile.open(archive) as source:
            source.extractall(self.snapshot, filter="data")
        self.environment_path = self.output / "provenance/source-environment.json"
        environment = {"schema_version": 1, "run_id": "a" * 32,
                       "source_commit": self.commit, "checkout_head": self.commit,
                       "checkout_clean": True, "source_git_object_verification": True,
                       "source_archive_sha256": proof["source_archive_sha256"],
                       "source_commit_object_sha256": proof["commit_object_sha256"],
                       "source_snapshot_relative": "project", "source_files": pins, "files": pins,
                       "source_verification": proof}
        self.environment_path.write_text(json.dumps(environment), encoding="utf-8")
        self.environment_sha = sha(self.environment_path)

    def tearDown(self) -> None:
        self.fixture.tearDown()

    def inspect(self, *, commit: str | None = None, environment_sha: str | None = None):
        errors: list[str] = []
        result = validate_fixed_source(
            self.output, "a" * 32, commit or self.commit, environment_sha or self.environment_sha,
            errors, snapshot_root=self.snapshot,
        )
        return result, errors

    def test_actual_git_tree_and_runtime_snapshot_reconstruct_commit(self) -> None:
        result, errors = self.inspect()
        self.assertEqual(errors, [])
        self.assertEqual(result["commit"], self.commit)
        self.assertTrue(result["verification"]["snapshot_checked"])

    def test_supervisor_environment_hash_and_commit_cannot_be_replaced(self) -> None:
        for overrides in ({"commit": "0" * 40}, {"environment_sha": "0" * 64}):
            with self.subTest(overrides=overrides):
                result, errors = self.inspect(**overrides)
                self.assertEqual(result, {})
                self.assertTrue(errors)

    def test_changed_runtime_source_fails_unchanged_archive(self) -> None:
        (self.snapshot / "pkg/module.py").write_text("print('changed runtime')\n", encoding="utf-8")
        result, errors = self.inspect()
        self.assertEqual(result, {})
        self.assertIn("runtime snapshot differ", errors[0])

    def test_mutated_commit_object_cannot_be_relabelled(self) -> None:
        path = self.output / "provenance/source-commit.raw"
        path.write_bytes(path.read_bytes() + b"changed\n")
        result, errors = self.inspect()
        self.assertEqual(result, {})
        self.assertTrue(errors)

    def test_failed_startup_returns_failed_report_when_scene_is_missing(self) -> None:
        session_root = self.base / "session"
        prepare_session(session_root)
        (self.output / "pair-exit.json").write_text(json.dumps({
            "run_id": "a" * 32, "profile": "qualification", "policy": "fifo",
            "condition": "normal", "seed": 0, "nav_exit_code": 130, "sim_exit_code": 2,
        }), encoding="utf-8")
        (self.output / "mission-result.json").write_text(json.dumps({"status": "failed"}), encoding="utf-8")
        before = {path.relative_to(self.output): path.read_bytes()
                  for path in self.output.rglob("*") if path.is_file()}
        with patch.dict("os.environ", {"FACTORY_OPS_G3_SESSION_SHA256": sha(session_root / "session.json")}), \
                patch("scripts.ros.g3_evidence.validate_fixed_source", return_value={}):
            report = validate_g3_evidence(session_root, self.output, "a" * 32)
        self.assertFalse(report["passed"])
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["trial_outcome"], "failed")
        self.assertTrue(report["errors"])
        self.assertEqual(before, {path.relative_to(self.output): path.read_bytes()
                                  for path in self.output.rglob("*") if path.is_file()})


if __name__ == "__main__":
    unittest.main()
