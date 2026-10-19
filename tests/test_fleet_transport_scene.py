"""Fail closed on stale attempts before importing an Isaac SDK application."""
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.fleet_transport import prepare_bundle
from scripts.isaac.fleet4_transport_scene import FleetTransportLifecycle, SOURCE_PINS


class FleetTransportSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temp.name)
        cls.bundle = cls.root / "bundle"
        prepare_bundle(cls.bundle)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.lifecycle = FleetTransportLifecycle(self.bundle, "a" * 32)
        self.output = self.root / self._testMethodName
        (self.output / "provenance").mkdir(parents=True)
        self.marker = {"run_id": "a" * 32, "scenario_id": self.lifecycle.session["scenario_id"],
                       "source_commit": "b" * 40, "bundle_sha256": self.lifecycle.compiled["bundle_sha256"]}
        self.metadata = {**self.marker, "source_archive_sha256": "c" * 64,
                         "source_files": {name: "d" * 64 for name in SOURCE_PINS}}
        (self.output / "fleet-transport-verified.json").write_text(json.dumps(self.marker), encoding="utf-8")
        (self.output / "provenance/source.tar").write_bytes(b"archive bytes are verified by the source-provenance boundary")
        (self.output / "provenance/source-commit.raw").write_bytes(b"commit object")
        self.metadata_path = self.output / "provenance/fleet-source.json"
        self.metadata_path.write_text(json.dumps(self.metadata), encoding="utf-8")
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.dict(os.environ, {"FACTORY_OPS_FLEET4_PROVENANCE_SHA256":
            hashlib.sha256(self.metadata_path.read_bytes()).hexdigest()}))
        self.stack.enter_context(patch("scripts.isaac.fleet4_transport_scene.transport_verification_record", return_value=self.marker))
        self.proof = self.stack.enter_context(patch("scripts.isaac.fleet4_transport_scene.verify_source_archive",
            return_value={"passed": True, "source_archive_sha256": "c" * 64}))

    def prepare(self):
        return self.lifecycle.prepare_worker_paths(self.bundle / "scene/factory-v3.usda", self.output)

    def test_valid_bootstrap_requires_source_proof_for_the_mounted_snapshot(self):
        scene, output = self.prepare()
        self.assertEqual(self.output, output)
        self.assertEqual(self.bundle / "scene/factory-v3.usda", scene)
        self.assertEqual("b" * 40, self.proof.call_args.args[2])
        self.assertEqual(SOURCE_PINS, set(self.proof.call_args.args[3]))
        self.assertIn("snapshot_root", self.proof.call_args.kwargs)

    def test_existing_attempt_content_is_preserved_and_rejected(self):
        original = self.output / "original-result.json"
        original.write_text("preserve")
        with self.assertRaisesRegex(ValueError, "exact file set"):
            self.prepare()
        self.assertEqual("preserve", original.read_text())
        self.proof.assert_not_called()

    def test_empty_unexpected_directory_is_also_a_stale_attempt(self):
        (self.output / "old-frames").mkdir()
        with self.assertRaisesRegex(ValueError, "exact file set"):
            self.prepare()

    def test_wrong_run_marker_is_rejected_before_source_or_sdk_work(self):
        (self.output / "fleet-transport-verified.json").write_text(json.dumps({**self.marker, "run_id": "f" * 32}))
        with self.assertRaisesRegex(ValueError, "marker differs"):
            self.prepare()
        self.proof.assert_not_called()

    def test_changed_supervisor_metadata_cannot_change_the_source_commit(self):
        self.metadata_path.write_text(json.dumps({**self.metadata, "source_commit": "f" * 40}))
        with self.assertRaisesRegex(ValueError, "metadata is not bound"):
            self.prepare()

    def test_git_archive_failure_cannot_start_the_sdk(self):
        self.proof.return_value = {"passed": False, "errors": ["commit tree differs"]}
        with self.assertRaisesRegex(ValueError, "source archive proof differs"):
            self.prepare()


if __name__ == "__main__":
    unittest.main()
