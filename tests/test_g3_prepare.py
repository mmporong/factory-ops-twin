"""Verify local G3a packaging and rejection of stale or relabelled artifacts."""
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.g3_prepare import CONFIG, FILES, G3B_GATES, MANIFEST_SCOPE, prepare_g3, verify_g3


class EmbeddedDataParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.in_data = False
        self.data = ""

    def handle_starttag(self, tag, attrs):
        if tag == "script" and dict(attrs).get("id") == "data":
            self.in_data = True

    def handle_endtag(self, tag):
        if tag == "script":
            self.in_data = False

    def handle_data(self, data):
        if self.in_data:
            self.data += data


class G3PreparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.output = Path(cls.temporary.name) / "package"
        cls.result = prepare_g3(CONFIG, cls.output)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_source_bound_package_has_exact_artifacts_and_pending_inputs(self):
        self.assertTrue(self.result["verified"])
        self.assertEqual(self.result["files_checked"], 16)
        self.assertEqual(self.result["routes_checked"], 112)
        self.assertEqual(self.result["pending_trials"], 60)
        self.assertFalse(self.result["gpu_execution_allowed"])
        self.assertEqual({p.relative_to(self.output).as_posix() for p in self.output.rglob("*") if p.is_file()}, FILES | {"manifest.json"})
        maps = [json.loads((self.output / f"maps/{state}/manifest.json").read_text(encoding="utf-8")) for state in ("open", "closed")]
        self.assertFalse(maps[0]["dynamic_barrier_included"])
        self.assertTrue(maps[1]["dynamic_barrier_included"])
        self.assertNotEqual(maps[0]["files"]["map.pgm"], maps[1]["files"]["map.pgm"])

    def test_report_embeds_the_same_route_data(self):
        parser = EmbeddedDataParser()
        parser.feed((self.output / "report.html").read_text(encoding="utf-8"))
        embedded = json.loads(parser.data)
        report = json.loads((self.output / "route-preflight.json").read_text(encoding="utf-8"))
        self.assertEqual(embedded["routes"], report["routes"])
        self.assertEqual(embedded["radius"], report["planning_radius_m"])

    def test_existing_output_is_preserved(self):
        before = (self.output / "manifest.json").read_bytes()
        with self.assertRaisesRegex(ValueError, "new dedicated"):
            prepare_g3(CONFIG, self.output)
        self.assertEqual(before, (self.output / "manifest.json").read_bytes())

    def test_modified_file_and_extra_file_are_rejected(self):
        target = self.output / "report.html"
        before = target.read_bytes()
        try:
            target.write_bytes(before + b"modified")
            with self.assertRaisesRegex(ValueError, "artifact differs"):
                verify_g3(self.output)
        finally:
            target.write_bytes(before)
        extra = self.output / "unexpected.txt"
        try:
            extra.write_text("unexpected", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "file set"):
                verify_g3(self.output)
        finally:
            extra.unlink()

    def test_physical_relabel_is_rejected(self):
        path = self.output / "manifest.json"
        before = path.read_bytes()
        manifest = json.loads(before)
        manifest["gpu_execution_allowed"] = True
        try:
            path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "nonphysical"):
                verify_g3(self.output)
        finally:
            path.write_bytes(before)

    def test_missing_route_is_rejected_even_with_updated_artifact_hash(self):
        report_path, manifest_path = self.output / "route-preflight.json", self.output / "manifest.json"
        report_before, manifest_before = report_path.read_bytes(), manifest_path.read_bytes()
        report, manifest = json.loads(report_before), json.loads(manifest_before)
        report["routes"].pop()
        report["route_count"] = len(report["routes"])
        try:
            report_path.write_text(json.dumps(report), encoding="utf-8")
            manifest["files"]["route-preflight.json"] = {"bytes": report_path.stat().st_size, "sha256": hashlib.sha256(report_path.read_bytes()).hexdigest()}
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "derived layout"):
                verify_g3(self.output)
        finally:
            report_path.write_bytes(report_before)
            manifest_path.write_bytes(manifest_before)

    def test_changed_route_distance_is_rejected_even_with_updated_artifact_hash(self):
        report_path, manifest_path = self.output / "route-preflight.json", self.output / "manifest.json"
        report_before, manifest_before = report_path.read_bytes(), manifest_path.read_bytes()
        report, manifest = json.loads(report_before), json.loads(manifest_before)
        report["routes"][0]["distance_m"] += 0.123
        try:
            report_path.write_text(json.dumps(report), encoding="utf-8")
            manifest["files"]["route-preflight.json"] = {
                "bytes": report_path.stat().st_size,
                "sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
            }
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "derived layout"):
                verify_g3(self.output)
        finally:
            report_path.write_bytes(report_before)
            manifest_path.write_bytes(manifest_before)

    def test_manifest_schema_scope_and_gate_contract_is_exact(self):
        path = self.output / "manifest.json"
        before = path.read_bytes()
        mutations = (
            ("schema", lambda value: value.update(schema_version=999), "schema_version"),
            ("scope", lambda value: value.update(scope="GPU physical performance verified"), "scope"),
            ("gates", lambda value: value.update(g3b_gates=[]), "scope or G3b gates"),
            ("extra", lambda value: value.update(unexpected=True), "schema fields"),
        )
        try:
            for name, mutate, message in mutations:
                with self.subTest(name=name):
                    manifest = json.loads(before)
                    mutate(manifest)
                    path.write_text(json.dumps(manifest), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, message):
                        verify_g3(self.output)
                    path.write_bytes(before)
        finally:
            path.write_bytes(before)
        manifest = json.loads(before)
        self.assertEqual(MANIFEST_SCOPE, manifest["scope"])
        self.assertEqual(G3B_GATES, manifest["g3b_gates"])

    def test_changed_inputs_do_not_mutate_target_and_partial_package_cannot_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "config"
            shutil.copytree(CONFIG, config)
            target_before = {name: (self.output / name).read_bytes() for name in FILES | {"manifest.json"}}
            routing = config / "g3-routing.json"
            routing.write_bytes(routing.read_bytes() + b"\n")
            with self.assertRaisesRegex(ValueError, "source inputs"):
                verify_g3(self.output, config_dir=config)
            self.assertEqual(
                target_before,
                {name: (self.output / name).read_bytes() for name in FILES | {"manifest.json"}},
            )

            partial = Path(temporary) / "partial"
            shutil.copytree(self.output, partial)
            (partial / "report.html").unlink()
            with self.assertRaisesRegex(ValueError, "file set"):
                verify_g3(partial)

    def test_custom_config_binding_uses_the_actual_config_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "config"
            shutil.copytree(CONFIG, config)
            routing = config / "g3-routing.json"
            routing.write_bytes(routing.read_bytes() + b"\n")
            inconsistent = Path(temporary) / "inconsistent"
            with self.assertRaisesRegex(ValueError, "source binding differs"):
                prepare_g3(config, inconsistent)
            self.assertFalse(inconsistent.exists())

            experiment_path = config / "g3-experiment.json"
            experiment = json.loads(experiment_path.read_text(encoding="utf-8"))
            for binding in experiment["source_bindings"]:
                if binding["path"] == "configs/factory-v3/g3-routing.json":
                    binding["sha256"] = hashlib.sha256(routing.read_bytes()).hexdigest()
            experiment_path.write_text(
                json.dumps(experiment, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
                newline="\n",
            )
            consistent = Path(temporary) / "consistent"
            result = prepare_g3(config, consistent)
            self.assertTrue(result["verified"])
            self.assertEqual(112, result["routes_checked"])
            self.assertEqual(60, result["pending_trials"])

    def test_stale_generator_identity_is_rejected(self):
        with patch("factory_ops.g3_prepare._generators", return_value={"changed.py": "0" * 64}):
            with self.assertRaisesRegex(ValueError, "generator bytes"):
                verify_g3(self.output)


if __name__ == "__main__":
    unittest.main()
