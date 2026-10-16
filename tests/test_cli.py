from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import csv
import hashlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.cli import DEFAULT_CONFIG, main, prepare, verify_manifest, write_csv
from factory_ops.layout import Layout, load_traffic
from factory_ops.report import layout_svg, write_report
from factory_ops.smoke import PlanningTravel, run_smoke


class CliTests(unittest.TestCase):
    def test_prepare_exports_hashes_and_replay_detects_mismatch(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            result = prepare(DEFAULT_CONFIG, output)
            self.assertEqual(result["backend"], "cpu_event_model")
            manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
            self.assertFalse(manifest["isaac_validated"])
            self.assertFalse(manifest["nav2_validated"])
            self.assertFalse(manifest["cloud_resources_created"])
            self.assertTrue(verify_manifest(output)["verified"])
            for name, digest in manifest["files"].items():
                self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)
            for name in ("map.pgm", "keepout.pgm"):
                self.assertTrue((output / "maps" / name).read_bytes().startswith(b"P5\n"))
            self.assertTrue((output / "scene" / "factory-v3.usda").read_text().startswith("#usda 1.0"))
            for policy in ("fifo", "eta"):
                arguments = ["replay", str(output / policy / "events.jsonl"),
                             "--expected", str(output / policy / "snapshot.json")]
                with redirect_stdout(io.StringIO()):
                    self.assertEqual(main(arguments), 0)
                snapshot = output / policy / "snapshot.json"
                changed = json.loads(snapshot.read_text(encoding="utf-8"))
                changed["ledger_seq"] += 1
                snapshot.write_text(json.dumps(changed), encoding="utf-8")
                with redirect_stdout(io.StringIO()):
                    self.assertEqual(main(arguments), 1)
            with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                verify_manifest(output)

    def test_replay_rejects_event_metadata_tampering(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            prepare(DEFAULT_CONFIG, output)
            path = output / "fifo" / "events.jsonl"
            events = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            events[-1]["ledger_seq"] += 1
            path.write_text("\n".join(json.dumps(event) for event in events), encoding="utf-8")
            with redirect_stderr(io.StringIO()), redirect_stdout(io.StringIO()):
                self.assertEqual(main(["replay", str(path), "--expected", str(output / "fifo" / "snapshot.json")]), 1)

    def test_artifacts_are_independent_of_checkout_and_output_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            manifests = []
            for name in ("checkout-a", "checkout-b"):
                config_dir = root / name / "configs"
                config_dir.mkdir(parents=True)
                for config in ("layout.json", "traffic.json", "smoke.json"):
                    shutil.copyfile(DEFAULT_CONFIG / config, config_dir / config)
                output = root / name / "output"
                prepare(config_dir, output)
                manifests.append(json.loads((output / "manifest.json").read_text(encoding="utf-8")))
            self.assertEqual(manifests[0], manifests[1])

    def test_unrelated_output_files_are_preserved_and_rejected_before_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            unrelated = output / "notes.txt"
            unrelated.write_text("keep me", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unrelated files"):
                prepare(DEFAULT_CONFIG, output)
            self.assertEqual(unrelated.read_text(encoding="utf-8"), "keep me")
            self.assertEqual(list(output.iterdir()), [unrelated])

    def test_verify_rejects_extra_files_and_missing_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            output = root / "generated"
            prepare(DEFAULT_CONFIG, output)
            (output / "extra.txt").write_text("extra", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "actual artifact file set"):
                verify_manifest(output)
            (output / "extra.txt").rename(root / "saved-extra.txt")
            (output / "fifo" / "orders.csv").rename(root / "saved-orders.csv")
            with self.assertRaisesRegex(ValueError, "actual artifact file set"):
                verify_manifest(output)

    def test_csv_formula_prefixes_are_literal_and_json_identifiers_are_preserved(self):
        inputs = ["=HYPERLINK(\"x\")", "+command", "-robot", "@SUM(1)", "\ttext", "\n=1", "  =1", "O001"]
        rows = [{"identifier": value, "seconds": -2.5} for value in inputs]
        before = deepcopy(rows)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "orders.csv"
            write_csv(output, rows)
            with output.open(encoding="utf-8", newline="") as stream:
                displayed = list(csv.DictReader(stream))
            for index, value in enumerate(inputs[:-1]):
                self.assertEqual(displayed[index]["identifier"], "'" + value)
                self.assertEqual(displayed[index]["seconds"], "-2.5")
            self.assertEqual(displayed[-1]["identifier"], "O001")
            self.assertEqual(rows, before)

    def test_failed_traffic_check_blocks_artifact_generation(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            with patch("factory_ops.cli.run_traffic_checks", return_value={"passed": False}):
                with self.assertRaisesRegex(RuntimeError, "traffic checks failed"):
                    prepare(DEFAULT_CONFIG, output)
            self.assertEqual(list(output.iterdir()), [])

    def test_report_escapes_external_labels(self):
        layout = Layout.from_file(DEFAULT_CONFIG / "layout.json")
        config = json.loads((DEFAULT_CONFIG / "smoke.json").read_text(encoding="utf-8"))
        result = run_smoke(config, "fifo", PlanningTravel(layout))
        result = deepcopy(result)
        attack = '<script>alert("x")</script>'
        result["orders"][0]["order_id"] = attack
        result["events"][0]["event_type"] = attack
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "report.html"
            write_report(target, layout, layout.planning_analysis(load_traffic(DEFAULT_CONFIG / "traffic.json")), [result])
            html = target.read_text(encoding="utf-8")
            self.assertNotIn(attack, html)
            self.assertIn("&lt;script&gt;", html)
            self.assertIn('color-scheme:light', html)
            self.assertIn("<svg", layout_svg(layout))

    def test_missing_input_returns_failure(self):
        with tempfile.TemporaryDirectory() as folder, redirect_stderr(io.StringIO()):
            self.assertEqual(main(["replay", str(Path(folder) / "missing.jsonl")]), 1)


if __name__ == "__main__":
    unittest.main()
