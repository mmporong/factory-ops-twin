"""Compact operations matrix report tests."""
from __future__ import annotations

from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.operations_report import build_report


SCENARIOS = ("normal", "tester_down", "aisle_closed")
POLICIES = ("fifo", "eta")


def _canonical(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _matrix() -> dict[str, object]:
    sources = {
        "config": {"g2.json": sha256(b"config\n").hexdigest()},
        "producer": {"factory_ops/a.py": sha256(b"producer\n").hexdigest()},
    }
    binding = sha256(_canonical(sources).encode("utf-8")).hexdigest()
    cases = []
    for index, (scenario, policy) in enumerate(
            (pair for scenario in SCENARIOS for pair in ((scenario, "fifo"), (scenario, "eta")))):
        status = "complete" if index == 0 else "incomplete"
        cases.append({
            "run_id": f"run-{index}", "status": status,
            "error": None, "scenario": scenario, "dispatch_policy": policy,
            "seed": 17, "seed_role": "identity_only_no_stochastic_inputs",
            "workload": {"order_count": 12, "release_interval_s": 30.0,
                         "sources": ["S1"] * 12, "due_offset_s": 240.0,
                         "tester_down_window_s": None, "aisle_closed_at_s": None},
            "simulation": {"horizon_s": 3600.0, "dt_s": 0.1},
            "case_identity": {"id": f"case-{index}", "spec": {"source_binding_sha256": binding}},
            "metrics": {
                "completed_cycles": 12 - index, "empty_returns_completed": max(0, 12 - 2 * index),
                "mean_wip": 1.0 + index, "traffic_wait_s": 10.0 * index,
                "total_tardiness_lower_bound_s": 20.0 * index,
                "elapsed_wall_s": 0.5 + index, "business_finished": status == "complete",
                "fleet_quiescent": status == "complete",
            },
            "final_snapshot": {"traffic": {"observed_min_clearance_m": 0.6 - index * 0.01,
                                             "peak_moving_robots": min(4, index + 1)}},
            "provenance": {"scope": "authored CPU comparison; no physics claim",
                           "source_stable_during_run": True, "sources_before_sha256": sources},
            "representative_snapshots": [{"large": "DO_NOT_COPY"}],
        })
    return {"schema_version": 1, "execution_mode": "operations", "cases": cases}


class OperationsReportTest(unittest.TestCase):
    def test_builds_svg_markdown_and_condensed_json_without_snapshots(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "matrix.json"
            source.write_text(json.dumps(_matrix()), encoding="utf-8")
            outputs = build_report(source, root / "report")
            machine = json.loads(outputs["json"].read_text(encoding="utf-8"))
            markdown = outputs["markdown"].read_text(encoding="utf-8")
            svg = outputs["svg"].read_text(encoding="utf-8")

            self.assertEqual(6, machine["comparison"]["case_count"])
            self.assertEqual(sha256(source.read_bytes()).hexdigest(), machine["input"]["sha256"])
            self.assertNotIn("representative_snapshots", _canonical(machine))
            self.assertNotIn("DO_NOT_COPY", _canonical(machine))
            self.assertIn("| aisle_closed | eta | incomplete |", markdown)
            self.assertIn("벽시계 실행 시간", markdown)
            self.assertIn("Completed cycles", svg)
            self.assertIn("Tardiness lower bound (s)", svg)
            self.assertIn("Non-complete or unstable", svg)
            self.assertEqual("run-0", machine["cases"][0]["run_id"])
            self.assertEqual("authored CPU comparison; no physics claim", machine["cases"][0]["scope"])

    def test_rejects_different_source_bindings(self) -> None:
        document = _matrix()
        changed = deepcopy(document["cases"][5]["provenance"]["sources_before_sha256"])
        changed["config"]["g2.json"] = "c" * 64
        document["cases"][5]["provenance"]["sources_before_sha256"] = changed
        document["cases"][5]["case_identity"]["spec"]["source_binding_sha256"] = sha256(
            _canonical(changed).encode("utf-8")).hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "matrix.json"
            source.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "bindings differ"):
                build_report(source, root / "report")

    def test_binds_verified_sibling_frozen_source_manifest(self) -> None:
        document = _matrix()
        binding = document["cases"][0]["case_identity"]["spec"]["source_binding_sha256"]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "matrix.json"
            source.write_text(json.dumps(document), encoding="utf-8")
            frozen = root / "frozen-source"
            (frozen / "config").mkdir(parents=True)
            (frozen / "factory_ops").mkdir()
            (frozen / "config" / "g2.json").write_bytes(b"config\n")
            (frozen / "factory_ops" / "a.py").write_bytes(b"producer\n")
            manifest = {
                "schema_version": 1,
                "source_binding_sha256": binding,
                "files": [
                    {"kind": "config", "source_key": "g2.json", "frozen_path": "config/g2.json",
                     "sha256": sha256(b"config\n").hexdigest()},
                    {"kind": "producer", "source_key": "factory_ops/a.py",
                     "frozen_path": "factory_ops/a.py", "sha256": sha256(b"producer\n").hexdigest()},
                ],
            }
            raw_manifest = json.dumps(manifest).encode("utf-8")
            (frozen / "manifest.json").write_bytes(raw_manifest)
            outputs = build_report(source, root / "report")
            summary = json.loads(outputs["json"].read_text(encoding="utf-8"))
            self.assertEqual(sha256(raw_manifest).hexdigest(), summary["frozen_source"]["manifest_sha256"])
            self.assertEqual(2, summary["frozen_source"]["file_count"])

    def test_rejects_condition_mismatch_and_preserves_failed_error(self) -> None:
        document = _matrix()
        document["cases"][1]["simulation"]["dt_s"] = 0.2
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "matrix.json"
            source.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "same workload"):
                build_report(source, root / "report")

        document = _matrix()
        document["cases"][2]["status"] = "failed"
        document["cases"][2]["error"] = {"type": "RuntimeError", "message": "blocked"}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "matrix.json"
            source.write_text(json.dumps(document), encoding="utf-8")
            outputs = build_report(source, root / "report")
            machine = json.loads(outputs["json"].read_text(encoding="utf-8"))
            self.assertEqual("failed", machine["cases"][2]["status"])
            self.assertEqual("blocked", machine["cases"][2]["error"]["message"])


if __name__ == "__main__":
    unittest.main()
