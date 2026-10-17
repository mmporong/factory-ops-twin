from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.experiments import build_matrix, canonical_hash, load_experiment_config
from factory_ops.g3_supervisor import (build_collection_failure_result, create_attempt_start,
                                       finalize_attempt, validate_attempt_receipt)


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/factory-v3/g3-experiment.json"


def save(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SupervisorReceiptTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.settings = load_experiment_config(CONFIG)
        self.trial = build_matrix(self.settings)[0]
        self.source_commit = "1" * 40
        self.run_id = "2" * 32
        self.config = self.root / "inputs/g3-experiment.json"
        self.session = self.root / "inputs/session.json"
        self.trial_path = self.root / "inputs/trial.json"
        self.gate = self.root / "inputs/gate.json"
        self.billing = self.root / "resources/billing.json"
        save(self.config, self.settings)
        save(self.session, {"run_id": self.run_id, "profile": "primary",
                            "condition": self.trial["condition"], "policy": self.trial["policy"],
                            "seed": self.trial["seed"], "workload_sha256": self.trial["workload_sha256"],
                            "trial_input_sha256": self.trial["trial_input_sha256"],
                            "fixed_source_commit": self.source_commit,
                            "experiment_config_sha256": self.trial["source_config_sha256"]})
        save(self.trial_path, self.trial)
        gate = {"classification": "g3_actual_six_pilot_calibration", "execution_eligible": True,
                "status": "primary_execution_eligible", "source_commit": self.source_commit,
                "experiment_config_sha256": self.trial["source_config_sha256"]}
        gate["calibration_sha256"] = canonical_hash(gate)
        save(self.gate, gate)
        self.primary_inputs = self.root / "inputs/primary-inputs.json"
        save(self.primary_inputs, {"trial_path": str(self.trial_path.resolve()),
                                   "calibration_path": str(self.gate.resolve()),
                                   "pilot_slots": {}, "resource_input": {},
                                   "expected_source_commit": self.source_commit})
        save(self.billing, {"observed_at": "2026-10-17T23:59:00Z", "source": "Brev UI",
                            "confirmation_method": "authenticated UI capture", "currency": "USD",
                            "confirmed_hourly_rate_usd": 3.36, "budget_remaining_usd": 400.0})
        self.start_path = self.root / "resources/attempt-start.json"
        with patch("factory_ops.g3_supervisor.verify_session", return_value={"profile": "primary"}):
            create_attempt_start(
                self.start_path, self.root, run_id=self.run_id, profile="primary",
                condition=self.trial["condition"], policy=self.trial["policy"], seed=self.trial["seed"],
                source_commit=self.source_commit, config_path=self.config, session_root=self.session.parent,
                trial=self.trial, trial_path=self.trial_path, gate_path=self.gate,
                primary_inputs_path=self.primary_inputs, primary_inputs_sha256=sha(self.primary_inputs),
                billing_path=self.billing, free_disk_bytes_before=1_000_000,
                started_at_utc="2026-10-18T00:00:00Z",
            )
        self.payload = self.root / "outputs/payload.bin"
        self.payload.parent.mkdir(parents=True)
        self.payload.write_bytes(b"preserved partial output")
        self.manifest = self.root / "resources/output-manifest.json"
        save(self.manifest, {"schema_version": 1, "files": [
            {"path": "payload.bin", "bytes": self.payload.stat().st_size, "sha256": sha(self.payload)}
        ]})
        self.archive = self.root / "resources/output.tar.gz"
        with tarfile.open(self.archive, "w:gz") as bundle:
            bundle.add(self.payload, arcname="payload.bin")
        self.gpu = self.root / "resources/gpu.csv"
        self.gpu.write_text("2026-10-18T00:00:01Z, NVIDIA L40S, 20000, 50\n", encoding="utf-8")
        self.scene = self.root / "resources/scene.json"
        save(self.scene, {"run_id": self.run_id, "timing": {"final_sim_time_s": 10.0, "wall_seconds": 5.0}})
        self.disk = self.root / "resources/disk.json"
        save(self.disk, {"run_id": self.run_id, "free_disk_bytes_before": 1_000_000,
                         "free_disk_bytes_after": 900_000})
        self.receipt = self.root / "resources/attempt-receipt.json"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def finalize_failed(self) -> None:
        finalize_attempt(
            self.start_path, self.receipt, self.root,
            workers_finished_at_utc="2026-10-18T00:01:00Z",
            validation_finished_at_utc="2026-10-18T00:01:10Z",
            gpu_csv_path=self.gpu, scene_summary_path=self.scene,
            output_manifest_path=self.manifest, archive_path=self.archive, disk_path=self.disk,
            collection_failure_reason="validator report missing after worker exit",
        )

    def test_collection_failure_preserves_denominator_with_unknown_bounds(self) -> None:
        self.finalize_failed()
        with patch("factory_ops.g3_supervisor.verify_session", return_value={"profile": "primary"}):
            validated = validate_attempt_receipt(
                self.receipt, self.root, self.trial, expected_source_commit=self.source_commit
            )
        self.assertEqual("collection_failed", validated["attempt_status"])
        with patch("factory_ops.g3_supervisor.verify_session", return_value={"profile": "primary"}):
            result = build_collection_failure_result(
                self.trial, self.receipt, self.root, source_commit=self.source_commit,
                experiment_config_path=self.config,
            )
        self.assertEqual("failed", result["trial_outcome"])
        self.assertEqual("collection", result["failure_kind"])
        self.assertEqual("invalid", result["validation_status"])
        self.assertEqual("unknown", result["metrics_state"])
        self.assertIsNone(result["metrics"]["orders_completed"])
        self.assertEqual([0, len(self.trial["workload"]["orders"])],
                         result["metrics"]["orders_completed_bounds"])

    def test_start_and_final_receipts_are_exclusive_and_replay_bound(self) -> None:
        with patch("factory_ops.g3_supervisor.verify_session", return_value={"profile": "primary"}):
            with self.assertRaises(FileExistsError):
                create_attempt_start(
                    self.start_path, self.root, run_id=self.run_id, profile="primary",
                    condition=self.trial["condition"], policy=self.trial["policy"], seed=self.trial["seed"],
                    source_commit=self.source_commit, config_path=self.config, session_root=self.session.parent,
                    trial=self.trial, trial_path=self.trial_path, gate_path=self.gate,
                    primary_inputs_path=self.primary_inputs, primary_inputs_sha256=sha(self.primary_inputs),
                    billing_path=self.billing, free_disk_bytes_before=1_000_000,
                    started_at_utc="2026-10-18T00:00:00Z",
                )
        self.finalize_failed()
        with self.assertRaises(FileExistsError):
            self.finalize_failed()
        self.gate.write_text("tampered", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "referenced bytes differ"):
            validate_attempt_receipt(
                self.receipt, self.root, self.trial, expected_source_commit=self.source_commit
            )

    def test_archive_bytes_must_match_manifest_before_receipt_is_written(self) -> None:
        self.payload.write_bytes(b"changed after archive")
        bad_manifest = {"schema_version": 1, "files": [
            {"path": "payload.bin", "bytes": self.payload.stat().st_size, "sha256": sha(self.payload)}
        ]}
        save(self.manifest, bad_manifest)
        with self.assertRaisesRegex(ValueError, "archive contents differ"):
            self.finalize_failed()
        self.assertFalse(self.receipt.exists())

    def test_startup_collection_failure_preserves_missing_artifacts_as_null(self) -> None:
        receipt_path = self.root / "resources/startup-failure-receipt.json"
        receipt = finalize_attempt(
            self.start_path, receipt_path, self.root,
            workers_finished_at_utc="2026-10-18T00:00:10Z",
            validation_finished_at_utc="2026-10-18T00:00:11Z",
            gpu_csv_path=None, scene_summary_path=None, output_manifest_path=self.manifest,
            archive_path=self.archive, disk_path=self.disk,
            collection_failure_reason="session validation failed before workers started",
        )
        self.assertEqual("collection_failed", receipt["attempt_status"])
        self.assertIsNone(receipt["references"]["gpu_csv"])
        self.assertIsNone(receipt["references"]["scene_summary"])
        self.assertIsNone(receipt["references"]["report"])
        self.assertIsNone(receipt["references"]["mission"])

    def test_self_declared_primary_gate_cannot_open_a_physical_receipt(self) -> None:
        with self.assertRaises(ValueError):
            create_attempt_start(
                self.root / "resources/rejected-start.json", self.root, run_id=self.run_id,
                profile="primary", condition=self.trial["condition"], policy=self.trial["policy"],
                seed=self.trial["seed"], source_commit=self.source_commit, config_path=self.config,
                session_root=self.session.parent, trial=self.trial, trial_path=self.trial_path,
                gate_path=self.gate, primary_inputs_path=self.primary_inputs,
                primary_inputs_sha256=sha(self.primary_inputs), billing_path=self.billing,
                free_disk_bytes_before=1_000_000, started_at_utc="2026-10-18T00:00:00Z",
            )

    def test_config_bytes_and_trial_bytes_are_both_bound(self) -> None:
        self.finalize_failed()
        changed = dict(self.settings)
        changed["revision"] = "foreign"
        save(self.config, changed)
        with self.assertRaisesRegex(ValueError, "referenced bytes differ"):
            validate_attempt_receipt(
                self.receipt, self.root, self.trial, expected_source_commit=self.source_commit
            )

    def test_pilot_receipt_uses_session_identity_without_primary_trial_or_gate(self) -> None:
        pilot_run = "4" * 32
        pilot_workload = "5" * 64
        save(self.session, {"run_id": pilot_run, "profile": "pilot", "condition": "closure",
                            "policy": "eta", "seed": 100, "workload_sha256": pilot_workload})
        pilot_start = self.root / "resources/pilot-start.json"
        create_attempt_start(
            pilot_start, self.root, run_id=pilot_run, profile="pilot", condition="closure",
            policy="eta", seed=100, source_commit=self.source_commit, config_path=self.config,
            session_root=self.session.parent, billing_path=self.billing,
            free_disk_bytes_before=1_000_000, started_at_utc="2026-10-18T00:00:00Z",
        )
        save(self.disk, {"run_id": pilot_run, "free_disk_bytes_before": 1_000_000,
                         "free_disk_bytes_after": 900_000})
        save(self.scene, {"run_id": pilot_run,
                          "timing": {"final_sim_time_s": 10.0, "wall_seconds": 5.0}})
        pilot_receipt = self.root / "resources/pilot-receipt.json"
        finalize_attempt(
            pilot_start, pilot_receipt, self.root,
            workers_finished_at_utc="2026-10-18T00:01:00Z",
            validation_finished_at_utc="2026-10-18T00:01:10Z", gpu_csv_path=self.gpu,
            scene_summary_path=self.scene, output_manifest_path=self.manifest,
            archive_path=self.archive, disk_path=self.disk,
            collection_failure_reason="pilot validator unavailable",
        )
        validated = validate_attempt_receipt(
            pilot_receipt, self.root, None, expected_source_commit=self.source_commit,
            expected_profile="pilot", expected_condition="closure", expected_policy="eta", expected_seed=100,
        )
        self.assertIsNone(validated["identity"]["trial_id"])
        self.assertEqual(pilot_workload, validated["bindings"]["workload_sha256"])


if __name__ == "__main__":
    unittest.main()
