from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.experiments import canonical_hash, load_experiment_config
from factory_ops.g3_campaign import (
    EXPERIMENT_CONFIG_SHA256,
    PRODUCER_SOURCE_COMMIT,
    _host_source_commit,
    freeze_primary_campaign,
    materialize_actual_six,
)
from tests.test_g3_calibration import make_raw_package


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/factory-v3/g3-experiment.json"


def save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class G3CampaignTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.evidence = self.root / "evidence"
        self.evidence.mkdir()
        if os.name != "posix":
            loader = patch(
                "factory_ops.g3_campaign.load_raw_envelope",
                side_effect=lambda path: json.loads(Path(path).read_text(encoding="utf-8")),
            )
            loader.start()
            self.addCleanup(loader.stop)
        self.host_repository = self.root / "host-source"
        for relative in (
            "factory_ops/g3_campaign.py", "scripts/brev/g3_campaign.py", "tests/test_g3_campaign.py",
        ):
            destination = self.host_repository / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, destination)
        subprocess.run(("git", "init", "-q"), cwd=self.host_repository, check=True)
        subprocess.run(("git", "config", "user.email", "fixture@example.invalid"),
                       cwd=self.host_repository, check=True)
        subprocess.run(("git", "config", "user.name", "Fixture"),
                       cwd=self.host_repository, check=True)
        subprocess.run(("git", "add", "factory_ops/g3_campaign.py", "scripts/brev/g3_campaign.py",
                        "tests/test_g3_campaign.py"), cwd=self.host_repository, check=True)
        subprocess.run(("git", "commit", "-qm", "fixture host source"),
                       cwd=self.host_repository, check=True)
        self.host_source = subprocess.check_output(
            ("git", "rev-parse", "HEAD"), cwd=self.host_repository, text=True,
        ).strip()
        host_boundary = patch("factory_ops.g3_campaign._host_source_commit", return_value=self.host_source)
        host_boundary.start()
        self.addCleanup(host_boundary.stop)
        self.run_ids = [f"{index + 1:032x}" for index in range(6)]
        self.runs = []
        slots = [
            "normal:fifo", "normal:eta", "closure:fifo", "closure:eta", "burst:fifo", "burst:eta",
        ]
        for index, (slot, run_id) in enumerate(zip(slots, self.run_ids, strict=True)):
            condition, policy = slot.split(":")
            run_root = self.evidence / "runs" / run_id
            session_root = self.evidence / "sessions" / run_id
            save(run_root / "g3-evidence.json", {
                "run_id": run_id, "profile": "pilot", "condition": condition, "policy": policy,
                "seed": 100, "experiment_config_sha256": EXPERIMENT_CONFIG_SHA256,
                "source": {"commit": PRODUCER_SOURCE_COMMIT},
            })
            save(run_root / "mission-result.json", {"run_id": run_id})
            save(session_root / "session.json", {
                "profile": "pilot", "condition": condition, "policy": policy, "seed": 100,
            })
            envelope_path, envelope, archive_payload, manifest_payload = make_raw_package(
                self.evidence, run_id=run_id, run_label=f"g3-p-{condition}-{policy}-{index + 1:03d}",
                source_commit=PRODUCER_SOURCE_COMMIT, condition=condition, policy=policy, index=index,
            )
            resource_path = self.evidence / "raw-resources" / f"{run_id}.json"
            references = {}
            for name in (
                "attempt_receipt", "gpu_csv", "gpu_raw", "gpu_sampler_status",
                "scene_summary", "output_manifest", "archive", "disk",
            ):
                path = self.evidence / "resources" / run_id / name
                path.parent.mkdir(parents=True, exist_ok=True)
                if name == "archive":
                    path.write_bytes(archive_payload)
                elif name == "output_manifest":
                    path.write_bytes(manifest_payload)
                else:
                    path.write_text(f"{run_id}:{name}\n", encoding="utf-8")
                references[name] = {
                    "path": path.relative_to(self.evidence).as_posix(), "sha256": digest(path),
                }
            save(resource_path, {
                "run_id": run_id, "started_at_utc": "2026-10-18T00:00:00Z",
                "finished_at_utc": "2026-10-18T00:01:00Z", **references,
            })
            closure_path = self.evidence / "closures" / f"{run_id}.json"
            save(closure_path, {
                "schema_version": 1, "classification": "g3_source_support_closure",
                "source_commit": PRODUCER_SOURCE_COMMIT,
                "experiment_config_sha256": EXPERIMENT_CONFIG_SHA256,
                "actual_six_run_ids": sorted(self.run_ids),
                "raw_envelope_sha256": digest(envelope_path),
                "source_bundle_sha256": envelope["source_bundle_sha256"],
                "support_manifest_sha256": envelope["support_manifest_sha256"],
                "per_trial_support_manifest_sha256": envelope["per_trial_support_manifest_sha256"],
                "shared_six_pilot_support_pack_sha256": envelope["shared_six_pilot_support_pack_sha256"],
            })
            self.runs.append({
                "slot": slot, "run_root": str(run_root), "session_root": str(session_root),
                "resource_path": str(resource_path), "closure_path": str(closure_path),
                "raw_envelope_path": str(envelope_path),
            })
        self.billing = self.evidence / "actual-extended-billing.json"
        save(self.billing, {
            "observed_at": "2026-10-18T05:48:00Z", "source": "Brev billing UI capture",
            "confirmation_method": "fresh supervisor observation", "currency": "USD",
            "confirmed_hourly_rate_usd": 3.35, "budget_remaining_usd": 279.78,
            "project_budget_cap_usd": 400.0, "project_actual_billing_usd": 100.22,
            "project_budget_remaining_usd": 299.78, "account_total_billed_usd": 120.22,
        })

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def materialize(self, name: str = "campaign") -> Path:
        return materialize_actual_six(
            self.evidence / name, evidence_root=self.evidence, runs=self.runs,
            actual_billing_path=self.billing, host_repository=self.host_repository,
        )

    def test_actual_six_materializes_exact_slots_rows_and_billing_provenance(self) -> None:
        output = self.materialize()
        slots = json.loads((output / "pilot-slots.json").read_text(encoding="utf-8"))
        supervisor = json.loads((output / "supervisor.json").read_text(encoding="utf-8"))
        manifest = json.loads((output / "campaign-inputs.json").read_text(encoding="utf-8"))
        self.assertEqual(6, len(slots))
        self.assertEqual(sorted(self.run_ids), [row["run_id"] for row in supervisor["runs"]])
        self.assertEqual(digest(self.billing), manifest["actual_billing"]["sha256"])
        self.assertEqual(PRODUCER_SOURCE_COMMIT, manifest["producer_source_commit"])
        self.assertEqual(self.host_source, manifest["host_source_commit"])
        self.assertFalse(manifest["actual_data_gate_succeeded"])
        self.assertFalse(manifest["launch_authorized"])
        planning = json.loads((output / "planning-billing.json").read_text(encoding="utf-8"))
        self.assertNotIn("account_total_billed_usd", planning)
        self.assertEqual(100.22, planning["project_actual_billing_usd"])
        with self.assertRaises(FileExistsError):
            self.materialize()

    def test_actual_six_rejects_missing_duplicate_or_foreign_bindings_without_publication(self) -> None:
        cases = []
        cases.append(("missing", self.runs[:-1], {}))
        duplicate = deepcopy(self.runs)
        duplicate[-1]["slot"] = duplicate[0]["slot"]
        cases.append(("duplicate", duplicate, {}))
        cases.append(("source", self.runs, {"source_commit": "f" * 40}))
        cases.append(("config", self.runs, {"experiment_config_sha256": "e" * 64}))
        for name, runs, extra in cases:
            output = self.evidence / f"rejected-{name}"
            with self.subTest(name=name), self.assertRaises(ValueError):
                materialize_actual_six(
                    output, evidence_root=self.evidence, runs=runs,
                    actual_billing_path=self.billing, host_repository=self.host_repository, **extra,
                )
            self.assertFalse(output.exists())

    def test_actual_six_rejects_wrong_closure_set_and_billing_arithmetic(self) -> None:
        closure = Path(self.runs[0]["closure_path"])
        value = json.loads(closure.read_text(encoding="utf-8"))
        value["actual_six_run_ids"] = value["actual_six_run_ids"][:-1]
        save(closure, value)
        with self.assertRaisesRegex(ValueError, "exact six"):
            self.materialize("bad-closure")
        value["actual_six_run_ids"] = sorted(self.run_ids)
        save(closure, value)
        billing = json.loads(self.billing.read_text(encoding="utf-8"))
        billing["project_budget_remaining_usd"] = 1.0
        save(self.billing, billing)
        with self.assertRaisesRegex(ValueError, "400 USD"):
            self.materialize("bad-billing")

    def test_actual_six_rejects_reused_closure_or_raw_envelope(self) -> None:
        duplicate_closure = deepcopy(self.runs)
        duplicate_closure[-1]["closure_path"] = duplicate_closure[0]["closure_path"]
        with self.assertRaises(ValueError):
            materialize_actual_six(
                self.evidence / "duplicate-closure", evidence_root=self.evidence,
                runs=duplicate_closure, actual_billing_path=self.billing,
                host_repository=self.host_repository,
            )
        duplicate_envelope = deepcopy(self.runs)
        duplicate_envelope[-1]["raw_envelope_path"] = duplicate_envelope[0]["raw_envelope_path"]
        with self.assertRaises(ValueError):
            materialize_actual_six(
                self.evidence / "duplicate-envelope", evidence_root=self.evidence,
                runs=duplicate_envelope, actual_billing_path=self.billing,
                host_repository=self.host_repository,
            )

    def test_host_source_requires_clean_committed_execution_bytes(self) -> None:
        fake_module = self.host_repository / "factory_ops/g3_campaign.py"
        with patch("factory_ops.g3_campaign.__file__", str(fake_module)):
            self.assertEqual(self.host_source, _host_source_commit(self.host_repository))
            host_file = self.host_repository / "factory_ops/g3_campaign.py"
            host_file.write_bytes(host_file.read_bytes() + b"\n")
            with self.assertRaisesRegex(ValueError, "clean"):
                _host_source_commit(self.host_repository)
            subprocess.run(("git", "checkout", "--", "factory_ops/g3_campaign.py"),
                           cwd=self.host_repository, check=True)
        with self.assertRaisesRegex(ValueError, "active execution checkout"):
            _host_source_commit(self.host_repository)

    def primary_inputs(self) -> tuple[Path, Path]:
        actual = self.materialize("actual")
        resource = self.root / "resource-input.json"
        save(resource, {
            "evidence_root": str(self.evidence), "supervisor_path": "actual/supervisor.json",
            "supervisor_sha256": digest(actual / "supervisor.json"),
            "requested_primary_runs": 60, "reserve_fraction": .2, "schema_version": 2,
            "sequential_plan_path": "sequential-capacity-plan.json",
            "sequential_plan_sha256": "a" * 64,
        })
        return actual / "pilot-slots.json", resource

    @staticmethod
    def eligible_calibration() -> dict:
        return {
            "schema_version": 1, "classification": "g3_actual_six_pilot_calibration",
            "status": "primary_execution_eligible", "execution_eligible": True,
            "calibration_sha256": "b" * 64,
            "fixture_boundary": "explicit_nonphysical_test_double_never_actual_evidence",
        }

    def test_primary_freeze_reuses_core_gate_and_writes_60_exact_manifests(self) -> None:
        pilots, resource = self.primary_inputs()
        calibration = self.eligible_calibration()
        with patch("factory_ops.g3_campaign.build_calibration", return_value=calibration) as build, \
                patch("factory_ops.g3_campaign.validate_calibration") as validate:
            output = freeze_primary_campaign(
                self.root / "primary", pilot_slots_path=pilots, resource_input_path=resource,
                experiment_config_path=CONFIG, host_repository=self.host_repository,
            )
        build.assert_called_once()
        validate.assert_called_once()
        manifest = json.loads((output / "campaign-manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(60, manifest["trial_count"])
        self.assertEqual(121, len(manifest["files"]))
        primaries = sorted((output / "primary-inputs").glob("*.json"))
        trials = sorted((output / "trials").glob("*.json"))
        self.assertEqual((60, 60), (len(primaries), len(trials)))
        first = json.loads(primaries[0].read_text(encoding="utf-8"))
        self.assertEqual({
            "trial_path", "calibration_path", "pilot_slots", "resource_input",
            "expected_source_commit",
        }, set(first))
        self.assertTrue(Path(first["trial_path"]).is_absolute())
        self.assertEqual(PRODUCER_SOURCE_COMMIT, first["expected_source_commit"])
        self.assertFalse(manifest["actual_data_gate_succeeded"])

    def test_primary_freeze_fails_closed_for_budget_or_capacity_ineligibility(self) -> None:
        pilots, resource = self.primary_inputs()
        for reason in ("budget", "capacity"):
            rejected = self.eligible_calibration()
            rejected.update({"status": "resource_gate_failed", "execution_eligible": False,
                             "failure_fixture": reason})
            output = self.root / f"primary-{reason}"
            with self.subTest(reason=reason), \
                    patch("factory_ops.g3_campaign.build_calibration", return_value=rejected), \
                    patch("factory_ops.g3_campaign.validate_calibration"), \
                    self.assertRaisesRegex(ValueError, "not primary execution eligible"):
                freeze_primary_campaign(
                    output, pilot_slots_path=pilots, resource_input_path=resource,
                    experiment_config_path=CONFIG, host_repository=self.host_repository,
                )
            self.assertFalse(output.exists())

    def test_primary_freeze_rejects_non_v2_resource_before_core_gate(self) -> None:
        pilots, resource = self.primary_inputs()
        value = json.loads(resource.read_text(encoding="utf-8"))
        value.pop("sequential_plan_sha256")
        save(resource, value)
        with patch("factory_ops.g3_campaign.build_calibration") as build, self.assertRaisesRegex(
            ValueError, "exact v2"
        ):
            freeze_primary_campaign(
                self.root / "primary", pilot_slots_path=pilots, resource_input_path=resource,
                experiment_config_path=CONFIG, host_repository=self.host_repository,
            )
        build.assert_not_called()

    def test_fixed_config_hash_matches_repository_config(self) -> None:
        self.assertEqual(EXPERIMENT_CONFIG_SHA256, canonical_hash(load_experiment_config(CONFIG)))


if __name__ == "__main__":
    unittest.main()
