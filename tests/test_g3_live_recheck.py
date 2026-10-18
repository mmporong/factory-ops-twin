from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timedelta, timezone
import hashlib
import json
from math import ceil
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.experiments import build_matrix, load_experiment_config
from factory_ops.g3_live_recheck import (
    EXPERIMENT_CONFIG_SHA256,
    PRODUCER_SOURCE_COMMIT,
    _parse_df_stdout,
    _validate_vm_inputs,
    recheck_campaign_launch,
)


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/factory-v3/g3-experiment.json"


def save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Usage:
    def __init__(self, free: int) -> None:
        self.total = free * 2
        self.used = free
        self.free = free


class LiveRecheckTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.evidence = self.root / "evidence"
        self.evidence.mkdir()
        self.store_roots = {"D": self.root / "store-D", "E": self.root / "store-E"}
        for path in self.store_roots.values():
            path.mkdir()
        self.identities = {"D": "sealed-D", "E": "sealed-E"}
        for store in ("D", "E"):
            receipt = self.evidence / f"coordinator-{store}.json"
            save(receipt, {"store_id": store, "device_identity": self.identities[store]})
        coordinator_refs = [
            {"path": f"coordinator-{store}.json",
             "sha256": digest(self.evidence / f"coordinator-{store}.json")}
            for store in ("D", "E")
        ]
        self.plan = self.evidence / "plan.json"
        self.capacity = {
            "vm": {"one_package_bytes": 100, "required_bytes": 1_000},
            "stores": {
                "D": {"deduplicated_extra_bytes": 50},
                "E": {"deduplicated_extra_bytes": 50},
            },
        }
        self.cost = {
            "per_run_closed_wall_time_s": 60.0,
            "planned_stopped_hours": 10.0,
            "stopped_storage_hourly_rate_usd": .04,
        }
        save(self.plan, {
            "source_commit": PRODUCER_SOURCE_COMMIT,
            "experiment_config_sha256": EXPERIMENT_CONFIG_SHA256,
            "reserve_fraction": .2, "absolute_reserve_bytes": 100,
            "coordinator_receipts": coordinator_refs,
            "derived_capacity": self.capacity, "cost_forecast": self.cost,
        })
        self.resource = {
            "evidence_root": str(self.evidence), "supervisor_path": "supervisor.json",
            "supervisor_sha256": "1" * 64, "requested_primary_runs": 60,
            "reserve_fraction": .2, "schema_version": 2,
            "sequential_plan_path": "plan.json", "sequential_plan_sha256": digest(self.plan),
        }
        self.calibration = self.root / "calibration.json"
        save(self.calibration, {
            "classification": "g3_actual_six_pilot_calibration",
            "status": "primary_execution_eligible", "execution_eligible": True,
            "resources": {"sequential_plan": {"plan_sha256": digest(self.plan)}},
            "fixture_boundary": "explicit_nonphysical_core_validator_double",
        })
        self.matrix = build_matrix(load_experiment_config(CONFIG))
        self.trial = self.root / "trial.json"
        save(self.trial, self.matrix[0])
        self.primary = self.root / "primary-inputs.json"
        self.write_primary()
        now = datetime.now(timezone.utc)
        self.not_before = now - timedelta(seconds=60)
        observed = now - timedelta(seconds=10)
        self.inventory = self.root / "inventory.json"
        self.supplement = self.root / "supplement.json"
        save(self.inventory, {"observed_at_utc": observed.isoformat().replace("+00:00", "Z")})
        save(self.supplement, {"sealed_at_utc": observed.isoformat().replace("+00:00", "Z")})
        self.billing = self.root / "billing.json"
        save(self.billing, {
            "observed_at": observed.isoformat().replace("+00:00", "Z"), "source": "fixture",
            "confirmation_method": "explicit nonphysical live probe double", "currency": "USD",
            "confirmed_hourly_rate_usd": 3.35, "budget_remaining_usd": 300.0,
            "project_budget_cap_usd": 400.0, "project_actual_billing_usd": 100.0,
            "project_budget_remaining_usd": 300.0,
        })
        self.previous: list[Path] = []
        self.free = {str(self.store_roots["D"]): 10**9, str(self.store_roots["E"]): 10**9}
        self.vm_free = 10**9
        self.vm_nonce = "1" * 32
        self.vm_binding = self.root / "vm-instance-binding.json"
        self.vm_probe = self.root / "vm-capacity-probe.json"
        save(self.vm_binding, {
            "schema_version": 1, "classification": "explicit_nonphysical_test_fixture",
            "observed_at_utc": observed.isoformat().replace("+00:00", "Z"),
            "source": "authenticated Brev CLI ls --json", "workspaces": [],
        })
        save(self.vm_probe, {
            "schema_version": 1, "classification": "explicit_nonphysical_test_fixture",
            "fixture_boundary": "validator patched; not production VM evidence",
        })

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_primary(self) -> None:
        save(self.primary, {
            "trial_path": str(self.trial), "calibration_path": str(self.calibration),
            "pilot_slots": {"fixture": {}}, "resource_input": self.resource,
            "expected_source_commit": PRODUCER_SOURCE_COMMIT,
        })

    def add_previous(self, package_bytes: int = 100) -> Path:
        ordinal = len(self.previous) + 1
        previous_sha = digest(self.previous[-1]) if self.previous else None
        prior_trial = self.root / f"previous-trial-{ordinal}.json"
        save(prior_trial, self.matrix[ordinal - 1])
        prior_inputs = self.root / f"previous-primary-inputs-{ordinal}.json"
        save(prior_inputs, {
            "trial_path": str(prior_trial), "calibration_path": str(self.calibration),
            "pilot_slots": {"fixture": {}}, "resource_input": self.resource,
            "expected_source_commit": PRODUCER_SOURCE_COMMIT,
        })
        preservation_paths = {}
        for store in ("D", "E"):
            preservation_path = self.root / f"previous-{ordinal}-{store}-preservation.json"
            save(preservation_path, {"store_id": store, "full_restore": True,
                                     "fixture_boundary": "explicit_nonphysical_preservation_double"})
            preservation_paths[store] = preservation_path
        path = self.root / f"previous-{ordinal}.json"
        save(path, {
            "schema_version": 1, "classification": "g3_closed_primary_preservation_receipt",
            "ordinal": ordinal, "trial_id": self.matrix[ordinal - 1]["trial_id"],
            "run_id": f"{ordinal:032x}", "source_commit": PRODUCER_SOURCE_COMMIT,
            "host_source_commit": "f" * 40, "primary_inputs_path": str(prior_inputs),
            "primary_inputs_sha256": digest(prior_inputs),
            "trial_outcome": "completed", "closed_at_utc": (
                self.not_before - timedelta(seconds=10 - ordinal)
            ).isoformat().replace("+00:00", "Z"),
            "previous_receipt_sha256": previous_sha,
            "package": {"sha256": "a" * 64, "bytes": package_bytes},
            "preservations": [
                {"store_id": store, "device_identity": self.identities[store],
                 "package_sha256": "a" * 64, "package_bytes": package_bytes,
                 "full_restore": True, "receipt_path": str(preservation_paths[store]),
                 "receipt_sha256": digest(preservation_paths[store])}
                for store in ("D", "E")
            ],
            "stopped_storage_hours": 0.5, "cleanup_authorized": False,
        })
        self.previous.append(path)
        save(self.trial, self.matrix[ordinal])
        self.write_primary()
        return path

    def invoke(self, name: str = "live") -> Path:
        def identity(**kwargs):
            return self.identities[str(kwargs["store_drive"])[0]]

        def usage(path):
            return Usage(self.free[str(Path(path))])

        def vm_probe(**kwargs):
            observed = datetime.now(timezone.utc) - timedelta(seconds=10)
            return self.vm_free, observed, observed, {"id": "fixture-vm", "name": "fixture-vm"}

        with patch("factory_ops.g3_live_recheck._host_source_commit", return_value="9" * 40), \
                patch("factory_ops.g3_live_recheck.validate_calibration"), \
                patch("factory_ops.g3_live_recheck.sealed_store_identity", side_effect=identity), \
                patch("factory_ops.g3_live_recheck._validate_vm_inputs", side_effect=vm_probe), \
                patch("factory_ops.g3_live_recheck.shutil.disk_usage", side_effect=usage):
            return recheck_campaign_launch(
                self.root / name, primary_inputs_path=self.primary,
                primary_inputs_sha256=digest(self.primary), experiment_config_path=CONFIG,
                previous_receipt_paths=self.previous, store_roots=self.store_roots,
                vm_capacity_probe_path=self.vm_probe,
                vm_capacity_probe_sha256=digest(self.vm_probe),
                vm_instance_binding_path=self.vm_binding,
                vm_instance_binding_sha256=digest(self.vm_binding),
                vm_probe_nonce=self.vm_nonce, storage_inventory_path=self.inventory,
                storage_supplement_manifest_path=self.supplement,
                storage_supplement_manifest_sha256=digest(self.supplement),
                fresh_billing_path=self.billing,
                not_before_utc=self.not_before.isoformat().replace("+00:00", "Z"),
                max_age_s=120.0, host_repository=self.root,
            )

    def test_eligible_receipt_preserves_manifest_and_never_launches(self) -> None:
        path = self.invoke()
        value = json.loads(path.read_text())
        self.assertEqual("eligible_for_host_coordinator", value["decision"])
        self.assertEqual(digest(self.primary), value["primary_inputs_sha256"])
        self.assertEqual((1, 60), (value["ordinal"], value["remaining_runs"]))
        self.assertFalse(value["launch_executed"])
        self.assertFalse(value["cleanup_authorized"])
        self.assertEqual(digest(path), json.loads((path.parent / "output-manifest.json").read_text())["files"][0]["sha256"])
        with self.assertRaises(FileExistsError):
            self.invoke()

    def test_space_budget_rate_nan_stale_future_and_device_fail_closed(self) -> None:
        cases = []
        cases.append(("store-space", lambda: self.free.__setitem__(str(self.store_roots["D"]), 1)))
        cases.append(("vm-space", lambda: setattr(self, "vm_free", 1)))
        for name, mutate in cases:
            original = deepcopy(self.free)
            original_vm = self.vm_free
            mutate()
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.invoke(name)
            self.free = original
            self.vm_free = original_vm

        billing = json.loads(self.billing.read_text())
        for name, update in (
            ("budget", {"budget_remaining_usd": 0.0}),
            ("rate", {"confirmed_hourly_rate_usd": 1000.0}),
            ("nan", {"confirmed_hourly_rate_usd": float("nan")}),
            ("stale", {"observed_at": (datetime.now(timezone.utc) - timedelta(hours=1)).isoformat()}),
            ("future", {"observed_at": (datetime.now(timezone.utc) + timedelta(hours=1)).isoformat()}),
        ):
            candidate = {**billing, **update}
            if name == "nan":
                self.billing.write_text(json.dumps(candidate, allow_nan=True), encoding="utf-8")
            else:
                save(self.billing, candidate)
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.invoke(name)
        save(self.billing, billing)
        inventory = json.loads(self.inventory.read_text())
        inventory["observed_at_utc"] = (
            datetime.now(timezone.utc) - timedelta(hours=1)
        ).isoformat()
        save(self.inventory, inventory)
        with self.assertRaisesRegex(ValueError, "stale"):
            self.invoke("stale-inventory")
        inventory["observed_at_utc"] = (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat()
        save(self.inventory, inventory)
        supplement = json.loads(self.supplement.read_text())
        supplement["sealed_at_utc"] = (datetime.now(timezone.utc) + timedelta(hours=1)).isoformat()
        save(self.supplement, supplement)
        with self.assertRaisesRegex(ValueError, "future"):
            self.invoke("future-supplement")
        supplement["sealed_at_utc"] = (datetime.now(timezone.utc) - timedelta(seconds=10)).isoformat()
        save(self.supplement, supplement)
        self.identities["E"] = "wrong-device"
        with self.assertRaisesRegex(ValueError, "identities differ"):
            self.invoke("device")

    def test_previous_chain_duplicate_ordinal_and_manifest_drift_are_rejected(self) -> None:
        first = self.add_previous()
        self.previous.append(first)
        with self.assertRaisesRegex(ValueError, "duplicated|binding"):
            self.invoke("duplicate")
        self.previous.pop()
        value = json.loads(first.read_text())
        value["ordinal"] = 2
        save(first, value)
        with self.assertRaisesRegex(ValueError, "chain binding"):
            self.invoke("ordinal")
        value["ordinal"] = 1
        save(first, value)
        save(self.trial, self.matrix[3])
        self.write_primary()
        with self.assertRaisesRegex(ValueError, "next canonical ordinal"):
            self.invoke("next-ordinal")
        save(self.trial, self.matrix[1])
        self.write_primary()
        with self.assertRaisesRegex(ValueError, "SHA"):
            with patch("factory_ops.g3_live_recheck._host_source_commit", return_value="9" * 40):
                recheck_campaign_launch(
                    self.root / "drift", primary_inputs_path=self.primary,
                    primary_inputs_sha256="0" * 64, experiment_config_path=CONFIG,
                    previous_receipt_paths=self.previous, store_roots=self.store_roots,
                    vm_capacity_probe_path=self.vm_probe,
                    vm_capacity_probe_sha256=digest(self.vm_probe),
                    vm_instance_binding_path=self.vm_binding,
                    vm_instance_binding_sha256=digest(self.vm_binding),
                    vm_probe_nonce=self.vm_nonce, storage_inventory_path=self.inventory,
                    storage_supplement_manifest_path=self.supplement,
                    storage_supplement_manifest_sha256=digest(self.supplement),
                    fresh_billing_path=self.billing, not_before_utc=self.not_before.isoformat(),
                    max_age_s=120.0, host_repository=self.root,
                )

    def test_larger_actual_package_raises_store_requirement(self) -> None:
        self.add_previous(package_bytes=1_000)
        remaining = 59
        expected = ceil(remaining * 1_000 * 1.2) + 50 + 100
        self.free[str(self.store_roots["D"])] = expected - 1
        with self.assertRaisesRegex(ValueError, "D free space"):
            self.invoke("raised-package")
        self.free[str(self.store_roots["D"])] = expected
        value = json.loads(self.invoke("raised-package-pass").read_text())
        self.assertEqual(expected, value["stores"]["D"]["required_bytes"])

    def test_nonphysical_vm_fixture_is_rejected_by_production_validator(self) -> None:
        now = datetime.now(timezone.utc)
        with self.assertRaisesRegex(ValueError, "binding differs"):
            _validate_vm_inputs(
                binding_path=self.vm_binding, binding_sha256=digest(self.vm_binding),
                probe_path=self.vm_probe, probe_sha256=digest(self.vm_probe),
                nonce=self.vm_nonce, host_source_commit="9" * 40, now=now,
                not_before=self.not_before, max_age_s=120.0,
            )

    def test_remote_df_parser_rederives_bytes_and_rejects_wrong_root(self) -> None:
        stdout = "Avail Mounted on\n123456 /\n"
        self.assertEqual(123456, _parse_df_stdout(
            stdout, "/home/ubuntu/workspace/factory-ops-g0",
        ))
        with self.assertRaisesRegex(ValueError, "root differs"):
            _parse_df_stdout(stdout, "/tmp/nonphysical-fixture")


if __name__ == "__main__":
    unittest.main()
