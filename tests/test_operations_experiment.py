"""Bounded operational experiment evidence tests."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.operations_experiment import (
    MAX_REPRESENTATIVE_SNAPSHOTS,
    _config_pins,
    _producer_pins,
    main,
    run_case,
)


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"


class _FakeOperationsModel:
    def __init__(self, config: Path, *, policy: str, initial_orders: int, run_id: str | None) -> None:
        assert initial_orders == 0
        self.policy = policy
        self.run_id = run_id
        self.sim_time = 0.0
        self.available = True
        self.closed = False
        self.orders: list[dict[str, object]] = []
        self.events: list[dict[str, object]] = []

    def create_order(self, parameters: dict[str, object]) -> str:
        order_id = f"ORD-{len(self.orders) + 1:04d}"
        self.orders.append({"id": order_id, "source": parameters["source"], "created": self.sim_time,
                            "complete": False, "robot": f"amr-{len(self.orders) % 4 + 1}"})
        return order_id

    def set_equipment_availability(self, parameters: dict[str, object]) -> None:
        self.available = bool(parameters["available"])
        self.events.append({"id": f"equipment-{len(self.events)}", "type": "equipment_state",
                            "sim_time_s": self.sim_time, "available": self.available})

    def set_aisle_closed(self, parameters: dict[str, object]) -> None:
        self.closed = bool(parameters["closed"])

    def advance(self, dt: float) -> None:
        self.sim_time += dt
        if self.available:
            for order in self.orders:
                if not order["complete"] and self.sim_time - float(order["created"]) >= 10.0:
                    order["complete"] = True

    def snapshot(self) -> dict[str, object]:
        tasks = [{"id": f"{o['id']}:out", "order_id": o["id"], "type": "outbound",
                  "state": "complete" if o["complete"] else "ready", "robot_id": o["robot"]}
                 for o in self.orders]
        complete = sum(bool(o["complete"]) for o in self.orders)
        return {
            "run_id": self.run_id,
            "sim_time_s": self.sim_time,
            "execution_mode": "operations",
            "dispatch_policy": self.policy,
            "robots": [{"id": f"amr-{i}", "status": "idle", "pose": {}} for i in range(1, 5)],
            "tasks": tasks,
            "events": list(self.events),
            "equipment": {"available": self.available, "wip_orders": len(self.orders) - complete},
            "traffic": {"wait_s": {"amr-1": 3.0, "amr-2": 2.0} if self.closed else {}},
            "kpis": {"completed_orders": complete},
        }


class _NeverCompletes(_FakeOperationsModel):
    def advance(self, dt: float) -> None:
        self.sim_time += dt


class _AdvanceFails(_FakeOperationsModel):
    def advance(self, dt: float) -> None:
        if self.sim_time >= 2:
            raise RuntimeError("authored failure")
        super().advance(dt)


class _MutatesConfig(_FakeOperationsModel):
    def __init__(self, config: Path, **kwargs: object) -> None:
        super().__init__(config, **kwargs)
        self.config = config
        self.mutated = False

    def advance(self, dt: float) -> None:
        if not self.mutated:
            (self.config / "g2.json").write_text('{"changed":true}\n', encoding="utf-8")
            self.mutated = True
        super().advance(dt)


def _write_minimal_config(root: Path) -> Path:
    config = root / "config"
    config.mkdir()
    for index, name in enumerate(("layout.json", "g2.json", "g3-routing.json", "transport-profile.json")):
        (config / name).write_text(json.dumps({"fixture": index}) + "\n", encoding="utf-8")
    return config


class OperationsExperimentTest(unittest.TestCase):
    def test_rejects_invalid_inputs_before_running(self) -> None:
        for field, value in (("scenario", "fire"), ("policy", "random")):
            kwargs = {"scenario": "normal", "policy": "fifo", field: value}
            with self.subTest(field=field), self.assertRaises(ValueError):
                run_case(CONFIG, **kwargs, horizon_s=1, model_factory=_FakeOperationsModel)
        with self.assertRaises(ValueError):
            run_case(CONFIG, scenario="normal", policy="fifo", dt_s=1.1,
                     model_factory=_FakeOperationsModel)
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(ValueError):
            run_case(Path(directory) / "absent", scenario="normal", policy="fifo",
                     model_factory=_FakeOperationsModel)

    def test_scenarios_use_identical_twelve_order_demand_and_bounded_snapshots(self) -> None:
        normal = run_case(CONFIG, scenario="normal", policy="fifo", horizon_s=341, dt_s=1,
                          model_factory=_FakeOperationsModel)
        outage = run_case(CONFIG, scenario="tester_down", policy="fifo", horizon_s=341, dt_s=1,
                          model_factory=_FakeOperationsModel)
        self.assertEqual(normal["workload"]["sources"], outage["workload"]["sources"])
        self.assertEqual(12, normal["metrics"]["released_orders"])
        self.assertEqual(12, outage["metrics"]["released_orders"])
        self.assertLessEqual(len(normal["representative_snapshots"]), MAX_REPRESENTATIVE_SNAPSHOTS)
        self.assertEqual("operations", normal["execution_mode"])
        self.assertEqual("fifo", normal["dispatch_policy"])

    def test_incomplete_demand_is_explicit_and_tardiness_is_not_erased(self) -> None:
        result = run_case(CONFIG, scenario="aisle_closed", policy="eta", horizon_s=400, dt_s=1,
                          model_factory=_NeverCompletes)
        metrics = result["metrics"]
        self.assertEqual("incomplete", result["status"])
        self.assertFalse(metrics["finished_all_demand"])
        self.assertEqual(0, metrics["completed_cycles"])
        self.assertEqual(12, metrics["incomplete_orders"])
        self.assertGreater(metrics["incomplete_tardiness_lower_bound_s"], 0)
        self.assertEqual(5.0, metrics["traffic_wait_s"])
        self.assertEqual("unavailable_assumed_false", metrics["fleet_quiescence_source"])

    def test_short_horizon_cannot_complete_the_unreleased_workload(self) -> None:
        first = run_case(CONFIG, scenario="normal", policy="fifo", horizon_s=20, dt_s=1,
                         model_factory=_FakeOperationsModel)
        second = run_case(CONFIG, scenario="normal", policy="fifo", horizon_s=20, dt_s=1,
                          model_factory=_FakeOperationsModel)
        self.assertEqual(1, first["metrics"]["released_orders"])
        self.assertEqual(11, first["metrics"]["unreleased_demand"])
        self.assertFalse(first["metrics"]["finished_all_demand"])
        self.assertEqual(first["case_identity"], second["case_identity"])
        self.assertNotEqual(first["run_id"], second["run_id"])
        self.assertEqual("identity_only_no_stochastic_inputs", first["seed_role"])

    def test_runtime_failure_preserves_partial_metrics_and_source_pins(self) -> None:
        result = run_case(CONFIG, scenario="normal", policy="eta", horizon_s=20, dt_s=1,
                          model_factory=_AdvanceFails)
        self.assertEqual("failed", result["status"])
        self.assertEqual("RuntimeError", result["error"]["type"])
        self.assertEqual(1, result["metrics"]["released_orders"])
        self.assertFalse(result["metrics"]["experiment_complete"])
        provenance = result["provenance"]
        self.assertTrue(provenance["source_stable_during_run"])
        self.assertIn("g2.json", provenance["sources_before_sha256"]["config"])
        self.assertIn("factory_ops/operations/ledger.py", provenance["sources_before_sha256"]["producer"])

    def test_source_change_during_run_marks_case_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = _write_minimal_config(Path(directory))
            result = run_case(config, scenario="normal", policy="fifo", horizon_s=2, dt_s=1,
                              model_factory=_MutatesConfig)
        self.assertEqual("invalid", result["status"])
        self.assertFalse(result["provenance"]["source_stable_during_run"])
        self.assertNotEqual(result["provenance"]["sources_before_sha256"],
                            result["provenance"]["sources_after_sha256"])

    def test_snapshot_observer_receives_isolated_copies_only_after_advance(self) -> None:
        observed: list[dict[str, object]] = []

        def capture(snapshot: object) -> None:
            assert isinstance(snapshot, dict)
            observed.append(snapshot)
            snapshot["sim_time_s"] = -999
            snapshot["robots"] = []

        result = run_case(CONFIG, scenario="normal", policy="fifo", horizon_s=2, dt_s=1,
                          model_factory=_FakeOperationsModel, snapshot_observer=capture)
        self.assertGreaterEqual(len(observed), 2)
        self.assertTrue(all(frame["sim_time_s"] == -999 for frame in observed))
        self.assertGreater(result["final_snapshot"]["sim_time_s"], 0)
        self.assertEqual(4, len(result["final_snapshot"]["robots"]))
        self.assertEqual(len(observed), result["snapshot_observer"]["calls_completed"])
        self.assertEqual("deepcopy", result["snapshot_observer"]["copy_mode"])

    def test_snapshot_observer_error_uses_partial_failure_report(self) -> None:
        def fail(_: object) -> None:
            raise RuntimeError("capture failed")

        result = run_case(CONFIG, scenario="normal", policy="eta", horizon_s=10, dt_s=1,
                          model_factory=_FakeOperationsModel, snapshot_observer=fail)
        self.assertEqual("failed", result["status"])
        self.assertEqual("capture failed", result["error"]["message"])
        self.assertEqual(0, result["snapshot_observer"]["calls_completed"])
        self.assertFalse(result["metrics"]["experiment_complete"])

    def test_cli_freezes_explicit_sources_and_records_verified_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = _write_minimal_config(root)
            output = root / "attempt" / "matrix.json"

            def fake_case(selected: Path, **_: object) -> dict[str, object]:
                return {
                    "status": "incomplete",
                    "provenance": {"sources_before_sha256": {
                        "config": _config_pins(selected), "producer": _producer_pins(),
                    }},
                }

            with patch("factory_ops.operations_experiment.run_case", side_effect=fake_case):
                self.assertEqual(0, main(["--config", str(config), "--output", str(output),
                                          "--horizon", "1", "--dt", "1"]))

            document = json.loads(output.read_text(encoding="utf-8"))
            frozen = root / "attempt" / "frozen-source"
            manifest = json.loads((frozen / "manifest.json").read_text(encoding="utf-8"))
            self.assertTrue(document["provenance"]["frozen_source"]["binding_matches_case_start"])
            self.assertEqual(document["frozen_source"]["file_count"], len(manifest["files"]))
            self.assertEqual((config / "g2.json").read_bytes(), (frozen / "config" / "g2.json").read_bytes())
            self.assertEqual(
                (Path(__file__).resolve().parents[1] / "factory_ops" / "operations_experiment.py").read_bytes(),
                (frozen / "factory_ops" / "operations_experiment.py").read_bytes(),
            )
            self.assertFalse((frozen / ".env").exists())

    def test_cli_rejects_existing_output_before_execution(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = _write_minimal_config(root)
            output = root / "matrix.json"
            output.write_text("preserve\n", encoding="utf-8")
            with patch("factory_ops.operations_experiment.run_case") as execute:
                with self.assertRaises(FileExistsError):
                    main(["--config", str(config), "--output", str(output),
                          "--horizon", "1", "--dt", "1"])
                execute.assert_not_called()
            self.assertEqual("preserve\n", output.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
