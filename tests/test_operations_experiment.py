"""Bounded operational experiment evidence tests."""
from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from factory_ops.operations_experiment import MAX_REPRESENTATIVE_SNAPSHOTS, run_case


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


if __name__ == "__main__":
    unittest.main()
