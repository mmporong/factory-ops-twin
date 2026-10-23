from pathlib import Path
import unittest

from factory_ops.dashboard import validate_snapshot
from factory_ops.operations_model import OperationsModel


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-dual-lane"


class OperationsModelTest(unittest.TestCase):
    def test_four_robots_complete_material_cycle_with_retained_clearance(self):
        model = OperationsModel(CONFIG, initial_orders=4, run_id="operations-cycle-test")
        for _ in range(2500):
            model.advance(1.0)
            if (all(o.state.value == "complete" for o in model.ledger.orders.values())
                    and len(model._return_tasks) == 4
                    and all(t["state"] == "complete" for t in model._return_tasks.values())
                    and model.snapshot()["traffic"]["quiescent"]):
                break
        state = validate_snapshot(model.snapshot())
        self.assertEqual(state["kpis"]["completed_orders"], 4)
        self.assertEqual(sum(t["state"] == "complete" for t in model._return_tasks.values()), 4)
        self.assertTrue(all(n > 0 for n in state["traffic"]["assignments"].values()))
        self.assertGreaterEqual(state["traffic"]["peak_moving_robots"], 2)
        self.assertGreater(state["traffic"]["observed_min_clearance_m"], 0.4)
        self.assertEqual(state["kpis"]["stock_units"], 56)
        self.assertEqual(state["execution_mode"], "operations")
        self.assertEqual(state["traffic"]["reservation_policy"], "bounded_segment_with_station_gates")

    def test_planned_closure_drains_accepted_work_and_reopens(self):
        model = OperationsModel(CONFIG, initial_orders=1, run_id="operations-closure-test")
        model.advance(1.0)
        result = model.set_aisle_closed({"closed": True})
        self.assertEqual(result["status"], "draining")
        self.assertFalse(model.closed)
        for _ in range(700):
            model.advance(1.0)
            if model.closed:
                break
        self.assertTrue(model.closed)
        self.assertTrue(any(e["type"] == "aisle_closure_applied" for e in model.snapshot()["events"]))
        model.set_aisle_closed({"closed": False})
        # New work may already have been accepted after closure applied.
        self.assertFalse(model.requested_closed)

    def test_idle_model_and_invalid_inputs(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="operations-idle-test")
        before = {r: dict(p) for r, p in model._poses.items()}
        model.advance(1.0)
        self.assertEqual(before, model._poses)
        self.assertEqual(model.set_aisle_closed({"closed": True})["status"], "applied")
        for invalid in ({"closed": 1}, {}, {"closed": True, "extra": 1}):
            with self.assertRaises(ValueError):
                model.set_aisle_closed(invalid)
        with self.assertRaises(ValueError):
            model.advance(True)


if __name__ == "__main__":
    unittest.main()
