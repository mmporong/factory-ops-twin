import json
from pathlib import Path
import unittest

from factory_ops.dashboard import validate_snapshot
from factory_ops.line_operations_model import LineOperationsModel
from factory_ops.process_line_layout import build_line_bundle


ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"


class LineOperationsModelTests(unittest.TestCase):
    def test_one_lot_completes_five_kinematic_transports_and_returns_its_carrier(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-line12", initial_lots=1, run_id="line-one-lot",
        )
        for _second in range(1500):
            model.advance(1.0)
            line = model.line.snapshot()
            if line["returned"] == 1 and model.snapshot()["traffic"]["quiescent"]:
                break
        else:
            self.fail(f"line did not drain: {model.line.snapshot()}")

        state = validate_snapshot(model.snapshot())
        line = state["production_line"]
        self.assertEqual(1, line["completed_lots"])
        self.assertEqual(1, line["returned"])
        self.assertEqual(5, len(line["transports"]))
        self.assertTrue(all(task["state"] == "complete" for task in line["transports"]))
        self.assertEqual(
            [("S1", "CI"), ("CO", "AOI_IN"), ("AOI_OUT", "PACK_IN"),
             ("PACK_OUT", "OUT"), ("OUT", "S1")],
            [(task["origin"], task["destination"]) for task in line["transports"]],
        )
        self.assertEqual(5, sum(state["traffic"]["assignments"].values()))
        self.assertEqual(5, sum(state["traffic"]["deliveries"].values()))
        self.assertGreater(sum(state["traffic"]["travel_m"].values()), 0.0)
        self.assertGreater(state["traffic"]["observed_min_clearance_m"], 0.0)
        self.assertTrue(state["traffic"]["quiescent"])
        self.assertEqual({}, state["inventory"]["robot_loads"])
        carrier = next(row for row in state["inventory"]["totes"] if row["tote_id"] == "T001")
        self.assertEqual(("available", "S1", None),
                         (carrier["state"], carrier["location"], carrier["request_id"]))
        self.assertEqual(59, state["kpis"]["stock_units"])
        self.assertTrue(state["inventory"]["conservation"][0]["balanced"])
        self.assertEqual(["TEST", "AOI", "PACK"],
                         [cell["station_id"] for cell in state["equipment_cells"]])

    def test_generated_configs_are_reproducible_and_invalid_or_carrier_exhausted_inputs_fail(self):
        for count in (12, 16):
            with self.subTest(count=count):
                expected = build_line_bundle(CONFIGS, count)
                target = CONFIGS / f"factory-semiconductor-line{count}"
                for name, value in expected.items():
                    self.assertEqual(value, json.loads((target / name).read_text(encoding="utf-8")))
                routing = expected["g3-routing.json"]
                directed = {tuple(edge) for edge in routing["directed_edges"]}
                self.assertIn(("LOW_W_EN", "PACK_IN"), directed)
                self.assertIn(("PACK_IN", "LOW_W_ES"), directed)
                self.assertIn(("HIGH_E_ES", "PACK_OUT"), directed)
                self.assertIn(("PACK_OUT", "HIGH_E_EN"), directed)

        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-line16", initial_lots=0, run_id="line-inputs",
        )
        for parameters in ({"source": "BAD"}, {"source": "S1", "quantity": 0},
                           {"source": "S1", "extra": True}):
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                model.create_order(parameters)
        for _ in range(6):
            model.create_order({"source": "S1"})
        with self.assertRaisesRegex(ValueError, "no returned carrier"):
            model.create_order({"source": "S1"})
        self.assertEqual(6, len(model.line.lots))
        self.assertEqual(14, model.snapshot()["inventory"]["stock"][0]["available"])


if __name__ == "__main__":
    unittest.main()
