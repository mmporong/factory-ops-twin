import json
from pathlib import Path
import unittest
from unittest.mock import patch

from factory_ops.dashboard import validate_snapshot
from factory_ops.line_operations_model import LineOperationsModel, _TripTask
from factory_ops.process_line_layout import build_line_bundle


ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"


class LineOperationsModelTests(unittest.TestCase):
    def test_pack_drop_protects_egress_and_stages_before_home_parking(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-campus16", initial_lots=0, run_id="line-staging",
        )
        task = _TripTask(
            "LOT-X:product_move:3", "LOT-X", "T001", "C_AOI_OUT", "C_PACK_IN", "product_move",
        )
        robot = "amr-16"
        pickup = model.book.plan(model._locations[robot], task.pickup_station)
        model._trip(
            robot, task, model._locations[robot], task.pickup_station,
            "pickup", kind="process_line",
        )
        window = model._next_route_window(pickup.points[0], pickup.points[1:], pickup.nodes[1:])
        self.assertEqual(window["tiles"], model._trips[robot]["admission_tiles"])

        drop = model.book.plan(task.pickup_station, task.drop_station)
        staging = model.book.plan(task.drop_station, "C_PACK_WAIT")
        drop_window = model._window_for_trip(
            drop.points[-2], drop.points[-1:], drop.nodes[-1:],
            {"kind": "process_line", "phase": "drop", "goal": task.drop_station, "task": task},
        )
        self.assertTrue(drop_window["retain_at_handover"])
        self.assertTrue(model._tiles(staging.points).issubset(drop_window["tiles"]))

        model._poses[robot].update(
            x_m=model.book.nodes["C_PACK_IN"][0], y_m=model.book.nodes["C_PACK_IN"][1],
        )
        model._trips[robot].update(phase="drop", start="C_AOI_OUT", goal="C_PACK_IN")
        model._station_owners["C_PACK_IN"] = robot
        model._jobs[robot] = ("C_AOI_OUT", "C_PACK_IN")
        with patch.object(model.line, "drop"), patch.object(model.inventory, "product_drop"):
            model._arrive_trip(robot, model._trips[robot])

        staged = model._trips[robot]
        self.assertEqual(("parking", "staging", "C_PACK_WAIT"),
                         (staged["kind"], staged["purpose"], staged["goal"]))
        self.assertEqual(robot, model._station_owners["C_PACK_IN"])
        row = next(row for row in model.snapshot()["robots"] if row["id"] == robot)
        self.assertEqual(("staging", "post_drop_staging"),
                         (row["task_type"], row["movement_purpose"]))

    def test_only_unstarted_home_parking_is_preemptible_for_business_work(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-line16", initial_lots=0, run_id="line-park-preempt",
        )
        robot = "amr-16"
        start = model._locations[robot]
        model._trip(robot, {}, start, "P1", "parking", kind="parking", purpose="home")
        self.assertTrue(model._can_withdraw_queued_parking(robot))
        self.assertTrue(model._withdraw_queued_parking(robot))
        self.assertNotIn(robot, model._trips)

        model._trip(robot, {}, start, "P1", "parking", kind="parking", purpose="staging")
        self.assertFalse(model._can_withdraw_queued_parking(robot))
        model._trips[robot]["purpose"] = "home"
        model._chunks[robot] = {"already_started": True}
        self.assertFalse(model._can_withdraw_queued_parking(robot))
    def test_conflicting_full_trip_is_not_assigned_before_admission(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-line16", initial_lots=1, run_id="line-dispatch-gate",
        )
        with patch.object(model, "_prospective_trip_blockers", return_value=("amr-2",)) as gate:
            model._dispatch_extra_jobs(False)
        self.assertTrue(gate.called)
        task = model.line.ready_transports[0]
        self.assertEqual("ready", task.state)
        self.assertIsNone(task.robot_id)
        self.assertFalse(model._trips)

        with patch.object(model, "_prospective_trip_blockers", return_value=()):
            model._dispatch_extra_jobs(False)
        assigned = next(task for task in model.line.snapshot()["transports"] if task["state"] == "assigned")
        self.assertIsNotNone(assigned["robot_id"])
        self.assertEqual(1, len(model._trips))

    def test_network_dispatch_drains_downstream_wip_and_uses_least_assigned_tie_break(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-campus16", initial_lots=0, run_id="campus-priority",
        )
        tasks = [
            type("Task", (), {"kind": "process_input", "destination": "A_TEST_IN",
                               "created_sim_time_s": 0.0, "task_id": "input"})(),
            type("Task", (), {"kind": "product_move", "destination": "B_AOI_IN",
                               "created_sim_time_s": 1.0, "task_id": "aoi"})(),
            type("Task", (), {"kind": "product_move", "destination": "D_OUT",
                               "created_sim_time_s": 2.0, "task_id": "ship"})(),
            type("Task", (), {"kind": "empty_return", "destination": "A_S1",
                               "created_sim_time_s": 3.0, "task_id": "return"})(),
        ]
        ordered = [task.task_id for task in sorted(tasks, key=model._line_task_priority)]
        self.assertEqual(["return", "ship", "aoi", "input"], ordered)
        self.assertEqual("legacy_fifo_eta", LineOperationsModel(
            CONFIGS / "factory-semiconductor-line16", initial_lots=0, run_id="legacy-priority",
        )._line_dispatch_mode)

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
