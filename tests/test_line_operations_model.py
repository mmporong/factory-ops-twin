import json
from pathlib import Path
import unittest
from unittest.mock import PropertyMock, patch

from factory_ops.dashboard import validate_snapshot
from factory_ops.line_operations_model import LineOperationsModel, _TripTask
from factory_ops.process_line import TransportRequest
from factory_ops.process_line_layout import build_line_bundle


ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"


class LineOperationsModelTests(unittest.TestCase):
    @staticmethod
    def _keep_only_available(model, *robot_ids):
        selected = set(robot_ids)
        for robot_id, state in model.ledger.robots.items():
            state.available = robot_id in selected

    def test_departure_funding_includes_ready_line_work_and_observed_traffic_wait(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-campus16", initial_lots=1,
            run_id="line-charge-departure-funding",
        )
        charging = model.operations_charging
        assert charging is not None
        task = model.line.ready_transports[0]

        with patch.object(
            charging, "_assignment_required_wh", wraps=charging._assignment_required_wh,
        ) as calculate:
            base = model._charging_departure_requirement("amr-1")
            self.assertIsNotNone(base)
            self.assertEqual(task.task_id, base["task_id"])
            first_count = calculate.call_count

            charging.energy.batteries["amr-1"].energy_wh -= 1.0
            self.assertEqual(base, model._charging_departure_requirement("amr-1"))
            self.assertEqual(first_count, calculate.call_count)

            charging.traffic_wait_budget_s = 9588.8
            congested = model._charging_departure_requirement("amr-1")
            self.assertEqual(first_count, calculate.call_count)
        expected_extra = charging.energy.profile["idle_power_w"] * 9588.8 / 3600.0
        self.assertEqual(task.task_id, congested["task_id"])
        self.assertAlmostEqual(base["required_wh"] + expected_extra, congested["required_wh"])

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

    def test_shipping_drop_chains_ready_return_without_vacating_the_station(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-campus16", initial_lots=0,
            run_id="line-same-station-handoff",
        )
        robot = "amr-16"
        lot_id = model.create_order({"source": "D_S1", "quantity": 1})
        line = model.line.lines["L4"]
        lot = line.lots[lot_id]
        carrier = lot.carrier_id
        for task in line.tasks.values():
            task.state = "complete"
        line.cells["PACK"].output_lots.append(lot_id)
        shipping = line._new_task(lot, "product_move", "A_PACK_OUT", "A_OUT")
        assigned_shipping = line.assign(shipping.task_id, robot)
        line.pickup(model.ledger.run_id, shipping.task_id, robot, "shipping-pickup", model.sim_time)
        model.inventory.source_pickup(lot_id, robot)
        model.inventory.product_drop(lot_id, carrier, robot, "A_PACK_OUT")
        model.inventory.product_pickup(lot_id, carrier, robot, "A_PACK_OUT")
        delivered = _TripTask.from_request(assigned_shipping)
        model._trip(
            robot, delivered, delivered.pickup_station, delivered.drop_station,
            "drop", kind="process_line",
        )
        model._poses[robot].update(
            x_m=model.book.nodes["A_OUT"][0], y_m=model.book.nodes["A_OUT"][1],
        )
        old_window = frozenset(model._trips[robot]["admission_tiles"])
        model.bridges.admit(
            robot,
            model._trips[robot]["admission_nodes"],
            route_tiles=old_window,
            handover_protected=True,
        )
        old_resources = model.bridges.held(robot)
        model._station_owners.update({"A_PACK_OUT": robot, "A_OUT": robot})
        model._jobs[robot] = ("A_PACK_OUT", "A_OUT")
        charging = model.operations_charging
        assert charging is not None

        with (
            patch.object(charging, "can_assign", return_value=True),
            patch.object(model, "_admission_observations",
                         return_value={robot: model._footprint(robot)}),
            patch.object(
                model, "_safe_exit_lookahead_blockers",
                wraps=model._safe_exit_lookahead_blockers,
            ) as lookahead,
        ):
            model._arrive_trip(robot, model._trips[robot])

        chained = model._trips[robot]
        self.assertEqual(("process_line", "drop", "D_S1"),
                         (chained["kind"], chained["phase"], chained["goal"]))
        self.assertFalse(chained["admission_tiles"].issubset(old_window))
        self.assertEqual(frozenset({"AC", "CD"}), model.bridges.required(
            model.book.plan("A_OUT", "D_S1", closed=model.closed).nodes,
        ))
        self.assertEqual("AC_S_QUEUE", lookahead.call_args.args[2])
        self.assertEqual(old_resources, model.bridges.held(robot))
        self.assertEqual(
            model.bridges.required(chained["admission_nodes"]),
            model.bridges.held(robot),
        )
        self.assertTrue(model.bridges.can_retarget(
            robot, chained["admission_nodes"], route_tiles=chained["admission_tiles"],
        ))
        state = model.line.snapshot()
        self.assertEqual((1, 0, 1),
                         (state["completed_lots"], state["returned"], model._deliveries[robot]))
        self.assertEqual(carrier, model.inventory.robot_loads[robot])
        event = next(row for row in model.snapshot()["events"]
                     if row["type"] == "line_transport_assigned")
        self.assertEqual(("same_station", True),
                         (event["handoff"], event["avoided_staging_reapproach"]))

        # This focused test jumps to the destination instead of advancing the
        # rolling windows that would release the retained route in production.
        model.bridges.release(robot, stopped=True)
        model._poses[robot].update(
            x_m=model.book.nodes["D_S1"][0], y_m=model.book.nodes["D_S1"][1],
        )
        model._arrive_trip(robot, model._trips[robot])
        state = model.line.snapshot()
        self.assertEqual((1, 1, 2),
                         (state["completed_lots"], state["returned"], model._deliveries[robot]))
        self.assertNotIn(robot, model.inventory.robot_loads)

    def test_shipping_drop_vacates_to_staging_when_next_loaded_window_is_blocked(self):
        model = LineOperationsModel(
            CONFIGS / "factory-semiconductor-campus16", initial_lots=0,
            run_id="line-same-station-blocked",
        )
        robot = "amr-16"
        delivered = _TripTask(
            "LOT-X:product_move:4", "LOT-X", "T999", "A_PACK_OUT", "A_OUT",
            "product_move",
        )
        model._trip(
            robot, delivered, delivered.pickup_station, delivered.drop_station,
            "drop", kind="process_line",
        )
        model._poses[robot].update(
            x_m=model.book.nodes["A_OUT"][0], y_m=model.book.nodes["A_OUT"][1],
        )
        old_window = frozenset(model._trips[robot]["admission_tiles"])
        model.bridges.admit(
            robot,
            model._trips[robot]["admission_nodes"],
            route_tiles=old_window,
            handover_protected=True,
        )
        model._station_owners.update({"A_PACK_OUT": robot, "A_OUT": robot})
        model._jobs[robot] = ("A_PACK_OUT", "A_OUT")
        ready = TransportRequest(
            "LOT-X:empty_return:5", "LOT-X", "T999", "empty_return",
            "A_OUT", "A_S1", model.sim_time,
        )
        adapter = _TripTask.from_request(ready)
        route = model.book.plan(ready.origin, ready.destination, closed=model.closed)
        successor = model._window_for_trip(
            route.points[0], route.points[1:], route.nodes[1:],
            {"kind": "process_line", "phase": "drop",
             "goal": ready.destination, "task": adapter},
            robot_id=robot,
        )
        self.assertFalse(successor["tiles"].issubset(old_window))
        self.assertFalse(model.bridges.can_retarget(
            robot, successor["nodes"], route_tiles=successor["tiles"],
        ))

        with (
            patch.object(type(model.line), "ready_transports", new_callable=PropertyMock,
                         return_value=(ready,)),
            patch.object(model.line, "drop"),
            patch.object(model.line, "assign") as assign,
            patch.object(model.inventory, "product_drop"),
            patch.object(model.inventory, "empty_pickup") as empty_pickup,
            patch.object(model.operations_charging, "can_assign", return_value=True),
            patch.object(model, "_safe_exit_lookahead_blockers", return_value=()),
            patch.object(
                model, "_admission_observations",
                return_value={robot: model._footprint(robot), "amr-2": successor["tiles"]},
            ),
        ):
            model._arrive_trip(robot, model._trips[robot])

        staged = model._trips[robot]
        self.assertEqual(("parking", "staging"), (staged["kind"], staged["purpose"]))
        self.assertTrue(staged["admission_tiles"].issubset(old_window))
        assign.assert_not_called()
        empty_pickup.assert_not_called()

    def test_home_parking_is_preemptible_at_unstarted_or_intermediate_safe_hold(self):
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

        del model._chunks[robot]
        route = model.book.plan(start, "P1")
        intermediate_hold = route.nodes[2]
        x, y = model.book.nodes[intermediate_hold]
        model._poses[robot].update(x_m=x, y_m=y)
        self.assertNotEqual(intermediate_hold, model._locations[robot])
        self.assertTrue(model._can_withdraw_queued_parking(robot))
        self.assertEqual(intermediate_hold, model._dispatch_origin(robot))
        charging = model.operations_charging
        assert charging is not None
        with patch.object(model.book, "plan", wraps=model.book.plan) as plan:
            charging._assignment_required_wh(model, robot, "S1", "CI")
        self.assertEqual(intermediate_hold, plan.call_args_list[0].args[0])
        self.assertTrue(model._withdraw_queued_parking(robot))
        self.assertEqual(intermediate_hold, model._locations[robot])
        self.assertNotIn(robot, model._trips)

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

    def test_coverage_eta_prefers_incomplete_delivery_quota_then_nearest_robot(self):
        coverage_config = CONFIGS / "factory-semiconductor-campus16-coverage-eta"
        model = LineOperationsModel(
            coverage_config, policy="eta", initial_lots=1, run_id="coverage-quota",
        )
        self._keep_only_available(model, "amr-1", "amr-3")
        model._deliveries.update({"amr-1": 2, "amr-3": 1})
        model._assignments.update({"amr-1": 0, "amr-3": 99})
        model._dispatch_extra_jobs(False)
        assigned = next(task for task in model.line.snapshot()["transports"]
                        if task["state"] == "assigned")
        self.assertEqual("amr-3", assigned["robot_id"])

        coverage = LineOperationsModel(
            coverage_config, policy="eta", initial_lots=1, run_id="coverage-complete",
        )
        self._keep_only_available(coverage, "amr-1", "amr-3")
        coverage._deliveries.update({"amr-1": 2, "amr-3": 2})
        coverage._assignments.update({"amr-1": 99, "amr-3": 0})
        coverage._dispatch_extra_jobs(False)
        assigned = next(task for task in coverage.line.snapshot()["transports"]
                        if task["state"] == "assigned")
        self.assertEqual("amr-1", assigned["robot_id"])

        existing = LineOperationsModel(
            CONFIGS / "factory-semiconductor-campus16", policy="eta", initial_lots=1,
            run_id="existing-least-assigned",
        )
        self._keep_only_available(existing, "amr-1", "amr-3")
        existing._assignments.update({"amr-1": 99, "amr-3": 0})
        existing._dispatch_extra_jobs(False)
        assigned = next(task for task in existing.line.snapshot()["transports"]
                        if task["state"] == "assigned")
        self.assertEqual("amr-3", assigned["robot_id"])

    def test_coverage_eta_keeps_capacity_reservation_energy_and_policy_gates(self):
        config = CONFIGS / "factory-semiconductor-campus16-coverage-eta"
        with self.assertRaisesRegex(ValueError, "requires eta policy"):
            LineOperationsModel(config, policy="fifo", initial_lots=0, run_id="coverage-fifo")

        model = LineOperationsModel(config, policy="eta", initial_lots=1, run_id="coverage-gates")
        self._keep_only_available(model, "amr-1")
        with patch.object(model, "_line_can_assign", return_value=False):
            model._dispatch_extra_jobs(False)
        self.assertFalse(model._trips)

        with patch.object(model, "_prospective_trip_blockers", return_value=("amr-2",)):
            model._dispatch_extra_jobs(False)
        self.assertFalse(model._trips)

        charging = model.operations_charging
        assert charging is not None
        with patch.object(charging, "can_assign", return_value=False) as can_assign:
            model._dispatch_extra_jobs(False)
        self.assertTrue(can_assign.called)
        self.assertFalse(model._trips)

    def test_coverage_eta_bundle_only_changes_dispatch_contract(self):
        baseline = CONFIGS / "factory-semiconductor-campus16"
        candidate = CONFIGS / "factory-semiconductor-campus16-coverage-eta"
        for path in baseline.iterdir():
            if path.name != "process-network.json":
                self.assertEqual(path.read_bytes(), (candidate / path.name).read_bytes())
        base_network = json.loads((baseline / "process-network.json").read_text(encoding="utf-8"))
        candidate_network = json.loads(
            (candidate / "process-network.json").read_text(encoding="utf-8"),
        )
        self.assertEqual("downstream_wip_drain_eta_after_coverage",
                         candidate_network.pop("dispatch_mode"))
        self.assertEqual(2, candidate_network.pop("coverage_deliveries_per_robot"))
        base_network.pop("dispatch_mode")
        self.assertEqual(base_network, candidate_network)

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
