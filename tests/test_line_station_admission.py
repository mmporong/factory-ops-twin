import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.dashboard import validate_snapshot
from factory_ops.line_operations_model import LineOperationsModel, _TripTask


ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / "configs"
BASELINE = CONFIGS / "factory-semiconductor-campus16-wide-bridges"
CANDIDATE = CONFIGS / "factory-semiconductor-campus16-wide-bridges-handoff-admission"


class LineStationAdmissionTests(unittest.TestCase):
    @staticmethod
    def _keep_only_available(model, *robot_ids):
        selected = set(robot_ids)
        for robot_id, state in model.ledger.robots.items():
            state.available = robot_id in selected

    @classmethod
    def _api_shape(cls, value):
        if isinstance(value, dict):
            return {key: cls._api_shape(item) for key, item in value.items()}
        if isinstance(value, list):
            return [] if not value else [cls._api_shape(value[0])]
        return type(value)

    def test_candidate_bundle_changes_only_station_gate_policy_and_validates(self):
        baseline_files = sorted(path.name for path in BASELINE.iterdir())
        candidate_files = sorted(path.name for path in CANDIDATE.iterdir())
        self.assertEqual(baseline_files, candidate_files)
        for name in baseline_files:
            if name != "process-network.json":
                self.assertEqual((BASELINE / name).read_bytes(), (CANDIDATE / name).read_bytes())

        baseline = json.loads((BASELINE / "process-network.json").read_text(encoding="utf-8"))
        candidate = json.loads((CANDIDATE / "process-network.json").read_text(encoding="utf-8"))
        self.assertEqual("service_window", candidate.pop("station_gate_acquisition"))
        self.assertEqual(baseline, candidate)

        legacy = LineOperationsModel(BASELINE, initial_lots=0, run_id="handoff-legacy-shape")
        model = LineOperationsModel(CANDIDATE, initial_lots=0, run_id="handoff-config")
        self.assertEqual("service_window", model._station_gate_acquisition)
        legacy_snapshot = validate_snapshot(legacy.snapshot())
        candidate_snapshot = validate_snapshot(model.snapshot())
        self.assertEqual(self._api_shape(legacy_snapshot), self._api_shape(candidate_snapshot))

        with tempfile.TemporaryDirectory() as directory:
            invalid = Path(directory) / "invalid"
            shutil.copytree(CANDIDATE, invalid)
            network_path = invalid / "process-network.json"
            network = json.loads(network_path.read_text(encoding="utf-8"))
            network["station_gate_acquisition"] = "early"
            network_path.write_text(json.dumps(network), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "station_gate_acquisition is unsupported"):
                LineOperationsModel(invalid, initial_lots=0, run_id="handoff-invalid")

    def test_service_window_assigns_two_unique_lots_with_capacity_credit_before_gate(self):
        model = LineOperationsModel(CANDIDATE, initial_lots=0, run_id="handoff-capacity")
        model.create_order({"source": "A_S1"})
        model.create_order({"source": "A_S2"})
        model.create_order({"source": "A_S3"})
        self._keep_only_available(model, "amr-1", "amr-2")

        with (
            patch.object(model, "_prospective_trip_blockers", return_value=()),
            patch.object(model.operations_charging, "can_assign", return_value=True),
        ):
            model._dispatch_extra_jobs(False)

        assigned = [task for task in model.line.snapshot()["transports"]
                    if task["state"] == "assigned"]
        self.assertEqual(2, len(assigned))
        self.assertEqual(2, len({task["lot_id"] for task in assigned}))
        self.assertEqual(2, len({task["carrier_id"] for task in assigned}))
        self.assertEqual(2, model.line.cells["L1.TEST"].reserved_inputs)
        self.assertEqual(1, sum(task["state"] == "ready"
                                for task in model.line.snapshot()["transports"]))
        self.assertEqual({}, model._station_owners)

        first, second = (task["robot_id"] for task in assigned)
        window = {"nodes": ("A_CI",)}
        trip = {"kind": "process_line", "phase": "drop", "goal": "A_CI"}
        self.assertEqual((), model._station_gate_window_blockers(first, trip, window))
        model._commit_station_gate_window(first, trip, window)
        self.assertEqual((first,), model._station_gate_window_blockers(second, trip, window))

        model._jobs.pop(first, None)
        model._trips.pop(first, None)
        home = model.book.nodes[model.homes[first]]
        model._poses[first].update(x_m=home[0], y_m=home[1])
        model._release_station_gates()
        self.assertNotIn("A_CI", model._station_owners)
        model._commit_station_gate_window(second, trip, window)
        self.assertEqual(second, model._station_owners["A_CI"])

    def test_service_window_dispatch_rejects_active_and_bridge_safe_exit_holds(self):
        model = LineOperationsModel(CANDIDATE, initial_lots=0, run_id="handoff-safe-idle")
        robot = "amr-1"
        self.assertTrue(model._service_window_dispatch_ready(robot))

        safe_exit = next(iter(model.bridges.safe_exit_nodes.values()))
        point = model.book.nodes[safe_exit]
        model._poses[robot].update(x_m=point[0], y_m=point[1])
        self.assertFalse(model._service_window_dispatch_ready(robot))

        home = model.book.nodes[model.homes[robot]]
        model._poses[robot].update(x_m=home[0], y_m=home[1])
        model._chunks[robot] = {"active": True}
        self.assertFalse(model._service_window_dispatch_ready(robot))

    def test_service_window_assigns_valid_work_then_waits_for_temporary_first_window(self):
        model = LineOperationsModel(CANDIDATE, initial_lots=1, run_id="handoff-first-window")
        robot, peer = "amr-1", "amr-2"
        self._keep_only_available(model, robot)
        with (
            patch.object(model, "_prospective_trip_blockers", return_value=(peer,)),
            patch.object(model.operations_charging, "can_assign", return_value=True),
        ):
            model._dispatch_extra_jobs(False)

        assigned = next(task for task in model.line.snapshot()["transports"]
                        if task["state"] == "assigned")
        self.assertEqual(robot, assigned["robot_id"])
        self.assertIn(robot, model._trips)
        self.assertEqual(1, model.line.cells["L1.TEST"].reserved_inputs)
        self.assertEqual({}, model._station_owners)

        trip = model._trips[robot]
        with (
            patch.object(model, "_safe_exit_lookahead_blockers", return_value=()),
            patch.object(model.bridges, "admit", return_value=(peer,)),
        ):
            model._move_trip(robot, trip, 0.1)
        self.assertEqual(
            {"reason": "bridge_admission_occupied", "blockers": [peer]},
            model._waiting[robot],
        )
        self.assertEqual("assigned", next(
            task["state"] for task in model.line.snapshot()["transports"]
            if task["task_id"] == assigned["task_id"]
        ))
        self.assertNotIn(robot, model.inventory.robot_loads)

    def test_gate_and_bridge_denials_preserve_trip_lease_and_material(self):
        model = LineOperationsModel(CANDIDATE, initial_lots=0, run_id="handoff-denials")
        robot, peer = "amr-1", "amr-2"
        task = _TripTask("TASK", "LOT", "T001", "A_S1", "A_CI", "process_input")
        route = model.book.plan("A_S1", "A_CI")
        start = route.nodes[-2]
        point = model.book.nodes[start]
        model._locations[robot] = start
        model._poses[robot].update(x_m=point[0], y_m=point[1])
        model._trip(robot, task, start, "A_CI", "drop", kind="process_line")
        trip = model._trips[robot]
        self.assertIn("A_CI", trip["admission_nodes"])

        pose_before = dict(model._poses[robot])
        lease_before = dict(model._leases[robot])
        route_before = tuple(trip["points"])
        model._station_owners["A_CI"] = peer
        with patch.object(model.bridges, "admit") as admit:
            model._move_trip(robot, trip, 0.1)
        admit.assert_not_called()
        self.assertEqual(pose_before, model._poses[robot])
        self.assertEqual(lease_before, model._leases[robot])
        self.assertEqual(route_before, tuple(trip["points"]))
        self.assertEqual((peer,), model.bridges.waiters[robot])
        self.assertEqual({"reason": "handover_station_occupied", "blockers": [peer]},
                         model._waiting[robot])

        del model._station_owners["A_CI"]
        with (
            patch.object(model, "_safe_exit_lookahead_blockers", return_value=()),
            patch.object(model.bridges, "admit", return_value=(peer,)),
            patch.object(model, "_claim") as claim,
        ):
            model._move_trip(robot, trip, 0.1)
        claim.assert_not_called()
        self.assertNotIn("A_CI", model._station_owners)
        self.assertEqual(route_before, tuple(trip["points"]))

        window_tiles = frozenset(trip["admission_tiles"])

        def claim_after_bridge(*_args):
            self.assertNotIn("A_CI", model._station_owners)
            model._leases[robot]["tiles"] = window_tiles
            return True

        def move_after_gate(*_args):
            self.assertEqual(robot, model._station_owners["A_CI"])

        with (
            patch.object(model, "_safe_exit_lookahead_blockers", return_value=()),
            patch.object(model.bridges, "admit", return_value=()),
            patch.object(model, "_claim", side_effect=claim_after_bridge),
            patch.object(model.reservations, "permission", return_value=True),
            patch.object(model.motions[robot], "advance", side_effect=move_after_gate),
        ):
            model._move_trip(robot, trip, 0.1)
        self.assertEqual(robot, model._station_owners["A_CI"])

    def test_zero_distance_pickup_waits_for_gate_before_material_transition(self):
        model = LineOperationsModel(CANDIDATE, initial_lots=1, run_id="handoff-zero")
        robot, peer = "amr-1", "amr-2"
        ready = model.line.ready_transports[0]
        assigned = model.line.assign(ready.task_id, robot)
        task = _TripTask.from_request(assigned)
        model._jobs[robot] = (task.pickup_station, task.drop_station)
        point = model.book.nodes[task.pickup_station]
        model._locations[robot] = task.pickup_station
        model._poses[robot].update(x_m=point[0], y_m=point[1])
        model._trip(
            robot, task, task.pickup_station, task.pickup_station,
            "pickup", kind="process_line",
        )
        self.assertFalse(model._trips[robot]["points"])
        model._station_owners[task.pickup_station] = peer

        model._arrive_trip(robot, model._trips[robot])
        state = next(row for row in model.line.snapshot()["transports"]
                     if row["task_id"] == task.task_id)
        self.assertEqual("assigned", state["state"])
        self.assertNotIn(robot, model.inventory.robot_loads)
        self.assertEqual((peer,), model.bridges.waiters[robot])

        del model._station_owners[task.pickup_station]
        model._arrive_trip(robot, model._trips[robot])
        state = next(row for row in model.line.snapshot()["transports"]
                     if row["task_id"] == task.task_id)
        self.assertEqual("picked", state["state"])
        self.assertEqual(task.payload_id, model.inventory.robot_loads[robot])
        self.assertEqual(robot, model._station_owners[task.pickup_station])
        model._release_station_gates()
        self.assertEqual(robot, model._station_owners[task.pickup_station])

    def test_shipping_return_keeps_same_station_gate_for_continuous_handoff(self):
        model = LineOperationsModel(CANDIDATE, initial_lots=0, run_id="handoff-chain")
        robot = "amr-16"
        lot_id = model.create_order({"source": "D_S1", "quantity": 1})
        line = model.line.lines["L4"]
        lot = line.lots[lot_id]
        carrier = lot.carrier_id
        for task in line.tasks.values():
            task.state = "complete"
        line.cells["PACK"].output_lots.append(lot_id)
        shipping = line._new_task(lot, "product_move", "A_PACK_OUT", "A_OUT")
        assigned = line.assign(shipping.task_id, robot)
        line.pickup(model.ledger.run_id, shipping.task_id, robot, "pickup", model.sim_time)
        model.inventory.source_pickup(lot_id, robot)
        model.inventory.product_drop(lot_id, carrier, robot, "A_PACK_OUT")
        model.inventory.product_pickup(lot_id, carrier, robot, "A_PACK_OUT")
        delivered = _TripTask.from_request(assigned)
        model._trip(robot, delivered, "A_PACK_OUT", "A_OUT", "drop", kind="process_line")
        point = model.book.nodes["A_OUT"]
        model._poses[robot].update(x_m=point[0], y_m=point[1])
        model._station_owners["A_OUT"] = robot
        model._jobs[robot] = ("A_PACK_OUT", "A_OUT")
        model.bridges.admit(
            robot,
            model._trips[robot]["admission_nodes"],
            route_tiles=model._trips[robot]["admission_tiles"],
            handover_protected=True,
        )

        with (
            patch.object(model.operations_charging, "can_assign", return_value=True),
            patch.object(model, "_safe_exit_lookahead_blockers", return_value=()),
            patch.object(model, "_admission_observations",
                         return_value={robot: model._footprint(robot)}),
        ):
            model._arrive_trip(robot, model._trips[robot])

        chained = model._trips[robot]
        self.assertEqual(("drop", "D_S1"), (chained["phase"], chained["goal"]))
        self.assertEqual(robot, model._station_owners["A_OUT"])
        self.assertNotIn("D_S1", model._station_owners)
        self.assertEqual(carrier, model.inventory.robot_loads[robot])

    def test_default_mode_keeps_dispatch_time_station_claims(self):
        model = LineOperationsModel(BASELINE, initial_lots=1, run_id="handoff-default")
        self.assertEqual("dispatch", model._station_gate_acquisition)
        self._keep_only_available(model, "amr-1")
        with (
            patch.object(model, "_prospective_trip_blockers", return_value=()),
            patch.object(model.operations_charging, "can_assign", return_value=True),
        ):
            model._dispatch_extra_jobs(False)
        assigned = next(task for task in model.line.snapshot()["transports"]
                        if task["state"] == "assigned")
        self.assertEqual("amr-1", model._station_owners[assigned["origin"]])
        self.assertEqual("amr-1", model._station_owners[assigned["destination"]])


if __name__ == "__main__":
    unittest.main()
