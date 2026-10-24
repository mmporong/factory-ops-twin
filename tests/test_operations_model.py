from collections import deque
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from factory_ops.dashboard import validate_snapshot
from factory_ops.operations_model import OperationsModel
from factory_ops.operations_replan import find_alternative_route


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-dual-lane"


class OperationsModelTest(unittest.TestCase):
    def test_denied_replan_keeps_original_route_pose_and_lease(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="operations-replan-denied")
        robot = "amr-4"
        model._trip(robot, {}, "P4", "P1", "parking", kind="parking")
        trip = model._trips[robot]
        original_route = tuple(trip["points"])
        original_pose = dict(model._poses[robot])
        original_lease = dict(model._leases[robot])
        alternative = model.book.plan("P4", "P3", closed=False)
        model._replans[robot] = {
            "points": alternative.points[1:], "nodes": alternative.nodes, "start": "P4",
            "reason": "persistent_reservation_wait_cycle", "cycle": ("amr-1", robot), "attempt": 1,
        }
        with patch.object(model, "_claim", return_value=False):
            model._move_trip(robot, trip, 0.1)
        self.assertEqual(tuple(trip["points"]), original_route)
        self.assertEqual(model._poses[robot], original_pose)
        self.assertEqual(model._leases[robot], original_lease)
        self.assertIn(robot, model._replans)
        self.assertNotIn(robot, model._chunks)

    def test_snapshot_route_drops_collinear_waypoints_already_passed(self):
        trip = {
            "points": deque(((2.0, 0.0), (4.0, 0.0), (6.0, 0.0))),
            "nodes": deque(("N1", "N2", "N3")),
        }
        chunk = {
            "points": deque(((6.0, 0.0),)), "count": 3, "consumed": 0,
            "start": (0.0, 0.0), "end": (6.0, 0.0), "corridor": None,
        }
        OperationsModel._consume_chunk_route(
            {"x_m": 4.5, "y_m": 0.0, "yaw_rad": 0.0}, trip, chunk,
        )
        self.assertEqual(tuple(trip["points"]), ((6.0, 0.0),))
        self.assertEqual(tuple(trip["nodes"]), ("N3",))
        self.assertEqual(chunk["consumed"], 2)

        OperationsModel._consume_chunk_route(
            {"x_m": 6.0, "y_m": 0.0, "yaw_rad": 0.0}, trip, chunk,
        )
        self.assertEqual(tuple(trip["points"]), ((6.0, 0.0),))
        self.assertEqual(chunk["consumed"], 2)

        chunk["points"].clear()
        OperationsModel._consume_chunk_route(
            {"x_m": 6.0, "y_m": 0.0, "yaw_rad": 0.0}, trip, chunk,
        )
        self.assertFalse(trip["points"])
        self.assertFalse(trip["nodes"])
        self.assertEqual(chunk["consumed"], 3)

    def test_trip_preview_is_only_the_next_stoppable_window(self):
        campus = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-campus16"
        model = OperationsModel(campus, initial_orders=0, run_id="rolling-window-preview")
        robot = "amr-16"
        model._trip(robot, {}, "D_P4", "A_PACK_IN", "parking", kind="parking")
        trip = model._trips[robot]
        route = model.book.plan("D_P4", "A_PACK_IN")
        window = model._next_route_window(route.points[0], route.points[1:], route.nodes[1:])

        self.assertEqual(window["nodes"], trip["admission_nodes"])
        self.assertEqual(window["tiles"], trip["admission_tiles"])
        self.assertLess(len(trip["admission_nodes"]), len(route.nodes) - 1)

    def test_mandatory_queue_truncates_before_atomic_corridor_input(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="mandatory-queue-barrier")
        model.bridges.queue_nodes = frozenset({"P2"})
        start = model.book.nodes["P1"]
        points = (model.book.nodes["P2"], model.book.nodes["P3"])
        with patch.object(model.corridors, "atomic_span", return_value=None) as atomic_span:
            window = model._next_route_window(start, points, ("P2", "P3"))

        self.assertEqual(((model.book.nodes["P2"]),), atomic_span.call_args.args[1])
        self.assertEqual(("P2",), window["nodes"])
        self.assertEqual("P2", window["queue_barrier"])

        with patch.object(model.corridors, "atomic_span", return_value=None) as after_queue:
            model._next_route_window(model.book.nodes["P2"], (model.book.nodes["P3"],), ("P3",))
        self.assertEqual((model.book.nodes["P3"],), after_queue.call_args.args[1])

    def test_mandatory_safe_exit_splits_opposing_queue_windows(self):
        campus = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-campus16"
        model = OperationsModel(campus, initial_orders=0, run_id="mandatory-safe-exit")
        route = model.book.plan("CD_E_QUEUE", "D_CO")
        points, nodes, safe_exit = model._route_via_safe_exit(
            route.points[0], {"goal": "D_CO"}, route.points[1:], route.nodes[1:],
        )
        window = model._next_route_window(route.points[0], points, nodes)

        expected_exit = model.bridges.safe_exit_nodes["CD_E_QUEUE"]
        self.assertEqual(expected_exit, safe_exit)
        self.assertEqual(expected_exit, window["nodes"][-1])
        self.assertTrue(window["tiles"].isdisjoint(
            model._tiles((model.book.nodes["CD_W_QUEUE"],))
        ))

    def test_queue_entry_waits_until_full_safe_exit_path_is_available(self):
        campus = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-campus16"
        model = OperationsModel(campus, initial_orders=0, run_id="safe-exit-lookahead")
        robot = "amr-1"
        model._trip(robot, {}, "A_P1", "B_PACK_IN", "parking", kind="parking")
        trip = model._trips[robot]
        original_pose = dict(model._poses[robot])

        with patch.object(
            model, "_safe_exit_lookahead_blockers", return_value=("amr-5",),
        ), patch.object(model.bridges, "admit", wraps=model.bridges.admit) as admit:
            model._move_trip(robot, trip, 0.1)

        self.assertFalse(admit.called)
        self.assertNotIn(robot, model._chunks)
        self.assertEqual(original_pose, model._poses[robot])
        self.assertEqual("safe_exit_lookahead_occupied", model._waiting[robot]["reason"])
        self.assertEqual(["amr-5"], model._waiting[robot]["blockers"])

    def test_pickup_handover_propagates_loaded_queue_lookahead(self):
        campus = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-campus16"
        model = OperationsModel(campus, initial_orders=0, run_id="pickup-exit-lookahead")
        route = model.book.plan("A_CELL_WAIT", "A_CO")
        trip = {
            "kind": "process_line",
            "phase": "pickup",
            "goal": "A_CO",
            "task": SimpleNamespace(drop_station="B_AOI_IN"),
        }

        window = model._window_for_trip(
            route.points[0], route.points[1:], route.nodes[1:], trip, robot_id="amr-1",
        )

        self.assertTrue(window["retain_at_handover"])
        self.assertEqual("AB_E_QUEUE", window["nodes"][-1])
        self.assertEqual("AB_E_QUEUE", window["queue_barrier"])

    def test_safe_exit_lookahead_checks_pose_and_active_window_occupancy(self):
        campus = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-campus16"
        model = OperationsModel(campus, initial_orders=0, run_id="safe-exit-occupancy")
        robot, peer, queue = "amr-1", "amr-5", "AB_E_QUEUE"
        model._trip(robot, {}, "A_P1", "B_PACK_IN", "parking", kind="parking")
        trip = model._trips[robot]
        route = model.book.plan(queue, model.bridges.safe_exit_nodes[queue])
        protected = model._tiles(route.points)
        empty = {candidate: frozenset() for candidate in model.ledger.robots}
        occupied = dict(empty)
        occupied[peer] = model._tiles((model.book.nodes[model.bridges.safe_exit_nodes[queue]],))

        with patch.object(model, "_footprint", side_effect=lambda candidate: occupied[candidate]):
            self.assertIn(peer, model._safe_exit_lookahead_blockers(robot, trip, queue))

        tile = next(iter(protected))
        self.assertEqual((), model.bridges.admit(peer, (), route_tiles=(tile,)))
        with patch.object(model, "_footprint", side_effect=lambda candidate: empty[candidate]):
            self.assertIn(peer, model._safe_exit_lookahead_blockers(robot, trip, queue))
        model.bridges.release(peer, stopped=True)
        with patch.object(model, "_footprint", side_effect=lambda candidate: empty[candidate]):
            self.assertEqual((), model._safe_exit_lookahead_blockers(robot, trip, queue))

    def test_cycle_replan_never_replaces_mandatory_queue_exit(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="queue-replan-excluded")
        robot = "amr-1"
        model._trip(robot, {}, "P1", "P3", "parking", kind="parking")
        model.bridges.queue_nodes = frozenset({"P1"})

        with patch("factory_ops.operations_model.find_alternative_route") as find:
            model._schedule_cycle_replan((robot,))

        self.assertFalse(find.called)
        self.assertNotIn(robot, model._replans)
        self.assertEqual(1, model.replan_unavailable_count)

    def test_cycle_replan_rejects_blocked_full_trip_window(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="full-window-replan-check")
        robot = "amr-1"
        model._trip(robot, {}, "P1", "P3", "parking", kind="parking")
        trip = model._trips[robot]
        alternative = model.book.plan("P1", "P3")
        full_window = {
            "nodes": ("LOADED_HANDOVER", "DROP_TAIL", "SAFE_EXIT"),
            "tiles": frozenset({(90, 90), (91, 90)}),
        }

        with patch(
            "factory_ops.operations_model.find_alternative_route", return_value=alternative,
        ), patch.object(
            model, "_window_for_trip", return_value=full_window,
        ) as window_for_trip, patch.object(
            model.bridges, "blockers", return_value=("amr-2",),
        ) as blockers:
            model._schedule_cycle_replan((robot,))

        self.assertIs(window_for_trip.call_args.args[3], trip)
        self.assertEqual(full_window["nodes"], blockers.call_args.args[1])
        self.assertEqual(full_window["tiles"], blockers.call_args.kwargs["route_tiles"])
        self.assertNotIn(robot, model._replans)
        self.assertEqual(1, model.replan_unavailable_count)

    def test_cycle_replan_rejects_candidate_with_blocked_safe_exit(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="replan-safe-exit-check")
        robot = "amr-1"
        model._trip(robot, {}, "P1", "P3", "parking", kind="parking")
        alternative = model.book.plan("P1", "P3")
        window = {
            "nodes": alternative.nodes[1:],
            "tiles": model._tiles(alternative.points),
            "queue_barrier": "Q",
        }

        with patch(
            "factory_ops.operations_model.find_alternative_route", return_value=alternative,
        ), patch.object(
            model, "_window_for_trip", return_value=window,
        ), patch.object(
            model.bridges, "blockers", return_value=(),
        ), patch.object(
            model, "_safe_exit_lookahead_blockers", return_value=("amr-2",),
        ) as lookahead:
            model._schedule_cycle_replan((robot,))

        lookahead.assert_called_once_with(robot, model._trips[robot], "Q")
        self.assertNotIn(robot, model._replans)
        self.assertEqual(1, model.replan_unavailable_count)

    def test_cycle_replan_retries_after_peer_footprint_moves(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="replan-peer-moved")
        robot, peer = "amr-1", "amr-2"
        model._trip(robot, {}, "P1", "P3", "parking", kind="parking")
        alternative = model.book.plan("P1", "P3")

        with patch(
            "factory_ops.operations_model.find_alternative_route", return_value=None,
        ) as find:
            model._schedule_cycle_replan((robot,))
            self.assertEqual(1, find.call_count)
            self.assertNotIn(robot, model._replans)

            model._poses[peer].update(
                x_m=model.book.nodes["P4"][0], y_m=model.book.nodes["P4"][1],
            )
            model._observe_all()
            find.return_value = alternative
            model._schedule_cycle_replan((robot,))

        self.assertEqual(2, find.call_count)
        self.assertIn(robot, model._replans)
        self.assertEqual(2, model.replan_attempt_count)

    def test_empty_pickup_traffic_yield_preserves_task_and_station_claims(self):
        campus = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-campus16"
        model = OperationsModel(campus, initial_orders=0, run_id="cycle-traffic-yield")
        robot = "amr-1"
        task = SimpleNamespace(
            task_id="LOT-YIELD:process_input:1",
            order_id="LOT-YIELD",
            payload_id="T-YIELD",
            pickup_station="A_S1",
            drop_station="A_CI",
        )
        model._station_owners.update(A_S1=robot, A_CI=robot)
        model._jobs[robot] = ("A_S1", "A_CI")
        model._trip(robot, task, "A_P1", "A_S1", "pickup", kind="process_line")
        resume = model._trips[robot]
        base_refuge_route = model.book.plan("A_P1", "A_P2")
        refuge_route = find_alternative_route(
            model.book,
            start="A_P1",
            goal="A_P2",
            closed=False,
            reservation_radius_m=model.radius,
            peer_footprint_tiles=(),
            existing_points=base_refuge_route.points[1:],
        )
        self.assertIsNotNone(refuge_route)
        observations = model.reservations.snapshot(now_s=model.sim_time)["observations"]

        def alternative(*args, **kwargs):
            return refuge_route if kwargs["goal"] == "A_P2" else None

        with patch(
            "factory_ops.operations_model.find_alternative_route", side_effect=alternative,
        ), patch.object(model.bridges, "blockers", return_value=()):
            self.assertTrue(model._schedule_traffic_yield((robot,), observations))

        yield_trip = model._trips[robot]
        self.assertEqual("parking", yield_trip["kind"])
        self.assertEqual("traffic_yield", yield_trip["purpose"])
        self.assertEqual("pickup", yield_trip["phase"])
        self.assertIs(resume, yield_trip["traffic_yield_resume"])
        self.assertNotEqual(base_refuge_route.nodes, refuge_route.nodes)
        self.assertEqual(refuge_route.nodes[1:], tuple(yield_trip["nodes"]))
        self.assertEqual(refuge_route.points[1:], tuple(yield_trip["points"]))
        self.assertEqual({"A_S1": robot, "A_CI": robot}, model._station_owners)
        self.assertEqual(0, model._assignments[robot])
        self.assertEqual(0, model._deliveries[robot])
        row = next(row for row in model.snapshot()["robots"] if row["id"] == robot)
        self.assertEqual(task.task_id, row["task_id"])
        self.assertEqual("traffic_yield", row["movement_purpose"])

        model._poses[robot].update(
            x_m=model.book.nodes["A_P2"][0], y_m=model.book.nodes["A_P2"][1],
        )
        model._arrive_trip(robot, yield_trip)

        resumed = model._trips[robot]
        self.assertEqual("process_line", resumed["kind"])
        self.assertEqual("pickup", resumed["phase"])
        self.assertIs(task, resumed["task"])
        self.assertEqual("A_S1", resumed["goal"])
        self.assertEqual("A_P2", model._locations[robot])
        self.assertEqual({"A_S1": robot, "A_CI": robot}, model._station_owners)
        self.assertEqual(1, model.traffic_yield_count)
        self.assertEqual(1, model.traffic_yield_completed_count)
        self.assertEqual(0, model._assignments[robot])
        self.assertEqual(0, model._deliveries[robot])

    def test_bridge_admission_observes_poses_without_unacquired_preview_cycles(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="operations-admission-order")
        model._trip("amr-1", {}, "P1", "P3", "parking", kind="parking")
        model._trip("amr-2", {}, "P2", "P4", "parking", kind="parking")
        first = model._trips["amr-1"]
        second = model._trips["amr-2"]
        shared_preview = frozenset({(20, 20), (21, 20)})
        first.update(admission_nodes=(), admission_tiles=shared_preview)
        second.update(admission_nodes=(), admission_tiles=shared_preview)
        poses = {robot: frozenset({(index, 0)})
                 for index, robot in enumerate(model.ledger.robots)}
        with patch.object(model, "_footprint", side_effect=lambda robot: poses[robot]):
            first_view = model._admission_observations("amr-1", first)
            second_view = model._admission_observations("amr-2", second)

        self.assertEqual(first_view["amr-2"], poses["amr-2"])
        self.assertEqual(second_view["amr-1"], poses["amr-1"])
        self.assertTrue(shared_preview.isdisjoint(first_view["amr-2"] | second_view["amr-1"]))
        self.assertEqual((), model.bridges.admit(
            "amr-1", (), route_tiles=shared_preview, observed_tiles=first_view,
        ))
        self.assertEqual(
            ("amr-1",),
            model.bridges.blockers(
                "amr-2", (), route_tiles=shared_preview, observed_tiles=second_view,
            ),
        )

    def test_holdable_contract_extends_turning_window_and_rejects_missing_exit(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="holdable-window")
        model.bridges.holdable_nodes = frozenset({"SAFE"})
        model.bridges.queue_nodes = frozenset()
        model._max_segment_m = 2.0
        with patch.object(model.corridors, "atomic_span", return_value=None):
            window = model._next_route_window(
                (0.0, 0.0), ((1.0, 0.0), (1.0, 1.0), (2.0, 1.0)),
                ("THROUGH_STATION", "TURN", "SAFE"),
            )
            self.assertEqual(("THROUGH_STATION", "TURN", "SAFE"), window["nodes"])
            self.assertEqual("holdable-tail", window["corridor"])
            self.assertFalse(window["terminal_service"])

            with self.assertRaisesRegex(RuntimeError, "no holdable endpoint"):
                model._next_route_window(
                    (0.0, 0.0), ((1.0, 0.0),), ("THROUGH_STATION",),
                )
            terminal = model._next_route_window(
                (0.0, 0.0), ((1.0, 0.0),), ("SERVICE_GOAL",),
                allow_terminal=True,
            )
            self.assertTrue(terminal["terminal_service"])

    def test_process_drop_keeps_protection_until_first_holdable_home_exit(self):
        model = OperationsModel(CONFIG, initial_orders=0, run_id="holdable-service-exit")
        model.bridges.holdable_nodes = frozenset({"P1", "P2", "P3", "P4"})
        drop = model.book.plan("OUT", "S1")
        window = model._window_for_trip(
            drop.points[-2], drop.points[-1:], drop.nodes[-1:],
            {"kind": "process_line", "phase": "drop", "goal": "S1"},
            robot_id="amr-1",
        )

        self.assertTrue(window["retain_at_handover"])
        self.assertEqual("S1", window["nodes"][0])
        self.assertEqual("P1", window["nodes"][-1])
        self.assertTrue(model._tiles(model.book.plan("S1", "P1").points).issubset(window["tiles"]))

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
        self.assertEqual(
            state["traffic"]["reservation_policy"],
            "bounded_segment_with_atomic_shared_corridors_and_station_gates",
        )

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

    def test_closed_eta_twelve_lots_clear_west_link_and_keep_progressing(self):
        model = OperationsModel(
            CONFIG, policy="eta", initial_orders=12, run_id="operations-closed-eta-regression",
        )
        self.assertEqual(model.set_aisle_closed({"closed": True})["status"], "applied")
        low_west = model.book.nodes["LOW_W_W"]
        west_mid = model.book.nodes["W_MID"]
        at_1360 = None
        for second in range(1, 1401):
            model.advance(1.0)
            if second >= 1200:
                first, third = model._poses["amr-1"], model._poses["amr-3"]
                first_next = next(iter(model._trips.get("amr-1", {}).get("points", ())), None)
                third_next = next(iter(model._trips.get("amr-3", {}).get("points", ())), None)
                opposing_cycle = (
                    model.motions["amr-1"].stopped
                    and model.motions["amr-3"].stopped
                    and (
                        ((first["x_m"], first["y_m"]) == west_mid and first_next == low_west
                         and (third["x_m"], third["y_m"]) == low_west and third_next == west_mid)
                        or
                        ((third["x_m"], third["y_m"]) == west_mid and third_next == low_west
                         and (first["x_m"], first["y_m"]) == low_west and first_next == west_mid)
                    )
                )
                self.assertFalse(opposing_cycle, f"west-link opposing hold cycle at {second}s")
            if second == 1360:
                at_1360 = (
                    model.snapshot()["kpis"]["completed_orders"],
                    tuple((robot, tuple(sorted(pose.items()))) for robot, pose in sorted(model._poses.items())),
                )
        self.assertIsNotNone(at_1360)
        at_1400 = (
            model.snapshot()["kpis"]["completed_orders"],
            tuple((robot, tuple(sorted(pose.items()))) for robot, pose in sorted(model._poses.items())),
        )
        self.assertNotEqual(at_1400, at_1360)

    def test_tester_down_eta_cycle_replans_and_then_moves_forward(self):
        sources = ("S1", "S3", "S2", "S1", "S2", "S3", "S1", "S3", "S2", "S1", "S2", "S3")
        model = OperationsModel(
            CONFIG, policy="eta", initial_orders=0, run_id="operations-tester-down-eta-replan",
        )
        released = 0
        applied_at = None
        travel_at_apply = None
        for second in range(2400):
            while released < len(sources) and released * 30 <= model.sim_time + 1e-9:
                model.create_order({"source": sources[released]})
                released += 1
            if second == 120:
                model.set_equipment_availability({"available": False, "reason": "authored test outage"})
            if second == 300:
                model.set_equipment_availability({"available": True, "reason": "authored test recovery"})
            model.advance(1.0)
            if applied_at is None and model.replan_applied_count:
                applied_at = model.sim_time
                travel_at_apply = dict(model._travel_m)
            if applied_at is not None and model.sim_time >= applied_at + 120.0:
                break
        self.assertEqual(model.replan_attempt_count, 1)
        self.assertEqual(model.replan_applied_count, 1)
        self.assertEqual(model.replan_unavailable_count, 0)
        self.assertIsNone(model.wait_cycle)
        self.assertIsNotNone(travel_at_apply)
        assert travel_at_apply is not None
        self.assertGreater(model._travel_m["amr-4"], travel_at_apply["amr-4"])
        events = model.snapshot()["events"]
        applied = [event for event in events if event["type"] == "traffic_replan_applied"]
        self.assertEqual(len(applied), 1)
        self.assertEqual(applied[0]["robot_id"], "amr-4")
        self.assertIn("EN_OUT", applied[0]["route_nodes"])
        traffic = model.snapshot()["traffic"]
        self.assertEqual(traffic["replan_applied_count"], 1)
        self.assertEqual(traffic["last_replan_reason"], "persistent_reservation_wait_cycle")


if __name__ == "__main__":
    unittest.main()
