"""Operations-mode charging through the real route and reservation loop."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from factory_ops.dashboard import validate_snapshot
from factory_ops.operations_model import OperationsModel


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-dual-lane"


class OperationsChargingIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.base_profile = json.loads((CONFIG / "operations-charging.json").read_text(encoding="utf-8"))

    def profile(self, low_robot: str):
        profile = deepcopy(self.base_profile)
        profile["initial_soc"][low_robot] = 0.19
        # Test-only acceleration of authored virtual charging. Route motion,
        # reservations, dwell and energy observations retain production logic.
        profile["energy"]["charger_power_w"] = 60000.0
        return profile

    def run_until(self, model, predicate, *, steps=4500):
        for _ in range(steps):
            model.advance(0.1)
            if predicate():
                return
        self.fail(f"condition not reached at {model.sim_time:.1f}s: {model.operations_charging.snapshot()}")

    def test_own_pad_route_alignment_contact_charge_exit_release_and_snapshot(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-own-pad",
            operations_charging_profile=self.profile("amr-1"),
        )
        charging = model.operations_charging
        assert charging is not None
        first = validate_snapshot(model.snapshot())
        self.assertEqual("amr-1", first["charging"]["docks"][0]["owner"])
        self.assertEqual("authored_operations_virtual_energy_model", first["robots"][0]["battery_source"])
        self.assertFalse(first["robots"][0]["hardware_calibrated"])
        self.assertIn("no physics", first["charging"]["scope"])

        phases = set()
        def released():
            state = model.snapshot()
            phase = next(robot for robot in state["robots"] if robot["id"] == "amr-1")["charging_phase"]
            if phase:
                phases.add(phase)
            return any(event["type"] == "charge_released" and event["robot_id"] == "amr-1"
                       for event in state["events"])

        self.run_until(model, released, steps=1500)
        self.assertTrue({"aligning", "contact_pending", "charging", "exiting", "exit_dwell"}.issubset(phases))
        self.assertGreater(charging.energy.snapshot("amr-1")["charged_wh"], 0.0)
        self.assertNotIn("CH1", charging.docks.owners)
        self.assertFalse(charging.faults)
        self.run_until(model, lambda: model.snapshot()["traffic"]["quiescent"], steps=1500)
        self.assertEqual(model.homes, model._locations)

    def test_amr_three_and_four_swap_real_parking_charge_release_then_take_work(self):
        for robot in ("amr-3", "amr-4"):
            with self.subTest(robot=robot):
                model = OperationsModel(
                    CONFIG, initial_orders=0, run_id=f"operations-charge-swap-{robot}",
                    operations_charging_profile=self.profile(robot),
                )
                charging = model.operations_charging
                assert charging is not None
                self.assertEqual(robot, charging.home_swaps[0]["charging_robot"])
                self.assertEqual(4, len(set(model.homes.values())))
                self.assertTrue(charging.relocating)
                self.assertNotIn("CH1", charging.docks.owners)

                self.run_until(model, lambda: model.snapshot()["traffic"]["quiescent"])
                self.assertFalse(charging.queue)
                self.assertFalse(charging.aux)
                self.assertFalse(charging.relocating)
                self.assertFalse(charging.faults)
                self.assertEqual(model.homes, model._locations)
                self.assertGreater(charging.energy.snapshot(robot)["charged_wh"], 0.0)
                self.assertTrue(any(event["type"] == "charge_released" and event["robot_id"] == robot
                                    for event in model.snapshot()["events"]))

                model.create_order({"source": "S1"})
                self.run_until(model, lambda: model._assignments[robot] > 0, steps=1000)
                state = validate_snapshot(model.snapshot())
                self.assertEqual(1, state["traffic"]["assignments"][robot])
                self.assertIsNone(next(row for row in state["robots"] if row["id"] == robot)["charging_phase"])

    def test_empty_reserved_pad_swaps_absent_home_owner_before_return(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-absent-home-owner",
            operations_charging_profile=self.base_profile,
        )
        charging = model.operations_charging
        assert charging is not None
        charging.queue.clear()
        charging.aux.clear()
        charging.relocating.clear()
        model._trips.clear()

        owner = "amr-1"
        charging_robot = "amr-3"
        model._locations[owner] = "S1"
        x, y = model.book.nodes["S1"]
        model._poses[owner].update(x_m=x, y_m=y)
        model._trip(owner, {}, "S1", "P1", "parking", kind="parking", purpose="home")

        self.assertTrue(charging._prepare_home_swap(model, charging_robot, "P1"))
        self.assertEqual("P1", model.homes[charging_robot])
        self.assertEqual("P3", model.homes[owner])
        self.assertEqual("P3", charging.relocating[owner])

        del model._trips[owner]
        model._locations[owner] = "P1"
        x, y = model.book.nodes["P1"]
        model._poses[owner].update(x_m=x, y_m=y)
        model._idle_since[owner] = -3.0
        model._dispatch()

        self.assertEqual("parking", model._trips[owner]["kind"])
        self.assertEqual("P3", model._trips[owner]["goal"])

    def test_home_swap_retargets_stopped_parking_trip_away_from_charging_pad(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-active-home-retarget",
            operations_charging_profile=self.base_profile,
        )
        charging = model.operations_charging
        assert charging is not None
        charging.queue.clear()
        charging.aux.clear()
        charging.relocating.clear()
        model._trips.clear()

        owner = "amr-1"
        charging_robot = "amr-3"
        model._locations[owner] = "P4"
        x, y = model.book.nodes["P4"]
        model._poses[owner].update(x_m=x, y_m=y)
        model._trip(owner, {}, "P4", "P1", "parking", kind="parking", purpose="home")

        self.assertTrue(charging._prepare_home_swap(model, charging_robot, "P1"))
        self.assertEqual("P1", model._trips[owner]["goal"])

        model._chunks[owner] = {"in_flight": True}
        charging.prepare(model)
        self.assertEqual("P1", model._trips[owner]["goal"])

        del model._chunks[owner]
        charging.prepare(model)

        self.assertEqual("P3", model.homes[owner])
        self.assertEqual("P3", model._trips[owner]["goal"])
        self.assertEqual("P4", model._locations[owner])
        self.assertEqual("home", model._trips[owner]["purpose"])
        self.assertTrue(any(
            event["type"] == "charge_home_relocation_retargeted"
            and event["robot_id"] == owner
            and event["prior_goal"] == "P1"
            and event["relocation_home"] == "P3"
            for event in model._events
        ))

    def test_relocated_robot_that_later_charges_drops_superseded_relocation(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-relocation-superseded",
            operations_charging_profile=self.base_profile,
        )
        charging = model.operations_charging
        assert charging is not None
        charging.queue.clear()
        charging.aux.clear()
        charging.relocating.clear()
        model._trips.clear()

        self.assertFalse(charging._prepare_home_swap(model, "amr-3", "P1"))
        self.assertEqual("P3", charging.relocating["amr-1"])

        self.assertFalse(charging._prepare_home_swap(model, "amr-1", "P2"))

        self.assertNotIn("amr-1", charging.relocating)
        self.assertEqual("P2", model.homes["amr-1"])
        self.assertEqual("P3", charging.relocating["amr-2"])
        self.assertEqual(4, len(set(model.homes.values())))
        self.assertTrue(any(
            event["type"] == "charge_home_relocation_superseded"
            and event["robot_id"] == "amr-1"
            and event["prior_relocation_home"] == "P3"
            and event["charging_home"] == "P2"
            for event in model._events
        ))

    def test_charge_queue_preempts_home_parking_only_at_safe_stop(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-parking-preempt",
            operations_charging_profile=self.profile("amr-3"),
        )
        charging = model.operations_charging
        assert charging is not None
        model._locations["amr-1"] = "P3"
        model._poses["amr-1"].update(
            x_m=1.0, y_m=1.0,
        )
        model._trip("amr-3", {}, "P3", "P1", "parking", kind="parking", purpose="home")
        parking = model._trips["amr-3"]
        retained_lease = dict(model._leases["amr-3"])
        model._chunks["amr-3"] = {"in_flight": True}

        charging.prepare(model)

        self.assertIs(parking, model._trips["amr-3"])
        self.assertEqual(["amr-3"], charging.queue)
        self.assertEqual(["amr-3"], charging.snapshot()["parking_preempt_deferred"])
        deferred = [event for event in model._events
                    if event["type"] == "charge_parking_preempt_deferred"]
        self.assertEqual(1, len(deferred))

        del model._chunks["amr-3"]
        charging.prepare(model)

        self.assertEqual("charging", model._trips["amr-3"]["kind"])
        self.assertEqual("charge_approach", model._trips["amr-3"]["phase"])
        self.assertEqual("P1", model._trips["amr-3"]["goal"])
        self.assertEqual(retained_lease, model._leases["amr-3"])
        self.assertEqual({"amr-3": "P1"}, model._charging_parking_resume)
        self.assertFalse(charging.snapshot()["parking_preempt_deferred"])
        self.assertTrue(any(event["type"] == "charge_parking_preempted"
                            for event in model._events))

    def test_home_parking_resumes_after_charge_release(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-parking-resume",
            operations_charging_profile=self.profile("amr-1"),
        )
        charging = model.operations_charging
        assert charging is not None
        charging.queue.clear()
        charging.aux.clear()
        charging.relocating.clear()
        model._trips.pop("amr-1", None)
        model._locations["amr-1"] = "LOW_E_P1"
        model._charging_parking_resume["amr-1"] = "P1"
        model._idle_since["amr-1"] = -3.0

        model._dispatch()

        trip = model._trips["amr-1"]
        self.assertEqual("parking", trip["kind"])
        self.assertEqual("home", trip["purpose"])
        self.assertEqual(model.homes["amr-1"], trip["goal"])
        self.assertNotIn("amr-1", model._charging_parking_resume)
        self.assertTrue(any(event["type"] == "charge_parking_resumed"
                            for event in model._events))

    def test_stopped_empty_pickup_charges_then_resumes_with_station_claims(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-pickup-resume",
            operations_charging_profile=self.base_profile,
        )
        charging = model.operations_charging
        assert charging is not None
        robot = "amr-3"
        task = SimpleNamespace(
            task_id="LOT-CHARGE:process_input:1",
            order_id="LOT-CHARGE",
            payload_id="T-CHARGE",
            pickup_station="S1",
            drop_station="CI",
        )
        model._locations["amr-1"] = "P4"
        model._poses["amr-1"].update(
            x_m=model.book.nodes["P4"][0], y_m=model.book.nodes["P4"][1],
        )
        model._station_owners.update(S1=robot, CI=robot)
        model._jobs[robot] = ("S1", "CI")
        model._trip(robot, task, "P3", "S1", "pickup", kind="process_line")
        interrupted = model._trips[robot]
        charging.energy.batteries[robot].energy_wh = (
            charging.energy.profile["capacity_wh"] * 0.19
        )

        charging.prepare(model)

        self.assertIs(interrupted, model._charging_task_resume[robot])
        self.assertEqual("charging", model._trips[robot]["kind"])
        self.assertEqual({"S1": robot, "CI": robot}, model._station_owners)
        self.assertTrue(any(event["type"] == "charge_task_preempted"
                            for event in model._events))

        charging.docks.cancel_approach(robot)
        charging.aux.clear()
        model._trips.pop(robot)
        model._locations[robot] = "LOW_E_P1"
        model._dispatch()

        resumed = model._trips[robot]
        self.assertIs(task, resumed["task"])
        self.assertEqual("process_line", resumed["kind"])
        self.assertEqual("pickup", resumed["phase"])
        self.assertEqual("S1", resumed["goal"])
        self.assertEqual({"S1": robot, "CI": robot}, model._station_owners)
        self.assertNotIn(robot, model._charging_task_resume)
        self.assertTrue(any(event["type"] == "charge_task_resumed"
                            for event in model._events))

    def test_blocked_precontact_approach_reassigns_to_clear_preferred_dock(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-reassign",
            operations_charging_profile=self.base_profile,
        )
        charging = model.operations_charging
        assert charging is not None
        robot = "amr-3"
        model._locations["amr-2"] = "P4"
        model._poses["amr-2"].update(
            x_m=model.book.nodes["P4"][0], y_m=model.book.nodes["P4"][1],
        )
        self.assertEqual(
            "CH1", charging.docks.reserve(
                robot, carrying_payload=False, preferred_dock="CH1",
            ),
        )
        charging.aux[robot] = {"dock_id": "CH1", "phase": "approach", "stationary_since": None}
        model._trip(robot, {}, "P3", "P1", "charge_approach", kind="charging")

        def option(_robot, goal):
            return {
                "start": "P3",
                "goal": goal,
                "distance_m": 8.0 if goal == "P2" else 9.0,
                "blockers": () if goal == "P2" else ("amr-1",),
                "window_nodes": (goal,),
            }

        with patch.object(model, "_charging_approach_option", side_effect=option):
            charging.prepare(model)

        self.assertNotIn("CH1", charging.docks.owners)
        self.assertEqual(robot, charging.docks.owners["CH2"])
        self.assertEqual("CH2", charging.aux[robot]["dock_id"])
        self.assertEqual("P2", model._trips[robot]["goal"])
        event = next(event for event in model._events if event["type"] == "charge_reassigned")
        self.assertEqual("CH1", event["old_dock_id"])
        self.assertEqual("CH2", event["dock_id"])


if __name__ == "__main__":
    unittest.main()
