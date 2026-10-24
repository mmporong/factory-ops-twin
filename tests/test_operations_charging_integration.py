"""Operations-mode charging through the real route and reservation loop."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import unittest

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

    def test_charge_queue_preempts_home_parking_only_at_safe_stop(self):
        model = OperationsModel(
            CONFIG, initial_orders=0, run_id="operations-charge-parking-preempt",
            operations_charging_profile=self.profile("amr-3"),
        )
        charging = model.operations_charging
        assert charging is not None
        model._locations["amr-1"] = "P3"
        model._poses["amr-1"].update(
            x_m=model.book.nodes["P3"][0], y_m=model.book.nodes["P3"][1],
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


if __name__ == "__main__":
    unittest.main()
