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


if __name__ == "__main__":
    unittest.main()
