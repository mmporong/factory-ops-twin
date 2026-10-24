"""Battery conservation and charger lifecycle with observed motion/contact."""
import json
from pathlib import Path
import unittest

from factory_ops.charging import ChargingFleet
from factory_ops.energy import EnergyFleet


PROFILE = Path(__file__).resolve().parents[1] / "configs/factory-fleet4/energy-profile.json"


class EnergyTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(PROFILE.read_text())
        self.energy = EnergyFleet(self.profile, {"r1": 0.5, "r2": 0.16})

    def sample(self, stamp, speed=0.0, turn=0.0, *, contact=False, loaded=False):
        return self.energy.observe("r1", stamp, speed, turn, loaded=loaded, dock_contact_confirmed=contact)

    def test_acceleration_and_braking_use_integrated_observed_motion(self):
        self.sample(0, 0)
        self.sample(0.2, 0.5)
        result = self.sample(0.4, 0)
        # The trapezoidal observations cover 0.1m and 0.4s of idle draw.
        self.assertAlmostEqual(0.1 * 0.05 + 45 * 0.4 / 3600, result["consumed_wh"])
        self.assertAlmostEqual(300 - result["consumed_wh"], result["energy_wh"])

    def test_reverse_turn_and_load_have_nonnegative_additive_cost(self):
        self.sample(0, -0.5, -1, loaded=True)
        result = self.sample(0.2, -0.5, -1, loaded=True)
        expected = 45 * 0.2 / 3600 + 0.05 * 0.1 * 1.35 + 0.015 * 0.2
        self.assertAlmostEqual(expected, result["consumed_wh"])
        self.assertEqual("authored_virtual_energy_model", result["battery_source"])
        self.assertIs(result["hardware_calibrated"], False)

    def test_same_sample_is_idempotent_but_conflict_and_stale_are_rejected(self):
        original = self.sample(1, 0.1)
        self.assertEqual(original, self.sample(1, 0.1))
        for args in ((1, 0.2), (0.9, 0.1), (1.4, 0.1)):
            with self.assertRaises(ValueError):
                self.sample(*args)
            self.assertEqual(original, self.energy.snapshot("r1"))

    def test_contact_confirmation_does_not_backdate_charging(self):
        self.sample(0)
        first = self.sample(0.2, contact=True)
        self.assertEqual(0, first["charged_wh"])
        next_sample = self.sample(0.4, contact=True)
        self.assertAlmostEqual(600 * 0.9 * 0.2 / 3600, next_sample["charged_wh"])
        lost = self.sample(0.6, contact=False)
        self.assertEqual(next_sample["charged_wh"], lost["charged_wh"])
        self.assertAlmostEqual(300 + lost["charged_wh"] - lost["consumed_wh"], lost["energy_wh"])

    def test_charging_at_capacity_is_capped_and_moving_contact_is_rejected(self):
        energy = EnergyFleet(self.profile, {"r1": 1})
        energy.observe("r1", 0, 0, 0, loaded=False, dock_contact_confirmed=True)
        result = energy.observe("r1", 0.2, 0, 0, loaded=False, dock_contact_confirmed=True)
        self.assertAlmostEqual(600, result["energy_wh"])
        self.assertAlmostEqual(result["consumed_wh"], result["charged_wh"])
        for speed, turn in ((0.03, 0), (0, -0.03)):
            with self.assertRaises(ValueError):
                energy.observe("r1", 0.4, speed, turn, loaded=False, dock_contact_confirmed=True)

    def test_assignment_requires_fresh_observation_and_charger_return_reserve(self):
        request = dict(outbound_m=10, charger_return_m=20, expected_wait_s=30,
                       nominal_speed_m_s=0.5, expected_rotation_rad=6)
        self.assertFalse(self.energy.can_assign("r1", **request, loaded=True, observed_at_s=0))
        self.sample(1)
        self.assertTrue(self.energy.can_assign("r1", **request, loaded=True, observed_at_s=1.1))
        self.assertFalse(self.energy.can_assign("r1", **request, loaded=True, observed_at_s=1.4))
        self.energy.observe("r2", 1, 0, 0, loaded=False)
        self.assertFalse(self.energy.can_assign("r2", **{**request, "outbound_m": 1000}, loaded=True, observed_at_s=1))


class ChargingTests(unittest.TestCase):
    def setUp(self):
        self.docks = ChargingFleet("run1", {"CH1": "P1"}, max_gap_s=0.3,
            contact_dwell_s=0.5, stopped_linear_m_s=0.02, stopped_angular_rad_s=0.02, resume_soc=0.8)

    def reserve_and_arrive(self):
        self.assertEqual("CH1", self.docks.reserve("r1", carrying_payload=False))
        self.docks.arrived("r1", run_id="run1", anchor="P1", navigation_succeeded=True)

    def observation(self, stamp, **changes):
        values = dict(run_id="run1", dock_id="CH1", stamp_s=stamp,
                      contact=True, linear_m_s=0.0, angular_rad_s=0.0, soc=0.2)
        return self.docks.observe("r1", **{**values, **changes})

    def test_slot_is_exclusive_and_loaded_vehicle_is_not_diverted(self):
        self.assertEqual("CH1", self.docks.reserve("r1", carrying_payload=False))
        self.assertEqual("CH1", self.docks.reserve("r1", carrying_payload=False))
        self.assertIsNone(self.docks.reserve("r2", carrying_payload=False))
        self.assertFalse(self.docks.can_dispatch("r1"))
        with self.assertRaises(ValueError):
            self.docks.reserve("r3", carrying_payload=True)

    def test_preferred_slot_can_be_cancelled_only_before_contact(self):
        docks = ChargingFleet("run1", {"CH1": "P1", "CH2": "P2"}, max_gap_s=0.3,
            contact_dwell_s=0.5, stopped_linear_m_s=0.02,
            stopped_angular_rad_s=0.02, resume_soc=0.8)
        self.assertEqual(
            "CH2", docks.reserve("r1", carrying_payload=False, preferred_dock="CH2"),
        )
        self.assertEqual("CH2", docks.cancel_approach("r1"))
        self.assertFalse(docks.owners)
        self.assertEqual(
            "CH1", docks.reserve("r1", carrying_payload=False, preferred_dock="CH1"),
        )
        docks.arrived("r1", run_id="run1", anchor="P1", navigation_succeeded=True)
        with self.assertRaises(RuntimeError):
            docks.cancel_approach("r1")

    def test_nav_success_alone_never_confirms_contact(self):
        self.reserve_and_arrive()
        state = self.observation(0, contact=False)
        self.assertEqual("contact_pending", state["phase"])
        self.assertIs(state["contact_confirmed"], False)
        with self.assertRaises(ValueError):
            self.docks.release("r1", run_id="run1", observed_at_s=0)

    def test_run_and_dock_identity_do_not_mutate_other_slot(self):
        self.reserve_and_arrive()
        for changes in ({"run_id": "other"}, {"dock_id": "CH2"}):
            with self.assertRaises(ValueError):
                self.observation(0, **changes)
        self.assertIsNone(self.docks.states["r1"].last_stamp_s)

    def test_contact_dwell_resets_on_motion_or_contact_loss(self):
        self.reserve_and_arrive()
        self.observation(0)
        self.observation(0.2)
        self.observation(0.4, linear_m_s=0.1)
        for stamp in (0.6, 0.8, 1.0):
            self.assertIs(self.observation(stamp)["contact_confirmed"], False)
        self.assertIs(self.observation(1.2)["contact_confirmed"], True)
        self.assertIs(self.observation(1.4, contact=False)["contact_confirmed"], False)

    def test_stale_contact_latches_fault_without_releasing_reserved_slot(self):
        self.reserve_and_arrive()
        self.observation(0)
        with self.assertRaises(ValueError):
            self.observation(0.4)
        state = self.docks.states["r1"]
        self.assertEqual("fault", state.phase)
        self.assertFalse(state.contact_confirmed)
        self.assertFalse(self.docks.can_dispatch("r1"))
        self.assertIsNone(self.docks.reserve("r2", carrying_payload=False))

    def test_resume_requires_soc_then_observed_disengagement(self):
        self.reserve_and_arrive()
        for stamp in (0, 0.2, 0.4):
            self.observation(stamp)
        state = self.observation(0.6, soc=0.8)
        self.assertEqual("undock_pending", state["phase"])
        self.assertFalse(state["contact_confirmed"])
        with self.assertRaises(ValueError):
            self.docks.release("r1", run_id="run1", observed_at_s=0.6)
        self.observation(0.8, contact=False, soc=0.8)
        self.docks.release("r1", run_id="run1", observed_at_s=0.8)
        self.assertTrue(self.docks.can_dispatch("r1"))
        self.assertEqual("CH1", self.docks.reserve("r2", carrying_payload=False))

    def test_ready_slot_recontact_or_motion_revokes_release_and_retains_owner(self):
        self.reserve_and_arrive()
        for stamp in (0, 0.2, 0.4):
            self.observation(stamp)
        self.observation(0.6, soc=0.8)
        self.observation(0.8, contact=False, soc=0.8)
        self.assertEqual("undock_pending", self.observation(1.0, contact=True, soc=0.8)["phase"])
        with self.assertRaises(ValueError):
            self.docks.release("r1", run_id="run1", observed_at_s=1.0)
        self.assertIsNone(self.docks.reserve("r2", carrying_payload=False))
        self.observation(1.2, contact=False, soc=0.8)
        self.assertEqual("undock_pending", self.observation(1.4, contact=False, linear_m_s=0.1, soc=0.8)["phase"])
        with self.assertRaises(ValueError):
            self.docks.release("r1", run_id="run1", observed_at_s=1.4)

    def test_release_requires_fresh_disengagement_at_decision_time(self):
        self.reserve_and_arrive()
        for stamp in (0, 0.2, 0.4):
            self.observation(stamp)
        self.observation(0.6, soc=0.8)
        self.observation(0.8, contact=False, soc=0.8)
        for stamp in (0.7, 1.2):
            with self.assertRaises(ValueError):
                self.docks.release("r1", run_id="run1", observed_at_s=stamp)
        self.assertIsNone(self.docks.reserve("r2", carrying_payload=False))
        self.docks.release("r1", run_id="run1", observed_at_s=0.9)


if __name__ == "__main__":
    unittest.main()
