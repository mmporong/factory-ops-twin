"""Exercise the virtual energy adapter with real EnergyFleet integration."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import unittest

from factory_ops.energy_runtime import EnergyRuntime, GEOMETRIC_CONTACT_SOURCE


PROFILE = Path(__file__).resolve().parents[1] / "configs/factory-fleet4/energy-profile.json"


class EnergyRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = json.loads(PROFILE.read_text(encoding="utf-8"))
        self.runtime = EnergyRuntime("run-015", self.profile, {"r1": .5, "r2": .8})

    def observe(self, stamp: float, linear: float = 0., angular: float = 0., *,
                loaded: bool = False, robot: str = "r1"):
        return self.runtime.observe("run-015", robot, stamp, linear, angular, loaded=loaded)

    def test_signed_motion_load_turn_and_idle_have_analytic_energy_cost(self) -> None:
        for stamp, linear, angular, loaded in (
            (0., 0., 0., False), (.2, .4, -.2, True), (.4, -.4, .2, True),
            (.6, 0., 0., False), (.8, 0., 0., False), (1., 0., 0., False),
        ):
            self.observe(stamp, linear, angular, loaded=loaded)
        state = self.runtime.snapshot()["robots"]["r1"]
        # 1s idle = .0125Wh; .16m loaded travel = .0108Wh;
        # .08rad observed turning = .0012Wh. Reverse motion also consumes energy.
        self.assertAlmostEqual(.0245, state["consumed_wh"])
        self.assertAlmostEqual(299.9755, state["energy_wh"])
        self.assertEqual(0., state["charged_wh"])
        self.assertAlmostEqual(300., state["energy_wh"] + state["consumed_wh"] - state["charged_wh"])

    def test_first_sample_is_only_a_baseline_at_its_actual_timestamp(self) -> None:
        projected = self.observe(100., 1., loaded=True)
        self.assertEqual("fresh", projected["freshness"])
        self.assertEqual(100., projected["observed_sim_time_s"])
        self.assertEqual(50., projected["battery_pct"])
        state = self.runtime.snapshot()["robots"]["r1"]
        self.assertEqual(0., state["consumed_wh"])
        self.assertEqual(300., state["energy_wh"])
        self.observe(100.2, 0., loaded=True)
        # .1m braking travel and .2s idle, with the interval's load factor.
        self.assertAlmostEqual(.00925, self.runtime.snapshot()["robots"]["r1"]["consumed_wh"])

    def test_unloaded_idle_and_rotation_do_not_charge_an_idle_robot(self) -> None:
        self.observe(0., angular=-1.)
        self.observe(.2, angular=-1.)
        self.observe(.4)
        self.observe(.6)
        state = self.runtime.snapshot()["robots"]["r1"]
        # .3rad rotation plus .6s idle, without translation.
        self.assertAlmostEqual(.012, state["consumed_wh"])
        self.assertEqual(0., state["charged_wh"])
        self.assertFalse(self.runtime.projection("run-015", "r1", .6)["charging_supported"])

    def test_exact_duplicate_is_idempotent_for_all_energy_state(self) -> None:
        self.observe(1., -.3, .2, loaded=True)
        first = self.observe(1.2, .1, -.2, loaded=False)
        before = self.runtime.snapshot()
        self.assertEqual(first, self.observe(1.2, .1, -.2, loaded=False))
        self.assertEqual(before, self.runtime.snapshot())

    def test_foreign_run_unknown_robot_and_invalid_queries_do_not_mutate(self) -> None:
        self.observe(1.)
        before = self.runtime.snapshot()
        for run_id, robot in (("other", "r1"), ("run-015", "missing")):
            with self.subTest(run_id=run_id, robot=robot):
                with self.assertRaises(ValueError):
                    self.runtime.observe(run_id, robot, float("nan"), 0., 0., loaded=False)
                with self.assertRaises(ValueError):
                    self.runtime.projection(run_id, robot, 1.)
                self.assertEqual(before, self.runtime.snapshot())
        for now in (float("nan"), float("inf"), -1.):
            with self.assertRaises(ValueError):
                self.runtime.projection("run-015", "r1", now)
            self.assertEqual(before, self.runtime.snapshot())

    def test_invalid_observations_latch_only_the_addressed_robot(self) -> None:
        bad_samples = [
            (.9, 0., 0., False), (1., .1, 0., False), (1., 0., 0., True),
            (1.31, 0., 0., False), (float("nan"), 0., 0., False),
            (1.1, float("inf"), 0., False), (1.1, 0., float("nan"), False),
            (-1., 0., 0., False), (True, 0., 0., False),
        ]
        for stamp, linear, angular, loaded in bad_samples:
            with self.subTest(sample=(stamp, linear, angular, loaded)):
                runtime = EnergyRuntime("run-015", self.profile, {"r1": .5, "r2": .8})
                for robot in ("r1", "r2"):
                    runtime.observe("run-015", robot, 1., 0., 0., loaded=False)
                before = runtime.snapshot()
                with self.assertRaises(ValueError):
                    runtime.observe("run-015", "r1", stamp, linear, angular, loaded=loaded)
                after = runtime.snapshot()
                for key in ("energy_wh", "consumed_wh", "charged_wh", "observed_sim_time_s"):
                    self.assertEqual(before["robots"]["r1"][key], after["robots"]["r1"][key])
                self.assertEqual(before["robots"]["r2"], after["robots"]["r2"])
                self.assertEqual("invalid_observation", after["robots"]["r1"]["fault"]["code"])
                projection = runtime.projection("run-015", "r1", 1.1)
                self.assertEqual("fault", projection["freshness"])
                self.assertIsNone(projection["battery_pct"])
                with self.assertRaises(ValueError):
                    runtime.observe("run-015", "r1", 1.1, 0., 0., loaded=False)
                self.assertEqual(after, runtime.snapshot())
                self.assertEqual("fresh", runtime.observe("run-015", "r2", 1.2, 0., 0., loaded=False)["freshness"])

    def test_first_invalid_observation_never_becomes_an_observed_seed_soc(self) -> None:
        with self.assertRaises(ValueError):
            self.observe(float("inf"))
        projected = self.runtime.projection("run-015", "r1", 1.)
        self.assertEqual("fault", projected["freshness"])
        self.assertIsNone(projected["observed_sim_time_s"])
        self.assertIsNone(projected["battery_pct"])
        with self.assertRaises(ValueError):
            self.observe(1.)

    def test_projection_distinguishes_missing_fresh_stale_and_future_without_mutating(self) -> None:
        initial = self.runtime.projection("run-015", "r1", 0.)
        self.assertEqual("missing", initial["freshness"])
        self.assertIsNone(initial["battery_pct"])
        self.assertIsNone(initial["observed_sim_time_s"])
        self.assertTrue(initial["virtual"])
        self.assertFalse(initial["hardware_calibrated"])
        self.assertEqual("authored_virtual_energy_model", initial["battery_source"])
        self.observe(1.)
        before = self.runtime.snapshot()
        for now, expected in ((1., "fresh"), (1.2, "fresh"), (1.4, "stale"), (.9, "future")):
            projected = self.runtime.projection("run-015", "r1", now)
            self.assertEqual(expected, projected["freshness"])
            self.assertEqual(1., projected["observed_sim_time_s"])
            if expected == "fresh":
                self.assertEqual(50., projected["battery_pct"])
            else:
                self.assertIsNone(projected["battery_pct"])
        self.assertEqual(before, self.runtime.snapshot())
        # A stale query is read-only: it does not falsify callback evidence.
        self.assertEqual("fresh", self.observe(1.2)["freshness"])

    def test_exhaustion_fault_preserves_last_energy_and_other_robot(self) -> None:
        runtime = EnergyRuntime("empty-run", self.profile, {"r1": 0., "r2": .5})
        runtime.observe("empty-run", "r1", 0., 0., 0., loaded=False)
        with self.assertRaises(ValueError):
            runtime.observe("empty-run", "r1", .1, 0., 0., loaded=False)
        state = runtime.snapshot()
        self.assertEqual(0., state["robots"]["r1"]["energy_wh"])
        self.assertEqual(0., state["robots"]["r1"]["consumed_wh"])
        self.assertIsNone(runtime.projection("empty-run", "r1", .1)["battery_pct"])
        self.assertEqual(50., runtime.observe("empty-run", "r2", .1, 0., 0., loaded=False)["battery_pct"])

    def test_no_public_contact_flag_and_no_run_identity_setter(self) -> None:
        before = self.runtime.snapshot()
        with self.assertRaises(TypeError):
            self.runtime.observe("run-015", "r1", 0., 0., 0., loaded=False,
                                 **{"dock_contact_confirmed": True})
        with self.assertRaises(AttributeError):
            setattr(self.runtime, "run_id", "new-run")
        self.assertEqual(before, self.runtime.snapshot())

    def test_detached_snapshot_and_profile_cannot_rewrite_accounting_or_fault(self) -> None:
        self.profile["idle_power_w"] = 9000.
        self.observe(0.)
        self.observe(.2)
        self.assertAlmostEqual(.0025, self.runtime.snapshot()["robots"]["r1"]["consumed_wh"])
        with self.assertRaises(ValueError):
            self.observe(.2, 1.)
        expected = self.runtime.snapshot()
        exposed = self.runtime.snapshot()
        exposed["robots"]["r1"]["energy_wh"] = 600.
        exposed["robots"]["r1"]["fault"]["reason"] = "cleared"
        exposed["provenance"]["kind"] = "hardware"
        projected = self.runtime.projection("run-015", "r1", .2)
        projected["fault"]["reason"] = "cleared"
        self.assertEqual(expected, self.runtime.snapshot())
        json.dumps(expected, allow_nan=False)

    def test_long_stream_keeps_bounded_state_and_configured_freshness_limit(self) -> None:
        runtime = EnergyRuntime("stream", {**self.profile, "max_observation_gap_s": .25}, {"r1": .5})
        runtime.observe("stream", "r1", 0., 0., 0., loaded=False)
        shape = deepcopy(runtime.snapshot())
        for i in range(1, 1001):
            runtime.observe("stream", "r1", i / 8, 0., 0., loaded=False)
        state = runtime.snapshot()
        self.assertEqual(set(shape), set(state))
        self.assertEqual(set(shape["robots"]["r1"]), set(state["robots"]["r1"]))
        self.assertEqual(125., state["robots"]["r1"]["observed_sim_time_s"])
        self.assertAlmostEqual(1.5625, state["robots"]["r1"]["consumed_wh"])
        self.assertEqual("fresh", runtime.projection("stream", "r1", 125.25)["freshness"])
        self.assertEqual("stale", runtime.projection("stream", "r1", 125.2501)["freshness"])
        self.assertLess(len(json.dumps(state)), len(json.dumps(shape)) + 100)

    def test_optional_virtual_source_requires_opt_in_and_exclusive_owner(self) -> None:
        owner = object()
        with self.assertRaises(ValueError):
            self.runtime.claim_virtual_charging_source(owner, source=GEOMETRIC_CONTACT_SOURCE)
        with self.assertRaises(ValueError):
            EnergyRuntime("run", self.profile, {"r1": .5}, virtual_charging_source="hardware")
        runtime = EnergyRuntime("run", self.profile, {"r1": .5},
                                virtual_charging_source=GEOMETRIC_CONTACT_SOURCE)
        runtime.claim_virtual_charging_source(owner, source=GEOMETRIC_CONTACT_SOURCE)
        before = runtime.snapshot()
        with self.assertRaises(ValueError):
            runtime.claim_virtual_charging_source(object(), source=GEOMETRIC_CONTACT_SOURCE)
        with self.assertRaises(ValueError):
            runtime.observe("run", "r1", 1., 0., 0., loaded=False)
        with self.assertRaises(ValueError):
            runtime.observe_virtual_charging(object(), "run", "r1", 1., 0., 0.,
                loaded=False, geometric_dock_confirmed=True)
        self.assertEqual(before, runtime.snapshot())
        self.assertEqual(GEOMETRIC_CONTACT_SOURCE, before["contact_source"])

    def test_virtual_source_retains_two_confirmed_endpoints_accounting(self) -> None:
        runtime = EnergyRuntime("run", self.profile, {"r1": .5},
                                virtual_charging_source=GEOMETRIC_CONTACT_SOURCE)
        owner = object()
        runtime.claim_virtual_charging_source(owner, source=GEOMETRIC_CONTACT_SOURCE)
        for stamp, confirmed in ((0., True), (.2, False), (.4, True), (.6, True)):
            runtime.observe_virtual_charging(owner, "run", "r1", stamp, 0., 0.,
                loaded=False, geometric_dock_confirmed=confirmed)
        state = runtime.snapshot()["robots"]["r1"]
        self.assertAlmostEqual(.03, state["charged_wh"])
        self.assertAlmostEqual(.0075, state["consumed_wh"])
        self.assertAlmostEqual(300.0225, state["energy_wh"])


if __name__ == "__main__":
    unittest.main()
