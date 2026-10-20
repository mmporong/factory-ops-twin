"""CPU evidence-gate tests only; no ROS nodes, contact sensors or SDK runs."""
from __future__ import annotations

import json
from pathlib import Path
import unittest
from unittest.mock import patch

from factory_ops.energy_runtime import EnergyRuntime, GEOMETRIC_CONTACT_SOURCE
from factory_ops.ros_charging import RosChargingRuntime


class RosChargingTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((Path(__file__).resolve().parents[1] /
            "configs/factory-fleet4/energy-profile.json").read_text(encoding="utf-8"))
        self.build()

    def build(self, *, resume_soc=.8):
        self.energy = EnergyRuntime("run", self.profile, {"r1": .5, "r2": .8},
                                    virtual_charging_source=GEOMETRIC_CONTACT_SOURCE)
        self.options = {"contact_dwell_s": .3, "position_tolerance_m": .1,
                        "yaw_tolerance_rad": .1, "resume_soc": resume_soc}
        self.docks = {"CH1": {"anchor": "C1", "x_m": 2., "y_m": 3., "yaw_rad": 0.}}
        self.runtime = RosChargingRuntime(self.energy, self.docks, **self.options)
        self.runtime.observe("run", "r1", .8, 0., 0., loaded=False, observed_at_s=.8)

    def pose(self, stamp, **changes):
        return {"run_id": "run", "robot_id": "r1", "frame_id": "map", "x_m": 2.,
                "y_m": 3., "yaw_rad": 0., "stamp_s": stamp, **changes}

    def reserve(self):
        return self.runtime.reserve("run", "r1", carrying_payload=False, observed_at_s=.9)

    def arrive(self):
        return self.runtime.arrived("run", "r1", dock_id="CH1", anchor="C1", goal_id="1" * 32,
            navigation_succeeded=True, stamp_s=1., observed_at_s=1.)

    def sample(self, stamp, **changes):
        arguments = {"loaded": False, "observed_at_s": stamp, "dock_id": "CH1", "pose": self.pose(stamp)}
        arguments.update(changes)
        return self.runtime.observe("run", "r1", stamp, 0., 0., **arguments)

    def charged(self):
        return self.energy.snapshot()["robots"]["r1"]["charged_wh"]

    def begin_charging(self):
        self.reserve()
        self.arrive()
        for stamp in (1., 1.2, 1.4, 1.6):
            self.sample(stamp)

    def test_real_successful_arrival_alone_never_engages_or_charges(self):
        self.reserve()
        # Being geometrically inside the pad without a successful action is insufficient.
        self.sample(1.)
        self.assertEqual("approach", self.runtime.snapshot("run", observed_at_s=1.)["robots"]["r1"]["charging"]["phase"])
        self.arrive()
        for stamp in (1.2, 1.4, 1.6):
            self.sample(stamp, pose=None)
        for stamp in (1.8, 2., 2.2):
            self.sample(stamp, pose=self.pose(stamp, x_m=2.2))
        for stamp in (2.4, 2.6, 2.8):
            self.sample(stamp, pose=self.pose(stamp, yaw_rad=.2))
        state = self.runtime.snapshot("run", observed_at_s=2.8)["robots"]["r1"]["charging"]
        self.assertEqual("contact_pending", state["phase"])
        self.assertFalse(state["geometric_dock_confirmed"])
        self.assertEqual(0., self.charged())

    def test_stationary_dwell_and_exact_two_sided_soc_increment(self):
        self.reserve()
        self.arrive()
        self.sample(1.)
        self.sample(1.2)
        state = self.sample(1.4)
        self.assertEqual("charging", state["charging"]["phase"])
        self.assertEqual(0., self.charged())
        state = self.sample(1.6)
        balance = self.energy.snapshot()["robots"]["r1"]
        self.assertAlmostEqual(.03, balance["charged_wh"])
        self.assertAlmostEqual(.01, balance["consumed_wh"])
        self.assertAlmostEqual(300.02, balance["energy_wh"])
        self.assertEqual("simulated_engaged", state["charging"]["virtual_connector_state"])
        self.assertEqual(GEOMETRIC_CONTACT_SOURCE, state["charging"]["contact_source"])
        self.assertFalse(state["charging"]["physical_contact_verified"])

    def test_exact_repeats_do_not_observe_energy_twice_or_advance_dwell(self):
        self.reserve()
        self.arrive()
        self.sample(1.)
        self.sample(1.2)
        self.sample(1.4)
        with patch.object(self.energy, "observe_virtual_charging", wraps=self.energy.observe_virtual_charging) as observe:
            first = self.sample(1.6)
            before = self.energy.snapshot()
            self.assertEqual(first, self.sample(1.6))
            self.assertEqual(before, self.energy.snapshot())
            self.assertEqual(1, observe.call_count)
        with self.assertRaises(ValueError):
            self.runtime.observe("run", "r1", 1.6, -.1, 0., loaded=False,
                observed_at_s=1.6, dock_id="CH1", pose=self.pose(1.6))
        self.sample(1.8)
        self.assertAlmostEqual(.03, self.charged())

    def test_owner_run_goal_and_dock_identity_are_exclusive(self):
        with self.assertRaises(ValueError):
            RosChargingRuntime(self.energy, self.docks, **self.options)
        self.reserve()
        self.assertIsNone(self.runtime.reserve("run", "r2", carrying_payload=False, observed_at_s=.9))
        before = self.runtime.snapshot("run", observed_at_s=1.)
        for run, robot, dock in (("other", "r1", "CH1"), ("run", "r2", "CH1"), ("run", "r1", "CH2")):
            with self.subTest(run=run, robot=robot, dock=dock), self.assertRaises(ValueError):
                self.runtime.observe(run, robot, 1., 0., 0., loaded=False,
                    observed_at_s=1., dock_id=dock, pose=self.pose(1.))
        self.assertEqual(before, self.runtime.snapshot("run", observed_at_s=1.))
        for goal, succeeded, anchor, stamp in (("", True, "C1", 1.), ("0" * 32, True, "C1", 1.),
                ("1" * 32, False, "C1", 1.), ("1" * 32, True, "C2", 1.),
                ("1" * 32, True, "C1", .8)):
            with self.subTest(goal=goal, anchor=anchor), self.assertRaises(ValueError):
                self.runtime.arrived("run", "r1", dock_id="CH1", anchor=anchor, goal_id=goal,
                    navigation_succeeded=succeeded, stamp_s=stamp, observed_at_s=1.)
        self.arrive()
        with self.assertRaises(ValueError):
            self.energy.observe("run", "r1", 1., 0., 0., loaded=False)

    def test_bad_or_moving_samples_never_credit_or_bridge_uncertain_interval(self):
        cases = [
            {"pose": self.pose(1.7, run_id="other")}, {"pose": self.pose(1.7, robot_id="r2")},
            {"pose": self.pose(1.7, frame_id="odom")}, {"pose": self.pose(1.7, x_m=float("nan"))},
            {"pose": self.pose(1.)}, {"pose": self.pose(1.9)},
            {"stamp_s": 1.9}, {"stamp_s": 1.2}, {"linear_m_s": float("inf")},
            {"linear_m_s": -.1}, {"angular_rad_s": -.1}, {"loaded": True},
        ]
        for changes in cases:
            with self.subTest(changes=changes):
                self.build()
                self.begin_charging()
                before = self.charged()
                arguments = {"stamp_s": 1.7, "linear_m_s": 0., "angular_rad_s": 0.,
                    "loaded": False, "observed_at_s": 1.7, "dock_id": "CH1", "pose": self.pose(1.7)}
                arguments.update(changes)
                try:
                    self.runtime.observe("run", "r1", **arguments)
                except ValueError:
                    pass  # Rejected evidence is expected; balances and next interval are the assertions.
                self.assertEqual(before, self.charged())
                self.sample(1.8)
                self.assertEqual(before, self.charged())
                self.assertFalse(self.runtime.snapshot("run", observed_at_s=1.8)["robots"]["r1"]["charging"]["geometric_dock_confirmed"])

    def test_resume_stops_charge_and_requires_simulated_disengagement_before_release(self):
        self.build(resume_soc=.50002)
        self.begin_charging()
        state = self.runtime.snapshot("run", observed_at_s=1.6)["robots"]["r1"]["charging"]
        self.assertEqual("undock_pending", state["phase"])
        self.assertEqual("simulated_disengaging", state["virtual_connector_state"])
        with self.assertRaises(ValueError):
            self.runtime.release("run", "r1", dock_id="CH1", observed_at_s=1.6)
        charged = self.charged()
        moving = self.runtime.observe("run", "r1", 1.8, -.05, 0., loaded=False,
            observed_at_s=1.8, dock_id="CH1", pose=self.pose(1.8))
        self.assertEqual("undock_pending", moving["charging"]["phase"])
        state = self.sample(2.)
        self.assertEqual("ready_to_leave", state["charging"]["phase"])
        self.assertEqual(charged, self.charged())
        released = self.runtime.release("run", "r1", dock_id="CH1", observed_at_s=2.)
        self.assertFalse(released["body_occupancy_released"])
        self.assertTrue(released["charging_reservation_released"])
        self.assertEqual({}, self.runtime.snapshot("run", observed_at_s=2.)["owners"])
        json.dumps(self.runtime.snapshot("run", observed_at_s=2.), allow_nan=False)
        self.runtime.reserve("run", "r1", carrying_payload=False, observed_at_s=2.)
        with self.assertRaises(ValueError):
            self.runtime.arrived("run", "r1", dock_id="CH1", anchor="C1", goal_id="1" * 32,
                navigation_succeeded=True, stamp_s=2., observed_at_s=2.)


if __name__ == "__main__":
    unittest.main()
