"""Operations charging adapter contract without coupling to OperationsModel internals."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from factory_ops.drive_profile import DriveLimits
from factory_ops.fleet_routing import RouteBook
from factory_ops.motion import DriveGeometry
from factory_ops.operations_charging import OperationsCharging
from factory_ops.operations_motion import PathMotion


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "factory-semiconductor-dual-lane"


class _Inventory:
    def snapshot(self):
        return {"robot_loads": {}}


class FakeOperationsModel:
    def __init__(self) -> None:
        self.book = RouteBook.from_files(CONFIG / "layout.json", CONFIG / "g3-routing.json")
        drive = json.loads((CONFIG / "g2.json").read_text(encoding="utf-8"))
        self.geometry = DriveGeometry(**{k: v for k, v in drive["drive_geometry"].items() if k != "provenance"})
        self.limits = DriveLimits(**{k: v for k, v in drive["drive_profile"].items() if k != "provenance"})
        robots = {f"amr-{i}": SimpleNamespace(payload_id=None) for i in range(1, 5)}
        self.ledger = SimpleNamespace(run_id="adapter-test", robots=robots)
        self.inventory = _Inventory()
        self.closed = False
        self.sim_time = 0.0
        self.homes = {f"amr-{i}": f"P{i}" for i in range(1, 5)}
        self._locations = dict(self.homes)
        self._poses = {
            robot: {"x_m": self.book.nodes[home][0], "y_m": self.book.nodes[home][1], "yaw_rad": 0.0}
            for robot, home in self.homes.items()
        }
        self.motions = {robot: PathMotion(self.limits, self.geometry) for robot in robots}
        self._trips: dict[str, dict] = {}
        self._wait_s = {robot: 0.0 for robot in robots}
        self._deliveries = {robot: 0 for robot in robots}
        self.departure_requirement = None
        self.events: list[dict] = []

    def _trip(self, robot_id, task, start, goal, phase, *, kind="order"):
        self._trips[robot_id] = {
            "task": task,
            "start": start,
            "goal": goal,
            "phase": phase,
            "kind": kind,
            "points": deque(self.book.plan(start, goal, closed=self.closed).points[1:]),
        }

    def _align_at_hold(self, robot, yaw_rad, dt):
        return self.motions[robot].rotate_to(self._poses[robot], yaw_rad, dt)

    def _event(self, kind, message, robot, **details):
        self.events.append({"type": kind, "message": message, "robot_id": robot, **details})

    def _charging_departure_requirement(self, robot_id):
        return self.departure_requirement


class OperationsChargingAdapterTests(unittest.TestCase):
    def profile(self):
        return json.loads((CONFIG / "operations-charging.json").read_text(encoding="utf-8"))

    def low_third_robot(self):
        model = FakeOperationsModel()
        profile = self.profile()
        profile["initial_soc"]["amr-3"] = 0.19
        adapter = OperationsCharging(model, profile)
        return model, adapter

    def test_dock_home_is_exchanged_and_actual_occupant_must_vacate_before_reservation(self):
        model, adapter = self.low_third_robot()
        self.assertEqual(["amr-3"], adapter.queue)
        self.assertFalse(adapter.docks.owners)
        self.assertEqual("P1", model.homes["amr-3"])
        self.assertEqual("P3", model.homes["amr-1"])
        self.assertEqual(4, len(set(model.homes.values())))
        self.assertNotIn("amr-3", model._trips)
        self.assertFalse(adapter.available("amr-1"))

        model._locations["amr-1"] = "P3"
        model._poses["amr-1"].update(x_m=model.book.nodes["P3"][0], y_m=model.book.nodes["P3"][1])
        adapter.prepare(model)
        self.assertTrue(adapter.available("amr-1"))
        self.assertEqual("amr-3", adapter.docks.owners["CH1"])
        self.assertEqual("charging", model._trips["amr-3"]["kind"])
        self.assertEqual("charge_approach", model._trips["amr-3"]["phase"])
        self.assertEqual("P1", model._trips["amr-3"]["goal"])
        self.assertFalse(adapter.can_park("amr-3"))

    def test_assignment_budget_includes_pickup_drop_charger_motion_wait_and_reserve(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())
        adapter.energy.batteries["amr-4"].energy_wh = 600.0 * 0.151
        self.assertFalse(adapter.can_assign(model, "amr-4", "S1", "CI"))
        self.assertIn("amr-4", adapter.denied)
        self.assertTrue(adapter.energy.can_assign(
            "amr-4", outbound_m=0.0, charger_return_m=0.0, expected_wait_s=0.0,
            nominal_speed_m_s=model.limits.max_linear_m_s, loaded=True, observed_at_s=0.0,
        ))

    def test_completed_transport_wait_extends_the_next_assignment_budget(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())
        base = adapter._assignment_required_wh(model, "amr-4", "S1", "CI")
        model._wait_s["amr-1"] = 9588.8
        model._deliveries["amr-1"] = 1

        adapter.prepare(model)

        required = adapter._assignment_required_wh(model, "amr-4", "S1", "CI")
        expected_extra = adapter.energy.profile["idle_power_w"] * 9588.8 / 3600.0
        self.assertAlmostEqual(9588.8, adapter.traffic_wait_budget_s)
        self.assertAlmostEqual(base + expected_extra, required)
        adapter.energy.batteries["amr-4"].energy_wh = required - 0.01
        self.assertFalse(adapter.can_assign(model, "amr-4", "S1", "CI"))

    def test_assignment_requirement_cache_tracks_route_inputs_without_soc_churn(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())

        with patch.object(adapter, "_plan", wraps=adapter._plan) as plan:
            base = adapter._assignment_required_wh(model, "amr-4", "S1", "CI")
            first_count = plan.call_count

            adapter.energy.batteries["amr-4"].energy_wh -= 1.0
            self.assertEqual(base, adapter._assignment_required_wh(model, "amr-4", "S1", "CI"))
            self.assertEqual(first_count, plan.call_count)

            adapter.traffic_wait_budget_s = 30.0
            waited = adapter._assignment_required_wh(model, "amr-4", "S1", "CI")
            self.assertGreater(waited, base)
            self.assertEqual(first_count, plan.call_count)
            wait_count = plan.call_count

            model._locations["amr-4"] = "P3"
            x, y = model.book.nodes["P3"]
            model._poses["amr-4"].update(x_m=x, y_m=y)
            adapter._assignment_required_wh(model, "amr-4", "S1", "CI")
            self.assertGreater(plan.call_count, wait_count)

    def test_in_progress_transport_wait_funds_new_work_before_delivery_completes(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())
        model._wait_s["amr-1"] = 8385.1

        adapter.prepare(model)

        self.assertEqual(0, model._deliveries["amr-1"])
        self.assertAlmostEqual(8385.1, adapter.traffic_wait_budget_s)

    def test_queued_parking_trip_is_not_replaced_by_charge_approach(self):
        model, adapter = self.low_third_robot()
        model._locations["amr-1"] = "P3"
        model._poses["amr-1"].update(x_m=model.book.nodes["P3"][0], y_m=model.book.nodes["P3"][1])
        model._trip("amr-3", {}, "P3", "P1", "parking", kind="parking")
        accepted = model._trips["amr-3"]
        adapter.prepare(model)
        self.assertIs(accepted, model._trips["amr-3"])
        self.assertFalse(adapter.docks.owners)

    def test_home_swap_does_not_relocate_a_peer_into_an_owned_dock_anchor(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())
        self.assertEqual("CH1", adapter.docks.reserve("amr-4", carrying_payload=False))
        adapter.energy.batteries["amr-1"].energy_wh = 600.0 * 0.19
        original = dict(model.homes)
        adapter.prepare(model)
        self.assertEqual(["amr-1"], adapter.queue)
        self.assertEqual(original, model.homes)
        self.assertNotIn("CH2", adapter.docks.owners)

    def test_observed_kinematic_motion_consumes_energy_without_charging(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())
        before = adapter.energy.snapshot("amr-2")
        model.motions["amr-2"].linear_m_s = 0.5
        model.sim_time = 0.1
        adapter.record(model, 0.1)
        after = adapter.energy.snapshot("amr-2")
        self.assertGreater(after["consumed_wh"], before["consumed_wh"])
        self.assertEqual(0.0, after["charged_wh"])

    def test_contact_dwell_charge_actual_exit_and_stationary_dwell_precede_release(self):
        model, adapter = self.low_third_robot()
        model._locations["amr-1"] = "P3"
        model._poses["amr-1"].update(x_m=model.book.nodes["P3"][0], y_m=model.book.nodes["P3"][1])
        adapter.prepare(model)
        trip = model._trips["amr-3"]
        dock_xy = model.book.nodes["P1"]
        model._poses["amr-3"].update(x_m=dock_xy[0], y_m=dock_xy[1])
        self.assertTrue(adapter.arrive(model, "amr-3", trip))
        self.assertEqual("aligning", adapter.aux["amr-3"]["phase"])

        for _ in range(100):
            model.sim_time += 0.1
            adapter.record(model, 0.1)
            if adapter.aux["amr-3"]["phase"] == "contact_pending":
                self.assertEqual(0.0, adapter.energy.snapshot("amr-3")["charged_wh"])
            if adapter.docks.states["amr-3"].contact_confirmed:
                break
        else:
            self.fail("authored alignment and contact dwell did not complete")

        # The observed traffic budget can keep confirmed contact beyond the
        # authored 35 percent threshold without changing the profile.
        model.departure_requirement = {"required_wh": 700.0, "task_id": "blocked-by-capacity"}
        adapter.energy.batteries["amr-3"].energy_wh = 600.0
        for _ in range(3):
            model.sim_time += 0.1
            adapter.record(model, 0.1)
        self.assertNotEqual("exiting", adapter.aux["amr-3"]["phase"])
        self.assertEqual(
            "assignment_funding_exceeds_capacity",
            adapter.snapshot()["departure_deferred"]["amr-3"]["reason"],
        )

        # Bounded fixture override keeps the lifecycle test short; charging still
        # requires two confirmed stationary contact samples in EnergyFleet.
        model.departure_requirement = {"required_wh": 300.0, "task_id": "next-transport"}
        adapter.energy.batteries["amr-3"].energy_wh = 600.0 * adapter.profile["resume_soc"] - 0.01
        for _ in range(10):
            model.sim_time += 0.1
            adapter.record(model, 0.1)
        self.assertNotEqual("exiting", adapter.aux["amr-3"]["phase"])
        self.assertEqual("observed_traffic_funding", adapter.snapshot()["departure_deferred"]["amr-3"]["reason"])

        adapter.energy.batteries["amr-3"].energy_wh = 300.0 - 0.01
        for _ in range(10):
            model.sim_time += 0.1
            adapter.record(model, 0.1)
            if adapter.aux["amr-3"]["phase"] == "exiting":
                break
        else:
            self.fail("funded virtual contact did not start exit")
        self.assertGreater(adapter.energy.snapshot("amr-3")["charged_wh"], 0.0)
        self.assertEqual("amr-3", adapter.docks.owners["CH1"])

        exit_trip = model._trips["amr-3"]
        # Keep the 3 m reserved exit leg in flight longer than max_gap. Dock
        # observations must continue without releasing the owner while moving.
        start_x, start_y = model.book.nodes["P1"]
        exit_x, exit_y = model.book.nodes["LOW_E_P1"]
        model.motions["amr-3"].linear_m_s = 0.2
        for step in range(1, 11):
            fraction = step / 12
            model._poses["amr-3"].update(
                x_m=start_x + (exit_x - start_x) * fraction,
                y_m=start_y + (exit_y - start_y) * fraction,
            )
            model.sim_time += 0.1
            adapter.record(model, 0.1)
            self.assertNotIn("amr-3", adapter.faults)
            self.assertEqual("amr-3", adapter.docks.owners["CH1"])
            self.assertIs(exit_trip, model._trips["amr-3"])

        exit_xy = model.book.nodes["LOW_E_P1"]
        model._poses["amr-3"].update(x_m=exit_xy[0], y_m=exit_xy[1])
        model.motions["amr-3"].linear_m_s = 0.0
        self.assertTrue(adapter.arrive(model, "amr-3", exit_trip))
        for _ in range(3):
            model.sim_time += 0.1
            adapter.record(model, 0.1)
            self.assertEqual("amr-3", adapter.docks.owners["CH1"])
        for _ in range(3):
            model.sim_time += 0.1
            adapter.record(model, 0.1)
            if "CH1" not in adapter.docks.owners:
                break
        self.assertNotIn("CH1", adapter.docks.owners)
        self.assertNotIn("amr-3", adapter.aux)
        self.assertFalse(adapter.pad_contact(model, "amr-3", "CH1"))
        self.assertTrue(adapter.can_park("amr-3"))

    def test_stale_observation_fault_freezes_energy_instead_of_fabricating_soc(self):
        model = FakeOperationsModel()
        adapter = OperationsCharging(model, self.profile())
        before = deepcopy(adapter.energy.snapshot("amr-1"))
        model.sim_time = 1.0
        adapter.record(model, 0.1)
        self.assertIn("amr-1", adapter.faults)
        self.assertEqual(before, adapter.energy.snapshot("amr-1"))
        model.sim_time = 1.1
        adapter.record(model, 0.1)
        self.assertEqual(before, adapter.energy.snapshot("amr-1"))


if __name__ == "__main__":
    unittest.main()
