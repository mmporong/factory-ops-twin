"""Optional CPU charge cycles; no physics, ROS or hardware contact evidence."""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from factory_ops.dashboard import validate_snapshot
from factory_ops.dashboard_model import DashboardModel


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v4"


class DashboardChargingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = json.loads((ROOT / "configs/web-charging-model.json").read_text(encoding="utf-8"))

    def model(self, profile=None) -> DashboardModel:
        return DashboardModel(CONFIG, charging_profile=profile or self.profile)

    def until_phase(self, model, phase, *, step=.1, limit=6000):
        for _ in range(limit):
            if model.charging.robot_snapshot("amr-4")["charging_phase"] == phase:
                return
            model.advance(step)
        self.fail(f"CPU charge phase did not reach {phase}: {model.snapshot()['charging']}")

    def assert_frozen(self, model) -> None:
        model.paused = True
        before = model.snapshot()
        balances = deepcopy({r: model.charging.energy.snapshot(r) for r in model.ledger.robots})
        model.advance(1.)
        after = model.snapshot()
        before.pop("observed_at")
        after.pop("observed_at")
        self.assertEqual(before, after)
        self.assertEqual(balances, {r: model.charging.energy.snapshot(r) for r in model.ledger.robots})
        model.paused = False
        before = model.charging.energy.snapshot("amr-4")
        model.advance(0.)
        self.assertEqual(before, model.charging.energy.snapshot("amr-4"))

    def test_default_low_soc_route_dwell_charge_undock_and_return_to_work(self) -> None:
        model = self.model()
        assert model.charging is not None
        charging = model.charging
        initial = dict(model._poses["amr-4"])
        self.assertEqual("approach", charging.robot_snapshot("amr-4")["charging_phase"])
        self.assert_frozen(model)
        # Run every public .1s approach step: displacement/yaw are bounded, not a teleport.
        previous = initial
        while charging.robot_snapshot("amr-4")["charging_phase"] != "contact_pending":
            model.advance(.1)
            pose = dict(model._poses["amr-4"])
            self.assertLessEqual(math.hypot(pose["x_m"]-previous["x_m"], pose["y_m"]-previous["y_m"]), .065 + 1e-9)
            self.assertEqual(0., charging.energy.snapshot("amr-4")["charged_wh"])
            previous = pose
            self.assertLess(model.sim_time, 200.)
        at_arrival = model.sim_time
        self.until_phase(model, "charging")
        self.assertGreaterEqual(model.sim_time-at_arrival, self.profile["contact_dwell_s"])
        self.assertEqual(0., charging.energy.snapshot("amr-4")["charged_wh"])
        self.assert_frozen(model)
        model.advance(.1)
        self.assertGreater(charging.energy.snapshot("amr-4")["charged_wh"], 0.)
        self.until_phase(model, "undocking", step=1.)
        self.assert_frozen(model)
        dock = charging.robot_snapshot("amr-4")["charging_dock"]
        slot = charging.slots[dock]
        while charging.robot_snapshot("amr-4")["charging_phase"] is not None:
            self.assertEqual("amr-4", charging.docks.owners[dock])
            self.assertEqual("amr-4", charging.exit_holds[slot["exit_anchor"]])
            model.advance(.1)
            self.assertLess(model.sim_time, 600.)
        self.assertNotIn(dock, charging.docks.owners)
        self.assertNotIn(slot["exit_anchor"], charging.exit_holds)
        exit_xy = model.book.nodes[slot["exit_anchor"]]
        self.assertLess(math.hypot(model._poses["amr-4"]["x_m"]-exit_xy[0],model._poses["amr-4"]["y_m"]-exit_xy[1]),1e-8)
        self.assertFalse(charging.pad_contact(model._poses["amr-4"], dock))
        for _ in range(700):
            model.advance(1.)
            state = validate_snapshot(model.snapshot())
            self.assertTrue(all(r["battery_source"] == "authored_cpu_virtual_energy_model" for r in state["robots"]))
            self.assertTrue(all(r["balanced"] for r in state["inventory"]["conservation"]))
            if state["kpis"]["completed_orders"] == 4 and state["kpis"]["pending_tasks"] == 0:
                break
        else:
            self.fail("CPU business/empty trips did not complete after charging")
        # Peers can now finish the backlog during charging; the released robot is dispatchable.
        self.assertTrue(charging.available("amr-4"))
        self.assertTrue(charging.can_assign(model, "amr-4", "S1", "CI"))
        self.assertTrue(all(t["state"] == "available" for t in state["inventory"]["totes"]))
        self.assertEqual(56, state["kpis"]["stock_units"])
        self.assertTrue(model.ledger.replay_matches())
        self.assertFalse(charging.faults)
        self.assertIn("no physics", state["model_scope"])
        for robot in model.ledger.robots:
            energy = charging.energy.snapshot(robot)
            self.assertAlmostEqual(600*self.profile["initial_soc"][robot], energy["energy_wh"]+energy["consumed_wh"]-energy["charged_wh"])

    def test_charging_does_not_hold_exit_or_block_default_s1_s2_dispatch(self) -> None:
        model = self.model()
        assert model.charging is not None
        self.until_phase(model, "charging")
        charging = model.charging
        self.assertFalse(charging.exit_holds)
        orders = []
        for source in ("S1", "S2"):
            model.create_order({"source": source})
            orders.append(f"ORD-{model._order_seq:04}")
        for _ in range(250):
            self.assertEqual("charging", charging.aux["amr-4"]["phase"])
            self.assertFalse(charging.exit_holds)
            assigned = [t for t in model.ledger.tasks.values() if t.order_id in orders and t.robot_id is not None]
            if {t.order_id for t in assigned} == set(orders):
                self.assertTrue(all(t.robot_id != "amr-4" for t in assigned))
                break
            model.advance(1.)
        else:
            self.fail("S1/S2 business orders were not assigned while CH1 was charging")

    def test_undock_waits_on_pad_until_current_business_or_empty_peer_leg_finishes(self) -> None:
        for kind in ("order", "empty_return"):
            with self.subTest(kind=kind):
                model = self.model()
                assert model.charging is not None
                charging = model.charging
                self.until_phase(model, "charging")
                slot = charging.slots[charging.aux["amr-4"]["dock_id"]]
                exit_xy = model.book.nodes[slot["exit_anchor"]]
                for _ in range(2500):
                    peer = next(((r, t) for r, t in model._trips.items()
                                 if t["kind"] == kind and t["phase"] == "drop" and exit_xy in t["points"]), None)
                    if peer is not None:
                        break
                    self.assertEqual("charging", charging.aux["amr-4"]["phase"])
                    model.advance(.1)
                else:
                    self.fail(f"no accepted {kind} leg crossing the CH1 exit")
                assert peer is not None
                peer_id, peer_leg = peer
                # Target-SOC fixture while an accepted business/empty leg is in flight.
                charging.energy.batteries["amr-4"].energy_wh = 600*self.profile["resume_soc"]
                pad_pose = dict(model._poses["amr-4"])
                charged = charging.energy.snapshot("amr-4")["charged_wh"]
                passed_exit_before_leg_end = False
                # Keep existing legs real; only hold new work for a bounded exit check.
                with patch.object(model, "_dispatch"):
                    for _ in range(2000):
                        if model._trips.get(peer_id) is not peer_leg:
                            break
                        model.advance(.1)
                        if model._trips.get(peer_id) is peer_leg:
                            self.assertEqual("undock_pending", charging.aux["amr-4"]["phase"])
                            self.assertEqual(pad_pose, model._poses["amr-4"])
                            self.assertNotIn(slot["exit_anchor"], charging.exit_holds)
                            self.assertEqual("amr-4", charging.docks.owners["CH1"])
                            self.assertEqual(charged, charging.energy.snapshot("amr-4")["charged_wh"])
                            passed_exit_before_leg_end |= exit_xy not in peer_leg["points"]
                    else:
                        self.fail("accepted peer leg did not finish")
                    self.assertTrue(passed_exit_before_leg_end)
                    self.until_phase(model, "undocking")
                    self.assertEqual("amr-4", charging.exit_holds[slot["exit_anchor"]])
                    self.assertEqual(pad_pose, model._poses["amr-4"])
                    model.advance(.1)
                    self.assertNotEqual(pad_pose, model._poses["amr-4"])

    def test_four_low_robots_queue_for_two_exclusive_docks_without_business_assignment(self) -> None:
        profile = deepcopy(self.profile)
        profile["initial_soc"] = {f"amr-{i}":.18 for i in range(1,5)}
        model = self.model(profile)
        assert model.charging is not None
        self.assertEqual(2, len(model.charging.queue))
        self.assertEqual(2, len(model.charging.docks.owners))
        for _ in range(1700):
            model.advance(1.)
            state = model.snapshot()
            owners = [d["owner"] for d in state["charging"]["docks"] if d["owner"] is not None]
            self.assertEqual(len(owners),len(set(owners)))
            self.assertLessEqual(len(owners),2)
            self.assertTrue(set(owners).isdisjoint(state["charging"]["queue"]))
            for robot in state["charging"]["queue"]:
                self.assertNotIn(robot, model._trips)
                self.assertEqual("queued", next(r for r in state["robots"] if r["id"] == robot)["charging_phase"])
            self.assertTrue(all(r["balanced"] for r in state["inventory"]["conservation"]))
            if all(model.charging.energy.snapshot(r)["charged_wh"] > 0 for r in model.ledger.robots) and not model.charging.aux and not model.charging.queue:
                break
        else:
            self.fail("four robots did not all leave the two-slot CPU queue")
        self.assertFalse(model.charging.faults)

    def test_accepted_loaded_trip_finishes_delivery_before_charging(self) -> None:
        profile = deepcopy(self.profile)
        profile["initial_soc"] = {f"amr-{i}":.85 for i in range(1,5)}
        model = self.model(profile)
        assert model.charging is not None
        for _ in range(100):
            model.advance(.1)
            loaded = next((r for r,t in model._trips.items() if t["kind"] == "order" and t["phase"] == "drop"), None)
            if loaded:
                break
        else:
            self.fail("no accepted product pickup")
        assert loaded is not None
        payload = model.ledger.robots[loaded].payload_id
        # Fault-injection fixture: an already accepted load now reports low virtual energy.
        model.charging.energy.batteries[loaded].energy_wh = 600*.18
        for _ in range(1000):
            if model.ledger.robots[loaded].payload_id is None:
                break
            self.assertEqual(payload,model.ledger.robots[loaded].payload_id)
            self.assertNotIn(loaded,model.charging.aux)
            self.assertNotIn(loaded,model.charging.queue)
            model.advance(.2)
        else:
            self.fail("loaded CPU trip did not deliver")
        self.assertTrue(loaded in model.charging.aux or loaded in model.charging.queue)
        self.assertTrue(all(r["balanced"] for r in model.snapshot()["inventory"]["conservation"]))

    def test_reserve_includes_full_task_and_charger_return_not_only_pickup(self) -> None:
        profile = deepcopy(self.profile)
        profile["initial_soc"] = {f"amr-{i}":.85 for i in range(1,5)}
        profile["initial_soc"]["amr-1"] = .21  # Above trigger, below full-trip+queue+reserve budget.
        profile["energy"]["reserve_fraction"] = .199
        model = self.model(profile)
        assert model.charging is not None
        self.assertFalse(model.charging.can_assign(model,"amr-1","S1","CI"))
        # Pickup distance alone is zero and affordable; the complete cost is not.
        self.assertTrue(model.charging.energy.can_assign("amr-1",outbound_m=0.,charger_return_m=0.,expected_wait_s=0.,
            nominal_speed_m_s=.65,loaded=True,observed_at_s=0.))
        model.advance(1.)
        self.assertNotIn("amr-1",model._trips)
        self.assertTrue("amr-1" in model.charging.aux or "amr-1" in model.charging.queue)

    def test_empty_return_is_not_assigned_without_its_full_energy_budget(self) -> None:
        profile = deepcopy(self.profile)
        profile["initial_soc"] = {f"amr-{i}":.85 for i in range(1,5)}
        profile["energy"]["reserve_fraction"] = .199
        model = self.model(profile)
        assert model.charging is not None
        model.advance(.1)
        # Build real empty-tote stock by finishing accepted trips, holding only new dispatch.
        with patch.object(model,"_dispatch"):
            for _ in range(200):
                model.advance(1.)
                if model._return_tasks and not model._trips:
                    break
        self.assertTrue(model._return_tasks)
        self.assertFalse(model._trips)
        for robot in model.ledger.robots:
            model.charging.energy.batteries[robot].energy_wh = 600*.21
        inventory = deepcopy(model.snapshot()["inventory"])
        model._dispatch()
        self.assertTrue(all(t["state"] == "ready" for t in model._return_tasks.values()))
        self.assertFalse(model._trips)
        self.assertEqual(inventory,model.snapshot()["inventory"])

    def test_no_charge_when_alignment_or_authored_geometric_contact_is_missing(self) -> None:
        model = self.model()
        assert model.charging is not None
        self.until_phase(model,"contact_pending")
        controller = model.charging
        dock = controller.aux["amr-4"]["dock_id"]
        # Separate authored geometry input fails even though route arrival succeeded.
        model._poses["amr-4"]["yaw_rad"] += .5
        for _ in range(10):
            model.advance(.1)
        self.assertFalse(controller.pad_contact(model._poses["amr-4"],dock))
        self.assertEqual(0.,controller.energy.snapshot("amr-4")["charged_wh"])
        self.assertEqual("contact_pending",controller.aux["amr-4"]["phase"])

    def test_recontact_during_undock_cannot_release_slot_or_exit_hold(self) -> None:
        model = self.model()
        assert model.charging is not None
        self.until_phase(model,"undocking",step=1.)
        controller = model.charging
        trip = controller.aux["amr-4"]
        dock, slot = trip["dock_id"],controller.slots[trip["dock_id"]]
        while trip["points"]:
            model.advance(.1)
        # Counterevidence fixture before the final stationary exit dwell.
        x,y = model.book.nodes[slot["anchor"]]
        model._poses["amr-4"].update(x_m=x,y_m=y,yaw_rad=slot["yaw_rad"])
        charged = controller.energy.snapshot("amr-4")["charged_wh"]
        for _ in range(10):
            model.advance(.1)
        self.assertEqual("amr-4",controller.docks.owners[dock])
        self.assertEqual("amr-4",controller.exit_holds[slot["exit_anchor"]])
        self.assertEqual("undock_pending",controller.docks.states["amr-4"].phase)
        self.assertEqual(charged,controller.energy.snapshot("amr-4")["charged_wh"])

    def test_public_one_second_steps_substep_below_gap_and_snapshot_is_detached(self) -> None:
        model = self.model()
        assert model.charging is not None
        for _ in range(10):
            model.advance(1.)
        self.assertFalse(model.charging.faults)
        self.assertAlmostEqual(10.,model.sim_time)
        for robot in model.ledger.robots:
            self.assertAlmostEqual(model.sim_time,model.charging.energy.snapshot(robot)["observed_sim_time_s"])
        snapshot = model.snapshot()
        snapshot["charging"]["docks"][0]["owner"] = "other"
        self.assertNotEqual("other",model.snapshot()["charging"]["docks"][0]["owner"])

    def test_motion_and_energy_ticks_preserve_command_cas_revision(self) -> None:
        profile = deepcopy(self.profile)
        profile["initial_soc"] = {f"amr-{i}":.18 for i in range(1,5)}
        model = self.model(profile)
        model.advance(.2)
        before = model.snapshot()
        model.advance(.1)
        after = model.snapshot()
        self.assertEqual(before["revision"],after["revision"])
        self.assertEqual(before["events"],after["events"])
        self.assertNotEqual(before["robots"][0]["pose"],after["robots"][0]["pose"])
        self.assertNotEqual(before["robots"][0]["battery_pct"],after["robots"][0]["battery_pct"])

    def test_legacy_mode_keeps_unavailable_soc_and_rejects_invalid_profiles(self) -> None:
        model = DashboardModel(CONFIG)
        model.advance(1.)
        state = model.snapshot()
        self.assertNotIn("charging",state)
        self.assertTrue(all(r["battery_pct"] is None and r["battery_source"] == "unavailable" for r in state["robots"]))
        for key,value in (("resume_soc",.1),("max_step_s",1.),("speed_m_s",0.)):
            profile = deepcopy(self.profile)
            profile[key] = value
            with self.assertRaises(ValueError):
                self.model(profile)


if __name__ == "__main__":
    unittest.main()
