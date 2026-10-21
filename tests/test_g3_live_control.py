import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any
import unittest

from factory_ops.fleet_execution import StoppedGate
from factory_ops.live_control import LiveControl
from factory_ops.navigation import GoalPose
from scripts.ros.g3_mission import (StopOdometryReceiver, dashboard_state, finish_stopped_goal, live_anchor_stopped)
from factory_ops.dashboard import DashboardService

ROOT = Path(__file__).resolve().parents[1]
RUN = "a"*32


class G3LiveControlTests(unittest.TestCase):
    def evidence(self):
        receiver = StopOdometryReceiver(StoppedGate(window_s=.5, max_linear_m_s=.02, max_angular_rad_s=.03, ttl_s=.3))
        for stamp in (1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7):
            receiver.observe(0., 0., stamp_s=stamp, stop_after_s=1., now_s=stamp)
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1700000000)))
        rt = {"phase": "idle", "generation": 3, "follow_uuid": "goal", "held_tiles": frozenset({(1, 1)}),
              "pose": {"x_m": 1., "y_m": 1., "yaw_rad": 0., "stamp_s": 1.7},
              "node": node, "live_stop_receiver": receiver, "live_hold_min_sequence": 20,
              "last_authorization": {"mode": "hold", "run_id": RUN, "issued_sim_time_s": 1.6, "valid_until_sim_time_s": 2.1}}
        hold = {"generation": 3, "goal_uuid": "goal", "accepted_goal_uuid": None, "tiles": [[1, 1]]}
        ack = {"run_id": RUN, "scenario_id": "normal", "robot_id": "carter1", "mode": "hold", "valid": True,
               "authorization_seq": 20, "scope": {"reservation_id": 3, "goal_uuid": "goal"},
               "physics_frame": 102, "evaluation_sim_time_s": 1.7, "input_command": {"linear_m_s": 0., "angular_rad_s": 0.}}
        limits = {"pose_ttl_s": .5, "station_xy_tolerance_m": .25, "station_yaw_tolerance_rad": .25, "authorization_ttl_s": .5}
        return rt, hold, ack, limits

    def check(self, rt, hold, ack, limits):
        return live_anchor_stopped("carter1", rt, hold, goal=GoalPose("map", 1., 1., 0.), now_s=1.7,
            pause_after_s=1., run_id=RUN, scenario_id="normal", ack=ack, limits=limits)

    def test_all_stop_uses_real_existing_window_pose_hold_and_ack(self):
        rt, hold, ack, limits = self.evidence()
        before = deepcopy(hold)
        self.assertTrue(self.check(rt, hold, ack, limits))
        self.assertEqual(hold, before)
        for phase in ("following", "compute_sent", "computing", "follow_sent", "stopping"):
            rt["phase"] = phase
            self.assertFalse(self.check(rt, hold, ack, limits))
        for phase in ("request", "dwell", "charging", "idle"):
            rt["phase"] = phase
            self.assertTrue(self.check(rt, hold, ack, limits))

    def test_stale_pose_wrong_hold_old_ack_and_moving_odom_cannot_ack(self):
        for defect in ("pose", "hold", "ack", "odom", "lease"):
            rt, hold, ack, limits = self.evidence()
            if defect == "pose": rt["pose"]["stamp_s"] = 1.
            if defect == "hold": hold["generation"] = 4
            if defect == "ack": ack["evaluation_sim_time_s"] = 1.1
            if defect == "lease": rt["last_authorization"]["valid_until_sim_time_s"] = 1.6
            if defect == "odom":
                rt["live_stop_receiver"].reset()
                rt["live_stop_receiver"].observe(.1, 0., stamp_s=1.7, stop_after_s=1., now_s=1.7)
            self.assertFalse(self.check(rt, hold, ack, limits), defect)

    def test_current_stop_finishes_and_retains_space_while_control_blocked(self):
        rt, hold, ack, limits = self.evidence()
        gate = LiveControl(RUN, tuple("carter%d" % i for i in range(1, 5)))
        gate.phase = "pause_pending"
        rt.update(phase="stopping", stop_after=1., stop_receiver=rt["live_stop_receiver"],
                  chunk=SimpleNamespace(goal_pose=GoalPose("map", 1., 1., 0.)))
        calls = []
        manager = SimpleNamespace(mark_goal_finished=lambda *a, **k: (calls.append((a, k)) or SimpleNamespace(granted=True)))
        held = rt["held_tiles"]
        result = finish_stopped_goal(manager, "carter1", rt, local_now_s=1.7, now_s=1.7, xy_limit=.25, yaw_limit=.25)
        self.assertIsNotNone(result)
        self.assertEqual(rt["held_tiles"], held)
        self.assertEqual(len(calls), 1)
        self.assertTrue(gate.blocked)

    def test_actual_new_leg_hook_defers_without_mutating_retained_leg(self):
        tree = ast.parse((ROOT/"scripts/ros/g3_mission.py").read_text(encoding="utf-8"))
        node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "new_leg")
        gate = LiveControl(RUN, tuple("carter%d" % i for i in range(1, 5)))
        gate.phase = "pause_pending"
        rt = {"leg": {"route": "current"}, "held_tiles": frozenset({(1, 1)}), "generation": 3}
        namespace = {"cutoff": None, "robots": {"carter1": rt}, "live_control": gate, "Any": Any}
        exec(compile(ast.Module(body=[node], type_ignores=[]), "actual_g3_new_leg_hook", "exec"), namespace)
        namespace["new_leg"]("carter1", "P1")
        self.assertEqual(rt["leg"], {"route": "current"})
        self.assertEqual(rt["held_tiles"], frozenset({(1, 1)}))
        self.assertEqual(rt["generation"], 3)
        self.assertEqual(rt["live_deferred_leg"], ("P1", None))

    def test_default_readonly_and_opted_in_stable_control_revision(self):
        service = DashboardService(ROOT/"configs/factory-v4")
        ledger = service.model.ledger.snapshot()
        gate = LiveControl(ledger["run_id"], tuple(ledger["robots"]))
        ordinary = dashboard_state(ledger, {}, sim_time_s=0., absolute_sim_time_s=0., pose_ttl_s=.5)
        self.assertEqual(ordinary["capabilities"], [])
        self.assertNotIn("live_control", ordinary)
        state = dashboard_state(ledger, {}, sim_time_s=0., absolute_sim_time_s=0., pose_ttl_s=.5, control=gate)
        ledger["ledger_seq"] += 20
        state2 = dashboard_state(ledger, {}, sim_time_s=2., absolute_sim_time_s=2., pose_ttl_s=.5, control=gate)
        self.assertEqual(state["revision"], state2["revision"])
        self.assertEqual(state2["live_control"]["ledger_revision"]-state["live_control"]["ledger_revision"], 20)
        gate.close("mission_ended_or_failed")
        closed = dashboard_state(ledger, {}, sim_time_s=2., absolute_sim_time_s=2., pose_ttl_s=.5, control=gate)
        self.assertEqual(closed["connection"], "disconnected")
        self.assertEqual(closed["capabilities"], [])


if __name__ == "__main__":
    unittest.main()
