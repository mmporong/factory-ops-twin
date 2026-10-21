import ast
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any
import unittest
import json
import tempfile
from unittest.mock import patch

from factory_ops.fleet_execution import StoppedGate
from factory_ops.live_control import LiveControl
from factory_ops.navigation import GoalPose
from scripts.ros.g3_mission import (StopOdometryReceiver, dashboard_state, finish_stopped_goal,
    live_anchor_stopped, record_live_anchor_diagnostic)
from factory_ops.dashboard import DashboardService

ROOT = Path(__file__).resolve().parents[1]
RUN = "a"*32


class G3LiveControlTests(unittest.TestCase):
    def test_wait_scene_keeps_exact_SDK_bootstrap_file_set(self):
        from factory_ops.fleet_transport import _compile, DEFAULT_CONFIG
        from scripts.isaac.fleet4_transport_scene import BOOTSTRAP_FILES
        from scripts.ros.g3_mission import run_mission
        compiled = _compile(DEFAULT_CONFIG)
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            session = folder/"inputs"
            output = folder/"run"
            session.mkdir(); output.mkdir()
            (session/"session.json").write_text(json.dumps(compiled["session"]), encoding="utf-8")
            # File-set fixture only; no SDK or source-verification success claim.
            for name in BOOTSTRAP_FILES-{"fleet-transport-verified.json"}:
                p = output/name
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(b"bootstrap file-set fixture")
            args = SimpleNamespace(output=output, run_id=RUN, dashboard=True,
                live_anchor_controls=True, runtime_profile="local45_development")
            observed = []
            def inspect_before_ready(_):
                observed.append({p.relative_to(output).as_posix() for p in output.rglob("*") if p.is_file()})
                raise RuntimeError("bounded unit stop before SDK ready")
            with patch.dict("os.environ", {"FACTORY_OPS_G3_SOURCE_COMMIT": "c"*40}), \
                    patch("scripts.ros.g3_mission.time.sleep", side_effect=inspect_before_ready):
                result = run_mission(args, session_root=session, session=compiled["session"],
                    config=compiled["config"], book=compiled["book"], registry=compiled["registry"],
                    bundle_sha256="b"*64)
            self.assertEqual(result, 1)
            self.assertEqual(len(observed), 1)
            self.assertEqual(observed[0], set(BOOTSTRAP_FILES))
            self.assertFalse((output/"commands").exists())
            self.assertFalse((output/"live-control-state.json").exists())

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

    def test_native_parked_accepted_binding_and_bounded_missing_condition_diagnostic(self):
        from factory_ops.route_reservation import RouteReservationManager
        ids = tuple("carter%d" % i for i in range(1,5))
        manager = RouteReservationManager(RUN, ids, pose_ttl_s=.5)
        for i, robot in enumerate(ids):
            manager.observe(robot, {(1+10*i,1)}, pose_stamp_s=1.7, now_s=1.7)
        # Native042 C1 goal identity; all timestamps/pose/odometry below are a
        # constructed CPU regression, not recovered native stop measurements.
        goal_uuid = "1feb23e1-a3fd-4128-8075-8e49f9fc1beb"
        grant = manager.reserve("carter1", "native042-parked", goal_uuid, {(1,1),(1,2)}, now_s=1.7)
        self.assertTrue(grant.granted)
        self.assertTrue(manager.mark_goal_accepted("carter1", grant.generation, goal_uuid, now_s=1.7).granted)
        self.assertTrue(manager.mark_goal_finished("carter1", grant.generation, goal_uuid, now_s=1.7).granted)
        before_shrink = manager.snapshot(now_s=1.7)["holds"]["carter1"]
        shrunk = manager.shrink_hold("carter1", grant.generation, {(1,1)}, stopped=True, now_s=1.7)
        self.assertTrue(shrunk.granted)
        hold = manager.snapshot(now_s=1.7)["holds"]["carter1"]
        self.assertEqual(hold["accepted_goal_uuid"], goal_uuid)
        self.assertTrue(hold["parked"])
        rt, _, ack, limits = self.evidence()
        rt.update(generation=hold["generation"], follow_uuid=goal_uuid, held_tiles=frozenset({(1,1)}), anchor="P1")
        ack["scope"].update(reservation_id=hold["generation"], goal_uuid=goal_uuid)
        untouched = deepcopy(hold)
        diagnostic = {}
        self.assertTrue(live_anchor_stopped("carter1", rt, hold, goal=GoalPose("map",1.,1.,0.), now_s=1.7,
            pause_after_s=1., run_id=RUN, scenario_id="normal", ack=ack, limits=limits, diagnostic=diagnostic))
        self.assertEqual(diagnostic["reason"], None)
        self.assertTrue(diagnostic["qualified"])
        self.assertEqual(hold, untouched)
        # The finished but unshrunk retained hold must qualify too (C3 case).
        rt_full = deepcopy(rt)
        rt_full.update(generation=before_shrink["generation"], held_tiles=frozenset({(1,1),(1,2)}))
        ack_full = deepcopy(ack); ack_full["scope"]["reservation_id"] = rt_full["generation"]
        self.assertTrue(self.check(rt_full, before_shrink, ack_full, limits))
        for defect in ("unparked", "wrong_accepted", "generation", "tiles", "pose", "arrival", "ack", "nearzero_only"):
            row, retained, receipt = deepcopy(rt), deepcopy(hold), deepcopy(ack)
            if defect == "unparked": retained["parked"] = False
            if defect == "wrong_accepted": retained["accepted_goal_uuid"] = "another-goal"
            if defect == "generation": retained["generation"] += 1
            if defect == "tiles": retained["tiles"] = [[1,2]]
            if defect == "pose": row["pose"]["stamp_s"] = 1.
            if defect == "arrival": row["pose"]["x_m"] = 2.
            if defect == "ack": receipt["scope"]["goal_uuid"] = "another-goal"
            if defect == "nearzero_only":
                row["live_stop_receiver"].reset()
                row["live_stop_receiver"].observe(0.,0.,stamp_s=1.7,stop_after_s=1.,now_s=1.7)
            detail = {}
            self.assertFalse(live_anchor_stopped("carter1", row, retained, goal=GoalPose("map",1.,1.,0.),
                now_s=1.7, pause_after_s=1., run_id=RUN, scenario_id="normal", ack=receipt, limits=limits,
                diagnostic=detail), defect)
            self.assertTrue(detail["reason"], defect)

        control = LiveControl(RUN, ids)
        wall = datetime.now(timezone.utc)
        control.submit({"command_id":"pause", "run_id":RUN,"expected_run_id":RUN,"expected_revision":0,
            "action":"pause","parameters":{},"requested_at":wall.isoformat()}, now_s=1., wall_now=wall)
        state, journal_rows = {}, []
        rows = {robot: dict(diagnostic) for robot in ids}
        for i in range(100):
            record_live_anchor_diagnostic(control, rows, now_s=1.7+i/1000, state=state,
                journal=lambda kind, body: journal_rows.append((kind, body)))
        self.assertEqual(len(journal_rows),1)  # Clock/sample progress creates no churn.
        for i in range(40):
            rows["carter2"] = dict(diagnostic, reason="missing" if i%2 else "different", qualified=False)
            record_live_anchor_diagnostic(control, rows, now_s=1.9+i/1000, state=state,
                journal=lambda kind, body: journal_rows.append((kind, body)))
        self.assertEqual(len(journal_rows),16)
        self.assertEqual(len(control.diagnostics),1)
        self.assertEqual(control.revision,1)
        control.settle({robot:robot != "carter2" for robot in ids})
        self.assertEqual(control.phase,"pause_pending")
        self.assertEqual(control.receipts["pause"]["status"],"pending")
        # Only a complete four-robot evidence set can apply the unchanged gate.
        control.settle({robot:True for robot in ids})
        self.assertEqual(control.phase,"paused")
        self.assertEqual(control.receipts["pause"]["status"],"applied")


if __name__ == "__main__":
    unittest.main()
