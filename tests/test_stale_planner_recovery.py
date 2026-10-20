"""CPU stale-result rejection/re-entry; no ROS or physical observation is synthesized."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from uuid import UUID, uuid4

from factory_ops.fleet_execution import StaleComputedPathError, make_chunk, path_content_sha256, path_header_stamp_ns, qualify_computed_path
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.navigation import GoalPose
from factory_ops.route_reservation import RouteReservationManager
from scripts.ros.g2_mission import make_authorization
from scripts.ros.g3_mission import fresh_guided_compute_request, retained_planner_retry_hold, schedule_stale_planner_retry


ROOT = Path(__file__).resolve().parents[1]
LIMITS = dict(chunk_timeout_s=180., odom_ttl_s=.3, stopped_linear_m_s=.02, stopped_angular_rad_s=.02)
RUN, SCENARIO, ROBOT = "cpu-stale-083", "fleet4_transport_normal", "carter1"


def pose_message():
    return NS(header=NS(frame_id="", stamp=None), pose=NS(position=NS(x=0., y=0., z=0.),
        orientation=NS(x=0., y=0., z=0., w=1.)))


class StalePlannerRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture = json.loads(os.environ.get("STALE_PLANNER_RECOVERY_FIXTURE", "{}"))
        cls.book = RouteBook.from_files(fixture.get("layout", ROOT/"configs/factory-fleet4/layout.json"),
                                       fixture.get("routing", ROOT/"configs/factory-fleet4/g3-routing.json"))
        cls.actual = None
        if "result" in fixture:
            path = Path(fixture["result"])
            if path.stat().st_size > 64*1024*1024:
                raise ValueError("bounded closed JSON fixture exceeds64MiB")
            data = json.loads(path.read_text(encoding="utf-8"))
            matches = [row for row in data["actions"] if row.get("planner_action_uuid") == fixture["planner_action_uuid"]]
            if len(matches) != 1:
                raise ValueError("closed fixture requires one exact planner action")
            cls.actual = matches[0]
        goal = GoalPose(**cls.actual["chunk"]["final_goal"]) if cls.actual else GoalPose("map", *cls.book.nodes["CI"], 0.)
        cls.chunk = make_chunk(cls.book, "C_LOW", "CI", closed=False, final_goal=goal)

    def setUp(self):
        self.manager = RouteReservationManager(RUN, ("carter1", "carter2", "carter3", "carter4"))
        self.payload = deepcopy(self.actual["unqualified_planner_path"]) if self.actual else self.path_payload(10.)
        self.now = self.actual["path_clock"]["admitted_clock_ns"]/1e9 if self.actual else 10.516666694
        self.pose = deepcopy(self.actual["planner_start_pose"]) if self.actual else dict(x_m=15., y_m=5., yaw_rad=0., stamp_s=self.now)
        b = self.book.layout.bounds
        bounds = (b.xmin, b.ymin, b.xmax, b.ymax)
        plan = self.book.plan("C_LOW", "CI", closed=False)
        held = swept_tiles(plan.points, self.book.planning_radius_m+.3, tile_size_m=self.book.tile_size_m, bounds_xy=bounds)
        occupied = swept_tiles([(self.pose["x_m"], self.pose["y_m"])], self.book.planning_radius_m,
                               tile_size_m=self.book.tile_size_m, bounds_xy=bounds)
        for robot, tiles in ((ROBOT, occupied), ("carter2", {(0, 0)}), ("carter3", {(30, 0)}), ("carter4", {(30, 20)})):
            self.manager.observe(robot, tiles, pose_stamp_s=self.now, now_s=self.now)
        follow = self.actual["follow_action_uuid"] if self.actual else str(uuid4())
        decision = self.manager.reserve(ROBOT, self.chunk.route_id, follow, held, now_s=self.now)
        self.assertTrue(decision.granted)
        record = deepcopy(self.actual) if self.actual else {}
        record.update(chunk=self.chunk.to_dict(), generation=decision.generation, follow_action_uuid=follow,
            leg_id=self.actual["leg_id"] if self.actual else plan.route_id,
            planner_action_uuid=self.actual["planner_action_uuid"] if self.actual else str(uuid4()),
            planner_method="compute_path_through_poses", planner_status=4, planner_error_code=0,
            planner_path_qualified=False, unqualified_planner_path=self.payload,
            unqualified_planner_path_content_sha256=path_content_sha256(self.payload),
            path_clock={"admitted_clock_ns": int(round(self.now*1e9)), "path_stamp_ns": path_header_stamp_ns(self.payload["header"])})
        self.runtime = dict(phase="computing", phase_started=self.now-.7, chunk=self.chunk,
            leg={"plan": NS(route_id=record["leg_id"])}, held_tiles=held, pending_tiles=held,
            follow_uuid=follow, compute_uuid=record["planner_action_uuid"], generation=decision.generation, sequence=10,
            record=record, pose=self.pose, pending_path=NS(), pending_path_payload=self.payload,
            last_odom=dict(stamp_s=self.now-.05, linear_m_s=0., angular_rad_s=0.))
        self.snapshot = self.manager.snapshot(now_s=self.now)
        auth = make_authorization(run_id=RUN, scenario_id=SCENARIO, robot_id=ROBOT, sequence=10,
            now_s=self.now, ttl_s=.5, mode="hold", decision_event_seq=0,
            route_id=self.chunk.route_id, goal_uuid=follow, reservation_id=decision.generation)
        auth["scope"]["tile_hash"] = self.snapshot["holds"][ROBOT]["tile_hash"]
        self.runtime["last_authorization"] = auth

    def path_payload(self, stamp_s):
        sec = int(stamp_s)
        header = dict(frame_id="map", sec=sec, nanosec=int(round((stamp_s-sec)*1e9)))
        return dict(header=header, poses=[dict(header=deepcopy(header), position_xyz_m=[x, y, 0.],
            orientation_xyzw=[0., 0., math.sin(self.chunk.goal_pose.yaw/2), math.cos(self.chunk.goal_pose.yaw/2)])
            for x, y in self.chunk.points])

    def qualify(self, payload=None, **changes):
        payload = payload or self.payload
        options = dict(frame_id=payload["header"]["frame_id"], stamp_s=path_header_stamp_ns(payload["header"])/1e9,
            now_s=self.now, start_xy=(self.pose["x_m"], self.pose["y_m"]), max_age_s=.5)
        options.update(changes)
        return qualify_computed_path(self.book, self.chunk, [p["position_xyz_m"][:2] for p in payload["poses"]], **options)

    def stale(self):
        with self.assertRaises(StaleComputedPathError) as caught:
            self.qualify()
        self.assertGreater(caught.exception.age_s, .5)
        return caught.exception

    def schedule(self, error, **changes):
        args = dict(error=error, enabled=True, cutoff=None, run_id=RUN, scenario_id=SCENARIO,
                    now_s=self.now, limits=LIMITS)
        args.update(changes)
        return schedule_stale_planner_retry(ROBOT, self.runtime, self.snapshot, **args)

    def test_stale_rejected_original_retained_and_safe_fresh_request_has_new_uuid_pose_stamp(self):
        original = deepcopy(self.runtime["record"])
        before_hold = deepcopy(self.snapshot["holds"])
        phase_start = self.runtime["phase_started"]
        event = self.schedule(self.stale())
        self.assertEqual(event["retry_number"], 1)
        self.assertEqual(self.runtime["phase"], "request")
        self.assertIsNone(self.runtime["pending_path"])
        self.assertEqual(self.runtime["phase_started"], phase_start)
        self.assertEqual(self.manager.snapshot(now_s=self.now)["holds"], before_hold)
        for name in ("unqualified_planner_path", "unqualified_planner_path_content_sha256", "planner_action_uuid"):
            self.assertEqual(self.runtime["record"][name], original[name])
        self.assertFalse(self.runtime["record"]["planner_path_qualified"])
        self.assertNotIn("follow_goal_sent_sim_time_s", self.runtime["record"])
        retained = retained_planner_retry_hold(ROBOT, self.runtime, self.snapshot, run_id=RUN,
            scenario_id=SCENARIO, now_s=self.now, limits=LIMITS)
        self.assertEqual(retained, self.runtime["planner_retry_pending"]["retained_hold"])
        stamp = NS(sec=int(self.now), nanosec=int(round((self.now-int(self.now))*1e9)))
        estimated = dict(self.pose, x_m=self.pose["x_m"]+.001)
        new_id, request = fresh_guided_compute_request(self.chunk, estimated, stamp, goal_type=NS, pose_type=pose_message)
        self.assertNotEqual(UUID(new_id), UUID(original["planner_action_uuid"]))
        self.assertIs(request.start.header.stamp, stamp)
        self.assertEqual(request.start.pose.position.x, estimated["x_m"])
        self.assertEqual(len(request.goals), len(self.chunk.points[1:])*4)
        fresh = self.path_payload(self.now)
        self.assertTrue(self.qualify(fresh)["passed"])
        self.assertEqual(original["unqualified_planner_path"], self.payload)

    def test_three_retry_budget_exhausts_without_releasing_or_following(self):
        before = deepcopy(self.snapshot["holds"])
        error = self.stale()
        history = []
        initial = deepcopy(self.runtime["record"])
        for number in (1, 2, 3):
            event = self.schedule(error)
            self.assertEqual(event["retry_number"], number)
            history.append(self.runtime["record"])
            self.runtime["phase"] = "computing"
            self.runtime["planner_retry_pending"] = None
            self.runtime["record"] = deepcopy(initial)
            self.runtime["record"]["planner_action_uuid"] = str(uuid4())
            self.runtime["compute_uuid"] = self.runtime["record"]["planner_action_uuid"]
        with self.assertRaisesRegex(RuntimeError, "exhausted after3"):
            self.schedule(error)
        self.assertEqual(self.runtime["record"]["planner_retry_exhausted"]["retries_already_used"], 3)
        self.assertEqual(self.manager.snapshot(now_s=self.now)["holds"], before)
        self.assertTrue(all(not row["planner_path_qualified"] for row in history))

    def test_only_a_genuinely_different_authored_chunk_has_a_new_budget(self):
        other = make_chunk(self.book, "C_HIGH", "P3", closed=False,
            final_goal=GoalPose("map", *self.book.nodes["P3"], math.pi/2))
        self.assertNotEqual(other.route_id, self.chunk.route_id)
        self.runtime["planner_retry_budget"] = {"key": other.route_id, "retries": 3}
        event = self.schedule(self.stale())
        self.assertEqual(event["retry_number"], 1)
        self.assertEqual(self.runtime["planner_retry_budget"]["key"], self.chunk.route_id)

    def test_canonical_cutoff_and_other_errors_never_schedule(self):
        for error, changes in ((self.stale(), {"enabled": False}), (self.stale(), {"cutoff": {"sealed": True}}),
                               (ValueError("different failure"), {}), (RuntimeError("planner aborted"), {})):
            with self.subTest(changes=changes):
                before = deepcopy(self.runtime["record"])
                self.assertIsNone(self.schedule(error, **changes))
                self.assertEqual(self.runtime["record"], before)
                self.assertEqual(self.runtime["phase"], "computing")
                self.assertNotIn("planner_retry_pending", self.runtime)

    def test_geometry_frame_future_and_terminal_yaw_fail_closed(self):
        bad = deepcopy(self.payload)
        bad["poses"].insert(1, dict(header=deepcopy(bad["header"]), position_xyz_m=[3., 5., 0.], orientation_xyzw=[0., 0., 0., 1.]))
        for options, payload in (({}, bad), ({"frame_id": "odom"}, self.payload), ({"stamp_s": self.now+.1}, self.payload)):
            with self.subTest(options=options), self.assertRaises(ValueError) as caught:
                self.qualify(payload, **options)
            self.assertNotIsInstance(caught.exception, StaleComputedPathError)
        error = self.stale()
        changed = self.runtime["record"]["unqualified_planner_path"]
        yaw = self.chunk.goal_pose.yaw+1.
        changed["poses"][-1]["orientation_xyzw"] = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
        self.runtime["record"]["unqualified_planner_path_content_sha256"] = path_content_sha256(changed)
        with self.assertRaisesRegex(RuntimeError, "terminal yaw"):
            self.schedule(error)
        self.assertNotIn("planner_retry_pending", self.runtime)

    def test_changed_hold_bad_lease_moving_or_stale_encoder_and_deadline_cannot_reenter(self):
        error = self.stale()
        original_rt, original_snapshot = deepcopy(self.runtime), deepcopy(self.snapshot)
        for kind in ("accepted-follow", "generation", "tiles", "navigate", "expired-lease", "moving", "stale-odom", "deadline", "stale-peer"):
            with self.subTest(kind=kind):
                self.runtime, self.snapshot = deepcopy(original_rt), deepcopy(original_snapshot)
                hold = self.snapshot["holds"][ROBOT]
                if kind == "accepted-follow": hold["accepted_goal_uuid"] = self.runtime["follow_uuid"]
                elif kind == "generation": hold["generation"] += 1
                elif kind == "tiles": hold["tiles"] = hold["tiles"][:-1]
                elif kind == "navigate": self.runtime["last_authorization"]["mode"] = "navigate"
                elif kind == "expired-lease": self.runtime["last_authorization"]["valid_until_sim_time_s"] = self.now-.1
                elif kind == "moving": self.runtime["last_odom"]["linear_m_s"] = .021
                elif kind == "stale-odom": self.runtime["last_odom"]["stamp_s"] = self.now-.31
                elif kind == "deadline": self.runtime["phase_started"] = self.now-181
                elif kind == "stale-peer": self.snapshot["observations"]["carter2"]["fresh"] = False
                with self.assertRaises(RuntimeError):
                    self.schedule(error)
                self.assertNotIn("planner_retry_pending", self.runtime)


if __name__ == "__main__":
    unittest.main()
