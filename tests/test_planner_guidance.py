"""CPU request/geometry checks; optional closed JSON fixtures never prove live time."""
from copy import deepcopy
from dataclasses import replace
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from factory_ops.fleet_execution import make_chunk, path_content_sha256, path_header_stamp_ns, qualify_computed_path
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.navigation import GoalPose
from factory_ops.planner_guidance import authored_guides, guided_planner_scope, guided_request
from scripts.ros.g3_mission import retain_unqualified_planner_path


ROOT = Path(__file__).resolve().parents[1]


def pose_message():
    return NS(header=NS(frame_id="", stamp=None), pose=NS(
        position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))


def ros_path(payload):
    def header(value):
        return NS(frame_id=value["frame_id"], stamp=NS(sec=value["sec"], nanosec=value["nanosec"]))

    poses = []
    for value in payload["poses"]:
        x, y, z = value["position_xyz_m"]
        qx, qy, qz, qw = value["orientation_xyzw"]
        poses.append(NS(header=header(value["header"]), pose=NS(position=NS(x=x, y=y, z=z),
            orientation=NS(x=qx, y=qy, z=qz, w=qw))))
    return NS(header=header(payload["header"]), poses=poses)


class PlannerGuidanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Paths are supplied by the invocation; portable tests have no user artifact dependency.
        cls.fixtures = json.loads(os.environ.get("PLANNER_GUIDANCE_FIXTURES", "{}"))
        cls.book = RouteBook.from_files(cls.fixtures.get("layout", ROOT/"configs/factory-fleet4/layout.json"),
                                       cls.fixtures.get("routing", ROOT/"configs/factory-fleet4/g3-routing.json"))
        cls.chunk = make_chunk(cls.book, "C_HIGH", "P3", closed=False,
                               final_goal=GoalPose("map", 17., 1.5, math.pi/2))

    def test_guides_follow_authored_reference_and_preserve_chunk_goal(self):
        guides = authored_guides(self.chunk)
        self.assertEqual(len(guides), 24)
        self.assertEqual((guides[0].x, guides[0].y), (15., 16.5))
        previous = self.chunk.points[0]
        for guide in guides:
            self.assertEqual(guide.x, 15.)
            self.assertGreater(math.dist(previous, (guide.x, guide.y)), 0.)
            self.assertLessEqual(math.dist(previous, (guide.x, guide.y)), .5)
            previous = (guide.x, guide.y)
        self.assertEqual(guides[-1], self.chunk.goal_pose)
        self.assertNotEqual(guides[-1], self.chunk.final_goal)
        corner = replace(self.chunk, points=((0., 0.), (1., 0.), (1., 1.)),
                         goal_pose=GoalPose("map", 1., 1., .7))
        self.assertEqual([(g.x, g.y) for g in authored_guides(corner)],
                         [(.5, 0.), (1., 0.), (1., .5), (1., 1.)])
        self.assertEqual(authored_guides(corner)[-1].yaw, .7)

    def test_zero_edge_and_invalid_or_excessive_guides(self):
        zero = make_chunk(self.book, "P3", "P3", closed=False,
                          final_goal=GoalPose("map", 17., 1.5, .6))
        self.assertEqual(authored_guides(zero), (zero.goal_pose,))
        for value in (True, 0., -.1, .5001, math.nan, math.inf):
            with self.subTest(spacing=value), self.assertRaises(ValueError):
                authored_guides(self.chunk, spacing_m=value)
        for chunk in (replace(self.chunk, points=((15., math.nan), (15., 5.))),
                      replace(self.chunk, goal_pose=GoalPose("map", 15., 5.1, 0.)),
                      replace(self.chunk, points=((0., 5.), (2000., 5.)),
                              goal_pose=GoalPose("map", 2000., 5., 0.))):
            with self.subTest(chunk=chunk), self.assertRaises(ValueError):
                authored_guides(chunk)

    def test_scope_excludes_canonical_rtx_and_other_workloads(self):
        scope = dict(runtime_profile="local45_development", development=True, robot_count=4,
                     material_enabled=True, sensor_backends=["physx_collision_planar"]*8)
        self.assertTrue(guided_planner_scope(**scope))
        for change in ({"runtime_profile": "canonical62"}, {"development": False}, {"robot_count": 2},
                       {"material_enabled": False}, {"sensor_backends": ["rtx"]*8},
                       {"sensor_backends": []}, {"sensor_backends": ["physx_collision_planar", "rtx"]}):
            with self.subTest(change=change):
                self.assertFalse(guided_planner_scope(**(scope | change)))

    def test_request_keeps_observed_start_original_stamp_and_installed_goals_field(self):
        start = dict(x_m=14.946113760329675, y_m=16.996795582406115, yaw_rad=.004293838726940977)
        original = deepcopy(start)
        stamp = NS(sec=827, nanosec=700043168)
        request = guided_request(self.chunk, start, stamp, goal_type=NS, pose_type=pose_message)
        self.assertTrue(request.use_start)
        self.assertEqual(request.planner_id, "GridBased")
        self.assertFalse(hasattr(request, "goal"))
        self.assertEqual(len(request.goals), 24)
        self.assertEqual((request.start.pose.position.x, request.start.pose.position.y),
                         (start["x_m"], start["y_m"]))
        self.assertEqual(request.start.pose.orientation.z, math.sin(start["yaw_rad"]/2))
        for pose in [request.start, *request.goals]:
            self.assertIs(pose.header.stamp, stamp)
            self.assertEqual(pose.header.frame_id, "map")
        self.assertEqual(request.goals[-1].pose.orientation.z, math.sin(-math.pi/4))
        self.assertEqual(start, original)
        for value in (True, math.nan, math.inf, "15"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                guided_request(self.chunk, start | {"x_m": value}, stamp, goal_type=NS, pose_type=pose_message)

    def counterexample(self):
        if "original_case" in self.fixtures:
            case = json.loads(Path(self.fixtures["original_case"]).read_text(encoding="utf-8"))
            self.assertEqual(self.chunk.to_dict(), case["failed_action"]["chunk"])
            self.assertEqual(len(case["path"]["poses"]), 899)
            return case["path"], case["previous_qualified_arrival"]
        header = dict(frame_id="map", sec=10, nanosec=123)
        return dict(header=header, poses=[dict(header=deepcopy(header), position_xyz_m=[x, y, 0.],
                    orientation_xyzw=[0., 0., math.sin(-math.pi/4), math.cos(-math.pi/4)])
                    for x, y in ((15., 17.), (9., 17.), (9., 5.), (15., 5.))]), dict(x_m=15., y_m=17.)

    def test_rejected_path_payload_and_hash_survive_original_geometry_guard(self):
        payload, start = self.counterexample()
        record = {}
        path = ros_path(payload)
        retained = retain_unqualified_planner_path(record, path, status=4, error_code=0, error_msg="")
        self.assertEqual(retained, payload)
        self.assertEqual(record["unqualified_planner_path_content_sha256"], path_content_sha256(payload))
        # Offline geometry invocation: source stamp is unchanged; this is not live freshness evidence.
        source_s = path_header_stamp_ns(payload["header"])/1e9
        with self.assertRaisesRegex(ValueError, "leaves reserved chunk tiles"):
            qualify_computed_path(self.book, self.chunk, [v["position_xyz_m"][:2] for v in retained["poses"]],
                frame_id=retained["header"]["frame_id"], stamp_s=source_s, now_s=source_s,
                start_xy=(start["x_m"], start["y_m"]))
        path.poses[0].pose.position.x = 999.
        retained["poses"][0]["position_xyz_m"][0] = 888.
        self.assertEqual(record["unqualified_planner_path"], payload)
        self.assertFalse(record["planner_path_qualified"])
        self.assertNotIn("path_geometry", record)
        invalid = ros_path(payload)
        invalid.poses[0].header.frame_id = "odom"
        with self.assertRaises(ValueError):
            retain_unqualified_planner_path(record, invalid, status=4, error_code=0, error_msg="")
        self.assertEqual(record["planner_path_admission_error"]["type"], "ValueError")

    def test_actual_cpu_guided_path_reuses_unchanged_swept_tiles_and_raw_headers(self):
        if "probe_directory" in self.fixtures:
            folder = Path(self.fixtures["probe_directory"])
            report = json.loads((folder/"result.json").read_text(encoding="utf-8"))
            baseline = json.loads((folder/"to_pose-path.json").read_text(encoding="utf-8"))
            guided = json.loads((folder/"through_poses-path.json").read_text(encoding="utf-8"))
            self.assertEqual(report["cases"]["through_poses"]["actual_clock_admission"], "passed")
            self.assertEqual((len(baseline["poses"]), len(guided["poses"])), (899, 431))
            for name, payload in (("to_pose", baseline), ("through_poses", guided)):
                self.assertEqual(payload["header"], report["cases"][name]["raw_header"])
                self.assertEqual(path_content_sha256(payload), report["cases"][name]["path_sha256"])
        else:
            baseline, _ = self.counterexample()
            guided = deepcopy(baseline)
            guided["poses"] = [dict(header=deepcopy(guided["header"]), position_xyz_m=[g.x, g.y, 0.],
                orientation_xyzw=[0., 0., math.sin(g.yaw/2), math.cos(g.yaw/2)])
                for g in (GoalPose("map", 15., 17., -math.pi/2), *authored_guides(self.chunk))]
        bounds = self.book.layout.bounds
        def outside(payload):
            return swept_tiles([p["position_xyz_m"][:2] for p in payload["poses"]], self.book.evaluation_radius_m,
                tile_size_m=self.book.tile_size_m, bounds_xy=(bounds.xmin, bounds.ymin, bounds.xmax, bounds.ymax))-self.chunk.tiles
        self.assertTrue(outside(baseline))
        self.assertEqual(outside(guided), frozenset())
        if self.fixtures:
            self.assertEqual(len(outside(baseline)), 42)


if __name__ == "__main__":
    unittest.main()
