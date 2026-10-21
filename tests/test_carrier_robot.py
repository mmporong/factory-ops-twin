"""Synthetic CPU geometry fixtures only; no native robot measurements."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

from factory_ops.carrier_module import build_manifest, load_profile, manifest_json
from factory_ops.carrier_robot import (ASSET_PATH, CHASSIS_PATH, IDENTITY, ROBOT_PATH,
    assess_mount, inverse_rigid, pose_matrix, rigid_matrix, transform_point, transformed_box)


RUN_ID = "a"*32
PROFILE = Path(__file__).resolve().parents[1]/"configs/vehicles/tote-carrier-v1.json"


def synthetic_measurement(manifest, frame=None, robot_path=ROBOT_PATH):
    frame = deepcopy(IDENTITY if frame is None else frame)
    chassis_path = robot_path+"/chassis_link"
    colliders = []
    for index, post in enumerate(b for b in manifest["geometry"] if b["name"].startswith("post_")):
        colliders.append({"path": chassis_path+"/test_mount_%d" % index, "body_path": chassis_path,
            "world_aabb_m": transformed_box({"center_m": [*post["center_m"][:2], manifest["mount_plane_z_m"]-.02],
                "size_m": [.12, .12, .04]}, frame)})
    colliders.append({"path": chassis_path+"/test_lower_body", "body_path": chassis_path,
        "world_aabb_m": transformed_box({"center_m": [-.1, 0., .15], "size_m": [.8, .5, .3]}, frame)})
    sensor_frame = pose_matrix(transform_point(frame, [.2, 0., 1.2]), [1., 0., 0., 0.])
    return {"schema_version": 1, "run_id": RUN_ID, "robot_path": robot_path, "chassis_path": chassis_path,
        "asset_relative_path": ASSET_PATH, "meters_per_unit": 1., "up_axis": "Z",
        "robot_world_from_local": frame, "chassis_world_from_local": frame,
        "articulation_roots": [chassis_path], "rigid_bodies": [{"path": chassis_path, "enabled": True, "kinematic": False}],
        "colliders": colliders, "sensors": [{"path": chassis_path+"/test_camera", "world_from_local": sensor_frame}]}


class CarrierRobotTests(unittest.TestCase):
    def setUp(self):
        self.manifest = build_manifest(load_profile(PROFILE))
        self.measured = synthetic_measurement(self.manifest)

    def test_legal_synthetic_mount_deterministic_and_nonmutating(self):
        before = deepcopy(self.measured)
        result = assess_mount(self.manifest, self.measured, RUN_ID)
        self.assertTrue(result["admitted"])
        self.assertEqual(result["rejections"], [])
        self.assertEqual(len(result["mount_faces"]), 4)
        self.assertFalse(result["physical_attachment_verified"])
        self.assertFalse(result["map_or_footprint_changed"])
        self.assertEqual(result["loaded_collision_aabb_radius_m"], result["original_collision_aabb_radius_m"])
        self.assertEqual(self.measured, before)
        self.assertEqual(manifest_json(result), manifest_json(assess_mount(self.manifest, self.measured, RUN_ID)))
        self.assertEqual(result, json.loads(manifest_json(result)))

    def test_rigid_transform_yaw_translation_and_column_convention(self):
        matrix = pose_matrix([2., 3., .15], [math.sqrt(.5), 0., 0., math.sqrt(.5)])
        point = transform_point(matrix, [1., 0., 0.])
        for actual, expected in zip(point, [2., 4., .15]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(transform_point(inverse_rigid(matrix), point), [1., 0., 0.]):
            self.assertAlmostEqual(actual, expected)
        result = assess_mount(self.manifest, synthetic_measurement(self.manifest, matrix), RUN_ID)
        self.assertTrue(result["admitted"])
        self.assertAlmostEqual(result["original_collision_aabb_radius_m"], math.hypot(.5, .25))

    def test_explicit_fleet_roots_bind_every_native_path(self):
        for robot_path in ("/World/Carter1", "/World/Carter2", "/World/Carter3", "/World/Carter4"):
            with self.subTest(robot_path=robot_path):
                measured = synthetic_measurement(self.manifest, robot_path=robot_path)
                result = assess_mount(self.manifest, measured, RUN_ID, robot_path=robot_path)
                self.assertTrue(result["admitted"])
                self.assertEqual(result["robot_path"], robot_path)
                self.assertEqual(result["chassis_path"], robot_path+"/chassis_link")
        for robot_path in ("/World/Carter1/Nested", "/World"):
            with self.subTest(invalid_expected_root=robot_path), self.assertRaises(ValueError):
                assess_mount(self.manifest, self.measured, RUN_ID, robot_path=robot_path)

        expected = "/World/Carter1"
        for field, value in (("robot_path", "/World/Carter10"),
                             ("robot_path", expected+"/nested"),
                             ("chassis_path", expected+"/nested/chassis_link"),
                             ("articulation_roots", ["/World/Carter2/chassis_link"])):
            with self.subTest(field=field):
                measured = synthetic_measurement(self.manifest, robot_path=expected)
                measured[field] = value
                with self.assertRaises(ValueError):
                    assess_mount(self.manifest, measured, RUN_ID, robot_path=expected)
        for collection, field, value in (("rigid_bodies", "path", "/World/Carter2/chassis_link"),
                                         ("colliders", "path", "/World/Carter10/chassis_link/collider"),
                                         ("colliders", "body_path", "/World/Carter2/chassis_link"),
                                         ("sensors", "path", "/World/Carter1_extra/camera")):
            with self.subTest(collection=collection, field=field):
                measured = synthetic_measurement(self.manifest, robot_path=expected)
                measured[collection][0][field] = value
                with self.assertRaises(ValueError):
                    assess_mount(self.manifest, measured, RUN_ID, robot_path=expected)

    def test_native_identity_and_incomplete_geometry_fail_closed(self):
        cases = [("run_id", "b"*32), ("chassis_path", ROBOT_PATH+"/wrong_body"),
                 ("asset_relative_path", "/OtherRobot.usd"), ("meters_per_unit", .01),
                 ("up_axis", "Y"), ("articulation_roots", []), ("sensors", []), ("colliders", [])]
        for field, value in cases:
            with self.subTest(field=field):
                measured = deepcopy(self.measured)
                measured[field] = value
                with self.assertRaises(ValueError):
                    assess_mount(self.manifest, measured, RUN_ID)
        for patch in ({"kinematic": True}, {"enabled": False}, {"enabled": 1}):
            measured = deepcopy(self.measured)
            measured["rigid_bodies"][0].update(patch)
            with self.assertRaises(ValueError):
                assess_mount(self.manifest, measured, RUN_ID)
        measured = deepcopy(self.measured)
        measured["colliders"][0]["body_path"] = ROBOT_PATH+"/unknown"
        with self.assertRaises(ValueError):
            assess_mount(self.manifest, measured, RUN_ID)
        measured = deepcopy(self.measured)
        measured["colliders"].append(deepcopy(measured["colliders"][0]))
        with self.assertRaises(ValueError):
            assess_mount(self.manifest, measured, RUN_ID)

    def test_collision_sensor_support_and_radius_rejections_retain_diagnostics(self):
        unsupported = deepcopy(self.measured)
        unsupported["colliders"].pop(0)
        result = assess_mount(self.manifest, unsupported, RUN_ID)
        self.assertIn("unsupported_mount_post", [r["code"] for r in result["rejections"]])
        collision = deepcopy(self.measured)
        collision["sensors"][0]["world_from_local"] = pose_matrix([-.18, 0., .68], [1., 0., 0., 0.])
        result = assess_mount(self.manifest, collision, RUN_ID)
        self.assertFalse(result["admitted"])
        self.assertTrue(any(r.get("prim", "").endswith("test_camera") for r in result["rejections"]))
        self.assertTrue(result["measured_colliders_in_chassis"])
        large = deepcopy(self.measured)
        large["colliders"][0]["world_aabb_m"]["center_m"][0] = -.8
        result = assess_mount(self.manifest, large, RUN_ID)
        self.assertIn("unchanged_footprint_exceeded", [r["code"] for r in result["rejections"]])
        self.assertEqual(result["unchanged_configured_radius_m"], self.manifest["compatibility"]["unchanged_footprint_radius_m"])
        # Admission must not expand the configured footprint at a close boundary.
        original = assess_mount(self.manifest, self.measured, RUN_ID)["original_collision_aabb_radius_m"]
        smaller_limit = deepcopy(self.manifest)
        smaller_limit["compatibility"]["unchanged_footprint_radius_m"] = original-5e-9
        result = assess_mount(smaller_limit, self.measured, RUN_ID)
        self.assertIn("unchanged_footprint_exceeded", [r["code"] for r in result["rejections"]])
        body = deepcopy(self.measured)
        body["colliders"][-1]["world_aabb_m"] = {"center_m": [-.18, 0., .68], "size_m": [.1, .1, .1]}
        self.assertIn("robot_or_sensor_keepout", [r["code"] for r in assess_mount(self.manifest, body, RUN_ID)["rejections"]])

    def test_invalid_numeric_frames_and_bounds_rejected(self):
        for value in (True, float("nan"), float("inf")):
            matrix = deepcopy(IDENTITY)
            matrix[0][3] = value
            with self.assertRaises(ValueError):
                rigid_matrix(matrix)
        for diagonal in (2., -1.):
            matrix = deepcopy(IDENTITY)
            matrix[0][0] = diagonal
            with self.assertRaises(ValueError):
                rigid_matrix(matrix)
        matrix = deepcopy(IDENTITY)
        matrix[3][0] = .01
        with self.assertRaises(ValueError):
            rigid_matrix(matrix)
        with self.assertRaises(ValueError):
            pose_matrix([0., 0., 0.], [2., 0., 0., 0.])
        for size in ([0., 1., 1.], [1., float("nan"), 1.]):
            measured = deepcopy(self.measured)
            measured["colliders"][0]["world_aabb_m"]["size_m"] = size
            with self.assertRaises(ValueError):
                assess_mount(self.manifest, measured, RUN_ID)


if __name__ == "__main__":
    unittest.main()
