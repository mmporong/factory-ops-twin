"""CPU fleet planning checks; no pxr, SimulationApp, PhysX or GPU."""
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import patch

from factory_ops.carrier_module import build_manifest, load_profile
from factory_ops.carrier_robot import ASSET_PATH, IDENTITY, pose_matrix, transform_point
from scripts.isaac.carrier_native import install_carrier_fleet, prepare_carrier_fleet


RUN_ID = "c" * 32
PROFILE = Path(__file__).resolve().parents[1] / "configs/vehicles/tote-carrier-v1.json"
DEFINITIONS = tuple({"robot_id": f"carter{i}", "prim_path": f"/World/Carter{i}"} for i in range(1, 5))


class _Prim:
    def __init__(self, valid=False):
        self._valid = valid

    def IsValid(self):
        return self._valid


class _Stage:
    def __init__(self, existing=()):
        self.existing = set(existing)

    def GetPrimAtPath(self, path):
        return _Prim(path in self.existing)


def _measurement(manifest, robot_path):
    chassis = robot_path + "/chassis_link"
    colliders = []
    for index, post in enumerate(box for box in manifest["geometry"] if box["name"].startswith("post_")):
        colliders.append({"path": f"{chassis}/mount_{index}", "body_path": chassis,
            "world_aabb_m": {"center_m": [*post["center_m"][:2], manifest["mount_plane_z_m"] - .02],
                             "size_m": [.12, .12, .04]}})
    colliders.append({"path": chassis + "/lower", "body_path": chassis,
        "world_aabb_m": {"center_m": [-.1, 0., .15], "size_m": [.8, .5, .3]}})
    sensor = pose_matrix(transform_point(IDENTITY, [.2, 0., 1.2]), [1., 0., 0., 0.])
    return {"schema_version": 1, "run_id": RUN_ID, "robot_path": robot_path, "chassis_path": chassis,
        "asset_relative_path": ASSET_PATH, "meters_per_unit": 1., "up_axis": "Z",
        "robot_world_from_local": deepcopy(IDENTITY), "chassis_world_from_local": deepcopy(IDENTITY),
        "articulation_roots": [chassis],
        "rigid_bodies": [{"path": chassis, "enabled": True, "kinematic": False}],
        "colliders": colliders,
        "sensors": [{"path": chassis + "/camera", "world_from_local": sensor}],
        "composition_errors": [], "prims_inspected": 12}


class CarrierNativePlanTests(unittest.TestCase):
    def setUp(self):
        self.manifest = build_manifest(load_profile(PROFILE))
        self.stage = _Stage()
        self.measure = lambda stage, path, run: _measurement(self.manifest, path)

    def test_four_robot_plan_is_unique_and_visual_tote_is_explicitly_nonphysical(self):
        definitions_by_id = {row["robot_id"]: row for row in DEFINITIONS}
        plan = prepare_carrier_fleet(self.stage, definitions_by_id, self.manifest, RUN_ID,
                                     clearance_m=.01, _measure_robot=self.measure)
        self.assertEqual(plan["fleet_size"], 4)
        self.assertEqual(len({row["carrier_path"] for row in plan["robots"]}), 4)
        self.assertEqual([row["robot_path"] for row in plan["robots"]],
                         [row["prim_path"] for row in DEFINITIONS])
        self.assertEqual(plan["robots"][0]["tote_visual_size_m"], [.44, .34, .18])
        self.assertFalse(plan["tote_physics_authored"])
        self.assertFalse(plan["physical_loading_or_unloading_verified"])

    def test_rejection_prevents_every_author_call(self):
        calls = []

        plan = prepare_carrier_fleet(self.stage, DEFINITIONS, self.manifest, RUN_ID,
                                     clearance_m=.01, _measure_robot=self.measure)

        def reject_one(stage, path, run):
            measured = _measurement(self.manifest, path)
            if path == "/World/Carter3":
                measured["colliders"].pop(0)
            return measured

        with patch("scripts.isaac.carrier_native._sdk") as load_sdk, self.assertRaises(ValueError):
            install_carrier_fleet(self.stage, plan, _measure_robot=reject_one,
                                  _author_carrier=lambda *args: calls.append(args))
        load_sdk.assert_not_called()
        self.assertEqual(calls, [])

    def test_existing_parent_or_leaf_path_blocks_preparation(self):
        for path in ("/World/CarrierModules", "/World/CarrierModules/carter2/Carrier/deck"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                prepare_carrier_fleet(_Stage([path]), DEFINITIONS, self.manifest, RUN_ID,
                                      clearance_m=.01, _measure_robot=self.measure)

    def test_registry_cross_vehicle_identity_is_rejected(self):
        def crossed(stage, path, run):
            return _measurement(self.manifest, "/World/Carter2" if path == "/World/Carter1" else path)

        with self.assertRaises(ValueError):
            prepare_carrier_fleet(self.stage, DEFINITIONS, self.manifest, RUN_ID,
                                  clearance_m=.01, _measure_robot=crossed)

    def test_install_rechecks_geometry_before_any_authoring(self):
        plan = prepare_carrier_fleet(self.stage, DEFINITIONS, self.manifest, RUN_ID,
                                     clearance_m=.01, _measure_robot=self.measure)
        calls = []

        def changed(stage, path, run):
            measured = _measurement(self.manifest, path)
            if path == "/World/Carter4":
                measured["chassis_world_from_local"][0][3] = .1
            return measured

        with patch("scripts.isaac.carrier_native._sdk") as load_sdk, self.assertRaises(ValueError):
            install_carrier_fleet(self.stage, plan, _measure_robot=changed,
                                  _author_carrier=lambda *args: calls.append(args))
        load_sdk.assert_not_called()
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
