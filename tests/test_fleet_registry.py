"""Fleet declaration checks run without importing ROS, USD or Isaac Sim."""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from typing import Any

from factory_ops.fleet_registry import FleetRegistry


PROJECT = Path(__file__).resolve().parents[1]


class FleetRegistryTest(unittest.TestCase):
    def inputs(self, count: int = 2) -> tuple[dict[str, Any], dict[str, Any]]:
        config = json.loads((PROJECT / "configs/factory-v4/g2.json").read_text(encoding="utf-8"))
        config["robots"] = [
            {"robot_id": f"carter{i}", "namespace": f"amr_{i}", "prim_path": f"/World/Fleet/Carter{i}"}
            for i in range(1, count + 1)]
        return config, {"spawns": {f"carter{i}": [i * 3, 4, 0] for i in range(1, count + 1)}}

    def test_one_two_four_preserve_order_and_namespace_topics(self):
        for count in (1, 2, 4):
            with self.subTest(count=count):
                config, scenario = self.inputs(count)
                config["robots"].reverse()
                registry = FleetRegistry(config, scenario)
                expected = tuple(robot["robot_id"] for robot in config["robots"])
                self.assertEqual(expected, registry.robot_ids)
                self.assertEqual(tuple(config["robots"]), registry.definitions)
                self.assertEqual({key: tuple(value) for key, value in scenario["spawns"].items()}, registry.spawns)
                self.assertEqual(frozenset((key, side) for key in expected for side in ("front", "rear")), registry.sensor_keys)
                manifest = registry.manifest()
                self.assertEqual(1, manifest["schema_version"])
                self.assertEqual(list(expected), manifest["robot_ids"])
                self.assertEqual(count * 2, manifest["lidar_count"])
                self.assertEqual("declared configuration only; not runtime qualification", manifest["scope"])
                for robot in manifest["robots"]:
                    self.assertEqual(list(registry.spawns[robot["robot_id"]]), robot["spawn_xy_yaw"])
                    for topic in ("scan_front_raw", "scan_front", "scan_rear_raw", "scan_rear", "odom", "cmd_vel"):
                        self.assertEqual(f"/{robot['namespace']}/{topic}", robot["topics"][topic])
                for lidar in manifest["lidars"]:
                    definition = next(robot for robot in config["robots"] if robot["robot_id"] == lidar["robot_id"])
                    self.assertEqual(f"{definition['prim_path']}/chassis_link/lidar_{lidar['name']}", lidar["prim_path"])
                    self.assertEqual(f"/{definition['namespace']}/scan_{lidar['name']}_raw", lidar["raw_topic"])
                    self.assertEqual(f"/{definition['namespace']}/scan_{lidar['name']}", lidar["filtered_topic"])
                json.dumps(manifest, allow_nan=False)

    def test_existing_g2_configuration_and_scenarios(self):
        config = json.loads((PROJECT / "configs/factory-v4/g2.json").read_text(encoding="utf-8"))
        document = json.loads((PROJECT / "configs/factory-v3/g2-scenarios.json").read_text(encoding="utf-8"))
        for name, scenario in document["scenarios"].items():
            with self.subTest(scenario=name):
                registry = FleetRegistry(config, scenario)
                self.assertEqual(("carter1", "carter2"), registry.robot_ids)
                self.assertEqual(4, registry.manifest()["lidar_count"])

    def test_robot_fields_are_unique_valid_and_not_nested(self):
        invalid_names = ("", " carter1", "carter1 ", "/carter1", "a/b", "1robot", "robot-1", "robot.name", "robot\n")
        for field in ("robot_id", "namespace"):
            for value in invalid_names:
                with self.subTest(field=field, value=value):
                    config, scenario = self.inputs()
                    config["robots"][0][field] = value
                    with self.assertRaises(ValueError):
                        FleetRegistry(config, scenario)
        for field in ("robot_id", "namespace", "prim_path"):
            config, scenario = self.inputs()
            config["robots"][1][field] = config["robots"][0][field]
            with self.subTest(duplicate=field), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        bad_paths = ("/World", "/World/", "/Other/Robot", "World/Robot", "/World//Robot",
                     "/World/Robot/", "/World/1Robot", "/World/Robot-name", "/World/A/../B", "/World/A.attr")
        for path in bad_paths:
            config, scenario = self.inputs()
            config["robots"][0]["prim_path"] = path
            with self.subTest(path=path), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        for reverse in (False, True):
            config, scenario = self.inputs()
            config["robots"][0]["prim_path"] = "/World/A"
            config["robots"][1]["prim_path"] = "/World/A/Child"
            if reverse:
                config["robots"].reverse()
            with self.subTest(nested_reverse=reverse), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)

    def test_spawn_key_set_finite_triples_and_xy_exclusivity(self):
        for spawns in ({}, {"carter1": [1, 2, 0]}, {"carter1": [1, 2, 0], "carter2": [3, 4, 0], "extra": [5, 6, 0]}, []):
            config, scenario = self.inputs()
            scenario["spawns"] = spawns
            with self.subTest(spawns=spawns), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        invalid = (None, "123", [1, 2], [1, 2, 3, 4], [True, 2, 0], [1, False, 0], [1, 2, True],
                   [math.inf, 2, 0], [1, math.nan, 0], [1, 2, -math.inf], [1, 2, "0"], [10**1000, 2, 0])
        for spawn in invalid:
            config, scenario = self.inputs()
            scenario["spawns"]["carter1"] = spawn
            with self.subTest(spawn=spawn), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        config, scenario = self.inputs()
        scenario["spawns"] = {"carter1": [0.0, 2, 0], "carter2": [-0.0, 2.0, math.pi]}
        with self.assertRaisesRegex(ValueError, "XY"):
            FleetRegistry(config, scenario)

    def test_front_rear_lidar_identity_and_extrinsics_are_required(self):
        invalid_sets = (None, [], [{}], ["front", "rear"], [{"name": "front"}] * 3)
        for lidars in invalid_sets:
            config, scenario = self.inputs()
            config["lidars"] = lidars
            with self.subTest(lidars=lidars), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        changes = (("name", "front"), ("name", "side"), ("frame_id", "front_scan"),
                   ("frame_id", "/scan"), ("frame_id", "front/scan"), ("frame_id", "1scan"),
                   ("translation_xyz_m", [1, 2]), ("translation_xyz_m", [True, 0, 1]),
                   ("translation_xyz_m", [0, math.nan, 1]), ("yaw_rad", math.inf),
                   ("yaw_rad", True), ("yaw_rad", None))
        for field, value in changes:
            config, scenario = self.inputs()
            config["lidars"][1][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        config, scenario = self.inputs()
        config["lidars"] = [deepcopy(config["lidars"][0]), deepcopy(config["lidars"][0])]
        with self.assertRaises(ValueError):
            FleetRegistry(config, scenario)

    def test_inputs_and_exports_cannot_mutate_validated_registry(self):
        config, scenario = self.inputs()
        config["robots"][0]["metadata"] = {"labels": ["original"]}
        before = deepcopy((config, scenario))
        registry = FleetRegistry(config, scenario)
        original = registry.manifest()
        self.assertEqual(before, (config, scenario))
        config["robots"][0]["metadata"]["labels"].append("changed")
        config["robots"][0]["namespace"] = "changed"
        config["lidars"][0]["translation_xyz_m"][0] = 999
        scenario["spawns"]["carter1"][0] = 999
        definitions = registry.definitions
        definitions[0]["metadata"]["labels"].append("export changed")
        spawns = registry.spawns
        spawns["carter1"] = (999, 999, 999)
        manifest = registry.manifest()
        manifest["robots"][0]["metadata"]["labels"].append("manifest changed")
        manifest["robots"][0]["topics"]["odom"] = "/bad/odom"
        manifest["lidars"][0]["translation_xyz_m"][0] = 999
        self.assertEqual(original, registry.manifest())
        for field in ("robot_ids", "definitions", "spawns", "sensor_keys"):
            with self.subTest(field=field), self.assertRaises(AttributeError):
                setattr(registry, field, None)

    def test_missing_or_malformed_containers_raise_value_error_without_mutation(self):
        for robots in (None, [], (), {}, [None], ["robot"], [{}]):
            config, scenario = self.inputs()
            config["robots"] = robots
            before = deepcopy((config, scenario))
            with self.subTest(robots=robots), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
            self.assertEqual(before, (config, scenario))

    def test_missing_identity_or_sensor_fields_are_rejected(self):
        for field in ("robot_id", "namespace", "prim_path"):
            config, scenario = self.inputs()
            del config["robots"][0][field]
            with self.subTest(robot_field=field), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)
        for field in ("name", "frame_id", "translation_xyz_m", "yaw_rad"):
            config, scenario = self.inputs()
            del config["lidars"][0][field]
            with self.subTest(lidar_field=field), self.assertRaises(ValueError):
                FleetRegistry(config, scenario)

    def test_distinct_xy_is_not_a_footprint_or_runtime_qualification(self):
        config, scenario = self.inputs()
        config["robots"][0]["prim_path"] = "/World/Robot"
        config["robots"][1]["prim_path"] = "/World/Robot2"
        scenario["spawns"] = {"carter1": [1, 2, 0], "carter2": [1.001, 2, 0]}
        registry = FleetRegistry(config, scenario)
        self.assertEqual(2, len(registry.robot_ids))
        self.assertIn("not runtime qualification", registry.manifest()["scope"])


if __name__ == "__main__":
    unittest.main()
