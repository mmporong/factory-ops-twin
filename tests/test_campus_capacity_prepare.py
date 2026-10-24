import copy
import json
from pathlib import Path
import unittest

from scripts.demo.prepare_campus_capacity import build_robot_definitions
from scripts.isaac.local_fleet_runtime_probe import validate_config


ROOT = Path(__file__).resolve().parents[1]


class CampusCapacityPrepareTests(unittest.TestCase):
    def test_existing_four_robot_config_keeps_sensor_default(self) -> None:
        config = json.loads((ROOT / "configs/local45-fleet-runtime-probe.json").read_text(encoding="utf-8"))
        contract = validate_config(config)
        self.assertEqual(contract["runtime_mode"], "sensors")
        self.assertEqual(len(contract["robots"]), 4)

    def test_sixteen_unique_homes_map_to_bounded_motion(self) -> None:
        robots = [{"robot_id": f"carter{i}", "namespace": f"carter{i}", "prim_path": f"/World/Carter{i}"}
            for i in range(1, 17)]
        stations = [f"{bay}_P{pocket}" for bay in "ABCD" for pocket in range(1, 5)]
        offsets = {"A": (0., 0.), "B": (42., 0.), "C": (0., 32.), "D": (42., 32.)}
        base = {"P1": (12., 9.5), "P2": (12., 34.), "P3": (17., 9.5), "P4": (17., 34.)}
        nodes = {station: [base[station[-2:]][0] + offsets[station[0]][0],
            base[station[-2:]][1] + offsets[station[0]][1]] for station in stations}
        regions = []
        for station, (x, y) in nodes.items():
            regions.append({"id": station, "rect": [x-1.5, y-1.5, x+1.5, y+2.5]})
        definitions = build_robot_definitions(
            {"robots": robots}, {"robot_stations": stations},
            {"parking": {f"amr-{i}": station for i, station in enumerate(stations, 1)}, "nodes": nodes},
            {"amr_allowed_regions": regions, "physical_obstacles": []}, 16)
        self.assertEqual(len({row["robot_id"] for row in definitions}), 16)
        self.assertEqual(len({row["home_station"] for row in definitions}), 16)
        for row in definitions:
            start = row["spawn_xy_yaw"]
            end = row["expected_motion_endpoint_xy"]
            self.assertAlmostEqual(((end[0]-start[0])**2 + (end[1]-start[1])**2) ** .5, .6)

    def test_duplicate_identity_and_unbounded_count_fail_closed(self) -> None:
        config = json.loads((ROOT / "configs/local45-fleet-runtime-probe.json").read_text(encoding="utf-8"))
        duplicate = copy.deepcopy(config)
        duplicate["robots"][1]["namespace"] = duplicate["robots"][0]["namespace"]
        with self.assertRaisesRegex(ValueError, "namespace"):
            validate_config(duplicate)
        config["robot_count"] = 12
        with self.assertRaisesRegex(ValueError, "1, 4, 8, or 16"):
            validate_config(config)


if __name__ == "__main__":
    unittest.main()
