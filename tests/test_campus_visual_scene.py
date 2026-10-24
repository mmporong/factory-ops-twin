from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.layout import CAMPUS16_BUFFERED_REVISION, Layout, LayoutError
from factory_ops.usda import export_usda
from factory_ops.visual_scene import _validate_config, export_visual_scene
from scripts.isaac.factory_visuals import _expected_source_stage_counts


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "configs" / "factory-semiconductor"
CAMPUS = ROOT / "configs" / "factory-semiconductor-campus16"


class CampusVisualSceneTests(unittest.TestCase):
    def test_buffered_campus_and_frozen_v1_keep_separate_coordinates(self) -> None:
        legacy = Layout.from_file(ROOT / "tests/fixtures/campus16-v1-layout-290.json")
        current = Layout.from_file(CAMPUS / "layout.json")
        self.assertEqual("amr-semiconductor-campus16-v1", legacy.data["revision"])
        self.assertEqual((3.5, 4.5), legacy.stations["A_S1"])
        self.assertEqual((0.0, 0.0, 78.0, 60.0), tuple(legacy.data["bounds_xy"]))
        self.assertEqual(CAMPUS16_BUFFERED_REVISION, current.data["revision"])
        self.assertEqual((3.5, 12.5), current.stations["A_S1"])
        self.assertEqual((0.0, 0.0, 82.0, 76.0), tuple(current.data["bounds_xy"]))
        cameras = {camera["id"]: camera for camera in json.loads(
            (CAMPUS / "render-profile.json").read_text(encoding="utf-8"))["cameras"]}
        self.assertEqual([26.0, 12.0, 13.0], cameras["cell_process"]["eye_xyz_m"])
        self.assertEqual([19.0, 20.5, 1.0], cameras["cell_process"]["target_xyz_m"])
        self.assertEqual([0, 0, .35], cameras["carter1_follow"]["target_xyz_m"])
        shifted = json.loads(json.dumps(current.data))
        shifted["stations"][0]["xy"][1] -= 8.0
        with self.assertRaisesRegex(LayoutError, "immutable"):
            Layout(shifted)

    def test_base_and_campus_counts_follow_each_layout(self) -> None:
        for config, obstacles in ((BASE, 19), (CAMPUS, 76)):
            with self.subTest(config=config.name), tempfile.TemporaryDirectory() as temporary:
                layout = Layout.from_file(config / "layout.json")
                result = export_usda(layout, Path(temporary) / "factory-v3.usda")
                text = Path(result["path"]).read_text(encoding="utf-8")

                self.assertEqual(obstacles, result["obstacle_collider_count"])
                self.assertEqual(obstacles + 1, result["static_collider_count"])
                self.assertEqual(obstacles + 1, text.count('prepend apiSchemas = ["PhysicsCollisionAPI"]'))
                self.assertEqual(len(layout.stations) + len(layout.pockets), result["non_colliding_marker_count"])
                self.assertEqual({
                    "obstacles": obstacles,
                    "colliders": obstacles + 1,
                    "markers": len(layout.stations) + len(layout.pockets),
                }, _expected_source_stage_counts(layout))

                try:
                    from pxr import Usd
                except ImportError:
                    continue
                stage = Usd.Stage.Open(str(result["path"]))
                self.assertIsNotNone(stage)
                self.assertTrue(stage.GetPrimAtPath("/World").IsValid())

    def test_campus_visual_manifest_uses_all_assignments_fixtures_and_authored_labels(self) -> None:
        layout = Layout.from_file(CAMPUS / "layout.json")
        config = json.loads((CAMPUS / "visual-assets.json").read_text(encoding="utf-8"))
        _validate_config(config, layout)
        stale = json.loads(json.dumps(config))
        stale["campus_transform"]["bay_offsets_xy_m"]["A"][1] -= 8.0
        with self.assertRaisesRegex(LayoutError, "do not match the layout revision"):
            _validate_config(stale, layout)

        obstacle_ids = {str(item["id"]) for item in layout.data["physical_obstacles"]}
        fixtures = config["environment"]["fixtures"]
        self.assertEqual(76, len(obstacle_ids))
        self.assertEqual(88, len(fixtures))
        self.assertEqual(len(fixtures), config["environment"]["fixture_count"])
        self.assertEqual(52, len(config["environment"]["excluded_fixtures"]))
        self.assertTrue(all(item["id"].split("_", 1)[0] in {"A", "B", "C", "D"} for item in fixtures))
        self.assertEqual({"A": 20, "B": 24, "C": 23, "D": 21}, {
            bay: sum(item["id"].startswith(f"{bay}_") for item in fixtures)
            for bay in ("A", "B", "C", "D")
        })
        self.assertTrue(all(item.get("source_obstacle_id") in obstacle_ids
                            for item in fixtures if "source_obstacle_id" in item))

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root / "scene"
            base = export_usda(layout, output / "factory-v3.usda")
            text_config = json.loads(json.dumps(config))
            text_config.pop("environment")
            for assignment in text_config["obstacle_assets"].values():
                if assignment["recipe"] == "external_asset":
                    assignment["recipe"] = "cleanroom_utility"
            text_config_path = root / "visual-assets-text-export.json"
            text_config_path.write_text(json.dumps(text_config), encoding="utf-8")
            manifest = export_visual_scene(
                layout,
                output,
                config_path=text_config_path,
                render_profile_path=CAMPUS / "render-profile.json",
            )

            invariants = manifest["physical_invariants"]
            self.assertEqual(76, invariants["source_obstacle_count"])
            self.assertEqual(77, invariants["source_static_collider_count"])
            self.assertEqual(0, invariants["visual_collision_api_count"])
            self.assertEqual(77, base["static_collider_count"])

            visual = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")
            composition = (output / "factory-v3.render.usda").read_text(encoding="utf-8")
            self.assertNotIn("PhysicsCollisionAPI", visual)
            self.assertEqual(76, composition.count('token visibility = "invisible"'))
            for station_id in ("A_S1", "B_CI", "C_OUT", "D_P4"):
                self.assertIn(f'custom string stableId = "{station_id}"', visual)
            for label in ("A FINAL TEST", "B AOI", "C PACK", "D LOT OUT"):
                self.assertIn(f'custom string stableId = "floor_stencil:{label}"', visual)
            for bay in "ABCD":
                self.assertIn(f'custom string stableId = "floor_stencil:{bay} WAIT"', visual)
            for queue in layout.data["bridge_queues"]:
                label = queue["id"].replace("_QUEUE", " WAIT").replace("_", " ")
                self.assertIn(f'custom string stableId = "floor_stencil:{label}"', visual)
            self.assertIn('custom string stateRole = "bridge_admission_queue"', visual)
            self.assertEqual(8, len(layout.data["bridge_safe_exits"]))
            for exit_node in layout.data["bridge_safe_exits"]:
                label = exit_node["id"].replace("_SAFE_EXIT", " EXIT").replace("_", " ")
                self.assertIn(f'custom string stableId = "floor_stencil:{label}"', visual)
            self.assertIn('custom string stateRole = "bridge_release_buffer"', visual)


if __name__ == "__main__":
    unittest.main()
