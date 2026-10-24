from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.layout import Layout
from factory_ops.usda import export_usda
from factory_ops.visual_scene import _validate_config, export_visual_scene
from scripts.isaac.factory_visuals import _expected_source_stage_counts


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "configs" / "factory-semiconductor"
CAMPUS = ROOT / "configs" / "factory-semiconductor-campus16"


class CampusVisualSceneTests(unittest.TestCase):
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

        obstacle_ids = {str(item["id"]) for item in layout.data["physical_obstacles"]}
        fixtures = config["environment"]["fixtures"]
        self.assertEqual(76, len(obstacle_ids))
        self.assertEqual(104, len(fixtures))
        self.assertEqual(len(fixtures), config["environment"]["fixture_count"])
        self.assertEqual(36, len(config["environment"]["excluded_fixtures"]))
        self.assertTrue(all(item["id"].split("_", 1)[0] in {"A", "B", "C", "D"} for item in fixtures))
        self.assertEqual({"A": 26, "B": 26, "C": 27, "D": 25}, {
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


if __name__ == "__main__":
    unittest.main()
