from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.layout import DUAL_LANE_REVISION, Layout, LayoutError, Rect
from factory_ops.usda import export_usda
from factory_ops.visual_scene import _dual_lane_specs, export_visual_scene
from scripts.isaac.factory_visuals import signal_placements, world_slot_placement


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "factory-semiconductor-dual-lane"


class DualLaneVisualTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(CONFIG / "layout.json")

    def test_visual_export_uses_authored_lane_directions_and_semiconductor_equipment(self) -> None:
        visual_config = json.loads((CONFIG / "visual-assets.json").read_text(encoding="utf-8"))
        visual_config.pop("environment", None)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            config_path = output / "visual-assets.json"
            config_path.write_text(json.dumps(visual_config), encoding="utf-8")
            export_usda(self.layout, output / "factory-v3.usda")
            manifest = export_visual_scene(
                self.layout,
                output,
                0,
                config_path,
                CONFIG / "render-profile.json",
            )
            text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")

        self.assertEqual(self.layout.data["revision"], DUAL_LANE_REVISION)
        self.assertEqual(manifest["physical_invariants"]["layout_bounds_xy_m"], [0.0, 0.0, 36.0, 28.0])
        self.assertEqual(text.count('stateRole = "directed_lane_arrow"'), 18)
        self.assertEqual(text.count('stateRole = "directed_lane_boundary"'), 9)
        for recipe in ("semiconductor_handler", "ate_rack", "aoi_mark", "esd_packing"):
            self.assertIn(f'assetRecipe = "{recipe}"', text)
        self.assertIn('rel material:binding = </World/Visuals/Looks/FloorEpoxy>', text)
        self.assertNotIn("PhysicsCollisionAPI", text)

    def test_runtime_signals_and_payload_slots_follow_relocated_equipment(self) -> None:
        obstacles = {str(item["id"]): item for item in self.layout.data["physical_obstacles"]}
        bindings = {
            "CI_0": "CI_BUFFER", "CI_1": "CI_BUFFER",
            "CO_0": "CO_BUFFER", "CO_1": "CO_BUFFER",
            "Cell": "PROCESS_2", "Complete": "FG_HIGH",
        }
        for signal, obstacle_id in bindings.items():
            x, y, z = signal_placements(self.layout)[signal]
            rect = Rect.from_value(obstacles[obstacle_id]["rect"], obstacle_id)
            self.assertTrue(rect.contains_point((x, y)), signal)
            self.assertGreater(z, 0)
            self.assertLessEqual(z, float(obstacles[obstacle_id]["height_m"]))

        expected = {
            "source_S1": "KIT_BENCH_1", "source_S2": "RACK_A2",
            "source_S3": "KIT_BENCH_2", "ci": "PROCESS_1",
            "cell": "PROCESS_2", "co": "PROCESS_3", "out": "FG_HIGH",
        }
        for location, obstacle_id in expected.items():
            placement = world_slot_placement(self.layout, location, 0)
            self.assertEqual(placement["source_obstacle_id"], obstacle_id)
            self.assertGreater(placement["capacity"], 0)

    def test_lane_visual_contract_rejects_duplicate_direction_ids(self) -> None:
        broken = deepcopy(self.layout.data)
        broken["traffic_lanes"][1]["id"] = broken["traffic_lanes"][0]["id"]
        proxy = deepcopy(self.layout)
        proxy.data = broken
        with self.assertRaisesRegex(LayoutError, "unsupported or duplicated"):
            _dual_lane_specs(proxy)


if __name__ == "__main__":
    unittest.main()
