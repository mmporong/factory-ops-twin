from __future__ import annotations

from pathlib import Path
import re
import tempfile
import unittest

from factory_ops.layout import Layout
from factory_ops.usda import export_usda


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"


class UsdaTests(unittest.TestCase):
    def test_usda_contains_expected_metadata_colliders_and_markers(self) -> None:
        layout = Layout.from_file(LAYOUT_PATH)
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "factory.usda"
            manifest = export_usda(layout, target)
            text = target.read_text(encoding="utf-8")
        self.assertTrue(text.startswith("#usda 1.0\n"))
        self.assertIn('defaultPrim = "World"', text)
        self.assertIn('upAxis = "Z"', text)
        self.assertIn("metersPerUnit = 1", text)
        self.assertEqual(text.count('prepend apiSchemas = ["PhysicsCollisionAPI"]'), 20)
        self.assertEqual(text.count("physics:collisionEnabled = true"), 20)
        self.assertEqual(manifest["obstacle_collider_count"], 19)
        self.assertEqual(manifest["non_colliding_marker_count"], 8)
        self.assertNotIn("Nova_Carter", text)

    def test_obstacle_positions_and_floor_extent_are_derived_from_geometry(self) -> None:
        layout = Layout.from_file(LAYOUT_PATH)
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "factory.usda"
            export_usda(layout, target)
            text = target.read_text(encoding="utf-8")
        floor = re.search(r'def Cube "Floor".*?xformOp:scale = \(([^)]+)\).*?xformOp:translate = \(([^)]+)\)', text, re.S)
        if floor is None:
            self.fail("exported floor transform is missing")
        self.assertEqual(floor.group(1), "20, 12, 0.05")
        self.assertEqual(floor.group(2), "20, 12, -0.05")
        rack = re.search(r'def Cube "RACK_A1".*?xformOp:scale = \(([^)]+)\).*?xformOp:translate = \(([^)]+)\)', text, re.S)
        if rack is None:
            self.fail("exported rack transform is missing")
        self.assertEqual(rack.group(1), "3.5, 0.5, 1.05")
        self.assertEqual(rack.group(2), "6.5, 6.75, 1.05")

    def test_markers_are_non_colliding_and_preserve_authored_coordinates(self) -> None:
        layout = Layout.from_file(LAYOUT_PATH)
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "factory.usda"
            export_usda(layout, target)
            text = target.read_text(encoding="utf-8")
        marker_scope = text.split('def Scope "Markers"', 1)[1]
        self.assertNotIn("PhysicsCollisionAPI", marker_scope)
        self.assertIsNotNone(re.search(r'def Xform "S1".*?xformOp:translate = \(4, 4, 0\)', marker_scope, re.S))
        self.assertIsNotNone(re.search(r'def Xform "P2".*?xformOp:translate = \(16, 22\.5, 0\)', marker_scope, re.S))


if __name__ == "__main__":
    unittest.main()
