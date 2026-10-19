"""Four registered cargo parents and translated production geometry."""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from factory_ops.fleet_transport import DEFAULT_CONFIG
from factory_ops.layout import Layout
from factory_ops.usda import export_usda
from factory_ops.visual_scene import export_visual_scene
from factory_ops.visual_state import SCOPE, canonical_sha256
from scripts.isaac.factory_visuals import build_payload_visibility_plan, validate_visual_state_document


class FleetTransportVisualTests(unittest.TestCase):
    def setUp(self):
        self.ids = tuple(f"carter{i}" for i in range(1, 5))
        self.paths = {robot: f"/World/{robot.title()}/chassis_link" for robot in self.ids}
        self.layout = Layout.from_file(DEFAULT_CONFIG / "layout.json")
        self.projection = {"scope": SCOPE, "physical_loading_validated": False, "ground_truth_control_input": False,
            "signals": {"ci_slots_used": 0, "co_slots_used": 0, "cell_processing": False, "orders_completed": 0},
            "payloads": [{"order_id": f"order-{robot}", "payload_id": f"lot-{robot}",
                "state": "inbound_loaded", "state_version": 2, "owner": robot,
                "location": f"robot_{robot}", "slot": 0, "robot_id": robot} for robot in self.ids]}
        self.document = {"schema_version": 1, "run_id": "run-1", "scenario_id": "normal", "sequence": 0,
            "emitted_sim_time_ns": 10, "ledger_seq": 20, "event_hash": "a" * 64,
            "projection": self.projection, "projection_sha256": canonical_sha256(self.projection)}

    def test_all_four_cargo_copies_have_their_declared_unique_parent(self):
        validate_visual_state_document(self.document, run_id="run-1", scenario_id="normal", robot_ids=self.ids)
        plan = build_payload_visibility_plan(self.layout, self.projection, self.paths)
        self.assertEqual(4, len(plan))
        self.assertEqual({path + "/FactoryVisualCargo" for path in self.paths.values()},
                         {row["placement"]["parent_path"] for row in plan})
        self.assertEqual(4, len({row["prim_path"] for row in plan}))
        self.assertFalse(self.projection["physical_loading_validated"])

    def test_default_two_robot_contract_rejects_undeclared_third_robot(self):
        with self.assertRaisesRegex(ValueError, "location or slot"):
            validate_visual_state_document(self.document, run_id="run-1", scenario_id="normal")

    def test_unknown_robot_and_cross_robot_parent_are_rejected(self):
        for robot_id, location in (("carter5", "robot_carter5"), ("carter4", "robot_carter3")):
            with self.subTest(robot_id=robot_id, location=location):
                projection = deepcopy(self.projection)
                projection["payloads"][0].update(robot_id=robot_id, location=location)
                with self.assertRaises(ValueError):
                    build_payload_visibility_plan(self.layout, projection, self.paths)

    def test_new_layout_preserves_production_recipe_and_all_four_parking_labels(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            export_usda(self.layout, output / "factory-v3.usda")
            manifest = export_visual_scene(self.layout, output,
                config_path=DEFAULT_CONFIG / "visual-assets.json", render_profile_path=DEFAULT_CONFIG / "render-profile.json")
            text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")
        for component in ("ShelfDeck_3", "Roller_7", "InspectionStage"):
            self.assertIn(component, text)
        physical = manifest["physical_invariants"]
        assert isinstance(physical, dict)
        self.assertEqual(10, physical["source_marker_count"])
        for parking in ("P1", "P2", "P3", "P4"):
            self.assertIn(f'Stencil_{parking}', text)
        self.assertNotIn("PhysicsCollisionAPI", text)


if __name__ == "__main__":
    unittest.main()
