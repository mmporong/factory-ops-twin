from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from factory_ops.factory_context import build_factory_context
from factory_ops.layout import Layout, LayoutError
from factory_ops.usda import export_usda
from factory_ops.visual_scene import _external_asset_mapping, export_visual_scene


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-semiconductor" / "layout.json"
VISUAL_CONFIG_PATH = ROOT / "configs" / "factory-semiconductor" / "visual-assets.json"
RENDER_PROFILE_PATH = ROOT / "configs" / "factory-semiconductor" / "render-profile.json"
REVISION = "1" * 40
SOURCE_SHA = "2" * 64


class FactoryContextTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(LAYOUT_PATH)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.cache = Path(self.temporary.name) / "cache"
        package = self.cache / REVISION / "furniture_workbench_modular_a01"
        package.mkdir(parents=True)
        self.prepared = package / "simready_usd" / "factory_renderonly.usda"
        self.prepared.parent.mkdir()
        self.prepared.write_text('#usda 1.0\ndef Xform "Asset" {}\n', encoding="utf-8")
        self.prepared_sha = hashlib.sha256(self.prepared.read_bytes()).hexdigest()
        index = {
            "revision": REVISION,
            "status": "passed",
            "assets": {
                "workbench": {
                    "relative_path": "furniture_workbench_modular_a01/simready_usd/factory_renderonly.usda",
                    "sha256": self.prepared_sha,
                    "bounds_xyz_m": [-0.6, -0.3, 0, 0.6, 0.3, 1.1],
                    "scope": "render-only; physics APIs and typed prims removed",
                    "source_files": [{"path": "simready_usd/source.usd", "sha256": SOURCE_SHA, "size": 31}],
                }
            },
        }
        (self.cache / REVISION / "prepared-index.json").write_text(
            json.dumps(index), encoding="utf-8",
        )
        self.environment = {
            "revision": REVISION,
            "repository": "nvidia/simready-assets",
            "catalog": {
                "workbench": {
                    "package": "furniture_workbench_modular_a01",
                    "root_usd": "simready_usd/sm_furniture_workbench_modular_a01_01.usd",
                    "author": "NVIDIA",
                    "license": "CC-BY-4.0",
                    "license_url": "https://creativecommons.org/licenses/by/4.0/",
                    "source_url": "https://github.com/NVIDIA/simready-assets",
                }
            },
            "fixtures": [{
                "id": "SERVICE_WORKBENCH_1",
                "asset": "workbench",
                "center_xy": [8.3, 1.4],
                "yaw_deg": 90,
                "role": "maintenance workbench",
                "semantic_class": "workbench",
            }],
        }

    def test_builds_measured_render_only_reference_and_provenance(self) -> None:
        result = build_factory_context(self.layout, self.environment, cache_root=self.cache)

        text = "\n".join(result["lines"])
        self.assertIn('def Xform "FactoryContext"', text)
        self.assertIn("prepend references = @", text)
        self.assertIn("@</Asset>", text)
        self.assertIn("instanceable = true", text)
        for measured, expected in zip(
            result["instances"][0]["bounds_xyz_m"],
            (8.0, 0.8, 0.0, 8.6, 2.0, 1.1),
        ):
            self.assertAlmostEqual(expected, measured)
        self.assertEqual("nvidia:workbench", result["instances"][0]["asset"])
        self.assertEqual(self.prepared_sha, result["packages"][0]["sha256"])
        self.assertEqual(SOURCE_SHA, result["packages"][0]["source_files"][0]["sha256"])
        self.assertIn("no physics", result["scope"])

    def test_rejects_fixture_on_amr_route_or_existing_equipment(self) -> None:
        for center in ([8.3, 4.5], [6.3, 2.0]):
            with self.subTest(center=center):
                environment = copy.deepcopy(self.environment)
                environment["fixtures"][0]["center_xy"] = center
                with self.assertRaisesRegex(LayoutError, "0.1m clearance"):
                    build_factory_context(self.layout, environment, cache_root=self.cache)

    def test_rejects_cache_traversal_and_hash_mismatch(self) -> None:
        index_path = self.cache / REVISION / "prepared-index.json"
        index = json.loads(index_path.read_text(encoding="utf-8"))
        for relative_path, sha256, message in (
            ("../outside.usda", self.prepared_sha, "remain inside"),
            ("furniture_workbench_modular_a01/simready_usd/factory_renderonly.usda", "f" * 64, "SHA-256 mismatch"),
        ):
            with self.subTest(message=message):
                changed = copy.deepcopy(index)
                changed["assets"]["workbench"]["relative_path"] = relative_path
                changed["assets"]["workbench"]["sha256"] = sha256
                index_path.write_text(json.dumps(changed), encoding="utf-8")
                with self.assertRaisesRegex(LayoutError, message):
                    build_factory_context(self.layout, self.environment, cache_root=self.cache)

    def test_mounts_external_asset_inside_source_obstacle_envelope(self) -> None:
        layout = self._layout_with_fg_low_height(2.4)
        environment = self._mounted_environment()
        self._set_prepared_bounds([-0.771, -2.0, 0, 0.771, 2.0, 2.311])

        result = build_factory_context(layout, environment, cache_root=self.cache)

        instance = result["instances"][0]
        self.assertEqual("FG_LOW", instance["source_obstacle_id"])
        self.assertEqual([26.8, 6.8, 31.4, 9.3], list(instance["source_bounds_xy_m"]))
        for measured, expected in zip(
            instance["bounds_xyz_m"], (27.1, 7.279, 0.0, 31.1, 8.821, 2.311),
        ):
            self.assertAlmostEqual(expected, measured)
        self.assertIs(instance, result["mounted_by_obstacle"]["FG_LOW"])
        text = "\n".join(result["lines"])
        self.assertIn('custom string sourceObstacleId = "FG_LOW"', text)
        self.assertNotIn("PhysicsCollisionAPI", text)
        self.assertNotIn("physics:collisionEnabled", text)
        self.assertIn("inside source obstacle envelopes", result["scope"])

    def test_rejects_missing_oversize_and_duplicate_source_mounts(self) -> None:
        layout = self._layout_with_fg_low_height(2.4)
        self._set_prepared_bounds([-0.771, -2.0, 0, 0.771, 2.0, 2.311])
        for source_id, message in (("MISSING", "missing source obstacle"),):
            with self.subTest(source_id=source_id):
                environment = self._mounted_environment()
                environment["fixtures"][0]["source_obstacle_id"] = source_id
                with self.assertRaisesRegex(LayoutError, message):
                    build_factory_context(layout, environment, cache_root=self.cache)

        self._set_prepared_bounds([-0.771, -2.4, 0, 0.771, 2.4, 2.311])
        with self.assertRaisesRegex(LayoutError, "exceeds source obstacle FG_LOW XYZ envelope"):
            build_factory_context(layout, self._mounted_environment(), cache_root=self.cache)

        self._set_prepared_bounds([-0.771, -2.0, 0, 0.771, 2.0, 2.311])
        environment = self._mounted_environment()
        duplicate = copy.deepcopy(environment["fixtures"][0])
        duplicate["id"] = "SHIPPING_GATE_02"
        environment["fixtures"].append(duplicate)
        with self.assertRaisesRegex(LayoutError, "duplicate factory context fixtures"):
            build_factory_context(layout, environment, cache_root=self.cache)

    def test_external_assignment_requires_exact_semantic_mount_mapping(self) -> None:
        config = {
            "obstacle_assets": {
                "FG_LOW": {
                    "recipe": "external_asset",
                    "semantic_class": "process_equipment",
                    "state_role": "shipping packaging inspection gate",
                },
            },
        }
        with self.assertRaisesRegex(LayoutError, r"missing=\['FG_LOW'\]"):
            _external_asset_mapping(config, None)

        instance = {
            "semantic_class": "process_equipment",
            "state_role": "wrong role",
        }
        with self.assertRaisesRegex(LayoutError, "semantic_class/state_role mismatch"):
            _external_asset_mapping(config, {"mounted_by_obstacle": {"FG_LOW": instance}})

        with self.assertRaisesRegex(LayoutError, r"unexpected=\['FG_LOW'\]"):
            _external_asset_mapping(
                {"obstacle_assets": {"FG_LOW": {"recipe": "esd_packing"}}},
                {"mounted_by_obstacle": {"FG_LOW": instance}},
            )

    def test_external_mount_replaces_one_recipe_and_preserves_physical_scene(self) -> None:
        layout = self._layout_with_fg_low_height(2.4)
        environment = self._mounted_environment()
        self._set_prepared_bounds([-0.771, -2.0, 0, 0.771, 2.0, 2.311])
        config = json.loads(VISUAL_CONFIG_PATH.read_text(encoding="utf-8"))
        config["environment"] = environment
        config["obstacle_assets"]["FG_LOW"] = {
            "recipe": "external_asset",
            "semantic_class": "process_equipment",
            "state_role": "shipping packaging inspection gate",
        }
        config_path = Path(self.temporary.name) / "visual-assets.json"
        config_path.write_text(json.dumps(config), encoding="utf-8")
        output = Path(self.temporary.name) / "scene"
        export_usda(layout, output / "factory-v3.usda")

        with mock.patch.dict(os.environ, {"FACTORY_OPS_ASSET_CACHE": str(self.cache)}):
            manifest = export_visual_scene(
                layout, output, config_path=config_path,
                render_profile_path=RENDER_PROFILE_PATH,
            )

        instances = manifest["instances"]
        obstacle_ids = {item["id"] for item in layout.data["physical_obstacles"]}
        obstacle_roots = [item for item in instances if item["stable_id"] in obstacle_ids]
        mounted = [item for item in instances if item["source_obstacle_id"] == "FG_LOW"]
        self.assertEqual(18, len(obstacle_roots))
        self.assertEqual(["SHIPPING_GATE_01"], [item["stable_id"] for item in mounted])
        self.assertEqual("nvidia:workbench", mounted[0]["asset"])
        self.assertEqual(19, manifest["physical_invariants"]["source_obstacle_count"])
        self.assertEqual(20, manifest["physical_invariants"]["source_static_collider_count"])
        self.assertEqual(0, manifest["physical_invariants"]["visual_collision_api_count"])
        visual_text = (output / "factory-v3.visual.usda").read_text(encoding="utf-8")
        self.assertNotIn('def Xform "FG_LOW"', visual_text)
        self.assertNotIn("PhysicsCollisionAPI", visual_text)
        composition = (output / "factory-v3.render.usda").read_text(encoding="utf-8")
        self.assertEqual(19, composition.count('token visibility = "invisible"'))

    def _mounted_environment(self) -> dict[str, object]:
        environment = copy.deepcopy(self.environment)
        environment["fixtures"] = [{
            "id": "SHIPPING_GATE_01",
            "asset": "workbench",
            "center_xy": [29.1, 8.05],
            "yaw_deg": 90,
            "role": "shipping packaging inspection gate",
            "semantic_class": "process_equipment",
            "source_obstacle_id": "FG_LOW",
        }]
        return environment

    def _layout_with_fg_low_height(self, height: float) -> Layout:
        data = copy.deepcopy(self.layout.data)
        obstacle = next(item for item in data["physical_obstacles"] if item["id"] == "FG_LOW")
        obstacle["height_m"] = height
        return Layout(data)

    def _set_prepared_bounds(self, bounds: list[float]) -> None:
        index_path = self.cache / REVISION / "prepared-index.json"
        index = json.loads(index_path.read_text(encoding="utf-8"))
        index["assets"]["workbench"]["bounds_xyz_m"] = bounds
        index_path.write_text(json.dumps(index), encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
