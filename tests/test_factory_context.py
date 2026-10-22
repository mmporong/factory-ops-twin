from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.factory_context import build_factory_context
from factory_ops.layout import Layout, LayoutError


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-semiconductor" / "layout.json"
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


if __name__ == "__main__":
    unittest.main()
