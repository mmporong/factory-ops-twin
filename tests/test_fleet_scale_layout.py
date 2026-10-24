import json
import math
from pathlib import Path
import unittest

from factory_ops.fleet_registry import FleetRegistry
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.fleet_scale_layout import build_scale_bundle, minimum_home_spacing


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "configs" / "factory-semiconductor-dual-lane"


class FleetScaleLayoutTests(unittest.TestCase):
    def test_checked_in_bundles_are_deterministic_and_preserve_production_geometry(self):
        base_layout = json.loads((BASE / "layout.json").read_text(encoding="utf-8"))
        base_routing = json.loads((BASE / "g3-routing.json").read_text(encoding="utf-8"))
        base_g2 = json.loads((BASE / "g2.json").read_text(encoding="utf-8"))
        baseline_bytes = {path.name: path.read_bytes() for path in BASE.iterdir() if path.is_file()}

        for count in (12, 16):
            with self.subTest(fleet_count=count):
                target = ROOT / "configs" / f"factory-semiconductor-scale{count}"
                generated = build_scale_bundle(BASE, count)
                for name, expected in generated.items():
                    self.assertEqual(expected, json.loads((target / name).read_text(encoding="utf-8")))
                layout = generated["layout.json"]
                routing = generated["g3-routing.json"]
                self.assertEqual(base_layout["stations"], layout["stations"])
                self.assertEqual(base_layout["physical_obstacles"], layout["physical_obstacles"])
                self.assertEqual(base_layout["blocked_aisle"], layout["blocked_aisle"])
                self.assertTrue({tuple(edge) for edge in base_routing["edges"]}.issubset(
                    {tuple(edge) for edge in routing["edges"]}))
                self.assertTrue({tuple(edge) for edge in base_routing["directed_edges"]}.issubset(
                    {tuple(edge) for edge in routing["directed_edges"]}))
                self.assertEqual(count, generated["fleet-profile.json"]["fleet_count"])
                self.assertEqual(
                    {f"amr-{index}" for index in range(1, count + 1)},
                    set(generated["operations-charging.json"]["initial_soc"]),
                )
                spawns = {
                    f"carter{index}": [*layout["pockets"][f"P{index}"], 0.0]
                    for index in range(1, count + 1)
                }
                registry = FleetRegistry(generated["g2.json"], {"spawns": spawns})
                self.assertEqual(count, len(registry.robot_ids))
                self.assertEqual(count * 2, registry.manifest()["lidar_count"])
                for key in (
                    "physics_dt_s", "encoder_publish_every_steps", "command_timeout_s",
                    "max_linear_m_s", "max_angular_rad_s", "max_linear_accel_m_s2",
                    "max_angular_accel_rad_s2", "drive_profile", "drive_geometry", "lidars",
                ):
                    self.assertEqual(base_g2[key], generated["g2.json"][key])
                self.assertGreaterEqual(minimum_home_spacing(generated["fleet-profile.json"], layout), 4.0)

        self.assertEqual(baseline_bytes, {path.name: path.read_bytes() for path in BASE.iterdir() if path.is_file()})

    def test_every_home_has_open_and_closed_routes_and_disjoint_stationary_reservations(self):
        for count in (12, 16):
            with self.subTest(fleet_count=count):
                config = ROOT / "configs" / f"factory-semiconductor-scale{count}"
                book = RouteBook.from_files(config / "layout.json", config / "g3-routing.json")
                homes = [f"P{index}" for index in range(1, count + 1)]
                bounds = (0.0, 0.0, book.layout.bounds.xmax, book.layout.bounds.ymax)
                covers = {
                    home: swept_tiles(
                        [book.nodes[home]], book.planning_radius_m,
                        tile_size_m=book.tile_size_m, bounds_xy=bounds,
                    )
                    for home in homes
                }
                for index, home in enumerate(homes):
                    for closed in (False, True):
                        outbound = book.plan(home, "CI", closed=closed)
                        inbound = book.plan("CI", home, closed=closed)
                        self.assertEqual(home, outbound.nodes[0])
                        self.assertEqual("CI", outbound.nodes[-1])
                        self.assertEqual("CI", inbound.nodes[0])
                        self.assertEqual(home, inbound.nodes[-1])
                    for other in homes[index + 1:]:
                        self.assertFalse(covers[home] & covers[other], (home, other))
                        self.assertGreater(
                            math.dist(book.nodes[home], book.nodes[other]),
                            2 * book.planning_radius_m,
                        )

    def test_annex_westbound_chains_run_from_far_turnaround_back_to_existing_lane(self):
        for count in (12, 16):
            routing = build_scale_bundle(BASE, count)["g3-routing.json"]
            directed = {tuple(edge) for edge in routing["directed_edges"]}
            for prefix, entry, pockets in (
                ("LOW_W", "LOW_W_EN", list(range(5, count + 1, 2))),
                ("HIGH_W", "HIGH_W_EN", list(range(6, count + 1, 2))),
            ):
                far = pockets[-1]
                self.assertNotIn((entry, f"{prefix}_P{far}"), directed)
                chain = [f"{prefix}_P{index}" for index in reversed(pockets)] + [entry]
                self.assertTrue(all(edge in directed for edge in zip(chain, chain[1:])))


if __name__ == "__main__":
    unittest.main()
