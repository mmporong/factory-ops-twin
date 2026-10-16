from __future__ import annotations

from pathlib import Path
import unittest

from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout, LayoutError


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"


class FootprintQualificationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(LAYOUT_PATH)

    def test_small_measured_bounds_fit_all_authored_anchors(self) -> None:
        result = qualify_footprint(
            self.layout,
            ((-0.2, -0.2, 0.0), (0.2, 0.2, 0.6)),
            clearance_m=0.2,
        )

        self.assertTrue(result["passed"])
        self.assertEqual(set(result["anchors"]), {"S1", "S2", "S3", "CI", "CO", "OUT", "P1", "P2"})
        self.assertTrue(all(anchor["blocked_cell_count"] == 0 for anchor in result["anchors"].values()))
        self.assertAlmostEqual(result["collision_radius_m"], 2**0.5 * 0.2)
        self.assertIn("circular yaw envelope", result["method"])
        self.assertNotIn("29x29", result["method"])

    def test_wide_measured_bounds_fail_constrained_anchors(self) -> None:
        result = qualify_footprint(
            self.layout,
            ((-1.5, -1.5, 0.0), (1.5, 1.5, 1.0)),
            clearance_m=0.2,
        )

        self.assertFalse(result["passed"])
        self.assertTrue(any(not anchor["valid"] for anchor in result["anchors"].values()))
        self.assertTrue(any(anchor["blocked_cell_count"] > 0 for anchor in result["anchors"].values()))

    def test_root_offset_uses_farthest_corner_for_yaw_conservative_radius(self) -> None:
        result = qualify_footprint(
            self.layout,
            ((0.5, -0.2, -0.1), (1.0, 0.3, 0.8)),
            clearance_m=0.15,
        )

        self.assertAlmostEqual(result["collision_radius_m"], (1.0**2 + 0.3**2) ** 0.5)
        self.assertAlmostEqual(result["inflated_radius_m"], (1.0**2 + 0.3**2) ** 0.5 + 0.15)
        self.assertEqual(result["collision_bounds_xyz"], [[0.5, -0.2, -0.1], [1.0, 0.3, 0.8]])

    def test_circle_touching_a_blocked_cell_boundary_is_rejected(self) -> None:
        layout = self._layout_with_single_anchor_geometry(
            allowed_rect=[3.0, 3.0, 5.0, 5.0],
            obstacle_rect=[4.5, 4.0, 4.55, 4.05],
        )
        result = qualify_footprint(
            layout,
            ((-0.3, -0.4, 0.0), (0.3, 0.4, 0.5)),
            clearance_m=0.0,
        )

        self.assertFalse(result["anchors"]["S1"]["valid"])
        self.assertIn([90, 80], result["anchors"]["S1"]["sample_bad_cells"])

    def test_keepout_cell_touched_by_circle_is_rejected(self) -> None:
        layout = self._layout_with_single_anchor_geometry(
            allowed_rect=[3.0, 3.0, 4.5, 5.0],
        )
        result = qualify_footprint(
            layout,
            ((-0.3, -0.4, 0.0), (0.3, 0.4, 0.5)),
            clearance_m=0.0,
        )

        self.assertFalse(result["anchors"]["S1"]["valid"])
        self.assertIn([90, 80], result["anchors"]["S1"]["sample_bad_cells"])

    def test_circle_reaching_outside_map_is_rejected(self) -> None:
        layout = self._layout_with_single_anchor_geometry(
            allowed_rect=[0.0, 0.0, 5.0, 5.0],
            station_xy=[0.25, 0.25],
        )
        result = qualify_footprint(
            layout,
            ((-0.3, -0.01, 0.0), (0.3, 0.01, 0.5)),
            clearance_m=0.0,
        )

        self.assertFalse(result["anchors"]["S1"]["valid"])
        self.assertTrue(any(col < 0 or row < 0 for col, row in result["anchors"]["S1"]["sample_bad_cells"]))

    def test_invalid_bounds_and_clearance_are_rejected(self) -> None:
        invalid_cases = (
            (((0.0, 0.0, 0.0), (0.0, 1.0, 1.0)), 0.2),
            (((0.0, 0.0, 0.0), (1.0, float("nan"), 1.0)), 0.2),
            (((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)), -0.01),
            (((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)), float("nan")),
        )
        for bounds, clearance in invalid_cases:
            with self.subTest(bounds=bounds, clearance=clearance):
                with self.assertRaises(LayoutError):
                    qualify_footprint(self.layout, bounds, clearance)

    def _layout_with_single_anchor_geometry(
        self,
        *,
        allowed_rect: list[float],
        obstacle_rect: list[float] | None = None,
        station_xy: list[float] | None = None,
    ) -> Layout:
        # The production Layout contract freezes anchor positions and obstacle
        # count. Build normally, then alter only the raster and test anchor to
        # exercise qualification geometry without weakening that contract.
        layout = Layout.from_file(LAYOUT_PATH)
        layout.allowed_regions = tuple(type(layout.bounds)(*allowed_rect) for _ in range(1))
        if obstacle_rect is not None:
            obstacles = list(layout.obstacles)
            obstacles[0] = type(layout.bounds)(*obstacle_rect)
            layout.obstacles = tuple(obstacles)
        layout._raw_normal = layout._make_raw_mask()
        if station_xy is not None:
            layout.stations = {**layout.stations, "S1": tuple(station_xy)}
        return layout


if __name__ == "__main__":
    unittest.main()
