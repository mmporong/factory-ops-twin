from __future__ import annotations

from pathlib import Path
import unittest

from factory_ops.operations_model import OperationsModel
from factory_ops.operations_replan import anchor_at_pose, find_alternative_route


CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-semiconductor-dual-lane"


class OperationsReplanTest(unittest.TestCase):
    def setUp(self) -> None:
        self.model = OperationsModel(CONFIG, initial_orders=0, run_id="operations-replan-unit")

    def test_peer_footprints_force_east_spine_detour(self) -> None:
        book = self.model.book
        peer_tiles = (
            self.model._tiles((book.nodes["LOW_W_P1"],))
            | self.model._tiles((book.nodes["LOW_W_P3"],))
        )
        existing = book.plan("LOW_E_P1", "P4", closed=False)
        alternative = find_alternative_route(
            book,
            start="LOW_E_P1",
            goal="P4",
            closed=False,
            reservation_radius_m=self.model.radius,
            peer_footprint_tiles=peer_tiles,
            existing_points=existing.points[1:],
        )
        self.assertIsNotNone(alternative)
        assert alternative is not None
        self.assertNotEqual(alternative.points[1:], existing.points[1:])
        self.assertIn("EN_OUT", alternative.nodes)
        self.assertIn("HIGH_W_EN", alternative.nodes)
        self.assertEqual(alternative.nodes[0], "LOW_E_P1")
        self.assertEqual(alternative.nodes[-1], "P4")

    def test_blocked_goal_has_no_detour_and_anchor_requires_exact_stop(self) -> None:
        book = self.model.book
        goal_tiles = self.model._tiles((book.nodes["P4"],))
        existing = book.plan("LOW_E_P1", "P4", closed=False)
        self.assertIsNone(find_alternative_route(
            book,
            start="LOW_E_P1",
            goal="P4",
            closed=False,
            reservation_radius_m=self.model.radius,
            peer_footprint_tiles=goal_tiles,
            existing_points=existing.points[1:],
        ))
        self.assertEqual(
            anchor_at_pose(book.nodes, {"x_m": 12.0, "y_m": 4.5}),
            "LOW_E_P1",
        )
        self.assertIsNone(anchor_at_pose(book.nodes, {"x_m": 12.01, "y_m": 4.5}))

    def test_existing_shortest_path_is_skipped_when_an_alternative_exists(self) -> None:
        book = self.model.book
        existing = book.plan("LOW_E_P1", "P4", closed=False)
        alternative = find_alternative_route(
            book,
            start="LOW_E_P1",
            goal="P4",
            closed=False,
            reservation_radius_m=self.model.radius,
            peer_footprint_tiles=(),
            existing_points=existing.points[1:],
        )
        self.assertIsNotNone(alternative)
        assert alternative is not None
        self.assertNotEqual(alternative.points[1:], existing.points[1:])


if __name__ == "__main__":
    unittest.main()
