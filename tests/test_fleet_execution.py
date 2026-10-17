"""Synthetic G3b adapter contracts; these tests make no Nav2 or physical-run claim."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import unittest

from factory_ops.fleet_execution import (
    CHUNK_SCOPE,
    PATH_SCOPE,
    StoppedGate,
    make_chunk,
    path_clock_ready,
    path_header_stamp_ns,
    qualify_computed_path,
)
from factory_ops.fleet_routing import RouteBook, swept_tiles
from factory_ops.layout import Layout
from factory_ops.navigation import GoalPose


ROOT = Path(__file__).resolve().parents[1]


class PathClockTests(unittest.TestCase):
    def test_separate_clock_defers_one_frame_then_admits_exact_integer_boundary(self):
        header = {"sec": 243, "nanosec": 950_000_000}
        self.assertEqual(243_950_000_000, path_header_stamp_ns(header))
        self.assertFalse(path_clock_ready(header, clock_ns=243_933_333_333,
                                         elapsed_wall_s=0., max_wait_wall_s=5.))
        self.assertFalse(path_clock_ready(header, clock_ns=243_949_999_999,
                                         elapsed_wall_s=.1, max_wait_wall_s=5.))
        self.assertTrue(path_clock_ready(header, clock_ns=243_950_000_000,
                                        elapsed_wall_s=.2, max_wait_wall_s=5.))
        self.assertTrue(path_clock_ready(header, clock_ns=243_950_000_000,
                                        elapsed_wall_s=5., max_wait_wall_s=5.))
        with self.assertRaises(TimeoutError):
            path_clock_ready(header, clock_ns=243_933_333_333, elapsed_wall_s=5., max_wait_wall_s=5.)

    def test_clock_gate_rejects_malformed_units_and_limits(self):
        for header in ({"sec": -1, "nanosec": 0}, {"sec": True, "nanosec": 0},
                       {"sec": 1, "nanosec": -1}, {"sec": 1, "nanosec": 1_000_000_000},
                       {"sec": 1, "nanosec": 1.}):
            with self.subTest(header=header), self.assertRaises(ValueError): path_header_stamp_ns(header)
        header = {"sec": 0, "nanosec": 1}
        for changes in ({"clock_ns": True}, {"clock_ns": -1}, {"elapsed_wall_s": -1},
                        {"max_wait_wall_s": 0}, {"elapsed_wall_s": math.nan}):
            values = {"clock_ns": 1, "elapsed_wall_s": 0., "max_wait_wall_s": 5.}; values.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError): path_clock_ready(header, **values)


class RouteChunkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.book = RouteBook.from_files(
            ROOT / "configs/factory-v3/layout.json",
            ROOT / "configs/factory-v3/g3-routing.json",
        )

    def goal(self, station: str, yaw: float = 0.0) -> GoalPose:
        x, y = self.book.nodes[station]
        return GoalPose("map", x, y, yaw)

    def test_first_edge_uses_planning_cover_and_intermediate_heading(self) -> None:
        chunk = make_chunk(
            self.book, "S1", "CI", closed=False, final_goal=self.goal("CI", 1.7)
        )
        self.assertEqual((chunk.start, chunk.goal), ("S1", "W_LOW"))
        self.assertEqual(chunk.points, (self.book.nodes["S1"], self.book.nodes["W_LOW"]))
        self.assertFalse(chunk.final)
        self.assertAlmostEqual(chunk.goal_pose.yaw, 0.0)
        self.assertEqual(
            chunk.tiles,
            swept_tiles(chunk.points, self.book.planning_radius_m),
        )
        self.assertEqual(
            chunk.route_id,
            make_chunk(
                self.book, "S1", "CI", closed=False, final_goal=self.goal("CI", 1.7)
            ).route_id,
        )
        self.assertEqual(len(chunk.route_id), 64)
        self.assertEqual(CHUNK_SCOPE, "g3b_authored_first_edge_planning_chunk_v1")

    def test_final_edge_preserves_requested_station_orientation_in_identity(self) -> None:
        first = make_chunk(
            self.book, "S1", "W_LOW", closed=False, final_goal=self.goal("W_LOW", 1.2)
        )
        changed = make_chunk(
            self.book, "S1", "W_LOW", closed=False, final_goal=self.goal("W_LOW", -1.2)
        )
        self.assertTrue(first.final)
        self.assertAlmostEqual(first.goal_pose.yaw, 1.2)
        self.assertNotEqual(first.route_id, changed.route_id)

    def test_current_closure_state_selects_the_plans_actual_first_edge(self) -> None:
        open_chunk = make_chunk(
            self.book, "W_MID", "CI", closed=False, final_goal=self.goal("CI")
        )
        closed_chunk = make_chunk(
            self.book, "W_MID", "CI", closed=True, final_goal=self.goal("CI")
        )
        self.assertEqual(open_chunk.goal, "W_LOW")
        self.assertEqual(closed_chunk.goal, "W_HIGH")
        self.assertAlmostEqual(open_chunk.goal_pose.yaw, -math.pi / 2)
        self.assertAlmostEqual(closed_chunk.goal_pose.yaw, math.pi / 2)
        self.assertNotEqual(open_chunk.route_id, closed_chunk.route_id)

    def test_zero_distance_chunk_preserves_orientation_and_qualifies_one_point(self) -> None:
        chunk = make_chunk(
            self.book, "S1", "S1", closed=False, final_goal=self.goal("S1", -2.1)
        )
        self.assertTrue(chunk.final)
        self.assertEqual(chunk.points, (self.book.nodes["S1"],))
        self.assertAlmostEqual(chunk.goal_pose.yaw, -2.1)
        result = qualify_computed_path(
            self.book,
            chunk,
            [self.book.nodes["S1"]],
            frame_id="map",
            stamp_s=5.0,
            now_s=5.2,
            start_xy=self.book.nodes["S1"],
        )
        self.assertTrue(result["passed"])

    def test_final_goal_position_must_match_target_anchor(self) -> None:
        with self.assertRaisesRegex(ValueError, "target anchor"):
            make_chunk(
                self.book,
                "S1",
                "W_LOW",
                closed=False,
                final_goal=GoalPose("map", 12.1, 4.0, 0.0),
            )


class ComputedPathQualificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.book = RouteBook.from_files(
            ROOT / "configs/factory-v3/layout.json",
            ROOT / "configs/factory-v3/g3-routing.json",
        )

    def goal(self, station: str, yaw: float = 0.0) -> GoalPose:
        x, y = self.book.nodes[station]
        return GoalPose("map", x, y, yaw)

    def setUp(self) -> None:
        self.chunk = make_chunk(
            self.book, "S1", "W_LOW", closed=False, final_goal=self.goal("W_LOW", 0.3)
        )

    def qualify(self, points, **changes):
        values = {
            "frame_id": "map",
            "stamp_s": 10.0,
            "now_s": 10.2,
            "start_xy": self.book.nodes["S1"],
        }
        values.update(changes)
        return qualify_computed_path(self.book, self.chunk, points, **values)

    def test_authored_geometry_returns_stable_hashes_and_declared_radii(self) -> None:
        result = self.qualify(self.chunk.points)
        self.assertTrue(result["passed"])
        self.assertEqual(result["scope"], PATH_SCOPE)
        self.assertEqual(result["planning_radius_m"], self.book.planning_radius_m)
        self.assertEqual(result["evaluation_radius_m"], self.book.evaluation_radius_m)
        self.assertEqual(len(result["points_sha256"]), 64)
        self.assertEqual(len(result["swept_tiles_sha256"]), 64)
        self.assertEqual(result, self.qualify(self.chunk.points))
        json.dumps(result, allow_nan=False)

    def test_clock_catchup_does_not_bypass_the_existing_stale_path_limit(self):
        header = {"sec": 10, "nanosec": 0}
        self.assertTrue(path_clock_ready(header, clock_ns=10_600_000_000,
                                        elapsed_wall_s=.1, max_wait_wall_s=5.))
        with self.assertRaisesRegex(ValueError, "stale"):
            self.qualify(self.chunk.points, stamp_s=path_header_stamp_ns(header)/1e9, now_s=10.6)

    def test_continuous_swept_cover_rejects_detour_between_valid_endpoints(self) -> None:
        start, goal = self.chunk.points
        with self.assertRaisesRegex(ValueError, "leaves reserved"):
            self.qualify([start, (8.0, 7.0), goal])

    def test_chunk_must_exactly_match_current_book_compilation(self) -> None:
        forged_values = (
            replace(self.chunk, route_id="0" * 64),
            replace(self.chunk, tiles=self.chunk.tiles | {(39, 23)}),
            replace(self.chunk, points=((4.0, 4.0), (12.0, 4.1))),
            replace(self.chunk, goal_pose=GoalPose("map", 12.0, 4.0, -0.3)),
            replace(self.chunk, target="CI", final_goal=self.goal("CI", 0.3)),
            replace(self.chunk, closed=True),
            replace(self.chunk, final_goal=GoalPose("map", 12.0, 4.0, -0.3)),
        )
        for forged in forged_values:
            with self.subTest(forged=forged), self.assertRaisesRegex(
                ValueError, "differs from the current authored"
            ):
                qualify_computed_path(
                    self.book,
                    forged,
                    self.chunk.points,
                    frame_id="map",
                    stamp_s=10.0,
                    now_s=10.2,
                    start_xy=self.book.nodes["S1"],
                )

        changed_config = deepcopy(self.book.config)
        changed_config["authored_allowances"]["localization_planning_margin_m"] += 0.1
        changed_book = RouteBook(
            Layout.from_file(ROOT / "configs/factory-v3/layout.json"), changed_config
        )
        with self.assertRaisesRegex(ValueError, "differs from the current authored"):
            qualify_computed_path(
                changed_book,
                self.chunk,
                self.chunk.points,
                frame_id="map",
                stamp_s=10.0,
                now_s=10.2,
                start_xy=self.book.nodes["S1"],
            )

    def test_endpoint_tolerances_are_checked_independently(self) -> None:
        start, goal = self.chunk.points
        with self.assertRaisesRegex(ValueError, "authored chunk start"):
            self.qualify([start, goal], start_xy=(4.2, 4.0))
        with self.assertRaisesRegex(ValueError, "current start"):
            self.qualify([(4.2, 4.0), goal])
        with self.assertRaisesRegex(ValueError, "chunk goal"):
            self.qualify([start, (11.8, 4.0)])

    def test_frame_and_timestamp_contract_is_fail_closed(self) -> None:
        for changes in (
            {"frame_id": "odom"},
            {"stamp_s": 10.3},
            {"stamp_s": 9.6},
            {"stamp_s": math.nan},
            {"now_s": math.inf},
            {"max_age_s": -0.1},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.qualify(self.chunk.points, **changes)

    def test_empty_short_and_malformed_geometry_is_rejected(self) -> None:
        for points in (
            [],
            [self.chunk.points[0]],
            [(4.0, 4.0, 0.0), self.chunk.points[1]],
            [(math.nan, 4.0), self.chunk.points[1]],
            [(4.0, 4.0), (41.0, 4.0), self.chunk.points[1]],
        ):
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.qualify(points)


class StoppedGateTests(unittest.TestCase):
    def test_distinct_stationary_samples_must_span_window(self) -> None:
        gate = StoppedGate()
        self.assertFalse(gate.update(0.0, 0.0, stamp_s=0.0, now_s=0.0))
        self.assertFalse(gate.update(0.01, 0.02, stamp_s=0.25, now_s=0.25))
        self.assertTrue(gate.update(-0.01, -0.02, stamp_s=0.5, now_s=0.5))
        self.assertTrue(gate.is_stopped(now_s=0.79))

    def test_repeated_timestamp_never_manufactures_stationary_duration(self) -> None:
        gate = StoppedGate()
        for now in (0.0, 0.1, 0.2, 0.3):
            self.assertFalse(gate.update(0.0, 0.0, stamp_s=0.0, now_s=now))
        self.assertFalse(gate.is_stopped(now_s=0.31))
        self.assertFalse(gate.update(0.0, 0.0, stamp_s=0.31, now_s=0.31))

    def test_stationary_evidence_gap_cannot_exceed_ttl(self) -> None:
        exact = StoppedGate()
        exact.update(0.0, 0.0, stamp_s=0.0, now_s=0.0)
        self.assertFalse(exact.update(0.0, 0.0, stamp_s=0.3, now_s=0.3))
        self.assertTrue(exact.update(0.0, 0.0, stamp_s=0.5, now_s=0.5))

        over = StoppedGate()
        over.update(0.0, 0.0, stamp_s=0.0, now_s=0.0)
        self.assertFalse(over.update(0.0, 0.0, stamp_s=0.300001, now_s=0.300001))
        self.assertFalse(over.update(0.0, 0.0, stamp_s=0.5, now_s=0.5))

        sparse = StoppedGate()
        sparse.update(0.0, 0.0, stamp_s=0.0, now_s=0.0)
        self.assertFalse(sparse.update(0.0, 0.0, stamp_s=0.5, now_s=0.5))
        self.assertFalse(sparse.update(0.0, 0.0, stamp_s=0.8, now_s=0.8))
        self.assertTrue(sparse.update(0.0, 0.0, stamp_s=1.0, now_s=1.0))

    def test_movement_and_stale_observation_reset_accumulation(self) -> None:
        gate = StoppedGate()
        gate.update(0.0, 0.0, stamp_s=1.0, now_s=1.0)
        gate.update(0.0, 0.0, stamp_s=1.3, now_s=1.3)
        gate.update(0.0, 0.0, stamp_s=1.5, now_s=1.5)
        self.assertTrue(gate.is_stopped(now_s=1.5))
        self.assertFalse(gate.update(0.021, 0.0, stamp_s=1.6, now_s=1.6))
        self.assertFalse(gate.update(0.0, 0.0, stamp_s=1.7, now_s=1.7))
        self.assertFalse(gate.update(0.0, 0.0, stamp_s=1.7, now_s=2.01))
        self.assertFalse(gate.update(0.0, 0.0, stamp_s=2.02, now_s=2.02))

    def test_future_nan_and_nonmonotonic_samples_fail_closed(self) -> None:
        gate = StoppedGate()
        gate.update(0.0, 0.0, stamp_s=2.0, now_s=2.0)
        with self.assertRaises(ValueError):
            gate.update(0.0, 0.0, stamp_s=2.2, now_s=2.1)
        self.assertFalse(gate.is_stopped(now_s=2.1))
        with self.assertRaises(ValueError):
            gate.update(math.nan, 0.0, stamp_s=2.1, now_s=2.1)
        self.assertFalse(gate.is_stopped(now_s=2.1))
        gate.update(0.0, 0.0, stamp_s=3.0, now_s=3.0)
        with self.assertRaises(ValueError):
            gate.update(0.0, 0.0, stamp_s=2.9, now_s=3.0)
        self.assertFalse(gate.is_stopped(now_s=3.0))

    def test_motion_or_stale_window_reset_preserves_observation_chronology(self) -> None:
        for moving in (True, False):
            with self.subTest(moving=moving):
                gate = StoppedGate()
                gate.update(.1 if moving else 0., 0., stamp_s=10., now_s=10. if moving else 10.4)
                with self.assertRaisesRegex(ValueError, "monotonic"):
                    gate.update(0., 0., stamp_s=9.9, now_s=10. if moving else 10.4)
                gate.update(0., 0., stamp_s=10.4, now_s=10.4)
                self.assertFalse(gate.update(0., 0., stamp_s=10.6, now_s=10.6))
                self.assertFalse(gate.update(0., 0., stamp_s=10.8, now_s=10.8))
                self.assertTrue(gate.update(0., 0., stamp_s=10.95, now_s=10.95))

    def test_same_stamp_with_changed_velocity_is_contaminated_replay(self) -> None:
        gate = StoppedGate()
        gate.update(.1, 0., stamp_s=10., now_s=10.)
        with self.assertRaisesRegex(ValueError, "different velocity"):
            gate.update(0., 0., stamp_s=10., now_s=10.)
        self.assertFalse(gate.is_stopped(now_s=10.))

    def test_exported_stop_samples_exclude_braking_and_disconnected_windows(self) -> None:
        gate = StoppedGate()
        gate.update(.1, 0., stamp_s=1., now_s=1.)
        self.assertEqual([], gate.samples)
        gate.update(0., 0., stamp_s=1.1, now_s=1.1)
        gate.update(0., 0., stamp_s=1.5, now_s=1.5)
        self.assertEqual([1.5], [row["stamp_s"] for row in gate.samples])
        copied = gate.samples; copied[0]["linear_m_s"] = 99.
        self.assertEqual(0., gate.samples[0]["linear_m_s"])

    def test_reset_and_constructor_validation(self) -> None:
        gate = StoppedGate(window_s=0.1, ttl_s=0.2)
        gate.update(0.0, 0.0, stamp_s=1.0, now_s=1.0)
        gate.update(0.0, 0.0, stamp_s=1.1, now_s=1.1)
        self.assertTrue(gate.is_stopped(now_s=1.1))
        gate.reset()
        self.assertFalse(gate.is_stopped(now_s=0.0))
        for kwargs in (
            {"window_s": 0.0},
            {"max_linear_m_s": -0.1},
            {"max_angular_rad_s": math.nan},
            {"ttl_s": 0.0},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                StoppedGate(**kwargs)


if __name__ == "__main__":
    unittest.main()
