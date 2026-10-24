import json
import math
from pathlib import Path
import unittest

from factory_ops.campus_layout import build_campus_bundle, build_wide_bridge_campus_bundle
from factory_ops.factory_context import build_factory_context
from factory_ops.fleet_routing import RouteBook, _bad_touched_cells
from factory_ops.layout import Layout
from factory_ops.operations_model import OperationsModel
from factory_ops.route_reservation import RouteReservationManager


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "configs" / "factory-semiconductor-dual-lane"
TARGET = ROOT / "configs" / "factory-semiconductor-campus16-wide-bridges"
ORIGINAL = ROOT / "configs" / "factory-semiconductor-campus16"
WIDE_VARIANTS = (
    TARGET,
    ROOT / "configs" / "factory-semiconductor-campus16-wide-bridges-coverage-eta",
    ROOT / "configs" / "factory-semiconductor-campus16-wide-bridges-handoff-admission",
)
ASSET_CACHE = Path("E:/FactoryOpsAssets/simready-context")


class WideBridgeCampusTest(unittest.TestCase):
    def _window(self, model: OperationsModel, queue: str):
        safe_exit = model.bridges.safe_exit_nodes[queue]
        route = model.book.plan(queue, safe_exit)
        window = model._next_route_window(
            route.points[0], route.points[1:], route.nodes[1:]
        )
        return safe_exit, route, window

    def _place_at_queues(self, model: OperationsModel, queues: tuple[str, str]) -> tuple[str, str]:
        robots = ("amr-1", "amr-2")
        for robot, queue in zip(robots, queues):
            x, y = model.book.nodes[queue]
            model._poses[robot].update(x_m=x, y_m=y, yaw_rad=0.0)
            model._locations[robot] = queue

        model.reservations = RouteReservationManager(
            model.ledger.run_id, tuple(model.ledger.robots), record_pose_events=False,
        )
        model._leases = {}
        model._waiting = {}
        model._observe_all()
        for robot in model.ledger.robots:
            self.assertTrue(model._claim(robot, "test-reset", model._footprint(robot)))
            model._park_hold(robot)
        return robots

    def test_opposing_ab_and_cd_windows_admit_and_progress_together(self):
        for queues in (("AB_E_QUEUE", "AB_W_QUEUE"), ("CD_E_QUEUE", "CD_W_QUEUE")):
            with self.subTest(queues=queues):
                model = OperationsModel(TARGET, initial_orders=0, run_id=f"wide-{'-'.join(queues)}")
                robots = self._place_at_queues(model, queues)
                windows = [self._window(model, queue) for queue in queues]

                self.assertTrue(windows[0][2]["tiles"].isdisjoint(windows[1][2]["tiles"]))
                for robot, queue, (safe_exit, _, _) in zip(robots, queues, windows):
                    preview = {"goal": safe_exit, "kind": "parking", "phase": "parking"}
                    self.assertEqual((), model._safe_exit_lookahead_blockers(robot, preview, queue))
                    model._trip(
                        robot, {}, queue, safe_exit, "parking", kind="parking", purpose="staging",
                    )

                starts = {
                    robot: (model._poses[robot]["x_m"], model._poses[robot]["y_m"])
                    for robot in robots
                }
                for _ in range(100):
                    model._observe_all()
                    for robot in robots:
                        model._move_trip(robot, model._trips[robot], 0.1)

                self.assertEqual({}, model._waiting)
                for robot in robots:
                    pose = (model._poses[robot]["x_m"], model._poses[robot]["y_m"])
                    self.assertNotEqual(starts[robot], pose)
                    self.assertIn(robot, model._chunks)
                self.assertEqual(
                    {queues[0].removesuffix("_QUEUE")}, model.bridges.held(robots[0]),
                )
                self.assertEqual(
                    {queues[1].removesuffix("_QUEUE")}, model.bridges.held(robots[1]),
                )

    def test_same_lane_window_and_actual_geometry_conflicts_remain_blocked(self):
        model = OperationsModel(TARGET, initial_orders=0, run_id="wide-same-lane")
        _, route, window = self._window(model, "AB_E_QUEUE")
        self.assertEqual((), model.bridges.admit(
            "amr-1", route.nodes[1:], route_tiles=window["tiles"], observed_tiles={},
        ))
        self.assertEqual(("amr-1",), model.bridges.admit(
            "amr-2", route.nodes[1:], route_tiles=window["tiles"], observed_tiles={},
        ))

        planning_radius = model.book.planning_radius_m + model.book.layout.resolution_m / 2
        for queue in ("AB_E_QUEUE", "AB_W_QUEUE", "CD_E_QUEUE", "CD_W_QUEUE"):
            safe_exit = model.bridges.safe_exit_nodes[queue]
            route = model.book.plan(queue, safe_exit)
            route_tiles = model._tiles(route.points)
            self.assertEqual(
                [],
                [
                    node for node in sorted(model.bridges.holdable_nodes)
                    if node not in {queue, safe_exit}
                    and route_tiles & model._tiles((model.book.nodes[node],))
                ],
                (queue, safe_exit),
            )
            for start, end in zip(route.points, route.points[1:]):
                steps = max(1, math.ceil(math.dist(start, end) / model.book.layout.resolution_m))
                for index in range(steps + 1):
                    fraction = index / steps
                    point = (
                        start[0] + fraction * (end[0] - start[0]),
                        start[1] + fraction * (end[1] - start[1]),
                    )
                    self.assertFalse(
                        _bad_touched_cells(model.book.layout, point, planning_radius),
                        (queue, safe_exit, point),
                    )

    def test_cd_east_safe_exit_resumes_the_d_local_delivery_window(self):
        model = OperationsModel(TARGET, initial_orders=0, run_id="wide-cd-east-d-local")
        goal = "D_S1"
        full_od = model.book.plan("A_OUT", goal)
        self.assertIn("AC_S_QUEUE", full_od.nodes)
        self.assertIn("CD_E_QUEUE", full_od.nodes)
        trip = {"goal": goal, "kind": "process_line", "phase": "drop"}
        direct = model.book.plan("CD_E_QUEUE", goal)
        points, nodes, safe_exit = model._route_via_safe_exit(
            direct.points[0], trip, direct.points[1:], direct.nodes[1:],
        )
        first = model._next_route_window(direct.points[0], points, nodes, allow_terminal=True)
        self.assertEqual("D_LOW_E_P1", safe_exit)
        self.assertEqual(safe_exit, first["nodes"][-1])

        resumed = model.book.plan(safe_exit, goal)
        second = model._next_route_window(
            resumed.points[0], resumed.points[1:], resumed.nodes[1:], allow_terminal=True,
        )
        self.assertEqual(goal, second["nodes"][-1])
        self.assertIn("D_center-shared-link", second["corridor"])
        self.assertIn("D_west-shared-link", second["corridor"])
        xmin, ymin, xmax, ymax = model.book.config["policy_region_overrides"][
            "CAMPUS_H_CD_RESERVATION"
        ]
        x, y = model.book.nodes[goal]
        self.assertFalse(xmin <= x <= xmax and ymin <= y <= ymax)

    def test_variant_preserves_base_bundle_and_shared_vertical_bridges(self):
        base_before = build_campus_bundle(BASE)
        variant = build_wide_bridge_campus_bundle(BASE)
        self.assertEqual(base_before, build_campus_bundle(BASE))
        for name in set(base_before) - {"layout.json", "g3-routing.json", "visual-assets.json"}:
            self.assertEqual(base_before[name], variant[name], name)
            self.assertEqual((ORIGINAL / name).read_bytes(), (TARGET / name).read_bytes(), name)

        fixtures = {
            item["id"]: item
            for item in variant["visual-assets.json"]["environment"]["fixtures"]
        }
        self.assertEqual([29.6, 7.5], fixtures["A_MAINT_TOOL_STORAGE_06"]["center_xy"])
        self.assertEqual([29.3, 10.45], fixtures["A_MAINT_ROLLING_BENCH_06"]["center_xy"])

        original = base_before["g3-routing.json"]
        routing = variant["g3-routing.json"]
        self.assertEqual(
            {name: original["bridge_admission"]["resources"][name] for name in ("AC", "BD")},
            {name: routing["bridge_admission"]["resources"][name] for name in ("AC", "BD")},
        )
        self.assertEqual(
            original["authored_allowances"], routing["authored_allowances"],
        )
        self.assertEqual(
            {"AB_E", "AB_W", "CD_E", "CD_W", "AC", "BD"},
            set(routing["bridge_admission"]["resources"]),
        )
        geometry = routing["wide_bridge_geometry"]
        self.assertGreater(
            geometry["lane_center_separation_m"],
            geometry["minimum_center_separation_m"],
        )
        self.assertEqual("D_LOW_E_P1", routing["bridge_admission"]["safe_exit_nodes"]["CD_E_QUEUE"])
        self.assertEqual([54.0, 44.5], routing["nodes"]["D_LOW_E_P1"])

        generated = build_wide_bridge_campus_bundle(BASE)
        for name, value in generated.items():
            self.assertEqual(value, json.loads((TARGET / name).read_text(encoding="utf-8")))
        RouteBook.from_files(TARGET / "layout.json", TARGET / "g3-routing.json")

    @unittest.skipUnless(ASSET_CACHE.is_dir(), "prepared SimReady cache unavailable")
    def test_wide_variants_clear_actual_prepared_fixture_bounds(self):
        generated = build_wide_bridge_campus_bundle(BASE)
        for config in WIDE_VARIANTS:
            self.assertEqual(
                generated["visual-assets.json"],
                json.loads((config / "visual-assets.json").read_text(encoding="utf-8")),
                config.name,
            )
            context = build_factory_context(
                Layout(json.loads((config / "layout.json").read_text(encoding="utf-8"))),
                generated["visual-assets.json"]["environment"],
                cache_root=ASSET_CACHE,
            )
            self.assertEqual(88, len(context["instances"]), config.name)


if __name__ == "__main__":
    unittest.main()
