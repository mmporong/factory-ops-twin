"""CPU-only counterflow admission checks; no Nav2 or physical-motion claim."""
from __future__ import annotations

from pathlib import Path
import unittest

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_routing import opposing_task_blockers
from factory_ops.layout import Layout
from factory_ops.material_flow import EmptyTrip, MaterialFlow
from factory_ops.navigation import GoalPose
from factory_ops.route_reservation import RouteReservationManager
from scripts.ros.g3_mission import active_transport_pairs


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-semiconductor"
ROBOTS = {"carter1": "P1", "carter2": "P2", "carter3": "P3", "carter4": "P4"}


def make_flow(*sources: str) -> tuple[FleetMission, MaterialFlow]:
    layout = Layout.from_file(CONFIG / "layout.json")
    goals = {
        name: GoalPose("map", *xy, 0.0)
        for name, xy in {**layout.stations, **layout.pockets}.items()
    }
    orders = [
        FleetOrder(f"O{index}", f"PAY{index}", source, 0.0)
        for index, source in enumerate(sources, start=1)
    ]
    mission = FleetMission(
        "counterflow-cpu",
        orders,
        robot_stations=ROBOTS,
        station_goals=goals,
        goal_timeout_s=600.0,
    )
    material = MaterialFlow(
        mission,
        {
            "schema_version": 1,
            "initial_stock": [
                {"location": source, "sku": "tray", "quantity": 10}
                for source in sorted(set(sources))
            ],
            "materials": [
                {
                    "order_id": order.order_id,
                    "source": order.source_station,
                    "sku": "tray",
                    "quantity": 1,
                    "tote_id": f"TOTE-{order.order_id}",
                }
                for order in orders
            ],
        },
    )
    return mission, material


def admit(robot: str, pickup: str, drop: str, mission: FleetMission, material: MaterialFlow) -> bool:
    return not opposing_task_blockers(
        robot_id=robot,
        pickup_station=pickup,
        drop_station=drop,
        active_tasks=active_transport_pairs(mission, material),
    )


class SemiconductorCounterflowTests(unittest.TestCase):
    def test_only_exact_reverse_pair_blocks_and_reservations_are_unchanged(self):
        active = {
            "carter1": ("S1", "CI"),
            "carter2": ("S2", "CI"),
            "carter3": ("CO", "OUT"),
        }
        manager = RouteReservationManager("counterflow-read-only", tuple(ROBOTS))
        before = manager.snapshot(now_s=0.0)

        self.assertEqual(
            ("carter1",),
            opposing_task_blockers(
                robot_id="carter4", pickup_station="CI", drop_station="S1", active_tasks=active
            ),
        )
        self.assertEqual(
            (),
            opposing_task_blockers(
                robot_id="carter4", pickup_station="CI", drop_station="S3", active_tasks=active
            ),
        )
        self.assertEqual(
            (),
            opposing_task_blockers(
                robot_id="carter4", pickup_station="S3", drop_station="CI", active_tasks=active
            ),
        )
        self.assertEqual(
            (),
            opposing_task_blockers(
                robot_id="carter4", pickup_station="S1", drop_station="CI", active_tasks=active
            ),
        )
        self.assertEqual(before, manager.snapshot(now_s=0.0))

    def test_active_pairs_read_only_current_assignments(self):
        mission, material = make_flow("S1", "S2", "S3")
        product = mission.dispatch(0.0, lambda robot, task: 0.0, eligible_robot_ids=["carter1"])
        self.assertEqual(1, len(product))
        material.jobs["empty:active"] = EmptyTrip("empty:active", "O2", "TOTE-O2", "CI", "S2", 0.0)
        material.jobs["empty:future"] = EmptyTrip("empty:future", "O3", "TOTE-O3", "CI", "S3", 10.0)
        material.jobs["empty:complete"] = EmptyTrip(
            "empty:complete", "O3", "TOTE-O3", "CI", "S3", 0.0, state="complete", robot_id="carter4"
        )
        empty = material.dispatch(["carter2"], sim_time_s=0.0, cost=lambda robot, job: 0.0)
        self.assertEqual(1, len(empty))

        self.assertEqual(
            {"carter1": ("S1", "CI"), "carter2": ("CI", "S2")},
            active_transport_pairs(mission, material),
        )

    def test_both_dispatchers_recheck_same_tick_assignments(self):
        mission, material = make_flow("S1", "S2")
        material.jobs["empty:seed"] = EmptyTrip("empty:seed", "O1", "TOTE-O1", "CI", "S1", 0.0)
        empty = material.dispatch(
            ["carter1"],
            sim_time_s=0.0,
            cost=lambda robot, job: 0.0 if admit(robot, job.source, job.destination, mission, material) else None,
        )
        self.assertEqual(["empty:seed"], [intent.task_id for intent in empty])

        product_evaluations: list[tuple[str, str, dict[str, tuple[str, str]]]] = []

        def product_cost(robot, task):
            product_evaluations.append((robot.robot_id, task.task_id, active_transport_pairs(mission, material)))
            if not admit(robot.robot_id, task.pickup_station, task.drop_station, mission, material):
                return None
            return 0.0 if task.pickup_station == "S2" else 1.0

        products = mission.dispatch(
            0.0,
            product_cost,
            eligible_robot_ids=["carter2", "carter3"],
        )
        self.assertEqual(["O2:in"], [intent.task_id for intent in products])
        self.assertTrue(all("carter1" in pairs for _, _, pairs in product_evaluations))
        self.assertNotIn("O1:in", {intent.task_id for intent in mission.active_intents.values()})

        reverse_mission, reverse_material = make_flow("S1", "S2")
        selected = reverse_mission.dispatch(
            0.0,
            lambda robot, task: 0.0 if task.pickup_station == "S1" else None,
            eligible_robot_ids=["carter1"],
        )
        self.assertEqual(["O1:in"], [intent.task_id for intent in selected])
        reverse_material.jobs["empty:S1"] = EmptyTrip("empty:S1", "O1", "TOTE-O1", "CI", "S1", 0.0)
        reverse_material.jobs["empty:S2"] = EmptyTrip("empty:S2", "O2", "TOTE-O2", "CI", "S2", 0.0)
        empty_evaluations: list[tuple[str, str, dict[str, tuple[str, str]]]] = []

        def empty_cost(robot, job):
            empty_evaluations.append((robot, job.task_id, active_transport_pairs(reverse_mission, reverse_material)))
            return 0.0 if admit(robot, job.source, job.destination, reverse_mission, reverse_material) else None

        returns = reverse_material.dispatch(
            ["carter2", "carter3"], sim_time_s=0.0, cost=empty_cost
        )
        self.assertEqual(["empty:S2"], [intent.task_id for intent in returns])
        self.assertTrue(all("carter1" in pairs for _, _, pairs in empty_evaluations))
        self.assertEqual("ready", reverse_material.jobs["empty:S1"].state)

        for active_pairs in (
            active_transport_pairs(mission, material),
            active_transport_pairs(reverse_mission, reverse_material),
        ):
            for robot, (pickup, drop) in active_pairs.items():
                self.assertEqual(
                    (),
                    opposing_task_blockers(
                        robot_id=robot,
                        pickup_station=pickup,
                        drop_station=drop,
                        active_tasks=active_pairs,
                    ),
                )


if __name__ == "__main__":
    unittest.main()
