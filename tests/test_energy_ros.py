"""Check the virtual-energy boundary using the real mission and web projection."""
import json
from pathlib import Path
import unittest
from uuid import uuid4

from factory_ops.dashboard import validate_snapshot
from factory_ops.energy_runtime import EnergyRuntime
from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.layout import Layout
from factory_ops.material_flow import MaterialFlow
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from scripts.ros.g3_mission import dashboard_state, mission_energy_load


CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"


class EnergyRosTests(unittest.TestCase):
    def setUp(self):
        profile = json.loads((CONFIG / "transport-profile.json").read_text())
        layout = Layout.from_file(CONFIG / "layout.json")
        self.mission = FleetMission("energy-ros-run", [FleetOrder(**row) for row in profile["orders"]],
            robot_stations=profile["initial_robots"], goal_timeout_s=5000,
            station_goals={key: GoalPose("map", *xy, 0) for key, xy in
                           {**layout.stations, **layout.pockets}.items()})
        self.material = MaterialFlow(self.mission, profile["material_flow"])
        self.energy = EnergyRuntime(self.mission.run_id,
            json.loads((CONFIG / "energy-profile.json").read_text()), profile["energy"]["initial_soc"])
        self.clock = 0.0

    def arrival(self, intent):
        self.clock = max(self.clock, self.mission.sim_time, intent.issued_sim_time) + 1
        return NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station, 0, 4, self.clock,
            LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, self.clock))

    def test_load_factor_follows_accepted_business_pickup_and_drop(self):
        self.mission.dispatch(0, lambda robot, task: 1)
        robot, intent = next(iter(self.mission.active_intents.items()))
        self.assertFalse(mission_energy_load(self.mission, self.material, robot))
        receipt = self.mission.submit_result(robot, self.arrival(intent), error_code=0)
        self.material.business_handoff(intent, receipt)
        self.assertTrue(mission_energy_load(self.mission, self.material, robot))
        intent = self.mission.active_intents[robot]
        receipt = self.mission.submit_result(robot, self.arrival(intent), error_code=0)
        self.material.business_handoff(intent, receipt)
        self.assertFalse(mission_energy_load(self.mission, self.material, robot))

    def test_empty_tote_only_counts_as_loaded_after_accepted_pickup(self):
        for _ in range(300):
            self.mission.dispatch(self.clock, lambda robot, task: 1)
            for robot, intent in list(self.mission.active_intents.items()):
                receipt = self.mission.submit_result(robot, self.arrival(intent), error_code=0)
                self.material.business_handoff(intent, receipt)
            self.material.advance()
            self.clock += 1
            if self.mission.completed:
                break
        self.assertTrue(self.mission.completed)
        self.material.dispatch(list(self.mission.ledger.robots), sim_time_s=self.clock, cost=lambda robot, trip: 1)
        robot, intent = next(iter(self.material.active_intents.items()))
        self.assertFalse(mission_energy_load(self.mission, self.material, robot))
        self.material.submit_result(self.mission.run_id, robot, self.arrival(intent), error_code=0)
        self.assertTrue(mission_energy_load(self.mission, self.material, robot))
        self.material.submit_result(self.mission.run_id, robot,
            self.arrival(self.material.active_intents[robot]), error_code=0)
        self.assertFalse(mission_energy_load(self.mission, self.material, robot))

    def test_web_uses_observed_virtual_energy_and_null_for_missing_or_stale(self):
        self.energy.observe(self.mission.run_id, "carter1", 100, 0, 0, loaded=False)
        self.energy.observe(self.mission.run_id, "carter1", 100.2, -.4, .5, loaded=True)
        snapshot = self.mission.ledger.snapshot()
        web = validate_snapshot(dashboard_state(snapshot, {}, sim_time_s=2,
            absolute_sim_time_s=100.2, pose_ttl_s=.5, energy=self.energy))
        observed = next(row for row in web["robots"] if row["id"] == "carter1")
        self.assertLess(observed["battery_pct"], 85)
        self.assertGreater(observed["battery_pct"], 84)
        self.assertEqual("authored_virtual_energy_model", observed["battery_source"])
        self.assertTrue(all(row["battery_pct"] is None for row in web["robots"] if row["id"] != "carter1"))
        stale = dashboard_state(snapshot, {}, sim_time_s=3, absolute_sim_time_s=101,
            pose_ttl_s=.5, energy=self.energy)
        self.assertTrue(all(row["battery_pct"] is None for row in stale["robots"]))
        self.assertIn("charging unconnected", stale["model_scope"])

    def test_web_cannot_reuse_another_runs_energy(self):
        snapshot = self.mission.ledger.snapshot()
        snapshot["run_id"] = "other-run"
        with self.assertRaises(ValueError):
            dashboard_state(snapshot, {}, sim_time_s=0, absolute_sim_time_s=0,
                pose_ttl_s=.5, energy=self.energy)


if __name__ == "__main__":
    unittest.main()
