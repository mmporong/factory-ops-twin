"""Same-run reservation projection; no SDK, ROS or HTTP server required."""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.dashboard import DashboardService, validate_snapshot


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v4"


class DashboardRosChargingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.primary_path = Path(self.temp.name) / "dashboard-state.json"
        self.charging_path = Path(self.temp.name) / "charging-state.json"
        self.primary = DashboardService(CONFIG).snapshot()
        self.primary.update(mode="live", run_id="ros-run-040", revision=9, sim_time_s=24.05,
            observed_at=datetime.now(timezone.utc).isoformat(), capabilities=["pause"],
            model_scope="ROS observations; virtual battery, uncalibrated; charging unconnected")
        self.companion = {"run_id": self.primary["run_id"],
            "contact_source": "estimated_pose_geometric_dock", "virtual": True,
            "hardware_calibrated": False, "physical_contact_verified": False,
            "physical_electrical_charging_verified": False,
            "body_occupancy_owner": "G3 reservation manager",
            "docks": {"CH1": {"anchor": "P1", "x_m": 12, "y_m": 1.5, "yaw_rad": 1.5708},
                      "CH2": {"anchor": "P2", "x_m": 12, "y_m": 20.5, "yaw_rad": -1.5708}},
            "owners": {}, "robots": {}}
        for index, robot in enumerate(self.primary["robots"]):
            robot.update(battery_pct=84.9 - index * 5, battery_source="authored_virtual_energy_model",
                observed_sim_time_s=24.05, scan_counts={"front_raw": 123, "rear_raw": 121},
                odom_count=590, charging_dock="CH1" if index == 0 else None,
                charging_phase="approach" if index == 0 else None)
            charging = None
            if index == 0:
                self.companion["owners"]["CH1"] = robot["id"]
                charging = {"dock_id": "CH1", "anchor": "P1", "phase": "approach",
                    "reserved_at_s": 6.066, "last_stamp_s": None,
                    "virtual_connector_state": "not_engaged", "geometric_dock_confirmed": False,
                    "contact_source": "estimated_pose_geometric_dock", "virtual": True,
                    "hardware_calibrated": False, "physical_contact_verified": False}
            self.companion["robots"][robot["id"]] = {"run_id": self.primary["run_id"],
                "robot_id": robot["id"], "charging": charging,
                "energy": {"run_id": self.primary["run_id"], "robot_id": robot["id"],
                           "battery_pct": 85 - index * 5, "observed_sim_time_s": 6.05}}
        self.write_sources()

    def write_sources(self):
        self.primary_path.write_text(json.dumps(self.primary), encoding="utf-8")
        self.charging_path.write_text(json.dumps(self.companion), encoding="utf-8")

    def joined_service(self):
        return DashboardService(CONFIG, source=self.primary_path, charging_source=self.charging_path)

    def test_real_shaped_join_preserves_primary_and_is_read_only(self):
        before = (self.primary_path.read_bytes(), self.charging_path.read_bytes())
        service = self.joined_service()
        state = validate_snapshot(service.snapshot())
        for key in ("robots", "inventory", "kpis", "tasks", "events", "observed_at", "sim_time_s", "revision"):
            self.assertEqual(self.primary[key], state[key], key)
        self.assertEqual([], state["capabilities"])
        self.assertEqual("connected", state["connection"])
        charging = state["charging"]
        self.assertEqual([], charging["queue"])
        self.assertIs(False, charging["queue_available"])
        self.assertEqual("unavailable_in_ros_charging_state", charging["queue_source"])
        self.assertEqual(self.primary["robots"][0]["id"], charging["docks"][0]["owner"])
        self.assertEqual("approach", charging["docks"][0]["phase"])
        self.assertEqual("unreserved", charging["docks"][1]["phase"])
        for key in ("hardware_calibrated", "physical_contact_verified", "physical_electrical_charging_verified"):
            self.assertIs(False, charging[key])
        self.assertNotIn("charging unconnected", state["model_scope"])
        self.assertIn("unverified", state["model_scope"])
        self.assertEqual(str(self.charging_path), state["api_producer"]["input_sources"]["charging"])
        receipt = service.command({"command_id": "readonly", "expected_revision": state["revision"],
                                   "expected_run_id": state["run_id"], "action": "pause"})
        self.assertEqual("rejected", receipt["status"])
        self.assertIsNone(service.outbox)
        self.assertIsNone(service.model)
        self.assertEqual(before, (self.primary_path.read_bytes(), self.charging_path.read_bytes()))
        self.assertEqual(2, len(list(self.primary_path.parent.iterdir())))

    def test_invalid_companion_keeps_last_good_then_recovers(self):
        original = deepcopy(self.companion)
        robot_id = self.primary["robots"][0]["id"]
        for corruption in ("run", "owner", "duplicate_owner", "fleet", "nested_run", "phase", "physical_claim", "json"):
            with self.subTest(corruption=corruption):
                self.companion = deepcopy(original)
                self.write_sources()
                service = self.joined_service()
                first = service.snapshot()
                if corruption == "run":
                    self.companion["run_id"] = "other-run"
                elif corruption == "owner":
                    self.companion["owners"]["CH1"] = "foreign-robot"
                elif corruption == "duplicate_owner":
                    self.companion["owners"]["CH2"] = robot_id
                elif corruption == "fleet":
                    del self.companion["robots"][robot_id]
                elif corruption == "nested_run":
                    self.companion["robots"][robot_id]["energy"]["run_id"] = "other-run"
                elif corruption == "phase":
                    self.companion["robots"][robot_id]["charging"]["phase"] = "charging"
                elif corruption == "physical_claim":
                    self.companion["physical_contact_verified"] = True
                self.write_sources()
                if corruption == "json":
                    self.charging_path.write_text("{", encoding="utf-8")
                current = service.snapshot()
                self.assertEqual("disconnected", current["connection"])
                self.assertEqual(first["robots"], current["robots"])
                self.assertEqual(first["charging"], current["charging"])
                self.assertEqual([], current["capabilities"])
                with self.assertRaises(ValueError):
                    self.joined_service().snapshot()
                self.companion = deepcopy(original)
                self.write_sources()
                self.assertEqual("connected", service.snapshot()["connection"])
        self.charging_path.unlink()
        self.assertEqual("disconnected", service.snapshot()["connection"])

    def test_opt_in_guards_and_default_sources_are_unchanged(self):
        for options in ({}, {"source": self.primary_path, "outbox": self.primary_path.parent / "outbox"},
                        {"source": self.primary_path, "charging_profile": {}}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                DashboardService(CONFIG, charging_source=self.charging_path, **options)
        cpu = DashboardService(CONFIG)
        state = cpu.snapshot()
        self.assertEqual("local_model", state["mode"])
        self.assertNotIn("charging", state)
        self.assertTrue(all(row["battery_pct"] is None for row in state["robots"]))
        self.assertEqual("applied", cpu.command({"command_id": "cpu-pause", "action": "pause",
                                                "expected_revision": state["revision"]})["status"])
        for robot in self.primary["robots"]:
            robot.update(charging_dock=None, charging_phase=None)
        self.write_sources()
        plain = DashboardService(CONFIG, source=self.primary_path).snapshot()
        self.assertNotIn("charging", plain)
        self.assertEqual(self.primary["model_scope"], plain["model_scope"])

    def test_unknown_queue_cannot_masquerade_as_observed_data(self):
        joined = self.joined_service().snapshot()
        for corruption in ("populated", "missing_source", "not_boolean"):
            broken = deepcopy(joined)
            if corruption == "populated":
                broken["charging"]["queue"] = [self.primary["robots"][1]["id"]]
            elif corruption == "missing_source":
                del broken["charging"]["queue_source"]
            else:
                broken["charging"]["queue_available"] = "false"
            with self.subTest(corruption=corruption), self.assertRaises(ValueError):
                validate_snapshot(broken)
        profile = json.loads((ROOT / "configs/web-charging-model.json").read_text(encoding="utf-8"))
        cpu = validate_snapshot(DashboardService(CONFIG, charging_profile=profile).snapshot())
        self.assertNotIn("queue_available", cpu["charging"])
        self.assertIsInstance(cpu["charging"]["queue"], list)


if __name__ == "__main__":
    unittest.main()
