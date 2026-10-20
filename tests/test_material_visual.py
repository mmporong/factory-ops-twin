"""Real MaterialFlow CPU lifecycle and applied-only visual provenance; no SDK."""
from copy import deepcopy
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.layout import Layout
from factory_ops.material_flow import MaterialFlow
from factory_ops.material_visual import (AppliedMaterialVisual, STATE_FILE, make_material_visual_state,
                                        project_empty_totes, publish_material_visual)
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult
from factory_ops.visual_state import canonical_sha256
from scripts.isaac.factory_visuals import world_slot_placement
from scripts.isaac.local45_fleet4_transport_scene import EmptyToteGeometry
from scripts.isaac.probe_local45_material_visual import load_inputs

CONFIG = Path(__file__).resolve().parents[1] / "configs/factory-fleet4"


class MaterialVisualTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = json.loads((CONFIG / "transport-profile.json").read_text(encoding="utf-8"))
        cls.layout = Layout.from_file(CONFIG / "layout.json")
        cls.robots = tuple(cls.profile["initial_robots"])
        cls.materials = cls.profile["material_flow"]["materials"]
        cls.capacity = cls.profile["load"]["ci_capacity"]

    def fixture(self):
        mission = FleetMission("material076", [FleetOrder(**row) for row in self.profile["orders"]],
            robot_stations=self.profile["initial_robots"], goal_timeout_s=5000,
            station_goals={name: GoalPose("map", *xy, 0.) for name, xy in
                {**self.layout.stations, **self.layout.pockets}.items()})
        flow = MaterialFlow(mission, self.profile["material_flow"])
        self.assertEqual([], self.project(flow.snapshot()))
        clock = 0.
        for _ in range(1000):
            mission.dispatch(clock, lambda robot, task: 1)
            for robot, intent in list(mission.active_intents.items()):
                clock = max(clock, mission.sim_time, intent.issued_sim_time) + 1
                result = self.result(intent, clock)
                receipt = mission.submit_result(robot, result, error_code=0)
                flow.business_handoff(intent, receipt)
            flow.advance()
            clock += 1
            if mission.completed:
                return flow, clock
        self.fail("bounded logical product fixture did not complete")

    def result(self, intent, clock):
        return NavigationResult(str(uuid4()), intent.goal_id, intent.task_id, intent.station, 0, 4, clock,
            LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, clock))

    def project(self, snapshot):
        return project_empty_totes(snapshot, self.materials, self.robots, ci_capacity=self.capacity)

    def document(self, snapshot, stamp=1.):
        return make_material_visual_state(snapshot, run_id=snapshot["run_id"], scenario_id="fixture076",
            source_sim_time_s=stamp, materials=self.materials, robot_ids=self.robots, ci_capacity=self.capacity)

    def test_real_lifecycle_wait_carry_return_reuse_and_authored_support_slots(self):
        flow, clock = self.fixture()
        waiting = self.project(flow.snapshot())
        self.assertEqual(4, len(waiting))
        self.assertTrue(all(row["location"] == "ci" and row["slot"] >= self.capacity for row in waiting))
        for row in waiting:
            world_slot_placement(self.layout, row["location"], row["slot"])
        intent = flow.dispatch(["carter3"], sim_time_s=clock, cost=lambda robot, job: 1)[0]
        self.assertTrue(all(row["robot_id"] is None for row in self.project(flow.snapshot())))
        flow.submit_result(flow.mission.run_id, "carter3", self.result(intent, clock+1), error_code=0)
        carried = next(row for row in self.project(flow.snapshot()) if row["robot_id"] == "carter3")
        self.assertEqual(intent.payload_id, carried["tote_id"])
        self.assertEqual("robot_carter3", carried["location"])
        self.assertTrue(carried["tote_id"].startswith("F4-TOTE-"))
        self.assertNotIn(carried["tote_id"], {order.payload_id for order in flow.mission.declared_orders})
        drop = flow.active_intents["carter3"]
        flow.submit_result(flow.mission.run_id, "carter3", self.result(drop, clock+2), error_code=0)
        source = flow.snapshot()
        returned = next(row for row in self.project(source) if row["tote_id"] == carried["tote_id"])
        self.assertEqual("source_"+drop.station, returned["location"])
        self.assertIsNone(returned["robot_id"])
        world_slot_placement(self.layout, returned["location"], returned["slot"])
        visual = SimpleNamespace(layout=self.layout, last_event={"displayed_payloads": [
            {"location": returned["location"], "slot": 0}]}, robot_paths={}, stage=Mock(),
            UsdGeom=Mock(), UsdPhysics=Mock(), Gf=Mock())
        visual.UsdGeom.Cube.Define.return_value.GetPrim.return_value.HasAPI.return_value = False
        geometry = EmptyToteGeometry(visual)
        placed = geometry.apply([returned])
        self.assertEqual(1, placed[0]["slot"])
        self.assertEqual(5, visual.UsdGeom.Cube.Define.call_count)
        visual.last_event["displayed_payloads"].append({"location": returned["location"], "slot": 1})
        self.assertIsNone(geometry.apply([returned]))
        next(row for row in source["inventory"]["totes"] if row["tote_id"] == carried["tote_id"])["state"] = "filled"
        self.assertNotIn(carried["tote_id"], {row["tote_id"] for row in self.project(source)})

    def test_unknown_id_owner_location_and_missing_trip_are_invalid(self):
        flow, _ = self.fixture()
        source = flow.snapshot()
        for changes in ({"tote_id": "F4-PAY-001"}, {"owner": "unknown"}, {"location": "OUT"}, {"state": "unknown"}):
            bad = deepcopy(source)
            bad["inventory"]["totes"][0].update(changes)
            with self.assertRaises(ValueError):
                self.project(bad)
        bad = deepcopy(source)
        bad["empty_trips"] = []
        with self.assertRaises(ValueError): self.project(bad)

    def test_binding_revision_and_exact_future_wait_without_rebinding_time(self):
        flow, _ = self.fixture()
        source = flow.snapshot()
        cache = AppliedMaterialVisual(source["run_id"], "fixture076", self.materials, self.robots, self.capacity)
        document = self.document(source, 1.000000005)
        cache.offer(document)
        apply = Mock(return_value=[{"prim_path": "/World/TestEmpty", "parent_path": "/World"}])
        self.assertFalse(cache.advance(1., 60, apply))
        self.assertFalse(cache.advance(1.000000004, 61, apply))
        apply.assert_not_called()
        placement = apply.return_value
        apply.return_value = None
        self.assertFalse(cache.advance(1.000000005, 62, apply))
        self.assertIsNone(cache.applied)
        apply.return_value = placement
        self.assertTrue(cache.advance(1.000000005, 62, apply))
        self.assertEqual(1.000000005, cache.applied["document"]["source_sim_time_s"])
        self.assertEqual(canonical_sha256(source), cache.applied["document"]["source_sha256"])
        self.assertFalse(cache.advance(2., 63, apply))
        self.assertEqual(2, apply.call_count)
        for key, value in (("run_id", "wrong"), ("scenario_id", "wrong"), ("material_revision", True),
                           ("source_sha256", "0"*64)):
            bad = deepcopy(document); bad[key] = value
            with self.assertRaises(ValueError): cache.offer(bad)
        bad = deepcopy(source); bad["revision"] -= 1
        with self.assertRaises(ValueError): cache.offer(self.document(bad, 3.))
        with self.assertRaises(ValueError): cache.offer(self.document(source, 3.))

    def test_capture_uses_applied_source_while_newer_file_is_pending(self):
        flow, _ = self.fixture()
        source = flow.snapshot()
        first = self.document(source, 1.)
        newer_source = deepcopy(source); newer_source["revision"] += 1
        newer = self.document(newer_source, 5.)
        cache = AppliedMaterialVisual(source["run_id"], "fixture076", self.materials, self.robots, self.capacity)
        with TemporaryDirectory() as temporary:
            output = Path(temporary)
            publish_material_visual(output, first)
            cache.offer(json.loads((output / STATE_FILE).read_text(encoding="utf-8")))
            cache.advance(1., 60, lambda rows: [{"tote_id": row["tote_id"], "slot": row["slot"]} for row in rows])
            applied = cache.capture_metadata()
            publish_material_visual(output, newer)
            cache.offer(json.loads((output / STATE_FILE).read_text(encoding="utf-8")))
            self.assertFalse(cache.advance(2., 120, Mock()))
            self.assertEqual(applied, cache.capture_metadata())
            self.assertEqual(first, cache.capture_metadata()["applied"]["document"])
            self.assertEqual(2, len(list((output / "material-visual-snapshots").glob("*.json"))))
            cache.capture_metadata()["applied"]["placements"].clear()
            self.assertTrue(cache.capture_metadata()["applied"]["placements"])

    def test_captured_observation_schema_without_checkpoint_time_reinterpretation(self):
        flow, _ = self.fixture()
        self.assertEqual(4, len(self.project(flow.snapshot())))
        # Optional captured inputs are supplied by the invoking CPU command, never hardcoded here.
        for name in json.loads(os.environ.get("FACTORY_OPS_MATERIAL_VISUAL_OBSERVATIONS", "[]")):
            observed = json.loads(Path(name).read_text(encoding="utf-8"))
            self.assertEqual(observed["source_run_id"], observed["snapshot"]["run_id"])
            projected = self.project(observed["snapshot"])
            self.assertTrue(projected)
            for row in projected:
                if row["robot_id"] is None:
                    world_slot_placement(self.layout, row["location"], row["slot"])

    def test_render_fixture_refuses_active_source_before_sdk(self):
        flow, _ = self.fixture()
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            observation, closure, config = root/"observed.json", root/"pair.json", root/"config.json"
            observation.write_text(json.dumps({"source_run_id": flow.mission.run_id, "snapshot": flow.snapshot()}))
            closure.write_text(json.dumps({"run_id": flow.mission.run_id, "windows_closed": False,
                "linux_closed": False, "cleanup_errors": []}))
            import hashlib
            config.write_text(json.dumps({"observation_path": str(observation),
                "observation_sha256": hashlib.sha256(observation.read_bytes()).hexdigest(),
                "closed_pair_host_path": str(closure), "closed_pair_host_sha256": hashlib.sha256(closure.read_bytes()).hexdigest()}))
            with self.assertRaisesRegex(ValueError, "verified closure"):
                load_inputs(config, root/"new-output", "b"*32)
            self.assertFalse((root/"new-output").exists())


if __name__ == "__main__":
    unittest.main()
