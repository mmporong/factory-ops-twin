"""G3 scene-extension contracts using lightweight USD attribute fakes."""

from __future__ import annotations

import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from scripts.isaac.g3_scene import BarrierLifecycle


RUN_ID = "a" * 32
SCENARIO_ID = "PILOT_CLOSURE"


class Attribute:
    def __init__(self, value=None) -> None:
        self.value = value
        self.history = [value]

    def Set(self, value) -> None:
        self.value = value
        self.history.append(value)

    def Get(self):
        return self.value


class Cube:
    def __init__(self) -> None:
        self.prim = object()
        self.size = Attribute()
        self.translate = Attribute()
        self.scale = Attribute()
        self.color = Attribute()

    def CreateSizeAttr(self, value):
        self.size.Set(value)

    def AddTranslateOp(self):
        return self.translate

    def AddScaleOp(self):
        return self.scale

    def CreateDisplayColorAttr(self, value):
        self.color.Set(value)

    def GetPrim(self):
        return self.prim


class FakeUSD:
    def __init__(self) -> None:
        self.cube = Cube()
        self.collision = Attribute()
        self.visibility = Attribute()
        owner = self

        class CubeFactory:
            @staticmethod
            def Define(_stage, path):
                if path != "/World/G3Barrier":
                    raise AssertionError(path)
                return owner.cube

        class CollisionAPI:
            @staticmethod
            def Apply(prim):
                if prim is not owner.cube.prim:
                    raise AssertionError("wrong prim")
                return SimpleNamespace(CreateCollisionEnabledAttr=lambda value: owner._collision(value))

        class Imageable:
            def __init__(self, prim) -> None:
                if prim is not owner.cube.prim:
                    raise AssertionError("wrong prim")

            @staticmethod
            def CreateVisibilityAttr(value):
                owner.visibility.Set(value)
                return owner.visibility

        self.UsdGeom = SimpleNamespace(Cube=CubeFactory, Imageable=Imageable)
        self.UsdPhysics = SimpleNamespace(CollisionAPI=CollisionAPI)
        self.Gf = SimpleNamespace(Vec3d=lambda *values: values, Vec3f=lambda *values: values)

    def _collision(self, value):
        self.collision.Set(value)
        return self.collision


def closure_session():
    return {
        "scenario_id": SCENARIO_ID,
        "scene_closure": {
            "source": "configs/factory-v3/layout.json",
            "layout_object_id": "blocked_aisle",
            "rect_xyxy_m": [25.5, 2.5, 26.5, 5.5],
            "height_m": 1.5,
            "scene_active_interval_s": [0.0, 600.0],
        },
    }


class BarrierLifecycleTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.output = Path(self.temporary.name)
        self.usd = FakeUSD()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def lifecycle(self) -> BarrierLifecycle:
        lifecycle = BarrierLifecycle(closure_session(), RUN_ID)
        metadata = lifecycle.setup(
            object(), self.usd.UsdGeom, self.usd.UsdPhysics, self.usd.Gf, self.output
        )
        self.assertFalse(metadata["ground_truth_control_input"])
        return lifecycle

    def write_start(self, **changes) -> None:
        value = {"run_id": RUN_ID, "scenario_id": SCENARIO_ID, "sim_time_s": 10.0}
        value.update(changes)
        (self.output / "mission-start.json").write_text(json.dumps(value), encoding="utf-8")

    def test_authored_startup_closure_uses_mission_origin_then_opens_at_end(self) -> None:
        lifecycle = self.lifecycle()
        self.assertIs(self.usd.collision.Get(), True)
        self.assertEqual(self.usd.visibility.Get(), "inherited")
        self.write_start()
        lifecycle.step(10.0, 1)
        self.assertEqual(lifecycle.origin, 10.0)
        lifecycle.step(609.999, 2)
        self.assertIs(self.usd.collision.Get(), True)
        lifecycle.step(610.0, 3)
        self.assertIs(self.usd.collision.Get(), False)
        self.assertEqual(self.usd.visibility.Get(), "invisible")
        events = [json.loads(line) for line in (self.output / "scene-events.jsonl").read_text().splitlines()]
        self.assertEqual([event["current_closed"] for event in events], [True, False])
        self.assertEqual(events[-1]["physics_frame"], 3)

    def test_scene_state_does_not_expose_schedule_or_removal_time(self) -> None:
        lifecycle = self.lifecycle()
        state = json.loads((self.output / "scene-state.json").read_text(encoding="utf-8"))
        self.assertEqual(
            set(state),
            {
                "schema_version", "run_id", "scenario_id", "current_closed",
                "sim_time_s", "physics_frame", "collision_enabled_readback",
            },
        )
        self.assertNotIn("scene_active_interval_s", state)
        self.assertNotIn("removal_time", state)
        self.assertIsNone(lifecycle.origin)

    def test_mission_start_identity_and_timestamp_are_validated(self) -> None:
        for changes, now, message in (
            ({"run_id": "b" * 32}, 10.0, "identity"),
            ({"scenario_id": "OTHER"}, 10.0, "identity"),
            ({"sim_time_s": -1.0}, 10.0, "time is invalid"),
            ({"sim_time_s": math.nan}, 10.0, "time is invalid"),
            ({"sim_time_s": 11.0}, 10.0, "time is invalid"),
            ({"sim_time_s": True}, 10.0, "time is invalid"),
            ({"sim_time_s": False}, 10.0, "time is invalid"),
            ({"sim_time_s": "10.0"}, 10.0, "time is invalid"),
        ):
            with self.subTest(changes=changes):
                folder = Path(tempfile.mkdtemp(dir=self.output))
                lifecycle = BarrierLifecycle(closure_session(), RUN_ID)
                lifecycle.setup(object(), self.usd.UsdGeom, self.usd.UsdPhysics, self.usd.Gf, folder)
                value = {"run_id": RUN_ID, "scenario_id": SCENARIO_ID, "sim_time_s": 10.0}
                value.update(changes)
                (folder / "mission-start.json").write_text(json.dumps(value), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    lifecycle.step(now, 1)

    def test_post_step_control_ack_tracks_new_sequence_start_and_stop_without_ground_truth(self) -> None:
        lifecycle = self.lifecycle()
        (self.output / "robots/carter1").mkdir(parents=True)
        sample = {
            "physics_frame": 40,
            "authorization": {
                "sequence": 7,
                "mode": "navigate",
                "valid": True,
                "evaluation_sim_time_s": 12.5,
                "payload": {"scope": {"goal_uuid": "g", "reservation_id": 2}},
            },
            "motion_profile": {
                "input_command": {"linear_m_s": 0.4, "angular_rad_s": 0.1},
                "stop_reason": "none",
            },
            "ground_truth": {"position_xyz_m": [99, 99, 99]},
        }
        lifecycle.after_control("carter1", sample)
        path = self.output / "robots/carter1/control-ack.json"
        started = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(
            (started["authorization_seq"], started["mode"], started["input_command"]),
            (7, "navigate", {"linear_m_s": 0.4, "angular_rad_s": 0.1}),
        )
        self.assertNotIn("ground_truth", started)
        original = path.read_bytes()
        sample["motion_profile"]["input_command"] = {"linear_m_s": 0.2, "angular_rad_s": 0.0}
        lifecycle.after_control("carter1", sample)
        self.assertEqual(path.read_bytes(), original)
        sample["authorization"].update({"sequence": 8, "mode": "hold"})
        sample["motion_profile"]["input_command"] = {"linear_m_s": 0.0, "angular_rad_s": 0.0}
        lifecycle.after_control("carter1", sample)
        stopped = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(
            (stopped["authorization_seq"], stopped["mode"], stopped["input_command"]),
            (8, "hold", {"linear_m_s": 0.0, "angular_rad_s": 0.0}),
        )
        self.assertNotIn("ground_truth", stopped)


if __name__ == "__main__":
    unittest.main()
