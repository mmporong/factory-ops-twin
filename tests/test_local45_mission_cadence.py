"""Mission routing and post-render publication boundary; actual ranges need the SDK fixture."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from scripts.isaac.g2_scene import _run, _setup_mission_stepper, _update_physics_app


class World:
    def __init__(self):
        self.current_time_step_index = 124  # Deliberately not aligned to a six-frame boundary.
        self.current_time = 124 / 60
        self.renders = []

    def step(self, *, render):
        if render:
            raise AssertionError("physics step must not request a render")
        self.current_time_step_index += 1
        self.current_time += 1 / 60

    def render(self):
        self.renders.append(self.current_time_step_index)


class MissionCadenceTests(unittest.TestCase):
    def test_only_local45_physx_selects_explicit_stepper(self):
        for local, backend in ((False, "rtx_flat_scan"), (False, "physx_planar"), (True, "rtx_flat_scan")):
            for cadence in ("owned_step6", "app_update"):
                self.assertIsNone(_setup_mission_stepper(object(), 1/60,
                    local_sdk45=local, lidar_backend=backend, physics_cadence=cadence))
        world = World()
        stepper = _setup_mission_stepper(SimpleNamespace(world=world), 1/60,
            local_sdk45=True, lidar_backend="physx_planar")
        self.assertIs(stepper.world, world)

    def test_explicit_app_update_baseline_uses_existing_update_clock_and_scan_path(self):
        stepper = _setup_mission_stepper(object(), 1/60, local_sdk45=True,
            lidar_backend="physx_planar", physics_cadence="app_update")
        self.assertIsNone(stepper)
        order = []
        app = SimpleNamespace(update=lambda: order.append("update"))
        owner = SimpleNamespace(publish=lambda: order.append("clock"))
        for _ in range(2):
            self.assertTrue(_update_physics_app(app, owner, stepper=stepper))
        self.assertEqual(["update", "clock"]*2, order)

    def test_invalid_cadence_is_rejected_before_sdk_or_world_access(self):
        with self.assertRaisesRegex(ValueError, "physics cadence"):
            _setup_mission_stepper(object(), 1/60, local_sdk45=True,
                lidar_backend="physx_planar", physics_cadence="owned_step3")
        with self.assertRaisesRegex(ValueError, "physics cadence"):
            _run(SimpleNamespace(), local_sdk45=True, lidar_backend="physx_planar",
                physics_cadence="owned_step3")

    def test_existing_app_path_publishes_clock_after_update_and_keeps_scan_eligibility(self):
        order = []
        app = SimpleNamespace(update=lambda: order.append("update"))
        owner = SimpleNamespace(publish=lambda: order.append("clock"))
        self.assertTrue(_update_physics_app(app, owner))
        self.assertEqual(["update", "clock"], order)

    def test_each_control_tick_advances_native_physics_but_only_render_ticks_admit_scans(self):
        world = World()
        stepper = _setup_mission_stepper(SimpleNamespace(world=world), 1/60,
            local_sdk45=True, lidar_backend="physx_planar")
        app = Mock()
        app.update.side_effect = AssertionError("no app.update fallback")
        clocks, scan_frames = [], []
        owner = SimpleNamespace(publish=lambda: clocks.append(world.current_time_step_index))
        for _ in range(12):
            if _update_physics_app(app, owner, stepper=stepper):
                scan_frames.append(world.current_time_step_index)
        self.assertEqual(list(range(125, 137)), clocks)
        self.assertEqual([130, 136], world.renders)
        self.assertEqual(world.renders, scan_frames)
        app.update.assert_not_called()

    def test_failed_physics_does_not_publish_clock_or_fallback(self):
        app, owner, stepper = Mock(), Mock(), Mock()
        stepper.step.side_effect = RuntimeError("physical step failed")
        with self.assertRaisesRegex(RuntimeError, "physical step failed"):
            _update_physics_app(app, owner, stepper=stepper)
        owner.publish.assert_not_called()
        app.update.assert_not_called()


if __name__ == "__main__":
    unittest.main()
