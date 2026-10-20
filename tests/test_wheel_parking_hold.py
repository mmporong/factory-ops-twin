"""Focused CPU contracts with native API substitutes, not physical validation."""
from copy import deepcopy
import builtins
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from factory_ops import isaac45_runtime as facade
from factory_ops import wheel_parking_hold as parking


def array(values, *, integer=False):
    return list(values)


def action(**kwargs):
    return SimpleNamespace(**{"joint_positions": None, "joint_efforts": None, **kwargs})


class Controller:
    def __init__(self, native):
        self.native = native
        self.kps, self.kds = [0., 150., 0., 200.], [8., 3., 4., 9.]
        self.efforts, self.modes = [20., 30., 10., 40.], ["force"]*4

    def get_gains(self):
        if not self.native.initialized:
            raise AssertionError("gain read before reset")
        return list(self.kps), list(self.kds)

    def get_max_efforts(self):
        return list(self.efforts)

    def get_effort_modes(self):
        return list(self.modes)

    def set_gains(self, kps, kds, save_to_usd=False):
        if save_to_usd:
            raise AssertionError("must not author USD gains")
        self.kps, self.kds = list(kps), list(kds)
        self.native.events.append(("gains", deepcopy(self.kps), deepcopy(self.kds)))


class Native:
    def __init__(self, **kwargs):
        self.dof_names = ["right", "caster", "left", "head"]
        self.wheel_dof_indices = [2, 0]
        self.positions = [.8, 1.2, -.4, 2.]
        self.position_targets, self.velocity_targets = [10., 11., 12., 13.], [20., 21., 22., 23.]
        self.events, self.wrapper_actions = [], []
        self.initialized = True
        self.controller = Controller(self)

    def get_dof_index(self, name):
        return self.dof_names.index(name)

    def get_articulation_controller(self):
        return self.controller

    def get_joint_positions(self):
        return list(self.positions)

    def apply_action(self, command):
        self.events.append(("action", deepcopy(command)))
        for slot, index in enumerate(command.joint_indices):
            if command.joint_positions is not None:
                self.position_targets[index] = command.joint_positions[slot]
            self.velocity_targets[index] = command.joint_velocities[slot]

    def apply_wheel_actions(self, command):
        self.wrapper_actions.append(command)


class WheelParkingHoldTests(unittest.TestCase):
    def hold(self, native=None):
        return parking.WheelParkingHold(native or Native(), ("left", "right"),
            lambda **kwargs: SimpleNamespace(joint_efforts=None, **kwargs), array_factory=array)

    def test_zero_latches_encoder_once_and_preserves_nonwheel_gains_targets_and_state(self):
        native = Native()
        native.controller.modes[2] = "acceleration"
        original_positions = list(native.positions)
        hold = self.hold(native)
        hold.apply([0., -0.])
        self.assertEqual(["gains", "action"], [event[0] for event in native.events])
        self.assertEqual([40., 150., 20., 200.], native.controller.kps)
        self.assertEqual([8., 3., 4., 9.], native.controller.kds)
        self.assertEqual([.8, 11., -.4, 13.], native.position_targets)
        self.assertEqual([0., 21., 0., 23.], native.velocity_targets)
        self.assertEqual(original_positions, native.positions)
        native.positions[0], native.positions[2] = 1.3, .6
        hold.apply([0., 0.])
        self.assertEqual([.8, 11., -.4, 13.], native.position_targets)
        self.assertEqual(1, sum(event[0] == "gains" for event in native.events))
        report = hold.report()
        self.assertEqual([2, 0], report["wheel_indices"])
        self.assertEqual([-.4, .8], report["anchor_rad"])
        self.assertEqual([20., 40.], report["readback"]["kps"])
        self.assertEqual([10., 20.], report["readback"]["max_efforts"])
        self.assertEqual(["acceleration", "force"], report["readback"]["effort_modes"])
        self.assertEqual((1, 0, 2), (report["hold_entries"], report["hold_releases"], report["hold_actions"]))
        json.dumps(report, allow_nan=False)
        report["original"]["kds"][0] = 99.
        report["anchor_rad"][0] = 99.
        self.assertEqual([4., 8.], hold.report()["original"]["kds"])
        self.assertEqual([-.4, .8], hold.report()["anchor_rad"])
        self.assertEqual([], native.wrapper_actions)

    def test_any_nonzero_releases_before_target_restores_gains_and_later_relatches(self):
        native = Native()
        hold = self.hold(native)
        hold.apply([1., -2.])
        self.assertEqual(["action"], [event[0] for event in native.events])
        hold.apply([0., 0.])
        native.controller.kps[1], native.controller.kds[3] = 151., 10.
        native.events.clear()
        hold.apply([1e-12, 0.])
        self.assertEqual(["gains", "action"], [event[0] for event in native.events])
        self.assertEqual([0., 151., 0., 200.], native.events[0][1])
        self.assertEqual([8., 3., 4., 10.], native.events[0][2])
        self.assertIsNone(native.events[1][1].joint_positions)
        self.assertEqual([0., 21., 1e-12, 23.], native.velocity_targets)
        self.assertEqual("velocity", hold.report()["state"])
        self.assertIsNone(hold.report()["anchor_rad"])
        native.positions[0], native.positions[2] = 1.5, 2.5
        hold.apply([0., 0.])
        self.assertEqual([2.5, 1.5], hold.report()["anchor_rad"])
        self.assertEqual((2, 1), (hold.report()["hold_entries"], hold.report()["hold_releases"]))

    def test_invalid_original_drives_or_indices_reject_before_any_write(self):
        for field, slot, value in (("kps", 2, 1.), ("kds", 2, 0.), ("kds", 0, -1.),
                                  ("kds", 2, math.nan), ("kds", 2, True), ("kds", 2, 1e308),
                                  ("efforts", 0, 0.), ("efforts", 2, math.inf),
                                  ("efforts", 2, -1.), ("modes", 2, "unknown")):
            with self.subTest(field=field, value=value):
                native = Native()
                getattr(native.controller, field)[slot] = value
                with self.assertRaises(ValueError):
                    self.hold(native)
                self.assertEqual([], native.events)
        for indices in ([0, 2], [2, 2], [True, 0]):
            native = Native()
            native.wheel_dof_indices = indices
            with self.assertRaises(ValueError):
                self.hold(native)
            self.assertEqual([], native.events)

    def test_bad_targets_encoders_and_converted_gains_fail_before_writes(self):
        for invalid in ([0.], [0., 0., 0.], [True, 0.], [math.inf, 0.], [math.nan, 0.],
                        ["0", 0.], [[0.], [0.]]):
            with self.subTest(target=invalid):
                native = Native()
                hold = self.hold(native)
                with self.assertRaises(ValueError):
                    hold.apply(invalid)
                self.assertEqual([], native.events)
        native = Native()
        hold = self.hold(native)
        native.positions[2] = math.nan
        with self.assertRaises(ValueError):
            hold.apply([0., 0.])
        self.assertEqual([], native.events)
        native = Native()
        hold = self.hold(native)

        def overflow(values, *, integer=False):
            return [math.inf]*4 if len(values) == 4 else list(values)

        hold.array = overflow
        with self.assertRaises(ValueError):
            hold.apply([0., 0.])
        self.assertEqual([], native.events)

    def test_native_failure_is_not_counted_as_success_or_retried_silently(self):
        native = Native()
        hold = self.hold(native)

        def failed_action(command):
            raise RuntimeError("native target failure")

        native.apply_action = failed_action
        with self.assertRaisesRegex(RuntimeError, "native target failure"):
            hold.apply([0., 0.])
        report = hold.report()
        self.assertEqual("faulted", report["state"])
        self.assertEqual(0, report["hold_entries"])
        self.assertIsNone(report["anchor_rad"])
        writes = len(native.events)
        with self.assertRaisesRegex(RuntimeError, "faulted"):
            hold.apply([1., 1.])
        self.assertEqual(writes, len(native.events))

    def test_facade_default_preserves_wrapper_and_optin_routes_indexed_actions(self):
        runtime = facade.Runtime45(None, SimpleNamespace(Action=action))
        old = runtime.WheeledRobot("/Old", ["left", "right"])
        enabled = runtime.WheeledRobot("/Enabled", ["left", "right"], wheel_parking_hold=True)
        self.assertEqual("not_initialized", enabled.parking_brake_report()["state"])
        old._native, enabled._native = Native(), Native()
        runtime.SimulationManager.ready = True
        old._initialize_parking_hold()
        with patch.object(parking, "_array", array):
            enabled._initialize_parking_hold()
            vector = [0., 0.]
            old.apply_wheel_actions(vector)
            enabled.apply_wheel_actions(vector)
        self.assertIs(vector, old._native.wrapper_actions[0].joint_velocities)
        self.assertEqual([], old._native.events)
        self.assertEqual([], enabled._native.wrapper_actions)
        self.assertEqual("disabled", old.parking_brake_report()["state"])
        self.assertEqual("hold", enabled.parking_brake_report()["state"])
        for option in (1, None, "true"):
            with self.assertRaises(ValueError):
                runtime.WheeledRobot("/Invalid", ["left", "right"], wheel_parking_hold=option)
        with self.assertRaises(ValueError):
            runtime.WheeledRobot("/Three", ["left", "right", "caster"], wheel_parking_hold=True)

    def test_setup_initializes_hold_after_reset_without_adding_world_or_updates(self):
        prim = SimpleNamespace(IsA=lambda schema: True, GetPath=lambda: "/Physics")
        stage = SimpleNamespace(Traverse=lambda: [prim])
        natives, events = [], []

        class World:
            @staticmethod
            def instance():
                return None

            def __init__(self, **kwargs):
                events.append("world")
                self.scene = SimpleNamespace(add=lambda native: native)

            def get_physics_context(self):
                return SimpleNamespace(enable_gpu_dynamics=lambda value: None)

            def reset(self):
                events.append("reset")
                for native in natives:
                    native.initialized = True

        def native_factory(**kwargs):
            native = Native()
            native.initialized = False
            natives.append(native)
            return native

        bindings = SimpleNamespace(World=World, Robot=native_factory, Action=action,
            usd=SimpleNamespace(get_context=lambda: SimpleNamespace(get_stage=lambda: stage)),
            physics=SimpleNamespace(Scene=object()))
        runtime = facade.Runtime45(None, bindings)
        robot = runtime.WheeledRobot("/Hold", ["left", "right"], wheel_parking_hold=True)
        runtime.SimulationManager.setup_simulation(1/60)
        self.assertEqual(["world", "reset"], events)
        self.assertEqual("velocity", robot.parking_brake_report()["state"])
        self.assertEqual([], natives[0].events)
        self.assertTrue(runtime.SimulationManager.ready)

    def test_modules_remain_cold_importable_without_numpy_or_sdk(self):
        original_import = builtins.__import__

        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in ("numpy", "omni", "isaacsim", "pxr"):
                raise AssertionError("unexpected eager import: " + name)
            return original_import(name, *args, **kwargs)

        for module in (facade, parking):
            path = Path(module.__file__)
            spec = importlib.util.spec_from_file_location("cold_063_" + path.stem, path)
            assert spec is not None and spec.loader is not None
            with patch("builtins.__import__", side_effect=guarded):
                spec.loader.exec_module(importlib.util.module_from_spec(spec))


if __name__ == "__main__":
    unittest.main()
