"""CPU contract checks; these substitutes provide no native SDK evidence."""
from __future__ import annotations

import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

from factory_ops.local45_stepping import Local45Stepper, physics_state
from scripts.isaac import local45_cadence_probe as probe


class World:
    def __init__(self):
        self.current_time = 2.0
        self.current_time_step_index = 120
        self.steps: list[bool] = []
        self.renders = 0
        self.frame_delta = 1
        self.time_delta = 1 / 60
        self.render_advances = False
        self.render_fails = False

    def step(self, *, render: bool):
        self.steps.append(render)
        self.current_time_step_index += self.frame_delta
        self.current_time += self.time_delta

    def render(self):
        self.renders += 1
        if self.render_fails:
            raise OSError("render failed")
        if self.render_advances:
            self.current_time_step_index += 1


class SteppingTests(unittest.TestCase):
    def test_one_native_step_each_call_and_render_each_six(self):
        world = World()
        stepper = Local45Stepper(world, 1 / 60)
        rows = [stepper.step() for _ in range(120)]
        self.assertEqual(world.steps, [False] * 120)
        self.assertEqual(world.renders, 20)
        self.assertEqual([row["physics_frame"] for row in rows], list(range(121, 241)))
        self.assertEqual([i + 1 for i, row in enumerate(rows) if row["rendered"]], list(range(6, 121, 6)))
        self.assertAlmostEqual(rows[-1]["sim_time_s"], 4.)

    def test_factor_one_uses_same_one_physics_step_contract(self):
        world = World()
        stepper = Local45Stepper(world, 1 / 60, 1)
        for _ in range(4):
            stepper.step()
        self.assertEqual(world.steps, [False] * 4)
        self.assertEqual(world.renders, 4)

    def test_missing_or_extra_native_step_and_bad_time_poison_instance(self):
        for frames, dt in ((0, 1 / 60), (2, 1 / 60), (1, 0.), (1, .1), (1, float("nan"))):
            with self.subTest(frames=frames, dt=dt):
                world = World()
                world.frame_delta, world.time_delta = frames, dt
                stepper = Local45Stepper(world, 1 / 60)
                with self.assertRaises(RuntimeError):
                    stepper.step()
                with self.assertRaisesRegex(RuntimeError, "stepper is failed"):
                    stepper.step()
                self.assertEqual(len(world.steps), 1)

    def test_render_failure_and_render_physics_advance_fail(self):
        for failure in ("render_fails", "render_advances"):
            with self.subTest(failure=failure):
                world = World()
                setattr(world, failure, True)
                stepper = Local45Stepper(world, 1 / 60, 1)
                with self.assertRaises((RuntimeError, OSError)):
                    stepper.step()
                with self.assertRaisesRegex(RuntimeError, "stepper is failed"):
                    stepper.step()
                self.assertEqual(world.renders, 1)

    def test_external_step_is_rejected_before_own_step(self):
        world = World()
        stepper = Local45Stepper(world, 1 / 60)
        world.current_time_step_index += 1
        with self.assertRaisesRegex(RuntimeError, "outside"):
            stepper.step()
        self.assertFalse(world.steps)

    def test_invalid_configuration_and_native_state(self):
        for dt, factor in ((0., 6), (float("inf"), 6), (True, 6), (1 / 60, True), (1 / 60, 2)):
            with self.assertRaises(ValueError):
                Local45Stepper(World(), dt, factor)
        for frame, stamp in ((True, 1.), (1.5, 1.), (-1, 1.), (1, float("inf")), (1, -1.)):
            world: Any = World()
            world.current_time_step_index, world.current_time = frame, stamp
            with self.assertRaises(RuntimeError):
                physics_state(world)

    def test_output_refusal_preserves_existing_bytes(self):
        with TemporaryDirectory() as temporary:
            output = Path(temporary)
            sentinel = output / "original.txt"
            sentinel.write_bytes(b"preserve")
            with self.assertRaisesRegex(ValueError, "new absolute"):
                probe.inputs(Path("unused.json"), output, "a" * 32)
            self.assertEqual(sentinel.read_bytes(), b"preserve")
        with self.assertRaisesRegex(ValueError, "new absolute"):
            probe.inputs(Path("unused.json"), Path("relative-output"), "a" * 32)

    def test_cpu_preflight_binds_existing_fleet_and_supports_factor_one(self):
        original = probe.ROOT / "configs/local45-cadence-probe.json"
        with TemporaryDirectory() as temporary:
            output = Path(temporary) / "never-created"
            cfg, fleet, scene = probe.inputs(original, output, "a" * 32)
            self.assertEqual(len(fleet["robots"]) * len(fleet["lidars"]), 8)
            self.assertEqual(probe.sha(scene), fleet["scene_sha256"])
            cfg.update(render_every_steps=1, rendering_dt_s=1 / 60)
            alternative = Path(temporary) / "factor-one.json"
            alternative.write_text(json.dumps(cfg), encoding="utf-8")
            self.assertEqual(probe.inputs(alternative, output, "b" * 32)[0]["render_every_steps"], 1)
            cfg["fleet_config_sha256"] = "0" * 64
            alternative.write_text(json.dumps(cfg), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA/path"):
                probe.inputs(alternative, output, "b" * 32)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
