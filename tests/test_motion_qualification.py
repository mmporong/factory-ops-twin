from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.drive_profile import DriveProfile
from factory_ops.motion import DriveGeometry
from scripts.isaac.motion_qualification import (
    _drive_limits,
    _joint_velocities,
    _load_config,
    _prepare_output,
    _schedule_entry,
    _testbench_usda,
)
from scripts.isaac.stage_smoke import SmokeError


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs" / "factory-v3" / "motion-qualification.json"


class MotionQualificationTests(unittest.TestCase):
    def test_config_is_contiguous_bounded_and_covers_realistic_motion_transitions(self) -> None:
        config = _load_config(CONFIG)
        schedule = config["schedule"]
        self.assertEqual(float(schedule[0]["start_s"]), 0.0)
        self.assertLess(float(schedule[-1]["end_s"]), 80.0)
        self.assertEqual(int(config["capture_every_steps"]), 6)
        self.assertEqual(int(config["warmup_steps"]), 120)
        self.assertIn("not measured motor calibration", config["drive_limits"]["provenance"])
        self.assertEqual(_schedule_entry(schedule, 10.0)["name"], "normal_brake")
        self.assertEqual(_schedule_entry(schedule, 15.0)["name"], "turn_approach")
        self.assertEqual(_schedule_entry(schedule, 44.0)["name"], "opposite_curvature")
        self.assertTrue(_schedule_entry(schedule, 75.0)["emergency_stop"])
        for window in config["steady_evaluation_windows"]:
            self.assertGreaterEqual(float(window["last_s"]), 1.0)

    def test_generated_profile_brakes_before_turn_and_settles_before_departure(self) -> None:
        config = _load_config(CONFIG)
        geometry = DriveGeometry(
            float(config["drive_geometry"]["wheel_radius_m"]),
            float(config["drive_geometry"]["wheel_separation_m"]),
        )
        profile = DriveProfile(geometry, _drive_limits(config["drive_limits"]))
        dt = float(config["physics_dt_s"])
        phases_by_segment: dict[str, set[str]] = {}
        total_steps = round(float(config["schedule"][-1]["end_s"]) / dt)
        for step in range(total_steps):
            entry = _schedule_entry(config["schedule"], step * dt)
            command = profile.step(
                float(entry["linear_m_s"]), float(entry["angular_rad_s"]), dt,
                emergency_stop=bool(entry["emergency_stop"]),
            )
            phases_by_segment.setdefault(str(entry["name"]), set()).add(command.phase)
        self.assertIn("brake_before_turn", phases_by_segment["translate_to_inplace_turn"])
        self.assertIn("turn_in_place", phases_by_segment["translate_to_inplace_turn"])
        self.assertIn("settle_before_departure", phases_by_segment["turn_settle_departure"])
        self.assertIn("translate", phases_by_segment["turn_settle_departure"])

    def test_config_rejects_schedule_gap_and_short_steady_window(self) -> None:
        source = json.loads(CONFIG.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "bad.json"
            source["schedule"][1]["start_s"] = 10.1
            target.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "contiguous"):
                _load_config(target)
            source = json.loads(CONFIG.read_text(encoding="utf-8"))
            source["steady_evaluation_windows"][0]["last_s"] = 0.5
            target.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(SmokeError, "at least one second"):
                _load_config(target)

    def test_output_requires_unique_nonexistent_absolute_directory_and_hex_run_id(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "new-run"
            self.assertEqual(_prepare_output(output, "a" * 32), output.resolve())
            with self.assertRaisesRegex(SmokeError, "must not already exist"):
                _prepare_output(output, "b" * 32)
            with self.assertRaisesRegex(SmokeError, "lowercase hex"):
                _prepare_output(Path(folder) / "other", "G" * 32)

    def test_testbench_has_separate_visual_cube_and_invisible_z_plane(self) -> None:
        config = _load_config(CONFIG)
        text = _testbench_usda(config)
        self.assertIn('def Cube "FloorVisual"', text)
        self.assertIn("bool physics:collisionEnabled = false", text)
        self.assertIn('def Plane "Floor"', text)
        self.assertIn('uniform token axis = "Z"', text)
        self.assertIn('token visibility = "invisible"', text)
        self.assertEqual(text.count("PhysicsCollisionAPI"), 1)
        self.assertIn("not a factory transport run", text)
        grid = text.split('def Scope "VisualReferenceGrid"', 1)[1]
        self.assertNotIn("PhysicsCollisionAPI", grid)
        self.assertIn('def Cube "GridX_neg_40"', grid)
        self.assertIn('def Cube "GridY_pos_40"', grid)
        self.assertIn('def Cube "WorldXAxis"', grid)
        self.assertIn('def Cube "WorldYAxis"', grid)
        self.assertGreater(grid.count("bool physics:collisionEnabled = false"), 150)

    def test_raw_joint_velocities_are_mapped_by_actual_dof_names(self) -> None:
        class Array:
            ndim = 2

            def __getitem__(self, index: int) -> list[float]:
                if index != 0:
                    raise IndexError(index)
                return [0.1, 1.2, -1.3]

        class Tensor:
            @staticmethod
            def numpy() -> Array:
                return Array()

        class Robot:
            dof_names = ["caster", "joint_wheel_left", "joint_wheel_right"]

            @staticmethod
            def get_dof_velocities() -> Tensor:
                return Tensor()

        self.assertEqual(_joint_velocities(Robot()), {
            "caster": 0.1, "joint_wheel_left": 1.2, "joint_wheel_right": -1.3,
        })


if __name__ == "__main__":
    unittest.main()
