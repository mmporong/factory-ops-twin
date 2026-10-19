#!/usr/bin/env python3
"""Opt-in four-Carter stationary smoke; the G2/G3 qualification paths stay separate."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.fleet_bootstrap import QUALIFICATION, SCENARIO, validate_result, verify_bundle
from scripts.isaac import g2_scene
from scripts.isaac.g3_scene import BarrierLifecycle


class FleetSmokeLifecycle(BarrierLifecycle):
    def __init__(self, root: Path, run_id: str) -> None:
        config = json.loads((root / "config.json").read_text(encoding="utf-8"))
        self.profile = json.loads((root / "profile.json").read_text(encoding="utf-8"))
        self.robot_ids = tuple(r["robot_id"] for r in config["robots"])
        self.physics_dt_s = config["physics_dt_s"]
        self.control_samples = {robot: 0 for robot in self.robot_ids}
        self.latest_control: dict[str, float] = {}
        self.first_control_time_s: float | None = None
        self.completed = False
        super().__init__({"scenario_id": SCENARIO, "scene_closure": None}, run_id, root,
                         robot_ids=self.robot_ids, config_directory=root)

    def prepare_worker_paths(self, scene_arg: Path, output_arg: Path) -> tuple[Path, Path]:
        verify_bundle(self.session_root)
        return g2_scene.base._prepare_worker_paths(scene_arg, output_arg)

    def capture_due(self) -> bool:
        return False  # Shared worker records the first and final frame only.

    def after_control(self, robot_id: str, sample: dict[str, Any]) -> None:
        super().after_control(robot_id, sample)
        if robot_id not in self.control_samples:
            raise ValueError("stationary smoke received an undeclared robot")
        if any(sample["command"].get(key) != 0 for key in ("linear_m_s", "angular_rad_s")):
            raise ValueError("stationary smoke received a nonzero applied command")
        for key, limit_key in (("linear_m_s", "maximum_stationary_linear_m_s"),
                               ("angular_rad_s", "maximum_stationary_angular_rad_s")):
            value = sample["odometry"].get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > self.profile[limit_key]:
                raise ValueError(f"stationary encoder motion exceeded envelope: {robot_id}/{key}")
        stamp = sample["sim_time_s"]
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= self.latest_control.get(robot_id, -1):
            raise ValueError("stationary control sample timestamp differs")
        if self.first_control_time_s is None:
            self.first_control_time_s = float(stamp)
        self.latest_control[robot_id] = float(stamp)
        self.control_samples[robot_id] += 1

    def step(self, sim_time_s: float, physics_frame: int) -> None:
        super().step(sim_time_s, physics_frame)
        if self.completed or self.first_control_time_s is None or self.output is None:
            return
        if (sim_time_s - self.first_control_time_s >= self.profile["observation_window_s"]
                and all(n >= self.profile["minimum_control_samples_per_robot"] for n in self.control_samples.values())
                and all(0 <= sim_time_s - self.latest_control[r] <= 2 * self.physics_dt_s for r in self.robot_ids)):
            g2_scene._atomic_json(self.output / g2_scene.CONTROL_FILE,
                                 {"run_id": self.run_id, "scenario_id": SCENARIO, "status": "passed",
                                  "scope": "stationary sensor smoke only; no navigation goal issued",
                                  "control_samples": dict(self.control_samples)})
            self.completed = True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    root = args.bundle.resolve(strict=True)
    verify_bundle(root)
    shared = argparse.Namespace(scene=root / "scene/factory-v3.usda", layout=root / "layout.json",
                                config=root / "config.json", scenarios=root / "scenarios.json",
                                scenario=SCENARIO, output=args.output, run_id=args.run_id)
    code = g2_scene._run(shared, qualification=QUALIFICATION, scenario_ids={SCENARIO},
                         result_file="fleet4-scene.json", expected_robot_count=4,
                         scene_extension=FleetSmokeLifecycle(root, args.run_id),
                         extra_inputs={"fleet_bundle": root / "bundle.json", "fleet_profile": root / "profile.json",
                                       "fleet_worker_source": Path(__file__).resolve()})
    if code == 0:
        report = validate_result(args.output, root)
        g2_scene._atomic_json(args.output / "fleet4-smoke-validation.json", report)
        code = 0 if report["passed"] else 1
    print(f"FACTORY_OPS_FLEET4_SENSOR_SMOKE={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
