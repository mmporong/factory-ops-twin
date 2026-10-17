#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Two-Carter G3 worker using the qualified G2 drive/sensor/capture loop."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.g3_session import verify_session
from scripts.isaac import g2_scene


class BarrierLifecycle:
    """Exogenous collider schedule; no truth is supplied to the dispatcher."""

    def __init__(self, session: dict[str, Any], run_id: str) -> None:
        self.session = session
        self.run_id = run_id
        self.collision = self.visible = None
        self.output: Path | None = None
        self.last_state: bool | None = None
        self.origin: float | None = None
        self.control_sequences: dict[str, int | None] = {}

    def setup(self, stage: Any, UsdGeom: Any, UsdPhysics: Any, Gf: Any, output: Path) -> dict[str, Any]:
        self.output = output
        specification = self.session["scene_closure"]
        if specification is not None:
            x0, y0, x1, y1 = specification["rect_xyxy_m"]
            height = specification["height_m"]
            cube = UsdGeom.Cube.Define(stage, "/World/G3Barrier")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, height / 2))
            cube.AddScaleOp().Set(Gf.Vec3d(x1 - x0, y1 - y0, height))
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.33, 0.08)])
            self.collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
            self.visible = UsdGeom.Imageable(cube.GetPrim()).CreateVisibilityAttr("inherited")
        self.step(0.0, 0)
        return {"kind": "g3_scheduled_physical_barrier", "prim_path": "/World/G3Barrier" if specification else None,
                "schedule": specification, "ground_truth_control_input": False}

    def step(self, sim_time_s: float, physics_frame: int) -> None:
        assert self.output is not None
        origin_path = self.output / "mission-start.json"
        if self.origin is None and origin_path.is_file():
            start = json.loads(origin_path.read_text(encoding="utf-8"))
            if start.get("run_id") != self.run_id or start.get("scenario_id") != self.session["scenario_id"]:
                raise ValueError("G3 mission-start identity mismatch")
            raw_origin = start["sim_time_s"]
            if type(raw_origin) not in (int, float) or not math.isfinite(raw_origin) or not 0 <= raw_origin <= sim_time_s:
                raise ValueError("G3 mission-start time is invalid")
            self.origin = float(raw_origin)
        specification = self.session["scene_closure"]
        closed = specification is not None
        if closed and self.origin is not None:
            begin, end = specification["scene_active_interval_s"]
            closed = begin <= sim_time_s - self.origin < end
        if closed != self.last_state:
            if self.collision is not None:
                assert self.visible is not None
                self.collision.Set(closed)
                self.visible.Set("inherited" if closed else "invisible")
                if self.collision.Get() is not closed:
                    raise ValueError("G3 barrier collision readback differs")
            state = {"schema_version": 1, "run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                     "current_closed": closed, "sim_time_s": sim_time_s, "physics_frame": physics_frame,
                     "collision_enabled_readback": closed if self.collision is not None else False}
            g2_scene._atomic_json(self.output / "scene-state.json", state)
            g2_scene._append_jsonl(self.output / "scene-events.jsonl", state)
            self.last_state = closed

    def after_control(self, robot_id: str, sample: dict[str, Any]) -> None:
        assert self.output is not None
        authorization = sample["authorization"]
        sequence = authorization["sequence"]
        if sequence is None or self.control_sequences.get(robot_id) == sequence:
            return
        acknowledgment = {"run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                          "robot_id": robot_id, "authorization_seq": sequence,
                          "mode": authorization["mode"], "valid": authorization["valid"],
                          "evaluation_sim_time_s": authorization["evaluation_sim_time_s"],
                          "physics_frame": sample["physics_frame"],
                          "input_command": sample["motion_profile"]["input_command"],
                          "scope": authorization["payload"]["scope"],
                          "source": "post-step drive authorization evaluation; no ground truth"}
        g2_scene._atomic_json(self.output / "robots" / robot_id / "control-ack.json", acknowledgment)
        self.control_sequences[robot_id] = sequence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    root = args.session.resolve(strict=True)
    try:
        session = verify_session(root, expected_sha256=os.environ["FACTORY_OPS_G3_SESSION_SHA256"])
    except Exception as error:
        g2_scene._atomic_json(args.output / "g3-scene.json", {"run_id": args.run_id, "status": "failed", "stage": "session_preflight", "error": str(error)})
        raise
    shared = argparse.Namespace(scene=root / "scene/factory-v3.usda", layout=root / "layout.json",
                                config=root / "config.json", scenarios=root / "scenarios.json",
                                scenario=session["scenario_id"], output=args.output, run_id=args.run_id)
    code = g2_scene._run(shared, qualification="g3_two_robot_orders", scenario_ids={session["scenario_id"]},
                         result_file="g3-scene.json", scene_extension=BarrierLifecycle(session, args.run_id),
                         extra_inputs={"g3_scene_source": Path(__file__).resolve(), "session": root / "session.json"})
    print(f"FACTORY_OPS_G3_SCENE_STATUS={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
