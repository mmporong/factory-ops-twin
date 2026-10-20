#!/usr/bin/env python3
"""Opt-in SDK4.5 four-AMR worker with native FlatScan evidence, separate from GMO."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.fleet_transport import QUALIFICATION, load_bundle
from scripts.isaac import g2_scene
from scripts.isaac.fleet4_transport_scene import FleetTransportLifecycle, SOURCE_PINS as BASE_SOURCE_PINS


RESULT_FILE = "local45-fleet4-transport-scene.json"
LOCAL_SOURCE_PINS = BASE_SOURCE_PINS | frozenset({
    "scripts/isaac/local45_fleet4_transport_scene.py", "factory_ops/isaac45_runtime.py",
    "factory_ops/flat_scan_observation.py", "factory_ops/flat_scan_ros.py", "factory_ops/flat_scan_completion.py", "factory_ops/compat.py",
    "factory_ops/operations/model.py", "scripts/isaac/robot_qualification.py",
    "scripts/isaac/nav_scene.py", "scripts/isaac/stage_smoke.py",
    "factory_ops/visual_state.py", "factory_ops/wsl_process_supervisor.py",
    "scripts/isaac/run_local45_fleet_pair.py",
    "scripts/isaac/windows_job_runner.py",
})


class Local45FleetLifecycle(FleetTransportLifecycle):
    def __init__(self, root: Path, run_id: str) -> None:
        super().__init__(root, run_id)
        self._native_observers: list[Any] = []
        self._native_journal_opened = False
        self._native_periodic_ns: int | None = None
        project = Path(__file__).resolve().parents[2]
        self._observer_sources = {name: g2_scene.base._sha256(project/name) for name in (
            "scripts/isaac/local45_fleet4_transport_scene.py", "factory_ops/isaac45_runtime.py",
            "factory_ops/flat_scan_observation.py")}

    def prepare_worker_paths(self, scene_arg: Path, output_arg: Path) -> tuple[Path, Path]:
        scene, output = super().prepare_worker_paths(scene_arg, output_arg)
        metadata = json.loads((output/"provenance/fleet-source.json").read_text(encoding="utf-8"))
        if (metadata.get("runtime_profile") != "local45_development"
                or not LOCAL_SOURCE_PINS.issubset(metadata["source_files"])):
            raise ValueError("local45 bootstrap must declare its runtime and all additional source pins")
        return scene, output

    def register_lidar_sensors(
        self, sensors: Sequence[Any], metadata: Sequence[Mapping[str, Any]], timeline: Any,
    ) -> dict[str, Any]:
        expected = {(robot, side) for robot in self.expected_robot_ids for side in ("front", "rear")}
        identities = {(row.get("robot_id"), row.get("name")) for row in metadata}
        if (self._native_observers or len(sensors) != len(expected) or len(metadata) != len(expected)
                or identities != expected or len({row.get("prim_path") for row in metadata}) != len(expected)
                or len({row.get("topic") for row in metadata}) != len(expected)):
            raise ValueError("local45 requires exactly the declared eight distinct robot sensors")
        self.timeline = timeline
        registrations = []
        try:
            for sensor, row in zip(sensors, metadata, strict=True):
                robot = self.registry.definitions[self.registry.robot_ids.index(row["robot_id"])]
                path = robot["prim_path"]+"/chassis_link/lidar_"+row["name"]
                topic = "/"+robot["namespace"]+"/scan_"+row["name"]+"_raw"
                if row["prim_path"] != path or row["topic"] != topic:
                    raise ValueError("native sensor prim/topic differs from the registry")
                identity = {"run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                    "robot_id": row["robot_id"], "side": row["name"], "prim_path": path,
                    "topic": topic, "frame_id": row["frame_id"],
                    "actual_profile_sha256": row["profile_sha256"]}
                observer = sensor.open_native_observer(identity, row["near_range_m"], row["far_range_m"])
                self._native_observers.append(observer)
                registrations.append({**identity, "status": "attached"})
        except BaseException:
            for observer in self._native_observers:
                try:
                    observer.close()
                except Exception:
                    pass  # Sensor ownership retains the handle for final cleanup.
            self._native_observers.clear()
            raise
        return {"schema_name": "local45-flat-scan-observation", "schema_version": 1,
            "annotator": "RtxSensorCpuIsaacComputeRTXLidarFlatScanSimulationTime",
            "sensor_count": len(expected), "registrations": registrations,
            "not_equivalent_to_gmo": True}

    def _record_lidar_acquisition(self, phase: str, sim_time_s: float, physics_frame: int) -> None:
        if not self._native_observers:
            return
        if self.output is None:
            raise RuntimeError("native lidar journal requires the owned output")
        path = self.output/"local45-lidar-observations.jsonl"
        if not self._native_journal_opened:
            if path.exists() or path.is_symlink():
                raise ValueError("native lidar journal must be new")
            self._native_journal_opened = True
        for observer in self._native_observers:
            row = observer.read(read_at_sim_time_s=sim_time_s, read_at_physics_frame=physics_frame,
                timeline_playing=bool(self.timeline is not None and self.timeline.is_playing()), phase=phase)
            row.update(runtime_version="4.5", observer_sources_sha256=self._observer_sources,
                read_cadence="periodic1Hz and capture pre/post; unique index counts only sampled native timestamps")
            g2_scene._append_jsonl(path, row)

    def step(self, sim_time_s: float, physics_frame: int) -> None:
        super().step(sim_time_s, physics_frame)
        stamp = round(sim_time_s*1_000_000_000)
        if self._native_observers and (self._native_periodic_ns is None
                or stamp-self._native_periodic_ns >= 1_000_000_000):
            self._record_lidar_acquisition("periodic_1hz", sim_time_s, physics_frame)
            self._native_periodic_ns = stamp

    def before_capture(self, rep: Any) -> None:
        if self.rgb_render_state != "enabled" or self.timeline is None:
            raise RuntimeError("local45 RGB render products are not active before capture")
        from isaacsim.core.api import World

        world = World.instance()
        if world is None:
            raise RuntimeError("local45 frozen capture requires an initialized World")
        # The outer G2 capture guard owns timeline pause/resume and frozen-state checks.
        for _ in range(4):
            world.render()

    def cleanup_render_products(self) -> None:
        errors = []
        for observer in self._native_observers:
            try:
                observer.close()
            except Exception as error:
                errors.append(str(error))
        if not errors:
            self._native_observers.clear()
        super().cleanup_render_products()
        if errors:
            raise RuntimeError("native observer cleanup: "+"; ".join(errors))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--capture-resolution", choices=("640x360", "1280x720"), default="640x360")
    args = parser.parse_args()
    root = args.bundle.resolve(strict=True)
    compiled = load_bundle(root)
    scenario_id = compiled["session"]["scenario_id"]
    shared = argparse.Namespace(scene=root/"scene/factory-v3.usda", layout=root/"layout.json",
        config=root/"config.json", scenarios=root/"scenarios.json", scenario=scenario_id,
        output=args.output, run_id=args.run_id)
    project = Path(__file__).resolve().parents[2]
    extras = {"local45_source_"+str(index): project/name for index, name in enumerate(sorted(LOCAL_SOURCE_PINS))}
    extras["fleet_bundle"] = root/"bundle.json"
    return g2_scene._run(shared, qualification=QUALIFICATION, scenario_ids={scenario_id},
        result_file=RESULT_FILE, expected_robot_count=4, local_sdk45=True,
        capture_resolution=tuple(int(value) for value in args.capture_resolution.split("x")),
        scene_extension=Local45FleetLifecycle(root, args.run_id), extra_inputs=extras)


if __name__ == "__main__":
    raise SystemExit(main())
