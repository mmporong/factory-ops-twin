#!/usr/bin/env python3
"""Opt-in SDK4.5 four-AMR worker with native FlatScan evidence, separate from GMO."""
from __future__ import annotations

import argparse
from copy import deepcopy
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
    "scripts/isaac/local45_fleet4_transport_scene.py", "factory_ops/isaac45_runtime.py", "factory_ops/wheel_parking_hold.py",
    "factory_ops/flat_scan_observation.py", "factory_ops/flat_scan_ros.py", "factory_ops/flat_scan_completion.py", "factory_ops/physx_planar.py", "factory_ops/world_clock.py", "factory_ops/local45_stepping.py", "factory_ops/compat.py",
    "factory_ops/operations/model.py", "scripts/isaac/robot_qualification.py",
    "scripts/isaac/nav_scene.py", "scripts/isaac/stage_smoke.py",
    "factory_ops/visual_state.py", "factory_ops/wsl_process_supervisor.py",
    "factory_ops/material_visual.py",
    "factory_ops/planner_guidance.py",
    "scripts/isaac/run_local45_fleet_pair.py",
    "scripts/isaac/windows_job_runner.py",
})


class EmptyToteGeometry:
    """Non-colliding hollow markers using the existing authored cargo envelope/supports."""
    def __init__(self, visuals: Any):
        self.visuals, self.copies = visuals, {}

    def apply(self, rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]] | None:
        from scripts.isaac.factory_visuals import TOTE_SIZE_M, world_slot_placement, _session_edit_target
        v, placements = self.visuals, []
        occupied = {(row["location"], row["slot"]) for row in (v.last_event or {}).get("displayed_payloads", [])}
        for row in rows:
            row = dict(row)
            if row["robot_id"] is None:
                capacity = world_slot_placement(v.layout, row["location"], 0)["capacity"]
                free = [slot for slot in range(capacity) if (row["location"], slot) not in occupied]
                if not free:
                    return None  # Wait for the existing product projection to free a support slot.
                row["slot"] = row["slot"] if row["slot"] in free else free[0]
                occupied.add((row["location"], row["slot"]))
                placement = world_slot_placement(v.layout, row["location"], row["slot"])
            else:
                if (row["location"], 0) in occupied:
                    return None
                placement = {"parent_path": v.robot_paths[row["robot_id"]] + "/FactoryVisualCargo",
                    "translate_xyz_m": [0., 0., .52], "size_xyz_m": list(TOTE_SIZE_M),
                    "scope": "existing authored robot cargo offset; no measured mount/contact claim"}
            suffix = f"{row['location']}_{row['slot']}".replace("-", "_")
            path = f"{placement['parent_path']}/Empty_{row['tote_id'].replace('-', '_')}_{suffix}"
            placements.append({**dict(row), **placement, "prim_path": path})
        with _session_edit_target(v.stage):
            for item in placements:
                path = item["prim_path"]
                if path not in self.copies:
                    group = v.UsdGeom.Xform.Define(v.stage, path)
                    group.AddTranslateOp().Set(v.Gf.Vec3d(*item["translate_xyz_m"]))
                    group.GetPrim().SetCustomDataByKey("materialToteId", item["tote_id"])
                    sx, sy, sz = TOTE_SIZE_M
                    wall = min(TOTE_SIZE_M) * .08  # Authored visual thickness, not a measured tote.
                    parts = [("Base", (0, 0, -sz/2+wall/2), (sx, sy, wall)),
                        ("Left", (-(sx-wall)/2, 0, wall/2), (wall, sy, sz-wall)),
                        ("Right", ((sx-wall)/2, 0, wall/2), (wall, sy, sz-wall)),
                        ("Front", (0, -(sy-wall)/2, wall/2), (sx-2*wall, wall, sz-wall)),
                        ("Rear", (0, (sy-wall)/2, wall/2), (sx-2*wall, wall, sz-wall))]
                    for name, xyz, size in parts:
                        cube = v.UsdGeom.Cube.Define(v.stage, path + "/" + name)
                        cube.CreateSizeAttr(1.)
                        cube.AddTranslateOp().Set(v.Gf.Vec3d(*xyz))
                        cube.AddScaleOp().Set(v.Gf.Vec3d(*size))
                        cube.CreateDisplayColorAttr([v.Gf.Vec3f(.92, .56, .12)])
                        if cube.GetPrim().HasAPI(v.UsdPhysics.CollisionAPI):
                            raise ValueError("empty tote marker unexpectedly has a collider")
                    self.copies[path] = v.UsdGeom.Imageable(group.GetPrim())
            visible = {item["prim_path"] for item in placements}
            for path, imageable in self.copies.items():
                imageable.CreateVisibilityAttr().Set("inherited" if path in visible else "invisible")
        return placements

    def capture_placements(self, metadata: dict[str, Any]) -> dict[str, Any]:
        applied = metadata["applied"]
        if applied is not None:
            cache = self.visuals.UsdGeom.XformCache()
            for item in applied["placements"]:
                matrix = cache.GetLocalToWorldTransform(self.visuals.stage.GetPrimAtPath(item["prim_path"]))
                item["captured_world_transform"] = [[float(matrix[i][j]) for j in range(4)] for i in range(4)]
        return metadata


class Local45FleetLifecycle(FleetTransportLifecycle):
    def __init__(self, root: Path, run_id: str, *, lidar_backend: str = "rtx_flat_scan") -> None:
        super().__init__(root, run_id)
        if lidar_backend not in {"rtx_flat_scan", "physx_planar"}:
            raise ValueError("unsupported local45 observation backend")
        self.lidar_backend = lidar_backend
        self.material_visual = self.empty_geometry = None
        self.material_capture = None
        self.last_captured_material_revision = None
        self.last_material_product_sequence = None
        if lidar_backend == "physx_planar" and "material_flow" in self.session["workload"]:
            from factory_ops.material_visual import AppliedMaterialVisual
            self.material_visual = AppliedMaterialVisual(run_id, self.session["scenario_id"],
                self.session["workload"]["material_flow"]["materials"], self.expected_robot_ids,
                self.session["workload"]["load"]["ci_capacity"])
        self._native_observers: list[Any] = []
        self._native_journal_opened = False
        self._native_periodic_ns: int | None = None
        project = Path(__file__).resolve().parents[2]
        self._observer_sources = {name: g2_scene.base._sha256(project/name) for name in (
            "scripts/isaac/local45_fleet4_transport_scene.py", "factory_ops/isaac45_runtime.py",
            "factory_ops/flat_scan_observation.py")}
        if self.lidar_backend == "physx_planar":
            self._observer_sources["factory_ops/physx_planar.py"] = g2_scene.base._sha256(project/"factory_ops/physx_planar.py")

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
                expected_backend = "physx_collision_planar" if self.lidar_backend == "physx_planar" else "rtx_flat_scan"
                if row.get("sensor_backend", "rtx_flat_scan") != expected_backend:
                    raise ValueError("sensor metadata differs from the selected observation backend")
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
        return {"schema_name": "local45-physx-planar-observation" if self.lidar_backend == "physx_planar" else "local45-flat-scan-observation", "schema_version": 1,
            "annotator": None if self.lidar_backend == "physx_planar" else "RtxSensorCpuIsaacComputeRTXLidarFlatScanSimulationTime",
            "sensor_backend": self.lidar_backend,
            "sensor_count": len(expected), "registrations": registrations,
            "not_equivalent_to_gmo": True}

    def _record_lidar_acquisition(self, phase: str, sim_time_s: float, physics_frame: int) -> None:
        if not self._native_observers:
            return
        if self.output is None:
            raise RuntimeError("native lidar journal requires the owned output")
        path = self.output/("physx-planar-observations.jsonl" if self.lidar_backend == "physx_planar" else "local45-lidar-observations.jsonl")
        if not self._native_journal_opened:
            if path.exists() or path.is_symlink():
                raise ValueError("native lidar journal must be new")
            self._native_journal_opened = True
        for observer in self._native_observers:
            row = observer.read(read_at_sim_time_s=sim_time_s, read_at_physics_frame=physics_frame,
                timeline_playing=bool(self.timeline is not None and self.timeline.is_playing()), phase=phase)
            row.update(runtime_version="4.5", observer_sources_sha256=self._observer_sources,
                read_cadence=("periodic1Hz and capture pre/post; cached authored World acquisition, no native timestamp"
                    if self.lidar_backend == "physx_planar" else
                    "periodic1Hz and capture pre/post; unique index counts only sampled native timestamps"))
            g2_scene._append_jsonl(path, row)

    def step(self, sim_time_s: float, physics_frame: int) -> None:
        super().step(sim_time_s, physics_frame)
        if self.material_visual is not None and self.empty_geometry is not None:
            from factory_ops.material_visual import STATE_FILE
            assert self.output is not None
            assert self.visuals is not None
            path = self.output / STATE_FILE
            if path.is_symlink():
                raise ValueError("material visual companion cannot be a symlink")
            if path.is_file():
                self.material_visual.offer(json.loads(path.read_text(encoding="utf-8")))
            self.material_visual.advance(sim_time_s, physics_frame, self.empty_geometry.apply)
            if (self.material_visual.applied is not None
                    and self.last_material_product_sequence != self.visuals.applied_sequence):
                placements = self.empty_geometry.apply(self.material_visual.applied["document"]["empty_totes"])
                if placements is None:
                    raise ValueError("applied empty tote placements conflict with current product markers")
                self.material_visual.applied["placements"] = placements
                self.material_visual.applied["placement_physics_frame"] = physics_frame
                self.last_material_product_sequence = self.visuals.applied_sequence
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
        if self.material_visual is not None and self.empty_geometry is not None:
            self.material_capture = self.empty_geometry.capture_placements(self.material_visual.capture_metadata())
            self.material_capture["captured_physics_frame"] = int(world.current_time_step_index)
            self.material_capture["captured_sim_time_s"] = float(world.current_time)
        # The outer G2 capture guard owns timeline pause/resume and frozen-state checks.
        if self.lidar_backend == "physx_planar":
            rep.orchestrator.step(delta_time=0.0, rt_subframes=8, pause_timeline=True)
            rep.orchestrator.wait_until_complete()
        else:
            for _ in range(4):
                world.render()

    def after_robots_loaded(self, stage: Any, definitions: dict[str, Any]) -> dict[str, Any]:
        metadata = super().after_robots_loaded(stage, definitions)
        if self.material_visual is not None:
            self.empty_geometry = EmptyToteGeometry(self.visuals)
            metadata["empty_totes"] = "optional material companion; authored hollow visual-only geometry"
        return metadata

    def capture_due(self) -> bool:
        due = super().capture_due()
        applied = self.material_visual.applied if self.material_visual is not None else None
        return due or applied is not None and applied["document"]["material_revision"] != self.last_captured_material_revision

    def capture_metadata(self) -> dict[str, Any]:
        metadata = super().capture_metadata()
        if self.material_visual is not None:
            metadata["native_empty_totes"] = deepcopy(self.material_capture)
            applied = self.material_capture["applied"] if self.material_capture is not None else None
            self.last_captured_material_revision = applied["document"]["material_revision"] if applied else None
        return metadata

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
    parser.add_argument("--lidar-backend", choices=("rtx_flat_scan", "physx_planar"), default="rtx_flat_scan")
    parser.add_argument("--physics-cadence", choices=("app_update", "owned_step6"), default="owned_step6")
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
        lidar_backend=args.lidar_backend, physics_cadence=args.physics_cadence,
        scene_extension=Local45FleetLifecycle(root, args.run_id, lidar_backend=args.lidar_backend), extra_inputs=extras)


if __name__ == "__main__":
    raise SystemExit(main())
