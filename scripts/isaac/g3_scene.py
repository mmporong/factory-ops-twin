#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""Two-Carter G3 worker using the qualified G2 drive/sensor/capture loop."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import sys
from typing import Any, Mapping, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.g3_session import config_dir_from_session, verify_session
from factory_ops.g3_build_attestation import PRESERVED_BUILD_FILES
from factory_ops.source_provenance import verify_source_archive
from scripts.isaac import g2_scene
from scripts.isaac.factory_visuals import FactoryVisualRuntime


def _parse_gmo_header(data: Any) -> dict[str, int]:
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

    gmo = parse_generic_model_output_data(data)
    return {
        "frame_id": int(gmo.frameId),
        "timestamp_ns": int(gmo.timestampNs),
        "num_elements": int(gmo.numElements),
    }


class BarrierLifecycle:
    """Exogenous collider schedule; no truth is supplied to the dispatcher."""

    align_terminal_encoder = True

    def __init__(self, session: dict[str, Any], run_id: str, session_root: Path, *,
                 robot_ids: Sequence[str] = ("carter1", "carter2"), config_directory: Path | None = None) -> None:
        if not robot_ids or len(set(robot_ids)) != len(robot_ids) or any(not isinstance(r, str) or not r for r in robot_ids):
            raise ValueError("lidar diagnostics require distinct declared robot IDs")
        self.expected_robot_ids = tuple(robot_ids)
        self.config_directory = config_directory
        self.session = session
        self.run_id = run_id
        self.collision = self.visible = None
        self.output: Path | None = None
        self.last_state: bool | None = None
        self.origin: float | None = None
        self.control_sequences: dict[str, int | None] = {}
        self.session_root = session_root
        self.visuals: FactoryVisualRuntime | None = None
        self.last_captured_visual_sequence: int | None = None
        self.real_time_media = self._load_real_time_media_profile()
        self.media_window: dict[str, Any] | None = None
        self.media_window_ended = False
        self.current_media_frame: int | None = None
        self.current_media_time_ns: int | None = None
        self.last_media_due_frame: int | None = None
        self.pending_media_sample: dict[str, Any] | None = None
        self.media_journal_opened = False
        self.rgb_render_products: tuple[Any, Any, Any] | None = None
        self.rgb_render_state = "unregistered"
        self.render_breadcrumb_sequence = 0
        self.render_breadcrumb_opened = False
        self.timeline: Any = None
        self.lidar_diagnostics: list[tuple[Any, dict[str, Any], str, str | None]] = []
        self.lidar_event_sequence = 0
        self.lidar_journal_opened = False
        self.last_periodic_lidar_time_ns: int | None = None
        self.lidar_output_buffer_setting: dict[str, Any] = {"status": "not_read"}

    def _load_real_time_media_profile(self) -> dict[str, Any] | None:
        config_path = self.session_root / "config.json"
        if (self.session.get("profile") != "qualification"
                or self.session.get("primary_evaluation") is not False
                or not config_path.is_file() or config_path.is_symlink()):
            return None
        config = json.loads(config_path.read_text(encoding="utf-8"))
        profile = config.get("g3_runtime", {}).get("real_time_media")
        if not isinstance(profile, dict) or profile.get("profiles") != ["qualification"]:
            return None
        return {**profile, "source_profile_sha256": hashlib.sha256(
            json.dumps(profile, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        ).hexdigest()}

    def _append_media_event(self, event: dict[str, Any]) -> None:
        assert self.output is not None
        path = self.output / "real-time-media-events.jsonl"
        if not self.media_journal_opened:
            if path.exists() or path.is_symlink():
                raise ValueError("G3 real-time media journal must be new")
            self.media_journal_opened = True
        g2_scene._append_jsonl(path, event)

    def _observe_real_time_media(self, robot_id: str, sample: dict[str, Any]) -> None:
        profile = self.real_time_media
        if profile is None:
            return
        frame = sample.get("physics_frame")
        sim_time = sample.get("sim_time_s")
        if (isinstance(frame, bool) or not isinstance(frame, int) or frame < 0
                or isinstance(sim_time, bool) or not isinstance(sim_time, (int, float))
                or not math.isfinite(sim_time) or sim_time < 0):
            return
        sim_time_ns = round(float(sim_time) * 1_000_000_000)
        self.current_media_frame = frame
        self.current_media_time_ns = sim_time_ns
        if self.media_window is None:
            authorization = sample.get("authorization")
            motion_profile = sample.get("motion_profile")
            if not isinstance(authorization, dict) or not isinstance(motion_profile, dict):
                return
            command = motion_profile.get("input_command")
            if (authorization.get("mode") != "navigate" or authorization.get("valid") is not True
                    or not isinstance(command, dict)):
                return
            linear, angular = command.get("linear_m_s"), command.get("angular_rad_s")
            if (isinstance(linear, bool) or not isinstance(linear, (int, float))
                    or isinstance(angular, bool) or not isinstance(angular, (int, float))
                    or not math.isfinite(linear) or not math.isfinite(angular)
                    or (abs(linear) < profile["minimum_abs_linear_m_s"]
                        and abs(angular) < profile["minimum_abs_angular_rad_s"])):
                return
            duration_ns = round(float(profile["window_s"]) * 1_000_000_000)
            self.media_window = {
                "window_id": "qualification-motion-001",
                "source_profile_sha256": profile["source_profile_sha256"],
                "trigger_robot_id": robot_id,
                "trigger_command": {"linear_m_s": float(linear), "angular_rad_s": float(angular)},
                "trigger_frame": frame,
                "start_sim_time_ns": sim_time_ns,
                "end_sim_time_ns": sim_time_ns + duration_ns,
                "sample_every_physics_steps": profile["sample_every_physics_steps"],
            }
            self._append_media_event({"schema_version": 1, "event": "window_started",
                                      "run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                                      **self.media_window})
        if (not self.media_window_ended and self.media_window is not None
                and sim_time_ns >= self.media_window["end_sim_time_ns"]):
            self.media_window_ended = True
            self._append_media_event({"schema_version": 1, "event": "window_ended",
                                      "run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                                      "window_id": self.media_window["window_id"],
                                      "physics_frame": frame, "sim_time_ns": sim_time_ns})

    def prepare_worker_paths(self, scene_arg: Path, output_arg: Path) -> tuple[Path, Path]:
        if (not scene_arg.is_absolute() or not output_arg.is_absolute()
                or scene_arg.is_symlink() or output_arg.is_symlink()):
            raise g2_scene.base.SmokeError("G3 bootstrap paths must be absolute and nonsymlink")
        scene, output = scene_arg.resolve(strict=True), output_arg.resolve(strict=True)
        if (scene != (self.session_root / "scene/factory-v3.usda").resolve(strict=True)
                or not scene.is_file() or not output.is_dir()):
            raise g2_scene.base.SmokeError("G3 bootstrap scene/output differs from the verified session")
        required_files = {"session-verified.json", "provenance/source.tar",
                          "provenance/source-commit.raw", "provenance/source-environment.json",
                          "provenance/build-manifest.json", *PRESERVED_BUILD_FILES}
        required_directories = {parent.as_posix() for relative in required_files
                                for parent in PurePosixPath(relative).parents
                                if parent != PurePosixPath(".")}
        files, directories = set(), set()
        for path in output.rglob("*"):
            if path.is_symlink():
                raise g2_scene.base.SmokeError("G3 bootstrap contains a symlink")
            relative = path.relative_to(output).as_posix()
            if path.is_file():
                files.add(relative)
            elif path.is_dir():
                directories.add(relative)
            else:
                raise g2_scene.base.SmokeError("G3 bootstrap contains a special file")
        if files != required_files or directories != required_directories:
            raise g2_scene.base.SmokeError("G3 bootstrap exact file set differs; output must be a new attempt")
        source_commit = os.environ.get("FACTORY_OPS_G3_SOURCE_COMMIT", "")
        expected_session_sha = os.environ.get("FACTORY_OPS_G3_SESSION_SHA256", "")
        expected_environment_sha = os.environ.get("FACTORY_OPS_G3_PROVENANCE_SHA256", "")
        primary_sha = os.environ.get("FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256") or None
        if (re.fullmatch(r"[0-9a-f]{40}", source_commit) is None
                or re.fullmatch(r"[0-9a-f]{64}", expected_session_sha) is None
                or re.fullmatch(r"[0-9a-f]{64}", expected_environment_sha) is None
                or expected_session_sha != g2_scene.base._sha256(self.session_root / "session.json")
                or (self.session["primary_evaluation"] and re.fullmatch(r"[0-9a-f]{64}", primary_sha or "") is None)
                or (not self.session["primary_evaluation"] and primary_sha is not None)):
            raise g2_scene.base.SmokeError("G3 bootstrap supervisor source/session/input binding differs")
        marker = json.loads((output / "session-verified.json").read_text(encoding="utf-8"))
        expected_marker = {
            "schema_version": 1, "status": "verified", "run_id": self.run_id,
            "scenario_id": self.session["scenario_id"], "source_commit": source_commit,
            "session_sha256": expected_session_sha, "primary_evaluation": self.session["primary_evaluation"],
            "scope": self.session["scope"], "primary_inputs_sha256": primary_sha,
        }
        if marker != expected_marker:
            raise g2_scene.base.SmokeError("G3 bootstrap session verification marker differs")
        provenance = output / "provenance"
        environment_path = provenance / "source-environment.json"
        if g2_scene.base._sha256(environment_path) != expected_environment_sha:
            raise g2_scene.base.SmokeError("G3 bootstrap provenance bytes differ from the supervisor binding")
        environment = json.loads(environment_path.read_text(encoding="utf-8"))
        if (not isinstance(environment, dict) or environment.get("schema_version") != 1
                or environment.get("run_id") != self.run_id or environment.get("source_commit") != source_commit
                or environment.get("checkout_head") != source_commit or environment.get("checkout_clean") is not True
                or environment.get("source_git_object_verification") is not True
                or environment.get("session_json_sha256") != expected_session_sha
                or environment.get("scenario_id") != self.session["scenario_id"]):
            raise g2_scene.base.SmokeError("G3 bootstrap source environment identity differs")
        manifest_path = provenance / "build-manifest.json"
        if (environment.get("build_provenance_manifest_path") != "provenance/build-manifest.json"
                or g2_scene.base._sha256(manifest_path) != environment.get("build_provenance_manifest_sha256")):
            raise g2_scene.base.SmokeError("G3 bootstrap build provenance manifest pin differs")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if (not isinstance(manifest, dict) or set(manifest) != {"schema_version", "classification", "files"}
                or manifest.get("schema_version") != 1
                or manifest.get("classification") != "g3_build_provenance_bundle"
                or not isinstance(manifest.get("files"), list)):
            raise g2_scene.base.SmokeError("G3 bootstrap build provenance manifest differs")
        observed = set()
        for entry in manifest["files"]:
            if (not isinstance(entry, dict) or set(entry) != {"path", "bytes", "sha256"}
                    or not isinstance(entry.get("path"), str)
                    or entry["path"] not in PRESERVED_BUILD_FILES or entry["path"] in observed
                    or type(entry.get("bytes")) is not int or entry["bytes"] < 0
                    or not isinstance(entry.get("sha256"), str)
                    or re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]) is None):
                raise g2_scene.base.SmokeError("G3 bootstrap build provenance entry differs")
            path = output / entry["path"]
            if path.stat().st_size != entry["bytes"] or g2_scene.base._sha256(path) != entry["sha256"]:
                raise g2_scene.base.SmokeError("G3 bootstrap build provenance bytes differ")
            observed.add(entry["path"])
        if observed != PRESERVED_BUILD_FILES:
            raise g2_scene.base.SmokeError("G3 bootstrap build provenance exact set differs")
        source_files = environment.get("source_files")
        if not isinstance(source_files, dict) or not {
            "scripts/isaac/g3_scene.py", "scripts/isaac/g2_scene.py", "factory_ops/source_provenance.py"
        }.issubset(source_files):
            raise g2_scene.base.SmokeError("G3 bootstrap required source pins are absent")
        proof = verify_source_archive(provenance / "source.tar", provenance / "source-commit.raw",
                                      source_commit, source_files, snapshot_root=self.session_root.parents[1])
        if (proof.get("passed") is not True
                or proof.get("source_archive_sha256") != environment.get("source_archive_sha256")
                or proof.get("commit_object_sha256") != environment.get("source_commit_object_sha256")):
            raise g2_scene.base.SmokeError("G3 bootstrap source archive proof differs")
        return scene, output

    def setup(self, stage: Any, UsdGeom: Any, UsdPhysics: Any, Gf: Any, output: Path) -> dict[str, Any]:
        self.output = output
        self.visuals = FactoryVisualRuntime(
            layout_path=self.session_root / "layout.json",
            base_scene_path=self.session_root / "scene/factory-v3.usda", output=output,
            variant_seed=0, run_id=self.run_id, scenario_id=self.session["scenario_id"],
            **({"config_directory": self.config_directory} if self.config_directory is not None else {}))
        industrial_visuals = self.visuals.setup(stage, UsdGeom, UsdPhysics, Gf)
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
                "schedule": specification, "ground_truth_control_input": False,
                "industrial_visuals": industrial_visuals}

    def after_robots_loaded(self, stage: Any, definitions: dict[str, Any]) -> dict[str, Any]:
        assert self.visuals is not None
        return self.visuals.after_robots_loaded(stage, definitions)

    def configure_cameras(self, stage: Any, capture: Any, second: Any, UsdGeom: Any, Gf: Any) -> dict[str, Any]:
        profile_path = (self.config_directory or config_dir_from_session(self.session_root)) / "render-profile.json"
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        cameras_by_id = {row["id"]: row for row in profile["cameras"]}
        specifications = [
            ("portfolio_overview", "/World/FactoryOverviewCamera"),
            ("carter1_follow", capture.medium_camera.path),
            ("carter2_follow", second.path),
        ]
        records = []
        for camera_id, path in specifications:
            specification = cameras_by_id[camera_id]
            eye, target, focal_length = (specification["eye_xyz_m"], specification["target_xyz_m"], specification["focal_length_mm"])
            camera = UsdGeom.Camera(stage.GetPrimAtPath(path))
            if not camera or not camera.GetPrim().IsValid():
                raise ValueError("industrial camera prim is missing")
            transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
            operations = UsdGeom.Xformable(camera.GetPrim()).GetOrderedXformOps()
            if len(operations) != 1:
                raise ValueError("industrial camera transform stack differs")
            operations[0].Set(transform)
            camera.GetFocalLengthAttr().Set(float(focal_length))
            actual_transform = operations[0].Get()
            if any(abs(float(actual_transform[row][column]) - float(transform[row][column])) > 1e-9
                   for row in range(4) for column in range(4)):
                raise ValueError("industrial camera transform readback differs")
            if not math.isclose(float(camera.GetFocalLengthAttr().Get()), float(focal_length), abs_tol=1e-6):
                raise ValueError("industrial camera focal length readback differs")
            records.append({"profile_camera_id": camera_id, "prim_path": path, "eye_xyz_m": eye, "target_xyz_m": target,
                            "focal_length_mm": float(camera.GetFocalLengthAttr().Get()),
                            "local_transform_row_major": [float(actual_transform[row][column]) for row in range(4) for column in range(4)],
                            "horizontal_aperture_mm": float(camera.GetHorizontalApertureAttr().Get()),
                            "vertical_aperture_mm": float(camera.GetVerticalApertureAttr().Get()),
                            "readback_status": "verified",
                            "coordinate_frame": "world" if path.startswith("/World/Factory") else "robot chassis_link"})
        return {"source": str(profile_path.name), "source_sha256": g2_scene.base._sha256(profile_path), "cameras": records}

    def register_rgb_render_products(self, products_by_role: Mapping[str, Any]) -> dict[str, Any]:
        if self.rgb_render_state != "unregistered":
            raise RuntimeError("G3 RGB render products were already registered")
        roles = ("portfolio_overview", "carter1_follow", "carter2_follow")
        if set(products_by_role) != set(roles):
            raise ValueError("G3 requires exactly the three named RGB render product roles")
        products = tuple(products_by_role[role] for role in roles)
        if len({id(product) for product in products}) != 3:
            raise ValueError("G3 requires three unique RGB render products")
        self.rgb_render_products = (products[0], products[1], products[2])
        self.rgb_render_state = "enabled"
        return {
            "roles": list(roles),
            "requested_state_between_captures": "enabled",
            "lifecycle": "persistent render products; no per-capture Hydra update toggles",
        }

    @staticmethod
    def _read_lidar_output_buffer_setting() -> dict[str, Any]:
        try:
            import carb.settings

            value = carb.settings.get_settings().get("/app/sensors/nv/lidar/outputBufferOnGPU")
            return {"status": "observed", "value": value if isinstance(value, bool) else None}
        except BaseException as error:
            return {"status": "unavailable", "error": f"{type(error).__name__}: {error}"[:240]}

    def register_lidar_sensors(
        self, sensors: Sequence[Any], metadata: Sequence[Mapping[str, Any]], timeline: Any,
    ) -> dict[str, Any]:
        expected = {(robot, side) for robot in self.expected_robot_ids for side in ("front", "rear")}
        if len(sensors) != len(expected) or len(metadata) != len(expected):
            raise ValueError("lidar diagnostics require two sensors per declared robot")
        identities = {(item.get("robot_id"), item.get("name"), item.get("prim_path")) for item in metadata}
        if ({(item[0], item[1]) for item in identities} != expected
                or len({item[2] for item in identities}) != len(expected)
                or any(not isinstance(item[2], str) or not item[2] for item in identities)):
            raise ValueError("G3 lidar diagnostic identities differ")
        if self.lidar_diagnostics:
            raise RuntimeError("G3 lidar diagnostics were already registered")
        self.timeline = timeline
        self.lidar_output_buffer_setting = self._read_lidar_output_buffer_setting()
        registrations = []
        for sensor, source in zip(sensors, metadata, strict=True):
            status, error = "attached", None
            try:
                sensor.attach_annotators("generic-model-output")
            except BaseException as failure:
                status = "attach_error"
                error = f"{type(failure).__name__}: {failure}"[:240]
            record = {
                "robot_id": str(source["robot_id"]), "side": str(source["name"]),
                "prim_path": str(source["prim_path"]), "status": status,
            }
            if error is not None:
                record["error"] = error
            registrations.append(record)
            self.lidar_diagnostics.append((sensor, record, status, error))
        return {
            "annotator": "generic-model-output", "sensor_count": len(expected),
            "registrations": registrations,
            "output_buffer_on_gpu": self.lidar_output_buffer_setting,
        }

    def _record_lidar_acquisition(self, phase: str, sim_time_s: float, physics_frame: int) -> None:
        if not self.lidar_diagnostics:
            return
        assert self.output is not None
        path = self.output / "lidar-acquisition-events.jsonl"
        if not self.lidar_journal_opened:
            if path.exists() or path.is_symlink():
                raise ValueError("G3 lidar acquisition journal must be new")
            self.lidar_journal_opened = True
        timeline_playing = bool(self.timeline is not None and self.timeline.is_playing())
        for sensor, identity, registration_status, registration_error in self.lidar_diagnostics:
            row: dict[str, Any] = {
                "schema_version": 1, "sequence": self.lidar_event_sequence,
                "run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                "robot_id": identity["robot_id"], "side": identity["side"],
                "prim_path": identity["prim_path"], "sim_time_s": float(sim_time_s),
                "physics_frame": int(physics_frame), "timeline_playing": timeline_playing,
                "phase": phase, "annotator": "generic-model-output",
                "registration_status": registration_status,
                "output_buffer_on_gpu": self.lidar_output_buffer_setting,
            }
            if registration_error is not None:
                row.update(status="error", error=registration_error)
            else:
                try:
                    row["has_data"] = bool(sensor.has_data())
                    data, info = sensor.get_data("generic-model-output")
                    row["info_keys"] = sorted(str(key) for key in info) if isinstance(info, Mapping) else []
                    if data is None:
                        row["status"] = "warmup_none"
                    else:
                        row.update(status="gmo_header", gmo=_parse_gmo_header(data))
                except BaseException as error:
                    row.update(status="error", error=f"{type(error).__name__}: {error}"[:240])
            g2_scene._append_jsonl(path, row)
            self.lidar_event_sequence += 1

    @staticmethod
    def capture_timeline_policy() -> str:
        return "pause_resume"

    def before_capture(self, rep: Any) -> None:
        if self.rgb_render_state != "enabled" or self.timeline is None:
            raise RuntimeError("G3 RGB render products are not active before capture")
        # A manual step leaves Replicator STEPPED. The outer pause/resume pair
        # supplies the PLAY event that rearms continuous ROS sensor writers.
        # Kit applies a queued pause during update; is_playing() may still be
        # true here. The shared capture guard checks time/frame/pose instead.
        rep.orchestrator.step(delta_time=0.0, rt_subframes=4, pause_timeline=True)

    def after_capture(self) -> None:
        if self.rgb_render_state != "enabled" or self.timeline is None:
            raise RuntimeError("G3 RGB render products are not active after capture")

    def cleanup_render_products(self) -> None:
        if self.rgb_render_products is None:
            return
        self.rgb_render_state = "closed"

    def capture_breadcrumb(self, event: Mapping[str, Any]) -> None:
        assert self.output is not None
        required = {"capture_index", "physics_frame", "sim_time_s", "phase"}
        if set(event) != required:
            raise ValueError("G3 render breadcrumb fields differ")
        if (isinstance(event["capture_index"], bool) or not isinstance(event["capture_index"], int)
                or event["capture_index"] < 0
                or isinstance(event["physics_frame"], bool) or not isinstance(event["physics_frame"], int)
                or event["physics_frame"] < 0
                or isinstance(event["sim_time_s"], bool)
                or not isinstance(event["sim_time_s"], (int, float))
                or not math.isfinite(float(event["sim_time_s"])) or float(event["sim_time_s"]) < 0):
            raise ValueError("G3 render breadcrumb coordinates are invalid")
        phases = {
            "zero_delta_flush_start", "zero_delta_flush_complete",
            "rgb_readback_start", "rgb_readback_complete",
            "capture_finalize_start", "capture_finalize_complete",
            "frozen_state_validation_start", "frozen_state_validation_complete",
        }
        if event["phase"] not in phases:
            raise ValueError("G3 render breadcrumb phase is invalid")
        if event["phase"] == "zero_delta_flush_start":
            self._record_lidar_acquisition(
                "capture_pre", float(event["sim_time_s"]), int(event["physics_frame"]),
            )
        elif event["phase"] == "capture_finalize_complete":
            self._record_lidar_acquisition(
                "capture_post", float(event["sim_time_s"]), int(event["physics_frame"]),
            )
        path = self.output / "render-capture-breadcrumbs.jsonl"
        if not self.render_breadcrumb_opened:
            if path.exists() or path.is_symlink():
                raise ValueError("G3 render breadcrumb journal must be new")
            self.render_breadcrumb_opened = True
        record = {
            "schema_version": 1,
            "sequence": self.render_breadcrumb_sequence,
            "run_id": self.run_id,
            "scenario_id": self.session["scenario_id"],
            **event,
        }
        g2_scene._append_jsonl(path, record)
        self.render_breadcrumb_sequence += 1

    def capture_metadata(self) -> dict[str, Any]:
        assert self.visuals is not None
        metadata = dict(self.visuals.capture_metadata())
        self.last_captured_visual_sequence = self.visuals.applied_sequence
        if self.pending_media_sample is not None:
            metadata["real_time_media"] = self.pending_media_sample
            self.pending_media_sample = None
        return metadata

    def capture_due(self) -> bool:
        assert self.visuals is not None
        visual_due = self.visuals.applied_sequence != self.last_captured_visual_sequence
        window = self.media_window
        if (window is None or self.current_media_frame is None or self.current_media_time_ns is None
                or self.current_media_time_ns >= window["end_sim_time_ns"]):
            return visual_due
        offset = self.current_media_frame - window["trigger_frame"]
        media_due = (offset >= 0 and offset % window["sample_every_physics_steps"] == 0
                     and self.last_media_due_frame != self.current_media_frame)
        if media_due:
            self.last_media_due_frame = self.current_media_frame
            self.pending_media_sample = {
                "window_id": window["window_id"],
                "source_profile_sha256": window["source_profile_sha256"],
                "trigger_robot_id": window["trigger_robot_id"],
                "trigger_command": window["trigger_command"],
                "physics_frame": self.current_media_frame,
                "sim_time_ns": self.current_media_time_ns,
                "window_start_sim_time_ns": window["start_sim_time_ns"],
                "window_end_sim_time_ns": window["end_sim_time_ns"],
                "sample_index": offset // window["sample_every_physics_steps"],
                "sample_every_physics_steps": window["sample_every_physics_steps"],
            }
        return visual_due or media_due

    def step(self, sim_time_s: float, physics_frame: int) -> None:
        assert self.output is not None
        if self.visuals is not None:
            self.visuals.step(sim_time_s, physics_frame)
        sim_time_ns = round(sim_time_s * 1_000_000_000)
        if (self.lidar_diagnostics and (self.last_periodic_lidar_time_ns is None
                or sim_time_ns - self.last_periodic_lidar_time_ns >= 1_000_000_000)):
            self._record_lidar_acquisition("periodic_1hz", sim_time_s, physics_frame)
            self.last_periodic_lidar_time_ns = sim_time_ns
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
        self._observe_real_time_media(robot_id, sample)
        authorization = sample["authorization"]
        sequence = authorization["sequence"]
        if sequence is None or self.control_sequences.get(robot_id) == sequence:
            return
        acknowledgment = {"run_id": self.run_id, "scenario_id": self.session["scenario_id"],
                          "robot_id": robot_id, "authorization_seq": sequence,
                          "mode": authorization["mode"], "valid": authorization["valid"],
                          "evaluation_sim_time_s": authorization["evaluation_sim_time_s"],
                          "sample_sim_time_s": sample["sim_time_s"],
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
                         result_file="g3-scene.json", scene_extension=BarrierLifecycle(session, args.run_id, root),
                         extra_inputs={"g3_scene_source": Path(__file__).resolve(), "session": root / "session.json",
                                       "factory_visuals_source": Path(__file__).with_name("factory_visuals.py"),
                                       "visual_scene_source": root.parents[1] / "factory_ops/visual_scene.py",
                                       "visual_state_source": root.parents[1] / "factory_ops/visual_state.py",
                                       "render_profile": config_dir_from_session(root) / "render-profile.json",
                                       "visual_assets": config_dir_from_session(root) / "visual-assets.json"})
    print(f"FACTORY_OPS_G3_SCENE_STATUS={'PASS' if code == 0 else 'FAIL'}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
