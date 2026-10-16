from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import tempfile
import unittest
from typing import Any
import zlib

from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout
from scripts.isaac.robot_evidence import validate_robot_evidence
from tests.test_gif_evidence import make_gif


ROOT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = ROOT / "configs" / "factory-v3" / "layout.json"
PIXELS = 640 * 360


def _chunk(kind: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(data, zlib.crc32(kind)) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", crc)


def _png(width: int, height: int) -> bytes:
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    scanlines = (b"\x00" + b"\x00" * (width * 3)) * height
    return b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header) + _chunk(b"IDAT", zlib.compress(scanlines)) + _chunk(b"IEND", b"")


def _npy(descr: str, payload: bytes) -> bytes:
    header = repr({"descr": descr, "fortran_order": False, "shape": (360, 640)}).encode("latin1")
    padding = (16 - ((10 + len(header) + 1) % 16)) % 16
    header += b" " * padding + b"\n"
    return b"\x93NUMPY\x01\x00" + struct.pack("<H", len(header)) + header + payload


class RobotEvidenceTests(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.output = Path(temporary.name) / "output"
        self.output.mkdir()
        self.layout = Layout.from_file(LAYOUT_PATH)
        self.layout_sha = hashlib.sha256(LAYOUT_PATH.read_bytes()).hexdigest()
        self.result: dict[str, Any] = self._write_valid_evidence()

    def test_complete_robot_motion_sensor_and_media_evidence_passes(self) -> None:
        self.assertEqual(
            validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha),
            [],
        )

    def test_manifest_detects_content_tampering_and_missing_artifact(self) -> None:
        (self.output / "media/amr-motion.gif").write_bytes(b"GIF89a changed;")
        (self.output / "sensor/frame-000-camera.json").unlink()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("manifest SHA mismatch: media/amr-motion.gif" in error for error in errors))
        self.assertTrue(any("missing, non-regular, or a symlink: sensor/frame-000-camera.json" in error for error in errors))

    def test_unsafe_manifest_path_is_rejected(self) -> None:
        self.result["artifact_manifest"][0]["path"] = "../trajectory.json"

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("path is unsafe or reserved" in error for error in errors))
        self.assertTrue(any("does not exactly match output files" in error for error in errors))

    def test_gif_timing_protocol_is_enforced_after_full_decode(self) -> None:
        (self.output / "media/amr-motion.gif").write_bytes(make_gif(delay_ms=50))
        self._refresh_manifest()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertIn("media/amr-motion.gif frame delays are not all 100 ms", errors)

    def test_out_of_map_trajectory_pose_and_false_displacement_are_rejected(self) -> None:
        trajectory_path = self.output / "trajectory.json"
        trajectory = json.loads(trajectory_path.read_text(encoding="utf-8"))
        trajectory["samples"][50]["position"] = [-1.0, 4.0, 0.0]
        trajectory_path.write_text(json.dumps(trajectory), encoding="utf-8")
        self.result["motion"]["displacement_m"] = 99.0
        self._refresh_manifest()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("sample 50 footprint touches" in error for error in errors))
        self.assertIn("motion displacement_m does not match trajectory endpoints", errors)

    def test_invalid_depth_and_bbox_range_are_rejected(self) -> None:
        depth_path = self.output / "sensor/frame-000-depth.npy"
        depth_path.write_bytes(_npy("<f4", struct.pack("<f", float("nan")) * PIXELS))
        boxes_path = self.output / "sensor/frame-000-boxes.json"
        boxes = json.loads(boxes_path.read_text(encoding="utf-8"))
        boxes["boxes"][0]["x_max"] = 641
        boxes_path.write_text(json.dumps(boxes), encoding="utf-8")
        self._refresh_manifest()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("depth contains NaN" in error for error in errors))
        self.assertTrue(any("depth has less than 5 percent" in error for error in errors))
        self.assertTrue(any("box 0 is outside image bounds" in error for error in errors))

    def test_media_frame_must_match_trajectory_time_and_pose(self) -> None:
        frames_path = self.output / "media/frames.json"
        frames = json.loads(frames_path.read_text(encoding="utf-8"))
        frames["frames"][5]["sim_time"] += 1.0
        frames["frames"][6]["pose_xyz"][0] += 1.0
        frames_path.write_text(json.dumps(frames), encoding="utf-8")
        self._refresh_manifest()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("media frame 5 does not match the 6-step trajectory cadence" in error for error in errors))
        self.assertTrue(any("media frame 6 does not match the 6-step trajectory cadence" in error for error in errors))

    def test_motion_protocol_performance_and_clock_are_recomputed(self) -> None:
        trajectory_path = self.output / "trajectory.json"
        trajectory = json.loads(trajectory_path.read_text(encoding="utf-8"))
        trajectory["samples"][10]["wheel_target_rad_s"] = [0.0, 0.0]
        trajectory["samples"][11]["sim_time"] += 0.01
        trajectory_path.write_text(json.dumps(trajectory), encoding="utf-8")
        self.result["performance"]["measurement_loop_iterations"] = 599
        self.result["performance"]["sim_seconds"] = 9.0
        self.result["ros2"]["clock_samples"] = [0.0, 5.0, 4.0]
        self.result["ros2"]["clock_messages"] = 4
        self._refresh_manifest()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("480-drive/120-stop protocol" in error for error in errors))
        self.assertTrue(any("timestep is not 1/60 s" in error for error in errors))
        self.assertIn("performance measurement_loop_iterations is not 600", errors)
        self.assertIn("performance sim_seconds does not match trajectory span", errors)
        self.assertIn("ros2 clock_samples length does not match clock_messages", errors)
        self.assertIn("ros2 clock_samples is decreasing", errors)

    def test_robot_asset_dofs_and_collider_union_are_enforced(self) -> None:
        robot = self.result["robot"]
        robot["asset_url"] = "omniverse://test/wrong.usd"
        robot["wheel_dof_names"] = ["left", "right"]
        robot["dof_names"] = ["caster_joint"]
        robot["colliders"][1]["path"] = robot["colliders"][0]["path"]
        robot["colliders"][1]["max"][0] = 0.2

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertIn("robot asset_url is not the pinned Nova Carter asset", errors)
        self.assertIn("robot wheel_dof_names does not match the Nova Carter drive wheels", errors)
        self.assertIn("robot dof_names does not contain both configured wheel DOFs", errors)
        self.assertTrue(any("path is invalid or duplicated" in error for error in errors))
        self.assertIn("collision_bounds_xyz does not equal the union of enabled colliders", errors)

    def test_sensor_join_camera_calibration_and_semantic_classes_are_enforced(self) -> None:
        sensor_document_path = self.output / "sensor-frames.json"
        sensor_document = json.loads(sensor_document_path.read_text(encoding="utf-8"))
        sensor_document["frames"][0]["physics_frame"] += 1
        sensor_document_path.write_text(json.dumps(sensor_document), encoding="utf-8")
        camera_path = self.output / sensor_document["frames"][0]["paths"]["camera"]
        camera = json.loads(camera_path.read_text(encoding="utf-8"))
        camera["camera_intrinsics"][0] = 1.0
        camera["camera_to_world"][3] = 2.0
        camera["pose_xyz"][0] += 2.0
        camera_path.write_text(json.dumps(camera), encoding="utf-8")
        boxes_path = self.output / sensor_document["frames"][0]["paths"]["boxes"]
        boxes = json.loads(boxes_path.read_text(encoding="utf-8"))
        boxes["id_to_labels"]["2"]["class"] = "background"
        boxes_path.write_text(json.dumps(boxes), encoding="utf-8")
        self._refresh_manifest()

        errors = validate_robot_evidence(self.output, self.result, LAYOUT_PATH, self.layout_sha)

        self.assertTrue(any("does not match media index 0" in error for error in errors))
        self.assertTrue(any("camera_intrinsics is invalid" in error for error in errors))
        self.assertTrue(any("camera_to_world is invalid" in error for error in errors))
        self.assertTrue(any("more than 1.5 m" in error for error in errors))
        self.assertTrue(any("no non-background equipment mask pixels" in error for error in errors))
        self.assertTrue(any("no equipment bounding box" in error for error in errors))

    def _write_valid_evidence(self) -> dict[str, Any]:
        bounds = [[-0.1, -0.1, 0.0], [0.1, 0.1, 0.5]]
        footprint = qualify_footprint(self.layout, bounds)
        samples = []
        for frame in range(601):
            samples.append(
                {
                    "physics_frame": 120 + frame,
                    "sim_time": frame / 60,
                    "position": [4.0 + frame * 0.0005, 4.0, 0.0],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "wheel_target_rad_s": [2.0, 2.0] if 1 <= frame <= 480 else [0.0, 0.0],
                }
            )
        (self.output / "trajectory.json").write_text(json.dumps({"samples": samples}), encoding="utf-8")

        sensor_dir = self.output / "sensor"
        sensor_dir.mkdir()
        sensor_frames = []
        sensor_png = _png(640, 360)
        semantic_payload = struct.pack("<I", 1) * (PIXELS // 2) + struct.pack("<I", 2) * (PIXELS - PIXELS // 2)
        for index, sample_index in enumerate((6, 300, 600)):
            stem = f"frame-{index:03d}"
            paths = {
                "rgb": f"sensor/{stem}-rgb.png",
                "depth": f"sensor/{stem}-depth.npy",
                "semantic": f"sensor/{stem}-semantic.npy",
                "boxes": f"sensor/{stem}-boxes.json",
                "camera": f"sensor/{stem}-camera.json",
            }
            (self.output / paths["rgb"]).write_bytes(sensor_png)
            (self.output / paths["depth"]).write_bytes(_npy("<f4", struct.pack("<f", 2.0) * PIXELS))
            (self.output / paths["semantic"]).write_bytes(_npy("<u4", semantic_payload))
            (self.output / paths["boxes"]).write_text(
                json.dumps(
                    {
                        "boxes": [
                            {"semantic_id": 1, "x_min": 0, "y_min": 180, "x_max": 640, "y_max": 360},
                            {"semantic_id": 2, "x_min": 10, "y_min": 20, "x_max": 30, "y_max": 40},
                        ],
                        "id_to_labels": {"1": {"class": "floor"}, "2": {"class": "equipment"}},
                    }
                ),
                encoding="utf-8",
            )
            (self.output / paths["camera"]).write_text(
                json.dumps(
                    {
                        "camera_intrinsics": [480.0, 0.0, 320.0, 0.0, 480.0, 180.0, 0.0, 0.0, 1.0],
                        "camera_to_world": [1.0, 0.0, 0.0, 0.0,
                                            0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0,
                                            samples[sample_index]["position"][0] + 0.35,
                                            samples[sample_index]["position"][1], 0.65, 1.0],
                        "pose_xyz": [samples[sample_index]["position"][0] + 0.35,
                                     samples[sample_index]["position"][1], 0.65],
                        "pose_wxyz": [1.0, 0.0, 0.0, 0.0],
                        "sim_time": samples[sample_index]["sim_time"],
                    }
                ),
                encoding="utf-8",
            )
            sensor_frames.append(
                {
                    "physics_frame": samples[sample_index]["physics_frame"],
                    "sim_time": samples[sample_index]["sim_time"],
                    "pose_xyz": samples[sample_index]["position"],
                    "pose_wxyz": samples[sample_index]["orientation_wxyz"],
                    "paths": paths,
                }
            )
        (self.output / "sensor-frames.json").write_text(
            json.dumps(
                {
                    "width": 640,
                    "height": 360,
                    "optical_frame": "carter/camera_optical",
                    "frames": sensor_frames,
                }
            ),
            encoding="utf-8",
        )

        media_dir = self.output / "media"
        media_dir.mkdir()
        media_png = _png(1280, 720)
        media_frames = []
        for index in range(100):
            sample_index = (index + 1) * 6
            path = f"media/frame-{index:03d}.png"
            (self.output / path).write_bytes(media_png)
            sample = samples[sample_index]
            media_frames.append(
                {
                    "index": index,
                    "physics_frame": sample["physics_frame"],
                    "sim_time": sample["sim_time"],
                    "path": path,
                    "pose_xyz": sample["position"],
                }
            )
        (media_dir / "frames.json").write_text(json.dumps({"fps": 10, "frames": media_frames}), encoding="utf-8")
        (media_dir / "amr-motion.gif").write_bytes(make_gif())

        result: dict[str, Any] = {
            "qualification": "g0b",
            "observed_stage_scope": "factory_before_robot_insertion",
            "layout_path": str(LAYOUT_PATH),
            "layout_sha256": self.layout_sha,
            "robot": {
                "prim_path": "/World/Carter",
                "asset_url": "omniverse://test/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
                "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
                "dof_names": ["joint_wheel_left", "joint_wheel_right", "caster_joint"],
                "enabled_collision_count": 2,
                "colliders": [
                    {"path": "/World/Carter/chassis/collider", "min": [-0.1, -0.1, 0.0], "max": [0.05, 0.1, 0.4]},
                    {"path": "/World/Carter/wheels/collider", "min": [-0.05, -0.08, 0.1], "max": [0.1, 0.08, 0.5]},
                ],
                "collision_bounds_xyz": bounds,
                "footprint": footprint,
            },
            "motion": {
                "kind": "open_loop_wheel_velocity",
                "trajectory_file": "trajectory.json",
                "displacement_m": 0.3,
            },
            "performance": {
                "sim_seconds": 10.0,
                "wall_seconds": 20.0,
                "rtf": 0.5,
                "warmup_steps": 120,
                "measurement_loop_iterations": 600,
                "capture_included": True,
            },
            "ros2": {
                "distro": "jazzy",
                "clock_messages": 3,
                "clock_advanced": True,
                "clock_samples": [0.0, 5.0, 10.0],
            },
            "artifact_manifest": [],
        }
        self.result = result
        self._refresh_manifest()
        return result

    def _refresh_manifest(self) -> None:
        self.result["artifact_manifest"] = [
            {
                "path": path.relative_to(self.output).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for path in sorted(self.output.rglob("*"))
            if path.is_file() and path.name != "stage-smoke.json"
        ]


if __name__ == "__main__":
    unittest.main()
