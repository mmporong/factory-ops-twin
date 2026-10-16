from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from uuid import UUID, uuid4
import zlib

from factory_ops.footprint import qualify_footprint
from factory_ops.layout import Layout
from factory_ops.navigation import GoalPose, LocalizationPose, NavigationResult, SingleOrderMission
from scripts.ros.g1_evidence import REQUIRED_TOPICS, _result_path, validate_g1_evidence


PROJECT = Path(__file__).resolve().parents[1]
LAYOUT_PATH = PROJECT / "configs/factory-v3/layout.json"
RUN_ID = "a" * 32


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _png(width: int = 1280, height: int = 720) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(data, zlib.crc32(kind)) & 0xFFFFFFFF)
    rows = b"".join(b"\0" + b"\x80\x80\x80" * width for _ in range(height))
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b"")


class G1EvidenceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.output = Path(self.temporary.name)
        self.scene, self.mission_result, self.actions, self.bag = self._fixture()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_json(self, relative: str, value: object) -> None:
        path = self.output / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    def _result(self, mission: SingleOrderMission, sim_time: float) -> tuple[dict, dict, dict, dict]:
        intent = mission.active_intent
        assert intent is not None
        sent = {"run_id": RUN_ID, "kind": "goal_sent", "sim_time": sim_time - 0.2, "intent": intent.to_dict()}
        accepted = {"run_id": RUN_ID, "kind": "goal_accepted", "sim_time": sim_time - 0.1,
                    "goal_id": intent.goal_id, "server_goal_uuid": UUID(intent.goal_id).hex}
        result = NavigationResult(
            str(uuid4()), intent.goal_id, intent.task_id, intent.station, intent.attempt, 4, sim_time,
            LocalizationPose("map", intent.goal.x, intent.goal.y, intent.goal.yaw, sim_time),
        )
        action = {"run_id": RUN_ID, "kind": "action_result", "sim_time": sim_time,
                  "result": result.to_dict(), "nav2_error_code": 0, "nav2_error_msg": ""}
        receipt = mission.submit_result(result)
        handoff = {"run_id": RUN_ID, "kind": "handoff_commit", "sim_time": sim_time,
                   "receipt": receipt.to_dict(), "snapshot": mission.ledger.snapshot()}
        return sent, accepted, action, handoff

    def _fixture(self) -> tuple[dict, dict, list[dict], dict]:
        layout = Layout.from_file(LAYOUT_PATH)
        goals = {name: GoalPose("map", *layout.stations[name], 0.0) for name in ("S1", "CI", "CO", "OUT")}
        mission = SingleOrderMission(RUN_ID, order_id="G1-ORDER-001", payload_id="G1-TOTE-001", station_goals=goals)
        mission.start()
        actions: list[dict] = []
        for time_value in (1.0, 2.0):
            actions.extend(self._result(mission, time_value))
        mission.advance(22.0)
        for time_value in (23.0, 24.0):
            actions.extend(self._result(mission, time_value))
        self._write_json("mission-result.json", {
            "run_id": RUN_ID, "status": "passed", "mission": mission.export(),
            "sensor_messages": {"scan": 10, "odom": 10}, "ground_truth_input": False,
        })
        (self.output / "actions.jsonl").write_text("".join(json.dumps(item) + "\n" for item in actions), encoding="utf-8")

        trajectory: list[dict] = []
        for index in range(24 * 60 + 1):
            stamp = index / 60
            station = "S1" if index < 120 else "CI" if index < 1380 else "CO" if index < 1440 else "OUT"
            x, y = layout.stations[station]
            trajectory.append({
                "physics_frame": index, "sim_time_s": stamp,
                "wheel_angles_rad": [0.0, 0.0],
                "joint_names": ["caster", "joint_wheel_left", "joint_wheel_right"],
                "joint_positions_rad": [0.0, 0.0, 0.0],
                "ground_truth": {"position_xyz_m": [x, y, 0.1], "orientation_wxyz": [1, 0, 0, 0],
                                 "use": "evaluation and media only"},
                "odometry": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0,
                             "linear_m_s": 0.0, "angular_rad_s": 0.0},
            })
        (self.output / "trajectory.jsonl").write_text("".join(json.dumps(item) + "\n" for item in trajectory), encoding="utf-8")
        media_sample_indices = (0, 60, 120, 1320, 1380, 1440)
        (self.output / "localization.jsonl").write_text("".join(json.dumps({
            "frame_id": "map", "x": trajectory[index]["ground_truth"]["position_xyz_m"][0],
            "y": trajectory[index]["ground_truth"]["position_xyz_m"][1], "yaw": 0.0,
            "stamp": trajectory[index]["sim_time_s"],
        }) + "\n" for index in media_sample_indices), encoding="utf-8")
        self._write_json("sim-ready.json", {"drive_geometry": {
            "wheel_radius_m": 0.04295, "wheel_separation_m": 0.4132,
            "provenance": {"kind": "test fixture"},
        }})
        (self.output / "factory-overview.png").write_bytes(_png())
        (self.output / "diagnostic-load").mkdir()
        (self.output / "diagnostic-load/factory-overview.png").write_bytes(_png())
        media: list[dict] = []
        for index, trajectory_index in enumerate(media_sample_indices):
            sample = trajectory[trajectory_index]
            path = self.output / f"media/frame-{index:05d}.png"
            overview = self.output / f"media/overview-{index:05d}.png"
            path.parent.mkdir(exist_ok=True)
            path.write_bytes(_png())
            overview.write_bytes(_png())
            media.append({
                "index": index, "path": path.relative_to(self.output).as_posix(), "sha256": _sha(path), "bytes": path.stat().st_size,
                "width": 1280, "height": 720, "overview_path": overview.relative_to(self.output).as_posix(),
                "overview_sha256": _sha(overview), "overview_bytes": overview.stat().st_size,
                "overview_width": 1280, "overview_height": 720, "physics_frame": sample["physics_frame"],
                "sim_time_s": sample["sim_time_s"], "ground_truth": sample["ground_truth"],
                "source": "actual persistent Isaac Sim RGB render product; no interpolation",
            })
        self._write_json("media/frames.json", {"schema_version": 1, "frames": media})
        bounds = [[-0.1, -0.1, 0.0], [0.1, 0.1, 0.5]]
        footprint = qualify_footprint(layout, bounds, 0.2)
        scene: dict = {
            "run_id": RUN_ID, "qualification": "g1_single_order", "status": "validated",
            "cleanup": "worker_exit_requested", "mission_control": {"run_id": RUN_ID, "status": "passed"},
            "inputs": {"source": {"sha256": _sha(PROJECT / "scripts/isaac/nav_scene.py")}},
            "robot": {"footprint": footprint, "collision_bounds_xyz": bounds},
            "trajectory": {"sample_count": len(trajectory)},
            "media": {"frame_count": len(media), "initial_and_final": True},
            "timing": {"physics_dt_s": 1 / 60},
            "ros": {"scan": 10, "odom": 10, "tf": 10, "tf_static": 1, "cmd_vel": 10,
                    "clock": {"count": 10, "advanced": True}, "scan_samples": [{"finite_count": 100}]},
        }
        for key, relative in (("load_preview", "diagnostic-load/factory-overview.png"),
                              ("factory_overview", "factory-overview.png")):
            image = self.output / relative
            scene[key] = {"path": str(image), "sha256": _sha(image), "bytes": image.stat().st_size,
                          "width": 1280, "height": 720,
                          "source": "persistent Replicator RGB annotator; no interpolation"}
        owned = [self.output / "sim-ready.json", self.output / "trajectory.jsonl", self.output / "factory-overview.png",
                 *sorted((self.output / "diagnostic-load").rglob("*")), *sorted((self.output / "media").rglob("*"))]
        scene["artifact_manifest"] = [{"path": p.relative_to(self.output).as_posix(), "bytes": p.stat().st_size, "sha256": _sha(p)}
                                      for p in owned if p.is_file()]
        self._write_json("g1-scene.json", scene)
        mission_result = json.loads((self.output / "mission-result.json").read_text())
        self._write_json("pair-exit.json", {"run_id": RUN_ID, "nav_exit_code": 0, "sim_exit_code": 0,
                                             "mission-result.json": mission_result, "g1-scene.json": scene})
        bag = {
            "integrity_check": "ok", "topic_counts": {topic: 1 for topic in REQUIRED_TOPICS},
            "scan_positive_finite_points": 100,
            "goal_statuses": {UUID(item["intent"]["goal_id"]).hex: 4 for item in actions if item["kind"] == "goal_sent"},
            "mission_events": actions,
        }
        return scene, mission_result, actions, bag

    def test_valid_pair_passes_and_report_is_serializable(self) -> None:
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=self.bag)
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["evaluation"]["final_ground_truth_out_distance_m"], 0.0)
        json.dumps(report, allow_nan=False)
        paths = {item["path"] for item in report["artifact_manifest"]}
        self.assertNotIn("g1-validation.json", paths)

    def test_json_action_log_without_matching_bag_events_fails(self) -> None:
        bag = dict(self.bag, mission_events=[])
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=bag)
        self.assertFalse(report["passed"])
        self.assertTrue(any("bag mission events" in error for error in report["errors"]))

    def test_wrong_action_uuid_and_nonzero_error_code_fail(self) -> None:
        actions = list(self.actions)
        changed = dict(actions[2], nav2_error_code=7)
        actions[2] = changed
        (self.output / "actions.jsonl").write_text("".join(json.dumps(item) + "\n" for item in actions), encoding="utf-8")
        bag = dict(self.bag, mission_events=actions)
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=bag)
        self.assertTrue(any("SUCCEEDED/error_code 0" in error for error in report["errors"]))

    def test_ledger_snapshot_tamper_is_caught_by_independent_replay(self) -> None:
        mission = json.loads((self.output / "mission-result.json").read_text())
        mission["mission"]["snapshot"]["ledger_seq"] += 1
        self._write_json("mission-result.json", mission)
        pair = json.loads((self.output / "pair-exit.json").read_text())
        pair["mission-result.json"] = mission
        self._write_json("pair-exit.json", pair)
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=self.bag)
        self.assertTrue(any("independent ledger replay" in error for error in report["errors"]))

    def test_invalid_png_is_rejected_even_when_manifest_is_rehashed(self) -> None:
        path = self.output / "media/frame-00000.png"
        content = bytearray(path.read_bytes())
        content[-5] ^= 1
        path.write_bytes(content)
        frames = json.loads((self.output / "media/frames.json").read_text())
        frames["frames"][0]["sha256"] = _sha(path)
        frames["frames"][0]["bytes"] = path.stat().st_size
        self._write_json("media/frames.json", frames)
        scene = json.loads((self.output / "g1-scene.json").read_text())
        for entry in scene["artifact_manifest"]:
            artifact = self.output / entry["path"]
            entry["sha256"], entry["bytes"] = _sha(artifact), artifact.stat().st_size
        self._write_json("g1-scene.json", scene)
        pair = json.loads((self.output / "pair-exit.json").read_text())
        pair["g1-scene.json"] = scene
        self._write_json("pair-exit.json", pair)
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=self.bag)
        self.assertTrue(any("media PNG is invalid" in error for error in report["errors"]))

    def test_pair_exit_and_required_bag_topic_fail_closed(self) -> None:
        pair = json.loads((self.output / "pair-exit.json").read_text())
        pair["sim_exit_code"] = 1
        self._write_json("pair-exit.json", pair)
        counts = dict(self.bag["topic_counts"])
        counts["/scan"] = 0
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID,
                                      bag_observation=dict(self.bag, topic_counts=counts))
        self.assertTrue(any("worker pair" in error for error in report["errors"]))
        self.assertTrue(any("/scan" in error for error in report["errors"]))

    def test_encoder_joint_and_odometry_tampering_is_rejected(self) -> None:
        samples = [json.loads(line) for line in (self.output / "trajectory.jsonl").read_text().splitlines()]
        samples[1]["joint_positions_rad"][1] = 0.2
        samples[2]["odometry"]["x_m"] = 0.1
        (self.output / "trajectory.jsonl").write_text("".join(json.dumps(item) + "\n" for item in samples), encoding="utf-8")
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=self.bag)
        self.assertTrue(any("wheel angles do not match" in error for error in report["errors"]))
        self.assertTrue(any("odometry does not recompute" in error for error in report["errors"]))

    def test_decimated_trajectory_and_malformed_joint_name_are_rejected(self) -> None:
        samples = [json.loads(line) for line in (self.output / "trajectory.jsonl").read_text().splitlines()]
        del samples[700]
        samples[10]["joint_names"][0] = {"not": "hashable"}
        (self.output / "trajectory.jsonl").write_text("".join(json.dumps(item) + "\n" for item in samples), encoding="utf-8")
        report = validate_g1_evidence(self.output, LAYOUT_PATH, RUN_ID, bag_observation=self.bag)
        self.assertTrue(any("every adjacent physics frame" in error for error in report["errors"]))
        self.assertTrue(any("invalid joint/encoder fields" in error for error in report["errors"]))

    def test_action_pose_must_be_fresh_in_map_and_within_goal_tolerance(self) -> None:
        actions = list(self.actions)
        changed = json.loads(json.dumps(actions[2]))
        changed["result"]["pose"]["x"] += 0.251
        changed["result"]["pose"]["stamp"] = changed["result"]["received_sim_time"] - 0.501
        actions[2] = changed
        (self.output / "actions.jsonl").write_text("".join(json.dumps(item) + "\n" for item in actions), encoding="utf-8")
        report = validate_g1_evidence(
            self.output, LAYOUT_PATH, RUN_ID, bag_observation=dict(self.bag, mission_events=actions)
        )
        self.assertTrue(any("future or stale" in error for error in report["errors"]))
        self.assertTrue(any("outside goal tolerance" in error for error in report["errors"]))

    def test_result_path_rejects_parent_absolute_and_symlink_escape(self) -> None:
        outside = self.output.parent / "escape.json"
        for value in ("../escape.json", str(outside)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                _result_path(self.output.resolve(), value)
        link = self.output / "outside-link"
        try:
            link.symlink_to(self.output.parent, target_is_directory=True)
        except OSError:
            return
        with self.assertRaises(ValueError):
            _result_path(self.output.resolve(), "outside-link/escape.json")


if __name__ == "__main__":
    unittest.main()
