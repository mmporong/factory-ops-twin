from __future__ import annotations

import base64
from copy import deepcopy
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
from typing import Any

from factory_ops.g3_media import (_trajectory_index, plan_frame_selection, rehash_inputs, sha256_file,
                                  validate_export_inputs, verify_storage_separation)
from factory_ops.visual_state import canonical_sha256
from scripts.media.export_g3_media import _operations_timing, _uniform_indices, export_media


RUN_LABEL = "g3-q-normal-fifo-009"
RUN_ID = "a" * 32
SOURCE_COMMIT = "b" * 40
PROFILE_SHA = "c" * 64
PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAusB9Y9ZQMcAAAAASUVORK5CYII="
)


def save_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def record(path: Path, root: Path) -> dict[str, Any]:
    return {"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def make_frames() -> list[dict]:
    by_frame: dict[int, dict] = {}
    # A bounded initial endpoint exception followed by the stable source 60-step phase.
    for physics_frame in [2, *range(3, 1804, 60)]:
        by_frame[physics_frame] = {"physics_frame": physics_frame}
    # The 24-second source window overlaps some base frames; those frames remain base selections.
    for sample_index in range(240):
        physics_frame = 303 + sample_index * 6
        by_frame.setdefault(physics_frame, {})["tag"] = {
            "window_id": "window-001", "source_profile_sha256": PROFILE_SHA,
            "physics_frame": physics_frame, "sim_time_ns": physics_frame * 1_000_000_000 // 60,
            "sample_index": sample_index, "sample_every_physics_steps": 6,
        }
    frames = []
    for index, physics_frame in enumerate(sorted(by_frame)):
        tag = by_frame[physics_frame].get("tag")
        state = {"run_id": RUN_ID, "scenario_id": "QUALIFICATION_NORMAL",
                 "applied_sequence": None, "applied_state_hash": None,
                 "applied_snapshot": None, "latest_event": None}
        if tag is not None:
            state["real_time_media"] = tag
        frames.append({"index": index, "physics_frame": physics_frame,
                       "sim_time_s": physics_frame / 60,
                       "visual_state": state})
    return frames


class G3MediaPlanningTests(unittest.TestCase):
    def test_playback_speed_uses_original_timestamps_for_legacy_and_sparse_captures(self) -> None:
        for source_period_ns, expected_speed in ((1_000_000_000, 10), (10_000_000_000, 100)):
            with self.subTest(source_period_ns=source_period_ns):
                frames = [{"sim_time_ns": 50_000_000 + index * source_period_ns} for index in range(4)]
                timing = _operations_timing(frames)
                self.assertEqual(expected_speed, timing["nominal_speedup"])
                self.assertEqual(source_period_ns, timing["source_capture_period_ns"])
                self.assertEqual(f"g3-full-operations-{expected_speed}x.mp4", timing["filename"])

    def test_nonuniform_source_timestamps_cannot_be_labelled_as_constant_speed(self) -> None:
        frames = [{"sim_time_ns": value} for value in (0, 10_000_000_000, 20_001_000_000)]
        with self.assertRaisesRegex(ValueError, "not a uniform capture grid"):
            _operations_timing(frames)

    def test_sparse_ten_second_grid_retains_motion_clip_without_fabricated_frames(self) -> None:
        frames = [row for row in make_frames()
                  if row["physics_frame"] == 2 or row["physics_frame"] % 600 == 3
                  or row["visual_state"].get("real_time_media") is not None]
        for index, row in enumerate(frames):
            row["index"] = index
        plan = plan_frame_selection(frames, capture_every_steps=600)
        self.assertEqual(240, len(plan["real_time_frames"]))
        self.assertEqual([3, 603, 1203, 1803],
                         [row["physics_frame"] for row in plan["base_frames"]])

    def test_mixed_index_separates_240_grid_and_capture_first_base_phase(self) -> None:
        frames = make_frames()
        plan = plan_frame_selection(frames)
        self.assertEqual(240, len(plan["real_time_frames"]))
        self.assertEqual(3, plan["base_anchor_physics_frame"])
        self.assertEqual(list(range(3, 1804, 60)),
                         [row["physics_frame"] for row in plan["base_frames"]])
        self.assertEqual([2], plan["endpoint_exception_physics_frames"])
        self.assertTrue(any(row["visual_state"].get("real_time_media") is not None
                            for row in plan["base_frames"]))

    def test_missing_duplicate_offgrid_and_ambiguous_base_are_rejected(self) -> None:
        frames = make_frames()
        missing = deepcopy(frames)
        missing.pop(next(index for index, row in enumerate(missing)
                         if row["visual_state"].get("real_time_media", {}).get("sample_index") == 20))
        for index, row in enumerate(missing):
            row["index"] = index
        with self.assertRaisesRegex(ValueError, "exactly 240"):
            plan_frame_selection(missing)

        stale = deepcopy(frames)
        tagged = next(row for row in stale if row["visual_state"].get("real_time_media"))
        tagged["visual_state"]["real_time_media"]["physics_frame"] += 1
        with self.assertRaisesRegex(ValueError, "off-grid"):
            plan_frame_selection(stale)

        duplicate = deepcopy(frames)
        duplicate[1]["physics_frame"] = duplicate[0]["physics_frame"]
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            plan_frame_selection(duplicate)

    def test_trajectory_parent_context_accepts_absent_row_identity_and_rejects_nested_mismatch(self) -> None:
        row = {"physics_frame": 60, "sim_time_s": 1.0,
               "ground_truth": {"position_xyz_m": [1.0, 2.0, 0.0],
                                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
               "authorization": {"evaluation_sim_time_s": 1.0, "mode": "emergency",
                                 "payload": None, "reason": "authorization_missing",
                                 "sequence": None, "valid": False}}
        self.assertIn((60, 1_000_000_000),
                      _trajectory_index([row], RUN_ID, "QUALIFICATION_NORMAL", "carter1"))

        top_mismatch = {**row, "robot_id": "carter2"}
        with self.assertRaisesRegex(ValueError, "identity or clock"):
            _trajectory_index([top_mismatch], RUN_ID, "QUALIFICATION_NORMAL", "carter1")

        nested = deepcopy(row)
        nested["authorization"]["payload"] = {
            "run_id": RUN_ID, "robot_id": "carter2",
            "scope": {"scenario_id": "QUALIFICATION_NORMAL"},
        }
        with self.assertRaisesRegex(ValueError, "nested authorization identity"):
            _trajectory_index([nested], RUN_ID, "QUALIFICATION_NORMAL", "carter1")

    def test_storage_inventory_proves_distinct_physical_disks_and_rejects_alias_or_swap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inventory = root / "inventory.json"
            save_json(inventory, {"source": "Win32_LogicalDisk -> Win32_DiskPartition -> Win32_DiskDrive CIM associations",
                "drives": [
                    {"drive": "D:", "physical_disks": [{"index": 3, "device_id": r"\\.\PHYSICALDRIVE3",
                                                          "serial_sha256_prefix": "C44860F40B08"}]},
                    {"drive": "E:", "physical_disks": [{"index": 2, "device_id": r"\\.\PHYSICALDRIVE2",
                                                          "serial_sha256_prefix": "5E8530E83D67"}]},
                ]})
            supplement = root / "manifest.json"
            save_json(supplement, {"disk_identity_inventory": "artifacts/brev/inventory.json",
                                   "files": [{"path": "artifacts/brev/inventory.json",
                                              "bytes": inventory.stat().st_size,
                                              "sha256": sha256_file(inventory)}]})
            digest = sha256_file(supplement)
            archives = [{"path": r"D:\archive.tar.gz"}, {"path": r"E:\backup.tar.gz"}]
            result = verify_storage_separation(inventory, supplement, digest, archives)
            self.assertEqual(["D:", "E:"], result["archive_drives"])
            with self.assertRaisesRegex(ValueError, "disjoint physical disks"):
                verify_storage_separation(inventory, supplement, digest,
                                          [{"path": r"D:\a"}, {"path": r"d:\b"}])
            changed = json.loads(inventory.read_text())
            changed["drives"][1]["physical_disks"] = deepcopy(changed["drives"][0]["physical_disks"])
            save_json(inventory, changed)
            with self.assertRaisesRegex(ValueError, "sealed supplement"):
                verify_storage_separation(inventory, supplement, digest, archives)


class G3MediaGateTests(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.parent = Path(temporary.name).resolve()
        self.root = self.parent / "payload"
        self.run_root = self.root / "results" / RUN_LABEL
        self.run_root.mkdir(parents=True)
        self.session = self.root / "source-snapshots" / RUN_LABEL / "runs" / f"{RUN_LABEL}-session" / "session.json"
        self.identity = {"profile": "qualification", "condition": "normal", "policy": "fifo", "seed": 0}
        session_value = {**self.identity, "scenario_id": "QUALIFICATION_NORMAL",
                         "scope": "qualification_and_pilot_only_not_primary_comparison",
                         "primary_evaluation": False}
        save_json(self.session, session_value)
        save_json(self.session.parent / "config.json", {"g3_runtime": {}, "camera_capture_every_steps": 60})

        frames = make_frames()
        one_pixel = self.run_root / "media" / "pixel.png"
        one_pixel.parent.mkdir()
        one_pixel.write_bytes(PNG_1X1)
        png = {"path": "media/pixel.png", "bytes": one_pixel.stat().st_size,
               "sha256": sha256_file(one_pixel), "width": 1, "height": 1}
        ground_truth = {robot: {"position_xyz_m": [float(index), 0.0, 0.0],
                                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}
                        for index, robot in enumerate(("carter1", "carter2"))}
        for frame in frames:
            frame["sources"] = {camera: dict(png) for camera in ("overview", "carter1", "carter2")}
            frame["ground_truth"] = deepcopy(ground_truth)
        self.frames_path = self.run_root / "media" / "frames.json"
        save_json(self.frames_path, {"schema_version": 1, "frames": frames})
        self.journal = self.run_root / "real-time-media-events.jsonl"
        self.journal.write_text('{"event":"fixture"}\n', encoding="utf-8")
        self.actions = self.run_root / "actions.jsonl"
        self.actions.write_text(json.dumps({"run_id": RUN_ID, "event_seq": 1,
                                            "sim_time_s": 0.01, "kind": "mission_started"}) + "\n",
                                encoding="utf-8")
        self.route_events = self.run_root / "route-events.jsonl"
        self.route_events.write_text(json.dumps({"run_id": RUN_ID, "event_sequence": 1,
                                                  "sim_time": 0.01, "event_type": "footprint_observed",
                                                  "robot_id": "carter1"}) + "\n", encoding="utf-8")
        self.trajectory_paths = {}
        for robot in ("carter1", "carter2"):
            path = self.run_root / "robots" / robot / "trajectory.jsonl"
            path.parent.mkdir(parents=True)
            rows = [{"run_id": RUN_ID, "physics_frame": frame["physics_frame"],
                     "sim_time_s": frame["sim_time_s"], "ground_truth": ground_truth[robot],
                     "authorization": {"evaluation_sim_time_s": frame["sim_time_s"],
                                       "mode": "emergency", "payload": None,
                                       "reason": "authorization_missing", "sequence": None,
                                       "valid": False}} for frame in frames]
            path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            self.trajectory_paths[robot] = path

        artifacts = [record(one_pixel, self.run_root), record(self.frames_path, self.run_root),
                     record(self.journal, self.run_root), record(self.actions, self.run_root),
                     record(self.route_events, self.run_root),
                     *(record(path, self.run_root) for path in self.trajectory_paths.values())]
        self.report = self.run_root / "g3-evidence.json"
        self.report_value = {
            "passed": True, "status": "validated", "errors": [], "primary_evaluation": False,
            "scope": "qualification_and_pilot_only_not_primary_comparison", "run_id": RUN_ID,
            "scenario_id": "QUALIFICATION_NORMAL", **self.identity,
            "closed_bag": True, "actual_validator_passed": True, "collection_qa_passed": True,
            "safety_qa_passed": True, "terminal_qa_passed": True, "artifact_manifest": artifacts,
        }
        save_json(self.report, self.report_value)
        self.environment = self.root / "logs" / f"{RUN_LABEL}-environment.json"
        save_json(self.environment, {"schema_version": 1, "run_id": RUN_ID, "run_label": RUN_LABEL,
                  "scenario_id": "QUALIFICATION_NORMAL", **self.identity, "source_commit": SOURCE_COMMIT,
                  "checkout_head": SOURCE_COMMIT, "checkout_clean": True, "source_files": {"README.md": "d" * 64},
                  "source_snapshot_host_relative": f"source-snapshots/{RUN_LABEL}",
                  "session_json_sha256": sha256_file(self.session), "session": session_value})
        self.bundle = self.root / "source.bundle"
        self.bundle.write_bytes(b"test-only-source-bundle-placeholder")
        self.archive = self.parent / "archive.tar.gz"; self.archive.write_bytes(b"a")
        self.backup = self.parent / "backup.tar.gz"; self.backup.write_bytes(b"a")
        self.storage_inventory = self.parent / "storage-inventory.json"; self.storage_inventory.write_text("{}\n")
        self.storage_manifest = self.parent / "storage-manifest.json"; self.storage_manifest.write_text("{}\n")
        self.processor_source = self.parent / "processor.py"; self.processor_source.write_text("VALUE = 1\n")
        self.raw_manifest = self.parent / "raw-manifest.json"
        self.restored_manifest = self.root / "backup-manifest.json"
        self.record_path = self.parent / "backup-record.json"
        self.verification_path = self.parent / "verification.json"
        self.refresh_backup()

    @staticmethod
    def image_probe(path: Path) -> tuple[str, tuple[int, int]]:
        data = path.read_bytes()
        if data[:8] != b"\x89PNG\r\n\x1a\n" or data[12:16] != b"IHDR":
            raise ValueError("fixture is not a PNG")
        return "PNG", struct.unpack(">II", data[16:24])

    def refresh_backup(self) -> None:
        files = [path for path in self.root.rglob("*") if path.is_file() and path != self.restored_manifest]
        entries = [record(path, self.root) for path in sorted(files)]
        manifest = {"schema_version": 1, "created_at_utc": "2026-10-18T00:00:00+00:00",
                    "file_count": len(entries), "files": entries, "known_limitations": [],
                    "scope": "closed diagnostics", "source_bundle": "source.bundle",
                    "source_commit": SOURCE_COMMIT, "total_bytes": sum(int(item["bytes"]) for item in entries)}
        save_json(self.raw_manifest, manifest); save_json(self.restored_manifest, manifest)
        manifest_sha = sha256_file(self.raw_manifest)
        record_value = {"archive": "backup.tar.gz", "archive_bytes": 1, "archive_sha256": "a" * 64,
                        "containers": [], "file_count": len(entries), "gpu_processes": "",
                        "manifest": "backup-manifest.json", "manifest_sha256": manifest_sha,
                        "schema_version": 1, "scope": "closed diagnostics",
                        "sealed_at_utc": "2026-10-18T00:00:00+00:00", "source_bundle_sha256": "b" * 64,
                        "source_commit": SOURCE_COMMIT, "total_bytes": manifest["total_bytes"]}
        save_json(self.record_path, record_value)
        save_json(self.verification_path, {"status": "passed", "scope": "closed diagnostics",
                  "archive_sha256": "a" * 64, "file_count": len(entries), "total_bytes": manifest["total_bytes"],
                  "exact_restore": True, "source_bundle_restore_fsck": True, "runtime_source_pins_verified": 1,
                  "source_commit": SOURCE_COMMIT, "checked_at_utc": "2026-10-18T00:00:00+00:00",
                  "archive": str(self.archive), "backup": str(self.backup),
                  "restore_payload": str(self.root)})

    def source_verifier(self, _backup, _run_label, _source_commit):
        return {"environment": {"path": self.environment.relative_to(self.root).as_posix(),
                                "sha256": sha256_file(self.environment)},
                "bundle": {"path": self.bundle.relative_to(self.root).as_posix(),
                           "sha256": sha256_file(self.bundle), "main_commit": SOURCE_COMMIT},
                "source_archive_sha256": "e" * 64, "source_commit_object_sha256": "f" * 64,
                "verified_source_pin_count": 1,
                "archives": [{"path": str(path), "bytes": path.stat().st_size, "sha256": sha256_file(path)}
                             for path in (self.archive, self.backup)]}

    def storage_verifier(self, *_args):
        return {"inventory": {"path": str(self.storage_inventory), "bytes": self.storage_inventory.stat().st_size,
                              "sha256": sha256_file(self.storage_inventory)},
                "supplement_manifest": {"path": str(self.storage_manifest), "bytes": self.storage_manifest.stat().st_size,
                                        "sha256": sha256_file(self.storage_manifest)},
                "archive_drives": ["D:", "E:"], "physical_disks": [[{"index": 3}], [{"index": 2}]]}

    def validate(self):
        with (patch("factory_ops.g3_media._validate_real_time_media",
                    return_value={"required": True, "passed": True, "sample_count": 240}),
              patch("factory_ops.g3_media._validate_artifact_manifest", return_value=("d" * 64, 1))):
            evidence = validate_export_inputs(
                self.root, RUN_LABEL, self.raw_manifest, self.record_path, self.verification_path,
                source_commit=SOURCE_COMMIT, storage_inventory_path=self.storage_inventory,
                storage_supplement_manifest_path=self.storage_manifest,
                storage_supplement_manifest_sha256="f" * 64, image_probe=self.image_probe,
                source_verifier=self.source_verifier, storage_verifier=self.storage_verifier,
            )
        evidence["postprocessor_source"] = {"schema_version": 1, "mode": "test_fixture",
            "expected_commit": SOURCE_COMMIT, "git_head": None, "git_clean": None,
            "files": [{"path": "processor.py", "absolute_path": str(self.processor_source),
                       "bytes": self.processor_source.stat().st_size, "sha256": sha256_file(self.processor_source)}]}
        return evidence

    def test_closed_validated_pass_cross_binds_mixed_media_and_trajectories(self) -> None:
        evidence = self.validate()
        self.assertEqual(240, len(evidence["selection"]["real_time_frames"]))
        self.assertEqual(31, len(evidence["selection"]["base_frames"]))
        self.assertEqual({"media/pixel.png", "media/frames.json", "actions.jsonl", "route-events.jsonl",
                          "real-time-media-events.jsonl", "robots/carter1/trajectory.jsonl",
                          "robots/carter2/trajectory.jsonl"},
                         set(evidence["inputs"]))
        rehash_inputs(evidence)
        self.backup.write_bytes(b"mutated")
        with self.assertRaisesRegex(ValueError, "archive copy changed"):
            rehash_inputs(evidence)
        self.backup.write_bytes(b"a")
        pixel = self.run_root / "media" / "pixel.png"
        pixel.write_bytes(pixel.read_bytes() + b"tamper")
        with self.assertRaisesRegex(ValueError, "bytes differ"):
            rehash_inputs(evidence)

    def test_failed_restore_or_manifest_and_trajectory_tampering_are_rejected(self) -> None:
        verification = json.loads(self.verification_path.read_text())
        verification["status"] = "running"
        save_json(self.verification_path, verification)
        with self.assertRaisesRegex(ValueError, "not a passed exact restore"):
            self.validate()
        verification["status"] = "passed"
        save_json(self.verification_path, verification)

        original = self.frames_path.read_bytes()
        self.frames_path.write_bytes(original + b" ")
        with self.assertRaisesRegex(ValueError, "bytes differ"):
            self.validate()
        self.frames_path.write_bytes(original)

        trajectory = self.trajectory_paths["carter1"]
        rows = trajectory.read_text().splitlines()
        value = json.loads(rows[10]); value["sim_time_s"] += 0.001; rows[10] = json.dumps(value)
        trajectory.write_text("\n".join(rows) + "\n", encoding="utf-8")
        # Updating both manifests still cannot forge the frame/trajectory exact-time join.
        digest = sha256_file(trajectory); size = trajectory.stat().st_size
        document = json.loads(self.report.read_text())
        for item in document["artifact_manifest"]:
            if item["path"] == "robots/carter1/trajectory.jsonl":
                item.update(bytes=size, sha256=digest)
        save_json(self.report, document)
        self.refresh_backup()
        with self.assertRaisesRegex(ValueError, "exact two-robot"):
            self.validate()

    def test_environment_session_and_config_require_sealed_raw_manifest_entries(self) -> None:
        for path in (self.environment, self.session, self.session.parent / "config.json"):
            original = path.read_bytes()
            path.write_bytes(original + b" ")
            with self.assertRaisesRegex(ValueError, "bytes differ"):
                self.validate()
            path.write_bytes(original)

    def test_report_qa_flag_and_unsafe_camera_path_are_rejected(self) -> None:
        report = json.loads(self.report.read_text())
        report["terminal_qa_passed"] = False
        save_json(self.report, report)
        with self.assertRaisesRegex(ValueError, "terminal validated"):
            self.validate()

        report["terminal_qa_passed"] = True
        save_json(self.report, report)
        frames = json.loads(self.frames_path.read_text())
        frames["frames"][0]["sources"]["overview"]["path"] = "../escape.png"
        save_json(self.frames_path, frames)
        frames_digest = sha256_file(self.frames_path)
        document = json.loads(self.report.read_text())
        for item in document["artifact_manifest"]:
            if item["path"] == "media/frames.json":
                item.update(bytes=self.frames_path.stat().st_size, sha256=frames_digest)
        save_json(self.report, document)
        self.refresh_backup()
        with self.assertRaisesRegex(ValueError, "canonical POSIX"):
            self.validate()

    def test_applied_snapshot_is_bound_to_both_manifests_and_rehashed(self) -> None:
        projection = {"scope": "fixture", "payloads": [],
                      "signals": {"ci_slots_used": 0, "co_slots_used": 0,
                                  "cell_processing": False, "orders_completed": 0},
                      "physical_loading_validated": False, "ground_truth_control_input": False}
        state = {"schema_version": 1, "run_id": RUN_ID, "scenario_id": "QUALIFICATION_NORMAL",
                 "sequence": 0, "emitted_sim_time_ns": 0, "ledger_seq": 1,
                 "event_hash": "e" * 64, "projection_sha256": canonical_sha256(projection),
                 "projection": projection}
        state_hash = canonical_sha256(state)
        snapshot_path = self.run_root / "visual-state-snapshots" / f"00000000-{state_hash}.json"
        save_json(snapshot_path, {"schema_version": 1, "state_hash": state_hash, "visual_state": state})
        reference = record(snapshot_path, self.run_root)
        frames = json.loads(self.frames_path.read_text())
        for frame in frames["frames"]:
            frame["visual_state"].update({
                "applied_sequence": 0, "applied_state_hash": state_hash, "applied_snapshot": reference,
                "latest_event": {"run_id": RUN_ID, "scenario_id": "QUALIFICATION_NORMAL",
                    "source_sequence": 0, "source_state_hash": state_hash,
                    "source_projection_sha256": state["projection_sha256"], "source_ledger_seq": 1,
                    "source_event_hash": "e" * 64, "source_emitted_sim_time_ns": 0,
                    "applied_sim_time_ns": 0, "applied_physics_frame": 0,
                    "source_state_snapshot": reference, "displayed_payloads": [],
                    "signals": projection["signals"]},
            })
        save_json(self.frames_path, frames)
        report = json.loads(self.report.read_text())
        report["artifact_manifest"] = [item for item in report["artifact_manifest"]
                                       if item["path"] != "media/frames.json"]
        report["artifact_manifest"].extend((record(self.frames_path, self.run_root), reference))
        save_json(self.report, report)
        self.refresh_backup()

        evidence = self.validate()
        self.assertEqual("applied", evidence["selection"]["base_frames"][0]["operations"]["visual"]["status"])
        snapshot_path.write_bytes(snapshot_path.read_bytes() + b" ")
        with self.assertRaisesRegex(ValueError, "bytes differ"):
            self.validate()

    def test_destination_under_restore_is_rejected_without_creating_any_file(self) -> None:
        before = {path.relative_to(self.root).as_posix(): sha256_file(path)
                  for path in self.root.rglob("*") if path.is_file()}
        destination = self.root / "portfolio"
        with self.assertRaisesRegex(ValueError, "outside the immutable restore payload"):
            export_media(self.validate(), destination, ffmpeg="unused", ffprobe="unused")
        after = {path.relative_to(self.root).as_posix(): sha256_file(path)
                 for path in self.root.rglob("*") if path.is_file()}
        self.assertEqual(before, after)
        self.assertFalse(destination.exists())
        self.assertEqual([], list(self.root.glob(".g3-media-*")))

    def test_archive_mutation_during_render_blocks_atomic_publish(self) -> None:
        evidence = self.validate()
        destination = self.parent / "portfolio-output"

        def materialize(frames: list[dict[str, Any]], folder: Path) -> None:
            folder.mkdir()
            for index in range(len(frames)):
                (folder / f"frame-{index:05d}.png").write_bytes(PNG_1X1)
            if folder.name == ".actual-frames":
                self.backup.write_bytes(b"mutated-during-render")

        def encode(_ffmpeg: str, _ffprobe: str, _frames: Path, output: Path,
                   expected_count: int, expected_fps: int) -> dict[str, Any]:
            output.write_bytes(b"encoded")
            return {"width": 1280, "height": 720, "fps": expected_fps,
                    "frame_count": expected_count, "duration_s": expected_count / expected_fps}

        def encode_gif(_ffmpeg: str, _ffprobe: str, _frames: Path, output: Path,
                       expected_count: int) -> dict[str, Any]:
            return encode(_ffmpeg, _ffprobe, _frames, output, expected_count, 10)

        try:
            with (patch("scripts.media.export_g3_media._materialize_frames", side_effect=materialize),
                  patch("scripts.media.export_g3_media._encode", side_effect=encode),
                  patch("scripts.media.export_g3_media._gif", side_effect=encode_gif)):
                with self.assertRaisesRegex(ValueError, "archive copy changed during media export"):
                    export_media(evidence, destination, ffmpeg="unused", ffprobe="unused")
        finally:
            self.backup.write_bytes(b"a")

        self.assertFalse(destination.exists())
        self.assertEqual([], list(self.parent.glob(".g3-media-*")))

    def test_dependency_source_mutation_during_render_blocks_atomic_publish(self) -> None:
        evidence = self.validate()
        destination = self.parent / "source-mutated-output"

        def materialize(frames: list[dict[str, Any]], folder: Path) -> None:
            folder.mkdir()
            for index in range(len(frames)):
                (folder / f"frame-{index:05d}.png").write_bytes(PNG_1X1)
            if folder.name == ".actual-frames":
                self.processor_source.write_text("VALUE = 2\n")

        def encode(_ffmpeg: str, _ffprobe: str, _frames: Path, output: Path,
                   expected_count: int, expected_fps: int = 10) -> dict[str, Any]:
            output.write_bytes(b"encoded")
            return {"width": 1280, "height": 720, "fps": expected_fps,
                    "frame_count": expected_count, "duration_s": expected_count / expected_fps}

        try:
            with (patch("scripts.media.export_g3_media._materialize_frames", side_effect=materialize),
                  patch("scripts.media.export_g3_media._encode", side_effect=encode),
                  patch("scripts.media.export_g3_media._gif", side_effect=encode)):
                with self.assertRaisesRegex(ValueError, "postprocessor source changed"):
                    export_media(evidence, destination, ffmpeg="unused", ffprobe="unused")
        finally:
            self.processor_source.write_text("VALUE = 1\n")
        self.assertFalse(destination.exists())
        self.assertEqual([], list(self.parent.glob(".g3-media-*")))


class G3MediaPureHelpersTests(unittest.TestCase):
    def test_uniform_selection_preserves_endpoints_without_inventing_frames(self) -> None:
        selected = _uniform_indices(241, 120)
        self.assertEqual((0, 240), (selected[0], selected[-1]))
        self.assertEqual(120, len(selected))
        self.assertEqual(len(selected), len(set(selected)))

if __name__ == "__main__":
    unittest.main()
