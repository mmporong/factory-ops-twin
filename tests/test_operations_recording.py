"""Bounded exact operations snapshot recording tests."""
from __future__ import annotations

import gzip
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.operations_recording import MAX_FRAMES, SnapshotRecorder, record_case


class _RecordingModel:
    def __init__(self, config: Path, *, policy: str, initial_orders: int, run_id: str | None) -> None:
        self.policy, self.run_id = policy, run_id
        self.sim_time = 0.0
        self.orders: list[str] = []

    def create_order(self, parameters: dict[str, object]) -> str:
        order = f"ORD-{len(self.orders) + 1:04d}"
        self.orders.append(order)
        return order

    def set_equipment_availability(self, parameters: dict[str, object]) -> None:
        pass

    def set_aisle_closed(self, parameters: dict[str, object]) -> None:
        pass

    def advance(self, dt: float) -> None:
        self.sim_time += dt

    def snapshot(self) -> dict[str, object]:
        return {
            "schema_version": 1, "run_id": self.run_id, "sim_time_s": self.sim_time,
            "execution_mode": "operations", "dispatch_policy": self.policy,
            "robots": [{"id": "amr-1", "status": "idle", "pose": {"x_m": self.sim_time}}],
            "tasks": [{"id": f"{order}:out", "order_id": order, "type": "outbound",
                       "state": "ready", "robot_id": None} for order in self.orders],
            "events": [], "equipment": {"wip_orders": len(self.orders)},
            "traffic": {"wait_s": {}, "quiescent": True, "observed_min_clearance_m": None,
                        "peak_moving_robots": 0},
            "inventory": {}, "kpis": {"completed_orders": 0},
        }


def _config(root: Path) -> Path:
    config = root / "config"
    config.mkdir()
    for index, name in enumerate(("layout.json", "g2.json", "g3-routing.json", "transport-profile.json")):
        (config / name).write_text(json.dumps({"fixture": index}) + "\n", encoding="utf-8")
    return config


class OperationsRecordingTest(unittest.TestCase):
    def test_recorder_samples_exact_observed_snapshots_without_interpolation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshots.jsonl.gz"
            recorder = SnapshotRecorder(path, start_s=0, duration_s=1, sample_hz=5)
            frames = [
                {"sim_time_s": 0.1, "marker": "observed-a"},
                {"sim_time_s": 0.2, "marker": "skip"},
                {"sim_time_s": 0.3, "marker": "observed-b"},
                {"sim_time_s": 0.4, "marker": "skip"},
                {"sim_time_s": 0.5, "marker": "observed-c"},
            ]
            for frame in frames:
                recorder(frame)
            recorder.close()
            with gzip.open(path, "rt", encoding="utf-8") as stream:
                recorded = [json.loads(line) for line in stream]
            self.assertEqual([frames[0], frames[2], frames[4]], recorded)
            self.assertEqual(3, recorder.evidence()["frames"])
            self.assertEqual(0.1, recorder.evidence()["first_observed_sim_time_s"])
            self.assertEqual(0.5, recorder.evidence()["last_observed_sim_time_s"])

    def test_record_case_writes_bounded_manifest_frames_and_partial_report(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = _config(root)
            output = root / "recording"
            paths = record_case(
                config, scenario="normal", policy="fifo", horizon_s=0.5,
                window_start_s=0.1, window_duration_s=0.4, sample_hz=5,
                output_dir=output, model_factory=_RecordingModel,
            )
            manifest = json.loads(paths["manifest"].read_text(encoding="utf-8"))
            report = json.loads(paths["case_report"].read_text(encoding="utf-8"))
            with gzip.open(paths["frames"], "rt", encoding="utf-8") as stream:
                frames = [json.loads(line) for line in stream]

            self.assertEqual(1, manifest["schema_version"])
            self.assertEqual("observed snapshots only; no interpolation", manifest["window"]["sampling"])
            self.assertEqual(len(frames), manifest["frame_count"])
            self.assertEqual(len(frames), manifest["snapshots"]["frames"])
            self.assertRegex(manifest["snapshots"]["uncompressed_sha256"], r"^[0-9a-f]{64}$")
            self.assertRegex(manifest["snapshots"]["gzip_sha256"], r"^[0-9a-f]{64}$")
            self.assertTrue(1 <= len(frames) <= MAX_FRAMES)
            self.assertTrue(all(frame["execution_mode"] == "operations" for frame in frames))
            self.assertNotIn("representative_snapshots", report)
            self.assertGreater(report["representative_snapshot_count_not_copied"], 0)
            self.assertEqual("deepcopy", report["snapshot_observer"]["copy_mode"])
            self.assertTrue(report["provenance"]["frozen_source"]["binding_matches_case_start"])
            self.assertFalse(any(path.suffix.lower() in (".png", ".jpg") for path in output.rglob("*")))
            preserved = paths["frames"].read_bytes()
            with self.assertRaises(FileExistsError):
                record_case(
                    config, scenario="normal", policy="fifo", horizon_s=0.5,
                    window_start_s=0.1, window_duration_s=0.4, sample_hz=5,
                    output_dir=output, model_factory=_RecordingModel,
                )
            self.assertEqual(preserved, paths["frames"].read_bytes())

    def test_rejects_overwrite_and_unbounded_window(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            existing = root / "snapshots.jsonl.gz"
            existing.write_bytes(b"preserve")
            with self.assertRaises(FileExistsError):
                SnapshotRecorder(existing, start_s=0, duration_s=1, sample_hz=5)
            self.assertEqual(b"preserve", existing.read_bytes())
            with self.assertRaisesRegex(ValueError, "frame cap"):
                SnapshotRecorder(root / "large.jsonl.gz", start_s=0, duration_s=3000, sample_hz=10)
            config = _config(root)
            rejected = root / "rejected"
            with self.assertRaisesRegex(ValueError, "at most"):
                record_case(
                    config, scenario="normal", policy="fifo", horizon_s=1,
                    window_start_s=0, window_duration_s=1, sample_hz=11,
                    output_dir=rejected, model_factory=_RecordingModel,
                )
            self.assertFalse(rejected.exists())


if __name__ == "__main__":
    unittest.main()
