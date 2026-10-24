from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from factory_ops.operations_render import RecordedOperations, load_recording


def frame(stamp, x, yaw, payload):
    return {"run_id": "render-test", "execution_mode": "operations", "dispatch_policy": "eta",
            "sim_time_s": stamp, "kpis": {"completed_orders": int(stamp)},
            "robots": [{"id": f"amr-{i}", "pose": {"x_m": x, "y_m": i, "yaw_rad": yaw},
                        "payload_id": payload} for i in range(1, 5)]}


def file_sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_recording(folder: Path, *, frozen: bool, legacy: bool = False) -> str:
    folder.mkdir()
    frames = [frame(0, 0, 0, None), frame(1, .4, 0, "lot")]
    raw = b"".join(json.dumps(row, sort_keys=True).encode() + b"\n" for row in frames)
    stream = folder / "snapshots.jsonl.gz"
    with gzip.open(stream, "wb") as handle:
        handle.write(raw)
    layout_sha = hashlib.sha256(b'{"revision":"test"}\n').hexdigest()
    report = folder / "result.json"
    report.write_text(json.dumps({"run_id": "render-test", "source_stable_during_run": True}) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1, "run_id": "render-test", "case_status": "incomplete", "frame_count": 2,
        "snapshots": {"gzip_sha256": file_sha(stream), "uncompressed_sha256": hashlib.sha256(raw).hexdigest()},
        "case_report": {"path": "result.json", "sha256": file_sha(report)},
    }
    if frozen:
        frozen_root = folder / "frozen-source"
        (frozen_root / "config").mkdir(parents=True)
        (frozen_root / "factory_ops").mkdir()
        layout = frozen_root / "config/layout.json"
        producer = frozen_root / "factory_ops/model.py"
        layout.write_bytes(b'{"revision":"test"}\n')
        producer.write_bytes(b"SOURCE = 'test'\n")
        records = [
            {"path": "config/layout.json", "sha256": file_sha(layout), "bytes": layout.stat().st_size},
            {"path": "factory_ops/model.py", "sha256": file_sha(producer), "bytes": producer.stat().st_size},
        ]
        (frozen_root / "manifest.json").write_text(json.dumps(records) + "\n", encoding="utf-8")
    if legacy:
        manifest["sources"] = {"config_and_producer_sha256": {
            "config": {"layout.json": layout_sha},
            "producer": {"factory_ops/model.py": hashlib.sha256(b"SOURCE = 'test'\n").hexdigest()},
        }}
    (folder / "recording-manifest.json").write_text(json.dumps(manifest) + "\n", encoding="utf-8")
    return layout_sha


class OperationsRenderTest(unittest.TestCase):
    def test_sixteen_robot_recording_keeps_fleet_and_discrete_payload_identity(self):
        frames = [frame(0, 0, 0, None), frame(1, .4, 0, "lot")]
        for state in frames:
            state["robots"] = [
                {"id": f"amr-{i}", "pose": {"x_m": state["sim_time_s"], "y_m": i, "yaw_rad": 0.},
                 "payload_id": None if state["sim_time_s"] == 0 else f"lot-{i}"}
                for i in range(1, 17)
            ]
        replay = RecordedOperations(frames)
        sampled, _ = replay.sample(.5)
        self.assertEqual(len(sampled["robots"]), 16)
        self.assertTrue(all(row["payload_id"] is None for row in sampled["robots"]))
        self.assertEqual(replay.sample(1)[0]["robots"][-1]["payload_id"], "lot-16")
        frames[1]["robots"][-1]["id"] = "amr-1"
        with self.assertRaises(ValueError):
            RecordedOperations(frames)

    def test_pose_interpolation_does_not_anticipate_material_handover(self):
        original = [frame(0, 0, math.radians(179), None), frame(1, .4, math.radians(-179), "lot")]
        before = deepcopy(original)
        replay = RecordedOperations(original)
        state, meta = replay.sample(.5)
        self.assertAlmostEqual(state["robots"][0]["pose"]["x_m"], .2)
        self.assertAlmostEqual(state["robots"][0]["pose"]["yaw_rad"], math.pi)
        self.assertIsNone(state["robots"][0]["payload_id"])
        self.assertEqual(state["kpis"]["completed_orders"], 0)
        self.assertEqual(meta["state_observed_time_s"], 0)
        self.assertEqual(original, before)
        state, _ = replay.sample(1)
        self.assertEqual(state["robots"][0]["payload_id"], "lot")

    def test_no_extrapolation_or_mixed_run(self):
        frames = [frame(0, 0, 0, None), frame(1, .4, 0, None)]
        replay = RecordedOperations(frames)
        with self.assertRaises(ValueError):
            replay.sample(1.1)
        frames[1]["run_id"] = "other"
        with self.assertRaises(ValueError):
            RecordedOperations(frames)

    def test_frozen_source_fallback_returns_verified_pins_without_mutating_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / "recording"
            expected_layout = write_recording(folder, frozen=True)
            _recording, source = load_recording(folder)
            self.assertEqual(source["verified_source_pins"]["config"]["layout.json"], expected_layout)
            self.assertEqual(source["source_pin_provenance"]["origin"], "verified_frozen_source")
            self.assertNotIn("sources", source["manifest"])
            self.assertTrue(source["case_report_verified"])

    def test_frozen_source_tamper_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / "recording"
            write_recording(folder, frozen=True)
            (folder / "frozen-source/config/layout.json").write_text("tampered\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "differs from manifest"):
                load_recording(folder)

    def test_declared_source_stability_must_match_bound_case_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / "recording"
            write_recording(folder, frozen=True, legacy=True)
            manifest_path = folder / "recording-manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["sources"]["source_stable_during_run"] = False
            manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "stability binding"):
                load_recording(folder)

    def test_legacy_declared_source_pins_remain_loadable_without_frozen_copy(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / "recording"
            expected_layout = write_recording(folder, frozen=False, legacy=True)
            _recording, source = load_recording(folder)
            self.assertEqual(source["verified_source_pins"]["config"]["layout.json"], expected_layout)
            self.assertEqual(source["source_pin_provenance"]["origin"], "legacy_manifest")


if __name__ == "__main__":
    unittest.main()
