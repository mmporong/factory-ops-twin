from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.demo.import_operations_recording import import_recording


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/"configs/factory-semiconductor-dual-lane"
SOURCE = ROOT/"artifacts/operations/recording-223-normal/snapshots.jsonl.gz"


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(root: Path) -> tuple[Path, list[dict]]:
    recording = root/"recording"
    recording.mkdir()
    rows = []
    with gzip.open(SOURCE, "rt", encoding="utf-8") as stream:
        for index in range(4):
            row = json.loads(next(stream))
            row["sim_time_s"] = 1980.1 + index * 2.0
            rows.append(row)
    raw = b"".join((json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n").encode() for row in rows)
    gzip_path = recording/"snapshots.jsonl.gz"
    with gzip.open(gzip_path, "wb") as stream:
        stream.write(raw)
    case = {"schema_version": 1, "run_id": rows[0]["run_id"], "status": "complete",
            "metrics": {"completed_outbound_deliveries": 12, "empty_returns_completed": 12,
                        "business_finished": True, "fleet_quiescent": True}}
    case_path = recording/"case-report.json"
    case_path.write_text(json.dumps(case), encoding="utf-8")
    manifest = {
        "schema_version": 1, "run_id": rows[0]["run_id"], "case_status": "complete",
        "frame_count": len(rows),
        "snapshots": {"gzip_sha256": _sha(gzip_path),
                      "uncompressed_sha256": hashlib.sha256(raw).hexdigest()},
        "case_report": {"sha256": _sha(case_path)},
        "sources": {"config_and_producer_sha256": {"config": {
            "layout.json": _sha(CONFIG/"layout.json")}}},
    }
    (recording/"recording-manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return recording, rows


class ImportOperationsRecordingTests(unittest.TestCase):
    def test_preserves_raw_run_and_state_in_read_only_replay(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recording, rows = _fixture(root)
            output = root/"import"
            result = import_recording(recording=recording, config=CONFIG, start_s=1980.0,
                                      end_s=1990.0, output_dir=output, name="223 normal window")
            self.assertEqual(rows[0]["run_id"], result["source_recording"]["run_id"])
            self.assertEqual(4, result["selected_window"]["selected_frame_count"])
            self.assertTrue(result["source_recording"]["source_files_unchanged"])
            scenario = json.loads((output/result["scenario"]["scenario_file"]).read_text(encoding="utf-8"))
            self.assertEqual(rows, [frame["snapshot"] for frame in scenario["frames"]])
            self.assertEqual("replay", result["scenario"]["served_mode"])
            self.assertTrue(result["scenario"]["read_only"])

    def test_rejects_config_with_another_layout(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recording, _ = _fixture(root)
            other = root/"other-config"
            other.mkdir()
            layout = json.loads((CONFIG/"layout.json").read_text(encoding="utf-8"))
            layout["revision"] = "different-layout"
            (other/"layout.json").write_text(json.dumps(layout), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "layout differs"):
                import_recording(recording=recording, config=other, start_s=1980.0,
                                 end_s=1990.0, output_dir=root/"rejected", name="wrong layout")
            self.assertFalse((root/"rejected").exists())


if __name__ == "__main__":
    unittest.main()
