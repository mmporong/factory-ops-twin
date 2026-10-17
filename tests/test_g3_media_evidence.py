"""Fail-closed qualification media-window evidence tests."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from typing import Any, Callable

from factory_ops.visual_evidence import _validate_real_time_media


RUN_ID = "a" * 32
SCENARIO_ID = "QUALIFICATION_NORMAL"
PROFILE = {
    "profiles": ["qualification"],
    "window_s": 24.0,
    "sample_every_physics_steps": 6,
    "minimum_abs_linear_m_s": 0.15,
    "minimum_abs_angular_rad_s": 0.15,
}


def save(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n", encoding="utf-8")


def save_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows), encoding="utf-8")


class MediaFixture:
    def __init__(self, root: Path, *, profile: str = "qualification", primary: bool = False) -> None:
        self.session = root / "session"
        self.output = root / "output"
        self.session.mkdir(); self.output.mkdir()
        save(self.session / "session.json", {
            "profile": profile, "primary_evaluation": primary, "scenario_id": SCENARIO_ID,
        })
        save(self.session / "config.json", {
            "physics_dt_s": 1 / 60, "g3_runtime": {"real_time_media": PROFILE},
        })
        profile_sha = hashlib.sha256(json.dumps(
            PROFILE, sort_keys=True, separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")).hexdigest()
        self.start = {
            "schema_version": 1, "event": "window_started", "run_id": RUN_ID,
            "scenario_id": SCENARIO_ID, "window_id": "qualification-motion-001",
            "source_profile_sha256": profile_sha, "trigger_robot_id": "carter1",
            "trigger_command": {"linear_m_s": 0.2, "angular_rad_s": 0.0},
            "trigger_frame": 100, "start_sim_time_ns": 5_000_000_000,
            "end_sim_time_ns": 29_000_000_000, "sample_every_physics_steps": 6,
        }
        self.end = {
            "schema_version": 1, "event": "window_ended", "run_id": RUN_ID,
            "scenario_id": SCENARIO_ID, "window_id": "qualification-motion-001",
            "physics_frame": 1540, "sim_time_ns": 29_000_000_000,
        }
        save_jsonl(self.output / "real-time-media-events.jsonl", [self.start, self.end])
        frames = []
        for index in range(240):
            frame = 100 + 6 * index
            sim_ns = 5_000_000_000 + 100_000_000 * index
            frames.append({
                "index": index, "physics_frame": frame, "sim_time_s": sim_ns / 1_000_000_000,
                "visual_state": {"real_time_media": {
                    "window_id": "qualification-motion-001", "source_profile_sha256": profile_sha,
                    "trigger_robot_id": "carter1", "trigger_command": self.start["trigger_command"],
                    "physics_frame": frame, "sim_time_ns": sim_ns,
                    "window_start_sim_time_ns": 5_000_000_000,
                    "window_end_sim_time_ns": 29_000_000_000,
                    "sample_index": index, "sample_every_physics_steps": 6,
                }},
            })
        save(self.output / "media/frames.json", {"schema_version": 1, "frames": frames})
        save_jsonl(self.output / "robots/carter1/trajectory.jsonl", [{
            "physics_frame": 100, "sim_time_s": 5.0,
            "authorization": {"valid": True, "mode": "navigate"},
            "motion_profile": {"input_command": self.start["trigger_command"]},
        }])

    def load(self, relative: str) -> dict[str, Any]:
        return json.loads((self.output / relative).read_text(encoding="utf-8"))


class RealTimeMediaEvidenceTests(unittest.TestCase):
    def validate(self, fixture: MediaFixture) -> tuple[dict[str, Any], list[str]]:
        errors: list[str] = []
        report = _validate_real_time_media(
            fixture.output, fixture.session, RUN_ID, SCENARIO_ID, errors,
        )
        return report, errors

    def test_complete_240_sample_window_passes_with_journal_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = MediaFixture(Path(temporary))
            report, errors = self.validate(fixture)
        self.assertEqual(errors, [])
        self.assertEqual(
            (report["required"], report["passed"], report["sample_count"], report["journal_event_count"]),
            (True, True, 240, 2),
        )
        self.assertEqual(report["physics_frame_range"], [100, 1534])
        self.assertEqual(report["sim_time_ns_range"], [5_000_000_000, 28_900_000_000])
        self.assertRegex(report["journal_sha256"], r"^[0-9a-f]{64}$")

    def test_incomplete_or_tampered_media_fails_closed(self) -> None:
        def no_trigger(fixture: MediaFixture) -> None:
            (fixture.output / "real-time-media-events.jsonl").unlink()

        def truncated(fixture: MediaFixture) -> None:
            end = deepcopy(fixture.end); end["sim_time_ns"] -= 1
            save_jsonl(fixture.output / "real-time-media-events.jsonl", [fixture.start, end])

        def missing_grid(fixture: MediaFixture) -> None:
            document = fixture.load("media/frames.json"); document["frames"].pop(10)
            save(fixture.output / "media/frames.json", document)

        def duplicate_grid(fixture: MediaFixture) -> None:
            document = fixture.load("media/frames.json"); document["frames"].append(deepcopy(document["frames"][-1]))
            save(fixture.output / "media/frames.json", document)

        def off_grid(fixture: MediaFixture) -> None:
            document = fixture.load("media/frames.json")
            document["frames"][10]["physics_frame"] += 1
            document["frames"][10]["visual_state"]["real_time_media"]["physics_frame"] += 1
            save(fixture.output / "media/frames.json", document)

        def timestamp(fixture: MediaFixture) -> None:
            document = fixture.load("media/frames.json")
            document["frames"][10]["visual_state"]["real_time_media"]["sim_time_ns"] += 1
            save(fixture.output / "media/frames.json", document)

        def profile(fixture: MediaFixture) -> None:
            config = json.loads((fixture.session / "config.json").read_text(encoding="utf-8"))
            config["g3_runtime"]["real_time_media"]["minimum_abs_linear_m_s"] = 0.16
            save(fixture.session / "config.json", config)

        def profile_scope(fixture: MediaFixture) -> None:
            config = json.loads((fixture.session / "config.json").read_text(encoding="utf-8"))
            config["g3_runtime"]["real_time_media"]["profiles"] = ["pilot"]
            save(fixture.session / "config.json", config)

        def trigger(fixture: MediaFixture) -> None:
            path = fixture.output / "robots/carter1/trajectory.jsonl"
            row = json.loads(path.read_text(encoding="utf-8")); row["authorization"]["valid"] = False
            save_jsonl(path, [row])

        mutations: tuple[tuple[str, Callable[[MediaFixture], None]], ...] = (
            ("no-trigger", no_trigger), ("truncated-window", truncated),
            ("missing-grid", missing_grid), ("duplicate-grid", duplicate_grid),
            ("off-grid", off_grid), ("timestamp", timestamp),
            ("profile", profile), ("profile-scope", profile_scope), ("trigger", trigger),
        )
        for label, mutate in mutations:
            with self.subTest(label=label), tempfile.TemporaryDirectory() as temporary:
                fixture = MediaFixture(Path(temporary)); mutate(fixture)
                report, errors = self.validate(fixture)
                self.assertFalse(report["passed"])
                self.assertTrue(errors)
                self.assertIn("real-time media evidence invalid", errors[0])

    def test_pilot_primary_and_legacy_sessions_do_not_require_the_contract(self) -> None:
        for profile, primary in (("pilot", False), ("primary", True)):
            with self.subTest(profile=profile), tempfile.TemporaryDirectory() as temporary:
                fixture = MediaFixture(Path(temporary), profile=profile, primary=primary)
                (fixture.output / "real-time-media-events.jsonl").unlink()
                report, errors = self.validate(fixture)
                self.assertEqual((report["required"], report["passed"], errors), (False, True, []))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); session = root / "session"; output = root / "output"
            session.mkdir(); output.mkdir()
            report = _validate_real_time_media(output, session, RUN_ID, SCENARIO_ID, [])
            self.assertEqual((report["required"], report["passed"]), (False, True))


if __name__ == "__main__":
    unittest.main()
