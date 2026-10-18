from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.g3_prepare import _bindings, prepare_g3
from factory_ops.g3_session import (
    CONFIG,
    CONFIG_V4,
    _validate_runtime,
    config_dir_from_session,
    prepare_session,
    source_hashes,
    verify_session,
)


ROOT = Path(__file__).resolve().parents[1]
V4 = ROOT / "configs" / "factory-v4"


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class CompactSessionTests(unittest.TestCase):
    def test_v4_normal_qualification_is_two_orders_and_auto_selects_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session_root = Path(temporary) / "session"
            session = prepare_session(
                session_root,
                config_dir=V4,
                profile="qualification",
                policy="fifo",
                condition="normal",
                seed=0,
            )
            automatic = verify_session(session_root)
            resolved = config_dir_from_session(session_root)
            config = json.loads((session_root / "config.json").read_text(encoding="utf-8"))
            scenarios = json.loads((session_root / "scenarios.json").read_text(encoding="utf-8"))

        self.assertEqual(session, automatic)
        self.assertEqual(resolved, CONFIG_V4.resolve())
        self.assertEqual(session["config_profile"], {
            "name": "factory-v4",
            "layout_revision": "amr-production-bay-v4",
            "repository_relative": "configs/factory-v4",
        })
        self.assertEqual(len(session["workload"]["orders"]), 2)
        self.assertEqual(session["source_config"], source_hashes(V4))
        self.assertEqual(config["camera_capture_every_steps"], 600)
        self.assertEqual(config["g3_runtime"]["real_time_media"]["profiles"], [])
        scenario = scenarios["scenarios"]["QUALIFICATION_NORMAL"]
        self.assertEqual(scenario["spawns"], {
            "carter1": [12.0, 1.5, 1.5707963267948966],
            "carter2": [12.0, 18.5, -1.5707963267948966],
        })

    def test_v4_closure_session_and_local_preflight_bind_compact_barrier(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session_root = root / "session"
            session = prepare_session(
                session_root,
                config_dir=V4,
                profile="qualification",
                policy="eta",
                condition="closure",
                seed=1,
            )
            verified = verify_session(session_root)
            package = prepare_g3(V4, root / "preflight")
            closed_map = json.loads((session_root / "maps/closed/manifest.json").read_text(encoding="utf-8"))

        self.assertEqual(session, verified)
        self.assertEqual(len(session["workload"]["orders"]), 2)
        self.assertEqual(session["scene_closure"]["source"], "configs/factory-v4/layout.json")
        self.assertEqual(session["scene_closure"]["rect_xyxy_m"], [18.5, 2.5, 19.5, 5.5])
        self.assertTrue(closed_map["dynamic_barrier_included"])
        self.assertTrue(package["verified"])
        self.assertEqual(package["routes_checked"], 112)
        self.assertEqual(package["pending_trials"], 60)
        self.assertFalse(package["gpu_execution_allowed"])

    def test_v4_source_bindings_are_hash_bound_and_profile_tampering_fails(self) -> None:
        settings = json.loads((V4 / "g3-experiment.json").read_text(encoding="utf-8"))
        _bindings(settings, V4)
        expected = {entry["path"]: entry["sha256"] for entry in settings["source_bindings"]}
        for relative in ("layout.json", "traffic.json", "g2.json", "g3-routing.json"):
            self.assertEqual(expected[f"configs/factory-v4/{relative}"], _sha(V4 / relative))
        self.assertEqual(set(source_hashes(V4)), {
            "layout.json", "traffic.json", "g2.json", "g3-routing.json", "g3-experiment.json",
            "g3-runtime.json", "g3-goal-acceptance-provenance.json",
            "g3-controller-entry-provenance.json", "nav2-g2.yaml",
        })

        bad = copy.deepcopy(settings)
        bad["source_bindings"][0]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "source binding differs"):
            _bindings(bad, V4)

        renamed = copy.deepcopy(settings)
        for binding in renamed["source_bindings"]:
            binding["path"] = binding["path"].replace("configs/factory-v4/", "configs/factory-v3/")
        with self.assertRaisesRegex(ValueError, "canonical config profile"):
            _bindings(renamed, V4)

        with tempfile.TemporaryDirectory() as temporary:
            session_root = Path(temporary) / "session"
            prepare_session(session_root, config_dir=V4)
            session_path = session_root / "session.json"
            document = json.loads(session_path.read_text(encoding="utf-8"))
            document["config_profile"]["name"] = "factory-v3"
            session_path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "config profile differs"):
                config_dir_from_session(session_root)

    def test_runtime_media_accepts_disabled_or_qualification_only(self) -> None:
        runtime = json.loads((V4 / "g3-runtime.json").read_text(encoding="utf-8"))
        _validate_runtime(runtime)
        enabled = copy.deepcopy(runtime)
        enabled["real_time_media"]["profiles"] = ["qualification"]
        _validate_runtime(enabled)
        invalid = copy.deepcopy(runtime)
        invalid["real_time_media"]["profiles"] = ["pilot"]
        with self.assertRaisesRegex(ValueError, "disabled or qualification-only"):
            _validate_runtime(invalid)

    def test_existing_v3_session_keeps_default_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session_root = Path(temporary) / "session"
            session = prepare_session(session_root, config_dir=CONFIG)
            self.assertNotIn("config_profile", session)
            self.assertEqual(config_dir_from_session(session_root), CONFIG.resolve())
            self.assertEqual(verify_session(session_root), session)


if __name__ == "__main__":
    unittest.main()
