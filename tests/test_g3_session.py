"""G3 session provenance tests; generated inputs are not runtime evidence."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.g3_session import SCOPE, prepare_session, verify_session


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/factory-v3"


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def save(path: Path, value) -> None:
    path.write_text(
        json.dumps(value, sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class G3SessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.temporary = tempfile.TemporaryDirectory()
        cls.workspace = Path(cls.temporary.name)
        cls.qualification = cls.workspace / "qualification"
        cls.pilot_closure = cls.workspace / "pilot-closure"
        prepare_session(
            cls.qualification,
            profile="qualification",
            condition="normal",
            policy="fifo",
            seed=0,
        )
        prepare_session(
            cls.pilot_closure,
            profile="pilot",
            condition="closure",
            policy="eta",
            seed=3,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.temporary.cleanup()

    def copy_session(self, source: Path | None = None) -> Path:
        target = self.workspace / self.id().rsplit(".", 1)[-1]
        shutil.copytree(source or self.qualification, target)
        return target

    def test_public_profiles_are_source_bound_and_reproducible(self) -> None:
        qualification = verify_session(self.qualification)
        pilot = verify_session(self.pilot_closure)
        self.assertEqual(
            (qualification["scope"], qualification["primary_evaluation"], qualification["profile"]),
            (SCOPE, False, "qualification"),
        )
        self.assertEqual(
            (pilot["scope"], pilot["profile"], pilot["condition"], pilot["policy"], pilot["seed"]),
            (SCOPE, "pilot", "closure", "eta", 3),
        )
        self.assertIsNone(qualification["scene_closure"])
        self.assertEqual(
            pilot["scene_closure"]["scene_active_interval_s"],
            [0.0, 600.0],
        )
        self.assertEqual(set(qualification["files"]), set(pilot["files"]))

    def test_expected_run_context_rejects_policy_relabel_before_derivation(self) -> None:
        session_path = self.copy_session()
        value = load(session_path / "session.json")
        value["policy"] = "eta"
        save(session_path / "session.json", value)
        with self.assertRaisesRegex(ValueError, "expected run context"):
            verify_session(session_path, expected_context={"profile": "qualification", "policy": "fifo", "condition": "normal", "seed": 0})

    def test_outer_hash_updates_do_not_authorize_config_or_scene_tampering(self) -> None:
        for relative, mutate in (
            (
                "config.json",
                lambda value: value["g3_runtime"].__setitem__("chunk_timeout_s", 181),
            ),
            (
                "scene/factory-v3.usda",
                None,
            ),
        ):
            with self.subTest(relative=relative):
                session_path = self.copy_session()
                artifact = session_path / relative
                if mutate is None:
                    artifact.write_bytes(artifact.read_bytes() + b"\n# coherent outer-hash tamper\n")
                else:
                    value = load(artifact)
                    mutate(value)
                    save(artifact, value)
                manifest = load(session_path / "session.json")
                manifest["files"][relative] = digest(artifact)
                save(session_path / "session.json", manifest)
                with self.assertRaisesRegex(ValueError, "metadata differs|source-derived artifact differs"):
                    verify_session(session_path)
                shutil.rmtree(session_path)

    def test_current_config_source_change_invalidates_an_existing_session(self) -> None:
        config_dir = self.workspace / "alternate-config"
        shutil.copytree(CONFIG, config_dir)
        runtime = load(config_dir / "g3-runtime.json")
        runtime["camera_capture_every_steps"] = 61
        save(config_dir / "g3-runtime.json", runtime)
        session_path = self.workspace / "alternate-session"
        prepared = prepare_session(session_path, config_dir=config_dir)
        self.assertEqual(prepared["source_config"]["g3-runtime.json"], digest(config_dir / "g3-runtime.json"))
        runtime["camera_capture_every_steps"] = 62
        save(config_dir / "g3-runtime.json", runtime)
        with self.assertRaisesRegex(ValueError, "source config differs"):
            verify_session(session_path, config_dir=config_dir)

    def test_existing_output_is_rejected_without_changing_any_byte(self) -> None:
        session_path = self.copy_session()
        before = {
            path.relative_to(session_path).as_posix(): digest(path)
            for path in session_path.rglob("*")
            if path.is_file()
        }
        with self.assertRaisesRegex(ValueError, "new dedicated directory"):
            prepare_session(session_path, profile="pilot", condition="burst", policy="eta", seed=9)
        after = {
            path.relative_to(session_path).as_posix(): digest(path)
            for path in session_path.rglob("*")
            if path.is_file()
        }
        self.assertEqual(before, after)

    def test_extra_file_and_symlink_are_rejected(self) -> None:
        extra = self.copy_session()
        (extra / "unbound.json").write_text("{}\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "exact file set differs"):
            verify_session(extra)

        linked = self.workspace / "linked-session"
        shutil.copytree(self.qualification, linked)
        link = linked / "config-link.json"
        try:
            os.symlink(linked / "config.json", link)
        except OSError:
            original = Path.is_symlink
            with patch.object(
                Path,
                "is_symlink",
                autospec=True,
                side_effect=lambda path: path == linked / "config.json" or original(path),
            ):
                with self.assertRaisesRegex(ValueError, "regular files"):
                    verify_session(linked)
        else:
            with self.assertRaisesRegex(ValueError, "regular files"):
                verify_session(linked)

    def test_scope_scenario_and_closure_are_not_manifest_editable(self) -> None:
        scope = self.copy_session()
        manifest = load(scope / "session.json")
        manifest["scope"] = "physical_performance_verified"
        save(scope / "session.json", manifest)
        with self.assertRaisesRegex(ValueError, "scope differs"):
            verify_session(scope)

        scenario = self.workspace / "scenario-tamper"
        shutil.copytree(self.qualification, scenario)
        manifest = load(scenario / "session.json")
        old_id = manifest["scenario_id"]
        new_id = "QUALIFICATION_FORGED"
        manifest["scenario_id"] = new_id
        scenarios = load(scenario / "scenarios.json")
        scenarios["scenarios"][new_id] = scenarios["scenarios"].pop(old_id)
        save(scenario / "scenarios.json", scenarios)
        manifest["files"]["scenarios.json"] = digest(scenario / "scenarios.json")
        save(scenario / "session.json", manifest)
        with self.assertRaisesRegex(ValueError, "metadata differs"):
            verify_session(scenario)

        closure = self.workspace / "closure-tamper"
        shutil.copytree(self.pilot_closure, closure)
        manifest = load(closure / "session.json")
        forged = deepcopy(manifest["scene_closure"])
        forged["scene_active_interval_s"] = [0.0, 601.0]
        manifest["scene_closure"] = forged
        save(closure / "session.json", manifest)
        with self.assertRaisesRegex(ValueError, "metadata differs"):
            verify_session(closure)


if __name__ == "__main__":
    unittest.main()
