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

from factory_ops.experiments import build_matrix, load_experiment_config
from factory_ops.g3_session import (
    PRIMARY_SCOPE,
    SCOPE,
    _sequential_input_bytes,
    _validate_runtime,
    prepare_primary_session,
    prepare_session,
    verify_session,
)


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
            seed=100,
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
            (SCOPE, "pilot", "closure", "eta", 100),
        )
        self.assertIsNone(qualification["scene_closure"])
        self.assertEqual(
            pilot["scene_closure"]["scene_active_interval_s"],
            [0.0, 600.0],
        )
        self.assertEqual(set(qualification["files"]), set(pilot["files"]))
        qualification_config = load(self.qualification / "config.json")
        pilot_config = load(self.pilot_closure / "config.json")
        self.assertEqual(qualification_config["g3_runtime"]["real_time_media"],
                         pilot_config["g3_runtime"]["real_time_media"])
        self.assertEqual(
            qualification_config["g3_runtime"]["real_time_media"],
            {
                "profiles": ["qualification"], "window_s": 24.0,
                "sample_every_physics_steps": 6,
                "minimum_abs_linear_m_s": 0.15,
                "minimum_abs_angular_rad_s": 0.15,
            },
        )

    def test_real_time_media_profile_rejects_malformed_or_broadened_values(self) -> None:
        runtime = load(CONFIG / "g3-runtime.json")
        mutations = (
            lambda media: media.update({"extra": True}),
            lambda media: media.__setitem__("profiles", ["qualification", "pilot"]),
            lambda media: media.__setitem__("window_s", 30.1),
            lambda media: media.__setitem__("sample_every_physics_steps", True),
            lambda media: media.__setitem__("minimum_abs_linear_m_s", float("nan")),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                candidate = deepcopy(runtime)
                mutate(candidate["real_time_media"])
                with self.assertRaisesRegex(ValueError, "real-time media"):
                    _validate_runtime(candidate)

    def test_callback_drain_bound_is_source_bound_and_strictly_validated(self) -> None:
        runtime = load(CONFIG / "g3-runtime.json")
        self.assertEqual(runtime["callback_drain"], {"max_rounds": 64, "wall_budget_s": 0.05})
        mutations = (
            lambda value: value["callback_drain"].update({"extra": True}),
            lambda value: value["callback_drain"].__setitem__("max_rounds", True),
            lambda value: value["callback_drain"].__setitem__("max_rounds", 0),
            lambda value: value["callback_drain"].__setitem__("max_rounds", 257),
            lambda value: value["callback_drain"].__setitem__("wall_budget_s", 0),
            lambda value: value["callback_drain"].__setitem__("wall_budget_s", 0.100001),
            lambda value: value["callback_drain"].__setitem__("wall_budget_s", float("nan")),
        )
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                candidate = deepcopy(runtime)
                mutate(candidate)
                with self.assertRaisesRegex(ValueError, "callback drain"):
                    _validate_runtime(candidate)

    def test_g3_nav2_override_is_exactly_stateful_false(self) -> None:
        runtime = load(CONFIG / "g3-runtime.json")
        self.assertEqual(runtime["g3_nav2"], {"follow_path_stateful": False})
        for value in ({}, {"follow_path_stateful": True},
                      {"follow_path_stateful": False, "xy_goal_tolerance": 0.06}):
            with self.subTest(value=value):
                candidate = deepcopy(runtime)
                candidate["g3_nav2"] = value
                with self.assertRaisesRegex(ValueError, "G3 Nav2 override"):
                    _validate_runtime(candidate)

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
        runtime["real_time_media"]["minimum_abs_linear_m_s"] = 0.16
        save(config_dir / "g3-runtime.json", runtime)
        session_path = self.workspace / "alternate-session"
        prepared = prepare_session(session_path, config_dir=config_dir)
        self.assertEqual(prepared["source_config"]["g3-runtime.json"], digest(config_dir / "g3-runtime.json"))
        runtime["real_time_media"]["minimum_abs_linear_m_s"] = 0.17
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

    def test_sequential_primary_inputs_freeze_plan_receipts_and_support(self) -> None:
        root = self.workspace / self.id().rsplit(".", 1)[-1]
        root.mkdir()
        closure = root / "closure.json"
        save(closure, {"portable": "source-support-metadata"})
        envelope = root / "raw-envelope.json"
        save(envelope, {"portable": "raw-envelope-metadata"})
        manifest = root / "package-manifest.json"
        save(manifest, {"portable": True})
        package = root / "package-record.json"
        save(package, {"package_manifest": {"path": manifest.name, "sha256": digest(manifest)}})
        preservation = root / "preservation.json"
        restore = root / "restore.json"
        semantic = root / "semantic.json"
        for path in (preservation, restore, semantic):
            save(path, {"portable": path.stem})
        run = {"package_record": {"path": package.name, "sha256": digest(package)},
               "preservation_receipt": {"path": preservation.name, "sha256": digest(preservation)},
               "full_restore_receipt": {"path": restore.name, "sha256": digest(restore)},
               "semantic_replay_report": {"path": semantic.name, "sha256": digest(semantic)},
               "source_support_closure": {"path": closure.name, "sha256": digest(closure)},
               "raw_envelope": {"path": envelope.name, "sha256": digest(envelope)}}
        receipts = []
        for store in ("D", "E"):
            receipt = root / f"receipt-{store}.json"
            save(receipt, {"store_id": store, "runs": [run]})
            receipts.append({"path": receipt.name, "sha256": digest(receipt)})
        plan = root / "plan.json"
        save(plan, {"coordinator_receipts": receipts})
        resource = {"evidence_root": str(root), "supervisor_path": "supervisor.json",
                    "supervisor_sha256": "a" * 64, "requested_primary_runs": 60,
                    "reserve_fraction": .2, "schema_version": 2,
                    "sequential_plan_path": plan.name, "sequential_plan_sha256": digest(plan)}
        frozen = _sequential_input_bytes(resource)
        self.assertEqual(plan.read_bytes(), frozen["sequential-capacity/plan.json"])
        self.assertEqual(envelope.read_bytes(), frozen["sequential-capacity/coordinator-0-run-0-raw_envelope.json"])
        envelope.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "bytes differ"):
            _sequential_input_bytes(resource)

    def test_primary_session_preserves_and_revalidates_exact_external_inputs(self) -> None:
        settings = load_experiment_config(CONFIG / "g3-experiment.json")
        trial = build_matrix(settings)[0]
        trial_path = self.workspace / "primary-trial.json"
        trial_path.write_text(json.dumps(trial, indent=1) + "\n", encoding="utf-8")
        calibration = {
            "classification": "g3_actual_six_pilot_calibration",
            "status": "primary_execution_eligible",
            "execution_eligible": True,
            "calibration_sha256": "a" * 64,
        }
        calibration_path = self.workspace / "primary-calibration.json"
        calibration_path.write_text(json.dumps(calibration, indent=3) + "\n", encoding="utf-8")
        output = self.workspace / "primary-session"
        external = {
            "trial_path": trial_path,
            "calibration_path": calibration_path,
            "pilot_slots": {"external": {"paths": "are revalidated by calibration"}},
            "resource_input": {"external": "resource evidence"},
            "expected_source_commit": "b" * 40,
        }
        with patch("factory_ops.g3_calibration.validate_calibration") as validate:
            session = prepare_primary_session(output, **external)
            self.assertGreaterEqual(validate.call_count, 2)
            verified = verify_session(output, **external)
        self.assertEqual((session["scope"], session["primary_evaluation"]), (PRIMARY_SCOPE, True))
        self.assertEqual(verified, session)
        self.assertEqual((output / "trial-input.json").read_bytes(), trial_path.read_bytes())
        self.assertEqual((output / "calibration-gate.json").read_bytes(), calibration_path.read_bytes())
        self.assertEqual(
            (output / "base-experiment-config.json").read_bytes(),
            (CONFIG / "g3-experiment.json").read_bytes(),
        )
        self.assertEqual((output / "fixed-source-commit.txt").read_text(), "b" * 40 + "\n")

        calibration_path.write_bytes(calibration_path.read_bytes() + b" ")
        with patch("factory_ops.g3_calibration.validate_calibration"):
            with self.assertRaisesRegex(ValueError, "exact input bytes"):
                verify_session(output, **external)

    def test_primary_session_rejects_fixture_calibration_even_after_callback(self) -> None:
        settings = load_experiment_config(CONFIG / "g3-experiment.json")
        trial_path = self.workspace / "fixture-trial.json"
        save(trial_path, build_matrix(settings)[1])
        calibration_path = self.workspace / "fixture-calibration.json"
        save(calibration_path, {
            "classification": "explicit_non_evidence_test_fixture",
            "status": "primary_execution_eligible",
            "execution_eligible": True,
            "calibration_sha256": "c" * 64,
        })
        with patch("factory_ops.g3_calibration.validate_calibration"):
            with self.assertRaisesRegex(ValueError, "eligible actual calibration"):
                prepare_primary_session(
                    self.workspace / "fixture-primary-session",
                    trial_path=trial_path,
                    calibration_path=calibration_path,
                    pilot_slots={}, resource_input={}, expected_source_commit="d" * 40,
                )

    def test_primary_worker_verification_uses_only_sha_pinned_environment_manifest(self) -> None:
        settings = load_experiment_config(CONFIG / "g3-experiment.json")
        trial_path = self.workspace / "env-primary-trial.json"
        save(trial_path, build_matrix(settings)[2])
        calibration_path = self.workspace / "env-primary-calibration.json"
        save(calibration_path, {
            "classification": "g3_actual_six_pilot_calibration",
            "status": "primary_execution_eligible", "execution_eligible": True,
            "calibration_sha256": "e" * 64,
        })
        external = {
            "trial_path": trial_path, "calibration_path": calibration_path,
            "pilot_slots": {"external": {}}, "resource_input": {"external": True},
            "expected_source_commit": "f" * 40,
        }
        output = self.workspace / "env-primary-session"
        with patch("factory_ops.g3_calibration.validate_calibration"):
            prepare_primary_session(output, **external)
        manifest_path = self.workspace / "primary-inputs.json"
        save(manifest_path, {
            "trial_path": str(trial_path.resolve()),
            "calibration_path": str(calibration_path.resolve()),
            "pilot_slots": external["pilot_slots"],
            "resource_input": external["resource_input"],
            "expected_source_commit": external["expected_source_commit"],
        })
        environment = {
            "FACTORY_OPS_G3_PRIMARY_INPUTS_PATH": str(manifest_path.resolve()),
            "FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256": digest(manifest_path),
        }
        with patch.dict(os.environ, environment, clear=False), \
                patch("factory_ops.g3_calibration.validate_calibration") as validate:
            verified = verify_session(output, expected_sha256=digest(output / "session.json"))
        self.assertEqual(verified["scope"], PRIMARY_SCOPE)
        self.assertGreaterEqual(validate.call_count, 1)

        environment["FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256"] = "0" * 64
        with patch.dict(os.environ, environment, clear=False):
            with self.assertRaisesRegex(ValueError, "SHA differs"):
                verify_session(output)


if __name__ == "__main__":
    unittest.main()
