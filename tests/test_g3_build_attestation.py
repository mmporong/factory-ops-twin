from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.g3_build_attestation import (
    BUILDER_MODE, INSTALLED_PATHS, LEGACY_BUILD_PREFIX, SOURCE_PATHS,
    build_attested_image, create_nonphysical_fixture_attestation,
    validate_attestation,
)

COMMIT = "a" * 40
RUN = "g3-q-normal-fifo-014"

def is_legacy_build(args):
    return args[:4] == LEGACY_BUILD_PREFIX

class FakeRunner:
    def __init__(self, repository: Path):
        self.repository = repository
        self.commands = []
        self.base = [{"Id": "sha256:" + "1" * 64, "RepoDigests": ["base@sha256:" + "2" * 64]}]
        self.built = [{"Id": "sha256:" + "3" * 64, "RepoDigests": ["built@sha256:" + "4" * 64]}]
        self.built_exists = False

    def __call__(self, args, *, cwd=None):
        self.commands.append(list(args))
        stdout = ""
        if args[:3] == ["git", "rev-parse", "HEAD"]:
            stdout = COMMIT + "\n"
        elif args[:3] == ["git", "status", "--porcelain"]:
            stdout = ""
        elif args[:2] == ["git", "archive"]:
            output = Path(next(item.split("=", 1)[1] for item in args if item.startswith("--output=")))
            with tarfile.open(output, "w", format=tarfile.PAX_FORMAT,
                              pax_headers={"comment": COMMIT}) as archive:
                for path in sorted(self.repository.rglob("*")):
                    if path.is_file():
                        info = archive.gettarinfo(str(path), path.relative_to(self.repository).as_posix())
                        info.mtime = 0
                        with path.open("rb") as handle:
                            archive.addfile(info, handle)
        elif args == ["docker", "version", "--format", "{{json .}}"]:
            stdout = json.dumps({"Client": {"Version": "27.5.1"},
                                 "Server": {"Version": "27.5.1"}})
        elif args[:3] == ["docker", "image", "inspect"]:
            if args[3] != "base:pin" and not self.built_exists:
                raise subprocess.CalledProcessError(1, args)
            stdout = json.dumps(self.base if args[3] == "base:pin" else self.built)
        elif is_legacy_build(args):
            Path(args[args.index("--iidfile") + 1]).write_text(self.built[0]["Id"])
            self.built_exists = True
        elif args[:2] == ["docker", "create"]:
            stdout = "container-id\n"
        elif args[:2] == ["docker", "cp"]:
            source = args[2].split(":", 1)[1]
            Path(args[3]).write_bytes(("measured:" + source).encode())
        elif args[:2] != ["docker", "rm"]:
            raise AssertionError(args)
        return subprocess.CompletedProcess(args, 0, stdout, "")

class G3BuildAttestationTests(unittest.TestCase):
    @staticmethod
    def source_fixture(root: Path) -> None:
        for relative in SOURCE_PATHS:
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(("ARG BASE_IMAGE\nFROM ${BASE_IMAGE}\nFROM ${BASE_IMAGE}\n"
                             if relative == "scripts/ros/Dockerfile.g3" else relative))

    @staticmethod
    def validate_built(root: Path, output: Path, runner: FakeRunner, *,
                       built_inspect=None, measurement_name="preflight"):
        path = output / "build-attestation.json"
        return validate_attestation(path,
            expected_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_root=root, expected_source_commit=COMMIT, expected_run_label=RUN,
            base_inspect=runner.base, built_inspect=built_inspect or runner.built,
            source_archive=output / "source-before.tar",
            build_receipt=output / "build-receipt.json",
            expected_build_receipt_sha256=hashlib.sha256(
                (output / "build-receipt.json").read_bytes()).hexdigest(),
            build_commands=output / "commands.jsonl",
            build_iidfile=output / "built-image.iid",
            build_base_inspect=output / "base-image-inspect.json",
            build_built_inspect=output / "built-image-inspect.json",
            build_installed_measurement=output / "installed-measurement.json",
            measurement_dir=output.parent / measurement_name, runner=runner)

    def test_closed_build_orders_archive_pinned_base_build_measurement_and_recheck(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "build-proof"; runner = FakeRunner(root)
            with patch.dict(os.environ, {"DOCKER_BUILDKIT": "1"}):
                value = build_attested_image(output, repository=root, expected_commit=COMMIT,
                    run_label=RUN, base_reference="base:pin", runner=runner)
            self.assertEqual("built", value["status"])
            self.assertTrue(value["runtime_eligible"])
            self.assertEqual(value["base_image"]["id"], value["build_proof"]["base_image_arg"])
            build = next(command for command in runner.commands if is_legacy_build(command))
            self.assertEqual(LEGACY_BUILD_PREFIX, build[:4])
            self.assertIn(f"BASE_IMAGE={value['base_image']['id']}", build)
            self.assertIn("--iidfile", build)
            self.assertEqual(2, sum(command[:2] == ["git", "archive"] for command in runner.commands))
            self.assertEqual(3, sum(command[:2] == ["docker", "cp"] for command in runner.commands))
            receipt = json.loads((output / "build-receipt.json").read_text())
            self.assertEqual(BUILDER_MODE, receipt["builder_mode"])
            self.assertEqual("27.5.1", receipt["docker_client_version"])
            self.assertEqual("27.5.1", receipt["docker_engine_version"])
            path = output / "build-attestation.json"
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            validated = self.validate_built(root, output, runner)
            self.assertEqual(value["built_image"], validated["built_image"])
            self.assertRegex(validated["preflight_measurement_sha256"], r"^[0-9a-f]{64}$")
            preflight_creates = [command for command in runner.commands
                                 if command[:2] == ["docker", "create"]]
            self.assertEqual(value["built_image"]["id"], preflight_creates[-1][-1])
            run_script = (Path(__file__).parents[1] / "scripts/brev/run-g3.sh").read_text()
            self.assertNotIn("nav_image=$(docker image inspect", run_script)
            (root / SOURCE_PATHS[-1]).write_text("stale source")
            with self.assertRaises(ValueError):
                self.validate_built(root, output, runner, measurement_name="stale-preflight")

    def test_supplied_json_create_is_fixture_only_and_cannot_pass_runtime_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); self.source_fixture(root); runner = FakeRunner(root)
            installed = {name: "5" * 64 for name in INSTALLED_PATHS}
            path = root / "fixture.json"
            value = create_nonphysical_fixture_attestation(path, source_root=root, source_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", base_inspect=runner.base,
                built_reference="trace:q014", built_inspect=runner.built, installed_files=installed)
            self.assertEqual("nonphysical_fixture", value["status"])
            self.assertFalse(value["runtime_eligible"])
            with self.assertRaises(ValueError):
                validate_attestation(path, expected_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    source_root=root, expected_source_commit=COMMIT, expected_run_label=RUN,
                    base_inspect=runner.base, built_inspect=runner.built,
                    source_archive=root / "missing.tar", build_receipt=root / "missing-receipt",
                    expected_build_receipt_sha256="0" * 64,
                    build_commands=root / "missing-commands", build_iidfile=root / "missing-iid",
                    build_base_inspect=root / "missing-base", build_built_inspect=root / "missing-built",
                    build_installed_measurement=root / "missing-measurement",
                    measurement_dir=root / "missing-preflight", runner=runner)

    def test_failed_build_preserves_commands_but_issues_no_closed_attestation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "failed"; runner = FakeRunner(root)
            def fail_build(args, *, cwd=None):
                if is_legacy_build(args):
                    raise subprocess.CalledProcessError(7, args)
                return runner(args, cwd=cwd)
            with self.assertRaises(subprocess.CalledProcessError):
                build_attested_image(output, repository=root, expected_commit=COMMIT,
                    run_label=RUN, base_reference="base:pin", runner=fail_build)
            self.assertTrue((output / "commands.jsonl").is_file())
            self.assertFalse((output / "build-attestation.json").exists())

    def test_missing_docker_client_or_engine_version_cannot_issue_attestation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            for index, version in enumerate((
                {"Client": {"Version": "27.5.1"}},
                {"Client": {"Version": ""}, "Server": {"Version": "27.5.1"}},
            )):
                runner = FakeRunner(root)
                def invalid_version(args, *, cwd=None):
                    if args == ["docker", "version", "--format", "{{json .}}"]:
                        runner.commands.append(list(args))
                        return subprocess.CompletedProcess(args, 0, json.dumps(version), "")
                    return runner(args, cwd=cwd)
                output = Path(folder) / f"invalid-version-{index}"
                with self.subTest(index=index), self.assertRaisesRegex(ValueError, "versions"):
                    build_attested_image(output, repository=root, expected_commit=COMMIT,
                        run_label=RUN, base_reference="base:pin", runner=invalid_version)
                self.assertFalse((output / "build-attestation.json").exists())

    def test_current_image_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "proof"; runner = FakeRunner(root)
            build_attested_image(output, repository=root, expected_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", runner=runner)
            path = output / "build-attestation.json"
            changed = deepcopy(runner.built); changed[0]["Id"] = "sha256:" + "9" * 64
            with self.assertRaises(ValueError):
                self.validate_built(root, output, runner, built_inspect=changed,
                                    measurement_name="mutated-preflight")
            with self.assertRaisesRegex(ValueError, "receipt hash"):
                path = output / "build-attestation.json"
                validate_attestation(path,
                    expected_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    source_root=root, expected_source_commit=COMMIT, expected_run_label=RUN,
                    base_inspect=runner.base, built_inspect=runner.built,
                    source_archive=output / "source-before.tar",
                    build_receipt=output / "build-receipt.json",
                    expected_build_receipt_sha256="0" * 64,
                    build_commands=output / "commands.jsonl",
                    build_iidfile=output / "built-image.iid",
                    build_base_inspect=output / "base-image-inspect.json",
                    build_built_inspect=output / "built-image-inspect.json",
                    build_installed_measurement=output / "installed-measurement.json",
                    measurement_dir=output.parent / "wrong-pin-preflight", runner=runner)

    def test_incomplete_installed_measurement_removes_container_without_closed_proof(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "incomplete"; runner = FakeRunner(root)
            copies = 0
            def fail_copy(args, *, cwd=None):
                nonlocal copies
                if args[:2] == ["docker", "cp"]:
                    copies += 1
                    if copies == 2:
                        raise subprocess.CalledProcessError(8, args)
                return runner(args, cwd=cwd)
            with self.assertRaises(subprocess.CalledProcessError):
                build_attested_image(output, repository=root, expected_commit=COMMIT,
                    run_label=RUN, base_reference="base:pin", runner=fail_copy)
            self.assertTrue(any(command[:2] == ["docker", "rm"] for command in runner.commands))
            self.assertFalse((output / "build-attestation.json").exists())

    def test_existing_run_source_tag_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            runner = FakeRunner(root); runner.built_exists = True
            with self.assertRaisesRegex(ValueError, "already exists"):
                build_attested_image(Path(folder) / "proof", repository=root,
                    expected_commit=COMMIT, run_label=RUN, base_reference="base:pin", runner=runner)
            self.assertFalse(any(is_legacy_build(command) for command in runner.commands))

    def test_malformed_full_image_id_and_digest_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); self.source_fixture(root)
            installed = {name: "5" * 64 for name in INSTALLED_PATHS}
            runner = FakeRunner(root)
            for label, base in (
                ("id", [{"Id": "sha256:x", "RepoDigests": runner.base[0]["RepoDigests"]}]),
                ("digest", [{"Id": runner.base[0]["Id"], "RepoDigests": ["repo@sha256:x"]}]),
            ):
                with self.subTest(label=label), self.assertRaises(ValueError):
                    create_nonphysical_fixture_attestation(root / f"{label}.json", source_root=root,
                        source_commit=COMMIT, run_label=RUN, base_reference="base:pin",
                        base_inspect=base, built_reference="trace:q014",
                        built_inspect=runner.built, installed_files=installed)

    def test_fabricated_installed_hashes_fail_current_image_remeasurement(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "proof"; runner = FakeRunner(root)
            build_attested_image(output, repository=root, expected_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", runner=runner)
            attestation_path = output / "build-attestation.json"
            value = json.loads(attestation_path.read_text())
            value["installed_files"] = {name: "f" * 64 for name in INSTALLED_PATHS}
            measurement_path = output / "installed-measurement.json"
            measurement = json.loads(measurement_path.read_text())
            measurement["files"] = value["installed_files"]
            measurement_path.write_text(json.dumps(measurement, indent=2, sort_keys=True) + "\n")
            receipt_path = output / "build-receipt.json"
            receipt = json.loads(receipt_path.read_text())
            receipt["installed_measurement_sha256"] = hashlib.sha256(
                measurement_path.read_bytes()).hexdigest()
            receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
            value["build_receipt_sha256"] = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
            attestation_path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
            with self.assertRaisesRegex(ValueError, "installed bytes"):
                self.validate_built(root, output, runner, measurement_name="fabricated-preflight")

    def test_rewritten_receipt_cannot_substitute_a_fabricated_build_command_record(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "proof"; runner = FakeRunner(root)
            build_attested_image(output, repository=root, expected_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", runner=runner)
            commands_path = output / "commands.jsonl"
            rows = [json.loads(line) for line in commands_path.read_text().splitlines()]
            rows = [row for row in rows if not is_legacy_build(row["argv"])]
            commands_path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows))
            receipt_path = output / "build-receipt.json"
            receipt = json.loads(receipt_path.read_text())
            receipt["commands_sha256"] = hashlib.sha256(commands_path.read_bytes()).hexdigest()
            receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
            attestation_path = output / "build-attestation.json"
            attestation = json.loads(attestation_path.read_text())
            attestation["build_receipt_sha256"] = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
            attestation_path.write_text(json.dumps(attestation, indent=2, sort_keys=True) + "\n")
            with self.assertRaisesRegex(ValueError, "command sequence"):
                self.validate_built(root, output, runner, measurement_name="fabricated-command-preflight")

    def test_implicit_or_wrong_builder_mode_cannot_open_a_production_receipt(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "proof"; runner = FakeRunner(root)
            build_attested_image(output, repository=root, expected_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", runner=runner)
            commands_path = output / "commands.jsonl"
            original_commands = commands_path.read_bytes()
            rows = [json.loads(line) for line in commands_path.read_text().splitlines()]
            build_row = next(row for row in rows if is_legacy_build(row["argv"]))
            build_row["argv"] = build_row["argv"][2:]
            commands_path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows))
            receipt_path = output / "build-receipt.json"
            receipt = json.loads(receipt_path.read_text())
            receipt["commands_sha256"] = hashlib.sha256(commands_path.read_bytes()).hexdigest()
            receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
            attestation_path = output / "build-attestation.json"
            attestation = json.loads(attestation_path.read_text())
            attestation["build_receipt_sha256"] = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
            attestation_path.write_text(json.dumps(attestation, indent=2, sort_keys=True) + "\n")
            with self.assertRaisesRegex(ValueError, "command sequence"):
                self.validate_built(root, output, runner, measurement_name="implicit-builder-preflight")

            commands_path.write_bytes(original_commands)
            receipt["commands_sha256"] = hashlib.sha256(commands_path.read_bytes()).hexdigest()
            receipt["builder_mode"] = "implicit_default_builder"
            receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
            attestation["build_receipt_sha256"] = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
            attestation_path.write_text(json.dumps(attestation, indent=2, sort_keys=True) + "\n")
            with self.assertRaisesRegex(ValueError, "production build receipt"):
                self.validate_built(root, output, runner, measurement_name="wrong-builder-receipt")

    def test_tag_mutation_after_inspect_still_measures_attested_full_id(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "proof"; runner = FakeRunner(root)
            value = build_attested_image(output, repository=root, expected_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", runner=runner)
            inspected_before_mutation = deepcopy(runner.built)
            runner.built[0]["Id"] = "sha256:" + "9" * 64
            validated = self.validate_built(root, output, runner,
                built_inspect=inspected_before_mutation, measurement_name="race-preflight")
            self.assertEqual(value["built_image"]["id"], validated["built_image"]["id"])
            self.assertEqual(value["built_image"]["id"], [command for command in runner.commands
                if command[:2] == ["docker", "create"]][-1][-1])
