from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest

from factory_ops.g3_build_attestation import (
    INSTALLED_PATHS, SOURCE_PATHS, build_attested_image, create_nonphysical_fixture_attestation,
    validate_attestation,
)

COMMIT = "a" * 40
RUN = "g3-q-normal-fifo-014"

class FakeRunner:
    def __init__(self, repository: Path):
        self.repository = repository
        self.commands = []
        self.base = [{"Id": "sha256:" + "1" * 64, "RepoDigests": ["base@sha256:" + "2" * 64]}]
        self.built = [{"Id": "sha256:" + "3" * 64, "RepoDigests": ["built@sha256:" + "4" * 64]}]

    def __call__(self, args, *, cwd=None):
        self.commands.append(list(args))
        stdout = ""
        if args[:3] == ["git", "rev-parse", "HEAD"]:
            stdout = COMMIT + "\n"
        elif args[:3] == ["git", "status", "--porcelain"]:
            stdout = ""
        elif args[:2] == ["git", "archive"]:
            output = Path(next(item.split("=", 1)[1] for item in args if item.startswith("--output=")))
            with tarfile.open(output, "w") as archive:
                for path in sorted(self.repository.rglob("*")):
                    if path.is_file():
                        info = archive.gettarinfo(str(path), path.relative_to(self.repository).as_posix())
                        info.mtime = 0
                        with path.open("rb") as handle:
                            archive.addfile(info, handle)
        elif args[:3] == ["docker", "image", "inspect"]:
            stdout = json.dumps(self.base if args[3] == "base:pin" else self.built)
        elif args[:2] == ["docker", "build"]:
            Path(args[args.index("--iidfile") + 1]).write_text(self.built[0]["Id"])
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

    def test_closed_build_orders_archive_pinned_base_build_measurement_and_recheck(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "build-proof"; runner = FakeRunner(root)
            value = build_attested_image(output, repository=root, expected_commit=COMMIT,
                run_label=RUN, base_reference="base:pin", runner=runner)
            self.assertEqual("built", value["status"])
            self.assertTrue(value["runtime_eligible"])
            self.assertEqual(value["base_image"]["id"], value["build_proof"]["base_image_arg"])
            build = next(command for command in runner.commands if command[:2] == ["docker", "build"])
            self.assertIn(f"BASE_IMAGE={value['base_image']['id']}", build)
            self.assertIn("--iidfile", build)
            self.assertEqual(2, sum(command[:2] == ["git", "archive"] for command in runner.commands))
            self.assertEqual(3, sum(command[:2] == ["docker", "cp"] for command in runner.commands))
            path = output / "build-attestation.json"
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(value, validate_attestation(path, expected_sha256=digest,
                source_root=root, expected_source_commit=COMMIT, expected_run_label=RUN,
                base_inspect=runner.base, built_inspect=runner.built,
                source_archive=output / "source-before.tar"))
            (root / SOURCE_PATHS[-1]).write_text("stale source")
            with self.assertRaises(ValueError):
                validate_attestation(path, expected_sha256=digest, source_root=root,
                    expected_source_commit=COMMIT, expected_run_label=RUN,
                    base_inspect=runner.base, built_inspect=runner.built,
                    source_archive=output / "source-before.tar")

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
                    source_archive=root / "missing.tar")

    def test_failed_build_preserves_commands_but_issues_no_closed_attestation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / "repo"; root.mkdir(); self.source_fixture(root)
            output = Path(folder) / "failed"; runner = FakeRunner(root)
            def fail_build(args, *, cwd=None):
                if args[:2] == ["docker", "build"]:
                    raise subprocess.CalledProcessError(7, args)
                return runner(args, cwd=cwd)
            with self.assertRaises(subprocess.CalledProcessError):
                build_attested_image(output, repository=root, expected_commit=COMMIT,
                    run_label=RUN, base_reference="base:pin", runner=fail_build)
            self.assertTrue((output / "commands.jsonl").is_file())
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
                validate_attestation(path, expected_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    source_root=root, expected_source_commit=COMMIT, expected_run_label=RUN,
                    base_inspect=runner.base, built_inspect=changed,
                    source_archive=output / "source-before.tar")

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
