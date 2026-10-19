"""Exercise preparation against real temporary Git archive/commit objects."""
from __future__ import annotations

import ast
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
TOOL = "scripts/brev/fleet4_prepare.py"
RUN_ID = "a" * 32


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _files(root: Path) -> dict[str, str]:
    return {p.relative_to(root).as_posix(): _sha(p) for p in root.rglob("*") if p.is_file()}


class Fleet4PreparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if shutil.which("git") is None:
            raise unittest.SkipTest("temporary source proof fixtures require Git")
        cls.temporary = tempfile.TemporaryDirectory(prefix="fleet4-prepare-fixture-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.base = Path(cls.temporary.name).resolve()
        cls.repo = cls.base / "fixture-repo"
        cls.repo.mkdir()
        cls.git_env = {**os.environ, "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1",
                       "GIT_TERMINAL_PROMPT": "0"}
        for directory in ("factory_ops", "scripts/isaac", "scripts/ros", "scripts/brev"):
            for original in (PROJECT / directory).rglob("*.py"):
                destination = cls.repo / original.relative_to(PROJECT)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(original, destination)
        shutil.copyfile(PROJECT / "scripts/brev/run-fleet4.sh", cls.repo / "scripts/brev/run-fleet4.sh")
        shutil.copytree(PROJECT / "configs/factory-fleet4", cls.repo / "configs/factory-fleet4")
        cls._git("init", "--initial-branch=main")
        cls._git("config", "user.name", "Temporary Source Fixture")
        cls._git("config", "user.email", "fixture@example.invalid")
        cls._git("config", "core.autocrlf", "false")
        cls._git("config", "core.hooksPath", str(cls.base / "no-hooks"))
        cls._git("add", "--", *sorted(name for name in _files(cls.repo) if not name.startswith(".git/")))
        cls._git("-c", "commit.gpgsign=false", "commit", "-m", "test(fleet4): 준비 도구 검증용 임시 소스")
        cls.commit = cls._git("rev-parse", "HEAD").decode("ascii").strip()
        cls.archive = cls.base / "source.tar"
        cls._git("archive", "--format=tar", "--output", str(cls.archive), cls.commit)
        cls.raw_commit = cls.base / "source.commit"
        cls.raw_commit.write_bytes(cls._git("cat-file", "commit", cls.commit))

    @classmethod
    def _git(cls, *arguments: str) -> bytes:
        assert cls.repo != PROJECT and cls.repo.is_relative_to(cls.base)
        return subprocess.check_output(["git", *arguments], cwd=cls.repo, env=cls.git_env,
                                       stderr=subprocess.PIPE)

    def setUp(self) -> None:
        self.case = self.base / self._testMethodName
        self.case.mkdir()
        self.stage = self.case / "stage"
        self.stage.mkdir()
        shutil.copyfile(self.archive, self.stage / "source.tar")
        shutil.copyfile(self.raw_commit, self.stage / "source.commit")
        self.source = self.stage / "source"
        self.source.mkdir()
        with tarfile.open(self.archive) as archive:
            archive.extractall(self.source, filter="data")

    def cli(self, *, source: Path | None = None, prefix: list[str] | None = None,
            stage: Path | None = None, run_id: str = RUN_ID,
            runtime_profile: str | None = None) -> subprocess.CompletedProcess[str]:
        mounted = source or self.source
        args = [str(mounted / TOOL), "--stage", str(stage or self.stage), "--source-root", str(mounted),
                "--run-id", run_id, "--source-commit", self.commit]
        if runtime_profile is not None:
            args.extend(["--runtime-profile", runtime_profile])
        return subprocess.run([sys.executable, *(prefix or []), *args], cwd=self.case,
                              capture_output=True, text=True, encoding="utf-8", timeout=120,
                              env={**os.environ, "PYTHONUTF8": "1"})

    def assert_rejected(self, reply: subprocess.CompletedProcess[str]) -> dict:
        self.assertEqual(1, reply.returncode, reply.stdout + reply.stderr)
        result = json.loads(reply.stdout)
        self.assertIs(result["passed"], False)
        return result

    def assert_no_outputs(self) -> None:
        self.assertFalse((self.stage / "inputs").exists())
        self.assertFalse((self.stage / "results").exists())

    def test_actual_archive_positive_binds_distinct_mount_and_extracted_tree(self) -> None:
        mounted = self.case / "readonly-source"
        shutil.copytree(self.source, mounted)
        originals = (_files(self.source), _files(mounted), _sha(self.stage / "source.tar"),
                     _sha(self.stage / "source.commit"))
        reply = self.cli(source=mounted)
        self.assertEqual(0, reply.returncode, reply.stdout + reply.stderr)
        report = json.loads(reply.stdout)
        self.assertIs(report["passed"], True)
        proof = report["source_proof"]
        self.assertEqual(self.commit, proof["commit_sha1"])
        self.assertEqual(proof["commit_tree_sha1"], proof["archive_tree_sha1"])
        self.assertTrue(proof["snapshot_checked"])
        metadata = json.loads((self.stage / "results/run/provenance/fleet-source.json").read_text())
        self.assertEqual(RUN_ID, metadata["run_id"])
        self.assertEqual(self.commit, metadata["source_commit"])
        self.assertEqual(_sha(self.stage / "inputs/bundle.json"), metadata["bundle_sha256"])
        self.assertEqual(report["bundle_sha256"], metadata["bundle_sha256"])
        self.assertEqual(report["python_version"], metadata["preparation_python"])
        self.assertEqual(_sha(self.archive), metadata["source_archive_sha256"])
        for name in (TOOL, "scripts/brev/fleet4_cloud_run.py", "scripts/brev/run-fleet4.sh",
                     "scripts/ros/fleet4_mission.py", "configs/factory-fleet4/layout.json"):
            self.assertEqual(_sha(mounted / name), metadata["source_files"][name])
        result_files = {p.relative_to(self.stage / "results/run").as_posix()
                        for p in (self.stage / "results/run").rglob("*") if p.is_file()}
        self.assertEqual({"provenance/source.tar", "provenance/source-commit.raw",
                          "provenance/fleet-source.json"}, result_files)
        self.assertEqual(_sha(self.raw_commit), _sha(self.stage / "results/run/provenance/source-commit.raw"))
        self.assertEqual(originals, (_files(self.source), _files(mounted), _sha(self.stage / "source.tar"),
                                     _sha(self.stage / "source.commit")))
        session = json.loads((self.stage / "inputs/session.json").read_text())
        self.assertEqual(session["scenario_id"], metadata["scenario_id"])
        self.assertFalse(session["primary_evaluation"])

    def test_unsupported_python_guard_rejects_before_project_import_or_outputs(self) -> None:
        # The module must also parse on 3.10, before the runtime rejection.
        ast.parse((PROJECT / TOOL).read_text(encoding="utf-8"), feature_version=(3, 10))
        shim = """import builtins,runpy,sys
original=builtins.__import__
def guarded(name,*args,**kwargs):
 if name.startswith(('factory_ops','scripts.isaac')):raise AssertionError('PROJECT_IMPORT_BEFORE_GUARD')
 return original(name,*args,**kwargs)
builtins.__import__=guarded
sys.version_info=VERSION
sys.argv=sys.argv[1:]
runpy.run_path(sys.argv[0],run_name='__main__')
"""
        for version in ("(3,10,14,'final',0)", "(3,13,0,'final',0)"):
            with self.subTest(version=version):
                result = self.assert_rejected(self.cli(prefix=["-c", shim.replace("VERSION", version)]))
                self.assertIn("Python 3.12", result["error"])
                self.assertNotIn("PROJECT_IMPORT_BEFORE_GUARD", result["error"])
        self.assert_no_outputs()

    def test_local45_profile_binds_its_worker_and_observer_to_the_archive(self) -> None:
        reply = self.cli(runtime_profile="local45_development")
        self.assertEqual(0, reply.returncode, reply.stdout + reply.stderr)
        metadata = json.loads((self.stage / "results/run/provenance/fleet-source.json").read_text())
        self.assertEqual("local45_development", metadata["runtime_profile"])
        for name in ("scripts/isaac/local45_fleet4_transport_scene.py",
                     "factory_ops/isaac45_runtime.py", "factory_ops/flat_scan_observation.py"):
            self.assertEqual(_sha(self.source / name), metadata["source_files"][name])

    def test_local45_source_change_rejects_before_preparing(self) -> None:
        with (self.source / "factory_ops/flat_scan_observation.py").open("ab") as stream:
            stream.write(b"\n# changed native observer\n")
        result = self.assert_rejected(self.cli(runtime_profile="local45_development"))
        self.assertIn("source archive proof failed", result["error"])
        self.assert_no_outputs()

    def test_mounted_source_change_is_rejected_before_preparing(self) -> None:
        with (self.source / "scripts/ros/fleet4_mission.py").open("ab") as stream:
            stream.write(b"\n# changed mounted source\n")
        result = self.assert_rejected(self.cli())
        self.assertIn("source archive proof failed", result["error"])
        self.assert_no_outputs()

    def test_consumed_configuration_change_is_also_bound_to_archive(self) -> None:
        with (self.source / "configs/factory-fleet4/layout.json").open("ab") as stream:
            stream.write(b"\n ")
        result = self.assert_rejected(self.cli())
        self.assertIn("source archive proof failed", result["error"])
        self.assert_no_outputs()

    def test_separate_stage_source_change_is_rejected(self) -> None:
        mounted = self.case / "readonly-source"
        shutil.copytree(self.source, mounted)
        with (self.source / "scripts/brev/run-fleet4.sh").open("ab") as stream:
            stream.write(b"\n# changed extracted stage\n")
        result = self.assert_rejected(self.cli(source=mounted))
        self.assertIn("extracted source archive proof failed", result["error"])
        self.assert_no_outputs()

    def test_invalid_raw_commit_is_rejected_and_original_is_preserved(self) -> None:
        original = (self.stage / "source.commit").read_bytes() + b"invalid"
        (self.stage / "source.commit").write_bytes(original)
        result = self.assert_rejected(self.cli())
        self.assertIn("source archive proof failed", result["error"])
        self.assertEqual(original, (self.stage / "source.commit").read_bytes())
        self.assert_no_outputs()

    def test_existing_inputs_and_results_are_never_reused(self) -> None:
        for name in ("inputs", "results"):
            with self.subTest(name=name):
                target = self.stage / name
                target.mkdir()
                (target / "original.txt").write_text("preserve")
                result = self.assert_rejected(self.cli())
                self.assertIn("preserve existing", result["error"])
                self.assertEqual("preserve", (target / "original.txt").read_text())
                # Only remove the fixture created by this subtest.
                (target / "original.txt").unlink()
                target.rmdir()

    def test_relative_paths_and_invalid_run_identity_create_no_outputs(self) -> None:
        for reply in (self.cli(stage=Path("stage")), self.cli(run_id="invalid")):
            self.assert_rejected(reply)
        self.assert_no_outputs()

    def test_symlinked_or_junction_source_is_rejected(self) -> None:
        link = self.case / "source-link"
        try:
            link.symlink_to(self.source, target_is_directory=True)
        except OSError as error:
            if os.name != "nt":
                self.skipTest(f"host cannot create a temporary symlink: {error}")
            # Junction creation needs no symlink privilege on Windows. Both targets
            # belong to this test's TemporaryDirectory, never the product tree.
            self.assertTrue(link.is_relative_to(self.base) and self.source.is_relative_to(self.base))
            reply = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command",
                "$ErrorActionPreference='Stop'; New-Item -ItemType Junction -Path $env:FLEET_TEST_LINK -Target $env:FLEET_TEST_SOURCE | Out-Null"],
                env={**os.environ, "FLEET_TEST_LINK": str(link), "FLEET_TEST_SOURCE": str(self.source)},
                capture_output=True, text=True, timeout=20)
            self.assertEqual(0, reply.returncode, reply.stderr)
            self.assertTrue(link.is_junction())
        result = self.assert_rejected(self.cli(source=link))
        self.assertIn("symlink", result["error"])
        self.assert_no_outputs()


if __name__ == "__main__":
    unittest.main()
