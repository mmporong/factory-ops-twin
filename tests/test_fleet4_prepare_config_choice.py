"""Selected frozen inputs with synthetic content-addressed archives; no Git command."""
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from uuid import uuid4

from factory_ops.fleet import FleetMission, FleetOrder
from factory_ops.fleet_transport import SOURCE_FILES, load_bundle
from factory_ops.material_flow import MaterialFlow
from factory_ops.navigation import GoalPose
from scripts.isaac.local45_fleet4_transport_scene import LOCAL_SOURCE_PINS


PROJECT = Path(__file__).resolve().parents[1]
TOOL = "scripts/brev/fleet4_prepare.py"
DEFAULT = "configs/factory-fleet4"
SELECTED = "configs/factory-fleet4-normal3"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def object_id(kind, data):
    return hashlib.sha1(kind+b" "+str(len(data)).encode("ascii")+b"\0"+data).digest()


def write_archive(source, archive, commit_file):
    """Independent tiny fixture encoder; use real verifier, not a mocked proof."""
    tree = {}
    with tarfile.open(archive, "w") as tar:
        for path in sorted(p for p in source.rglob("*") if p.is_file()):
            relative, data = path.relative_to(source), path.read_bytes()
            member = tarfile.TarInfo(relative.as_posix())
            member.size, member.mode = len(data), 0o644
            tar.addfile(member, io.BytesIO(data))
            node = tree
            for part in relative.parts[:-1]:
                node = node.setdefault(part, {})
            node[relative.name] = object_id(b"blob", data)

    def encode(node):
        body = b""
        for name, value in sorted(node.items(), key=lambda row: row[0].encode()+ (b"/" if isinstance(row[1], dict) else b"")):
            mode, digest = (b"40000", encode(value)) if isinstance(value, dict) else (b"100644", value)
            body += mode+b" "+name.encode()+b"\0"+digest
        return object_id(b"tree", body)

    raw = (b"tree "+encode(tree).hex().encode()+b"\nauthor CPU Fixture <fixture@example.invalid> 0 +0000\n"
           b"committer CPU Fixture <fixture@example.invalid> 0 +0000\n\nSynthetic CPU fixture, not producer Git evidence.\n")
    commit_file.write_bytes(raw)
    return object_id(b"commit", raw).hex()


class FrozenConfigChoiceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        supplied = os.environ.get("FACTORY_OPS_CONFIG_CHOICE_EVIDENCE")
        cls.base = Path(supplied) if supplied else Path(tempfile.mkdtemp(prefix="factory-ops-frozen-config-082-"))
        if supplied:
            if not cls.base.is_absolute() or cls.base.exists():
                raise ValueError("new absolute fixture evidence directory required")
            cls.base.mkdir(parents=True)
        cls.seed = cls.base/"seed"
        cls.seed.mkdir()
        # Reuse the established fixture's bounded Python/config copy pattern, without its Git setup.
        for directory in ("factory_ops", "scripts/isaac", "scripts/ros", "scripts/brev"):
            for original in (PROJECT/directory).rglob("*.py"):
                target = cls.seed/original.relative_to(PROJECT)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(original, target)
        shutil.copyfile(PROJECT/"scripts/brev/run-fleet4.sh", cls.seed/"scripts/brev/run-fleet4.sh")
        for directory in (DEFAULT, SELECTED):
            (cls.seed/directory).mkdir(parents=True)
            for name in SOURCE_FILES:
                shutil.copyfile(PROJECT/directory/name, cls.seed/directory/name)
        cls.archive, cls.raw_commit = cls.base/"source.tar", cls.base/"source.commit"
        cls.commit = write_archive(cls.seed, cls.archive, cls.raw_commit)
        print(json.dumps({"config_choice_evidence": str(cls.base), "synthetic_fixture_commit": cls.commit,
                          "source_archive_sha256": sha(cls.archive), "Git_commands": False}))

    def case(self, name):
        stage = self.base/name
        stage.mkdir()
        shutil.copyfile(self.archive, stage/"source.tar")
        shutil.copyfile(self.raw_commit, stage/"source.commit")
        shutil.copytree(self.seed, stage/"source")
        return stage

    def cli(self, stage, selected=None):
        command = [sys.executable, "-B", str(stage/"source"/TOOL), "--stage", str(stage),
            "--source-root", str(stage/"source"), "--source-commit", self.commit,
            "--run-id", uuid4().hex, "--runtime-profile", "local45_development"]
        if selected is not None:
            command += ["--config-dir", str(selected)]
        reply = subprocess.run(command, cwd=stage, capture_output=True, text=True,
            encoding="utf-8", timeout=90, env={**os.environ, "PYTHONUTF8": "1", "PYTHONDONTWRITEBYTECODE": "1"})
        (stage/"prepare.stdout.log").write_text(reply.stdout, encoding="utf-8")
        (stage/"prepare.stderr.log").write_text(reply.stderr, encoding="utf-8")
        return reply, json.loads(reply.stdout)

    def test_named_input_only_changes_three_orders_and_corresponding_materials(self):
        self.assertEqual({p.name for p in (PROJECT/SELECTED).iterdir()}, set(SOURCE_FILES))
        for name in SOURCE_FILES-{"transport-profile.json"}:
            self.assertEqual((PROJECT/DEFAULT/name).read_bytes(), (PROJECT/SELECTED/name).read_bytes(), name)
        original = json.loads((PROJECT/DEFAULT/"transport-profile.json").read_text(encoding="utf-8"))
        expected = deepcopy(original)
        expected["orders"] = expected["orders"][:3]
        expected["material_flow"]["materials"] = expected["material_flow"]["materials"][:3]
        chosen = json.loads((PROJECT/SELECTED/"transport-profile.json").read_text(encoding="utf-8"))
        self.assertEqual(chosen, expected)
        self.assertEqual(len(original["orders"]), 4)
        self.assertEqual(len(chosen["initial_robots"]), 4)

    def test_default_full_stage_remains_four_orders(self):
        stage = self.case("default4")
        reply, result = self.cli(stage)
        self.assertEqual(reply.returncode, 0, reply.stdout+reply.stderr)
        self.assertTrue(result["passed"])
        self.assertEqual(result["config_directory"], DEFAULT)
        compiled = load_bundle(stage/"inputs")
        self.assertEqual(len(compiled["session"]["workload"]["orders"]), 4)
        self.assertEqual(len(compiled["registry"].robot_ids), 4)
        self.assertFalse(compiled["session"]["primary_evaluation"])

    def test_selected_full_stage_binds_all_pins_loads_three_orders_and_conserves_stock(self):
        stage = self.case("selected3")
        reply, result = self.cli(stage, SELECTED)
        self.assertEqual(reply.returncode, 0, reply.stdout+reply.stderr)
        self.assertTrue(result["passed"])
        self.assertEqual(result["config_directory"], SELECTED)
        self.assertEqual(result["source_proof"]["commit_tree_sha1"], result["source_proof"]["archive_tree_sha1"])
        self.assertTrue(result["source_proof"]["snapshot_checked"])
        metadata = json.loads((stage/"results/run/provenance/fleet-source.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["config_directory"], SELECTED)
        expected_pins = LOCAL_SOURCE_PINS | {directory+"/"+name for directory in (DEFAULT, SELECTED) for name in SOURCE_FILES}
        self.assertTrue(expected_pins.issubset(metadata["source_files"]))
        for name in expected_pins:
            self.assertEqual(metadata["source_files"][name], sha(stage/"source"/name), name)
        compiled = load_bundle(stage/"inputs")
        workload, book = compiled["session"]["workload"], compiled["book"]
        ids = {row["order_id"] for row in workload["orders"]}
        self.assertEqual(ids, {"F4-001", "F4-002", "F4-003"})
        self.assertEqual(ids, {row["order_id"] for row in workload["material_flow"]["materials"]})
        self.assertEqual(len(compiled["registry"].robot_ids), 4)
        self.assertEqual(workload["horizon_s"], 1800)
        self.assertEqual(compiled["config"]["max_sim_time_s"], 2100)
        mission = FleetMission("CPU-config-082", [FleetOrder(**row) for row in workload["orders"]],
            robot_stations=workload["initial_robots"], station_goals={key: GoalPose("map", *xy, 0.)
                for key, xy in {**book.layout.stations, **book.layout.pockets}.items()})
        inventory = MaterialFlow(mission, workload["material_flow"]).snapshot()["inventory"]
        self.assertEqual(sum(row["on_hand"] for row in inventory["stock"]), 60)
        self.assertEqual(sum(row["reserved"] for row in inventory["stock"]), 6)
        self.assertTrue(all(row["balanced"] for row in inventory["conservation"]))
        (stage/"material-preflight.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
        copied = stage/"tampered-bundle"
        shutil.copytree(stage/"inputs", copied)
        with (copied/"transport-profile.json").open("ab") as stream:
            stream.write(b"\n ")
        with self.assertRaisesRegex(ValueError, "input changed: transport-profile.json"):
            load_bundle(copied)

    def test_external_missing_unpinned_and_tampered_selection_reject_before_outputs(self):
        for kind in ("external", "missing", "missing-file", "not-in-archive", "selected-tampered", "default-tampered"):
            with self.subTest(kind=kind):
                stage = self.case(kind)
                chosen = stage/"source"/SELECTED
                if kind == "external":
                    outside = stage/"external-config"
                    shutil.copytree(chosen, outside)
                    chosen = outside
                elif kind == "missing":
                    chosen = stage/"source/configs/missing"
                elif kind == "missing-file":
                    (chosen/"energy-profile.json").unlink()
                elif kind == "not-in-archive":
                    unpinned = stage/"source/configs/not-in-archive"
                    shutil.copytree(chosen, unpinned)
                    chosen = unpinned
                elif kind in ("selected-tampered", "default-tampered"):
                    changed = chosen if kind == "selected-tampered" else stage/"source"/DEFAULT
                    with (changed/"transport-profile.json").open("ab") as stream:
                        stream.write(b"\n ")
                reply, result = self.cli(stage, chosen)
                self.assertEqual(reply.returncode, 1, reply.stdout+reply.stderr)
                self.assertFalse(result["passed"])
                self.assertFalse((stage/"inputs").exists())
                self.assertFalse((stage/"results").exists())
                if kind in ("not-in-archive", "selected-tampered", "default-tampered"):
                    self.assertIn("source archive proof failed", result["error"])


if __name__ == "__main__":
    unittest.main()
