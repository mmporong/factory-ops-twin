"""CPU-only ownership fixtures for the owned disk watcher."""
from __future__ import annotations

import ast
from pathlib import Path
import re
import unittest


SHELL = (Path(__file__).resolve().parents[1] / "scripts" / "brev" / "run-cosmos-smoke.sh").read_text(encoding="utf-8")
BLOCK = next(
    value for value in re.findall(r"<<'PY'\n(.*?)\nPY", SHELL, re.S)
    if "def classify_parent_identity" in value
)
TREE = ast.parse(BLOCK)
FUNCTION = next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef) and node.name == "classify_parent_identity"
)
namespace = {"pathlib": __import__("pathlib")}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), "<disk-guard>", "exec"), namespace)
classify_parent_identity = namespace["classify_parent_identity"]


def expected() -> dict:
    return {
        "parent_pid": 200,
        "parent_starttime": "1000",
        "pgid": 150,
        "leader_starttime": "900",
        "run_script": "/home/ubuntu/workspace/factoryops-cosmos-386/run.sh",
        "batch_id": "386",
        "target_profile": "scale234",
    }


def observed() -> dict:
    return {
        "parent_pid": 200,
        "parent_starttime": "1000",
        "pgid": 150,
        "leader_pid": 150,
        "leader_starttime": "900",
        "leader_argv": [
            "/usr/bin/timeout", "--signal=TERM", "6600s", "/bin/bash",
            "/home/ubuntu/workspace/factoryops-cosmos-386/run.sh", "386", "scale234",
        ],
    }


class DiskGuardOwnershipTests(unittest.TestCase):
    def test_owned_timeout_group_is_admitted(self) -> None:
        self.assertEqual(
            classify_parent_identity(expected(), observed()),
            (True, True, "owned_timeout_group"),
        )

    def test_unrelated_group_is_rejected_without_rejecting_parent_identity(self) -> None:
        value = observed()
        value["leader_argv"] = ["/usr/bin/python3", "unrelated.py"]
        owned, parent_same, reason = classify_parent_identity(expected(), value)
        self.assertFalse(owned)
        self.assertTrue(parent_same)
        self.assertEqual(reason, "group_leader_is_not_gnu_timeout")

    def test_stale_parent_is_rejected_as_unsafe_to_signal(self) -> None:
        value = observed()
        value["parent_starttime"] = "replacement-process"
        owned, parent_same, reason = classify_parent_identity(expected(), value)
        self.assertFalse(owned)
        self.assertFalse(parent_same)
        self.assertEqual(reason, "parent_pid_or_starttime_changed")


if __name__ == "__main__":
    unittest.main()
