#!/usr/bin/env python3
"""Four-AMR development scene; preserve the separate G3 qualification entry."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.fleet_transport import QUALIFICATION, load_bundle, transport_verification_record
from factory_ops.source_provenance import verify_source_archive
from scripts.isaac import g2_scene
from scripts.isaac.g3_scene import BarrierLifecycle


RESULT_FILE = "fleet4-transport-scene.json"
SOURCE_PINS = frozenset({"factory_ops/fleet_transport.py", "factory_ops/fleet_registry.py",
    "factory_ops/fleet_routing.py", "factory_ops/layout.py", "factory_ops/source_provenance.py",
    "scripts/isaac/fleet4_transport_scene.py", "scripts/isaac/g2_scene.py", "scripts/isaac/g3_scene.py",
    "scripts/isaac/factory_visuals.py", "scripts/ros/g2_mission.py", "scripts/ros/g3_mission.py",
    "scripts/ros/fleet4_mission.py", "scripts/ros/fleet4.launch.py"})
BOOTSTRAP_FILES = frozenset({"fleet-transport-verified.json", "provenance/source.tar",
                           "provenance/source-commit.raw", "provenance/fleet-source.json"})


class FleetTransportLifecycle(BarrierLifecycle):
    def __init__(self, root: Path, run_id: str) -> None:
        self.compiled = load_bundle(root)
        self.registry = self.compiled["registry"]
        super().__init__(self.compiled["session"], run_id, root,
                         robot_ids=self.registry.robot_ids, config_directory=root)

    def prepare_worker_paths(self, scene_arg: Path, output_arg: Path) -> tuple[Path, Path]:
        if (not scene_arg.is_absolute() or not output_arg.is_absolute()
                or scene_arg.is_symlink() or output_arg.is_symlink()):
            raise ValueError("transport runtime paths must be absolute and regular")
        scene, output = scene_arg.resolve(strict=True), output_arg.resolve(strict=True)
        if scene != (self.session_root / "scene/factory-v3.usda").resolve(strict=True) or not output.is_dir():
            raise ValueError("transport scene/output differs from the verified bundle")
        files, directories = set(), set()
        for path in output.rglob("*"):
            if path.is_symlink() or not (path.is_file() or path.is_dir()):
                raise ValueError("transport bootstrap contains a symlink or special file")
            if path.is_file():
                files.add(path.relative_to(output).as_posix())
            else:
                directories.add(path.relative_to(output).as_posix())
        if files != BOOTSTRAP_FILES or directories != {"provenance"}:
            raise ValueError("transport bootstrap exact file set differs; use a new attempt")
        expected_marker = transport_verification_record(run_id=self.run_id, session=self.session,
            registry=self.registry, bundle_sha256=self.compiled["bundle_sha256"])
        marker = json.loads((output / "fleet-transport-verified.json").read_text(encoding="utf-8"))
        if marker != expected_marker:
            raise ValueError("transport marker differs from the run, bundle or source identities")
        metadata_path = output / "provenance/fleet-source.json"
        expected_sha = os.environ.get("FACTORY_OPS_FLEET4_PROVENANCE_SHA256", "")
        if re.fullmatch(r"[0-9a-f]{64}", expected_sha) is None or g2_scene.base._sha256(metadata_path) != expected_sha:
            raise ValueError("transport source metadata is not bound to the supervisor")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if any(metadata.get(key) != expected_marker[key]
               for key in ("run_id", "scenario_id", "source_commit", "bundle_sha256")):
            raise ValueError("transport source metadata identity differs")
        source_files = metadata.get("source_files")
        if not isinstance(source_files, dict) or not SOURCE_PINS.issubset(source_files):
            raise ValueError("transport required source pins are missing")
        proof = verify_source_archive(output / "provenance/source.tar", output / "provenance/source-commit.raw",
            expected_marker["source_commit"], source_files, snapshot_root=Path(__file__).resolve().parents[2])
        if (proof.get("passed") is not True
                or proof.get("source_archive_sha256") != metadata.get("source_archive_sha256")):
            raise ValueError(f"transport source archive proof differs: {proof.get('errors')}")
        return scene, output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    root = args.bundle.resolve(strict=True)
    compiled = load_bundle(root)
    scenario_id = compiled["session"]["scenario_id"]
    shared = argparse.Namespace(scene=root / "scene/factory-v3.usda", layout=root / "layout.json",
        config=root / "config.json", scenarios=root / "scenarios.json", scenario=scenario_id,
        output=args.output, run_id=args.run_id)
    return g2_scene._run(shared, qualification=QUALIFICATION, scenario_ids={scenario_id},
        result_file=RESULT_FILE, expected_robot_count=4,
        scene_extension=FleetTransportLifecycle(root, args.run_id),
        extra_inputs={"fleet_bundle": root / "bundle.json", "fleet_worker_source": Path(__file__).resolve()})


if __name__ == "__main__":
    raise SystemExit(main())
