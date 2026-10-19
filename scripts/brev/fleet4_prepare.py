#!/usr/bin/env python3
"""Prepare a new four-AMR input/bootstrap using a verified readonly source tree."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
from typing import Any


EXTRA_PINS = frozenset({"scripts/brev/fleet4_prepare.py", "scripts/brev/fleet4_cloud_run.py",
                        "scripts/brev/run-fleet4.sh"})


def python_version() -> str:
    return ".".join(str(part) for part in sys.version_info[:3])


def require_python() -> None:
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError("fleet4 preparation requires Python 3.12 before project imports")


def _no_links(path: Path) -> None:
    for candidate in (path, *path.parents):
        if candidate.is_symlink() or candidate.is_junction():
            raise ValueError(f"preparation paths must not traverse a symlink or junction: {candidate}")


def _directory(path: Path, label: str) -> Path:
    if not path.is_absolute():
        raise ValueError(f"{label} must be absolute")
    _no_links(path)
    path = path.resolve(strict=True)
    if not path.is_dir():
        raise ValueError(f"{label} must be an existing regular directory")
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _copy_new(source: Path, destination: Path) -> None:
    with source.open("rb") as original, destination.open("xb") as copied:
        shutil.copyfileobj(original, copied)


def prepare(stage: Path, source_root: Path, run_id: str, source_commit: str,
            *, runtime_profile: str = "canonical62") -> dict[str, Any]:
    require_python()
    if runtime_profile not in {"canonical62", "local45_development"}:
        raise ValueError("unknown fleet runtime profile")
    if re.fullmatch(r"[0-9a-f]{32}", run_id) is None:
        raise ValueError("run_id must be lowercase 32-hex")
    if re.fullmatch(r"[0-9a-f]{40}", source_commit) is None:
        raise ValueError("source_commit must be lowercase 40-hex")
    stage = _directory(stage, "stage")
    source_root = _directory(source_root, "source_root")
    if stage.is_relative_to(source_root):
        raise ValueError("stage outputs must be outside the readonly source_root")
    for name in ("inputs", "results"):
        output = stage / name
        if output.exists() or output.is_symlink() or output.is_junction():
            raise ValueError(f"preserve existing {name}; preparation requires new output paths")
    extracted = _directory(stage / "source", "extracted source")
    for tree in {stage, source_root}:
        for path in tree.rglob("*"):
            if path.is_symlink() or path.is_junction():
                raise ValueError(f"preparation source contains a symlink or junction: {path}")
    archive, raw_commit = stage / "source.tar", stage / "source.commit"
    for path in (archive, raw_commit):
        if not path.is_file():
            raise ValueError(f"source proof requires a regular file: {path.name}")

    # Runtime-owned source imports occur only after the Python/path/output guards.
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(source_root))
    from factory_ops import fleet_transport, source_provenance
    from scripts.isaac.fleet4_transport_scene import SOURCE_PINS

    runtime_pins = SOURCE_PINS
    if runtime_profile == "local45_development":
        from scripts.isaac.local45_fleet4_transport_scene import LOCAL_SOURCE_PINS
        runtime_pins = LOCAL_SOURCE_PINS

    for module, relative in ((fleet_transport, "factory_ops/fleet_transport.py"),
                             (source_provenance, "factory_ops/source_provenance.py")):
        module_file = module.__file__
        if module_file is None or Path(module_file).resolve() != source_root / relative:
            raise ValueError("project imports must belong to the requested readonly source_root")
    names = runtime_pins | EXTRA_PINS | {
        "configs/factory-fleet4/" + name for name in fleet_transport.SOURCE_FILES}
    pins = {name: _sha(source_root / name) for name in sorted(names)}
    proof = source_provenance.verify_source_archive(archive, raw_commit, source_commit, pins,
                                                    snapshot_root=source_root)
    if proof.get("passed") is not True:
        raise ValueError(f"source archive proof failed: {proof.get('errors')}")
    if extracted != source_root:
        extracted_proof = source_provenance.verify_source_archive(archive, raw_commit, source_commit, pins,
                                                                 snapshot_root=extracted)
        if extracted_proof.get("passed") is not True:
            raise ValueError(f"extracted source archive proof failed: {extracted_proof.get('errors')}")

    # A rejected proof must never create inputs/results or a transport marker.
    prepared = fleet_transport.prepare_bundle(stage / "inputs", source_root / "configs/factory-fleet4")
    session = json.loads((stage / "inputs/session.json").read_text(encoding="utf-8"))
    results = stage / "results"
    results.mkdir()
    (results / "run").mkdir()
    provenance = results / "run/provenance"
    provenance.mkdir()
    _copy_new(archive, provenance / "source.tar")
    _copy_new(raw_commit, provenance / "source-commit.raw")
    metadata = {"schema_version": 1, "run_id": run_id, "scenario_id": session["scenario_id"],
                "source_commit": source_commit, "bundle_sha256": prepared["bundle_sha256"],
                "source_files": pins, "source_archive_sha256": proof["source_archive_sha256"],
                "source_archive_proof": proof, "preparation_python": python_version()}
    if runtime_profile != "canonical62":
        metadata["runtime_profile"] = runtime_profile
    with (provenance / "fleet-source.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return {"passed": True, "bundle_sha256": prepared["bundle_sha256"],
            "python_version": python_version(), "run_id": run_id, "source_commit": source_commit,
            "source_proof": {key: proof[key] for key in ("passed", "commit_sha1", "commit_tree_sha1",
                "archive_tree_sha1", "source_archive_sha256", "source_pin_count", "snapshot_checked")},
            "scope": "development input/bootstrap preparation only; no SDK, ROS or physical transport run"}


def main() -> int:
    try:
        require_python()
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--stage", required=True, type=Path)
        parser.add_argument("--source-root", required=True, type=Path)
        parser.add_argument("--run-id", required=True)
        parser.add_argument("--source-commit", required=True)
        parser.add_argument("--runtime-profile", choices=("canonical62", "local45_development"),
                            default="canonical62")
        args = parser.parse_args()
        report = prepare(args.stage, args.source_root, args.run_id, args.source_commit,
                         runtime_profile=args.runtime_profile)
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(json.dumps({"passed": False, "python_version": python_version(),
                          "error": f"{type(error).__name__}: {error}"}, ensure_ascii=False))
        return 1
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
