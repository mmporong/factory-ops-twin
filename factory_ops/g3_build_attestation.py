"""Fail-closed source-to-container attestation for the G3 trace overlay."""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import tarfile
from pathlib import Path
from typing import Any, Mapping

SHA256 = re.compile(r"^[0-9a-f]{64}$")
IMAGE_ID = re.compile(r"^sha256:[0-9a-f]{64}$")
REPO_DIGEST = re.compile(r"^[A-Za-z0-9._/:+-]+@sha256:[0-9a-f]{64}$")
SOURCE_PATHS = (
    "scripts/ros/Dockerfile.g3",
    "ros_ws/src/factory_ops_goal_checker/CMakeLists.txt",
    "ros_ws/src/factory_ops_goal_checker/package.xml",
    "ros_ws/src/factory_ops_goal_checker/plugins.xml",
    "ros_ws/src/factory_ops_goal_checker/include/factory_ops_goal_checker/trace_contract.hpp",
    "ros_ws/src/factory_ops_goal_checker/include/factory_ops_goal_checker/telemetry_goal_checker.hpp",
    "ros_ws/src/factory_ops_goal_checker/include/factory_ops_goal_checker/telemetry_regulated_pure_pursuit_controller.hpp",
    "ros_ws/src/factory_ops_goal_checker/src/telemetry_goal_checker.cpp",
    "ros_ws/src/factory_ops_goal_checker/src/telemetry_regulated_pure_pursuit_controller.cpp",
    "ros_ws/src/factory_ops_goal_checker/test/test_plugin_loading.cpp",
    "ros_ws/src/factory_ops_goal_checker/test/test_telemetry_goal_checker.cpp",
    "ros_ws/src/factory_ops_goal_checker/test/test_telemetry_rpp.cpp",
)
INSTALLED_PATHS = (
    "/opt/factory_ops_goal_checker/share/factory_ops_goal_checker/package.xml",
    "/opt/factory_ops_goal_checker/share/factory_ops_goal_checker/plugins.xml",
    "/opt/factory_ops_goal_checker/lib/libfactory_ops_goal_checker.so",
)

def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()

def canonical_sha(value: Any) -> str:
    return sha256_bytes(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                   ensure_ascii=True).encode())

def _write_fresh_json(output: Path, value: Mapping[str, Any]) -> None:
    if output.exists() or output.is_symlink():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    with temporary.open("x", encoding="utf-8", newline="\n") as handle:
        handle.write(json.dumps(value, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    try:
        os.link(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)

def source_set(root: Path) -> dict[str, str]:
    root = root.resolve(strict=True)
    package_root = root / "ros_ws/src/factory_ops_goal_checker"
    expected_package = {name for name in SOURCE_PATHS if name.startswith(
        "ros_ws/src/factory_ops_goal_checker/")}
    discovered_package = {path.relative_to(root).as_posix() for path in package_root.rglob("*")
                          if path.is_file() or path.is_symlink()}
    if discovered_package != expected_package:
        raise ValueError("G3 plugin Docker build source set differs")
    result = {}
    for relative in SOURCE_PATHS:
        path = root / relative
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"attested source is not a regular file: {relative}")
        result[relative] = sha256_bytes(path.read_bytes())
    return result

def create_nonphysical_fixture_attestation(
    output: Path, *, source_root: Path, source_commit: str, run_label: str,
    base_reference: str, base_inspect: Any, built_reference: str, built_inspect: Any,
    installed_files: Mapping[str, str],
) -> dict[str, Any]:
    sources = source_set(source_root)
    if (not re.fullmatch(r"[0-9a-f]{40}", source_commit)
            or not re.fullmatch(r"g3-[qpm]-(normal|closure|burst)-(fifo|eta)-[0-9]{3}", run_label)
            or set(installed_files) != set(INSTALLED_PATHS)
            or not all(isinstance(value, str) and SHA256.fullmatch(value)
                       for value in installed_files.values())):
        raise ValueError("build attestation creation inputs differ")
    value = {
        "schema_version": 1, "status": "nonphysical_fixture", "source_commit": source_commit,
        "run_label": run_label, "source_files": sources,
        "source_set_sha256": canonical_sha(sources),
        "dockerfile_sha256": sources["scripts/ros/Dockerfile.g3"],
        "base_image": {"reference": base_reference, **_image(base_inspect, "base_image")},
        "built_image": {"reference": built_reference, **_image(built_inspect, "built_image")},
        "installed_files": dict(sorted(installed_files.items())),
        "build_proof": None, "build_completed": False, "runtime_eligible": False,
    }
    _write_fresh_json(output, value)
    return value

def _run(command: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=cwd, check=True, text=True, capture_output=True)

def build_attested_image(
    output_dir: Path, *, repository: Path, expected_commit: str, run_label: str,
    base_reference: str, runner: Any = _run,
) -> dict[str, Any]:
    """Build once from a fixed Git archive and issue a closed attestation from measured bytes."""
    if output_dir.exists() or output_dir.is_symlink():
        raise FileExistsError(output_dir)
    output_dir.mkdir(parents=True)
    context = output_dir / "context"
    context.mkdir()
    archive_before = output_dir / "source-before.tar"
    archive_after = output_dir / "source-after.tar"
    log = output_dir / "commands.jsonl"
    commands: list[dict[str, Any]] = []
    def execute(args: list[str], *, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
        try:
            result = runner(args, cwd=cwd)
            commands.append({"argv": args, "cwd": str(cwd) if cwd else None,
                             "exit_code": result.returncode, "stdout": result.stdout,
                             "stderr": result.stderr})
            return result
        except Exception as error:
            commands.append({"argv": args, "cwd": str(cwd) if cwd else None,
                             "error": f"{type(error).__name__}: {error}"})
            raise
        finally:
            log.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in commands), encoding="utf-8")
    head = execute(["git", "rev-parse", "HEAD"], cwd=repository).stdout.strip()
    clean = execute(["git", "status", "--porcelain"], cwd=repository).stdout
    if head != expected_commit or clean:
        raise ValueError("build repository is not the clean expected commit")
    execute(["git", "archive", "--format=tar", f"--output={archive_before}", expected_commit], cwd=repository)
    with tarfile.open(archive_before) as archive:
        archive.extractall(context, filter="data")
    sources = source_set(context)
    dockerfile_text = (context / "scripts/ros/Dockerfile.g3").read_text(encoding="utf-8")
    if ("ARG BASE_IMAGE" not in dockerfile_text
            or dockerfile_text.count("FROM ${BASE_IMAGE}") != 2):
        raise ValueError("G3 Dockerfile does not pin both stages to BASE_IMAGE")
    base_result = execute(["docker", "image", "inspect", base_reference])
    base_inspect = json.loads(base_result.stdout)
    base_inspect_path = output_dir / "base-image-inspect.json"
    _write_fresh_json(base_inspect_path, {"images": base_inspect})
    base_identity = _image(base_inspect, "base_image")
    built_reference = f"factory-ops-nav2:g3-{run_label}-{expected_commit[:12]}"
    try:
        existing = runner(["docker", "image", "inspect", built_reference], cwd=None)
    except subprocess.CalledProcessError as error:
        commands.append({"argv": ["docker", "image", "inspect", built_reference], "cwd": None,
                         "expected_absent_exit_code": error.returncode})
        log.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in commands), encoding="utf-8")
    else:
        commands.append({"argv": ["docker", "image", "inspect", built_reference], "cwd": None,
                         "exit_code": existing.returncode, "stdout": existing.stdout, "stderr": existing.stderr})
        log.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in commands), encoding="utf-8")
        raise ValueError("run/source image tag already exists")
    iidfile = output_dir / "built-image.iid"
    execute(["docker", "build", "--pull=false", "--build-arg", f"BASE_IMAGE={base_identity['id']}",
             "--iidfile", str(iidfile), "--tag", built_reference, "--file",
             str(context / "scripts/ros/Dockerfile.g3"), str(context)])
    built_inspect = json.loads(execute(["docker", "image", "inspect", built_reference]).stdout)
    built_inspect_path = output_dir / "built-image-inspect.json"
    _write_fresh_json(built_inspect_path, {"images": built_inspect})
    built_identity = _image(built_inspect, "built_image")
    iid = iidfile.read_text(encoding="utf-8").strip()
    if iid != built_identity["id"]:
        raise ValueError("docker iidfile differs from the built image")
    container_name = f"factory-ops-attest-{run_label}-{expected_commit[:12]}"
    installed_root = output_dir / "installed"
    installed_root.mkdir()
    container_created = False
    try:
        container_id = execute(["docker", "create", "--name", container_name, built_identity["id"]]).stdout.strip()
        if not container_id:
            raise ValueError("docker create returned no container identity")
        container_created = True
        for source in INSTALLED_PATHS:
            execute(["docker", "cp", f"{container_id}:{source}", str(installed_root / Path(source).name)])
    finally:
        if container_created:
            execute(["docker", "rm", "-f", container_name])
    installed = {source: sha256_bytes((installed_root / Path(source).name).read_bytes())
                 for source in INSTALLED_PATHS}
    execute(["git", "archive", "--format=tar", f"--output={archive_after}", expected_commit], cwd=repository)
    if sha256_bytes(archive_before.read_bytes()) != sha256_bytes(archive_after.read_bytes()):
        raise ValueError("fixed Git archive changed during the build")
    if execute(["git", "rev-parse", "HEAD"], cwd=repository).stdout.strip() != expected_commit:
        raise ValueError("build repository HEAD changed during the build")
    if execute(["git", "status", "--porcelain"], cwd=repository).stdout:
        raise ValueError("build repository became dirty during the build")
    proof = {
        "source_archive_sha256": sha256_bytes(archive_before.read_bytes()),
        "base_inspect_sha256": sha256_bytes(base_inspect_path.read_bytes()),
        "built_inspect_sha256": sha256_bytes(built_inspect_path.read_bytes()),
        "iid": iid, "base_image_arg": base_identity["id"], "container_measurement_closed": True,
    }
    installed_measurement = output_dir / "installed-measurement.json"
    _write_fresh_json(installed_measurement, {"schema_version": 1, "image_id": built_identity["id"],
                                              "files": installed})
    receipt = {
        "schema_version": 1, "source_commit": expected_commit, "run_label": run_label,
        "source_archive_sha256": proof["source_archive_sha256"],
        "commands_sha256": sha256_bytes(log.read_bytes()),
        "iidfile_sha256": sha256_bytes(iidfile.read_bytes()),
        "base_inspect_sha256": proof["base_inspect_sha256"],
        "built_inspect_sha256": proof["built_inspect_sha256"],
        "installed_measurement_sha256": sha256_bytes(installed_measurement.read_bytes()),
        "built_image_id": built_identity["id"], "cleanup_complete": True,
    }
    receipt_path = output_dir / "build-receipt.json"
    _write_fresh_json(receipt_path, receipt)
    value = {
        "schema_version": 1, "status": "built", "source_commit": expected_commit,
        "run_label": run_label, "source_files": sources,
        "source_set_sha256": canonical_sha(sources),
        "dockerfile_sha256": sources["scripts/ros/Dockerfile.g3"],
        "base_image": {"reference": base_reference, **base_identity},
        "built_image": {"reference": built_reference, **built_identity},
        "installed_files": installed, "build_proof": proof,
        "build_receipt_sha256": sha256_bytes(receipt_path.read_bytes()),
        "build_completed": True, "runtime_eligible": True,
    }
    attestation = output_dir / "build-attestation.json"
    _write_fresh_json(attestation, value)
    return value

def _image(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], Mapping):
        raise ValueError(f"{label} docker inspect must contain one image")
    row = value[0]
    result = {"id": row.get("Id"), "repo_digests": sorted(row.get("RepoDigests") or [])}
    if not isinstance(result["id"], str) or not IMAGE_ID.fullmatch(result["id"]):
        raise ValueError(f"{label} image ID is invalid")
    if not all(isinstance(item, str) and REPO_DIGEST.fullmatch(item)
               for item in result["repo_digests"]):
        raise ValueError(f"{label} image digest is invalid")
    return result

def _archive_commit(path: Path) -> str:
    with tarfile.open(path) as archive:
        commit = archive.pax_headers.get("comment")
    if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("source archive lacks the fixed Git commit identity")
    return commit


def _validate_build_commands(
    path: Path, *, source_commit: str, base_reference: str, base_image_id: str,
    built_reference: str, built_image_id: str,
) -> None:
    try:
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError("build command record is invalid") from error
    if not rows or any(not isinstance(row, Mapping) or not isinstance(row.get("argv"), list)
                       for row in rows):
        raise ValueError("build command record is invalid")
    commands = [row["argv"] for row in rows]
    if any("error" in row and "expected_absent_exit_code" not in row for row in rows):
        raise ValueError("build command record contains a failed operation")
    absent = [index for index, row in enumerate(rows)
              if row["argv"] == ["docker", "image", "inspect", built_reference]
              and type(row.get("expected_absent_exit_code")) is int
              and row["expected_absent_exit_code"] != 0]
    builds = [index for index, command in enumerate(commands) if command[:2] == ["docker", "build"]]
    creates = [index for index, command in enumerate(commands)
               if command[:3] == ["docker", "create", "--name"] and command[-1] == built_image_id]
    removes = [index for index, command in enumerate(commands)
               if command[:3] == ["docker", "rm", "-f"]]
    copies = [(index, command) for index, command in enumerate(commands)
              if command[:2] == ["docker", "cp"]]
    archives = [command for command in commands if command[:2] == ["git", "archive"]]
    if (len(absent) != 1 or len(builds) != 1 or len(creates) != 1 or len(removes) != 1
            or len(copies) != len(INSTALLED_PATHS) or len(archives) != 2
            or not (absent[0] < builds[0] < creates[0] < min(index for index, _ in copies)
                    <= max(index for index, _ in copies) < removes[0])
            or any(command[-1] != source_commit for command in archives)
            or [command for command in commands if command == ["docker", "image", "inspect", base_reference]] == []
            or len([command for command in commands
                    if command == ["docker", "image", "inspect", built_reference]]) != 2):
        raise ValueError("closed build command sequence differs")
    build = commands[builds[0]]
    required_pairs = {
        "--build-arg": f"BASE_IMAGE={base_image_id}",
        "--tag": built_reference,
    }
    for option, expected in required_pairs.items():
        if option not in build or build.index(option) + 1 >= len(build) or build[build.index(option) + 1] != expected:
            raise ValueError("closed docker build arguments differ")
    copied_sources = {command[2].split(":", 1)[1] for _, command in copies
                      if len(command) == 4 and ":" in command[2]}
    if copied_sources != set(INSTALLED_PATHS):
        raise ValueError("closed installed measurement command set differs")

def _remeasure_installed(
    image_id: str, output_dir: Path, *, runner: Any,
) -> tuple[dict[str, str], Path]:
    if output_dir.exists() or output_dir.is_symlink():
        raise FileExistsError(output_dir)
    output_dir.mkdir(parents=True)
    installed_root = output_dir / "installed"
    installed_root.mkdir()
    container_name = f"factory-ops-preflight-{image_id.removeprefix('sha256:')[:16]}"
    commands: list[list[str]] = []
    container_created = False
    cleanup_complete = False
    try:
        command = ["docker", "create", "--name", container_name, image_id]
        commands.append(command)
        container_id = runner(command, cwd=None).stdout.strip()
        if not container_id:
            raise ValueError("preflight docker create returned no container identity")
        container_created = True
        for source in INSTALLED_PATHS:
            command = ["docker", "cp", f"{container_id}:{source}",
                       str(installed_root / Path(source).name)]
            commands.append(command)
            runner(command, cwd=None)
    finally:
        if container_created:
            command = ["docker", "rm", "-f", container_name]
            commands.append(command)
            runner(command, cwd=None)
            cleanup_complete = True
    measured = {source: sha256_bytes((installed_root / Path(source).name).read_bytes())
                for source in INSTALLED_PATHS}
    receipt_path = output_dir / "preflight-measurement.json"
    _write_fresh_json(receipt_path, {"schema_version": 1, "image_id": image_id,
                                     "files": measured, "commands": commands,
                                     "cleanup_complete": cleanup_complete})
    return measured, receipt_path

def validate_attestation(
    path: Path, *, expected_sha256: str, source_root: Path, expected_source_commit: str,
    expected_run_label: str, base_inspect: Any, built_inspect: Any, source_archive: Path,
    build_receipt: Path, expected_build_receipt_sha256: str,
    build_commands: Path, build_iidfile: Path,
    build_base_inspect: Path, build_built_inspect: Path,
    build_installed_measurement: Path, measurement_dir: Path, runner: Any = _run,
) -> dict[str, Any]:
    raw = path.read_bytes()
    if path.is_symlink() or not SHA256.fullmatch(expected_sha256) or sha256_bytes(raw) != expected_sha256:
        raise ValueError("build attestation external hash differs")
    value = json.loads(raw)
    required = {"schema_version", "status", "source_commit", "run_label", "source_files",
                "source_set_sha256", "dockerfile_sha256", "base_image", "built_image",
                "installed_files", "build_proof", "build_receipt_sha256",
                "build_completed", "runtime_eligible"}
    if not isinstance(value, dict) or set(value) != required:
        raise ValueError("build attestation schema differs")
    actual_sources = source_set(source_root)
    if (value["schema_version"] != 1 or value["status"] != "built"
            or value["source_commit"] != expected_source_commit
            or value["run_label"] != expected_run_label or value["source_files"] != actual_sources
            or value["source_set_sha256"] != canonical_sha(actual_sources)
            or value["dockerfile_sha256"] != actual_sources["scripts/ros/Dockerfile.g3"]
            or value["build_completed"] is not True or value["runtime_eligible"] is not True):
        raise ValueError("build attestation source or run binding differs")
    if _archive_commit(source_archive) != expected_source_commit:
        raise ValueError("source archive commit differs")
    for name, inspect in (("base_image", base_inspect), ("built_image", built_inspect)):
        row = value[name]
        if not isinstance(row, dict) or set(row) != {"reference", "id", "repo_digests"}:
            raise ValueError(f"{name} schema differs")
        if row | {} != {"reference": row["reference"], **_image(inspect, name)}:
            raise ValueError(f"{name} current identity differs")
    installed = value["installed_files"]
    if (not isinstance(installed, dict) or set(installed) != set(INSTALLED_PATHS)
            or not all(isinstance(v, str) and SHA256.fullmatch(v) for v in installed.values())):
        raise ValueError("installed artifact hashes differ")
    proof = value["build_proof"]
    if (not isinstance(proof, dict) or set(proof) != {"source_archive_sha256", "base_inspect_sha256",
            "built_inspect_sha256", "iid", "base_image_arg", "container_measurement_closed"}
            or proof["iid"] != value["built_image"]["id"]
            or proof["base_image_arg"] != value["base_image"]["id"]
            or proof["container_measurement_closed"] is not True
            or proof["source_archive_sha256"] != sha256_bytes(source_archive.read_bytes())
            or proof["base_inspect_sha256"] != sha256_bytes(build_base_inspect.read_bytes())
            or proof["built_inspect_sha256"] != sha256_bytes(build_built_inspect.read_bytes())
            or not all(SHA256.fullmatch(proof[name]) for name in (
                "source_archive_sha256", "base_inspect_sha256", "built_inspect_sha256"))):
        raise ValueError("closed build proof differs")
    receipt_raw = build_receipt.read_bytes()
    if (not SHA256.fullmatch(expected_build_receipt_sha256)
            or value["build_receipt_sha256"] != expected_build_receipt_sha256
            or sha256_bytes(receipt_raw) != expected_build_receipt_sha256):
        raise ValueError("build receipt hash differs")
    receipt = json.loads(receipt_raw)
    _validate_build_commands(
        build_commands, source_commit=expected_source_commit,
        base_reference=value["base_image"]["reference"], base_image_id=value["base_image"]["id"],
        built_reference=value["built_image"]["reference"], built_image_id=value["built_image"]["id"],
    )
    build_base_value = json.loads(build_base_inspect.read_text(encoding="utf-8"))
    build_built_value = json.loads(build_built_inspect.read_text(encoding="utf-8"))
    expected_receipt_keys = {"schema_version", "source_commit", "run_label",
        "source_archive_sha256", "commands_sha256", "iidfile_sha256",
        "base_inspect_sha256", "built_inspect_sha256", "installed_measurement_sha256",
        "built_image_id", "cleanup_complete"}
    if (not isinstance(receipt, dict) or set(receipt) != expected_receipt_keys
            or receipt["schema_version"] != 1 or receipt["source_commit"] != expected_source_commit
            or receipt["run_label"] != expected_run_label
            or receipt["source_archive_sha256"] != sha256_bytes(source_archive.read_bytes())
            or receipt["commands_sha256"] != sha256_bytes(build_commands.read_bytes())
            or receipt["iidfile_sha256"] != sha256_bytes(build_iidfile.read_bytes())
            or receipt["base_inspect_sha256"] != sha256_bytes(build_base_inspect.read_bytes())
            or receipt["built_inspect_sha256"] != sha256_bytes(build_built_inspect.read_bytes())
            or receipt["installed_measurement_sha256"] != sha256_bytes(
                build_installed_measurement.read_bytes())
            or receipt["built_image_id"] != value["built_image"]["id"]
            or receipt["cleanup_complete"] is not True
            or build_iidfile.read_text(encoding="utf-8").strip() != value["built_image"]["id"]):
        raise ValueError("closed production build receipt differs")
    if (not isinstance(build_base_value, dict) or set(build_base_value) != {"images"}
            or not isinstance(build_built_value, dict) or set(build_built_value) != {"images"}
            or _image(build_base_value["images"], "build_base_image") != {
                "id": value["base_image"]["id"], "repo_digests": value["base_image"]["repo_digests"]}
            or _image(build_built_value["images"], "build_built_image") != {
                "id": value["built_image"]["id"], "repo_digests": value["built_image"]["repo_digests"]}):
        raise ValueError("build image inspect originals differ")
    build_measurement = json.loads(build_installed_measurement.read_text(encoding="utf-8"))
    if build_measurement != {"schema_version": 1, "image_id": value["built_image"]["id"],
                             "files": installed}:
        raise ValueError("build installed measurement differs")
    measured, measurement_receipt = _remeasure_installed(
        value["built_image"]["id"], measurement_dir, runner=runner)
    if measured != installed:
        raise ValueError("current immutable image installed bytes differ")
    value = dict(value)
    value["preflight_measurement_sha256"] = sha256_bytes(measurement_receipt.read_bytes())
    return value

def references(path: Path, expected_sha256: str) -> tuple[str, str]:
    raw = path.read_bytes()
    if path.is_symlink() or sha256_bytes(raw) != expected_sha256:
        raise ValueError("build attestation external hash differs")
    value = json.loads(raw)
    return value["base_image"]["reference"], value["built_image"]["reference"]
