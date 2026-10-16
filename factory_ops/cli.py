"""Local preparation CLI. Does not provision cloud resources or run robots."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import sys

from .layout import Layout, load_traffic
from .operations import Ledger
from .report import write_report
from .smoke import PlanningTravel, canonical_hash, run_smoke
from .traffic_fixtures import run_traffic_checks
from .usda import export_usda

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-v3"
ARTIFACT_FILES = {
    "geometry.json", "report.html", "traffic-checks.json",
    "maps/map.pgm", "maps/keepout.pgm", "maps/map.yaml", "maps/keepout.yaml", "maps/manifest.json",
    "scene/factory-v3.usda", "scene/manifest.json",
    *(f"{policy}/{name}" for policy in ("fifo", "eta") for name in
      ("summary.json", "snapshot.json", "events.jsonl", "orders.csv", "assignments.csv")),
}


def _artifact_path(output: Path, name: str) -> Path:
    target = output / name
    if not target.resolve().is_relative_to(output.resolve()):
        raise ValueError(f"artifact path escapes output directory: {name}")
    return target


def _check_output_directory(output: Path) -> None:
    existing = {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()}
    unexpected = existing - ARTIFACT_FILES - {"manifest.json"}
    if unexpected:
        raise ValueError(f"output contains unrelated files; choose a dedicated directory: {sorted(unexpected)}")
    for name in ARTIFACT_FILES | {"manifest.json"}:
        _artifact_path(output, name)


def verify_manifest(output: Path) -> dict:
    """Check saved file consistency; this is not cryptographic authentication."""
    manifest = json.loads(_artifact_path(output, "manifest.json").read_text(encoding="utf-8"))
    files = manifest["files"]
    if set(files) != ARTIFACT_FILES:
        raise ValueError("artifact manifest file set differs from the preparation contract")
    actual_files = {path.relative_to(output).as_posix() for path in output.rglob("*") if path.is_file()}
    if actual_files != ARTIFACT_FILES | {"manifest.json"}:
        raise ValueError("actual artifact file set differs from the saved preparation contract")
    for name, expected in files.items():
        actual = hashlib.sha256(_artifact_path(output, name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"artifact SHA256 mismatch: {name}")
    return {"verified": True, "files_checked": len(files), "backend": manifest["backend"]}


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    def display_value(value: object) -> object:
        if isinstance(value, str) and (
            value.lstrip().startswith(("=", "+", "-", "@")) or value.startswith(("\t", "\r", "\n"))
        ):
            return "'" + value
        return value

    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({key: display_value(value) for key, value in row.items()} for row in rows)


def prepare(config_dir: Path, output: Path) -> dict:
    _check_output_directory(output)
    layout_path, traffic_path = config_dir / "layout.json", config_dir / "traffic.json"
    workload_path = config_dir / "smoke.json"
    layout = Layout.from_file(layout_path)
    analysis = layout.planning_analysis(load_traffic(traffic_path))
    config = json.loads(workload_path.read_text(encoding="utf-8"))
    traffic_checks = run_traffic_checks(traffic_path)
    if not traffic_checks["passed"]:
        raise RuntimeError("scripted traffic checks failed; artifacts were not generated")
    travel = PlanningTravel(layout)
    results = [run_smoke(config, policy, travel) for policy in ("fifo", "eta")]
    maps = layout.export_maps(output / "maps")
    usd = export_usda(layout, output / "scene" / "factory-v3.usda")
    usd["path"] = "scene/factory-v3.usda"
    write_json(output / "geometry.json", analysis)
    write_json(output / "maps" / "manifest.json", maps)
    write_json(output / "scene" / "manifest.json", usd)
    write_json(output / "traffic-checks.json", traffic_checks)
    for result in results:
        target = output / result["summary"]["policy"]
        write_json(target / "summary.json", result["summary"])
        write_json(target / "snapshot.json", result["snapshot"])
        (target / "events.jsonl").write_text(
            "".join(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n" for event in result["events"]),
            encoding="utf-8", newline="\n",
        )
        write_csv(target / "orders.csv", result["orders"])
        write_csv(target / "assignments.csv", result["assignments"])
    write_report(output / "report.html", layout, analysis, results, traffic_checks)
    repo = Path(__file__).resolve().parents[1]
    generator_files = [repo / "pyproject.toml", repo / "uv.lock", *sorted((repo / "factory_ops").rglob("*.py"))]
    manifest = {
        "backend": "cpu_event_model", "primary_evaluation": False,
        "isaac_validated": False, "nav2_validated": False, "cloud_resources_created": False,
        "generator_source_sha256": canonical_hash({
            path.relative_to(repo).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in generator_files
        }),
        "python_version": platform.python_version(),
        "traffic_thresholds": traffic_checks["thresholds"],
        "sources": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in (layout_path, traffic_path, workload_path)},
        "files": {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in sorted(ARTIFACT_FILES)},
    }
    write_json(output / "manifest.json", manifest)
    verify_manifest(output)
    return {"output": str(output.resolve()), "report": str((output / "report.html").resolve()),
            "backend": manifest["backend"], "policies": [result["summary"] for result in results]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    prepare_parser = subparsers.add_parser("prepare", help="Validate, export maps/USDA and run CPU integration checks")
    prepare_parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG)
    prepare_parser.add_argument("--output", type=Path, default=Path("runs/local"))
    replay_parser = subparsers.add_parser("replay", help="Replay an event log and optionally compare its final snapshot")
    replay_parser.add_argument("events", type=Path)
    replay_parser.add_argument("--expected", type=Path)
    verify_parser = subparsers.add_parser("verify", help="Check every generated artifact against the saved SHA256 manifest")
    verify_parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare(args.config_dir, args.output)
        elif args.command == "verify":
            result = verify_manifest(args.output)
        else:
            events = [json.loads(line) for line in args.events.read_text(encoding="utf-8").splitlines() if line.strip()]
            ledger = Ledger.replay(events)
            matches = True
            if args.expected:
                matches = ledger.snapshot() == json.loads(args.expected.read_text(encoding="utf-8"))
            result = {"backend": "ledger_replay", "matches": matches, "snapshot": ledger.snapshot()}
            if not matches:
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 1
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        print(f"factory-ops: {exc}", file=sys.stderr)
        return 1
    return 0
