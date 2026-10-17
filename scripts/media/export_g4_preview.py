"""Copy one actual diagnostic RGB as an explicitly limited, traceable preview."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.dataset_evidence import _png_dimensions
from factory_ops.source_provenance import verify_source_archive


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_regular_json(path: Path) -> tuple[bytes, dict[str, object]]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"diagnostic source report must be a regular file: {path.name}")
    raw = path.read_bytes()
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError(f"diagnostic source report must be an object: {path.name}")
    return raw, value


def _bind_source_report(
    root: Path,
    source: Path,
    expected_commit: str,
    environment: dict[str, object],
) -> dict[str, object]:
    """Bind the selected RGB to exactly one failed or successful diagnostic report."""
    failed_path, dataset_path = root / "failed.json", root / "dataset.json"
    present = [path for path in (failed_path, dataset_path) if path.exists() or path.is_symlink()]
    if len(present) != 1:
        raise ValueError("diagnostic root must contain exactly one of failed.json or dataset.json")
    report_path = present[0]
    raw, report = _load_regular_json(report_path)
    report_sha = hashlib.sha256(raw).hexdigest()
    run_id = environment.get("run_id")
    if report.get("run_id") != run_id:
        raise ValueError("diagnostic source report run identity differs")
    if report_path == failed_path:
        if report.get("status") != "failed":
            raise ValueError("failed diagnostic report status differs")
        return {"kind": "failed", "path": "failed.json", "sha256": report_sha}

    source_record = report.get("source")
    if (report.get("status") != "diagnostic_unreviewed"
            or report.get("primary_evaluation") is not False
            or report.get("training_eligible") is not False
            or not isinstance(source_record, dict) or source_record.get("commit") != expected_commit):
        raise ValueError("successful diagnostic report scope or source identity differs")
    manifest = report.get("artifact_manifest")
    if not isinstance(manifest, list):
        raise ValueError("successful diagnostic artifact manifest is missing")
    relative = source.relative_to(root).as_posix()
    matches = [entry for entry in manifest if isinstance(entry, dict) and entry.get("path") == relative]
    if len(matches) != 1:
        raise ValueError("selected RGB is not uniquely declared by the artifact manifest")
    entry = matches[0]
    if (set(entry) != {"path", "bytes", "sha256"} or type(entry.get("bytes")) is not int
            or entry.get("bytes") != source.stat().st_size or entry.get("sha256") != sha(source)):
        raise ValueError("selected RGB artifact manifest bytes or SHA differ")
    if report_path.read_bytes() != raw:
        raise ValueError("successful diagnostic report changed during binding")
    return {"kind": "dataset", "path": "dataset.json", "sha256": report_sha,
            "artifact_manifest_entry": dict(entry)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--diagnostic", type=Path, required=True)
    parser.add_argument("--rgb", required=True, help="Relative RGB path under diagnostic root")
    parser.add_argument("--expected-commit", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--review", type=Path, required=True)
    args = parser.parse_args()
    root = args.diagnostic.resolve(strict=True)
    source = (root / args.rgb).resolve(strict=True)
    source_relative = source.relative_to(root).as_posix()
    if source.name != "rgb.png" or source.is_symlink():
        raise ValueError("preview input must be an original RGB file under the diagnostic root")
    provenance = root / "provenance"
    environment_path = provenance / "source-environment.json"
    environment = json.loads(environment_path.read_text(encoding="utf-8"))
    if environment.get("source_commit") != args.expected_commit or environment.get("scope") != "diagnostic_unreviewed":
        raise ValueError("preview source must be the expected diagnostic commit")
    proof = verify_source_archive(provenance / "source.tar", provenance / "source-commit.raw",
                                  args.expected_commit, environment["source_files"])
    if proof.get("passed") is not True:
        raise ValueError(f"diagnostic source archive failed verification: {proof}")
    errors: list[str] = []
    dimensions = _png_dimensions(source, errors, "preview RGB")
    if errors or dimensions is None:
        raise ValueError(f"diagnostic PNG failed integrity checks: {errors}")
    frame = json.loads(source.with_name("instance-id.raw-info.json").read_text(encoding="utf-8"))
    binding = {name: frame[name] for name in ("scene_id", "capture_id", "camera_id", "physics_frame", "sim_time_ns")}
    if source.parent.name != binding["camera_id"]:
        raise ValueError("preview camera identity differs")
    source_report = _bind_source_report(root, source, args.expected_commit, environment)
    source_sha = sha(source)
    review = json.loads(args.review.read_text(encoding="utf-8"))
    if (review.get("classification") != "static_media_visual_review"
            or review.get("source_rgb_sha256") != source_sha
            or review.get("render_source_commit") != args.expected_commit
            or review.get("run_id") != environment["run_id"] or review.get("camera_id") != binding["camera_id"]
            or review.get("verdict") != "limited_preview_pass"):
        raise ValueError("visual review does not approve this exact static preview input")
    record_path = args.output.with_name(args.output.stem + "-provenance.json")
    if any(path.exists() or path.is_symlink() for path in (args.output, record_path)):
        raise ValueError("preview export requires new output paths")
    if (sha(source) != source_sha
            or sha(root / str(source_report["path"])) != source_report["sha256"]):
        raise ValueError("preview diagnostic source changed during review binding")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=".g4-preview-", suffix=".png", dir=args.output.parent)
    os.close(descriptor)
    temporary_rgb = Path(temporary_name)
    temporary_record: Path | None = None
    created_output = created_record = False
    try:
        shutil.copyfile(source, temporary_rgb)
        if (sha(source) != source_sha or sha(temporary_rgb) != source_sha
                or sha(root / str(source_report["path"])) != source_report["sha256"]):
            raise ValueError("preview source or copied image changed during export")
        record = {"schema_version": 1, "classification": "actual_diagnostic_static_preview",
                  "run_id": environment["run_id"], "render_source_commit": args.expected_commit,
                  "isaac_image": environment["isaac_image"], "source_archive_proof": proof,
                  "source_environment_sha256": sha(environment_path), "source_rgb_path": source_relative,
                  "source_rgb_sha256": source_sha, "output_rgb_sha256": source_sha,
                  "frame_binding": binding, "frame_binding_scope": "producer raw tags; full dataset QA incomplete",
                  "width": dimensions[0], "height": dimensions[1], "pixel_and_png_integrity_checked": True,
                  "camera_raw_params_sha256": sha(source.with_name("camera.raw-params.json")),
                  "source_report": source_report,
                  "visual_review": {"path": args.review.name, "sha256": sha(args.review),
                                    "reviewer": review["reviewer"], "verdict": review["verdict"],
                                    "scope": review["scope"], "score_out_of_10": review["score_out_of_10"]},
                  "dataset_approved": False, "operational_evidence": False,
                  "exporter": {"path": "scripts/media/export_g4_preview.py", "sha256": sha(Path(__file__))}}
        if source_report["kind"] == "failed":
            record["source_failure_report_sha256"] = source_report["sha256"]
        else:
            record["source_dataset_report_sha256"] = source_report["sha256"]
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", newline="\n", prefix=".g4-preview-", suffix=".json",
            dir=args.output.parent, delete=False,
        ) as stream:
            temporary_record = Path(stream.name)
            json.dump(record, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        record_sha = sha(temporary_record)
        if (sha(source) != source_sha or sha(temporary_rgb) != source_sha
                or sha(root / str(source_report["path"])) != source_report["sha256"]):
            raise ValueError("preview source or copied image changed before publication")
        os.link(temporary_rgb, args.output)
        created_output = True
        os.link(temporary_record, record_path)
        created_record = True
        if (sha(source) != source_sha or sha(args.output) != source_sha
                or sha(root / str(source_report["path"])) != source_report["sha256"]
                or sha(record_path) != record_sha):
            raise ValueError("preview output or provenance changed during publication")
    except Exception:
        if created_record:
            record_path.unlink(missing_ok=True)
        if created_output:
            args.output.unlink(missing_ok=True)
        raise
    finally:
        temporary_rgb.unlink(missing_ok=True)
        if temporary_record is not None:
            temporary_record.unlink(missing_ok=True)
    print(json.dumps({"rgb": str(args.output), "provenance": str(record_path), "sha256": sha(args.output)}))


if __name__ == "__main__":
    main()
