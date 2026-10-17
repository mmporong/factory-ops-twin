"""Copy one actual diagnostic RGB as an explicitly limited, traceable preview."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.dataset_evidence import _png_dimensions
from factory_ops.source_provenance import verify_source_archive


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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
    source.relative_to(root)
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
    review = json.loads(args.review.read_text(encoding="utf-8"))
    if (review.get("classification") != "static_media_visual_review"
            or review.get("source_rgb_sha256") != sha(source)
            or review.get("render_source_commit") != args.expected_commit
            or review.get("run_id") != environment["run_id"] or review.get("camera_id") != binding["camera_id"]
            or review.get("verdict") != "limited_preview_pass"):
        raise ValueError("visual review does not approve this exact static preview input")
    record_path = args.output.with_name(args.output.stem + "-provenance.json")
    if any(path.exists() or path.is_symlink() for path in (args.output, record_path)):
        raise ValueError("preview export requires new output paths")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, args.output)
    if source.read_bytes() != args.output.read_bytes():
        raise ValueError("preview changed the original image bytes")
    record = {"schema_version": 1, "classification": "actual_diagnostic_static_preview",
              "run_id": environment["run_id"], "render_source_commit": args.expected_commit,
              "isaac_image": environment["isaac_image"], "source_archive_proof": proof,
              "source_environment_sha256": sha(environment_path), "source_rgb_path": args.rgb,
              "source_rgb_sha256": sha(source), "output_rgb_sha256": sha(args.output),
              "frame_binding": binding, "frame_binding_scope": "producer raw tags; full dataset QA incomplete",
              "width": dimensions[0], "height": dimensions[1], "pixel_and_png_integrity_checked": True,
              "camera_raw_params_sha256": sha(source.with_name("camera.raw-params.json")),
              "source_failure_report_sha256": sha(root / "failed.json"),
              "visual_review": {"path": args.review.name, "sha256": sha(args.review),
                                "reviewer": review["reviewer"], "verdict": review["verdict"],
                                "scope": review["scope"], "score_out_of_10": review["score_out_of_10"]},
              "dataset_approved": False, "operational_evidence": False,
              "exporter": {"path": "scripts/media/export_g4_preview.py", "sha256": sha(Path(__file__))}}
    with record_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(record, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"rgb": str(args.output), "provenance": str(record_path), "sha256": sha(args.output)}))


if __name__ == "__main__":
    main()
