#!/usr/bin/env python3
"""Display fixed-test predictions without rerunning inference or changing metrics."""

from __future__ import annotations

import argparse
from collections import defaultdict
from copy import deepcopy
import gzip
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from run_detector_pilot import canonical_json_sha256, merge_coco_parts, sha256_file


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def export(run: Path, real_images: Path, output: Path, selection_path: Path) -> dict:
    if output.exists():
        raise FileExistsError(f"output must be new: {output}")
    result = read(run / "result.json")
    selection = read(selection_path)
    image_ids = selection["fixed_test_ids"]
    if image_ids != [1, 270, 540, 810] or selection["display_score_threshold"] != .5 or selection["display_top_k"] != 20:
        raise ValueError("qualitative selection differs from the frozen display contract")
    prereg_path = run / "preregistration.json"
    prereg = read(prereg_path)
    if result["status"] != "complete" or sha256_file(prereg_path) != result["preregistration_sha256"]:
        raise ValueError("complete run and unchanged preregistration are required")
    parts = []
    for entry in prereg["real_registry"]["final_fixed_test"]:
        path = Path(entry["metadata_path"])
        if sha256_file(path) != entry["metadata_sha256"]:
            raise ValueError("fixed-test metadata changed")
        parts.append((read(path), entry["image_ids"], {int(k): v for k, v in entry["category_mapping"].items()}))
    gt, sources = merge_coco_parts(parts)
    gt_sha = canonical_json_sha256(gt)
    evaluated_gt = deepcopy(gt)
    # COCOeval 2.0.11 leaves bookkeeping fields on shared GT annotations.
    # The v1 writer hashes GT after the final (large-area) class evaluation.
    for annotation in evaluated_gt["annotations"]:
        annotation["ignore"] = annotation.get("iscrowd", 0)
        annotation["_ignore"] = int(bool(annotation["ignore"]) or annotation["area"] < 96**2
                                    or annotation["area"] > 1e10)
    evaluated_gt_sha = canonical_json_sha256(evaluated_gt)
    expected_ids = sorted(row["id"] for row in gt["images"])
    images = {row["id"]: row for row in gt["images"]}
    annotations = defaultdict(list)
    for row in gt["annotations"]:
        annotations[row["image_id"]].append(row)
    predictions = {}
    bindings = {}
    for condition in ("S", "R50", "SR50"):
        binding = result["evaluation"]["final_fixed_test"][condition]["prediction_artifact"]
        path = run / binding["path"]
        if sha256_file(path) != binding["sha256"]:
            raise ValueError("saved prediction hash changed")
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            payload = json.load(stream)
        if (payload["ground_truth_sha256"] not in (gt_sha, evaluated_gt_sha) or payload["image_ids"] != expected_ids
                or payload["condition"] != condition
                or payload["preregistration_sha256"] != result["preregistration_sha256"]
                or payload["config_sha256"] != prereg["config"]["sha256"]):
            raise ValueError("saved predictions differ from the fixed evaluation")
        by_image = defaultdict(list)
        for row in payload["rows"]:
            by_image[row["image_id"]].append(row)
        predictions[condition] = by_image
        bindings[condition] = binding
    if len(set(image_ids)) != len(image_ids) or not set(image_ids).issubset(images):
        raise ValueError("example IDs must be unique fixed-test IDs")
    font_path = Path("C:/Windows/Fonts/arial.ttf")
    heading = ImageFont.truetype(str(font_path), 23)
    font = ImageFont.truetype(str(font_path), 17)
    small = ImageFont.truetype(str(font_path), 14)
    tile_w, tile_h, row_h = 480, 360, 406
    canvas = Image.new("RGB", (tile_w * 4, 96 + len(image_ids) * row_h), "#f4f7fb")
    draw = ImageDraw.Draw(canvas)
    draw.text((16, 12), "LOCO fixed-test examples | same images, saved model predictions", font=heading, fill="#13263c")
    draw.text((16, 45), "Display: score >= 0.50, top 20 per image. Evaluation remains score >= 0.001 / maxDets 100.", font=font, fill="#34465a")
    colors = {1: "#2b81ff", 2: "#f18b20"}
    inventory = []
    for row_index, image_id in enumerate(image_ids):
        image_row = images[image_id]
        path = (real_images / image_row["file_name"]).resolve()
        path.relative_to(real_images.resolve())
        source_image = Image.open(path).convert("RGB")
        scale = min(tile_w / source_image.width, tile_h / source_image.height)
        resized = source_image.resize((round(source_image.width * scale), round(source_image.height * scale)))
        offset_x, offset_y = (tile_w - resized.width) // 2, (tile_h - resized.height) // 2
        displayed = {}
        for column, condition in enumerate(("GT", "S", "R50", "SR50")):
            tile = Image.new("RGB", (tile_w, tile_h), "#e8edf3")
            tile.paste(resized, (offset_x, offset_y))
            td = ImageDraw.Draw(tile)
            boxes = annotations[image_id] if condition == "GT" else sorted(
                (p for p in predictions[condition][image_id] if p["score"] >= .5),
                key=lambda p: p["score"], reverse=True)[:20]
            for box in boxes:
                left, top, width, height = box["bbox"]
                coords = [offset_x + left * scale, offset_y + top * scale,
                          offset_x + (left + width) * scale, offset_y + (top + height) * scale]
                td.rectangle(coords, outline=colors[box["category_id"]], width=2)
            x, y = column * tile_w, 96 + row_index * row_h
            canvas.paste(tile, (x, y))
            draw.text((x + 9, y + 363), f"{condition} | image {image_id} | {len(boxes)} displayed boxes", font=font, fill="#13263c")
            draw.text((x + 9, y + 386), "pallet: blue / small load carrier: orange", font=small, fill="#34465a")
            displayed[condition] = len(boxes)
        inventory.append({"fixed_test_image_id": image_id, "source": sources[image_id - 1],
                          "file_name": image_row["file_name"], "rgb_sha256": sha256_file(path),
                          "displayed_box_counts": displayed})
    output.mkdir(parents=True)
    media = output / "fixed-test-examples.jpg"
    canvas.save(media, quality=92, subsampling=0, optimize=True)
    provenance = {"schema_version": 1, "run_id": run.name, "source_result_sha256": sha256_file(run / "result.json"),
                  "preregistration_sha256": result["preregistration_sha256"], "ground_truth_sha256": gt_sha,
                  "evaluated_ground_truth_sha256": evaluated_gt_sha,
                  "v1_cocoeval_bookkeeping_reconstructed": True,
                  "qualitative_preregistration_sha256": sha256_file(selection_path),
                  "exporter_sha256": sha256_file(Path(__file__)),
                  "prediction_artifacts": bindings, "image_selection": "fixed IDs [1,270,540,810] before first model result",
                  "images": inventory, "display_score_threshold": .5, "display_top_k": 20,
                  "evaluation_threshold_unchanged": .001, "model_inference_rerun": False,
                  "purpose": "qualitative examples; does not replace the complete fixed-test metrics",
                  "dataset_credit": "LOCO, tum-fml/loco, CC0-1.0", "image_sha256": sha256_file(media)}
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8", newline="\n")
    return provenance


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--real-image-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--qualitative-preregistration", type=Path, required=True)
    args = parser.parse_args()
    result = export(args.run_root.resolve(), args.real_image_root.resolve(), args.output.resolve(),
                    args.qualitative_preregistration.resolve())
    print(json.dumps({"status": "exported_saved_predictions", "image_sha256": result["image_sha256"]}))


if __name__ == "__main__":
    main()
