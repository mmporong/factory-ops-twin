"""Strict normalization for the small native-asset perception probe."""

from __future__ import annotations

import ast
import json
import math
import re
from typing import Any, Mapping, Sequence


PRIMARY_CLASSES = {"pallet": 1, "small_load_carrier": 2}


class PerceptionDatasetError(ValueError):
    pass


def usd_safe_identifier(value: str) -> str:
    """Return a deterministic ASCII identifier accepted as one USD path component."""
    if not isinstance(value, str) or not value:
        raise PerceptionDatasetError("USD identifier source must be a nonempty string")
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if result[0].isdigit():
        result = "_" + result
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", result) is None:
        raise PerceptionDatasetError(f"could not derive a USD-safe identifier from {value!r}")
    return result


def jsonable(value: Any, *, path: str = "value") -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise PerceptionDatasetError(f"{path} contains a non-finite value")
        return value
    if isinstance(value, Mapping):
        return {str(key): jsonable(item, path=f"{path}.{key}") for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item, path=f"{path}[]") for item in value]
    if hasattr(value, "dtype") and getattr(value.dtype, "names", None):
        return [
            {
                str(name): jsonable(row[name].item() if hasattr(row[name], "item") else row[name], path=f"{path}[].{name}")
                for name in value.dtype.names
            }
            for row in value
        ]
    if hasattr(value, "tolist"):
        return jsonable(value.tolist(), path=path)
    if hasattr(value, "item"):
        return jsonable(value.item(), path=path)
    raise PerceptionDatasetError(f"{path} has unsupported type {type(value).__name__}")


def label_document(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return {str(key): item for key, item in value.items()}
    if isinstance(value, str):
        stripped = value.strip()
        for parser in (json.loads, ast.literal_eval):
            try:
                result = parser(stripped)
            except (ValueError, SyntaxError, json.JSONDecodeError):
                continue
            if isinstance(result, dict):
                return {str(key): item for key, item in result.items()}
        return {"class": stripped}
    return {}


def raw_payload(payload: Any, label: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not isinstance(payload, Mapping) or "data" not in payload or not isinstance(payload.get("info"), Mapping):
        raise PerceptionDatasetError(f"{label} payload has no data/info boundary")
    rows = jsonable(payload["data"], path=f"{label}.data")
    info = jsonable(payload["info"], path=f"{label}.info")
    if not isinstance(rows, list) or not isinstance(info, dict):
        raise PerceptionDatasetError(f"{label} payload is not JSON compatible")
    if rows and not all(isinstance(row, dict) for row in rows):
        raise PerceptionDatasetError(f"{label} rows are not objects")
    return rows, info


def raw_info(payload: Any, label: str) -> dict[str, Any]:
    if not isinstance(payload, Mapping) or "data" not in payload or not isinstance(payload.get("info"), Mapping):
        raise PerceptionDatasetError(f"{label} payload has no data/info boundary")
    info = jsonable(payload["info"], path=f"{label}.info")
    if not isinstance(info, dict):
        raise PerceptionDatasetError(f"{label} info is not JSON compatible")
    return info


def _label_map(info: Mapping[str, Any], label: str) -> dict[str, dict[str, Any]]:
    raw = info.get("idToLabels")
    if not isinstance(raw, Mapping):
        raise PerceptionDatasetError(f"{label} idToLabels is absent")
    return {str(key): label_document(value) for key, value in raw.items()}


def _field(row: Mapping[str, Any], names: Sequence[str], label: str) -> Any:
    for name in names:
        if name in row:
            return row[name]
    raise PerceptionDatasetError(f"bbox row has no {label} field")


def validate_capture(
    *,
    width: int,
    height: int,
    semantic: Any,
    semantic_info: Mapping[str, Any],
    bbox_rows: Sequence[Mapping[str, Any]],
    bbox_info: Mapping[str, Any],
    expected_objects: Mapping[str, str],
) -> dict[str, Any]:
    if width <= 0 or height <= 0:
        raise PerceptionDatasetError("capture dimensions must be positive")
    if not expected_objects or any(value not in PRIMARY_CLASSES for value in expected_objects.values()):
        raise PerceptionDatasetError("expected objects must use both authored primary class names only")
    shape = getattr(semantic, "shape", None)
    if shape != (height, width):
        raise PerceptionDatasetError(f"semantic shape differs from capture: {shape} != {(height, width)}")
    semantic_labels = _label_map(semantic_info, "semantic")
    semantic_class_ids: dict[str, set[int]] = {name: set() for name in PRIMARY_CLASSES}
    for raw_id, document in semantic_labels.items():
        class_name = document.get("class")
        if class_name in semantic_class_ids:
            semantic_class_ids[class_name].add(int(raw_id))
    class_pixel_counts = {}
    for class_name in set(expected_objects.values()):
        ids = semantic_class_ids[class_name]
        if not ids:
            raise PerceptionDatasetError(f"semantic label map has no target class: {class_name}")
        count = sum(int((semantic == semantic_id).sum()) for semantic_id in ids)
        if count <= 0:
            raise PerceptionDatasetError(f"semantic map has no pixels for target class: {class_name}")
        class_pixel_counts[class_name] = count

    bbox_labels = _label_map(bbox_info, "bbox")
    seen_objects: set[str] = set()
    annotations = []
    for row in bbox_rows:
        semantic_id = int(_field(row, ("semanticId", "semantic_id"), "semantic ID"))
        labels = bbox_labels.get(str(semantic_id))
        if labels is None:
            raise PerceptionDatasetError(f"bbox semantic ID is absent from label map: {semantic_id}")
        class_name = labels.get("class")
        object_id = labels.get("object_id")
        if class_name not in PRIMARY_CLASSES or not isinstance(object_id, str) or not object_id:
            raise PerceptionDatasetError(f"bbox labels do not bind an authored class and object_id: {labels}")
        if expected_objects.get(object_id) != class_name:
            raise PerceptionDatasetError(f"bbox class/object binding differs from provenance: {object_id} -> {class_name}")
        if object_id in seen_objects:
            raise PerceptionDatasetError(f"object has more than one bbox row: {object_id}")
        values = [
            int(_field(row, ("x_min", "xMin"), "x_min")),
            int(_field(row, ("y_min", "yMin"), "y_min")),
            int(_field(row, ("x_max", "xMax"), "x_max")),
            int(_field(row, ("y_max", "yMax"), "y_max")),
        ]
        x0, y0, x1, y1 = values
        if not (0 <= x0 <= x1 < width and 0 <= y0 <= y1 < height):
            raise PerceptionDatasetError(f"bbox extent is outside the RGB frame: {object_id}: {values}")
        # The existing G4 mask/raw-bbox contract uses inclusive maxima. The
        # repository's G4 contract checks them against mask min/max, so COCO
        # width and height include both endpoint pixels.
        box_width, box_height = x1 - x0 + 1, y1 - y0 + 1
        area = box_width * box_height
        if area <= 0:
            raise PerceptionDatasetError(f"bbox has no pixel area: {object_id}")
        seen_objects.add(object_id)
        annotations.append({
            "category_id": PRIMARY_CLASSES[class_name],
            "class": class_name,
            "object_id": object_id,
            "bbox": [x0, y0, box_width, box_height],
            "area": area,
            "iscrowd": 0,
            "source_semantic_id": semantic_id,
        })
    missing = set(expected_objects) - seen_objects
    extra = seen_objects - set(expected_objects)
    if missing or extra:
        raise PerceptionDatasetError(f"bbox/object coverage differs: missing={sorted(missing)}, extra={sorted(extra)}")
    if set(class_pixel_counts) != set(expected_objects.values()):
        raise PerceptionDatasetError("semantic class coverage differs from object provenance")
    return {
        "annotations": sorted(annotations, key=lambda item: item["object_id"]),
        "class_pixel_counts": class_pixel_counts,
        "bbox_object_count": len(annotations),
        "semantic_classes": sorted(class_pixel_counts),
    }
