"""Validated render-only NVIDIA assets for unused factory floor areas."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
from typing import Any, Mapping

from .layout import Layout, LayoutError, Rect


_CLEARANCE_M = 0.1
_HEX40 = re.compile(r"[0-9a-f]{40}")
_HEX64 = re.compile(r"[0-9a-f]{64}")
_YAW_VALUES = {0, 90, 180, 270}


def build_factory_context(
    layout: Layout,
    environment: Mapping[str, Any],
    *,
    cache_root: Path,
) -> dict[str, object]:
    """Build USDA references for vetted, render-only factory context assets."""
    revision = _string(environment.get("revision"), "environment.revision")
    if not _HEX40.fullmatch(revision):
        raise LayoutError("environment.revision must be a lowercase 40-character hex revision")
    repository = _string(environment.get("repository"), "environment.repository")
    if repository != "nvidia/simready-assets":
        raise LayoutError("environment.repository must be nvidia/simready-assets")
    catalog = _mapping(environment.get("catalog"), "environment.catalog")
    fixtures = environment.get("fixtures")
    if not isinstance(fixtures, list):
        raise LayoutError("environment.fixtures must be a list")

    root = Path(cache_root)
    _require_plain(root, "cache_root", directory=True)
    revision_root = root / revision
    _require_plain(revision_root, "cache revision", directory=True)
    index_path = revision_root / "prepared-index.json"
    index = _load_json(index_path, "prepared index")
    if index.get("revision") != revision:
        raise LayoutError("prepared index revision does not match environment revision")
    if index.get("status") != "passed":
        raise LayoutError("prepared index must be complete before scene composition")
    prepared_assets = _mapping(index.get("assets"), "prepared index assets")

    lines = [
        '        def Xform "FactoryContext"',
        "        {",
    ]
    instances: list[dict[str, object]] = []
    packages_by_asset: dict[str, dict[str, object]] = {}
    occupied: list[tuple[str, Rect]] = []
    mounted_by_obstacle: dict[str, dict[str, object]] = {}
    stable_ids: set[str] = set()
    for ordinal, raw_fixture in enumerate(fixtures):
        fixture = _mapping(raw_fixture, f"environment.fixtures[{ordinal}]")
        stable_id = _string(fixture.get("id"), f"fixture[{ordinal}].id")
        if stable_id in stable_ids:
            raise LayoutError(f"duplicate factory context fixture id {stable_id!r}")
        stable_ids.add(stable_id)
        asset_id = _string(fixture.get("asset"), f"fixture {stable_id}.asset")
        catalog_item = _mapping(catalog.get(asset_id), f"catalog.{asset_id}")
        prepared = _mapping(prepared_assets.get(asset_id), f"prepared index asset {asset_id}")
        prepared_path, prepared_sha = _prepared_asset(revision_root, prepared, asset_id)
        source_bounds = _bounds(prepared.get("bounds_xyz_m"), f"prepared asset {asset_id}.bounds_xyz_m")
        if source_bounds[2] != 0.0:
            raise LayoutError(f"prepared asset {asset_id} must have floor Z=0")
        if not math.isclose(source_bounds[0] + source_bounds[3], 0.0, abs_tol=1e-9) or not math.isclose(
            source_bounds[1] + source_bounds[4], 0.0, abs_tol=1e-9
        ):
            raise LayoutError(f"prepared asset {asset_id} must be centred at XY origin")

        center = fixture.get("center_xy")
        if not isinstance(center, list) or len(center) != 2:
            raise LayoutError(f"fixture {stable_id}.center_xy must be [x, y]")
        cx = _number(center[0], f"fixture {stable_id}.center_xy[0]")
        cy = _number(center[1], f"fixture {stable_id}.center_xy[1]")
        yaw = fixture.get("yaw_deg")
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or float(yaw) not in _YAW_VALUES:
            raise LayoutError(f"fixture {stable_id}.yaw_deg must be 0, 90, 180, or 270")
        yaw_value = int(yaw)
        world_bounds = _world_bounds(source_bounds, cx, cy, yaw_value)
        footprint = Rect(world_bounds[0], world_bounds[1], world_bounds[3], world_bounds[4])
        source_obstacle_id = fixture.get("source_obstacle_id")
        if source_obstacle_id is not None:
            source_obstacle_id = _string(
                source_obstacle_id, f"fixture {stable_id}.source_obstacle_id",
            )
            if source_obstacle_id in mounted_by_obstacle:
                raise LayoutError(
                    f"source obstacle {source_obstacle_id!r} has duplicate factory context fixtures"
                )
        source_bounds_xy = _validate_placement(
            layout, world_bounds, stable_id, occupied,
            source_obstacle_id=source_obstacle_id,
        )
        occupied.append((stable_id, footprint))

        semantic_class = _string(fixture.get("semantic_class"), f"fixture {stable_id}.semantic_class")
        state_role = _string(fixture.get("role"), f"fixture {stable_id}.role")
        prim_name = _prim_name(stable_id)
        prim_path = f"/World/Visuals/FactoryContext/{prim_name}"
        asset_path = _usd_asset_path(prepared_path)
        lines.extend([
            f'            def Xform "{prim_name}"',
            "            {",
            f"                double3 xformOp:translate = ({_num(cx)}, {_num(cy)}, 0)",
            f"                double xformOp:rotateZ = {yaw_value}",
            '                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]',
            f"                custom string semanticClass = {_quote(semantic_class)}",
            f"                custom string stateRole = {_quote(state_role)}",
            f"                custom string stableId = {_quote(stable_id)}",
            *([] if source_obstacle_id is None else [
                f"                custom string sourceObstacleId = {_quote(source_obstacle_id)}",
            ]),
            '                def Xform "Model" (',
            "                    instanceable = true",
            f"                    prepend references = @{asset_path}@</Asset>",
            "                )",
            "                {",
            "                }",
            "            }",
        ])
        instance = {
            "stable_id": stable_id,
            "prim_path": prim_path,
            "semantic_class": semantic_class,
            "state_role": state_role,
            "source_obstacle_id": source_obstacle_id,
            "bounds_xyz_m": world_bounds,
            "source_bounds_xy_m": source_bounds_xy,
            "asset": f"nvidia:{asset_id}",
        }
        instances.append(instance)
        if source_obstacle_id is not None:
            mounted_by_obstacle[source_obstacle_id] = instance
        packages_by_asset.setdefault(
            asset_id,
            _package_record(asset_id, repository, revision, catalog_item, prepared, prepared_path, prepared_sha),
        )

    lines.append("        }")
    return {
        "lines": lines,
        "instances": instances,
        "mounted_by_obstacle": mounted_by_obstacle,
        "packages": [packages_by_asset[key] for key in sorted(packages_by_asset)],
        "scope": "render-only factory context; mounted assets remain inside source obstacle envelopes; no physics, navigation, or safety claim",
    }


def _prepared_asset(root: Path, item: Mapping[str, Any], asset_id: str) -> tuple[Path, str]:
    relative_value = _string(item.get("relative_path"), f"prepared asset {asset_id}.relative_path")
    if "\\" in relative_value:
        raise LayoutError(f"prepared asset {asset_id} relative_path must use cache-relative POSIX separators")
    relative = PurePosixPath(relative_value)
    if relative.is_absolute() or not relative.parts or any(part in ("", ".", "..") for part in relative.parts):
        raise LayoutError(f"prepared asset {asset_id} relative_path must remain inside the revision cache")
    path = root.joinpath(*relative.parts)
    _require_plain(path, f"prepared asset {asset_id}", file=True)
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise LayoutError(f"prepared asset {asset_id} escapes the revision cache") from exc
    expected = _string(item.get("sha256"), f"prepared asset {asset_id}.sha256")
    if not _HEX64.fullmatch(expected):
        raise LayoutError(f"prepared asset {asset_id}.sha256 must be lowercase hex")
    measured = hashlib.sha256(path.read_bytes()).hexdigest()
    if measured != expected:
        raise LayoutError(f"prepared asset {asset_id} SHA-256 mismatch")
    if item.get("scope") != "render-only; physics APIs and typed prims removed":
        raise LayoutError(f"prepared asset {asset_id} must be render-only with physics APIs and typed prims removed")
    return path, measured


def _validate_placement(
    layout: Layout,
    world_bounds: tuple[float, float, float, float, float, float],
    stable_id: str,
    occupied: list[tuple[str, Rect]],
    *,
    source_obstacle_id: str | None,
) -> tuple[float, float, float, float] | None:
    footprint = Rect(world_bounds[0], world_bounds[1], world_bounds[3], world_bounds[4])
    padded = Rect(
        footprint.xmin - _CLEARANCE_M,
        footprint.ymin - _CLEARANCE_M,
        footprint.xmax + _CLEARANCE_M,
        footprint.ymax + _CLEARANCE_M,
    )
    bounds = layout.bounds
    if (padded.xmin < bounds.xmin or padded.ymin < bounds.ymin
            or padded.xmax > bounds.xmax or padded.ymax > bounds.ymax):
        raise LayoutError(f"factory context fixture {stable_id} violates layout bounds or 0.1m clearance")
    for region in layout.allowed_regions:
        if padded.overlaps(region):
            raise LayoutError(f"factory context fixture {stable_id} violates AMR region 0.1m clearance")
    obstacles = {
        str(item["id"]): item for item in layout.data["physical_obstacles"]
    }
    if source_obstacle_id is not None and source_obstacle_id not in obstacles:
        raise LayoutError(
            f"factory context fixture {stable_id} references missing source obstacle {source_obstacle_id!r}"
        )
    source_bounds_xy = None
    for raw_obstacle in layout.data["physical_obstacles"]:
        obstacle_id = str(raw_obstacle["id"])
        obstacle = Rect.from_value(raw_obstacle["rect"], f"obstacle {obstacle_id}")
        if obstacle_id == source_obstacle_id:
            source_bounds = (
                obstacle.xmin, obstacle.ymin, 0.0,
                obstacle.xmax, obstacle.ymax, float(raw_obstacle["height_m"]),
            )
            if not _contains_bounds(source_bounds, world_bounds):
                raise LayoutError(
                    f"factory context fixture {stable_id} exceeds source obstacle "
                    f"{source_obstacle_id} XYZ envelope"
                )
            source_bounds_xy = (obstacle.xmin, obstacle.ymin, obstacle.xmax, obstacle.ymax)
            continue
        if padded.overlaps(obstacle):
            raise LayoutError(f"factory context fixture {stable_id} violates obstacle 0.1m clearance")
    for other_id, other in occupied:
        if padded.overlaps(other):
            raise LayoutError(f"factory context fixtures {stable_id} and {other_id} overlap")
    return source_bounds_xy


def _contains_bounds(
    outer: tuple[float, float, float, float, float, float],
    inner: tuple[float, float, float, float, float, float],
) -> bool:
    return all(inner[index] >= outer[index] for index in range(3)) and all(
        inner[index] <= outer[index] for index in range(3, 6)
    )


def _world_bounds(bounds: tuple[float, float, float, float, float, float], cx: float, cy: float,
                  yaw: int) -> tuple[float, float, float, float, float, float]:
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    corners = ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax))
    angle = math.radians(yaw)
    cosine, sine = round(math.cos(angle)), round(math.sin(angle))
    points = [(cx + cosine * x - sine * y, cy + sine * x + cosine * y) for x, y in corners]
    return (
        min(point[0] for point in points), min(point[1] for point in points), zmin,
        max(point[0] for point in points), max(point[1] for point in points), zmax,
    )


def _package_record(asset_id: str, repository: str, revision: str, catalog: Mapping[str, Any],
                    prepared: Mapping[str, Any], path: Path, sha256: str) -> dict[str, object]:
    source_files = prepared.get("source_files")
    if not isinstance(source_files, list):
        raise LayoutError(f"prepared asset {asset_id}.source_files must be a list")
    summaries: list[dict[str, object]] = []
    for ordinal, raw in enumerate(source_files):
        source = _mapping(raw, f"prepared asset {asset_id}.source_files[{ordinal}]")
        source_path = _string(source.get("path"), f"source file {ordinal}.path")
        source_sha = _string(source.get("sha256"), f"source file {ordinal}.sha256")
        size = source.get("size")
        if not _HEX64.fullmatch(source_sha) or isinstance(size, bool) or not isinstance(size, int) or size < 0:
            raise LayoutError(f"prepared asset {asset_id} has invalid source file provenance")
        summaries.append({"path": source_path, "sha256": source_sha, "size": size})
    return {
        "asset_id": asset_id,
        "repository": repository,
        "revision": revision,
        "package": _string(catalog.get("package"), f"catalog.{asset_id}.package"),
        "root_usd": _string(catalog.get("root_usd"), f"catalog.{asset_id}.root_usd"),
        "author": _string(catalog.get("author"), f"catalog.{asset_id}.author"),
        "license": _string(catalog.get("license"), f"catalog.{asset_id}.license"),
        "license_url": _string(catalog.get("license_url"), f"catalog.{asset_id}.license_url"),
        "source_url": _string(catalog.get("source_url"), f"catalog.{asset_id}.source_url"),
        "prepared_path": str(path),
        "sha256": sha256,
        "source_files": summaries,
        "scope": prepared["scope"],
    }


def _bounds(value: object, label: str) -> tuple[float, float, float, float, float, float]:
    if not isinstance(value, list) or len(value) != 6:
        raise LayoutError(f"{label} must contain six numbers")
    result = tuple(_number(item, f"{label}[{index}]") for index, item in enumerate(value))
    if result[3] <= result[0] or result[4] <= result[1] or result[5] <= result[2]:
        raise LayoutError(f"{label} must be ordered")
    return result  # type: ignore[return-value]


def _require_plain(path: Path, label: str, *, file: bool = False, directory: bool = False) -> None:
    absolute = path.absolute()
    chain = [absolute, *absolute.parents]
    for candidate in chain:
        if candidate.exists() and _is_link_or_reparse(candidate):
            raise LayoutError(f"{label} must not use symlinks or reparse points")
    if file and not absolute.is_file():
        raise LayoutError(f"{label} must be a regular file")
    if directory and not absolute.is_dir():
        raise LayoutError(f"{label} must be a directory")


def _is_link_or_reparse(path: Path) -> bool:
    try:
        stat = path.lstat()
    except OSError:
        return False
    return path.is_symlink() or bool(getattr(stat, "st_file_attributes", 0) & 0x400)


def _load_json(path: Path, label: str) -> dict[str, Any]:
    _require_plain(path, label, file=True)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LayoutError(f"invalid {label}: {exc}") from exc
    return dict(_mapping(value, label))


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise LayoutError(f"{label} must be an object")
    return value


def _string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or "\x00" in value or "\r" in value or "\n" in value:
        raise LayoutError(f"{label} must be a non-empty single-line string")
    return value


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise LayoutError(f"{label} must be a finite number")
    return float(value)


def _prim_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return "_" + result if result[0].isdigit() else result


def _usd_asset_path(path: Path) -> str:
    value = path.absolute().as_posix()
    if any(character in value for character in ("\x00", "\r", "\n")):
        raise LayoutError("prepared asset path is not safe for USDA")
    return value.replace("@", "@@")


def _quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def _num(value: float) -> str:
    return str(int(value)) if value == int(value) else format(value, ".12g")
