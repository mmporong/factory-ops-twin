#!/usr/bin/env python3
"""Prepare immutable semantic-free copies of pinned perception candidate USDs.

This CPU-only tool consumes the existing render-only preparation result. It
never edits the vendor cache or the generic ``.prepared`` assets.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Mapping


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.assets.prepare_factory_context import load_usd, physics_typed_paths


EXPECTED_ASSETS = {"nvidia:pallet_block_a01", "nvidia:crate_gray_a20"}


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    temporary.replace(path)


def _is_semantic_name(value: str) -> bool:
    return value.lower().startswith("semantic")


def _is_semantic_schema(value: str) -> bool:
    return "semantic" in value.lower()


def _stable(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("USD contract contains a non-finite float")
        return value
    if isinstance(value, Mapping):
        return {str(key): _stable(item) for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))}
    if isinstance(value, (list, tuple)):
        return [_stable(item) for item in value]
    return {"type": type(value).__name__, "text": str(value)}


def nonsemantic_contract(stage: Any) -> dict[str, Any]:
    """Hash all composed prim/property state except the intentionally removed semantics."""
    hasher = hashlib.sha256()
    prim_count = property_count = 0
    for prim in sorted(stage.TraverseAll(), key=lambda item: str(item.GetPath())):
        prim_count += 1
        metadata = {
            key: _stable(prim.GetMetadata(key))
            for key in sorted(prim.GetAllMetadata())
            if key != "apiSchemas" and not _is_semantic_name(key)
        }
        schemas = sorted(schema for schema in prim.GetAppliedSchemas() if not _is_semantic_schema(schema))
        record = {
            "kind": "prim", "path": str(prim.GetPath()), "type": prim.GetTypeName(),
            "active": prim.IsActive(), "instanceable": prim.IsInstanceable(),
            "schemas": schemas, "metadata": metadata,
        }
        hasher.update((json.dumps(record, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8"))
        properties = [("attribute", prop) for prop in prim.GetAttributes()]
        properties.extend(("relationship", prop) for prop in prim.GetRelationships())
        for kind, prop in sorted(properties, key=lambda item: item[1].GetName()):
            if _is_semantic_name(prop.GetName()):
                continue
            property_count += 1
            prop_metadata = {key: _stable(prop.GetMetadata(key)) for key in sorted(prop.GetAllMetadata())}
            if kind == "relationship":
                payload = {"targets": [str(path) for path in prop.GetTargets()]}
                prop_type = "relationship"
            else:
                samples = {str(time): _stable(prop.Get(time)) for time in prop.GetTimeSamples()}
                payload = {
                    "default": _stable(prop.Get()), "connections": [str(path) for path in prop.GetConnections()],
                    "time_samples": samples,
                }
                prop_type = str(prop.GetTypeName())
            record = {
                "kind": "property", "path": str(prop.GetPath()), "type": prop_type,
                "metadata": prop_metadata, "payload": payload,
            }
            hasher.update((json.dumps(record, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8"))
    return {"sha256": hasher.hexdigest(), "prim_count": prim_count, "property_count": property_count}


def semantic_inventory(stage: Any) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {"api_schemas": [], "metadata": [], "properties": []}
    for prim in sorted(stage.TraverseAll(), key=lambda item: str(item.GetPath())):
        path = str(prim.GetPath())
        for schema in prim.GetAppliedSchemas():
            if _is_semantic_schema(schema):
                result["api_schemas"].append({
                    "path": path, "name": "apiSchemas", "type": "applied_api_schema", "data": schema,
                })
        for name in sorted(prim.GetAllMetadata()):
            if name != "apiSchemas" and _is_semantic_name(name):
                result["metadata"].append({
                    "path": path, "name": name, "type": "metadata", "data": _stable(prim.GetMetadata(name)),
                })
        properties = [("attribute", prop) for prop in prim.GetAttributes()]
        properties.extend(("relationship", prop) for prop in prim.GetRelationships())
        for kind, prop in sorted(properties, key=lambda item: item[1].GetName()):
            if not _is_semantic_name(prop.GetName()):
                continue
            if kind == "relationship":
                prop_type = "relationship"
                data = {"targets": [str(target) for target in prop.GetTargets()]}
            else:
                prop_type = str(prop.GetTypeName())
                data = {"default": _stable(prop.Get())}
            result["properties"].append({
                "path": str(prop.GetPath()), "prim_path": path,
                "name": prop.GetName(), "type": prop_type, "data": data,
            })
    return result


def remove_semantics(stage: Any, Sdf: Any) -> dict[str, list[dict[str, Any]]]:
    removed = semantic_inventory(stage)
    for prim in list(stage.TraverseAll()):
        schemas = list(prim.GetAppliedSchemas())
        retained = [schema for schema in schemas if not _is_semantic_schema(schema)]
        if retained != schemas:
            prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(retained))
        for name in list(prim.GetAllMetadata()):
            if name != "apiSchemas" and _is_semantic_name(name):
                prim.ClearMetadata(name)
        for prop in list(prim.GetProperties()):
            if _is_semantic_name(prop.GetName()) and not prim.RemoveProperty(prop.GetName()):
                raise ValueError(f"semantic property could not be removed: {prop.GetPath()}")
    remaining = semantic_inventory(stage)
    if any(remaining.values()):
        raise ValueError(f"semantic state remains after removal: {remaining}")
    return removed


def geometry_contract(stage: Any, Usd: Any, UsdGeom: Any) -> dict[str, Any]:
    default = stage.GetDefaultPrim()
    if not default.IsValid():
        raise ValueError("perception asset has no default prim")
    measured = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(default).ComputeAlignedBox()
    bounds = [float(value) for value in measured.GetMin()] + [float(value) for value in measured.GetMax()]
    if any(not math.isfinite(value) for value in bounds):
        raise ValueError("perception asset has non-finite bounds")
    return {"bounds_xyz_m": bounds, "mesh_count": sum(prim.GetTypeName() == "Mesh" for prim in stage.TraverseAll())}


def assert_geometry(expected: Mapping[str, Any], actual: Mapping[str, Any]) -> None:
    if actual["mesh_count"] != expected.get("mesh_count"):
        raise ValueError("perception copy mesh count differs from the generic prepared asset")
    expected_bounds = expected.get("bounds_xyz_m")
    if not isinstance(expected_bounds, list) or len(expected_bounds) != 6:
        raise ValueError("generic prepared asset has no valid bounds contract")
    if any(not math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=1e-9) for a, b in zip(actual["bounds_xyz_m"], expected_bounds)):
        raise ValueError("perception copy bounds differ from the generic prepared asset")


def asset_filename(asset_id: str) -> str:
    leaf = asset_id.split(":", 1)[-1]
    if re.fullmatch(r"[A-Za-z0-9._-]+", leaf) is None or ":" in leaf:
        raise ValueError(f"asset ID has no Windows-safe leaf: {asset_id}")
    return leaf + ".usda"


def prepare_asset(
    *, cache_root: Path, asset_id: str, source_definition: Mapping[str, Any], destination: Path,
    Usd: Any, UsdGeom: Any, UsdPhysics: Any, Sdf: Any,
) -> dict[str, Any]:
    source = cache_root / str(source_definition["relative_path"])
    if not source.is_file() or digest(source) != source_definition.get("sha256"):
        raise ValueError(f"generic prepared asset is missing or changed: {source}")
    source_stage = Usd.Stage.Open(str(source))
    if source_stage is None:
        raise ValueError(f"generic prepared asset cannot be opened: {source}")
    stage = Usd.Stage.Open(source_stage.Flatten())
    before = nonsemantic_contract(stage)
    removed = remove_semantics(stage, Sdf)
    after_removal = nonsemantic_contract(stage)
    if after_removal != before:
        raise ValueError("non-semantic USD contract changed while removing semantics")
    if destination.exists():
        raise ValueError(f"perception asset is immutable; destination already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not stage.GetRootLayer().Export(str(destination)):
        raise ValueError(f"perception asset could not be exported: {destination}")
    check = Usd.Stage.Open(str(destination))
    if check is None:
        raise ValueError(f"exported perception asset cannot be reopened: {destination}")
    if any(semantic_inventory(check).values()):
        raise ValueError("exported perception asset still contains semantic state")
    after_export = nonsemantic_contract(check)
    if after_export != before:
        raise ValueError("non-semantic USD contract changed during perception export")
    physics_api = [
        {"prim_path": str(prim.GetPath()), "schema": schema}
        for prim in check.TraverseAll() for schema in prim.GetAppliedSchemas()
        if "physics" in schema.lower() or "physx" in schema.lower()
    ]
    physics_typed = [str(path) for path in physics_typed_paths(check, UsdPhysics)]
    if physics_api or physics_typed:
        raise ValueError(f"exported perception asset contains physics state: api={physics_api}, typed={physics_typed}")
    geometry = geometry_contract(check, Usd, UsdGeom)
    assert_geometry(source_definition, geometry)
    result = deepcopy(dict(source_definition))
    result.update({
        "relative_path": destination.relative_to(cache_root).as_posix(),
        "sha256": digest(destination),
        **geometry,
        "removed_semantics": removed,
        "removed_semantic_api_count": len(removed["api_schemas"]),
        "removed_semantic_metadata_count": len(removed["metadata"]),
        "removed_semantic_property_count": len(removed["properties"]),
        "semantic_state_remaining_count": 0,
        "nonsemantic_contract": before,
        "source_prepared_relative_path": source_definition["relative_path"],
        "source_prepared_sha256": source_definition["sha256"],
        "scope": "render-only perception copy; native semantics and physics absent",
    })
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepared-result", type=Path, required=True, help="absolute path to result 267 JSON")
    parser.add_argument("--cache-root", type=Path, required=True, help="absolute pinned revision cache root")
    parser.add_argument("--sdk", type=Path, required=True, help="absolute Isaac Sim 4.5 SDK root")
    parser.add_argument("--output", type=Path, required=True, help="new artifact directory for result.json")
    args = parser.parse_args()
    for value, label in ((args.prepared_result, "prepared result"), (args.cache_root, "cache root"), (args.sdk, "SDK"), (args.output, "output")):
        if not value.is_absolute():
            raise ValueError(f"{label} path must be absolute")
    prepared_path = args.prepared_result.resolve(strict=True)
    cache_root = args.cache_root.resolve(strict=True)
    sdk = args.sdk.resolve(strict=True)
    output = args.output.resolve(strict=False)
    if output.exists():
        raise ValueError("--output must be a new directory")
    prepared = json.loads(prepared_path.read_text(encoding="utf-8"))
    if prepared.get("status") != "passed" or prepared.get("training_ready") is not False or prepared.get("mapping_approved") is not False:
        raise ValueError("input is not the immutable candidate-only preparation result")
    if set(prepared.get("assets", {})) != EXPECTED_ASSETS:
        raise ValueError("input must contain exactly the two pinned perception candidates")
    preparer_sha = digest(Path(__file__).resolve())
    destination_root = cache_root / ".perception" / preparer_sha[:16]
    if destination_root.exists():
        raise ValueError("perception preparation is immutable; preparer destination already exists")
    output.mkdir(parents=True)
    Usd, UsdGeom, UsdPhysics, Sdf, _Gf, _handles = load_usd(sdk)
    assets = {}
    for asset_id in sorted(EXPECTED_ASSETS):
        assets[asset_id] = prepare_asset(
            cache_root=cache_root, asset_id=asset_id, source_definition=prepared["assets"][asset_id],
            destination=destination_root / asset_filename(asset_id),
            Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics, Sdf=Sdf,
        )
    result = {
        "schema_version": 1,
        "status": "passed",
        "scope": "CPU OpenUSD preparation of semantic-free render-only perception candidates; no Kit or GPU render",
        "training_ready": False,
        "mapping_approved": False,
        "source_receipt_sha256": prepared["source_receipt_sha256"],
        "source_prepared_result_path": str(prepared_path),
        "source_prepared_result_sha256": digest(prepared_path),
        "preparer_sha256": preparer_sha,
        "assets": assets,
    }
    atomic_json(output / "result.json", result)
    print(json.dumps({"status": "passed", "result": str(output / "result.json"), "assets": sorted(assets)}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
