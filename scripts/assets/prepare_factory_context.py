"""Fetch pinned NVIDIA props and prepare a metre/Z-up visual-only USD cache.

Uses the existing Isaac Sim SDK's USD libraries on the CPU; never starts Kit,
SimulationApp, ROS, a renderer or a GPU job. Vendor files stay outside Git.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import sys
from urllib.parse import quote
from urllib.request import Request, urlopen


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def read_url(url: str):
    return urlopen(Request(url, headers={"User-Agent": "FactoryOpsTwin-AssetPreparation/1"}), timeout=45)


def package_files(repository: str, revision: str, package: str) -> list[dict]:
    url = f"https://huggingface.co/api/datasets/{repository}/tree/{revision}/{quote(package)}?recursive=true&limit=1000"
    with read_url(url) as response:
        if response.headers.get("Link"):
            raise ValueError("asset package listing is larger than the bounded one-page inventory")
        rows = json.load(response)
    result = []
    for row in rows:
        if row["type"] != "file":
            continue
        name = PurePosixPath(row["path"])
        if name.parts[0] != package or ".." in name.parts or name.is_absolute():
            raise ValueError("asset package path escapes its declared root")
        relative = name.relative_to(package)
        if ".thumbs" in relative.parts and relative.as_posix() != f"simready_usd/.thumbs/256x256/sm_{package}_01.usd.png":
            continue
        if relative.parts[0] not in {"simready_usd", ".metadata"} and relative.name != "com.nvidia.simready.packaging.json":
            continue
        result.append(row)
    if not result or len(result) > 400:
        raise ValueError("asset runtime inventory is empty or exceeds the bounded package size")
    return result


def fetch_file(root: Path, row: dict, repository: str, revision: str) -> dict:
    target = root.joinpath(*PurePosixPath(row["path"]).parts)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink() or not target.resolve().is_relative_to(root.resolve()):
        raise ValueError("asset destination must remain inside the cache")
    if not target.exists():
        temporary = target.with_name(target.name+".download")
        if temporary.exists():
            raise ValueError(f"incomplete prior download requires inspection: {temporary}")
        url = f"https://huggingface.co/datasets/{repository}/resolve/{revision}/{quote(row['path'])}?download=true"
        with read_url(url) as response, temporary.open("xb") as stream:
            shutil.copyfileobj(response, stream, length=1024*1024)
        temporary.replace(target)
    if not target.is_file() or target.stat().st_size != row["size"]:
        raise ValueError(f"asset byte size differs: {row['path']}")
    actual = digest(target)
    lfs = row.get("lfs")
    if lfs:
        if actual != lfs["oid"]:
            raise ValueError(f"asset LFS SHA256 differs: {row['path']}")
    else:
        content = target.read_bytes()
        if hashlib.sha1(f"blob {len(content)}\0".encode()+content).hexdigest() != row["oid"]:
            raise ValueError(f"asset Git object hash differs: {row['path']}")
    return {"path": row["path"], "size": row["size"], "sha256": actual}


def load_usd(sdk: Path):
    candidates = list((sdk/"extscache").glob("omni.usd.libs-*/pxr/Usd/__init__.py"))
    if len(candidates) != 1:
        raise ValueError("existing SDK USD library is missing or ambiguous")
    library = candidates[0].parents[2]
    handles = [os.add_dll_directory(str(p)) for p in (library/"bin", library/"bin/deps", library/"bin/usd", sdk/"kit") if p.is_dir()]
    sys.path.insert(0, str(library))
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    return Usd, UsdGeom, UsdPhysics, Sdf, Gf, handles


def physics_typed_paths(stage, UsdPhysics) -> list:
    """Return active or inactive prims whose authored type is physical."""
    return [prim.GetPath() for prim in stage.TraverseAll()
        if prim.IsA(UsdPhysics.Joint)
        or prim.GetTypeName().lower().startswith(("physics", "physx"))]


def remove_physics_typed_prims(stage, UsdPhysics) -> list[str]:
    """Remove leaf physics schema prims from a flattened consumer stage."""
    paths = physics_typed_paths(stage, UsdPhysics)
    for path in sorted(paths, key=lambda value: len(str(value)), reverse=True):
        prim = stage.GetPrimAtPath(path)
        if list(prim.GetAllChildren()):
            raise ValueError(f"physics typed prim has children; refusing visual subtree removal: {path}")
        if not stage.RemovePrim(path) or stage.GetPrimAtPath(path).IsValid():
            raise ValueError(f"physics typed prim could not be removed from flattened stage: {path}")
    return [str(path) for path in paths]


def prepare_asset(root: Path, asset_id: str, definition: dict, files: list[dict], sdk: Path, preparer: str) -> dict:
    Usd, UsdGeom, UsdPhysics, Sdf, Gf, handles = load_usd(sdk)
    raw = root/definition["package"]/definition["root_usd"]
    source = Usd.Stage.Open(str(raw))
    if source is None or not source.GetDefaultPrim().IsValid():
        raise ValueError(f"asset has no usable default prim: {asset_id}")
    units = UsdGeom.GetStageMetersPerUnit(source)
    up_axis = UsdGeom.GetStageUpAxis(source)
    if not math.isfinite(units) or units <= 0 or up_axis not in ("Y", "Z"):
        raise ValueError("asset stage units/up-axis are unsupported")
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")
    wrapper = UsdGeom.Xform.Define(stage, "/Asset")
    stage.SetDefaultPrim(wrapper.GetPrim())
    normalization = UsdGeom.Xform.Define(stage, "/Asset/Units")
    if up_axis == "Y":
        normalization.AddRotateXOp().Set(90.)
    normalization.AddScaleOp().Set(Gf.Vec3f(units, units, units))
    model = UsdGeom.Xform.Define(stage, "/Asset/Units/Model").GetPrim()
    model.GetReferences().AddReference(str(raw), source.GetDefaultPrim().GetPath())
    variants = {}
    for prim in list(stage.Traverse()):
        for name in prim.GetVariantSets().GetNames():
            v = prim.GetVariantSet(name)
            if "Disabled" in v.GetVariantNames() and any(token in name.lower() for token in ("phys", "mujoco", "newton")):
                v.SetVariantSelection("Disabled")
                variants[f"{prim.GetPath()}:{name}"] = "Disabled"
    for _ in range(16):
        instanced = [p for p in stage.Traverse() if p.IsInstanceable()]
        if not instanced:
            break
        for prim in instanced:
            prim.SetInstanceable(False)
    else:
        raise ValueError("asset instance nesting exceeds preparation bound")
    removed = 0
    for prim in stage.Traverse():
        schemas = list(prim.GetAppliedSchemas())
        retained = [s for s in schemas if "physics" not in s.lower() and "physx" not in s.lower()]
        if retained != schemas:
            removed += len(schemas)-len(retained)
            prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(retained))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    bound = cache.ComputeWorldBound(wrapper.GetPrim()).ComputeAlignedBox()
    lo, hi = bound.GetMin(), bound.GetMax()
    if any(not math.isfinite(float(value)) for value in [*lo, *hi]) or any(hi[i] <= lo[i] for i in range(3)):
        raise ValueError("asset has no finite visible geometry bounds")
    wrapper.AddTranslateOp().Set(Gf.Vec3d(-(lo[0]+hi[0])/2, -(lo[1]+hi[1])/2, -lo[2]))
    destination = root/".prepared"/preparer[:16]/f"{asset_id}.usda"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise ValueError("prepared asset is immutable; use its existing index or a new preparer revision")
    flattened = Usd.Stage.Open(stage.Flatten())
    removed_typed = remove_physics_typed_prims(flattened, UsdPhysics)
    flattened.GetRootLayer().Export(str(destination))
    check = Usd.Stage.Open(str(destination))
    if any("physics" in s.lower() or "physx" in s.lower() for p in check.Traverse() for s in p.GetAppliedSchemas()):
        raise ValueError("prepared background asset still has physics APIs")
    remaining_typed = physics_typed_paths(check, UsdPhysics)
    if remaining_typed:
        raise ValueError("prepared background asset still has physics typed prims: "
            + ", ".join(str(path) for path in remaining_typed))
    measured = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(check.GetDefaultPrim()).ComputeAlignedBox()
    bounds = [float(x) for x in measured.GetMin()]+[float(x) for x in measured.GetMax()]
    mesh_count = sum(p.GetTypeName() == "Mesh" for p in check.Traverse())
    if mesh_count < 1:
        raise ValueError("prepared asset contains no render mesh")
    return {"relative_path": destination.relative_to(root).as_posix(), "sha256": digest(destination),
        "bounds_xyz_m": bounds, "mesh_count": mesh_count, "source_units_m": units, "source_up_axis": up_axis,
        "disabled_variants": variants, "removed_physics_api_count": removed,
        "removed_physics_typed_prims": removed_typed,
        "scope": "render-only; physics APIs and typed prims removed", "source_files": files,
        "license": definition["license"], "author": definition["author"], "source_url": definition["source_url"],
        "license_url": definition["license_url"]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    args = parser.parse_args()
    environment = json.loads(args.config.read_text(encoding="utf-8"))["environment"]
    revision, repository = environment["revision"], environment["repository"]
    if re.fullmatch(r"[0-9a-f]{40}", revision) is None or repository != "nvidia/simready-assets":
        raise ValueError("only the pinned public NVIDIA catalog is supported")
    cache = args.cache_root.resolve()/revision
    cache.mkdir(parents=True, exist_ok=True)
    if args.cache_root.is_symlink() or cache.is_symlink():
        raise ValueError("asset cache must not use symlinks")
    preparer = digest(Path(__file__))
    catalog_sha = hashlib.sha256(json.dumps(environment["catalog"], sort_keys=True).encode()).hexdigest()
    index_path = cache/"prepared-index.json"
    completed = {}
    if index_path.exists():
        previous = json.loads(index_path.read_text(encoding="utf-8"))
        if previous.get("preparer_sha256") == preparer and previous.get("catalog_sha256") == catalog_sha:
            for entry in previous["assets"].values():
                if digest(cache/entry["relative_path"]) != entry["sha256"]:
                    raise ValueError("existing prepared asset changed")
                for source_file in entry["source_files"]:
                    if digest(cache/source_file["path"]) != source_file["sha256"]:
                        raise ValueError("existing source asset changed")
            completed = previous["assets"]
            if previous.get("status") == "passed" and set(completed) == set(environment["catalog"]):
                print(json.dumps({"status": "reused", "index": str(index_path)}), flush=True)
                return
    inventory = {key: package_files(repository, revision, value["package"]) for key, value in environment["catalog"].items()}
    total = sum(row["size"] for rows in inventory.values() for row in rows)
    if total > 2_000_000_000 or shutil.disk_usage(cache).free < total+8_000_000_000:
        raise ValueError("selected runtime assets exceed the authored download/free-space budget")
    print(json.dumps({"status": "fetching", "bytes": total, "assets": list(inventory), "GPU_started": False}), flush=True)
    result = {"schema_version": 1, "revision": revision, "repository": repository,
        "preparer_sha256": preparer, "catalog_sha256": catalog_sha, "started_at": datetime.now(timezone.utc).isoformat(),
        "scope": "CPU preparation of attributed visual context; no simulation or navigation qualification",
        "GPU_started": False, "assets": {}}
    for asset_id, definition in environment["catalog"].items():
        files = [fetch_file(cache, row, repository, revision) for row in inventory[asset_id]]
        result["assets"][asset_id] = completed.get(asset_id) or prepare_asset(cache, asset_id, definition, files, args.sdk, preparer)
        temporary = index_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
        temporary.replace(index_path)
        print(json.dumps({"status": "prepared", "asset": asset_id, "bounds_m": result["assets"][asset_id]["bounds_xyz_m"], "files": len(files)}), flush=True)
    result["status"] = "passed"
    index_path.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"status": "passed", "index": str(index_path)}), flush=True)


if __name__ == "__main__":
    main()
