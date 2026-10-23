"""Deterministic, collision-free industrial visual layer for the factory twin."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

from .layout import (
    DUAL_LANE_REVISION,
    FLEET4_LAYOUT_REVISIONS,
    PRODUCTION_BAY_REVISIONS,
    Layout,
    LayoutError,
    Rect,
)


SCHEMA_VERSION = 1
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "visual-assets.json"
DEFAULT_RENDER_PROFILE = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "render-profile.json"
PRODUCTION_BAY_GEOMETRY = "production_bay_v4"
_STENCIL_GLYPHS = {
    "1": ("01100", "11100", "01100", "01100", "01100", "01100", "11111"),
    "2": ("11110", "00001", "00001", "11110", "10000", "10000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "4": ("10010", "10010", "10010", "11111", "00010", "00010", "00010"),
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "W": ("10001", "10001", "10001", "10101", "10101", "10101", "01010"),
    "X": ("10001", "10001", "01010", "00100", "01010", "10001", "10001"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
}


@dataclass(frozen=True)
class VisualInstance:
    stable_id: str
    prim_path: str
    semantic_class: str
    state_role: str
    source_obstacle_id: str | None
    bounds_xyz_m: tuple[float, float, float, float, float, float]
    source_bounds_xy_m: tuple[float, float, float, float] | None
    asset: str


@dataclass(frozen=True)
class _Box:
    name: str
    bounds: tuple[float, float, float, float, float, float]
    material: str
    semantic_class: str | None = None
    state_role: str | None = None
    render_style: str = "cube"
    render_chamfer_m: float = 0.0


def export_visual_scene(
    layout: Layout,
    output_dir: str | Path,
    variant_seed: int = 0,
    config_path: str | Path | None = None,
    render_profile_path: str | Path | None = None,
) -> dict[str, object]:
    """Author a visual-only USD layer and a composition over ``factory-v3.usda``.

    The visual layer never authors collision APIs. Equipment geometry remains
    inside the source obstacle's authored XYZ envelope. Optional background
    props occupy measured floor footprints outside all AMR operating regions.
    """
    if isinstance(variant_seed, bool) or not isinstance(variant_seed, int) or not 0 <= variant_seed < 12:
        raise LayoutError("variant_seed must be an integer in [0, 11]")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    base_scene = output / "factory-v3.usda"
    _require_regular_file(base_scene, "base scene")

    config_source = Path(config_path) if config_path is not None else DEFAULT_CONFIG
    config_bytes, config = _load_json_object(config_source, "visual asset config")
    render_source = Path(render_profile_path) if render_profile_path is not None else DEFAULT_RENDER_PROFILE
    render_bytes, render_profile = _load_json_object(render_source, "render profile")
    _validate_config(config, layout)
    _validate_render_profile(render_profile)
    geometry_profile = PRODUCTION_BAY_GEOMETRY if layout.data.get("revision") in PRODUCTION_BAY_REVISIONS else "legacy"

    context = None
    if config.get("environment", {}).get("fixtures"):
        from .factory_context import build_factory_context
        context = build_factory_context(layout, config["environment"], cache_root=Path(
            os.environ.get("FACTORY_OPS_ASSET_CACHE", str(Path(__file__).resolve().parents[1]/"assets/vendor/simready-context"))))
    external_by_obstacle = _external_asset_mapping(config, context)

    palette = _palette(variant_seed)
    obstacle_by_id = {str(item["id"]): item for item in layout.data["physical_obstacles"]}
    instances: list[VisualInstance] = []
    lines = _visual_header(variant_seed, palette)
    lines.extend(_materials(palette, semiconductor=layout.data.get("production_domain", {}).get("id") == "semiconductor_final_test_pack"))
    lines.extend(_factory_shell(layout, palette))
    floor_semantic_roots: dict[str, str] = {}
    lines.extend(_floor_graphics(layout, palette, semantic_roots=floor_semantic_roots))
    assignments = config["obstacle_assets"]
    assert isinstance(assignments, dict)
    for obstacle_id in sorted(obstacle_by_id):
        obstacle = obstacle_by_id[obstacle_id]
        assignment = assignments[obstacle_id]
        assert isinstance(assignment, dict)
        rect = Rect.from_value(obstacle["rect"], f"obstacle {obstacle_id}")
        height = float(obstacle["height_m"])
        source_bounds = (rect.xmin, rect.ymin, rect.xmax, rect.ymax)
        bounds = (rect.xmin, rect.ymin, 0.0, rect.xmax, rect.ymax, height)
        recipe = str(assignment["recipe"])
        semantic_class = str(assignment["semantic_class"])
        state_role = str(assignment["state_role"])
        if obstacle_id in external_by_obstacle:
            continue
        prim_path = f"/World/Visuals/{_prim_name(obstacle_id)}"
        instances.append(VisualInstance(
            stable_id=obstacle_id,
            prim_path=prim_path,
            semantic_class=semantic_class,
            state_role=state_role,
            source_obstacle_id=obstacle_id,
            bounds_xyz_m=bounds,
            source_bounds_xy_m=source_bounds,
            asset=recipe,
        ))
        boxes = _recipe_boxes(
            recipe, rect, height, variant_seed, obstacle_id,
            geometry_profile=geometry_profile,
        )
        _validate_boxes(obstacle_id, boxes, bounds)
        lines.extend(_equipment_root(obstacle_id, semantic_class, state_role, recipe, boxes, indent=8))
        for box in boxes:
            if box.semantic_class is None:
                continue
            component_path = f"{prim_path}/{_prim_name(box.name)}"
            instances.append(VisualInstance(
                stable_id=f"{obstacle_id}:{box.name}",
                prim_path=component_path,
                semantic_class=box.semantic_class,
                state_role=box.state_role or state_role,
                source_obstacle_id=obstacle_id,
                bounds_xyz_m=box.bounds,
                source_bounds_xy_m=source_bounds,
                asset=f"{recipe}/{box.name}",
            ))
    if context is not None:
        lines.extend(context["lines"])
        instances.extend(VisualInstance(**item) for item in context["instances"])
    lines.extend(["    }", "}", ""])
    visual_text = "\n".join(lines)
    if "PhysicsCollisionAPI" in visual_text or "physics:collisionEnabled" in visual_text:
        raise LayoutError("visual layer must not author collision schemas or attributes")

    visual_path = output / "factory-v3.visual.usda"
    visual_path.write_text(visual_text, encoding="utf-8", newline="\n")
    composition_path = output / "factory-v3.render.usda"
    composition_path.write_text(_composition_text(obstacle_by_id), encoding="utf-8", newline="\n")

    output_hashes = {
        visual_path.name: _sha256(visual_path.read_bytes()),
        composition_path.name: _sha256(composition_path.read_bytes()),
    }
    config_sha = _sha256(config_bytes)
    render_sha = _sha256(render_bytes)
    manifest: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "variant": {"id": f"industrial-v{variant_seed:02d}", "seed": variant_seed, "count": 12},
        "layout_sha256": layout.source_sha256,
        "visual_asset_config": {"path": str(config_source), "sha256": config_sha},
        "render_profile": {"path": str(render_source), "sha256": render_sha, "value": render_profile},
        "outputs": output_hashes,
        "composition": {
            "path": composition_path.name,
            "base_scene": base_scene.name,
            "visual_layer": visual_path.name,
            "base_obstacle_rendering": "hidden by composition overs; collision prims remain composed",
        },
        "asset_provenance": {
            "provider": config["provider"],
            "author": config["author"],
            "license": config["license"],
            "source": config["source"],
            "asset_source_sha256": config_sha,
            "external_asset_packages": context["packages"] if context else [],
        },
        "semantic_taxonomy": config["required_semantic_taxonomy"],
        "runtime_semantics": config["runtime_semantics"],
        "floor_semantic_roots": dict(sorted(floor_semantic_roots.items())),
        "instances": [_instance_document(instance) for instance in instances],
        "physical_invariants": {
            "source_obstacle_count": 19,
            "source_static_collider_count": 20,
            "source_marker_count": len(layout.stations) + len(layout.pockets),
            "visual_collision_api_count": 0,
            "layout_bounds_xy_m": [layout.bounds.xmin, layout.bounds.ymin, layout.bounds.xmax, layout.bounds.ymax],
            "visual_equipment_contained_by_source_xyz_envelopes": True,
            "station_centres_free_of_visual_payload_geometry": True,
        },
        "variation_contract": {
            "changes": ["material palette", "light temperature", "prop order inside source obstacle envelopes"],
            "unchanged": ["source obstacle bounds", "physical colliders", "aisles", "stations", "LiDAR reference envelopes", "instance stable IDs"],
        },
    }
    if context is not None:
        manifest["physical_invariants"]["source_xyz_invariant_scope"] = "all visual instances with a source_obstacle_id, including mounted external assets, remain inside source XYZ envelopes"
        manifest["factory_context"] = {"scope": context["scope"], "fixture_count": len(context["instances"]),
            "placement": "native asset dimensions; mounted assets inside source XYZ envelopes; all fixtures preserve 0.1m AMR and unrelated obstacle clearance",
            "physics": "render-only references; source obstacle collision geometry remains authoritative"}
    manifest_path = output / "visual-scene-manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    result = dict(manifest)
    result["manifest"] = {"path": manifest_path.name, "sha256": _sha256(manifest_path.read_bytes())}
    return result


def _load_json_object(path: Path, label: str) -> tuple[bytes, dict[str, Any]]:
    _require_regular_file(path, label)
    source = path.read_bytes()
    try:
        value = json.loads(source)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise LayoutError(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise LayoutError(f"{label} root must be an object")
    return source, value


def _instance_document(instance: VisualInstance) -> dict[str, object]:
    document: dict[str, object] = asdict(instance)
    document["bounds_xyz_m"] = list(instance.bounds_xyz_m)
    document["source_bounds_xy_m"] = None if instance.source_bounds_xy_m is None else list(instance.source_bounds_xy_m)
    return document


def _require_regular_file(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise LayoutError(f"{label} must be a regular non-symlink file: {path}")


def _validate_config(config: Mapping[str, Any], layout: Layout) -> None:
    required = {"schema_version", "provider", "author", "license", "source", "variant_count", "required_semantic_taxonomy", "runtime_semantics", "obstacle_assets"}
    if not required <= set(config) or not set(config) <= required | {"environment"}:
        raise LayoutError(f"visual asset config keys must be exactly {sorted(required)}")
    if config["schema_version"] != SCHEMA_VERSION or config["variant_count"] != 12:
        raise LayoutError("visual asset config schema or variant count is unsupported")
    for key in ("provider", "author", "license", "source"):
        if not isinstance(config[key], str) or not config[key]:
            raise LayoutError(f"visual asset config {key} must be a non-empty string")
    expected_taxonomy = {"floor", "floor_marking", "rack", "workbench", "process_equipment", "control_cabinet", "buffer_fixture", "pallet", "tote", "payload_tote", "amr", "station_marker", "barrier"}
    taxonomy = config["required_semantic_taxonomy"]
    if not isinstance(taxonomy, list) or set(taxonomy) != expected_taxonomy or len(taxonomy) != len(expected_taxonomy):
        raise LayoutError("visual semantic taxonomy is incomplete or duplicated")
    runtime = config["runtime_semantics"]
    if not isinstance(runtime, dict) or set(runtime) != {"amr", "payload_tote"}:
        raise LayoutError("runtime semantics must define amr and payload_tote")
    obstacle_ids = {str(item["id"]) for item in layout.data["physical_obstacles"]}
    assets = config["obstacle_assets"]
    if not isinstance(assets, dict) or set(assets) != obstacle_ids:
        raise LayoutError("visual asset assignments must match all 19 source obstacles exactly")
    valid_recipes = {
        "kitting_bench", "flow_rack", "assembly_bench", "inspection_station",
        "packing_bench", "process_cell", "control_cabinet", "buffer_fixture",
        "finished_goods", "service_cabinet", "semiconductor_handler", "ate_rack",
        "tray_stocker", "esd_tray_loadport", "aoi_mark", "esd_packing",
        "cleanroom_utility", "external_asset",
    }
    for obstacle_id, value in assets.items():
        if not isinstance(value, dict) or set(value) != {"recipe", "semantic_class", "state_role"}:
            raise LayoutError(f"visual asset assignment {obstacle_id} is invalid")
        if value["recipe"] not in valid_recipes or value["semantic_class"] not in expected_taxonomy:
            raise LayoutError(f"visual asset assignment {obstacle_id} has an unsupported recipe or class")
        if not isinstance(value["state_role"], str) or not value["state_role"]:
            raise LayoutError(f"visual asset assignment {obstacle_id} state_role is invalid")


def _external_asset_mapping(
    config: Mapping[str, Any], context: Mapping[str, object] | None,
) -> Mapping[str, Mapping[str, object]]:
    assignments = config["obstacle_assets"]
    assert isinstance(assignments, Mapping)
    expected = {
        str(obstacle_id): value for obstacle_id, value in assignments.items()
        if isinstance(value, Mapping) and value.get("recipe") == "external_asset"
    }
    mounted: Mapping[str, Mapping[str, object]] = {}
    if context is not None:
        raw_mounted = context.get("mounted_by_obstacle")
        if not isinstance(raw_mounted, Mapping) or not all(
            isinstance(key, str) and isinstance(value, Mapping)
            for key, value in raw_mounted.items()
        ):
            raise LayoutError("factory context mounted asset mapping is invalid")
        mounted = raw_mounted  # type: ignore[assignment]
    if set(mounted) != set(expected):
        missing = sorted(set(expected) - set(mounted))
        unexpected = sorted(set(mounted) - set(expected))
        raise LayoutError(
            f"external asset fixtures must match external_asset assignments exactly; "
            f"missing={missing}, unexpected={unexpected}"
        )
    for obstacle_id, instance in mounted.items():
        assignment = expected[obstacle_id]
        if (instance.get("semantic_class") != assignment.get("semantic_class")
                or instance.get("state_role") != assignment.get("state_role")):
            raise LayoutError(
                f"external asset fixture for {obstacle_id} semantic_class/state_role mismatch"
            )
    return mounted


def _validate_render_profile(profile: Mapping[str, Any]) -> None:
    required = {"schema_version", "renderer", "resolution", "render_settings", "roof_mode", "cameras", "capture_contract"}
    if set(profile) != required or profile["schema_version"] != 1:
        raise LayoutError("render profile keys or schema are invalid")
    if profile["roof_mode"] != "cutaway":
        raise LayoutError("render profile must preserve the roof cutaway")
    if profile["render_settings"] != {"sdk_renderer": "RealTimePathTracing", "dlss_exec_mode": 2, "rt_subframes": 4}:
        raise LayoutError("render profile must bind the applied SDK renderer, DLSS quality and frozen subframes")
    cameras = profile["cameras"]
    if not isinstance(cameras, list) or len(cameras) < 4:
        raise LayoutError("render profile must contain the portfolio camera set")
    identifiers: set[str] = set()
    for camera in cameras:
        if not isinstance(camera, dict) or set(camera) != {"id", "eye_xyz_m", "target_xyz_m", "focal_length_mm", "purpose"}:
            raise LayoutError("render profile camera is invalid")
        identifier = camera["id"]
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise LayoutError("render profile camera IDs must be unique strings")
        identifiers.add(identifier)
        for field in ("eye_xyz_m", "target_xyz_m"):
            values = camera[field]
            if not isinstance(values, list) or len(values) != 3 or not all(_is_finite_number(value) for value in values):
                raise LayoutError(f"render profile camera {identifier} {field} is invalid")


def _palette(seed: int) -> dict[str, tuple[float, float, float]]:
    accents = [(0.03, 0.40, 0.72), (0.00, 0.55, 0.42), (0.92, 0.43, 0.05), (0.52, 0.24, 0.72)]
    return {
        "accent": accents[seed % len(accents)],
        "accent2": accents[(seed + 2) % len(accents)],
        "steel": (0.24 + 0.01 * (seed % 3), 0.28, 0.32),
        "light": (1.0, 0.94 + 0.005 * seed, 0.82 + 0.01 * (seed % 4)),
    }


def _visual_header(seed: int, palette: Mapping[str, tuple[float, float, float]]) -> list[str]:
    return [
        "#usda 1.0",
        "(",
        '    defaultPrim = "World"',
        '    upAxis = "Z"',
        "    metersPerUnit = 1",
        ")",
        "",
        'def Xform "World"',
        "{",
        f"    custom int visualVariantSeed = {seed}",
        '    custom string visualContract = "render-only; no physics or navigation geometry"',
        '    def Scope "Visuals"',
        "    {",
        f"        custom color3f variantAccent = {_vec(palette['accent'])}",
    ]


def _materials(palette: Mapping[str, tuple[float, float, float]], *, semiconductor: bool = False) -> list[str]:
    definitions = {
        "Floor": ((0.32, 0.35, 0.37), 0.0, 0.86, 1.0),
        "Steel": (palette["steel"], 0.72, 0.32, 1.0),
        "LightSteel": ((0.42, 0.46, 0.50), 0.65, 0.28, 1.0),
        "MachinePaint": ((0.12, 0.17, 0.21), 0.18, 0.55, 1.0),
        "MachinePanel": ((0.55, 0.59, 0.61), 0.12, 0.58, 1.0),
        "Dark": ((0.035, 0.045, 0.06), 0.18, 0.46, 1.0),
        "Accent": (palette["accent"], 0.25, 0.32, 1.0),
        "Accent2": (palette["accent2"], 0.18, 0.34, 1.0),
        "ZoneBlue": ((0.17, 0.31, 0.39), 0.0, 0.66, 1.0),
        "ZoneAmber": ((0.43, 0.34, 0.19), 0.0, 0.66, 1.0),
        "ZoneGreen": ((0.19, 0.35, 0.29), 0.0, 0.66, 1.0),
        "SafetyYellow": ((0.88, 0.60, 0.03), 0.05, 0.46, 1.0),
        "SafetyRed": ((0.66, 0.025, 0.02), 0.05, 0.38, 1.0),
        "Green": ((0.03, 0.52, 0.17), 0.0, 0.38, 1.0),
        "White": ((0.68, 0.72, 0.74), 0.02, 0.72, 1.0),
        "ToteBlue": ((0.035, 0.22, 0.43), 0.05, 0.46, 1.0),
        "ToteOrange": ((0.68, 0.25, 0.035), 0.05, 0.46, 1.0),
        "Glass": ((0.08, 0.21, 0.28), 0.12, 0.16, 0.28),
        "ESD": ((0.08, 0.14, 0.18), 0.0, 0.78, 1.0),
        "Wood": ((0.39, 0.22, 0.09), 0.0, 0.74, 1.0),
    }
    if semiconductor:
        definitions.update({
            "FloorEpoxy": ((0.55, 0.74, 0.68), 0.0, 0.58, 1.0),
            "White": ((0.82, 0.87, 0.89), 0.02, 0.62, 1.0),
            "MachinePaint": ((0.24, 0.33, 0.38), 0.12, 0.55, 1.0),
        })
    lines = ['        def Scope "Looks"', "        {"]
    for name, (color, metallic, roughness, opacity) in definitions.items():
        lines.extend([
            f'            def Material "{name}"',
            "            {",
            f'                token outputs:surface.connect = </World/Visuals/Looks/{name}/Shader.outputs:surface>',
            f'                def Shader "Shader"',
            "                {",
            '                    uniform token info:id = "UsdPreviewSurface"',
            f"                    color3f inputs:diffuseColor = {_vec(color)}",
            f"                    float inputs:metallic = {_num(metallic)}",
            f"                    float inputs:roughness = {_num(roughness)}",
            f"                    float inputs:opacity = {_num(opacity)}",
            "                    float inputs:opacityThreshold = 0.01",
            "                    token outputs:surface",
            "                }",
            "            }",
        ])
    return lines + ["        }"]


def _factory_shell(layout: Layout, palette: Mapping[str, tuple[float, float, float]]) -> list[str]:
    light = palette["light"]
    bounds = layout.bounds
    south_wall_height = .25 if layout.data.get("revision") in PRODUCTION_BAY_REVISIONS else 1.2
    lines = ['        def Xform "FactoryShell"', "        {"]
    for name, bounds in (
        ("WestWall", (bounds.xmin - .12, bounds.ymin, 0.0, bounds.xmin, bounds.ymax, 3.2)),
        ("EastWall", (bounds.xmax, bounds.ymin, 0.0, bounds.xmax + .12, bounds.ymax, 3.2)),
        ("SouthWall", (bounds.xmin, bounds.ymin - .12, 0.0, bounds.xmax, bounds.ymin, south_wall_height)),
        ("NorthWall", (bounds.xmin, bounds.ymax, 0.0, bounds.xmax, bounds.ymax + .12, 3.2)),
    ):
        lines.extend(_box_usda(_Box(name, bounds, "White"), 12))
    lines.extend([
        f"            custom color3f authoredLightColor = {_vec(light)}",
        '            custom string lightingGeometry = "omitted; runtime UsdLux only"',
        '            custom string roofMode = "cutaway"',
        "        }",
    ])
    return lines


def _floor_graphics(layout: Layout, palette: Mapping[str, tuple[float, float, float]], *,
                    semantic_roots: dict[str, str] | None = None) -> list[str]:
    del palette
    semantic_roots = {} if semantic_roots is None else semantic_roots
    root_path = "/World/Visuals/FloorGraphics"
    semantic_roots[root_path] = "floor"
    lines = [
        '        def Xform "FloorGraphics"',
        "        {",
        '            custom string stableId = "floor_graphics"',
        '            custom string semanticClass = "floor"',
        '            custom string stateRole = "floor_and_markings"',
    ]
    def add_box(box: _Box) -> None:
        lines.extend(_box_usda(box, 12))
        if box.semantic_class is not None:
            semantic_roots[f"{root_path}/{_prim_name(box.name)}"] = box.semantic_class

    bounds = layout.bounds
    production_domain = layout.data.get("production_domain")
    semiconductor_domain = isinstance(production_domain, dict) and production_domain.get("id") == "semiconductor_final_test_pack"
    add_box(_Box(
        "FloorFinish",
        (bounds.xmin, bounds.ymin, 0.001, bounds.xmax, bounds.ymax, 0.006),
        "FloorEpoxy" if semiconductor_domain else "Floor",
        "floor",
        "esd_epoxy_floor" if semiconductor_domain else "factory_floor",
    ))
    configured_zones = layout.data.get("visual_zones")
    if configured_zones is None:
        zones = (
            ("WestZone", (0.2, 0.2, 0.007, 11.0, 23.8, 0.014), "ZoneBlue"),
            ("CellZone", (12.0, 5.2, 0.007, 30.5, 18.8, 0.014), "ZoneAmber"),
            ("ShippingZone", (32.5, 5.2, 0.007, 39.8, 18.8, 0.014), "ZoneGreen"),
        )
    else:
        if not isinstance(configured_zones, list) or len(configured_zones) != 3:
            raise LayoutError("visual_zones must contain three authored zone rectangles")
        materials = ("ZoneBlue", "ZoneAmber", "ZoneGreen")
        authored_zones = []
        for index, zone in enumerate(configured_zones):
            if not isinstance(zone, dict) or set(zone) != {"id", "rect"}:
                raise LayoutError(f"visual_zones[{index}] must contain id and rect")
            rect = Rect.from_value(zone["rect"], f"visual_zones[{index}].rect")
            if (rect.xmin < bounds.xmin or rect.ymin < bounds.ymin
                    or rect.xmax > bounds.xmax or rect.ymax > bounds.ymax):
                raise LayoutError(f"visual_zones[{index}] lies outside layout bounds")
            authored_zones.append((
                str(zone["id"]),
                (rect.xmin, rect.ymin, .007, rect.xmax, rect.ymax, .014),
                materials[index],
            ))
        zones = tuple(authored_zones)
    for name, zone_bounds, material in zones:
        x0, y0, z0, x1, y1, z1 = zone_bounds
        thickness = 0.10
        for suffix, border in (
            ("South", (x0, y0, z0, x1, y0 + thickness, z1)),
            ("North", (x0, y1 - thickness, z0, x1, y1, z1)),
            ("West", (x0, y0 + thickness, z0, x0 + thickness, y1 - thickness, z1)),
            ("East", (x1 - thickness, y0 + thickness, z0, x1, y1 - thickness, z1)),
        ):
            add_box(_Box(f"{name}_{suffix}", border, material, "floor_marking", "zone_boundary"))
    revision = layout.data.get("revision")
    if revision == DUAL_LANE_REVISION:
        lane_specs = _dual_lane_specs(layout)
        boundaries: set[tuple[str, float, float, float]] = set()
        for lane_id, (x0, y0), (x1, y1) in lane_specs:
            if math.isclose(y0, y1, abs_tol=1e-9):
                for y in (y0 - 1.5, y0 + 1.5):
                    boundaries.add(("horizontal", y, min(x0, x1), max(x0, x1)))
            else:
                for x in (x0 - 1.5, x0 + 1.5):
                    boundaries.add(("vertical", x, min(y0, y1), max(y0, y1)))
        for index, (axis, offset, start, end) in enumerate(sorted(boundaries)):
            if axis == "horizontal":
                bounds_xyz = (start, offset - .025, .011, end, offset + .025, .016)
            else:
                bounds_xyz = (offset - .025, start, .011, offset + .025, end, .016)
            add_box(_Box(
                f"LaneBoundary_{index}", bounds_xyz, "SafetyYellow",
                "floor_marking", "directed_lane_boundary",
            ))
        for lane_id, start, end in lane_specs:
            for arrow_index, fraction in enumerate((.25, .5, .75)):
                center = (
                    start[0] + (end[0] - start[0]) * fraction,
                    start[1] + (end[1] - start[1]) * fraction,
                )
                arrow_name = f"LaneArrow_{_prim_name(lane_id)}_{arrow_index}"
                lines.extend(_lane_arrow_usda(arrow_name, center, start, end, indent=12))
                semantic_roots[f"{root_path}/{arrow_name}"] = "floor_marking"
        line_specs: tuple[tuple[float, float, float], ...] = ()
    elif revision in PRODUCTION_BAY_REVISIONS:
        line_specs = ((2.5, bounds.xmin + .4, bounds.xmax - .4), (5.5, bounds.xmin + .4, bounds.xmax - .4),
                      (8.5, 2.0, 9.0), (11.5, 2.0, 9.0),
                      (14.5, bounds.xmin + .4, bounds.xmax - .4), (17.5, bounds.xmin + .4, bounds.xmax - .4))
        if revision in FLEET4_LAYOUT_REVISIONS:
            line_specs = tuple((y + 1.0, x0, x1) for y, x0, x1 in line_specs)
    else:
        line_specs = tuple((y, .4, 39.6) for y in (2.3, 5.7, 10.2, 13.8, 18.3, 21.7))
    for index, (y, x0, x1) in enumerate(line_specs):
        add_box(_Box(f"AisleLine_{index}", (x0, y - 0.025, 0.011, x1, y + 0.025, 0.016), "SafetyYellow", "floor_marking", "safety_line"))
    for marker_id, xy in sorted({**layout.stations, **layout.pockets}.items()):
        x, y = xy
        root = _prim_name(marker_id)
        semantic_roots[f"{root_path}/Station_{root}"] = "station_marker"
        lines.extend([
            f'            def Xform "Station_{root}"',
            "            {",
            '                custom string semanticClass = "station_marker"',
            f"                custom string stateRole = {_quote('parking_pocket' if marker_id.startswith('P') else 'mission_station')}",
            f"                custom string stableId = {_quote(marker_id)}",
        ])
        for chevron in range(3):
            offset = (chevron - 1) * 0.34
            lines.extend(_box_usda(_Box(f"Chevron_{chevron}", (x - 0.56 + offset, y - 0.035, 0.017, x - 0.34 + offset, y + 0.035, 0.022), "White"), 16))
        lines.append("            }")
    if revision in PRODUCTION_BAY_REVISIONS:
        stencil_specs = (
            ("S1", (3.75, 3.55), "station_identifier"),
            ("S2", (3.75, 9.55), "station_identifier"),
            ("S3", (3.75, 15.55), "station_identifier"),
            ("CI", (21.25, 6.55), "station_identifier"),
            ("CO", (21.25, 12.55), "station_identifier"),
            ("OUT", (28.75, 9.55), "station_identifier"),
            ("P1", (11.75, 1.05), "station_identifier"),
            ("P2", (11.75, 18.05), "station_identifier"),
            ("MATERIAL", (5.0, 6.85), "process_identifier"),
            ("ASSEMBLY", (16.7, 4.9), "process_identifier"),
            ("INSPECT", (14.0, 9.8), "process_identifier"),
            ("PACK", (16.7, 14.6), "process_identifier"),
            ("SHIP", (27.2, 8.8), "process_identifier"),
        )
        if revision in FLEET4_LAYOUT_REVISIONS:
            stencil_specs = tuple((name, (xy[0] - .25, xy[1] - .45), "station_identifier")
                                  for name, xy in {**layout.stations, **layout.pockets}.items()) + tuple(
                (name, (origin[0], origin[1] + 1.0), role)
                for name, origin, role in stencil_specs if role == "process_identifier")
        if semiconductor_domain:
            stencil_specs = tuple(spec for spec in stencil_specs if spec[2] == "station_identifier")
        for label, origin, role in stencil_specs:
            lines.extend(_floor_stencil_usda(label, origin, role, indent=12, material="Dark" if semiconductor_domain else "White"))
            semantic_roots[f"{root_path}/Stencil_{_prim_name(label)}"] = "floor_marking"
    if semiconductor_domain:
        assert isinstance(production_domain, dict)
        allowed_domain_keys = {
            "id", "display_name", "scope", "logical_process", "quantity_unit",
            "zone_labels", "cargo_recipes", "reference_urls", "dimension_basis",
        }
        if (not {"id", "zone_labels"} <= set(production_domain)
                or not set(production_domain) <= allowed_domain_keys):
            raise LayoutError("semiconductor production_domain has unsupported keys")
        zone_labels = production_domain["zone_labels"]
        if not isinstance(zone_labels, list) or not zone_labels:
            raise LayoutError("semiconductor production_domain zone_labels must be a non-empty list")
        seen_labels: set[str] = set()
        for index, item in enumerate(zone_labels):
            if not isinstance(item, dict) or set(item) != {"label", "origin_xy", "role"}:
                raise LayoutError(f"production_domain.zone_labels[{index}] is invalid")
            label, origin, role = item["label"], item["origin_xy"], item["role"]
            if not isinstance(label, str) or not label or label in seen_labels:
                raise LayoutError("semiconductor floor labels must be unique non-empty strings")
            if (not isinstance(origin, list) or len(origin) != 2
                    or not all(_is_finite_number(value) for value in origin)):
                raise LayoutError(f"production_domain.zone_labels[{index}].origin_xy is invalid")
            if not isinstance(role, str) or not role:
                raise LayoutError(f"production_domain.zone_labels[{index}].role is invalid")
            origin_xy = (float(origin[0]), float(origin[1]))
            _points, _counts, _indices, stencil_bounds = _floor_stencil_geometry(label, origin_xy)
            if (stencil_bounds[0] < bounds.xmin or stencil_bounds[1] < bounds.ymin
                    or stencil_bounds[2] > bounds.xmax or stencil_bounds[3] > bounds.ymax):
                raise LayoutError(f"production_domain.zone_labels[{index}] lies outside layout bounds")
            domain_prim = f"Zone_{_prim_name(label)}"
            lines.extend(_floor_stencil_usda(label, origin_xy, role, indent=12, prim_name=domain_prim, material="Dark"))
            semantic_roots[f"{root_path}/Stencil_{domain_prim}"] = "floor_marking"
            seen_labels.add(label)
    lines.extend(["        }"])
    return lines


def _dual_lane_specs(
    layout: Layout,
) -> tuple[tuple[str, tuple[float, float], tuple[float, float]], ...]:
    """Validate the authored one-way lane markings consumed by the visual layer."""
    lanes = layout.data.get("traffic_lanes")
    if not isinstance(lanes, list) or len(lanes) != 6:
        raise LayoutError("dual-lane layout traffic_lanes must contain six directed lanes")
    expected_ids = {"LOW_E", "LOW_W", "HIGH_E", "HIGH_W", "EAST_N", "EAST_S"}
    seen: set[str] = set()
    result: list[tuple[str, tuple[float, float], tuple[float, float]]] = []
    for index, lane in enumerate(lanes):
        if not isinstance(lane, dict) or set(lane) != {"id", "from_xy", "to_xy"}:
            raise LayoutError(f"traffic_lanes[{index}] must contain id, from_xy and to_xy")
        lane_id = lane["id"]
        if not isinstance(lane_id, str) or lane_id not in expected_ids or lane_id in seen:
            raise LayoutError(f"traffic_lanes[{index}].id is unsupported or duplicated")
        points: list[tuple[float, float]] = []
        for field in ("from_xy", "to_xy"):
            value = lane[field]
            if (not isinstance(value, list) or len(value) != 2
                    or not all(_is_finite_number(number) for number in value)):
                raise LayoutError(f"traffic_lanes[{index}].{field} must be a finite XY pair")
            point = (float(value[0]), float(value[1]))
            if not layout.bounds.contains_point(point):
                raise LayoutError(f"traffic_lanes[{index}].{field} lies outside layout bounds")
            points.append(point)
        start, end = points
        if (math.isclose(start[0], end[0], abs_tol=1e-9)
                == math.isclose(start[1], end[1], abs_tol=1e-9)):
            raise LayoutError(f"traffic_lanes[{index}] must be a nonzero axis-aligned segment")
        seen.add(lane_id)
        result.append((lane_id, start, end))
    if seen != expected_ids:
        raise LayoutError("dual-lane layout traffic_lanes IDs differ from the visual contract")
    return tuple(result)


def _lane_arrow_usda(
    name: str,
    center: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    indent: int,
) -> list[str]:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    cx, cy = center
    tip = (cx + ux * .55, cy + uy * .55, .018)
    head_left = (cx + ux * .08 + px * .23, cy + uy * .08 + py * .23, .018)
    head_right = (cx + ux * .08 - px * .23, cy + uy * .08 - py * .23, .018)
    tail_left = (cx - ux * .55 + px * .055, cy - uy * .55 + py * .055, .018)
    tail_right = (cx - ux * .55 - px * .055, cy - uy * .55 - py * .055, .018)
    neck_left = (cx + ux * .08 + px * .055, cy + uy * .08 + py * .055, .018)
    neck_right = (cx + ux * .08 - px * .055, cy + uy * .08 - py * .055, .018)
    pad = " " * indent
    points = (tip, head_left, head_right, tail_left, tail_right, neck_left, neck_right)
    return [
        f'{pad}def Mesh "{_prim_name(name)}"',
        f"{pad}{{",
        f'{pad}    custom string semanticClass = "floor_marking"',
        f'{pad}    custom string stateRole = "directed_lane_arrow"',
        f'{pad}    custom string stableId = {_quote(name)}',
        f"{pad}    point3f[] points = [{', '.join(_vec(point) for point in points)}]",
        f"{pad}    int[] faceVertexCounts = [3, 4]",
        f"{pad}    int[] faceVertexIndices = [0, 1, 2, 3, 4, 6, 5]",
        f'{pad}    uniform token subdivisionScheme = "none"',
        f'{pad}    rel material:binding = </World/Visuals/Looks/White>',
        f"{pad}}}",
    ]


def _floor_stencil_geometry(
    label: str, origin_xy: tuple[float, float], pixel_m: float = .05,
) -> tuple[list[tuple[float, float, float]], list[int], list[int], tuple[float, float, float, float]]:
    if not label or any(character != " " and character not in _STENCIL_GLYPHS for character in label):
        raise LayoutError(f"unsupported floor stencil label: {label!r}")
    x0, y0 = origin_xy
    points: list[tuple[float, float, float]] = []
    indices: list[int] = []
    advance = 6 * pixel_m
    for character_index, character in enumerate(label):
        if character == " ":
            continue
        for row, pixels in enumerate(_STENCIL_GLYPHS[character]):
            for column, enabled in enumerate(pixels):
                if enabled != "1":
                    continue
                left = x0 + character_index * advance + column * pixel_m
                bottom = y0 + (6 - row) * pixel_m
                start = len(points)
                points.extend([
                    (left, bottom, .021), (left + pixel_m, bottom, .021),
                    (left + pixel_m, bottom + pixel_m, .021), (left, bottom + pixel_m, .021),
                ])
                indices.extend((start, start + 1, start + 2, start + 3))
    width = (len(label) * 6 - 1) * pixel_m
    return points, [4] * (len(indices) // 4), indices, (x0, y0, x0 + width, y0 + 7 * pixel_m)


def _floor_stencil_usda(
    label: str, origin_xy: tuple[float, float], role: str, *, indent: int,
    prim_name: str | None = None, material: str = "White",
) -> list[str]:
    points, counts, indices, bounds = _floor_stencil_geometry(label, origin_xy)
    pad = " " * indent
    point_values = ", ".join(f"({_num(x)}, {_num(y)}, {_num(z)})" for x, y, z in points)
    return [
        f'{pad}def Mesh "Stencil_{prim_name or _prim_name(label)}"',
        f"{pad}{{",
        f"{pad}    custom string stableId = {_quote(f'floor_stencil:{label}')}",
        f'{pad}    custom string semanticClass = "floor_marking"',
        f"{pad}    custom string stateRole = {_quote(role)}",
        f"{pad}    custom double4 boundsXY = ({', '.join(_num(value) for value in bounds)})",
        f"{pad}    float3[] extent = [({_num(bounds[0])}, {_num(bounds[1])}, 0.021), ({_num(bounds[2])}, {_num(bounds[3])}, 0.021)]",
        f"{pad}    int[] faceVertexCounts = [{', '.join(str(value) for value in counts)}]",
        f"{pad}    int[] faceVertexIndices = [{', '.join(str(value) for value in indices)}]",
        f"{pad}    point3f[] points = [{point_values}]",
        f"{pad}    rel material:binding = </World/Visuals/Looks/{material}>",
        f'{pad}    uniform token subdivisionScheme = "none"',
        f"{pad}}}",
    ]


def _equipment_root(obstacle_id: str, semantic_class: str, state_role: str, recipe: str, boxes: Sequence[_Box], *, indent: int) -> list[str]:
    pad = " " * indent
    lines = [
        f'{pad}def Xform "{_prim_name(obstacle_id)}"',
        f"{pad}{{",
        f"{pad}    custom string stableId = {_quote(obstacle_id)}",
        f"{pad}    custom string semanticClass = {_quote(semantic_class)}",
        f"{pad}    custom string stateRole = {_quote(state_role)}",
        f"{pad}    custom string sourceObstacleId = {_quote(obstacle_id)}",
        f"{pad}    custom string assetRecipe = {_quote(recipe)}",
    ]
    for box in boxes:
        stable_id = f"{obstacle_id}:{box.name}"
        if box.semantic_class == "tote":
            lines.extend(_tote_usda(box, indent + 4, stable_id, obstacle_id))
        else:
            lines.extend(_box_usda(
                box,
                indent + 4,
                stable_id=stable_id,
                source_obstacle_id=obstacle_id,
            ))
    return lines + [f"{pad}}}"]


def _tote_usda(box: _Box, indent: int, stable_id: str, source_obstacle_id: str) -> list[str]:
    pad = " " * indent
    parts = _tote_parts(box)
    _validate_boxes(stable_id, parts, box.bounds)
    lines = [
        f'{pad}def Xform "{_prim_name(box.name)}"',
        f"{pad}{{",
        f"{pad}    custom string stableId = {_quote(stable_id)}",
        f"{pad}    custom string semanticClass = {_quote(box.semantic_class or 'tote')}",
        f"{pad}    custom string stateRole = {_quote(box.state_role or 'material_tote')}",
        f"{pad}    custom string sourceObstacleId = {_quote(source_obstacle_id)}",
        f'{pad}    custom string geometryStyle = "open-rim industrial tote"',
    ]
    for part in parts:
        lines.extend(_box_usda(
            part,
            indent + 4,
            stable_id=f"{stable_id}:{part.name}",
            source_obstacle_id=source_obstacle_id,
        ))
    return lines + [f"{pad}}}"]


def _tote_parts(box: _Box) -> list[_Box]:
    x0, y0, z0, x1, y1, z1 = box.bounds
    width, depth, height = x1 - x0, y1 - y0, z1 - z0
    wall_x = min(width * 0.10, 0.075)
    wall_y = min(depth * 0.12, 0.075)
    base_top = z0 + height * 0.18
    wall_top = z0 + height * 0.82
    rim_bottom = z0 + height * 0.78
    handle_x0, handle_x1 = x0 + width * 0.34, x0 + width * 0.66
    return [
        _Box("Base", (x0 + wall_x, y0 + wall_y, z0, x1 - wall_x, y1 - wall_y, base_top), box.material),
        _Box("InnerRecess", (x0 + wall_x * 1.4, y0 + wall_y * 1.4, base_top, x1 - wall_x * 1.4, y1 - wall_y * 1.4, base_top + height * 0.035), "Dark"),
        _Box("WallWest", (x0, y0 + wall_y, z0 + height * 0.08, x0 + wall_x, y1 - wall_y, wall_top), box.material),
        _Box("WallEast", (x1 - wall_x, y0 + wall_y, z0 + height * 0.08, x1, y1 - wall_y, wall_top), box.material),
        _Box("WallSouth", (x0 + wall_x, y0, z0 + height * 0.08, x1 - wall_x, y0 + wall_y, wall_top), box.material),
        _Box("WallNorth", (x0 + wall_x, y1 - wall_y, z0 + height * 0.08, x1 - wall_x, y1, wall_top), box.material),
        _Box("RimWest", (x0, y0, rim_bottom, x0 + wall_x * 1.35, y1, z1), box.material),
        _Box("RimEast", (x1 - wall_x * 1.35, y0, rim_bottom, x1, y1, z1), box.material),
        _Box("RimSouth", (x0 + wall_x, y0, rim_bottom, x1 - wall_x, y0 + wall_y * 1.35, z1), box.material),
        _Box("RimNorth", (x0 + wall_x, y1 - wall_y * 1.35, rim_bottom, x1 - wall_x, y1, z1), box.material),
        _Box("HandleSouth", (handle_x0, y0, z0 + height * 0.57, handle_x1, y0 + wall_y * 1.5, z0 + height * 0.69), "Dark"),
        _Box("HandleNorth", (handle_x0, y1 - wall_y * 1.5, z0 + height * 0.57, handle_x1, y1, z0 + height * 0.69), "Dark"),
    ]


def _recipe_boxes(
    recipe: str,
    rect: Rect,
    height: float,
    seed: int,
    obstacle_id: str,
    *,
    geometry_profile: str = "legacy",
) -> list[_Box]:
    x0, y0, x1, y1 = rect.xmin, rect.ymin, rect.xmax, rect.ymax
    w, d = x1 - x0, y1 - y0
    if geometry_profile not in {"legacy", PRODUCTION_BAY_GEOMETRY}:
        raise LayoutError(f"unsupported visual geometry profile: {geometry_profile}")
    if recipe == "cleanroom_utility" and w > 2.6:
        count = math.ceil(w / 2.2)
        pitch = w / count
        modules = []
        for index in range(count):
            module = Rect(x0 + index * pitch + .03, y0, x0 + (index + 1) * pitch - .03, y1)
            for box in _recipe_boxes(recipe, module, height, seed, obstacle_id, geometry_profile=geometry_profile):
                modules.append(_Box(
                    f"Module_{index}_{box.name}", box.bounds, box.material,
                    box.semantic_class, box.state_role, box.render_style, box.render_chamfer_m,
                ))
        return modules
    def b(name: str, fx0: float, fy0: float, z0: float, fx1: float, fy1: float, z1: float, material: str, semantic: str | None = None, role: str | None = None, *, chamfer: bool = False) -> _Box:
        return _Box(
            name, (x0 + w * fx0, y0 + d * fy0, z0, x0 + w * fx1, y0 + d * fy1, min(z1, height)),
            material, semantic, role, "chamfered_box" if chamfer else "cube", .02 if chamfer else 0.0,
        )

    def absolute(name: str, ax0: float, ay0: float, z0: float, ax1: float, ay1: float, z1: float, material: str, semantic: str | None = None, role: str | None = None) -> _Box:
        return _Box(name, (ax0, ay0, z0, ax1, ay1, min(z1, height)), material, semantic, role)

    if recipe == "semiconductor_handler":
        boxes = [
            b("Plinth", .03, .04, 0, .97, .96, height * .08, "MachinePaint"),
            b("CabinetLeft", .03, .12, height * .08, .18, .96, height * .92, "White", "process_equipment", "handler_cabinet", chamfer=True),
            b("CabinetRight", .82, .12, height * .08, .97, .96, height * .92, "White", "process_equipment", "handler_cabinet", chamfer=True),
            b("CabinetRear", .18, .86, height * .08, .82, .96, height * .92, "White", chamfer=True),
            b("Roof", .03, .12, height * .88, .97, .96, height * .96, "White", chamfer=True),
            b("InspectionBed", .20, .25, height * .31, .80, .82, height * .38, "Steel", "process_equipment", "test_chamber"),
            b("WorkBed", .28, .31, height * .39, .72, .69, height * .44, "ESD", "workbench", "handler_tray_support"),
            b("SocketNest", .38, .45, height * .38, .62, .67, height * .43, "Accent", "process_equipment", "device_interface_board"),
            b("DoorFrameLeft", .17, .08, height * .25, .23, .14, height * .84, "MachinePanel"),
            b("DoorFrameRight", .77, .08, height * .25, .83, .14, height * .84, "MachinePanel"),
            b("DoorFrameTop", .17, .08, height * .80, .83, .14, height * .86, "MachinePanel"),
            b("ChamberWindow", .23, .09, height * .42, .77, .125, height * .79, "Glass", "process_equipment", "handler_chamber_window"),
            b("DoorHandle", .72, .055, height * .53, .77, .11, height * .69, "Dark"),
            b("LoadTrayElevator", .05, .01, height * .12, .30, .20, height * .34, "LightSteel", "process_equipment", "load_tray_elevator"),
            b("UnloadTrayElevator", .70, .01, height * .12, .95, .20, height * .34, "LightSteel", "process_equipment", "unload_tray_elevator"),
            b("LoadTrayMagazine", .08, .025, height * .20, .27, .17, height * .29, "ESD", "rack", "jedec_tray_magazine"),
            b("UnloadTrayMagazine", .73, .025, height * .20, .92, .17, height * .29, "ESD", "rack", "jedec_tray_magazine"),
            b("PassBin", .55, .02, height * .10, .68, .16, height * .24, "Green", "buffer_fixture", "pass_sort_bin"),
            b("RetestBin", .32, .02, height * .10, .45, .16, height * .24, "SafetyYellow", "buffer_fixture", "retest_sort_bin"),
            b("HMIArm", .80, .04, height * .56, .87, .13, height * .75, "Steel"),
            b("HMI", .73, .015, height * .66, .94, .075, height * .83, "Dark", "process_equipment", "handler_hmi"),
            b("LoadDockFace", .045, .005, height * .11, .305, .055, height * .35, "MachinePanel", "process_equipment", "front_load_tray_dock"),
            b("UnloadDockFace", .695, .005, height * .11, .955, .055, height * .35, "MachinePanel", "process_equipment", "front_unload_tray_dock"),
            b("LoadMagazineGuideLeft", .065, .002, height * .18, .09, .19, height * .32, "LightSteel"),
            b("LoadMagazineGuideRight", .26, .002, height * .18, .285, .19, height * .32, "LightSteel"),
            b("UnloadMagazineGuideLeft", .715, .002, height * .18, .74, .19, height * .32, "LightSteel"),
            b("UnloadMagazineGuideRight", .91, .002, height * .18, .935, .19, height * .32, "LightSteel"),
            b("LowerAccessPanelLeft", .045, .075, height * .40, .175, .125, height * .73, "MachinePanel"),
            b("LowerAccessPanelRight", .825, .075, height * .40, .955, .125, height * .73, "MachinePanel"),
            b("AccessHandleLeft", .135, .045, height * .51, .16, .10, height * .64, "Dark"),
            b("AccessHandleRight", .84, .045, height * .51, .865, .10, height * .64, "Dark"),
            b("TesterDockPlate", .38, .925, height * .47, .62, .975, height * .69, "MachinePanel", "process_equipment", "tester_interface_dock"),
            b("TesterDockSignal", .43, .918, height * .54, .48, .97, height * .61, "Accent"),
            b("TesterDockUtility", .52, .918, height * .54, .57, .97, height * .61, "SafetyYellow"),
            b("UtilityTrunk", .57, .90, height * .57, .84, .945, height * .62, "Dark", "process_equipment", "handler_tester_utility_link"),
            b("StackLightPost", .91, .78, height * .92, .94, .83, height * .98, "Steel"),
            b("StackLightGreen", .90, .77, height * .965, .95, .84, height, "Green"),
            b("StackLightAmber", .90, .77, height * .935, .95, .84, height * .965, "SafetyYellow"),
        ]
        for side, fx0 in (("Load", .09), ("Unload", .74)):
            for layer in range(4):
                boxes.append(b(
                    f"{side}MagazineTray_{layer}", fx0, .012, height * (.205 + .026 * layer),
                    fx0 + .17, .18, height * (.216 + .026 * layer), "ESD",
                ))
        for index in range(5):
            boxes.append(b(f"FrontVent_{index}", .20 + .115 * index, .075, height * .17, .27 + .115 * index, .115, height * .20, "Dark"))
        for index in range(4):
            boxes.append(b(f"SideVentLeft_{index}", .025, .34 + .12 * index, height * .76, .07, .42 + .12 * index, height * .79, "Dark"))
            boxes.append(b(f"SideVentRight_{index}", .93, .34 + .12 * index, height * .76, .975, .42 + .12 * index, height * .79, "Dark"))
        return boxes

    if recipe == "ate_rack":
        boxes = [
            b("Plinth", .04, .06, 0, .96, .94, height * .08, "MachinePaint"),
            b("RackLeft", .04, .10, height * .08, .10, .94, height, "White", chamfer=True),
            b("RackRight", .90, .10, height * .08, .96, .94, height, "White", chamfer=True),
            b("RackRear", .10, .88, height * .08, .90, .94, height, "MachinePanel", chamfer=True),
            b("TopCap", .04, .10, height * .94, .96, .94, height, "White", chamfer=True),
            b("ATE_Mainframe", .12, .18, height * .11, .88, .84, height * .52, "MachinePaint", "process_equipment", "ate_mainframe", chamfer=True),
            b("ATE_Controller", .12, .18, height * .56, .88, .84, height * .77, "MachinePanel", "process_equipment", "ate_controller", chamfer=True),
            b("UtilityPatchPanel", .12, .10, height * .80, .88, .22, height * .91, "Dark", "control_cabinet", "socket_utility"),
            b("HMI", .26, .055, height * .61, .74, .12, height * .73, "Glass", "process_equipment", "ate_hmi"),
            b("ControllerDoorLeft", .12, .105, height * .55, .49, .19, height * .78, "MachinePanel"),
            b("ControllerDoorRight", .51, .105, height * .55, .88, .19, height * .78, "MachinePanel"),
            b("ControllerDoorSeam", .49, .08, height * .56, .51, .20, height * .77, "Dark"),
            b("ControllerHandleLeft", .43, .055, height * .62, .46, .12, height * .70, "Dark"),
            b("ControllerHandleRight", .54, .055, height * .62, .57, .12, height * .70, "Dark"),
            b("TesterInterfaceDock", .76, .04, height * .79, .94, .105, height * .93, "MachinePanel", "process_equipment", "tester_interface_dock"),
            b("InterfaceSignalPort", .79, .025, height * .83, .84, .08, height * .88, "Accent"),
            b("InterfaceUtilityPort", .86, .025, height * .83, .91, .08, height * .88, "SafetyYellow"),
            b("UtilityUmbilicalDrop", .91, .035, height * .38, .95, .085, height * .83, "Dark", "process_equipment", "tester_utility_umbilical"),
            b("UtilityUmbilicalBridge", .84, .035, height * .36, .95, .085, height * .41, "Dark"),
        ]
        for index in range(6):
            boxes.append(b(f"MainframeSlot_{index}", .16, .105, height * (.16 + .055 * index), .84, .18, height * (.19 + .055 * index), "LightSteel"))
        for index in range(5):
            boxes.append(b(f"SocketUtility_{index}", .18 + .135 * index, .06, height * .83, .25 + .135 * index, .115, height * .88, "Accent" if index % 2 == 0 else "SafetyYellow"))
        for index in range(5):
            boxes.append(b(f"LowerVent_{index}", .18 + .13 * index, .075, height * .115, .27 + .13 * index, .125, height * .14, "Dark"))
        for index in range(4):
            boxes.append(b(f"SideVent_{index}", .035, .26 + .14 * index, height * .20, .085, .35 + .14 * index, height * .24, "Dark"))
        return boxes

    if recipe == "tray_stocker":
        boxes = [
            b("PostFL", .03, .04, 0, .075, .12, height, "LightSteel"),
            b("PostFR", .925, .04, 0, .97, .12, height, "LightSteel"),
            b("PostBL", .03, .88, 0, .075, .96, height, "LightSteel"),
            b("PostBR", .925, .88, 0, .97, .96, height, "LightSteel"),
            b("RearPanel", .075, .91, height * .06, .925, .96, height * .95, "White"),
            b("HMI", .72, .015, height * .72, .92, .07, height * .86, "Dark", "process_equipment", "stocker_hmi"),
        ]
        for level in range(4):
            z = height * (.10 + .205 * level)
            boxes.append(b(f"ShelfDeck_{level}", .06, .10, z, .94, .90, z + height * .035, "ESD", "rack", "esd_shelf"))
            for stack in range(3):
                fx0 = .10 + .285 * stack
                boxes.append(b(f"TrayStack_L{level}_{stack}", fx0, .18, z + height * .04, fx0 + .22, .80, z + height * .13, "Accent", "tote", "jedec_tray_stack"))
        top_z = height * .775
        for row in range(3):
            for column in range(5):
                boxes.append(b(
                    f"LotPocket_{row}_{column}", .125 + column * .15, .23 + row * .18,
                    top_z, .19 + column * .15, .34 + row * .18, top_z + height * .025,
                    "Dark", "tote", "lot_pocket",
                ))
        return boxes

    if recipe == "esd_tray_loadport":
        boxes = [
            b("Base", .04, .06, 0, .96, .94, height * .12, "MachinePaint"),
            b("Cabinet", .07, .36, height * .12, .93, .94, height * .72, "White", "process_equipment", "loadport_cabinet"),
            b("Worktop", .03, .06, height * .70, .97, .94, height * .78, "ESD", "workbench", "tray_transfer_deck"),
            b("DeckGuideLeft", .06, .55, height * .78, .12, .90, height * .84, "LightSteel"),
            b("DeckGuideRight", .88, .55, height * .78, .94, .90, height * .84, "LightSteel"),
            b("TrayMagazine", .17, .60, height * .79, .83, .88, height * .92, "Accent", "tote", "jedec_tray_magazine"),
            b("MagazineStop", .17, .84, height * .84, .83, .91, height * .96, "SafetyYellow"),
            b("HMI", .70, .02, height * .42, .91, .08, height * .59, "Dark", "process_equipment", "loadport_hmi"),
            b("StatusGreen", .10, .02, height * .50, .18, .08, height * .58, "Green"),
        ]
        for index in range(5):
            boxes.append(b(f"TrayLayer_{index}", .20, .62, height * (.80 + .018 * index), .80, .85, height * (.812 + .018 * index), "ESD"))
        return boxes

    if recipe == "aoi_mark":
        boxes = [
            b("Plinth", .04, .05, 0, .96, .95, height * .10, "MachinePaint"),
            b("ChamberLeft", .04, .12, height * .10, .17, .95, height * .90, "White", "process_equipment", "aoi_chamber", chamfer=True),
            b("ChamberRight", .83, .12, height * .10, .96, .95, height * .90, "White", "process_equipment", "aoi_chamber", chamfer=True),
            b("ChamberRear", .17, .88, height * .10, .83, .95, height * .90, "White", chamfer=True),
            b("ChamberRoof", .04, .12, height * .86, .96, .95, height * .94, "White", chamfer=True),
            b("InspectionStage", .20, .25, height * .27, .80, .78, height * .34, "ESD", "process_equipment", "tray_inspection_stage"),
            b("OpticalGantry", .28, .54, height * .34, .72, .67, height * .68, "MachinePanel"),
            b("OpticalHead", .39, .40, height * .57, .61, .62, height * .75, "Dark", "process_equipment", "aoi_optical_head"),
            b("Lens", .45, .36, height * .51, .55, .43, height * .60, "Glass"),
            b("ShadeWindow", .17, .08, height * .38, .83, .13, height * .82, "Glass", "process_equipment", "light_shield_window"),
            b("PassBuffer", .58, .02, height * .12, .78, .18, height * .27, "Green", "buffer_fixture", "pass_buffer"),
            b("ReviewBuffer", .80, .02, height * .12, .96, .18, height * .27, "SafetyYellow", "buffer_fixture", "review_buffer"),
            b("HMI", .08, .015, height * .50, .28, .075, height * .68, "Dark", "process_equipment", "aoi_hmi"),
            b("InputMagazineDock", .18, .02, height * .12, .42, .18, height * .29, "MachinePanel", "process_equipment", "inspection_tray_input"),
            b("InputGuideLeft", .18, .012, height * .12, .205, .19, height * .31, "LightSteel"),
            b("InputGuideRight", .395, .012, height * .12, .42, .19, height * .31, "LightSteel"),
            b("LotReader", .44, .025, height * .14, .55, .12, height * .28, "Dark", "process_equipment", "lot_id_reader"),
            b("ServicePanelLeft", .045, .075, height * .18, .165, .135, height * .42, "MachinePanel"),
            b("ServicePanelRight", .835, .075, height * .34, .955, .135, height * .62, "MachinePanel"),
            b("ServiceHandleLeft", .13, .045, height * .25, .155, .105, height * .35, "Dark"),
            b("ServiceHandleRight", .85, .045, height * .43, .875, .105, height * .55, "Dark"),
            b("MarkingUtilityPlate", .84, .90, height * .48, .96, .97, height * .66, "MachinePanel", "process_equipment", "marking_utility_interface"),
            b("MarkingUtilityLink", .72, .91, height * .54, .86, .955, height * .59, "Dark"),
        ]
        for layer in range(4):
            boxes.append(b(
                f"InputMagazineTray_{layer}", .205, .025, height * (.16 + .026 * layer),
                .395, .175, height * (.172 + .026 * layer), "ESD",
            ))
        for index in range(4):
            boxes.append(b(f"AOIVentLeft_{index}", .025, .35 + .12 * index, height * .72, .07, .43 + .12 * index, height * .75, "Dark"))
            boxes.append(b(f"AOIVentRight_{index}", .93, .35 + .12 * index, height * .72, .975, .43 + .12 * index, height * .75, "Dark"))
        return boxes

    if recipe == "esd_packing":
        top = height * .66
        boxes = [
            b("Worktop", .03, .05, top, .97, .95, top + height * .08, "MachinePanel"),
            b("ESDMat", .18, .14, top + height * .085, .70, .78, top + height * .105, "ESD", "workbench", "esd_pack_surface"),
            b("LegLeft", .06, .10, 0, .13, .90, top, "LightSteel"),
            b("LegRight", .87, .10, 0, .94, .90, top, "LightSteel"),
            b("UnderShelf", .12, .14, height * .16, .88, .86, height * .23, "MachinePaint"),
            b("LabelPrinter", .70, .42, top + height * .09, .89, .70, top + height * .25, "White", "process_equipment", "label_printer"),
            b("Scale", .30, .40, top + height * .09, .56, .68, top + height * .13, "Dark", "process_equipment", "packing_scale"),
            b("CoveredCarrier", .13, .18, top + height * .11, .29, .44, top + height * .25, "Glass", "tote", "covered_esd_carrier"),
            b("LabelRoll", .77, .23, top + height * .10, .86, .36, top + height * .21, "White"),
            b("HMI", .12, .80, top + height * .10, .34, .90, top + height * .28, "Dark", "process_equipment", "packing_hmi"),
        ]
        for index in range(8):
            fy = .12 + index * .085
            boxes.append(b(
                f"Roller_{index}", .18, fy, top + height * .085,
                .70, fy + .025, top + height * .12, "LightSteel",
            ))
        return boxes

    if recipe == "cleanroom_utility":
        boxes = [
            b("Plinth", .03, .05, 0, .97, .95, height * .08, "MachinePaint"),
            b("UtilityCabinet", .04, .12, height * .08, .96, .95, height, "White", "control_cabinet", "cleanroom_utility"),
            b("DoorLeft", .06, .07, height * .14, .49, .14, height * .88, "MachinePanel"),
            b("DoorRight", .51, .07, height * .14, .94, .14, height * .88, "MachinePanel"),
            b("DoorSeam", .485, .065, height * .14, .515, .14, height * .88, "Dark"),
            b("HandleLeft", .42, .04, height * .43, .46, .11, height * .64, "Dark"),
            b("HandleRight", .54, .04, height * .43, .58, .11, height * .64, "Dark"),
            b("FilterHeader", .08, .16, height * .82, .92, .88, height * .94, "LightSteel", "process_equipment", "filter_header"),
            b("DistributionPanel", .13, .035, height * .56, .39, .10, height * .75, "Dark", "control_cabinet", "power_distribution"),
            b("UtilityPanel", .61, .035, height * .56, .87, .10, height * .75, "Accent", "control_cabinet", "facility_utility"),
        ]
        for index in range(5):
            boxes.append(b(f"Vent_{index}", .12 + .155 * index, .055, height * .22, .22 + .155 * index, .115, height * .27, "Dark"))
        return boxes

    if recipe == "flow_rack":
        boxes = [
            b("Post_FL", .02, .05, 0, .055, .14, height, "Steel"), b("Post_FR", .945, .05, 0, .98, .14, height, "Steel"),
            b("Post_BL", .02, .86, 0, .055, .95, height, "Steel"), b("Post_BR", .945, .86, 0, .98, .95, height, "Steel"),
            b("RearBraceLow", .03, .89, .38, .97, .94, .44, "Steel"),
            b("RearBraceHigh", .03, .89, 1.42, .97, .94, 1.48, "Steel"),
        ]
        for level, z in enumerate((.14, .68, 1.22, 1.82)):
            boxes.extend([
                b(f"FrontBeam_{level}", .02, .04, z, .98, .12, z + .075, "MachinePaint"),
                b(f"RearBeam_{level}", .02, .88, z, .98, .96, z + .075, "MachinePaint"),
                b(f"ShelfDeck_{level}", .04, .12, z + .01, .96, .88, z + .04, "Steel"),
            ])
        order = list(range(12))
        shift = seed % len(order)
        order = order[shift:] + order[:shift]
        for index, value in enumerate(order):
            column, level = index % 4, index // 4
            material = "ToteBlue" if value % 2 == 0 else "ToteOrange"
            z0 = .23 + .54 * level
            boxes.append(b(f"Tote_{index:02d}", .075 + .225 * column, .16, z0, .255 + .225 * column, .82, z0 + .36, material, "tote", "inventory_tote"))
        return boxes
    if recipe in {"kitting_bench", "assembly_bench", "packing_bench"}:
        role = {"kitting_bench": "kitting", "assembly_bench": "pcb_assembly", "packing_bench": "packing"}[recipe]
        top_bottom = height * .68
        top_top = height * .78
        mat_bottom = height * .79
        mat_top = height * .81
        boxes = [
            b("Worktop", .03, .05, top_bottom, .97, .95, top_top, "MachinePanel"),
            b("LegL", .06, .10, 0, .12, .90, top_bottom, "Steel"),
            b("LegR", .88, .10, 0, .94, .90, top_bottom, "Steel"),
            b("UnderShelf", .10, .13, height * .18, .90, .87, height * .24, "MachinePaint"),
            b("RearRiser", .04, .88, top_top, .96, .94, height * .97, "Steel"),
            b("ToolRail", .12, .84, height * .86, .88, .90, height * .91, "Accent"),
            b("ESDMat", .22, .18, mat_bottom, .78, .78, mat_top, "ESD", "workbench", role),
            b("JigBase", .38, .30, height * .82, .62, .52, height * .86, "MachinePaint"),
        ]
        if recipe != "packing_bench":
            boxes.extend([b("Monitor", .38, .55, height * .82, .62, .63, height * .98, "Dark"), b("Terminal", .43, .43, height * .82, .57, .58, height * .90, "Accent", "process_equipment", "operator_terminal")])
        for index in range(3):
            fx = .16 + .25 * ((index + seed) % 3)
            boxes.append(b(f"Bin_{index}", fx, .12, height * .82, fx + .15, .34, height * .96, "ToteBlue" if index % 2 == 0 else "ToteOrange", "tote", "work_in_process_bin"))
        if recipe == "packing_bench":
            boxes.extend([
                b("TapeStand", .69, .48, height * .82, .76, .62, height * .94, "SafetyYellow"),
                b("ScaleBed", .28, .42, height * .82, .58, .72, height * .85, "Dark"),
            ])
        return boxes
    if recipe == "inspection_station":
        return [
            b("Plinth", .04, .05, 0, .96, .95, height * .24, "MachinePaint"),
            b("LeftTower", .04, .05, height * .24, .17, .95, height, "MachinePanel"),
            b("RightTower", .83, .05, height * .24, .96, .95, height, "MachinePanel"),
            b("Roof", .04, .05, height * .88, .96, .95, height, "MachinePanel"),
            b("ChamberBed", .17, .18, height * .34, .83, .90, height * .43, "Steel"),
            b("OpticalWindow", .17, .03, height * .43, .83, .08, height * .88, "Glass", "process_equipment", "inspection_window"),
            b("ControlPanel", .22, .01, height * .27, .78, .055, height * .40, "Dark"),
            b("DoorHandle", .74, .0, height * .55, .79, .05, height * .75, "SafetyYellow"),
            b("StatusGreen", .18, .005, height * .90, .34, .04, height * .97, "Green"),
        ]
    if recipe == "process_cell":
        common = [
            b("Plinth", .04, .05, 0, .96, .95, height * .22, "MachinePaint"),
            b("FrontStep", .14, .0, height * .08, .86, .06, height * .18, "Dark"),
        ]
        if obstacle_id == "PROCESS_1":
            return common + [
                b("WorkBed", .12, .16, height * .30, .88, .88, height * .39, "Steel"),
                b("AssemblyNest", .30, .24, height * .40, .70, .58, height * .45, "Accent", "process_equipment", "guarded_load_cell"),
                b("LocatorLeft", .34, .28, height * .45, .39, .52, height * .53, "SafetyYellow"),
                b("LocatorRight", .61, .28, height * .45, .66, .52, height * .53, "SafetyYellow"),
                b("ClampBridge", .42, .42, height * .46, .58, .50, height * .61, "Dark"),
                b("GuardPostFL", .06, .08, height * .22, .11, .14, height * .91, "MachinePaint"),
                b("GuardPostFR", .89, .08, height * .22, .94, .14, height * .91, "MachinePaint"),
                b("GuardPostBL", .06, .88, height * .22, .11, .94, height * .91, "MachinePaint"),
                b("GuardPostBR", .89, .88, height * .22, .94, .94, height * .91, "MachinePaint"),
                b("GuardCrossbeamFront", .06, .08, height * .84, .94, .14, height * .91, "MachinePaint"),
                b("GuardCrossbeamRear", .06, .88, height * .84, .94, .94, height * .91, "MachinePaint"),
                b("SafetyScanner", .10, .045, height * .25, .18, .10, height * .38, "SafetyYellow"),
                b("HMI", .75, .025, height * .52, .91, .08, height * .72, "Dark", "process_equipment", "hmi"),
                b("BeaconGreen", .885, .08, height * .91, .945, .15, height, "Green"),
            ]
        if obstacle_id == "PROCESS_2":
            return common + [
                b("RearShell", .06, .88, height * .22, .94, .94, height * .91, "MachinePanel"),
                b("LeftShell", .06, .12, height * .22, .12, .88, height * .91, "MachinePanel"),
                b("RightShell", .88, .12, height * .22, .94, .88, height * .91, "MachinePanel"),
                b("RoofFrontRail", .06, .12, height * .86, .94, .20, height * .92, "MachinePaint"),
                b("RoofRearRail", .06, .80, height * .86, .94, .88, height * .92, "MachinePaint"),
                b("DoorFrameLeft", .08, .055, height * .24, .16, .12, height * .88, "MachinePaint"),
                b("DoorFrameRight", .84, .055, height * .24, .92, .12, height * .88, "MachinePaint"),
                b("DoorFrameTop", .08, .055, height * .82, .92, .12, height * .88, "MachinePaint"),
                b("DoorFrameBottom", .08, .055, height * .24, .92, .12, height * .32, "MachinePaint"),
                b("ProcessWindow", .16, .06, height * .32, .72, .095, height * .82, "Glass", "process_equipment", "enclosed_inspection"),
                b("InspectionStage", .25, .30, height * .34, .75, .72, height * .42, "Steel"),
                b("CameraMast", .46, .67, height * .42, .54, .74, height * .76, "Dark"),
                b("CameraHead", .40, .54, height * .68, .60, .70, height * .79, "Accent"),
                b("CameraLens", .46, .49, height * .70, .54, .56, height * .77, "Glass"),
                b("DoorHandle", .48, .015, height * .48, .52, .07, height * .70, "SafetyYellow"),
                b("HMI", .72, .015, height * .30, .91, .07, height * .45, "Dark", "process_equipment", "hmi"),
                b("Vent", .10, .01, height * .27, .34, .06, height * .34, "Dark"),
                b("BeaconRed", .88, .34, height * .91, .94, .42, height, "SafetyRed"),
            ]
        discharge = common + [
            b("ConveyorBase", .10, .10, height * .28, .90, .92, height * .36, "MachinePaint"),
            b("ConveyorRailLeft", .08, .08, height * .35, .14, .94, height * .48, "Steel"),
            b("ConveyorRailRight", .86, .08, height * .35, .92, .94, height * .48, "Steel"),
            b("DischargeFixture", .18, .75, height * .37, .82, .91, height * .45, "Accent", "process_equipment", "discharge_pack"),
            b("PackStop", .22, .86, height * .45, .78, .92, height * .58, "SafetyYellow"),
            b("LabelerPostLeft", .10, .50, height * .43, .17, .58, height * .82, "MachinePaint"),
            b("LabelerPostRight", .83, .50, height * .43, .90, .58, height * .82, "MachinePaint"),
            b("LabelerCrossbeam", .10, .50, height * .76, .90, .58, height * .83, "MachinePaint"),
            b("LabelHead", .42, .45, height * .65, .58, .56, height * .76, "Dark"),
            b("HMI", .72, .02, height * .48, .91, .08, height * .68, "Dark", "process_equipment", "hmi"),
            b("BeaconGreen", .835, .50, height * .83, .895, .58, height, "Green"),
        ]
        for roller in range(8):
            fy = .13 + roller * .085
            discharge.append(b(f"Roller_{roller}", .15, fy, height * .37, .85, fy + .025, height * .43, "Steel"))
        return discharge
    if recipe in {"control_cabinet", "service_cabinet"}:
        if geometry_profile == PRODUCTION_BAY_GEOMETRY:
            module_count = max(2, int(w // .85))
            margin, gap = .08, .05
            module_width = (w - 2 * margin - gap * (module_count - 1)) / module_count
            if not .6 <= module_width <= 1.0:
                raise LayoutError(f"{obstacle_id} cannot fit 0.6-1.0m cabinet modules")
            front = y0 + d * (.05 if recipe == "service_cabinet" else .02)
            body_front, body_back = y0 + d * .12, y1 - d * .05
            boxes: list[_Box] = []
            for index in range(module_count):
                left = x0 + margin + index * (module_width + gap)
                right = left + module_width
                prefix = f"Module_{index:02d}"
                boxes.extend([
                    absolute(prefix, left, body_front, height * .12, right, body_back, height, "MachinePanel", "control_cabinet", "cabinet_module"),
                    absolute(f"{prefix}_Plinth", left + .03, y0 + d * .08, 0, right - .03, body_back, height * .12, "MachinePaint"),
                    absolute(f"{prefix}_Door", left + .025, front, height * .18, right - .025, body_front, height * .91, "MachinePaint"),
                    absolute(f"{prefix}_Handle", right - .13, front, height * .43, right - .085, body_front, height * .65, "Dark"),
                    absolute(f"{prefix}_VentLow", left + .12, front, height * .24, right - .18, body_front, height * .29, "Dark"),
                    absolute(f"{prefix}_VentHigh", left + .12, front, height * .80, right - .18, body_front, height * .85, "Dark"),
                ])
            boxes.extend([
                absolute("StatusGreen", x0 + margin + .12, front, height * .70, x0 + margin + .20, body_front, height * .77, "Green"),
                absolute("EStop", x1 - margin - .20, front, height * .57, x1 - margin - .10, body_front, height * .66, "SafetyRed", "control_cabinet", "emergency_stop"),
            ])
            return boxes
        body_front = .10 if obstacle_id == "SERVICE_CABINET" else .05
        facade_front = .05 if obstacle_id == "SERVICE_CABINET" else .01
        facade_back = .11 if obstacle_id == "SERVICE_CABINET" else .07
        boxes = [
            b("Plinth", .03, .05, 0, .97, .95, height * .12, "MachinePaint"),
            b("CabinetBody", .03, body_front, height * .12, .97, .95, height, "MachinePanel"),
            b("DoorInset", .08, facade_front, height * .18, .92, facade_back, height * .88, "MachinePaint"),
            b("DoorSeam", .49, facade_front, height * .18, .51, facade_back, height * .88, "Dark"),
            b("DoorHandle", .68, facade_front, height * .38, .72, facade_back, height * .62, "Dark"),
            b("VentLow", .17, facade_front, height * .20, .42, facade_back, height * .27, "Dark"),
            b("VentHigh", .17, facade_front, height * .82, .42, facade_back, height * .87, "Dark"),
            b("EStop", .75, facade_front, height * .54, .84, facade_back, min(height * .64, height), "SafetyRed", "control_cabinet", "emergency_stop"),
        ]
        for index in range(3):
            boxes.append(b(f"Status_{index}", .18 + .16 * index, facade_front, height * .72, .27 + .16 * index, facade_back, height * .79, "Green" if index == 0 else "SafetyYellow"))
        return boxes
    if recipe == "buffer_fixture":
        return [b("Rail", .10, .02, 0, .90, .98, .18, "Steel"), b("Slot0", .15, .08, .19, .85, .45, .24, "Accent", "buffer_fixture", "slot_0"), b("Slot1", .15, .55, .19, .85, .92, .24, "Accent2", "buffer_fixture", "slot_1"), b("Lamp0", .20, .15, .25, .80, .28, min(.65, height), "Green"), b("Lamp1", .20, .68, .25, .80, .81, min(.65, height), "SafetyYellow")]
    if recipe == "finished_goods":
        if geometry_profile == PRODUCTION_BAY_GEOMETRY:
            pallet_width, pallet_depth = min(1.2, (w - .8) / 2), min(1.0, (d - .2) / 2)
            x_margin = (w - 2 * pallet_width - .8) / 2
            y_margin = (d - 2 * pallet_depth - .2) / 2
            boxes = []
            tote_index = 0
            for pallet_index in range(4):
                col, row = pallet_index % 2, pallet_index // 2
                px0 = x0 + x_margin + col * (pallet_width + .8)
                py0 = y0 + y_margin + row * (pallet_depth + .2)
                px1, py1 = px0 + pallet_width, py0 + pallet_depth
                prefix = f"Pallet_{pallet_index:02d}"
                boxes.extend([
                    absolute(prefix, px0, py0, .08, px1, py1, .14, "Wood", "pallet", "finished_goods_pallet"),
                    absolute(f"{prefix}_RunnerL", px0 + .08, py0 + .05, 0, px0 + .20, py1 - .05, .08, "Wood"),
                    absolute(f"{prefix}_RunnerC", (px0 + px1) / 2 - .06, py0 + .05, 0, (px0 + px1) / 2 + .06, py1 - .05, .08, "Wood"),
                    absolute(f"{prefix}_RunnerR", px1 - .20, py0 + .05, 0, px1 - .08, py1 - .05, .08, "Wood"),
                ])
                tote_count = 0 if pallet_index == 1 else 2
                for local_index in range(tote_count):
                    tote_width, tote_depth = .50, .38
                    tx0 = px0 + ((pallet_width - tote_width) / 2 if tote_count == 1 else .08 + local_index * (tote_width + .04))
                    ty0 = py0 + (pallet_depth - tote_depth) / 2
                    boxes.append(absolute(
                        f"FinishedTote_{tote_index:02d}", tx0, ty0, .15,
                        tx0 + tote_width, ty0 + tote_depth, min(.58, height),
                        "ToteBlue" if (tote_index + seed) % 2 == 0 else "ToteOrange",
                        "tote", "finished_goods_tote",
                    ))
                    tote_index += 1
            return boxes
        boxes = [
            b("Pallet", .04, .08, 0, .96, .92, .10, "Wood", "pallet", "finished_goods_pallet"),
            b("PalletRunnerL", .08, .12, .10, .16, .88, .16, "Wood"),
            b("PalletRunnerC", .46, .12, .10, .54, .88, .16, "Wood"),
            b("PalletRunnerR", .84, .12, .10, .92, .88, .16, "Wood"),
        ]
        for index in range(6):
            col, row = index % 3, index // 3
            boxes.append(b(f"FinishedTote_{index}", .10 + .30 * col, .13 + .42 * row, .17, .34 + .30 * col, .45 + .42 * row, min(.70, height), "ToteBlue" if (index + seed) % 2 == 0 else "ToteOrange", "tote", "finished_goods_tote"))
        return boxes
    raise LayoutError(f"unsupported visual recipe: {recipe}")


def _validate_boxes(obstacle_id: str, boxes: Sequence[_Box], source: tuple[float, float, float, float, float, float]) -> None:
    sx0, sy0, sz0, sx1, sy1, sz1 = source
    names: set[str] = set()
    for box in boxes:
        if box.name in names:
            raise LayoutError(f"visual recipe {obstacle_id} contains duplicate component {box.name}")
        names.add(box.name)
        x0, y0, z0, x1, y1, z1 = box.bounds
        if not all(math.isfinite(value) for value in box.bounds) or not (x0 < x1 and y0 < y1 and z0 < z1):
            raise LayoutError(f"visual component {obstacle_id}/{box.name} has invalid bounds")
        if x0 < sx0 or y0 < sy0 or z0 < sz0 or x1 > sx1 or y1 > sy1 or z1 > sz1:
            raise LayoutError(f"visual component {obstacle_id}/{box.name} exceeds its source obstacle envelope")


def _chamfered_box_geometry(
    bounds: tuple[float, float, float, float, float, float],
    chamfer_m: float,
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, ...], ...]]:
    """Return a closed, right-handed render mesh with the exact source bounds."""
    x0, y0, z0, x1, y1, z1 = bounds
    dimensions = (x1 - x0, y1 - y0, z1 - z0)
    if (not all(math.isfinite(value) for value in (*bounds, chamfer_m))
            or not all(value > 0 for value in dimensions)
            or not 0 < chamfer_m < min(dimensions) / 2):
        raise LayoutError("render chamfer must be finite, positive and smaller than every half-extent")
    points: list[tuple[float, float, float]] = []
    index: dict[tuple[str, int, int, int], int] = {}
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                x_face, y_face, z_face = (x0 if sx < 0 else x1), (y0 if sy < 0 else y1), (z0 if sz < 0 else z1)
                x_inset = x0 + chamfer_m if sx < 0 else x1 - chamfer_m
                y_inset = y0 + chamfer_m if sy < 0 else y1 - chamfer_m
                z_inset = z0 + chamfer_m if sz < 0 else z1 - chamfer_m
                authored = {
                    "x": (x_face, y_inset, z_inset),
                    "y": (x_inset, y_face, z_inset),
                    "z": (x_inset, y_inset, z_face),
                }
                for axis in ("x", "y", "z"):
                    index[(axis, sx, sy, sz)] = len(points)
                    points.append(authored[axis])

    def face(keys: Sequence[tuple[str, int, int, int]], outward: tuple[int, int, int]) -> tuple[int, ...]:
        authored = tuple(index[key] for key in keys)
        a, b, c = (points[authored[position]] for position in range(3))
        ab, ac = tuple(b[i] - a[i] for i in range(3)), tuple(c[i] - a[i] for i in range(3))
        normal = (ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0])
        return authored if sum(normal[i] * outward[i] for i in range(3)) > 0 else tuple(reversed(authored))

    faces: list[tuple[int, ...]] = []
    for sx in (-1, 1):
        faces.append(face([("x", sx, -1, -1), ("x", sx, 1, -1), ("x", sx, 1, 1), ("x", sx, -1, 1)], (sx, 0, 0)))
    for sy in (-1, 1):
        faces.append(face([("y", -1, sy, -1), ("y", -1, sy, 1), ("y", 1, sy, 1), ("y", 1, sy, -1)], (0, sy, 0)))
    for sz in (-1, 1):
        faces.append(face([("z", -1, -1, sz), ("z", 1, -1, sz), ("z", 1, 1, sz), ("z", -1, 1, sz)], (0, 0, sz)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            faces.append(face([("x", sx, sy, -1), ("y", sx, sy, -1), ("y", sx, sy, 1), ("x", sx, sy, 1)], (sx, sy, 0)))
        for sz in (-1, 1):
            faces.append(face([("x", sx, -1, sz), ("z", sx, -1, sz), ("z", sx, 1, sz), ("x", sx, 1, sz)], (sx, 0, sz)))
    for sy in (-1, 1):
        for sz in (-1, 1):
            faces.append(face([("y", -1, sy, sz), ("z", -1, sy, sz), ("z", 1, sy, sz), ("y", 1, sy, sz)], (0, sy, sz)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                faces.append(face([("x", sx, sy, sz), ("y", sx, sy, sz), ("z", sx, sy, sz)], (sx, sy, sz)))
    return tuple(points), tuple(faces)


def _box_usda(
    box: _Box,
    indent: int,
    *,
    opacity: float | None = None,
    stable_id: str | None = None,
    source_obstacle_id: str | None = None,
) -> list[str]:
    x0, y0, z0, x1, y1, z1 = box.bounds
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    sx, sy, sz = (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2
    pad = " " * indent
    lines = [
        f'{pad}def {"Mesh" if box.render_style == "chamfered_box" else "Cube"} "{_prim_name(box.name)}"',
        f"{pad}{{",
        f"{pad}    custom string stableId = {_quote(stable_id or box.name)}",
        f"{pad}    rel material:binding = </World/Visuals/Looks/{box.material}>",
    ]
    if box.render_style == "chamfered_box":
        points, faces = _chamfered_box_geometry(box.bounds, box.render_chamfer_m)
        lines.extend([
            f'{pad}    custom string geometryStyle = "edge_chamfered_render_mesh"',
            f"{pad}    custom float renderEdgeChamferM = {_num(box.render_chamfer_m)}",
            f"{pad}    float3[] extent = [({_num(x0)}, {_num(y0)}, {_num(z0)}), ({_num(x1)}, {_num(y1)}, {_num(z1)})]",
            f"{pad}    int[] faceVertexCounts = [{', '.join(str(len(face)) for face in faces)}]",
            f"{pad}    int[] faceVertexIndices = [{', '.join(str(index) for face in faces for index in face)}]",
            f"{pad}    point3f[] points = [{', '.join(f'({_num(x)}, {_num(y)}, {_num(z)})' for x, y, z in points)}]",
            f'{pad}    uniform token orientation = "rightHanded"',
            f'{pad}    uniform token subdivisionScheme = "none"',
        ])
    else:
        lines.extend([
            f"{pad}    double size = 2",
            f"{pad}    double3 xformOp:scale = ({_num(sx)}, {_num(sy)}, {_num(sz)})",
            f"{pad}    double3 xformOp:translate = ({_num(cx)}, {_num(cy)}, {_num(cz)})",
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        ])
    if box.semantic_class is not None:
        lines.append(f"{pad}    custom string semanticClass = {_quote(box.semantic_class)}")
    if box.state_role is not None:
        lines.append(f"{pad}    custom string stateRole = {_quote(box.state_role)}")
    if source_obstacle_id is not None:
        lines.append(f"{pad}    custom string sourceObstacleId = {_quote(source_obstacle_id)}")
    if opacity is not None:
        lines.append(f"{pad}    custom float displayOpacity = {_num(opacity)}")
    return lines + [f"{pad}}}"]


def _composition_text(obstacles: Mapping[str, object]) -> str:
    lines = [
        "#usda 1.0",
        "(",
        '    defaultPrim = "World"',
        '    upAxis = "Z"',
        "    metersPerUnit = 1",
        "    subLayers = [",
        "        @./factory-v3.usda@,",
        "        @./factory-v3.visual.usda@",
        "    ]",
        ")",
        "",
        'over "World"',
        "{",
        '    over "Obstacles"',
        "    {",
    ]
    for obstacle_id in sorted(obstacles):
        lines.extend([f'        over "{_prim_name(obstacle_id)}"', "        {", '            token visibility = "invisible"', "        }"])
    return "\n".join(lines + ["    }", "}", ""])


def _prim_name(value: str) -> str:
    if not value:
        raise LayoutError("visual prim name source must not be empty")
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return "_" + result if result[0].isdigit() else result


def _is_finite_number(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))


def _quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def _vec(values: Sequence[float]) -> str:
    return "(" + ", ".join(_num(value) for value in values) + ")"


def _num(value: float) -> str:
    if not math.isfinite(value):
        raise LayoutError("visual USD numeric value must be finite")
    if value == int(value):
        return str(int(value))
    return format(value, ".12g")


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()
