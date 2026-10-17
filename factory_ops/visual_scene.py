"""Deterministic, collision-free industrial visual layer for the factory twin."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

from .layout import Layout, LayoutError, Rect


SCHEMA_VERSION = 1
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "visual-assets.json"
DEFAULT_RENDER_PROFILE = Path(__file__).resolve().parents[1] / "configs" / "factory-v3" / "render-profile.json"


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


def export_visual_scene(
    layout: Layout,
    output_dir: str | Path,
    variant_seed: int = 0,
    config_path: str | Path | None = None,
) -> dict[str, object]:
    """Author a visual-only USD layer and a composition over ``factory-v3.usda``.

    The visual layer never authors collision APIs. Equipment geometry remains
    inside the source obstacle's authored XYZ envelope; the only geometry over
    free map cells is millimetre-thin floor paint and overhead lighting.
    """
    if isinstance(variant_seed, bool) or not isinstance(variant_seed, int) or not 0 <= variant_seed < 12:
        raise LayoutError("variant_seed must be an integer in [0, 11]")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    base_scene = output / "factory-v3.usda"
    _require_regular_file(base_scene, "base scene")

    config_source = Path(config_path) if config_path is not None else DEFAULT_CONFIG
    config_bytes, config = _load_json_object(config_source, "visual asset config")
    render_bytes, render_profile = _load_json_object(DEFAULT_RENDER_PROFILE, "render profile")
    _validate_config(config, layout)
    _validate_render_profile(render_profile)

    palette = _palette(variant_seed)
    obstacle_by_id = {str(item["id"]): item for item in layout.data["physical_obstacles"]}
    instances: list[VisualInstance] = []
    lines = _visual_header(variant_seed, palette)
    lines.extend(_materials(palette))
    lines.extend(_factory_shell(palette))
    lines.extend(_floor_graphics(layout, palette))
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
        boxes = _recipe_boxes(recipe, rect, height, variant_seed, obstacle_id)
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
        "render_profile": {"path": str(DEFAULT_RENDER_PROFILE), "sha256": render_sha, "value": render_profile},
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
            "external_asset_packages": [],
        },
        "semantic_taxonomy": config["required_semantic_taxonomy"],
        "runtime_semantics": config["runtime_semantics"],
        "instances": [_instance_document(instance) for instance in instances],
        "physical_invariants": {
            "source_obstacle_count": 19,
            "source_static_collider_count": 20,
            "source_marker_count": 8,
            "visual_collision_api_count": 0,
            "layout_bounds_xy_m": [0.0, 0.0, 40.0, 24.0],
            "visual_equipment_contained_by_source_xyz_envelopes": True,
            "station_centres_free_of_visual_payload_geometry": True,
        },
        "variation_contract": {
            "changes": ["material palette", "light temperature", "prop order inside source obstacle envelopes"],
            "unchanged": ["source obstacle bounds", "physical colliders", "aisles", "stations", "LiDAR reference envelopes", "instance stable IDs"],
        },
    }
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
    if set(config) != required:
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
    valid_recipes = {"kitting_bench", "flow_rack", "assembly_bench", "inspection_station", "packing_bench", "process_cell", "control_cabinet", "buffer_fixture", "finished_goods", "service_cabinet"}
    for obstacle_id, value in assets.items():
        if not isinstance(value, dict) or set(value) != {"recipe", "semantic_class", "state_role"}:
            raise LayoutError(f"visual asset assignment {obstacle_id} is invalid")
        if value["recipe"] not in valid_recipes or value["semantic_class"] not in expected_taxonomy:
            raise LayoutError(f"visual asset assignment {obstacle_id} has an unsupported recipe or class")
        if not isinstance(value["state_role"], str) or not value["state_role"]:
            raise LayoutError(f"visual asset assignment {obstacle_id} state_role is invalid")


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


def _materials(palette: Mapping[str, tuple[float, float, float]]) -> list[str]:
    definitions = {
        "Floor": ((0.62, 0.66, 0.70), 0.0, 0.72),
        "Steel": (palette["steel"], 0.72, 0.32),
        "LightSteel": ((0.72, 0.76, 0.79), 0.56, 0.30),
        "Dark": ((0.035, 0.045, 0.06), 0.18, 0.46),
        "Accent": (palette["accent"], 0.25, 0.28),
        "Accent2": (palette["accent2"], 0.18, 0.30),
        "SafetyYellow": ((0.98, 0.68, 0.02), 0.05, 0.42),
        "SafetyRed": ((0.78, 0.035, 0.025), 0.05, 0.35),
        "Green": ((0.04, 0.72, 0.23), 0.0, 0.34),
        "White": ((0.92, 0.94, 0.96), 0.05, 0.55),
        "ToteBlue": ((0.04, 0.28, 0.56), 0.05, 0.40),
        "ToteOrange": ((0.92, 0.31, 0.035), 0.05, 0.40),
        "Glass": ((0.10, 0.25, 0.34), 0.12, 0.12),
        "ESD": ((0.08, 0.14, 0.18), 0.0, 0.78),
        "Wood": ((0.47, 0.25, 0.08), 0.0, 0.72),
        "Light": (palette["light"], 0.0, 0.18),
    }
    lines = ['        def Scope "Looks"', "        {"]
    for name, (color, metallic, roughness) in definitions.items():
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
            "                    token outputs:surface",
            "                }",
            "            }",
        ])
    return lines + ["        }"]


def _factory_shell(palette: Mapping[str, tuple[float, float, float]]) -> list[str]:
    light = palette["light"]
    lines = ['        def Xform "FactoryShell"', "        {"]
    for name, bounds in (
        ("WestWall", (-0.12, 0.0, 0.0, 0.0, 24.0, 3.2)),
        ("EastWall", (40.0, 0.0, 0.0, 40.12, 24.0, 3.2)),
        ("SouthWall", (0.0, -0.12, 0.0, 40.0, 0.0, 1.2)),
        ("NorthWall", (0.0, 24.0, 0.0, 40.0, 24.12, 3.2)),
    ):
        lines.extend(_box_usda(_Box(name, bounds, "White"), 12))
    for index, x in enumerate((5.0, 15.0, 25.0, 35.0)):
        for row, y in enumerate((4.0, 12.0, 20.0)):
            lines.extend(_box_usda(_Box(f"LED_{index}_{row}", (x - 1.7, y - 0.08, 5.4, x + 1.7, y + 0.08, 5.46), "Light"), 12))
    lines.extend([
        f"            custom color3f authoredLightColor = {_vec(light)}",
        '            custom string roofMode = "cutaway"',
        "        }",
    ])
    return lines


def _floor_graphics(layout: Layout, palette: Mapping[str, tuple[float, float, float]]) -> list[str]:
    del palette
    lines = [
        '        def Xform "FloorGraphics"',
        "        {",
        '            custom string stableId = "floor_graphics"',
        '            custom string semanticClass = "floor"',
        '            custom string stateRole = "floor_and_markings"',
    ]
    lines.extend(_box_usda(_Box("FloorFinish", (0.0, 0.0, 0.001, 40.0, 24.0, 0.006), "Floor", "floor", "factory_floor"), 12))
    zones = (
        ("WestZone", (0.2, 0.2, 0.007, 11.0, 23.8, 0.010), "Accent"),
        ("CellZone", (12.0, 5.2, 0.007, 30.5, 18.8, 0.010), "Accent2"),
        ("ShippingZone", (32.5, 5.2, 0.007, 39.8, 18.8, 0.010), "Green"),
    )
    for name, bounds, material in zones:
        lines.extend(_box_usda(_Box(name, bounds, material, "floor_marking", "zone_color"), 12, opacity=0.12))
    for index, y in enumerate((2.3, 5.7, 10.2, 13.8, 18.3, 21.7)):
        lines.extend(_box_usda(_Box(f"AisleLine_{index}", (0.4, y - 0.025, 0.011, 39.6, y + 0.025, 0.016), "SafetyYellow", "floor_marking", "safety_line"), 12))
    for marker_id, xy in sorted({**layout.stations, **layout.pockets}.items()):
        x, y = xy
        root = _prim_name(marker_id)
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
    lines.extend(["        }"])
    return lines


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
        lines.extend(_box_usda(
            box,
            indent + 4,
            stable_id=f"{obstacle_id}:{box.name}",
            source_obstacle_id=obstacle_id,
        ))
    return lines + [f"{pad}}}"]


def _recipe_boxes(recipe: str, rect: Rect, height: float, seed: int, obstacle_id: str) -> list[_Box]:
    del obstacle_id
    x0, y0, x1, y1 = rect.xmin, rect.ymin, rect.xmax, rect.ymax
    w, d = x1 - x0, y1 - y0
    def b(name: str, fx0: float, fy0: float, z0: float, fx1: float, fy1: float, z1: float, material: str, semantic: str | None = None, role: str | None = None) -> _Box:
        return _Box(name, (x0 + w * fx0, y0 + d * fy0, z0, x0 + w * fx1, y0 + d * fy1, min(z1, height)), material, semantic, role)

    if recipe == "flow_rack":
        boxes = [
            b("Post_FL", .02, .05, 0, .055, .14, height, "Steel"), b("Post_FR", .945, .05, 0, .98, .14, height, "Steel"),
            b("Post_BL", .02, .86, 0, .055, .95, height, "Steel"), b("Post_BR", .945, .86, 0, .98, .95, height, "Steel"),
        ]
        for level, z in enumerate((.18, .72, 1.26, 1.82)):
            boxes.append(b(f"Shelf_{level}", .02, .04, z, .98, .96, z + .07, "LightSteel"))
        order = list(range(8))
        shift = seed % len(order)
        order = order[shift:] + order[:shift]
        for index, value in enumerate(order):
            column, row = index % 4, index // 4
            material = "ToteBlue" if value % 2 == 0 else "ToteOrange"
            boxes.append(b(f"Tote_{index:02d}", .08 + .225 * column, .12 + .46 * row, .27 + .55 * row, .25 + .225 * column, .42 + .46 * row, .62 + .55 * row, material, "tote", "inventory_tote"))
        return boxes
    if recipe in {"kitting_bench", "assembly_bench", "packing_bench"}:
        role = {"kitting_bench": "kitting", "assembly_bench": "pcb_assembly", "packing_bench": "packing"}[recipe]
        top_bottom = height * .68
        top_top = height * .78
        mat_bottom = height * .79
        mat_top = height * .81
        boxes = [b("Worktop", .03, .05, top_bottom, .97, .95, top_top, "LightSteel"), b("LegL", .06, .10, 0, .12, .90, top_bottom, "Steel"), b("LegR", .88, .10, 0, .94, .90, top_bottom, "Steel"), b("ESDMat", .22, .18, mat_bottom, .78, .78, mat_top, "ESD", "workbench", role)]
        if recipe != "packing_bench":
            boxes.extend([b("Monitor", .38, .55, height * .82, .62, .63, height * .98, "Dark"), b("Terminal", .43, .43, height * .82, .57, .58, height * .90, "Accent", "process_equipment", "operator_terminal")])
        for index in range(3):
            fx = .16 + .25 * ((index + seed) % 3)
            boxes.append(b(f"Bin_{index}", fx, .12, height * .82, fx + .15, .34, height * .96, "ToteBlue" if index % 2 == 0 else "ToteOrange", "tote", "work_in_process_bin"))
        return boxes
    if recipe == "inspection_station":
        return [b("Cabinet", .04, .05, 0, .96, .95, height, "LightSteel"), b("OpticalWindow", .10, .02, .72, .90, .07, min(1.36, height), "Glass", "process_equipment", "inspection_window"), b("ControlPanel", .18, .01, .25, .82, .055, .58, "Dark"), b("StatusGreen", .18, .005, min(1.42, height - .12), .34, .04, min(1.55, height), "Green")]
    if recipe == "process_cell":
        return [b("MachineBody", .05, .05, 0, .95, .95, height * .78, "LightSteel"), b("AccessDoor", .18, .01, .18, .82, .06, height * .66, "Steel"), b("ProcessWindow", .26, .005, height * .38, .74, .04, height * .65, "Glass", "process_equipment", "process_view"), b("HMI", .72, .0, height * .52, .93, .045, height * .70, "Dark", "process_equipment", "hmi"), b("BeaconRed", .78, .20, height * .78, .84, .28, height * .96, "SafetyRed"), b("BeaconGreen", .78, .20, height * .96, .84, .28, height, "Green")]
    if recipe in {"control_cabinet", "service_cabinet"}:
        boxes = [b("CabinetBody", .03, .05, 0, .97, .95, height, "LightSteel"), b("DoorInset", .08, .01, .12, .92, .07, height * .88, "Steel"), b("EStop", .75, .0, height * .54, .84, .05, min(height * .64, height), "SafetyRed", "control_cabinet", "emergency_stop")]
        for index in range(3):
            boxes.append(b(f"Status_{index}", .18 + .16 * index, .0, height * .72, .27 + .16 * index, .045, height * .79, "Green" if index == 0 else "SafetyYellow"))
        return boxes
    if recipe == "buffer_fixture":
        return [b("Rail", .10, .02, 0, .90, .98, .18, "Steel"), b("Slot0", .15, .08, .19, .85, .45, .24, "Accent", "buffer_fixture", "slot_0"), b("Slot1", .15, .55, .19, .85, .92, .24, "Accent2", "buffer_fixture", "slot_1"), b("Lamp0", .20, .15, .25, .80, .28, min(.65, height), "Green"), b("Lamp1", .20, .68, .25, .80, .81, min(.65, height), "SafetyYellow")]
    if recipe == "finished_goods":
        boxes = [b("Pallet", .04, .08, 0, .96, .92, .16, "Wood", "pallet", "finished_goods_pallet")]
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
        f'{pad}def Cube "{_prim_name(box.name)}"',
        f"{pad}{{",
        f"{pad}    custom string stableId = {_quote(stable_id or box.name)}",
        f"{pad}    rel material:binding = </World/Visuals/Looks/{box.material}>",
        f"{pad}    double size = 2",
        f"{pad}    double3 xformOp:scale = ({_num(sx)}, {_num(sy)}, {_num(sz)})",
        f"{pad}    double3 xformOp:translate = ({_num(cx)}, {_num(cy)}, {_num(cz)})",
        f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
    ]
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
