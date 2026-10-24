"""Render recorded CPU operations in the factory USD scene; physics stays stopped."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace
from typing import Any, Callable
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from factory_ops.operations_render import load_recording, sha
from factory_ops.layout import CAMPUS_BAY_OFFSETS_BY_REVISION, CAMPUS_LAYOUT_REVISIONS


_WORLD_SLOT_NAMES = ("source_S1", "source_S2", "source_S3", "ci", "cell", "co", "out")


def replay_slot_placement(
    layout: Any, location: str, slot: int,
    placement_fn: Callable[[Any, str, int], dict[str, Any]],
) -> dict[str, Any]:
    """Resolve legacy or prefixed campus presentation slots without changing scene geometry."""
    if layout.data.get("revision") not in CAMPUS_LAYOUT_REVISIONS:
        return placement_fn(layout, location, slot)
    bay_offsets = CAMPUS_BAY_OFFSETS_BY_REVISION[layout.data["revision"]]
    bay, separator, local_location = location.partition("_")
    if not separator or bay not in bay_offsets or local_location not in _WORLD_SLOT_NAMES:
        raise ValueError(f"campus world cargo location is unsupported: {location}")
    dx, dy = bay_offsets[bay]
    prefix = f"{bay}_"
    obstacles = []
    for row in layout.data["physical_obstacles"]:
        if str(row["id"]).startswith(prefix):
            copied = dict(row)
            copied["id"] = str(row["id"])[len(prefix):]
            copied["rect"] = [row["rect"][0] - dx, row["rect"][1] - dy,
                              row["rect"][2] - dx, row["rect"][3] - dy]
            obstacles.append(copied)
    stations = {
        str(row["id"])[len(prefix):]: (row["xy"][0] - dx, row["xy"][1] - dy)
        for row in layout.data["stations"] if str(row["id"]).startswith(prefix)
    }
    bay_view = SimpleNamespace(data={
        "revision": "amr-semiconductor-dual-lane-v1",
        "physical_obstacles": obstacles,
        "production_domain": layout.data.get("production_domain", {}),
    }, stations=stations)
    placement = dict(placement_fn(bay_view, local_location, slot))
    placement["location"] = location
    placement["source_obstacle_id"] = prefix + str(placement["source_obstacle_id"])
    placement["parent_path"] = f"/World/Visuals/CargoSlots/{bay}_{local_location}"
    placement["translate_xyz_m"] = [placement["translate_xyz_m"][0] + dx,
                                     placement["translate_xyz_m"][1] + dy,
                                     placement["translate_xyz_m"][2]]
    bounds = placement["bounds_xyz_m"]
    placement["bounds_xyz_m"] = [bounds[0] + dx, bounds[1] + dy, bounds[2],
                                  bounds[3] + dx, bounds[4] + dy, bounds[5]]
    if "support_bounds_xyz_m" in placement:
        support = placement["support_bounds_xyz_m"]
        placement["support_bounds_xyz_m"] = [support[0] + dx, support[1] + dy, support[2],
                                               support[3] + dx, support[4] + dy, support[5]]
    return placement


def replay_slot_capacities(layout: Any, placement_fn: Callable[[Any, str, int], dict[str, Any]]) -> dict[str, int]:
    if layout.data.get("revision") in CAMPUS_LAYOUT_REVISIONS:
        locations = tuple(f"{bay}_{name}" for bay in CAMPUS_BAY_OFFSETS_BY_REVISION[layout.data["revision"]]
                          for name in _WORLD_SLOT_NAMES)
    else:
        locations = _WORLD_SLOT_NAMES
    return {name: replay_slot_placement(layout, name, 0, placement_fn)["capacity"] for name in locations}


def production_line_summary(snapshot: Any) -> dict[str, int] | None:
    """Summarize only cell states represented by the recorded process-line fields."""
    if not isinstance(snapshot, dict):
        return None
    production = snapshot.get("production_line")
    cells = production.get("cells") if isinstance(production, dict) else None
    if not isinstance(cells, dict) or not cells:
        return None
    processing = down = blocked = 0
    for cell in cells.values():
        if not isinstance(cell, dict):
            return None
        available = cell.get("available")
        active = cell.get("active_lot") is not None
        outputs = cell.get("output_lots")
        spec = cell.get("spec")
        capacity = spec.get("output_capacity") if isinstance(spec, dict) else None
        if type(available) is not bool or not isinstance(outputs, list) or type(capacity) is not int or capacity < 1:
            return None
        is_blocked = active and cell.get("remaining_s") == 0 and len(outputs) >= capacity
        down += not available
        blocked += available and is_blocked
        processing += available and active and not is_blocked
    return {"cell_count": len(cells), "processing_count": processing,
            "down_count": down, "blocked_count": blocked}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--factory", type=Path, required=True)
    parser.add_argument("--asset-cache", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--start", type=float)
    parser.add_argument("--duration", type=float, default=30.0, help="presentation duration, seconds")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera", default="portfolio_overview")
    parser.add_argument("--subframes", type=int, default=2)
    parser.add_argument("--maximum-wall-time", type=float, default=1800.0)
    args = parser.parse_args()
    if (args.output.exists() or not args.output.is_absolute()
            or not 0 < args.duration <= 180 or not 0 < args.speed <= 120
            or not 1 <= args.fps <= 60 or not 320 <= args.width <= 1920
            or not 180 <= args.height <= 1080 or not 1 <= args.subframes <= 16
            or not 30 <= args.maximum_wall_time <= 7200):
        parser.error("new absolute output and bounded capture parameters required")
    recording, source = load_recording(args.recording)
    start = recording.times[0] if args.start is None else args.start
    frame_count = max(1, math.ceil(args.duration * args.fps))
    end = start + (frame_count - 1) * args.speed / args.fps
    if start < recording.times[0] or end > recording.times[-1]:
        parser.error("requested capture must fit the recorded observations")
    factory = args.factory.resolve(strict=True)
    cache_root = args.asset_cache.resolve(strict=True)
    environment = json.loads((factory / "visual-assets.json").read_text(encoding="utf-8")).get("environment", {})
    cache_index = cache_root / environment.get("revision", "") / "prepared-index.json"
    if environment.get("fixtures") and not cache_index.is_file():
        parser.error("prepared asset index is absent; provide the existing --asset-cache before GPU launch")
    os.environ["FACTORY_OPS_ASSET_CACHE"] = str(cache_root)
    render_profile = json.loads((factory / "render-profile.json").read_text(encoding="utf-8"))
    camera_config = dict(next(row for row in render_profile["cameras"] if row["id"] == args.camera))
    layout_revision = json.loads((factory / "layout.json").read_text(encoding="utf-8")).get("revision")
    if layout_revision in CAMPUS_LAYOUT_REVISIONS and args.camera == "portfolio_overview":
        camera_config["focal_length_mm"] = 18
        camera_config["override_reason"] = "Retain campus margins with the selected revision's authored camera pose"
    if "follow" in args.camera:
        parser.error("this replay renderer currently accepts world cameras only")
    # Scene/layout bytes must match the recording's operations geometry.
    source_pins = source["verified_source_pins"]
    expected_layout = source_pins.get("config", {}).get("layout.json")
    if expected_layout is None or sha(factory / "layout.json") != expected_layout:
        raise ValueError("recorded operations and rendered layout hashes must match")
    args.output.mkdir(parents=True)
    rgb_folder = args.output / "rgb"
    rgb_folder.mkdir()
    render_run = args.run_id or uuid4().hex
    report = {"schema_version": 1, "render_run_id": render_run,
              "source_run_id": recording.run_id, "execution_mode": "operations_replay_render",
              "status": "starting", "scope": "recorded kinematic operations presentation; frozen physics; no sensor or physical motion evidence",
              "source": source, "renderer_source_sha256": sha(Path(__file__)),
              "render_state_source_sha256": sha(ROOT / "factory_ops/operations_render.py"),
              "factory_input_sha256": {name: sha(factory / name) for name in
                                       ("layout.json", "visual-assets.json", "render-profile.json")},
              "camera": camera_config, "image_size_px": [args.width, args.height],
              "fps": args.fps, "playback_speed": args.speed,
              "first_source_time_s": start, "last_source_time_s": end,
              "expected_frames": frame_count, "captured_frames": 0,
              "physical_loading_validated": False, "requires_host_closure": True,
              "cleanup_errors": []}
    report["asset_cache"] = {"root": str(cache_root), "index_sha256": sha(cache_index)}
    started = time.monotonic()
    app = product = rgb = None

    def save():
        (args.output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                                          allow_nan=False) + "\n", encoding="utf-8")

    def bound():
        if time.monotonic() - started > args.maximum_wall_time:
            raise TimeoutError("bounded operations replay render deadline exceeded")

    save()
    code = 1
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "width": args.width, "height": args.height,
                             "multi_gpu": False, "renderer": "RayTracedLighting"})
        import numpy as np
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.storage.native import get_assets_root_path
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from factory_ops.layout import Layout
        from factory_ops.usda import export_usda
        from factory_ops.carrier_module import build_manifest, load_profile
        from scripts.isaac.factory_visuals import FactoryVisualRuntime, world_slot_placement, TOTE_SIZE_M
        from factory_ops.operations_material_projection import project_operations_materials
        from scripts.isaac.local_asset_probe import ASSET_PATH

        layout = Layout.from_file(factory / "layout.json")
        scene = args.output / "factory.usda"
        export_usda(layout, scene)
        context = omni.usd.get_context()
        context.open_stage(str(scene))
        while context.get_stage_loading_status()[2] > 0:
            bound()
            app.update()
        stage = context.get_stage()
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        frozen_time = timeline.get_current_time()
        visuals = FactoryVisualRuntime(factory / "layout.json", scene, args.output / "visuals", 0,
                                       render_run, "operations-replay", config_directory=factory)
        report["visuals"] = visuals.setup(stage, UsdGeom, UsdPhysics, Gf)
        asset = get_assets_root_path().rstrip("/") + ASSET_PATH
        report["robot_asset_url"] = asset
        carrier_file = ROOT / "configs/vehicles/tote-carrier-carter-v2.json"
        carrier = build_manifest(load_profile(carrier_file))
        report["carrier_profile_sha256"] = sha(carrier_file)
        report["carrier_scope"] = "existing authored carrier geometry reused as visual children; no mass, collision or rating claim"
        transforms, loads = {}, {}
        world_cargo = {}
        previous_materials = ()
        slot_capacities = replay_slot_capacities(layout, world_slot_placement)
        campus_replay = layout.data.get("revision") in CAMPUS_LAYOUT_REVISIONS
        report["operational_signals"] = (
            "omitted for four-line campus replay; the legacy visual signal widget represents one aggregate cell"
            if campus_replay else "legacy single-line aggregate"
        )
        report["material_projection_source_sha256"] = sha(ROOT / "factory_ops/operations_material_projection.py")

        def box(path, center, size, color):
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*center))
            cube.AddScaleOp().Set(Gf.Vec3d(*size))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            return cube

        for row in recording.snapshots[0]["robots"]:
            robot_id = row["id"]
            path = "/World/Replay_" + robot_id.replace("-", "_")
            add_reference_to_stage(usd_path=asset, prim_path=path)
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            xf.ClearXformOpOrder()
            transforms[robot_id] = (
                xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="Replay"),
                xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="Replay"),
            )
        while context.get_stage_loading_status()[2] > 0:
            bound()
            app.update()
        for robot_id in transforms:
            path = "/World/Replay_" + robot_id.replace("-", "_") + "/chassis_link"
            if not stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError("Nova Carter chassis reference unavailable")
            for item in carrier["geometry"]:
                if item["body"] == "carrier":
                    box(path + "/ReplayCarrier/" + item["name"], item["center_m"], item["size_m"], (.25, .31, .35))
            cargo = UsdGeom.Xform.Define(stage, path + "/ReplayTote")
            cargo.AddTranslateOp().Set(Gf.Vec3d(*carrier["tote_rest_origin_m"]))
            fill = None
            for item in carrier["geometry"]:
                if item["body"] == "tote":
                    cube = box(str(cargo.GetPath()) + "/" + item["name"], item["center_m"], item["size_m"],
                               (.2, .38, .5) if item["name"] != "rigid_ballast" else (.82, .84, .88))
                    if item["name"] == "rigid_ballast":
                        fill = UsdGeom.Imageable(cube.GetPrim())
            loads[robot_id] = (UsdGeom.Imageable(cargo.GetPrim()), fill)
        prior_cameras = {str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)}
        camera = rep.create.camera(position=camera_config["eye_xyz_m"], look_at=camera_config["target_xyz_m"],
                                   focal_length=camera_config["focal_length_mm"])
        new_cameras = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Camera)
                       and str(prim.GetPath()) not in prior_cameras]
        if len(new_cameras) != 1:
            raise RuntimeError("replay camera identity must be unique")
        camera_prim = new_cameras[0]
        camera_schema = UsdGeom.Camera(camera_prim)
        product = rep.create.render_product(camera, (args.width, args.height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])
        rep.orchestrator.set_capture_on_play(False)
        save()
        with (args.output / "frames.jsonl").open("x", encoding="utf-8") as journal:
            for index in range(frame_count):
                bound()
                source_time = start + index * args.speed / args.fps
                snapshot, metadata = recording.sample(source_time)
                for row in snapshot["robots"]:
                    pose = row["pose"]
                    xyz, orientation = transforms[row["id"]]
                    xyz.Set(Gf.Vec3d(pose["x_m"], pose["y_m"], .15))
                    orientation.Set(Gf.Quatd(math.cos(pose["yaw_rad"] / 2), 0., 0., math.sin(pose["yaw_rad"] / 2)))
                    cargo, fill = loads[row["id"]]
                    cargo.GetVisibilityAttr().Set("inherited" if row["payload_id"] else "invisible")
                    if fill:
                        fill.GetVisibilityAttr().Set("invisible" if row["task_type"] == "empty_return" else "inherited")
                material = project_operations_materials(snapshot, slot_capacities, previous_materials)
                previous_materials = material["world_items"]
                visible = set()
                for item in material["world_items"]:
                    key = (item["kind"], item["id"], item["location"], item["slot"])
                    if key not in world_cargo:
                        placement = replay_slot_placement(layout, item["location"], item["slot"], world_slot_placement)
                        group_path = "/World/ReplayMaterials/Item_" + str(len(world_cargo))
                        group = UsdGeom.Xform.Define(stage, group_path)
                        center = list(placement["translate_xyz_m"])
                        center[2] -= TOTE_SIZE_M[2] / 2
                        group.AddTranslateOp().Set(Gf.Vec3d(*center))
                        group.GetPrim().SetCustomDataByKey("operationsItemId", item["id"])
                        fill = None
                        for part in carrier["geometry"]:
                            if part["body"] == "tote":
                                cube = box(group_path + "/" + part["name"], part["center_m"], part["size_m"],
                                           (.2, .38, .5) if part["name"] != "rigid_ballast" else (.82, .84, .88))
                                if part["name"] == "rigid_ballast":
                                    fill = UsdGeom.Imageable(cube.GetPrim())
                        world_cargo[key] = (UsdGeom.Imageable(group.GetPrim()), fill)
                    visible.add(key)
                    fill = world_cargo[key][1]
                    if fill:
                        fill.GetVisibilityAttr().Set("inherited" if item["filled"] else "invisible")
                for key, (group, _) in world_cargo.items():
                    group.GetVisibilityAttr().Set("inherited" if key in visible else "invisible")
                equipment, kpis = snapshot["equipment"], dict(snapshot["kpis"])
                line_summary = production_line_summary(snapshot)
                kpis["empty_returns_completed"] = sum(task["type"] == "empty_return" and task["state"] == "complete"
                                                      for task in snapshot["tasks"])
                if not campus_replay:
                    visuals._update_signals({"ci_slots_used": equipment["input_waiting_orders"],
                        "co_slots_used": equipment["output_waiting_orders"],
                        "cell_processing": equipment["available"] and equipment["active_order_id"] is not None,
                        "orders_completed": kpis["completed_orders"]})
                rep.orchestrator.step(delta_time=0., rt_subframes=args.subframes, pause_timeline=True)
                rep.orchestrator.wait_until_complete()
                if timeline.is_playing() or abs(timeline.get_current_time() - frozen_time) > 1e-8:
                    raise RuntimeError("presentation render advanced the physics timeline")
                data = np.asarray(rgb.get_data())
                if data.shape[:2] != (args.height, args.width) or data.ndim != 3 or data.shape[2] < 3 or data[:, :, :3].std() < 1:
                    raise RuntimeError("RGB replay buffer unavailable or blank")
                filename = f"rgb/{index:05d}.png"
                Image.fromarray(data[:, :, :3].astype(np.uint8)).save(args.output / filename)
                camera_world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(camera_prim)
                camera_inverse = camera_world.GetInverse()
                focal = float(camera_schema.GetFocalLengthAttr().Get())
                horizontal = float(camera_schema.GetHorizontalApertureAttr().Get())
                raw_vertical = float(camera_schema.GetVerticalApertureAttr().Get())
                vertical = horizontal * args.height / args.width
                robot_rows = []
                for row in snapshot["robots"]:
                    selected = {key: row.get(key) for key in ("id", "pose", "speed_mps", "battery_pct", "task_type",
                                                             "status", "payload_id", "destination", "waiting_reason")}
                    local = camera_inverse.Transform(Gf.Vec3d(row["pose"]["x_m"], row["pose"]["y_m"], 1.15))
                    screen = ([args.width * (.5 + focal * local[0] / (-local[2] * horizontal)),
                               args.height * (.5 - focal * local[1] / (-local[2] * vertical))]
                              if local[2] < 0 else None)
                    selected["screen_xy_px"] = screen if screen and 0 <= screen[0] < args.width and 0 <= screen[1] < args.height else None
                    robot_rows.append(selected)
                if index == 0:
                    report["camera_readback"] = {"prim_path": str(camera_prim.GetPath()),
                        "world_transform_row_major": [float(camera_world[i][j]) for i in range(4) for j in range(4)],
                        "focal_length_mm": focal, "horizontal_aperture_mm": horizontal,
                        "vertical_aperture_mm": raw_vertical, "effective_vertical_aperture_mm": vertical}
                    report["screen_projection"] = "horizontal_fit_square_pixels"
                journal.write(json.dumps({"frame": index, "rgb": filename, "sha256": sha(args.output / filename),
                    **metadata, "kpis": kpis, "equipment": equipment,
                    **({"production_line_summary": line_summary} if line_summary is not None else {}),
                    "material_counts": material["counts"],
                    "robots": robot_rows},
                    ensure_ascii=False, allow_nan=False) + "\n")
                report["captured_frames"] = index + 1
                if index % args.fps == 0:
                    journal.flush()
                    save()
        report.update(status="passed", frames_sha256=sha(args.output / "frames.jsonl"),
                      elapsed_wall_s=time.monotonic() - started, frozen_timeline_s=frozen_time)
        code = 0
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
    finally:
        for label, action in (("annotator", lambda: rgb.detach() if rgb else None),
                              ("product", lambda: product.destroy() if product else None)):
            try:
                action()
            except BaseException as error:
                report["cleanup_errors"].append(f"{label}: {error}")
        save()
        if app is not None:
            app.close(wait_for_replicator=False)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
