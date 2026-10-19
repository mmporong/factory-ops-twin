#!/usr/bin/env python3
"""Bounded SDK4.5 idle or authored wheel-motion component; not fleet qualification."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback


ASSET_PATH = "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd"
WHEEL_NAMES = ["joint_wheel_left", "joint_wheel_right"]


def project_path() -> Path:
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return root


def validate_motion_profile(profile: dict, hz: int) -> float:
    from factory_ops.drive_profile import DriveLimits

    limits = DriveLimits(**profile["drive_limits"])
    if profile["command_timeout_s"] != 0.3:
        raise ValueError("motion admission requires a 0.3s watchdog")
    expected = [("initial_stop", 0., 0.), ("forward", 0.3, 0.), ("normal_stop", 0., 0.),
                ("turn", 0., 0.4), ("turn_stop", 0., 0.), ("curve", 0.25, 0.3),
                ("watchdog_stop", 0., 0.)]
    phases = profile["phases"]
    if len(phases) != len(expected):
        raise ValueError("motion phase count mismatch")
    total = 0.
    for item, (name, linear, angular) in zip(phases, expected):
        duration = item["duration_s"]
        if (item["name"] != name or item["linear_m_s"] != linear or item["angular_rad_s"] != angular
                or item["refresh_commands"] is not (name != "watchdog_stop")
                or isinstance(duration, bool) or not isinstance(duration, (int, float))
                or not math.isfinite(duration) or duration < 1.
                or not math.isclose(duration * hz, round(duration * hz), abs_tol=1e-9)
                or abs(linear) > limits.max_linear_m_s or abs(angular) > limits.max_angular_rad_s):
            raise ValueError("invalid motion phase: " + name)
        total += duration
    if total > 18.:
        raise ValueError("motion schedule exceeds 18 seconds")
    evaluation = profile["evaluation"]
    for key in ("minimum_forward_m", "minimum_turn_rad", "maximum_turn_translation_m",
                "minimum_curve_distance_m", "minimum_curve_yaw_rad", "tail_window_s",
                "linear_speed_tolerance_m_s", "angular_speed_tolerance_rad_s", "maximum_tilt_rad"):
        value = evaluation[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("invalid motion evaluation setting: " + key)
    xy, z = vector(evaluation["xy_bounds_m"], 4), vector(evaluation["z_bounds_m"], 2)
    if xy[0] >= xy[1] or xy[2] >= xy[3] or z[0] >= z[1] or evaluation["tail_window_s"] > 1.:
        raise ValueError("invalid motion evaluation bounds")
    return total


def planned_phase(profile: dict, elapsed_s: float) -> tuple[dict, float]:
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("motion clock must be finite and nonnegative")
    end = 0.
    for item in profile["phases"]:
        end += item["duration_s"]
        if elapsed_s + 1e-9 < end:
            return item, end
    raise ValueError("motion clock is outside the bounded schedule")


class MotionPlan:
    """Time-only command source; no simulator truth or encoder pose input."""

    def __init__(self, profile: dict, geometry, start_s: float) -> None:
        from factory_ops.drive_profile import DriveLimits, DriveProfile
        from factory_ops.motion import VelocityWatchdog

        self.profile, self.start_s = profile, start_s
        limits = DriveLimits(**profile["drive_limits"])
        self.drive = DriveProfile(geometry, limits)
        self.watchdog = VelocityWatchdog(max_age_s=profile["command_timeout_s"],
            max_linear_m_s=limits.max_linear_m_s, max_angular_rad_s=limits.max_angular_rad_s)
        self.last_received_s = None

    def step(self, now_s: float, dt_s: float) -> dict:
        item, end = planned_phase(self.profile, now_s - self.start_s)
        if item["refresh_commands"]:
            self.watchdog.receive(now_s, item["linear_m_s"], item["angular_rad_s"])
            self.last_received_s = now_s
        requested = self.watchdog.command(now_s)
        expired = not self.watchdog.active
        target = self.drive.step(*requested, dt_s, emergency_stop=expired)
        return {"phase": item["name"], "phase_end_s": end, "issued_at_sim_time_s": now_s,
                "command_stamp_s": self.last_received_s,
                "command_refreshed": item["refresh_commands"], "watchdog_expired": expired,
                "requested_body_velocity": list(requested), "target": asdict(target)}


def drive_geometry_from_records(records: list[dict], robot_path: str) -> dict:
    """Apply nav_scene's strict three-collider, root-Y wheel measurement rules."""
    wheels = {}
    for side in ("left", "right"):
        selected = [r for r in records if r["path"].startswith(robot_path + "/wheel_" + side + "/")]
        if len(selected) != 3:
            raise ValueError("expected three active wheel collision geometries: " + side)
        points = [(vector(r["min"], 3), vector(r["max"], 3)) for r in selected]
        if any(hi[i] <= lo[i] for lo, hi in points for i in range(3)):
            raise ValueError("invalid wheel collision geometry bounds")
        lo = [min(p[0][i] for p in points) for i in range(3)]
        hi = [max(p[1][i] for p in points) for i in range(3)]
        size = [hi[i] - lo[i] for i in range(3)]
        if abs(size[0] - size[2]) > 1e-5 or size[1] >= min(size[0], size[2]):
            raise ValueError("wheel collider union is not root-Y radial: " + side)
        wheels[side] = {"min": lo, "max": hi, "size": size,
                        "center": [(hi[i] + lo[i]) / 2. for i in range(3)]}
    left, right = wheels["left"], wheels["right"]
    if (any(abs(a - b) > 1e-5 for a, b in zip(left["size"], right["size"]))
            or abs(left["center"][0] - right["center"][0]) > 1e-5
            or abs(left["center"][2] - right["center"][2]) > 1e-5
            or abs(left["center"][1] + right["center"][1]) > 1e-5
            or left["center"][1] <= right["center"][1]):
        raise ValueError("wheel collision geometry is not left-positive-Y symmetric")
    return {"wheel_radius_m": (max(left["size"][0], left["size"][2]) + max(right["size"][0], right["size"][2])) / 4.,
            "wheel_separation_m": left["center"][1] - right["center"][1],
            "provenance": {"kind": "fresh_asset_collision_geometry_measurement",
                "method": "ComputeExtentFromPlugins; root-relative XformCache; union three active collision geometries per wheel; X/Z diameter and Y centers",
                "unit": "m", "wheel_unions": wheels, "collision_geometry_records": records}}


def measure_drive_geometry(stage, robot_path: str, Usd, UsdGeom, UsdPhysics, Gf, deadline: float) -> dict:
    root = stage.GetPrimAtPath(robot_path)
    if UsdGeom.GetStageMetersPerUnit(stage) != 1. or str(UsdGeom.GetStageUpAxis(stage)) != "Z":
        raise ValueError("motion geometry requires meter units and Z-up")
    cache, sources = UsdGeom.XformCache(Usd.TimeCode.Default()), {}
    for collider in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        check_deadline(deadline, "wheel_geometry")
        if (not any(str(collider.GetPath()).startswith(robot_path + "/wheel_" + side + "/")
                    for side in ("left", "right")) or not collider.HasAPI(UsdPhysics.CollisionAPI)
                or collider.GetAttribute("physics:collisionEnabled").Get() is False):
            continue
        for child in Usd.PrimRange(collider, Usd.TraverseInstanceProxies()):
            if not child.IsA(UsdGeom.Boundable):
                continue
            cursor, disabled = child, False
            while cursor.IsValid() and cursor != collider:
                if cursor.HasAPI(UsdPhysics.CollisionAPI) and cursor.GetAttribute("physics:collisionEnabled").Get() is False:
                    disabled = True
                    break
                cursor = cursor.GetParent()
            if not disabled:
                sources[str(child.GetPath())] = (child, collider)
    records = []
    for path, (child, collider) in sources.items():
        check_deadline(deadline, "wheel_geometry")
        extent = UsdGeom.Boundable.ComputeExtentFromPlugins(UsdGeom.Boundable(child), Usd.TimeCode.Default())
        if extent is None or len(extent) != 2:
            raise ValueError("wheel collision extent unavailable: " + path)
        lo, hi = vector(extent[0], 3), vector(extent[1], 3)
        transform, resets = cache.ComputeRelativeTransform(child, root)
        if resets:
            raise ValueError("wheel collision resets root transform: " + path)
        bound = Gf.BBox3d(Gf.Range3d(Gf.Vec3d(*lo), Gf.Vec3d(*hi)), transform).ComputeAlignedRange()
        if bound.IsEmpty():
            raise ValueError("empty wheel collision extent: " + path)
        records.append({"path": path, "collision_api_path": str(collider.GetPath()),
            "min": list(bound.GetMin()), "max": list(bound.GetMax()), "local_extent": [lo, hi],
            "root_relative_transform": [list(row) for row in transform]})
    return drive_geometry_from_records(records, robot_path)


def pose_angles(orientation: list[float]) -> tuple[float, float, float]:
    w, x, y, z = vector(orientation, 4)
    if abs(math.sqrt(w*w + x*x + y*y + z*z) - 1.) > 1e-3:
        raise ValueError("non-unit measured quaternion")
    return (math.atan2(2.*(w*x + y*z), 1.-2.*(x*x + y*y)),
            math.asin(max(-1., min(1., 2.*(w*y-z*x)))),
            math.atan2(2.*(w*z + x*y), 1.-2.*(y*y + z*z)))


def evaluate_motion(samples: list[dict], initial: dict, cfg: dict) -> dict:
    """Simulator truth is evaluation only; target ramps cannot prove motion."""
    profile, e = cfg["motion_profile"], cfg["motion_profile"]["evaluation"]
    xy, z = e["xy_bounds_m"], e["z_bounds_m"]
    planar = all(xy[0] <= s["position_m"][0] <= xy[1] and xy[2] <= s["position_m"][1] <= xy[3]
                 and z[0] <= s["position_m"][2] <= z[1]
                 and max(abs(s["roll_rad"]), abs(s["pitch_rad"])) <= e["maximum_tilt_rad"] for s in samples)
    summaries, before = [], initial
    for item in profile["phases"]:
        rows = [s for s in samples if s["motion"]["phase"] == item["name"]]
        if not rows:
            raise ValueError("missing motion phase observations: " + item["name"])
        last = rows[-1]
        tail = [s for s in rows if s["sim_time_s"] >= last["sim_time_s"] - e["tail_window_s"] - 1e-9]
        dx, dy = (last["position_m"][i] - before["position_m"][i] for i in (0, 1))
        yaw_delta = sum(math.atan2(math.sin(b["yaw_rad"]-a["yaw_rad"]), math.cos(b["yaw_rad"]-a["yaw_rad"]))
                        for a, b in zip([before] + rows, rows))
        stationary_since = None
        for row in rows:
            stopped = row["linear_speed_m_s"] <= cfg["stationary_linear_m_s"] and row["angular_speed_rad_s"] <= cfg["stationary_angular_rad_s"]
            stationary_since = (row["sim_time_s"] if stationary_since is None else stationary_since) if stopped else None
        dwell = 0. if stationary_since is None else last["sim_time_s"] - stationary_since
        linear = sum(s["forward_velocity_m_s"] for s in tail) / len(tail)
        angular = sum(s["angular_velocity_rad_s"][2] for s in tail) / len(tail)
        checks = {"complete_phase": len(rows) == round(item["duration_s"] * cfg["physics_hz"])}
        name = item["name"]
        if name in ("forward", "turn", "curve"):
            checks.update(tail_linear=abs(linear-item["linear_m_s"]) <= e["linear_speed_tolerance_m_s"],
                          tail_angular=abs(angular-item["angular_rad_s"]) <= e["angular_speed_tolerance_rad_s"])
        else:
            checks["stationary_dwell"] = dwell + 1e-9 >= cfg["stationary_dwell_s"]
        if name == "forward":
            checks["forward_progress"] = dx*math.cos(before["yaw_rad"]) + dy*math.sin(before["yaw_rad"]) >= e["minimum_forward_m"]
        elif name == "turn":
            checks.update(rotation=yaw_delta >= e["minimum_turn_rad"], translation_drift=math.hypot(dx,dy) <= e["maximum_turn_translation_m"])
        elif name == "curve":
            checks.update(translation=math.hypot(dx,dy) >= e["minimum_curve_distance_m"], rotation=yaw_delta >= e["minimum_curve_yaw_rad"])
        elif name == "watchdog_stop":
            expired = [s for s in rows if s["motion"]["watchdog_expired"]]
            checks["timeout_observed"] = bool(expired) and not any(s["motion"]["command_refreshed"] for s in rows)
            checks["expired_targets_zero"] = bool(expired) and all(tuple(s["motion"]["target"]["wheel_targets_rad_s"]) == (0., 0.) for s in expired)
            first_age = None if not expired else expired[0]["motion"]["issued_at_sim_time_s"] - expired[0]["motion"]["command_stamp_s"]
            checks["timeout_timing"] = first_age is not None and profile["command_timeout_s"] - 1e-9 <= first_age <= profile["command_timeout_s"] + 1. / cfg["physics_hz"] + 1e-9
        summaries.append({"phase": name, "samples": len(rows), "displacement_xy_m": [dx, dy],
            "yaw_change_rad": yaw_delta, "stationary_dwell_s": dwell, "tail_forward_m_s": linear,
            "tail_angular_rad_s": angular, "encoder_delta": {key: last["encoder"][key] - before["encoder"][key] for key in ("x_m", "y_m", "yaw_rad")},
            "tail_measured_wheel_rad_s": [sum(s["measured_wheel_velocity_rad_s"][i] for s in tail) / len(tail) for i in (0, 1)],
            "tail_target_wheel_rad_s": [sum(s["motion"]["target"]["wheel_targets_rad_s"][i] for s in tail) / len(tail) for i in (0, 1)],
            "peak_actual_forward_accel_m_s2": max(abs(b["forward_velocity_m_s"]-a["forward_velocity_m_s"])/(b["sim_time_s"]-a["sim_time_s"]) for a,b in zip(rows,rows[1:])),
            "peak_target_linear_accel_m_s2": max(abs(b["motion"]["target"]["linear_m_s"]-a["motion"]["target"]["linear_m_s"])/(b["sim_time_s"]-a["sim_time_s"]) for a,b in zip(rows,rows[1:])),
            "checks": checks, "passed": all(checks.values())})
        before = last
    return {"passed": planar and all(s["passed"] for s in summaries), "planar_bounds_passed": planar,
            "phases": summaries, "truth_usage": "physics response evaluation only; never command source",
            "threshold_source": "authored component admission criteria, not measured vehicle specifications"}


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def vector(value, length: int) -> list[float]:
    """Reject missing/nonfinite physics data instead of manufacturing zero state."""
    if value is None or len(value) != length:
        raise ValueError("missing or incorrectly sized measured vector")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError("nonfinite measured vector")
    return result


def validate_config(cfg: dict) -> None:
    fixed = {"schema_version": 1, "robot_count": 1, "asset_relative_path": ASSET_PATH,
             "wheel_dof_names": WHEEL_NAMES, "physics_hz": 60,
             "image_size_px": [640, 480]}
    for key, expected in fixed.items():
        if cfg.get(key) != expected or isinstance(cfg.get(key), bool):
            raise ValueError("unsupported probe setting: " + key)
    for key, lower, upper in (("maximum_wall_time_s", 30., 400.),
                              ("stage_load_timeout_s", 1., 180.),
                              ("stationary_linear_m_s", 0.0001, 0.05),
                              ("stationary_angular_rad_s", 0.0001, 0.05),
                              ("stationary_dwell_s", 0.1, 1.)):
        number = cfg[key]
        if (isinstance(number, bool) or not isinstance(number, (float, int))
                or not math.isfinite(number) or not lower <= number <= upper):
            raise ValueError("invalid bounded setting: " + key)
    if cfg["stage_load_timeout_s"] >= cfg["maximum_wall_time_s"]:
        raise ValueError("stage load deadline must precede overall deadline")
    for key in ("initial_position_m", "camera_position_m", "camera_target_m"):
        vector(cfg[key], 3)
    if cfg["initial_position_m"] != [0., 0., 0.05]:
        raise ValueError("only the single-robot idle admission pose is supported")
    if not isinstance(cfg.get("scope"), str) or not cfg["scope"]:
        raise ValueError("scope is required")
    duration = validate_motion_profile(cfg["motion_profile"], cfg["physics_hz"]) if "motion_profile" in cfg else 3.
    if cfg["maximum_sim_time_s"] != duration:
        raise ValueError("maximum simulation time must match idle or bounded motion schedule")


def check_deadline(deadline: float, phase: str) -> None:
    if time.monotonic() >= deadline:
        raise TimeoutError("wall deadline exceeded during " + phase)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if not args.output.is_absolute() or args.output.exists():
        parser.error("output must be a new absolute directory")
    if len(args.run_id) != 32 or any(c not in "0123456789abcdef" for c in args.run_id):
        parser.error("run-id must be 32 lowercase hexadecimal characters")
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = {"schema_version": 1, "run_id": args.run_id, "status": "starting",
        "stage": "preflight", "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "python": sys.version, "media": [], "phases": [], "commands_sent": 0,
        "four_amr_verified": False, "amr_docking_verified": False,
        "ros_verified": False, "navigation_verified": False, "lidar_verified": False,
        "transport_verified": False, "charging_power_verified": False,
        "sdk62_qualification": "unchanged; this is SDK4.5 admission only",
        "requires_host_closure": True, "sdk_close_attempted": False, "sdk_close_returned": False}

    def phase(name: str) -> None:
        report["stage"] = name
        report["phases"].append({"phase": name, "elapsed_wall_s": time.monotonic() - started,
                                 "at_utc": datetime.now(timezone.utc).isoformat()})
        save(args.output / "result.json", report)

    app = world = None
    code = 1
    phase("preflight")
    try:
        raw_config = args.config.read_bytes()
        report["config_sha256"] = hashlib.sha256(raw_config).hexdigest()
        cfg = json.loads(raw_config.decode("utf-8-sig"))
        motion_profile = cfg.get("motion_profile")
        if "motion_profile" in cfg:
            project = project_path()
            report["project_source_sha256"] = {name: hashlib.sha256((project / name).read_bytes()).hexdigest()
                for name in ("factory_ops/__init__.py", "factory_ops/motion.py", "factory_ops/drive_profile.py")}
        validate_config(cfg)
        report.update(config=cfg, scope=cfg["scope"], robot_count=cfg["robot_count"])
        version_path = Path(os.environ["ISAAC_PATH"]) / "VERSION"
        report["isaac_version"] = version_path.read_text(encoding="utf-8").strip()
        if not report["isaac_version"].startswith("4.5.0"):
            raise ValueError("this probe is restricted to installed SDK4.5.0")
        deadline = started + cfg["maximum_wall_time_s"]
        phase("native_sdk_startup")
        from isaacsim import SimulationApp

        width, height = cfg["image_size_px"]
        app = SimulationApp({"headless": True, "width": width, "height": height,
                             "multi_gpu": False, "renderer": "RayTracedLighting"})
        check_deadline(deadline, "native_sdk_startup")
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import is_stage_loading
        from isaacsim.storage.native import get_assets_root_path
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics

        phase("asset_root_resolving")
        root = get_assets_root_path()
        check_deadline(deadline, "asset_root_resolving")
        if not isinstance(root, str) or not root:
            raise RuntimeError("installed get_assets_root_path returned no asset root")
        asset_url = root.rstrip("/") + ASSET_PATH
        report["asset"] = {"relative_path": ASSET_PATH, "resolved_endpoint": asset_url,
                           "resolver": "isaacsim.storage.native.get_assets_root_path",
                           "alternate_endpoints_attempted": 0}
        phase("world_setup")
        dt = 1. / cfg["physics_hz"]
        world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=dt,
                      backend="numpy", device="cpu")
        physics = world.get_physics_context()
        physics.enable_gpu_dynamics(False)
        physics.set_broadphase_type("MBP")
        physics.set_gravity(-9.81)
        # Procedural ground avoids a second environment asset endpoint.
        world.scene.add_ground_plane(size=10., restitution=0., static_friction=0.8,
                                     dynamic_friction=0.6, color=np.array([0.65, 0.68, 0.72]))
        phase("asset_reference_loading")
        load_deadline = min(deadline, time.monotonic() + cfg["stage_load_timeout_s"])
        robot_path = "/World/NovaCarter"
        robot = world.scene.add(WheeledRobot(prim_path=robot_path, name="idle_nova_carter",
            wheel_dof_names=WHEEL_NAMES, create_robot=True, usd_path=asset_url,
            position=np.array(cfg["initial_position_m"])))
        quiet_updates = 0
        for _ in range(20000):
            check_deadline(load_deadline, "asset_reference_loading")
            if not app.is_running():
                raise RuntimeError("SDK stopped during asset loading")
            app.update()
            check_deadline(load_deadline, "asset_reference_loading")
            quiet_updates = 0 if is_stage_loading() else quiet_updates + 1
            if quiet_updates >= 3:
                break
        else:
            raise TimeoutError("stage loading update limit exceeded")
        phase("asset_inspection")
        stage = omni.usd.get_context().get_stage()
        composition_errors = []
        composition_prims_checked = 0
        # SDK4.5 exposes composition errors through Pcp.PrimIndex, not Usd.Stage.
        for composed_prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            check_deadline(deadline, "asset_inspection")
            prim_index = composed_prim.GetPrimIndex()
            composition_errors.extend(str(composed_prim.GetPath()) + ": " + str(error)
                                      for error in prim_index.localErrors)
            composition_prims_checked += 1
        if not composition_prims_checked:
            raise RuntimeError("no composed prim indexes inspected")
        report["asset"]["composition_error_source"] = "Usd.Prim.GetPrimIndex().localErrors; instance proxies included"
        report["asset"]["composition_prims_checked"] = composition_prims_checked
        report["asset"]["composition_errors"] = composition_errors
        report["asset"]["used_layers"] = [layer.identifier for layer in stage.GetUsedLayers()]
        if composition_errors:
            raise RuntimeError("USD composition failed; no alternate asset attempted")
        prim = stage.GetPrimAtPath(robot_path)
        if not prim.IsValid():
            raise RuntimeError("referenced robot prim is unavailable")
        roots, bodies, colliders, joints = [], [], [], []
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy,
             UsdGeom.Tokens.guide], useExtentsHint=False, ignoreVisibility=True)
        collision_range = Gf.Range3d()
        for child in Usd.PrimRange(prim):
            check_deadline(deadline, "asset_inspection")
            path = str(child.GetPath())
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                roots.append(path)
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                body = UsdPhysics.RigidBodyAPI(child)
                bodies.append({"path": path, "enabled": bool(body.GetRigidBodyEnabledAttr().Get()),
                    "kinematic": bool(body.GetKinematicEnabledAttr().Get()),
                    "disable_gravity": bool(PhysxSchema.PhysxRigidBodyAPI(child).GetDisableGravityAttr().Get())})
            if child.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(child)
                joints.append({"path": path, "type": child.GetTypeName(),
                    "body0": [str(p) for p in joint.GetBody0Rel().GetTargets()],
                    "body1": [str(p) for p in joint.GetBody1Rel().GetTargets()]})
            if child.HasAPI(UsdPhysics.CollisionAPI):
                enabled = bool(UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Get())
                box = bounds.ComputeWorldBound(child).ComputeAlignedRange()
                if enabled and (box.IsEmpty() or not all(math.isfinite(v) for v in (*box.GetMin(), *box.GetMax()))):
                    raise RuntimeError("invalid enabled collision bound: " + path)
                colliders.append({"path": path, "type": child.GetTypeName(), "enabled": enabled,
                    "aabb_world_stage_units": None if box.IsEmpty() else
                        {"min": list(box.GetMin()), "max": list(box.GetMax())}})
                if enabled:
                    collision_range.UnionWith(box)
        units = float(UsdGeom.GetStageMetersPerUnit(stage))
        report["geometry"] = {"articulation_paths": roots, "rigid_bodies": bodies,
            "collisions": colliders, "joints": joints, "meters_per_unit": units,
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "collision_aabb_source": "composed USD enabled collision geometry world bounds before stepping; not contact or cooked hull measurement"}
        if (not roots or not joints or collision_range.IsEmpty() or units != 1.
                or not any(b["enabled"] and not b["kinematic"] and not b["disable_gravity"] for b in bodies)):
            raise RuntimeError("asset lacks valid dynamic articulation/collision geometry in meter units")
        report["geometry"]["collision_aabb_m"] = {
            "min": list(collision_range.GetMin()), "max": list(collision_range.GetMax()),
            "size": list(collision_range.GetSize())}
        if not all(v > 0 and math.isfinite(v) for v in collision_range.GetSize()):
            raise RuntimeError("degenerate robot collision bounds")
        if motion_profile:
            phase("wheel_geometry")
            report["drive_geometry"] = measure_drive_geometry(stage, robot_path, Usd, UsdGeom, UsdPhysics, Gf, deadline)
        light = UsdLux.DomeLight.Define(stage, "/World/AdmissionLight")
        light.CreateIntensityAttr(1000.)
        camera = rep.create.camera(position=tuple(cfg["camera_position_m"]),
                                   look_at=tuple(cfg["camera_target_m"]))
        product = rep.create.render_product(camera, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([product])

        def capture_image(label: str, stamp: float) -> None:
            if motion_profile:
                for _ in range(8):
                    check_deadline(deadline, "rgb_capture")
                    before_render = float(world.current_time)
                    world.render()
                    if not math.isclose(float(world.current_time), before_render, abs_tol=1e-9):
                        raise RuntimeError("render-only refresh advanced physics time")
            data = np.asarray(rgb.get_data())
            if (data.ndim != 3 or data.shape[:2] != (height, width) or data.shape[2] not in (3, 4)
                    or data.dtype != np.uint8 or not np.any(data[:, :, :3])):
                raise RuntimeError("native RGB source is missing, black or has invalid shape/type")
            check_deadline(deadline, "rgb_capture")
            image_path = args.output / (label + ".png")
            Image.fromarray(data[:, :, :3]).save(image_path)
            report["media"].append({"path": image_path.name, "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                "image_size_px": [width, height], "source": "native Replicator rgb annotator, RayTracedLighting",
                "sim_time_s_at_read": stamp, "media_clock_sync_verified": False})
            save(args.output / "result.json", report)

        phase("physics_initializing")
        check_deadline(deadline, "physics_initializing")
        world.reset()
        check_deadline(deadline, "physics_initializing")
        if not robot.handles_initialized:
            raise RuntimeError("native articulation physics handles not initialized")
        names = list(robot.dof_names)
        dof_count = int(robot.num_dof)
        if len(names) != dof_count or not set(WHEEL_NAMES).issubset(names):
            raise RuntimeError("required wheel DOFs missing from native articulation")
        direction, magnitude = physics.get_gravity()
        report["physics"] = {"source": "native World steps and initialized articulation state",
            "physics_hz": cfg["physics_hz"], "device": "cpu", "gpu_dynamics": bool(physics.is_gpu_dynamics_enabled()),
            "gravity_direction": vector(direction, 3), "gravity_m_s2": float(magnitude),
            "dof_names": names, "num_dof": dof_count, "num_bodies": int(robot.num_bodies),
            "joint_state_units": {"revolute": ["rad", "rad/s"], "prismatic": ["m", "m/s"],
                                  "type_source": "geometry.joints USD joint types matched by name"},
            "commands_sent": 0, "input_policy": "gravity and asset-authored joint drives; no wheel/pose/velocity commands during stepping"}
        if report["physics"]["gpu_dynamics"]:
            raise RuntimeError("CPU PhysX admission unexpectedly enabled GPU dynamics")
        report["status"] = "running"
        phase("motion_physics" if motion_profile else "uncommanded_physics")
        initial_time = float(world.current_time)
        last_time = initial_time
        stationary_since = None
        sample_count = 0
        motion_samples = []
        if motion_profile:
            from factory_ops.motion import DriveGeometry, WheelOdometry
            from isaacsim.core.utils.types import ArticulationAction

            measured = report["drive_geometry"]
            geometry = DriveGeometry(measured["wheel_radius_m"], measured["wheel_separation_m"])
            plan = MotionPlan(motion_profile, geometry, initial_time)
            encoder = WheelOdometry(geometry)
            wheel_indices = [names.index(name) for name in WHEEL_NAMES]
            angles = vector(robot.get_joint_positions(), dof_count)
            initial_encoder = encoder.update(initial_time, *(angles[i] for i in wheel_indices))
            position, orientation = robot.get_world_pose()
            initial = {"position_m": vector(position, 3), "yaw_rad": pose_angles(vector(orientation, 4))[2],
                       "encoder": asdict(initial_encoder)}
            report["physics"]["input_policy"] = "time-only schedule, watchdog and DriveProfile; wheel velocity targets only; GT used for evaluation only"
            report["render_policy"] = "physics steps render=False; eight render-only refreshes at each of three captures; media clock synchronization unverified"
        with (args.output / "observations.jsonl").open("x", encoding="utf-8") as stream:
            for index in range(round(cfg["physics_hz"] * cfg["maximum_sim_time_s"])):
                check_deadline(deadline, "motion_physics" if motion_profile else "uncommanded_physics")
                if not app.is_running():
                    raise RuntimeError("SDK stopped during physics sampling")
                if motion_profile:
                    command = plan.step(float(world.current_time), dt)
                    robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.array(command["target"]["wheel_targets_rad_s"])))
                    report["commands_sent"] += 1
                    report["physics"]["commands_sent"] += 1
                world.step(render=not bool(motion_profile))
                check_deadline(deadline, "motion_physics" if motion_profile else "uncommanded_physics")
                stamp = float(world.current_time)
                if not math.isfinite(stamp) or not math.isclose(stamp - last_time, dt, rel_tol=0., abs_tol=1e-6):
                    raise RuntimeError("physics time failed to advance by one 60Hz step")
                position, orientation = robot.get_world_pose()
                linear = vector(robot.get_linear_velocity(), 3)
                angular = vector(robot.get_angular_velocity(), 3)
                sample = {"run_id": args.run_id, "physics_frame": index + 1, "sim_time_s": stamp,
                    "position_m": vector(position, 3), "orientation_wxyz": vector(orientation, 4),
                    "linear_velocity_m_s": linear, "angular_velocity_rad_s": angular,
                    "joint_positions_native": vector(robot.get_joint_positions(), dof_count),
                    "joint_velocities_native": vector(robot.get_joint_velocities(), dof_count),
                    "linear_speed_m_s": math.sqrt(sum(v * v for v in linear)),
                    "angular_speed_rad_s": math.sqrt(sum(v * v for v in angular))}
                if motion_profile:
                    roll, pitch, yaw = pose_angles(sample["orientation_wxyz"])
                    sample.update(motion=command, roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
                        forward_velocity_m_s=linear[0]*math.cos(yaw) + linear[1]*math.sin(yaw),
                        encoder=asdict(encoder.update(stamp, *(sample["joint_positions_native"][i] for i in wheel_indices))),
                        measured_wheel_velocity_rad_s=[sample["joint_velocities_native"][i] for i in wheel_indices])
                    motion_samples.append(sample)
                stationary = (sample["linear_speed_m_s"] <= cfg["stationary_linear_m_s"]
                              and sample["angular_speed_rad_s"] <= cfg["stationary_angular_rad_s"])
                stationary_since = (stamp if stationary_since is None else stationary_since) if stationary else None
                stream.write(json.dumps(sample, allow_nan=False) + "\n")
                stream.flush()
                sample_count += 1
                last_time = stamp
                if motion_profile and stamp - initial_time + 1e-9 >= command["phase_end_s"]:
                    label = {"initial_stop": "motion-start", "turn_stop": "motion-after-turn",
                             "watchdog_stop": "motion-final"}.get(command["phase"])
                    if label:
                        capture_image(label, stamp)
        dwell = 0. if stationary_since is None else last_time - stationary_since
        report["physics"].update(samples=sample_count, initial_sim_time_s=initial_time,
            final_sim_time_s=last_time, elapsed_sim_time_s=last_time - initial_time,
            final_state=sample, stationary_dwell_observed_s=dwell,
            stationary_passed=dwell + 1e-9 >= cfg["stationary_dwell_s"])
        if motion_profile:
            report["motion_evaluation"] = evaluate_motion(motion_samples, initial, cfg)
            phase("motion_evaluation")
            if not report["motion_evaluation"]["passed"] or len(report["media"]) != 3:
                raise RuntimeError("actual motion/stop/planar/media admission criteria not met; inspect phase checks")
        elif not report["physics"]["stationary_passed"]:
            raise RuntimeError("uncommanded robot did not meet configured final stationary dwell")
        if not motion_profile:
            phase("rgb_capture")
            capture_image("nova-carter-idle", last_time)
        report["status"] = "passed"
        phase("admission_complete")
        code = 0
    except BaseException as error:
        report.update(status="failed", failed_phase=report["stage"], error={"type": type(error).__name__,
            "message": str(error), "traceback": traceback.format_exc(limit=20)})
    finally:
        # Persist evidence before close: SDK fast shutdown may not return to Python.
        save(args.output / "result.json", report)
        if world is not None:
            try:
                world.stop()
                report["timeline_stopped"] = bool(world.is_stopped())
                if not report["timeline_stopped"]:
                    raise RuntimeError("timeline did not stop")
            except BaseException:
                report.update(status="failed", stop_error=traceback.format_exc(limit=10))
                code = 1
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report["elapsed_wall_s_before_close"] = time.monotonic() - started
        report["sdk_close_attempted"] = app is not None
        save(args.output / "result.json", report)
        print("LOCAL_ASSET_ADMISSION_RESULT", report["status"], flush=True)
        if app is not None:
            try:
                app.close()
                report["sdk_close_returned"] = True
            except BaseException:
                report.update(status="failed", close_error=traceback.format_exc(limit=10))
                code = 1
            save(args.output / "result.json", report)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
