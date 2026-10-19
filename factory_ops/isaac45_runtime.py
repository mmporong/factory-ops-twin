"""Opt-in SDK4.5 facade. Importing this module never imports the SDK or numpy.

Create only after SimulationApp is ready. Native World owns physics and time;
this module does not advance a synthetic clock or author robot poses.
"""
from __future__ import annotations

import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any


ASSET_PATH = "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd"
LIDAR_PROFILE = "RPLIDAR_S2E"
LIDAR_WRITER = "RtxLidarROS2PublishLaserScan"
_DLL_HANDLES: dict[str, Any] = {}


def _sdk_root() -> Path:
    value = os.environ.get("ISAAC_PATH")
    if not value or not Path(value).is_absolute() or not Path(value).is_dir():
        raise ValueError("ISAAC_PATH must name the existing absolute SDK directory")
    return Path(value).resolve()


def configure_ros_environment(domain_id: int | None = None) -> dict[str, Any]:
    """Configure only this process, before enabling the bundled ROS extension."""
    library = _sdk_root() / "exts/isaacsim.ros2.bridge/humble/lib"
    if not (library / "rmw_fastrtps_cpp.dll").is_file():
        raise RuntimeError("bundled Humble middleware DLL is missing")
    if domain_id is not None and (type(domain_id) is not int or not 0 <= domain_id <= 232):
        raise ValueError("ROS domain must be an integer between zero and232")
    key = str(library)
    # Keep the handles alive: dropping them removes the DLL search directory.
    if hasattr(os, "add_dll_directory") and key not in _DLL_HANDLES:
        _DLL_HANDLES[key] = os.add_dll_directory(key)
    entries = os.environ.get("PATH", "").split(os.pathsep)
    if key.casefold() not in {entry.casefold() for entry in entries}:
        os.environ["PATH"] = key + os.pathsep + os.environ.get("PATH", "")
    os.environ.update(ROS_DISTRO="humble", RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_LOCALHOST_ONLY="0")
    if domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    return {"library_path": key, "ros_distro": "humble", "rmw": "rmw_fastrtps_cpp",
            "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "scope": "current process only; enable isaacsim.ros2.bridge before importing rclpy"}


def _load_bindings() -> SimpleNamespace:
    # This is called by create_runtime, never by importing the facade.
    return SimpleNamespace(
        stage=importlib.import_module("isaacsim.core.utils.stage"),
        usd=importlib.import_module("omni.usd"),
        physics=importlib.import_module("pxr.UsdPhysics"),
        World=importlib.import_module("isaacsim.core.api").World,
        Robot=importlib.import_module("isaacsim.robot.wheeled_robots.robots").WheeledRobot,
        Action=importlib.import_module("isaacsim.core.utils.types").ArticulationAction,
        assets=importlib.import_module("isaacsim.storage.native").get_assets_root_path,
    )


def _finite(value: Any) -> bool:
    try:
        return not isinstance(value, (bool, str, bytes)) and math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


class _StageUtils:
    def __init__(self, runtime):
        self._runtime = runtime

    def open_stage(self, path):
        if self._runtime._robots or self._runtime.SimulationManager.world is not None:
            raise RuntimeError("cannot replace a stage after creating facade robots or World")
        opened = bool(self._runtime._bindings.stage.open_stage(str(path)))
        stage = self._runtime._bindings.usd.get_context().get_stage() if opened else None
        return opened, stage

    def add_reference_to_stage(self, usd_path, path):
        if self._runtime.SimulationManager.world is not None:
            raise RuntimeError("robot references must be added before World setup")
        return self._runtime._bindings.stage.add_reference_to_stage(usd_path=usd_path, prim_path=path)

    def is_stage_loading(self):
        return self._runtime._bindings.stage.is_stage_loading()


class _Robot:
    def __init__(self, runtime, prim_path, wheel_dof_names):
        self._runtime = runtime
        self.prim_path = prim_path
        self._wheel_names = tuple(wheel_dof_names)
        self._native: Any = None

    def _ready(self):
        if self._native is None or not self._runtime.SimulationManager.ready:
            raise RuntimeError("robot is not initialized; call setup_simulation first")
        return self._native

    @property
    def dof_names(self):
        return self._ready().dof_names

    def get_dof_positions(self):
        return self._ready().get_joint_positions()

    def get_world_poses(self):
        position, orientation = self._ready().get_world_pose()
        return [position], [orientation]  # G2 expects a single-articulation batch.

    def apply_wheel_actions(self, vector):
        native = self._ready()
        if getattr(vector, "ndim", 1) != 1 or len(vector) != len(self._wheel_names):
            raise ValueError("wheel velocity vector shape differs from configured wheels")
        if not all(_finite(value) for value in vector):
            raise ValueError("wheel velocity vector must be finite")
        native.apply_wheel_actions(self._runtime._bindings.Action(joint_velocities=vector))


class _SimulationManager:
    def __init__(self, runtime):
        self._runtime = runtime
        self.world: Any = None
        self.ready = False
        self._dt = None

    def setup_simulation(self, dt, device="cpu"):
        if isinstance(dt, bool) or not _finite(dt) or dt <= 0 or device != "cpu":
            raise ValueError("SDK4.5 facade requires positive finite dt and CPU PhysX")
        if self.world is not None:
            if self.ready and self._dt == dt:
                return self.world
            raise RuntimeError("World already exists or prior setup failed; refusing another reset")
        bindings = self._runtime._bindings
        stage = bindings.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("an open stage is required")
        scenes = [prim for prim in stage.Traverse() if prim.IsA(bindings.physics.Scene)]
        if len(scenes) != 1:
            raise ValueError("stage must contain exactly one existing PhysicsScene")
        if bindings.World.instance() is not None:
            raise RuntimeError("a foreign World already owns the SDK simulation context")
        scene_path = str(scenes[0].GetPath())
        self.world = bindings.World(physics_dt=float(dt), rendering_dt=float(dt),
            stage_units_in_meters=1., physics_prim_path=scene_path, backend="numpy", device="cpu")
        self._dt = dt
        self.world.get_physics_context().enable_gpu_dynamics(False)
        for index, robot in enumerate(self._runtime._robots):
            robot._native = self.world.scene.add(bindings.Robot(prim_path=robot.prim_path,
                name=f"local45_robot_{index}", wheel_dof_names=list(robot._wheel_names), create_robot=False))
        self.world.reset()
        actual_scenes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(bindings.physics.Scene)]
        if actual_scenes != [scene_path]:
            raise RuntimeError("World changed the existing PhysicsScene identity")
        self.ready = True
        return self.world

    def get_simulation_time(self):
        # SDK SimulationContext subscribes its timer to real PhysX step events.
        return float(self.world.current_time) if self.ready else 0.

    def get_num_physics_steps(self):
        return int(self.world.current_time_step_index) if self.ready else 0


class _FlatScanObservationHandle:
    def __init__(self, product, identity, range_min_m, range_max_m):
        from factory_ops.flat_scan_observation import FlatScanObservationTracker

        rep = importlib.import_module("omni.replicator.core")
        self._tracker = FlatScanObservationTracker(identity, range_min_m, range_max_m)
        self._annotator = rep.AnnotatorRegistry.get_annotator(
            "RtxSensorCpuIsaacComputeRTXLidarFlatScanSimulationTime")
        self._annotator.attach([product])

    def read(self, **reader_context):
        if self._annotator is None:
            raise RuntimeError("native FlatScan observer is closed")
        payload = self._annotator.get_data()
        if isinstance(payload, dict):
            depths = payload.get("linearDepthData")
            reshape = getattr(depths, "reshape", None)
            if callable(reshape):
                payload = {**payload, "linearDepthData": reshape(-1)}
        return self._tracker.observe(payload, **reader_context)

    def close(self):
        if self._annotator is not None:
            self._annotator.detach()
            self._annotator = None


class _LidarSensor:
    def __init__(self, product):
        self._product = product
        self._writer: Any = None
        self._observer: Any = None

    def open_native_observer(self, identity, range_min_m, range_max_m):
        if self._product is None or self._observer is not None:
            raise RuntimeError("sensor is closed or already owns a FlatScan observer")
        observer = _FlatScanObservationHandle(self._product, identity, range_min_m, range_max_m)
        self._observer = observer
        return observer

    def detach_writer(self, name):
        if name != LIDAR_WRITER:
            raise ValueError("facade owns only the simulation-time LaserScan writer")
        errors = []
        if self._observer is not None:
            try:
                self._observer.close()
                self._observer = None
            except Exception as error:
                errors.append(f"observer.close: {error}")
        if self._writer is not None:
            try:
                self._writer.detach()
                self._writer = None
            except Exception as error:
                errors.append(f"writer.detach: {error}")
        if self._product is not None:
            try:
                self._product.destroy()
                self._product = None
            except Exception as error:
                errors.append(f"render_product.destroy: {error}")
        if errors:
            raise RuntimeError("; ".join(errors))


class _TfMessagePublisher:
    """Publish the TF wire protocol using the messages bundled with SDK4.5."""

    def __init__(self, node, *, static=False):
        message = importlib.import_module("tf2_msgs.msg")
        qos = importlib.import_module("rclpy.qos")
        self._message = message.TFMessage
        self._static = static
        self._transforms = {}
        profile = (qos.QoSProfile(depth=1, durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=qos.HistoryPolicy.KEEP_LAST) if static else qos.QoSProfile(depth=100))
        self._publisher = node.create_publisher(self._message, "tf_static" if static else "tf", profile)

    def sendTransform(self, transform):
        transforms = list(transform) if isinstance(transform, (list, tuple)) else [transform]
        if self._static:
            for row in transforms:
                self._transforms.setdefault(row.child_frame_id, row)
            transforms = list(self._transforms.values())
        self._publisher.publish(self._message(transforms=transforms))


class Runtime45:
    def __init__(self, app, bindings):
        self.app, self._bindings = app, bindings
        self._robots = []
        self.stage_utils = _StageUtils(self)
        self.SimulationManager = _SimulationManager(self)

    @staticmethod
    def tf_broadcaster_types():
        return (_TfMessagePublisher, lambda node: _TfMessagePublisher(node, static=True))

    def WheeledRobot(self, prim_path, wheel_dof_names):
        if self.SimulationManager.world is not None:
            raise RuntimeError("register robot before World setup")
        if isinstance(wheel_dof_names, (str, bytes)):
            raise ValueError("wheel names must be a sequence of joint names")
        names = tuple(wheel_dof_names)
        if (not isinstance(prim_path, str) or not prim_path.startswith("/") or not names
                or any(not isinstance(name, str) or not name for name in names) or len(set(names)) != len(names)
                or any(robot.prim_path == prim_path for robot in self._robots)):
            raise ValueError("robot path/wheel identities must be nonempty and distinct")
        robot = _Robot(self, prim_path, names)
        self._robots.append(robot)
        return robot

    def get_assets_root_path(self):
        root = self._bindings.assets()
        if not isinstance(root, str) or not root:
            raise RuntimeError("installed get_assets_root_path returned no asset root")
        return root.rstrip("/")

    @property
    def asset_url(self):
        return self.get_assets_root_path() + ASSET_PATH

    def create_capture_light(self):
        stage = self._bindings.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("capture light requires the current stage")
        lux = importlib.import_module("pxr.UsdLux")
        light = lux.DomeLight.Define(stage, "/World/G0bPersistentDomeLight")
        light.CreateIntensityAttr(800.)
        return light

    def setup_lidars(self, config, robots, stage, Usd, UsdGeom, Gf,
                     SimulationManager, app, author_mount, verify_mount):
        if SimulationManager is not self.SimulationManager or app is not self.app:
            raise ValueError("lidars and physics must belong to this runtime/application")
        if stage != self._bindings.usd.get_context().get_stage():
            raise ValueError("lidar stage differs from the current runtime stage")
        profile_path = _sdk_root() / "exts/isaacsim.sensors.rtx/data/lidar_configs/SLAMTEC/RPLIDAR_S2E.json"
        profile_bytes = profile_path.read_bytes()
        profile = json.loads(profile_bytes)["profile"]
        fields = ("scanRateBaseHz", "reportRateBaseHz", "nearRangeM", "farRangeM")
        if (profile.get("scanType") != "rotary" or any(not _finite(profile[key]) or profile[key] <= 0 for key in fields)
                or profile["nearRangeM"] >= profile["farRangeM"]):
            raise ValueError("installed rotary lidar profile parameters are invalid")
        specs, paths, topics = [], set(), set()
        for key, runtime in robots.items():
            definition = runtime["definition"]
            robot = runtime["robot"]
            if (robot not in self._robots or definition["robot_id"] != key
                    or robot.prim_path != definition["prim_path"]
                    or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", definition["namespace"])):
                raise ValueError("lidar robot definition differs from registered robot")
            chassis = definition["prim_path"] + "/chassis_link"
            if not stage.GetPrimAtPath(chassis).IsValid():
                raise ValueError("lidar chassis prim is missing")
            for lidar in config["lidars"]:
                if (not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", lidar["name"])
                        or not isinstance(lidar["frame_id"], str) or not lidar["frame_id"]
                        or len(lidar["translation_xyz_m"]) != 3
                        or not all(_finite(v) for v in lidar["translation_xyz_m"]) or not _finite(lidar["yaw_rad"])):
                    raise ValueError("lidar identity or mount is invalid")
                path = chassis + "/lidar_" + lidar["name"]
                topic = f"/{definition['namespace']}/scan_{lidar['name']}_raw"
                if path in paths or topic in topics or stage.GetPrimAtPath(path).IsValid():
                    raise ValueError("lidar prim/topic identity already exists")
                paths.add(path)
                topics.add(topic)
                specs.append((definition, lidar, chassis, path, topic))
        if not specs:
            raise ValueError("at least one configured lidar is required")
        # Initialize/reset World before creating any RTX sensor graph resources.
        SimulationManager.setup_simulation(dt=float(config["physics_dt_s"]), device="cpu")
        commands = importlib.import_module("omni.kit.commands")
        rep = importlib.import_module("omni.replicator.core")
        sensors, metadata = [], []
        try:
            for definition, lidar, chassis, path, topic in specs:
                ok, prim = commands.execute("IsaacSensorCreateRtxLidar", path=path, parent=None,
                    config=LIDAR_PROFILE, translation=(0., 0., 0.), orientation=Gf.Quatd(1., 0., 0., 0.))
                if not ok or prim is None or not prim.IsValid() or str(prim.GetPath()) != path:
                    raise RuntimeError("installed RTX command did not create the requested sensor")
                author_mount(prim, UsdGeom, Gf, lidar["translation_xyz_m"], float(lidar["yaw_rad"]))
                product = rep.create.render_product(prim.GetPath(), (1, 1), name=f"Local45_{definition['robot_id']}_{lidar['name']}")
                sensor = _LidarSensor(product)
                sensors.append(sensor)
                sensor._writer = rep.writers.get(LIDAR_WRITER)
                sensor._writer.initialize(topicName=topic, frameId=lidar["frame_id"])
                sensor._writer.attach([product])
                metadata.append({"robot_id": definition["robot_id"], "name": lidar["name"],
                    "namespace": definition["namespace"], "prim_path": path, "topic": topic,
                    "frame_id": lidar["frame_id"], "translation_xyz_m": list(lidar["translation_xyz_m"]),
                    "yaw_rad": float(lidar["yaw_rad"]), "profile": LIDAR_PROFILE,
                    "requested_profile": config.get("lidar_profile"), "profile_path": str(profile_path),
                    "profile_sha256": hashlib.sha256(profile_bytes).hexdigest(),
                    "rotation_frequency_hz": profile["scanRateBaseHz"], "report_rate_hz": profile["reportRateBaseHz"],
                    "near_range_m": profile["nearRangeM"], "far_range_m": profile["farRangeM"],
                    "horizontal_resolution_deg": 360. * profile["scanRateBaseHz"] / profile["reportRateBaseHz"],
                    "writer": LIDAR_WRITER, "clock_source": "simulation_time",
                    "parameter_source": "installed4.5 JSON profile; actual ROS scan header validation is separate",
                    "runtime": "SDK4.5 opt-in; no6.2 tickRate/accumulateOutputs attributes synthesized"})
            app.update()
            for row, (definition, lidar, chassis, path, topic) in zip(metadata, specs):
                row["local_transform"] = verify_mount(stage, Usd, UsdGeom, path, chassis,
                    lidar["translation_xyz_m"], float(lidar["yaw_rad"]))
            return sensors, metadata
        except BaseException as error:
            cleanup_errors = []
            for sensor in sensors:
                try:
                    sensor.detach_writer(LIDAR_WRITER)
                except Exception as cleanup_error:
                    cleanup_errors.append(str(cleanup_error))
            if cleanup_errors:
                raise RuntimeError(f"lidar setup failed: {error}; cleanup: {cleanup_errors}") from error
            raise


def create_runtime(app) -> Runtime45:
    """Bind the installed SDK only when called inside the native application."""
    return Runtime45(app, _load_bindings())
