"""Validate declared fleet identities without importing ROS or Isaac Sim."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
import math
import re
from typing import Any


_IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
_SENSOR_NAMES = frozenset(("front", "rear"))


def _name(value: Any, field: str) -> str:
    if not isinstance(value, str) or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(f"{field} must be a single valid identifier")
    return value


def _number(value: Any, field: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{field} must be a finite number, not bool")
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(f"{field} must be finite") from error
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _triple(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)) or len(value) != 3:
        raise ValueError(f"{field} must contain exactly three numbers")
    return (_number(value[0], f"{field}[0]"), _number(value[1], f"{field}[1]"),
            _number(value[2], f"{field}[2]"))


def _prim_path(value: Any) -> str:
    if not isinstance(value, str) or not value.startswith("/World/"):
        raise ValueError("prim_path must be a descendant of /World")
    for part in value.removeprefix("/World/").split("/"):
        _name(part, "prim_path segment")
    return value


class FleetRegistry:
    """Isolate and validate configuration; no runtime or footprint qualification.

    Identity properties have no setters. Mutable definitions, spawns and
    manifests are returned as detached copies, preserving the validated state.
    """

    __slots__ = ("_robot_ids", "_definitions", "_spawns", "_lidars", "_sensor_keys")

    def __init__(self, config: Mapping[str, Any], scenario: Mapping[str, Any]) -> None:
        if not isinstance(config, Mapping) or not isinstance(scenario, Mapping):
            raise ValueError("config and scenario must be mappings")
        robots = config.get("robots")
        if not isinstance(robots, list) or not robots:
            raise ValueError("robots must be a non-empty list")
        definitions: list[dict[str, Any]] = []
        robot_ids: set[str] = set()
        namespaces: set[str] = set()
        prim_paths: set[str] = set()
        for definition in robots:
            if not isinstance(definition, Mapping):
                raise ValueError("each robot definition must be a mapping")
            robot_id = _name(definition.get("robot_id"), "robot_id")
            namespace = _name(definition.get("namespace"), "namespace")
            prim_path = _prim_path(definition.get("prim_path"))
            if robot_id in robot_ids or namespace in namespaces or prim_path in prim_paths:
                raise ValueError("robot_id, namespace and prim_path must each be unique")
            if any(prim_path.startswith(path + "/") or path.startswith(prim_path + "/") for path in prim_paths):
                raise ValueError("robot prim paths must not be nested")
            robot_ids.add(robot_id)
            namespaces.add(namespace)
            prim_paths.add(prim_path)
            definitions.append(deepcopy(dict(definition)))

        spawns = scenario.get("spawns")
        if not isinstance(spawns, Mapping) or set(spawns) != robot_ids:
            raise ValueError("spawn keys must match robot_ids exactly")
        validated_spawns: dict[str, tuple[float, float, float]] = {}
        occupied: set[tuple[float, float]] = set()
        for definition in definitions:
            robot_id = definition["robot_id"]
            spawn = _triple(spawns[robot_id], f"spawns.{robot_id}")
            xy = spawn[:2]
            if xy in occupied:
                raise ValueError("robot spawn XY coordinates must be distinct")
            occupied.add(xy)
            validated_spawns[robot_id] = spawn

        lidars = config.get("lidars")
        if not isinstance(lidars, list) or len(lidars) != 2:
            raise ValueError("lidars must contain exactly front and rear")
        validated_lidars: list[dict[str, Any]] = []
        sensor_names: set[str] = set()
        frame_ids: set[str] = set()
        for lidar in lidars:
            if not isinstance(lidar, Mapping):
                raise ValueError("each lidar definition must be a mapping")
            name = _name(lidar.get("name"), "lidar.name")
            frame_id = _name(lidar.get("frame_id"), "lidar.frame_id")
            if name not in _SENSOR_NAMES or name in sensor_names or frame_id in frame_ids:
                raise ValueError("front/rear lidar names and frame_ids must be unique")
            _triple(lidar.get("translation_xyz_m"), f"lidars.{name}.translation_xyz_m")
            _number(lidar.get("yaw_rad"), f"lidars.{name}.yaw_rad")
            sensor_names.add(name)
            frame_ids.add(frame_id)
            validated_lidars.append(deepcopy(dict(lidar)))

        self._definitions = tuple(definitions)
        self._robot_ids = tuple(definition["robot_id"] for definition in definitions)
        self._spawns = validated_spawns
        self._lidars = tuple(validated_lidars)
        self._sensor_keys = frozenset((robot_id, name) for robot_id in self._robot_ids for name in _SENSOR_NAMES)

    @property
    def robot_ids(self) -> tuple[str, ...]:
        return self._robot_ids

    @property
    def definitions(self) -> tuple[dict[str, Any], ...]:
        return deepcopy(self._definitions)

    @property
    def spawns(self) -> dict[str, tuple[float, float, float]]:
        return dict(self._spawns)

    @property
    def sensor_keys(self) -> frozenset[tuple[str, str]]:
        return self._sensor_keys

    def manifest(self) -> dict[str, Any]:
        """Describe namespaced topics and intended sensors; create no publishers."""
        robots = []
        lidars = []
        for definition in self._definitions:
            robot_id, namespace = definition["robot_id"], definition["namespace"]
            topics = {name: f"/{namespace}/{name}" for name in (
                "scan_front_raw", "scan_front", "scan_rear_raw", "scan_rear", "odom", "cmd_vel")}
            robots.append({**deepcopy(definition), "spawn_xy_yaw": list(self._spawns[robot_id]), "topics": topics})
            for lidar in self._lidars:
                name = lidar["name"]
                lidars.append({**deepcopy(lidar), "robot_id": robot_id, "namespace": namespace,
                               "prim_path": f"{definition['prim_path']}/chassis_link/lidar_{name}",
                               "raw_topic": topics[f"scan_{name}_raw"], "filtered_topic": topics[f"scan_{name}"]})
        return {"schema_version": 1, "robot_ids": list(self._robot_ids), "robots": robots,
                "lidars": lidars, "lidar_count": len(lidars),
                "scope": "declared configuration only; not runtime qualification"}
