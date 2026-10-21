"""SDK-free admission of one unchanged carrier candidate against native bounds.

Matrices use column vectors (translation in the last column), SI units and a
right-handed Z-up frame. Conservative composed AABBs can reject a legal mount;
they do not establish a mesh support surface, sensor FOV or mechanical strength.
"""
from __future__ import annotations

from copy import deepcopy
import itertools
import math
import re
from typing import Any, Sequence

from .carrier_module import manifest_sha256


ROBOT_PATH = "/World/NovaCarter"
CHASSIS_PATH = ROBOT_PATH+"/chassis_link"
ASSET_PATH = "/Isaac/Robots/NVIDIA/Carter/nova_carter/nova_carter.usd"
IDENTITY = [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]


def _finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("native measurement must contain finite numeric values")
    return float(value)


def _vector(value: Any, length: int) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("native measurement vector length mismatch")
    return [_finite(v) for v in value]


def rigid_matrix(value: Any) -> list[list[float]]:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("rigid transform needs four rows")
    m = [_vector(row, 4) for row in value]
    if any(abs(m[3][j]-IDENTITY[3][j]) > 1e-8 for j in range(4)):
        raise ValueError("nonaffine native transform")
    if any(abs(sum(m[k][i]*m[k][j] for k in range(3))-(i == j)) > 1e-6
           for i in range(3) for j in range(3)):
        raise ValueError("scaled or sheared native transform")
    determinant = sum(m[0][j]*(m[1][(j+1)%3]*m[2][(j+2)%3]-m[1][(j+2)%3]*m[2][(j+1)%3]) for j in range(3))
    if abs(determinant-1.) > 1e-6:
        raise ValueError("native transform must be right handed")
    return m


def inverse_rigid(value: Any) -> list[list[float]]:
    m = rigid_matrix(value)
    return [[m[j][i] for j in range(3)]+[-sum(m[j][i]*m[j][3] for j in range(3))]
            for i in range(3)]+[[0., 0., 0., 1.]]


def transform_point(matrix: Any, point: Any) -> list[float]:
    m, p = rigid_matrix(matrix), _vector(point, 3)
    return [sum(m[i][j]*p[j] for j in range(3))+m[i][3] for i in range(3)]


def pose_matrix(position: Any, orientation_wxyz: Any) -> list[list[float]]:
    p, q = _vector(position, 3), _vector(orientation_wxyz, 4)
    if abs(sum(v*v for v in q)-1.) > 1e-4:
        raise ValueError("native quaternion is not normalized")
    w, x, y, z = q
    return rigid_matrix([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), p[0]],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), p[1]],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), p[2]],
                         [0., 0., 0., 1.]])


def transformed_box(box: dict, matrix: Any) -> dict:
    center, size = _vector(box["center_m"], 3), _vector(box["size_m"], 3)
    if any(v <= 0 for v in size):
        raise ValueError("native bound must have positive extent")
    corners = [transform_point(matrix, [center[i]+signs[i]*size[i]/2 for i in range(3)])
               for signs in itertools.product((-1, 1), repeat=3)]
    lo = [min(p[i] for p in corners) for i in range(3)]
    hi = [max(p[i] for p in corners) for i in range(3)]
    return {"center_m": [(a+b)/2 for a, b in zip(lo, hi)], "size_m": [b-a for a, b in zip(lo, hi)]}


def _bounds(box: dict) -> tuple[list[float], list[float]]:
    return ([c-s/2 for c, s in zip(box["center_m"], box["size_m"])],
            [c+s/2 for c, s in zip(box["center_m"], box["size_m"])])


def _radius(boxes: Sequence[dict]) -> float:
    return max(math.hypot(x, y) for b in boxes for lo, hi in [_bounds(b)]
               for x in (lo[0], hi[0]) for y in (lo[1], hi[1]))


def _path(value: Any, root: str = ROBOT_PATH) -> str:
    if (not isinstance(value, str) or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", value)
            or not (value == root or value.startswith(root+"/"))):
        raise ValueError("unexpected native prim identity")
    return value


def assess_mount(manifest: dict, measured: dict, expected_run_id: str, *, clearance_m: float = .01,
                 robot_path: str = ROBOT_PATH) -> dict:
    """Return admission/rejection; malformed identity/frame inputs raise ValueError.

    Only identity carrier-to-chassis placement is proposed. Four post bottoms
    must lie above a chassis collision AABB face within the existing clearance.
    No geometry search, collider suppression, radius increase or pose correction.
    """
    if not isinstance(robot_path, str) or not re.fullmatch(r"/World/[A-Za-z_][A-Za-z0-9_]*", robot_path):
        raise ValueError("expected robot root must be one prim below /World")
    chassis_path = robot_path+"/chassis_link"
    if manifest["schema_version"] != 1 or manifest["profile_id"] != "tote-carrier-v1":
        raise ValueError("unexpected carrier manifest identity")
    if (not isinstance(expected_run_id, str) or not re.fullmatch("[0-9a-f]{32}", expected_run_id)
            or measured["run_id"] != expected_run_id or measured["schema_version"] != 1
            or isinstance(measured["schema_version"], bool)):
        raise ValueError("native run/schema identity mismatch")
    if (measured["robot_path"] != robot_path or measured["chassis_path"] != chassis_path
            or measured["asset_relative_path"] != ASSET_PATH or measured["up_axis"] != "Z"
            or _finite(measured["meters_per_unit"]) != 1.):
        raise ValueError("native robot, chassis, asset or units mismatch")
    robot_matrix = rigid_matrix(measured["robot_world_from_local"])
    chassis_matrix = rigid_matrix(measured["chassis_world_from_local"])
    if chassis_matrix[2][2] < 1.-1e-6:
        raise ValueError("candidate requires upright chassis local Z")
    articulation = measured["articulation_roots"]
    if len(articulation) != 1:
        raise ValueError("one unambiguous native articulation root required")
    _path(articulation[0], robot_path)
    bodies = measured["rigid_bodies"]
    paths = [_path(b["path"], robot_path) for b in bodies]
    if len(paths) != len(set(paths)) or chassis_path not in paths:
        raise ValueError("missing or ambiguous chassis rigid body")
    chassis = bodies[paths.index(chassis_path)]
    if chassis["enabled"] is not True or chassis["kinematic"] is not False:
        raise ValueError("chassis must be an enabled dynamic body")
    to_chassis, to_robot = inverse_rigid(chassis_matrix), inverse_rigid(robot_matrix)
    colliders, root_boxes, seen = [], [], set()
    for c in measured["colliders"]:
        path, owner = _path(c["path"], robot_path), _path(c["body_path"], robot_path)
        if path in seen or owner not in paths or not (path == owner or path.startswith(owner+"/")):
            raise ValueError("ambiguous collider ownership")
        seen.add(path)
        colliders.append(dict(transformed_box(c["world_aabb_m"], to_chassis), path=path, body_path=owner))
        root_boxes.append(transformed_box(c["world_aabb_m"], to_robot))
    if not colliders or not any(c["body_path"] == chassis_path for c in colliders):
        raise ValueError("native chassis collision geometry is missing")
    sensors, seen = [], set()
    for sensor in measured["sensors"]:
        path = _path(sensor["path"], robot_path)
        if path in seen:
            raise ValueError("ambiguous native sensor identity")
        seen.add(path)
        frame = rigid_matrix(sensor["world_from_local"])
        # Transform a declared 0.16 m housing clearance around each actual sensor frame.
        world_box = transformed_box({"center_m": [0., 0., 0.], "size_m": [.16]*3}, frame)
        sensors.append(dict(transformed_box(world_box, to_chassis), path=path))
    if not sensors:
        raise ValueError("no actual camera/lidar sensor frames found")
    geometry = deepcopy(manifest["geometry"])
    if (sum(b["name"].startswith("post_") and b["body"] == "carrier" for b in geometry) != 4
            or sum(b["name"] == "deck" and b["body"] == "carrier" for b in geometry) != 1
            or not any(b["body"] == "tote" for b in geometry)):
        raise ValueError("complete carrier and tote geometry required")
    for box in geometry:
        if box["body"] == "tote":
            box["center_m"] = [a+b for a, b in zip(box["center_m"], manifest["tote_rest_origin_m"])]
        elif box["body"] != "carrier":
            raise ValueError("unexpected carrier manifest body")
        transformed_box(box, IDENTITY)
    gap = _finite(manifest["compatibility"]["unchanged_footprint_radius_m"])
    if gap <= 0:
        raise ValueError("existing footprint radius must be positive")
    canonical_radius = gap
    clearance = _finite(clearance_m)
    if clearance <= 0:
        raise ValueError("positive declared clearance required")
    rejection, mount_faces = [], []
    for post in (b for b in geometry if b["name"].startswith("post_")):
        low, high = _bounds(post)
        faces = []
        for c in colliders:
            lo, hi = _bounds(c)
            if (c["body_path"] == chassis_path and -1e-8 <= low[2]-hi[2] <= clearance
                    and all(low[i] >= lo[i]-1e-8 and high[i] <= hi[i]+1e-8 for i in (0, 1))):
                faces.append(c["path"])
        mount_faces.append({"post": post["name"], "candidate_chassis_faces": sorted(faces)})
        if not faces:
            rejection.append({"code": "unsupported_mount_post", "part": post["name"]})
    for box in geometry:
        low, high = _bounds(box)
        allowed = next((set(f["candidate_chassis_faces"]) for f in mount_faces if f["post"] == box["name"]), set())
        for c in colliders+sensors:
            lo, hi = _bounds(c)
            separation = max(max(lo[i]-high[i], low[i]-hi[i]) for i in range(3))
            if separation < clearance-1e-8 and c["path"] not in allowed:
                rejection.append({"code": "robot_or_sensor_keepout", "part": box["name"], "prim": c["path"]})
    loaded_root_boxes = [transformed_box(transformed_box(b, chassis_matrix), to_robot) for b in geometry]
    original_radius, loaded_radius = _radius(root_boxes), _radius(root_boxes+loaded_root_boxes)
    if original_radius > canonical_radius or loaded_radius > min(original_radius, canonical_radius):
        rejection.append({"code": "unchanged_footprint_exceeded"})
    return {"schema_version": 1, "run_id": expected_run_id, "admitted": not rejection,
        "rejections": rejection, "manifest_sha256": manifest_sha256(manifest),
        "robot_path": robot_path, "chassis_path": chassis_path,
        "carrier_from_chassis": deepcopy(IDENTITY), "mount_faces": mount_faces,
        "original_collision_aabb_radius_m": original_radius, "loaded_collision_aabb_radius_m": loaded_radius,
        "unchanged_configured_radius_m": canonical_radius,
        "measured_colliders_in_chassis": colliders, "sensor_keepouts_in_chassis": sensors,
        "scope": "composed collision AABB/frame admission only; authored sensor housing boxes, no FOV or mount strength proof",
        "authored_geometry_changed": False, "map_or_footprint_changed": False,
        "physical_attachment_verified": False}
