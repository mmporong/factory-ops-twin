"""Lazy SDK4.5 adapter for opt-in chassis-fixed fleet carrier bodies.

The adapter measures and admits the complete four-robot fleet before it authors
anything.  Only the carrier body is physical here; tote and payload handling
remain presentation models and are not loading/unloading evidence.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import re
from types import SimpleNamespace
from typing import Any, Callable, Mapping, Sequence

from factory_ops.carrier_module import manifest_sha256
from factory_ops.carrier_robot import ASSET_PATH, assess_mount, rigid_matrix


_RUN_ID = re.compile(r"[0-9a-f]{32}\Z")
_IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")
_ROOT_PATH = re.compile(r"/World/[A-Za-z_][A-Za-z0-9_]*\Z")
_MODULE_ROOT = "/World/CarrierModules"
_MATERIAL_PATH = _MODULE_ROOT + "/CarrierMaterial"


def _sdk() -> SimpleNamespace:
    """Import pxr only inside an initialized Isaac Sim SDK process."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade
    return SimpleNamespace(Gf=Gf, PhysxSchema=PhysxSchema, Usd=Usd, UsdGeom=UsdGeom,
                           UsdPhysics=UsdPhysics, UsdShade=UsdShade)


def _matrix(prim: Any, transforms: Any) -> list[list[float]]:
    value = transforms.GetLocalToWorldTransform(prim)
    # USD Gf matrices use row vectors; carrier_robot validates column vectors.
    return rigid_matrix([[float(value[j][i]) for j in range(4)] for i in range(4)])


def _collision_bound(collider: Any, transforms: Any, sdk: SimpleNamespace) -> tuple[Any, list[dict[str, Any]]]:
    """Compute geometry extents afresh; never trust an authored extent cache."""
    Gf, Usd, UsdGeom = sdk.Gf, sdk.Usd, sdk.UsdGeom
    combined, leaves = Gf.Range3d(), []
    for leaf in Usd.PrimRange(collider, Usd.TraverseInstanceProxies()):
        boundable = UsdGeom.Boundable(leaf)
        if not boundable:
            continue
        computed = UsdGeom.Boundable.ComputeExtentFromPlugins(boundable, Usd.TimeCode.Default())
        if computed is None or len(computed) != 2:
            raise ValueError("no geometry extent computation for collider leaf: " + str(leaf.GetPath()))
        lo, hi = ([float(v) for v in point] for point in computed)
        if (not all(math.isfinite(v) for v in lo + hi)
                or any(b < a for a, b in zip(lo, hi)) or all(b == a for a, b in zip(lo, hi))):
            raise ValueError("invalid geometry-computed collider extent: " + str(leaf.GetPath()))
        local = Gf.Range3d(Gf.Vec3d(*lo), Gf.Vec3d(*hi))
        world = Gf.BBox3d(local, transforms.GetLocalToWorldTransform(leaf)).ComputeAlignedRange()
        combined.UnionWith(world)
        authored = boundable.GetExtentAttr().Get()
        leaves.append({"path": str(leaf.GetPath()), "type": leaf.GetTypeName(),
            "authored_extent": None if authored is None else [[float(v) for v in p] for p in authored],
            "computed_extent": [lo, hi],
            "world_aabb_m": {"center_m": [float(v) for v in world.GetMidpoint()],
                             "size_m": [float(v) for v in world.GetSize()]}})
    if not leaves or combined.IsEmpty():
        raise ValueError("enabled collider has no computable geometry leaves: " + str(collider.GetPath()))
    return combined, leaves


def measure_robot_geometry(stage: Any, robot_path: str, run_id: str) -> dict[str, Any]:
    """Measure one composed Nova Carter using live USD transforms and geometry."""
    sdk = _sdk()
    Usd, UsdGeom, UsdPhysics = sdk.Usd, sdk.UsdGeom, sdk.UsdPhysics
    if _ROOT_PATH.fullmatch(robot_path) is None or _RUN_ID.fullmatch(run_id) is None:
        raise ValueError("valid immediate /World robot path and run id required")
    root, chassis_path = stage.GetPrimAtPath(robot_path), robot_path + "/chassis_link"
    chassis = stage.GetPrimAtPath(chassis_path)
    if not root.IsValid() or not chassis.IsValid():
        raise ValueError("declared robot root or chassis is absent: " + robot_path)
    transforms = UsdGeom.XformCache(Usd.TimeCode.Default())
    units = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isfinite(units):
        raise ValueError("nonfinite native meters-per-unit")
    measured = {"schema_version": 1, "run_id": run_id, "robot_path": robot_path,
        "chassis_path": chassis_path, "asset_relative_path": ASSET_PATH,
        "meters_per_unit": units, "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "robot_world_from_local": _matrix(root, transforms),
        "chassis_world_from_local": _matrix(chassis, transforms),
        "rigid_bodies": [], "articulation_roots": [], "colliders": [], "sensors": [],
        "composition_errors": [],
        "source": "geometry-computed local extents transformed by native XformCache before authoring; authored extent retained only as diagnosis",
        "sensor_keepout_source": "authored 0.16m cube around measured camera/lidar frame candidates; no FOV claim"}
    count = 0
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        count += 1
        if count > 50000:
            raise RuntimeError("asset inspection prim budget exceeded")
        path = str(prim.GetPath())
        measured["composition_errors"].extend(path + ": " + str(error)
                                                for error in prim.GetPrimIndex().localErrors)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            measured["articulation_roots"].append(path)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = UsdPhysics.RigidBodyAPI(prim)
            measured["rigid_bodies"].append({"path": path,
                "enabled": bool(body.GetRigidBodyEnabledAttr().Get()),
                "kinematic": bool(body.GetKinematicEnabledAttr().Get()),
                "world_from_local": _matrix(prim, transforms)})
        if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            box, sources = _collision_bound(prim, transforms, sdk)
            owner = prim
            while owner.IsValid() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                owner = owner.GetParent()
            center, size = [float(v) for v in box.GetMidpoint()], [float(v) for v in box.GetSize()]
            if not all(math.isfinite(v) for v in center + size) or not all(v > 0 for v in size):
                raise ValueError("invalid enabled native collider bound: " + path)
            measured["colliders"].append({"path": path,
                "body_path": str(owner.GetPath()) if owner.IsValid() else "",
                "shape_type": prim.GetTypeName(),
                "world_aabb_m": {"center_m": center, "size_m": size},
                "extent_sources": sources})
        if (prim.IsA(UsdGeom.Camera) or (prim.IsA(UsdGeom.Xformable)
                and "lidar" in (prim.GetName() + " " + prim.GetTypeName() + " "
                                + " ".join(prim.GetAppliedSchemas())).lower())):
            measured["sensors"].append({"path": path, "type": prim.GetTypeName(),
                "applied_schemas": list(prim.GetAppliedSchemas()),
                "world_from_local": _matrix(prim, transforms)})
    measured["prims_inspected"] = count
    if measured["composition_errors"]:
        raise ValueError("asset composition errors: " + robot_path)
    return measured


def _canonical_sha(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode("utf-8")).hexdigest()


def _visual_tote(manifest: Mapping[str, Any]) -> tuple[list[float], list[float]]:
    boxes = [box for box in manifest["geometry"] if box["body"] == "tote"]
    if not boxes:
        raise ValueError("tote geometry required for visual placement metadata")
    low = [min(box["center_m"][i] - box["size_m"][i] / 2 for box in boxes) for i in range(3)]
    high = [max(box["center_m"][i] + box["size_m"][i] / 2 for box in boxes) for i in range(3)]
    origin = manifest["tote_rest_origin_m"]
    return ([origin[i] + (low[i] + high[i]) / 2 for i in range(3)],
            [high[i] - low[i] for i in range(3)])


def _planned_paths(robot_id: str, manifest: Mapping[str, Any]) -> dict[str, Any]:
    group = f"{_MODULE_ROOT}/{robot_id}"
    carrier = group + "/Carrier"
    return {"group_path": group, "carrier_path": carrier, "joint_path": group + "/ChassisJoint",
            "collider_paths": [carrier + "/" + box["name"]
                               for box in manifest["geometry"] if box["body"] == "carrier"]}


def _validate_authoring_manifest(manifest: Mapping[str, Any]) -> None:
    """Validate every value the SDK authoring pass will consume."""
    boxes = [box for box in manifest["geometry"] if box["body"] == "carrier"]
    if not boxes or len({box["name"] for box in boxes}) != len(boxes):
        raise ValueError("unique carrier compound collider names required")
    props = manifest["mass_properties"]["carrier"]
    mass, center, inertia = props["mass_kg"], props["center_of_mass_m"], props["inertia_matrix_kg_m2"]
    numbers = [mass, *center, *(value for row in inertia for value in row)]
    if (any(type(value) not in (int, float) or not math.isfinite(float(value)) for value in numbers)
            or mass <= 0 or len(center) != 3 or len(inertia) != 3 or any(len(row) != 3 for row in inertia)
            or any(inertia[i][i] <= 0 for i in range(3))
            or any(abs(inertia[i][j]) > 1e-10 for i in range(3) for j in range(3) if i != j)):
        raise ValueError("finite positive mass and axis-aligned carrier principal inertia required")
    contact = manifest["contact"]
    values = [contact[key] for key in ("static_friction", "dynamic_friction", "restitution",
                                        "contact_offset_m", "rest_offset_m")]
    if any(type(value) not in (int, float) or not math.isfinite(float(value)) for value in values):
        raise ValueError("finite carrier contact properties required")


def _assert_paths_absent(stage: Any, paths: Sequence[str]) -> None:
    collisions = [path for path in paths if stage.GetPrimAtPath(path).IsValid()]
    if collisions:
        raise ValueError("carrier authoring paths already exist: " + ", ".join(collisions))


def prepare_carrier_fleet(
    stage: Any, registry_definitions: Sequence[Mapping[str, Any]] | Mapping[str, Mapping[str, Any]],
    manifest: Mapping[str, Any],
    run_id: str, *, clearance_m: float, _measure_robot: Callable[[Any, str, str], dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Measure/admit all four robots and validate every authoring path, with no writes."""
    if isinstance(registry_definitions, Mapping):
        raw_definitions = []
        for key, value in registry_definitions.items():
            if not isinstance(value, Mapping) or value.get("robot_id") != key:
                raise ValueError("registry mapping keys must equal definition robot ids")
            raw_definitions.append(value)
    else:
        raw_definitions = list(registry_definitions)
    if _RUN_ID.fullmatch(run_id or "") is None or len(raw_definitions) != 4:
        raise ValueError("carrier fleet requires one run id and exactly four registry definitions")
    _validate_authoring_manifest(manifest)
    definitions, robot_ids, robot_paths = [], set(), set()
    for raw in raw_definitions:
        definition = dict(raw)
        robot_id, robot_path = definition.get("robot_id"), definition.get("prim_path")
        if (_IDENTIFIER.fullmatch(robot_id or "") is None or _ROOT_PATH.fullmatch(robot_path or "") is None
                or robot_id in robot_ids or robot_path in robot_paths):
            raise ValueError("four unique registry robot ids and immediate /World paths required")
        robot_ids.add(robot_id)
        robot_paths.add(robot_path)
        definitions.append({"robot_id": robot_id, "prim_path": robot_path})
    measure = measure_robot_geometry if _measure_robot is None else _measure_robot
    visual_center, visual_size = _visual_tote(manifest)
    rows, paths = [], [_MODULE_ROOT, _MATERIAL_PATH]
    for definition in definitions:
        robot_id, robot_path = definition["robot_id"], definition["prim_path"]
        measured = measure(stage, robot_path, run_id)
        admission = assess_mount(dict(manifest), measured, run_id,
                                 clearance_m=clearance_m, robot_path=robot_path)
        if admission["admitted"] is not True:
            raise ValueError("carrier mount rejected for " + robot_id + ": " + repr(admission["rejections"]))
        authored = _planned_paths(robot_id, manifest)
        paths.extend([authored["group_path"], authored["carrier_path"], authored["joint_path"],
                      *authored["collider_paths"]])
        rows.append({"robot_id": robot_id, "robot_path": robot_path,
            "chassis_path": robot_path + "/chassis_link", **authored,
            "measurement": measured, "admission": admission,
            "tote_visual_center_m": visual_center, "tote_visual_size_m": visual_size})
    if len(paths) != len(set(paths)):
        raise ValueError("carrier plan contains duplicate authoring paths")
    _assert_paths_absent(stage, paths)
    identity = {"run_id": run_id, "manifest_sha256": manifest_sha256(manifest),
                "definitions": definitions,
                "robots": [{k: row[k] for k in ("robot_id", "robot_path", "chassis_path", "group_path",
                            "carrier_path", "joint_path", "collider_paths", "measurement", "admission")}
                           for row in rows]}
    return {"schema_version": 1, "state": "prepared", "run_id": run_id,
        "manifest_sha256": identity["manifest_sha256"], "fleet_size": 4,
        "module_root_path": _MODULE_ROOT, "material_path": _MATERIAL_PATH,
        "all_authoring_paths": paths, "registry_definitions": definitions,
        "manifest": deepcopy(dict(manifest)), "clearance_m": float(clearance_m), "robots": rows,
        "verification_sha256": _canonical_sha(identity),
        "carrier_physics_authored": False, "tote_physics_authored": False,
        "payload_physics_authored": False, "physical_loading_or_unloading_verified": False,
        "collision_masks_changed": False, "footprint_changed": False,
        "controller_or_sensor_policy_changed": False}


def _gf_matrix_from_column(matrix: Sequence[Sequence[float]], Gf: Any) -> Any:
    column = rigid_matrix(matrix)
    row = [[column[j][i] for j in range(4)] for i in range(4)]
    return Gf.Matrix4d(*(row[i][j] for i in range(4) for j in range(4)))


def _author_one(stage: Any, row: Mapping[str, Any], manifest: Mapping[str, Any], sdk: SimpleNamespace,
                material: Any) -> dict[str, Any]:
    Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade = (sdk.Gf, sdk.PhysxSchema, sdk.UsdGeom,
                                                       sdk.UsdPhysics, sdk.UsdShade)
    UsdGeom.Xform.Define(stage, row["group_path"])
    carrier = UsdGeom.Xform.Define(stage, row["carrier_path"])
    carrier.AddTransformOp().Set(_gf_matrix_from_column(row["measurement"]["chassis_world_from_local"], Gf))
    rigid = UsdPhysics.RigidBodyAPI.Apply(carrier.GetPrim())
    rigid.CreateRigidBodyEnabledAttr(True)
    props = manifest["mass_properties"]["carrier"]
    inertia = props["inertia_matrix_kg_m2"]
    mass = UsdPhysics.MassAPI.Apply(carrier.GetPrim())
    mass.CreateMassAttr(props["mass_kg"])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*props["center_of_mass_m"]))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*(inertia[i][i] for i in range(3))))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    contact = manifest["contact"]
    for box in (box for box in manifest["geometry"] if box["body"] == "carrier"):
        shape = UsdGeom.Cube.Define(stage, row["carrier_path"] + "/" + box["name"])
        shape.CreateSizeAttr(1.)
        shape.AddTranslateOp().Set(Gf.Vec3d(*box["center_m"]))
        shape.AddScaleOp().Set(Gf.Vec3d(*box["size_m"]))
        shape.CreateDisplayColorAttr([Gf.Vec3f(.12, .32, .65)])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
        collision = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        collision.CreateContactOffsetAttr(contact["contact_offset_m"])
        collision.CreateRestOffsetAttr(contact["rest_offset_m"])
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose="physics")
    joint = UsdPhysics.FixedJoint.Define(stage, row["joint_path"])
    joint.GetBody0Rel().SetTargets([row["chassis_path"]])
    joint.GetBody1Rel().SetTargets([row["carrier_path"]])
    for index in (0, 1):
        joint.GetPrim().GetAttribute(f"physics:localPos{index}").Set(Gf.Vec3f(0.))
        joint.GetPrim().GetAttribute(f"physics:localRot{index}").Set(Gf.Quatf(1.))
    joint.GetExcludeFromArticulationAttr().Set(True)
    joint.GetCollisionEnabledAttr().Set(False)
    body0 = [str(path) for path in joint.GetBody0Rel().GetTargets()]
    body1 = [str(path) for path in joint.GetBody1Rel().GetTargets()]
    local_positions = [[float(v) for v in joint.GetPrim().GetAttribute(f"physics:localPos{i}").Get()]
                       for i in (0, 1)]
    local_rotations = []
    for index in (0, 1):
        value = joint.GetPrim().GetAttribute(f"physics:localRot{index}").Get()
        local_rotations.append([float(value.GetReal()), *[float(v) for v in value.GetImaginary()]])
    observed_mass = float(mass.GetMassAttr().Get())
    observed_center = [float(v) for v in mass.GetCenterOfMassAttr().Get()]
    observed_inertia = [float(v) for v in mass.GetDiagonalInertiaAttr().Get()]
    axes = mass.GetPrincipalAxesAttr().Get()
    observed_axes = [float(axes.GetReal()), *[float(v) for v in axes.GetImaginary()]]
    expected_inertia = [float(inertia[i][i]) for i in range(3)]
    close = lambda actual, expected: len(actual) == len(expected) and all(
        math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-8) for a, b in zip(actual, expected))
    observed = {"robot_id": row["robot_id"], "carrier_path": row["carrier_path"],
        "joint_path": row["joint_path"], "body0": body0, "body1": body1,
        "carrier_dynamic_body": bool(rigid.GetRigidBodyEnabledAttr().Get()),
        "joint_enabled": bool(joint.GetJointEnabledAttr().Get()),
        "exclude_from_articulation": bool(joint.GetExcludeFromArticulationAttr().Get()),
        "connected_body_pair_collision_enabled": bool(joint.GetCollisionEnabledAttr().Get()),
        "local_positions_m": local_positions, "local_rotations_wxyz": local_rotations,
        "mass_kg": observed_mass, "center_of_mass_m": observed_center,
        "diagonal_inertia_kg_m2": observed_inertia, "principal_axes_wxyz": observed_axes,
        "compound_collider_count": len(row["collider_paths"])}
    if (body0 != [row["chassis_path"]] or body1 != [row["carrier_path"]]
            or observed["carrier_dynamic_body"] is not True or observed["joint_enabled"] is not True
            or observed["exclude_from_articulation"] is not True
            or observed["connected_body_pair_collision_enabled"] is not False
            or any(not close(value, [0., 0., 0.]) for value in local_positions)
            or any(not close(value, [1., 0., 0., 0.]) for value in local_rotations)
            or not math.isclose(observed_mass, float(props["mass_kg"]), rel_tol=1e-6, abs_tol=1e-8)
            or not close(observed_center, [float(v) for v in props["center_of_mass_m"]])
            or not close(observed_inertia, expected_inertia)
            or not close(observed_axes, [1., 0., 0., 0.])):
        raise ValueError("carrier rigid body, mass, inertia or fixed-joint readback differs")
    return observed


def install_carrier_fleet(
    stage: Any, prepared: Mapping[str, Any], *,
    _measure_robot: Callable[[Any, str, str], dict[str, Any]] | None = None,
    _author_carrier: Callable[[Any, Mapping[str, Any], Mapping[str, Any], Any, Any], dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Revalidate the complete prepared fleet before authoring any carrier prim."""
    if prepared.get("schema_version") != 1 or prepared.get("state") != "prepared":
        raise ValueError("fresh prepared carrier fleet required")
    current = prepare_carrier_fleet(stage, prepared["registry_definitions"], prepared["manifest"],
        prepared["run_id"], clearance_m=prepared["clearance_m"], _measure_robot=_measure_robot)
    if current["verification_sha256"] != prepared.get("verification_sha256"):
        raise ValueError("robot geometry, transforms, manifest or admissions changed after preparation")
    sdk = _sdk()
    author = _author_one if _author_carrier is None else _author_carrier
    UsdGeom, UsdPhysics, UsdShade = sdk.UsdGeom, sdk.UsdPhysics, sdk.UsdShade
    UsdGeom.Xform.Define(stage, _MODULE_ROOT)
    material = UsdShade.Material.Define(stage, _MATERIAL_PATH)
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    contact = prepared["manifest"]["contact"]
    material_api.CreateStaticFrictionAttr(contact["static_friction"])
    material_api.CreateDynamicFrictionAttr(contact["dynamic_friction"])
    material_api.CreateRestitutionAttr(contact["restitution"])
    attachments = [author(stage, row, prepared["manifest"], sdk, material)
                   for row in current["robots"]]
    return {"schema_version": 1, "state": "installed", "run_id": current["run_id"],
        "manifest_sha256": current["manifest_sha256"], "fleet_size": 4,
        "verification_sha256": current["verification_sha256"], "module_root_path": _MODULE_ROOT,
        "material_path": _MATERIAL_PATH, "robots": deepcopy(current["robots"]),
        "attachments": attachments, "carrier_physics_authored": True,
        "tote_physics_authored": False, "payload_physics_authored": False,
        "physical_loading_or_unloading_verified": False,
        "collision_masks_changed": False, "footprint_changed": False,
        "controller_or_sensor_policy_changed": False,
        "scope": "four chassis-fixed dynamic carrier modules; tote/payload remain visual models"}
