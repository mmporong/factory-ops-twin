"""Focused cold-USD regression for render-only context preparation."""
import os
from pathlib import Path
import unittest

from scripts.assets.prepare_factory_context import (
    load_usd, physics_typed_paths, remove_physics_typed_prims,
)


@unittest.skipUnless(os.environ.get("ISAAC_SIM_SDK"), "requires existing Isaac Sim SDK")
class FactoryContextPreparationTests(unittest.TestCase):
    def test_typed_joint_is_removed_without_changing_visible_mesh_bounds(self):
        Usd, UsdGeom, UsdPhysics, _Sdf, Gf, _handles = load_usd(
            Path(os.environ["ISAAC_SIM_SDK"]))
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
        stage.SetDefaultPrim(root)
        mesh = UsdGeom.Mesh.Define(stage, "/Asset/Visible")
        mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(2, 0, 0),
                               Gf.Vec3f(0, 3, 0), Gf.Vec3f(0, 0, 4)])
        mesh.CreateFaceVertexCountsAttr([3, 3, 3, 3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2])
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Asset/Joints/Door")
        self.assertEqual("PhysicsRevoluteJoint", joint.GetPrim().GetTypeName())

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        before = cache.ComputeWorldBound(root).ComputeAlignedBox()
        removed = remove_physics_typed_prims(stage, UsdPhysics)
        after = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(root).ComputeAlignedBox()

        self.assertEqual(["/Asset/Joints/Door"], removed)
        self.assertEqual([], physics_typed_paths(stage, UsdPhysics))
        self.assertTrue(stage.GetPrimAtPath("/Asset/Visible").IsValid())
        self.assertEqual(tuple(before.GetMin()), tuple(after.GetMin()))
        self.assertEqual(tuple(before.GetMax()), tuple(after.GetMax()))


if __name__ == "__main__":
    unittest.main()
