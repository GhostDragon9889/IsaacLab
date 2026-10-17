# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Keep detailed robot rendering out of IRA's rigid-body obstacle geometry."""

from dataclasses import dataclass

import omni.physx
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


@dataclass
class _Visual:
    source: Sdf.Path
    session_type: str | None
    body: str
    local_transform: Gf.Matrix4d
    transform: UsdGeom.XformOp


class RobotVisuals:
    """Render pure visual meshes outside rigid bodies, following each physics step.

    Only the session layer is edited. Geometry and materials are referenced from
    their original prims; collision shapes, joints, and link paths stay in place.
    """

    def __init__(self, stage: Usd.Stage, settings: Usd.Prim) -> None:
        """Apply the optional optimization to the robot selected by the settings.

        Args:
            stage: Active simulation stage.
            settings: Prim carrying ``separate_robot_visuals`` and ``robot_yield_body``.
        """
        self._stage = stage
        self._visuals: list[_Visual] = []
        self._root: Sdf.Path | None = None
        self._subscription = None
        targets = settings.GetRelationship("robot_yield_body").GetTargets()
        if not settings.GetAttribute("separate_robot_visuals").Get() or not targets:
            return
        body = stage.GetPrimAtPath(targets[0])
        if not body or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"Robot visual settings require a rigid body: {targets[0]}")
        root = body.GetParent()
        ancestor = root
        while ancestor and not ancestor.IsPseudoRoot():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError("Robot rendering root must be outside rigid bodies")
            ancestor = ancestor.GetParent()
        path = root.GetPath().AppendChild("RenderVisuals")
        suffix = 1
        while stage.GetPrimAtPath(path):
            path = root.GetPath().AppendChild(f"RenderVisuals_{suffix}")
            suffix += 1
        self._root = path
        cache = UsdGeom.XformCache()
        meshes = [prim for prim in Usd.PrimRange(root) if prim.IsA(UsdGeom.Mesh)]
        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                UsdGeom.Scope.Define(stage, path)
                for mesh in meshes:
                    # Never move collision geometry or a subtree containing physics.
                    if any(
                        prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI)
                        for prim in Usd.PrimRange(mesh)
                    ):
                        continue
                    parent = mesh.GetParent()
                    while parent != root and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
                        if (
                            parent.HasAPI(UsdPhysics.CollisionAPI)
                            and UsdPhysics.CollisionAPI(parent).GetCollisionEnabledAttr().Get()
                        ):
                            break
                        parent = parent.GetParent()
                    if parent == root or (
                        parent.HasAPI(UsdPhysics.CollisionAPI)
                        and UsdPhysics.CollisionAPI(parent).GetCollisionEnabledAttr().Get()
                    ):
                        continue
                    local = cache.GetLocalToWorldTransform(mesh) * cache.GetLocalToWorldTransform(parent).GetInverse()
                    bindings = []
                    for part in Usd.PrimRange(mesh):
                        material = UsdShade.MaterialBindingAPI(part).ComputeBoundMaterial()[0]
                        if material:
                            bindings.append((part.GetPath().MakeRelativePath(mesh.GetPath()), material))
                    visibility = UsdGeom.Imageable(mesh).ComputeVisibility()
                    purpose = UsdGeom.Imageable(mesh).ComputePurpose()
                    session_spec = stage.GetSessionLayer().GetPrimAtPath(mesh.GetPath())
                    session_type = session_spec.typeName if session_spec and session_spec.HasInfo("typeName") else None
                    copy = stage.DefinePrim(path.AppendChild(f"mesh_{len(self._visuals)}"), "Mesh")
                    copy.GetReferences().AddInternalReference(mesh.GetPath())
                    operation = UsdGeom.Xformable(copy).MakeMatrixXform()
                    UsdGeom.Xformable(copy).SetResetXformStack(True)
                    operation.Set(local * cache.GetLocalToWorldTransform(parent))
                    self._visuals.append(_Visual(mesh.GetPath(), session_type, str(parent.GetPath()), local, operation))
                    mesh.SetTypeName("Xform")
                    UsdGeom.Imageable(copy).CreateVisibilityAttr(visibility)
                    UsdGeom.Imageable(copy).CreatePurposeAttr(purpose)
                    for relative, material in bindings:
                        target = (
                            copy
                            if relative == Sdf.Path.reflexiveRelativePath
                            else stage.GetPrimAtPath(copy.GetPath().AppendPath(relative))
                        )
                        UsdShade.MaterialBindingAPI.Apply(target).Bind(material)
            self._physics = omni.physx.get_physx_interface()
            self._subscription = self._physics.subscribe_physics_on_step_events(self._on_physics_step, False, 0)
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        """Restore original visual prims and remove temporary rendering copies."""
        if self._subscription is not None:
            self._subscription.unsubscribe()
            self._subscription = None
        if self._stage:
            layer = self._stage.GetSessionLayer()
            with Usd.EditContext(self._stage, layer):
                for visual in self._visuals:
                    spec = layer.GetPrimAtPath(visual.source)
                    if spec:
                        if visual.session_type is None:
                            spec.ClearInfo("typeName")
                        else:
                            spec.typeName = visual.session_type
                if self._root is not None:
                    self._stage.RemovePrim(self._root)
        self._visuals.clear()
        self._root = None

    def _on_physics_step(self, delta_time: float) -> None:
        transforms = {}
        # Publish a complete pose once per step instead of notifying for every mesh.
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()), Sdf.ChangeBlock():
            for visual in self._visuals:
                if visual.body not in transforms:
                    pose = self._physics.get_rigidbody_transformation(visual.body)
                    if not pose["ret_val"]:
                        continue
                    rotation = pose["rotation"]
                    transform = Gf.Transform()
                    transform.SetTranslation(Gf.Vec3d(*pose["position"]))
                    transform.SetRotation(
                        Gf.Rotation(Gf.Quatd(rotation[3], Gf.Vec3d(*[rotation[i] for i in range(3)])))
                    )
                    transforms[visual.body] = transform.GetMatrix()
                visual.transform.Set(visual.local_transform * transforms[visual.body])
