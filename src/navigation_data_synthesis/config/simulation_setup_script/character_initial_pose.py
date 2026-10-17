# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Show arms-down character previews while the simulation is stopped."""

import math

import BehaviorSchema
import RetargetingSchema

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel


class InitialStandingPose:
    """Temporarily bind a preview animation without changing the source skeleton."""

    def __init__(self, stage: Usd.Stage, settings: Usd.Prim) -> None:
        """Read character roots and reserve a session-only animation scope.

        Args:
            stage: Active simulation stage.
            settings: Prim containing the character root relationship.
        """
        self._stage = stage
        self._roots = settings.GetRelationship("character_root").GetTargets()
        self._scope = settings.GetPath().AppendChild("InitialStandingPoses")
        self._bindings: list[Sdf.Path] = []
        self._backup = Sdf.Layer.CreateAnonymous()

    def apply(self) -> None:
        """Display lowered arms for loaded Human rigs while playback is stopped."""
        for root in self._roots:
            prim = self._stage.GetPrimAtPath(root)
            if not prim:
                continue
            for descendant in Usd.PrimRange(prim):
                if not descendant.IsA(UsdSkel.Skeleton) or not descendant.HasAPI(RetargetingSchema.ControlRigAPI):
                    continue
                owner = descendant.GetParent()
                while owner and not owner.IsA(UsdSkel.Root):
                    owner = owner.GetParent()
                if not owner or not owner.HasAPI(BehaviorSchema.BehaviorAgentAPI):
                    continue
                skeleton = UsdSkel.Skeleton(descendant)
                binding_path = descendant.GetPath().AppendProperty("skel:animationSource")
                if binding_path in self._bindings:
                    continue
                tags = RetargetingSchema.ControlRigAPI(descendant).GetRetargetTagsAttr().Get()
                indices = {str(tag): index for index, tag in enumerate(tags or []) if tag}
                if not all(side + part in indices for side in ("Left", "Right") for part in ("Arm", "Hand")):
                    continue
                local = skeleton.GetRestTransformsAttr().Get()
                joints = skeleton.GetJointsAttr().Get()
                if not local or len(local) != len(joints):
                    continue
                topology = UsdSkel.Topology(joints)
                transforms = UsdSkel.ConcatJointTransforms(topology, local)
                up = Gf.Vec3d(0, 0, 1) if UsdGeom.GetStageUpAxis(self._stage) == UsdGeom.Tokens.z else Gf.Vec3d(0, 1, 0)
                world = UsdGeom.Xformable(skeleton).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                down = world.GetInverse().TransformDir(-up).GetNormalized()
                for side in ("Left", "Right"):
                    arm, hand = indices[side + "Arm"], indices[side + "Hand"]
                    direction = transforms[hand].ExtractTranslation() - transforms[arm].ExtractTranslation()
                    outward = (direction - down * Gf.Dot(direction, down)).GetNormalized()
                    # Leave ten degrees of clearance between the hands and torso.
                    target = down * math.cos(math.radians(10)) + outward * math.sin(math.radians(10))
                    posed = transforms[arm] * Gf.Matrix4d().SetRotate(Gf.Rotation(direction, target))
                    posed.SetTranslateOnly(transforms[arm].ExtractTranslation())
                    local[arm] = posed * transforms[topology.GetParentIndices()[arm]].GetInverse()
                self._bind(skeleton, binding_path, local)

    def _bind(self, skeleton: UsdSkel.Skeleton, path: Sdf.Path, transforms) -> None:
        session = self._stage.GetSessionLayer()
        if session.GetPropertyAtPath(path):
            Sdf.CreatePrimInLayer(self._backup, path.GetPrimPath())
            Sdf.CopySpec(session, path, self._backup, path)
        with Usd.EditContext(self._stage, session):
            animation = UsdSkel.Animation.Define(self._stage, self._scope.AppendChild(f"pose_{len(self._bindings)}"))
            translations, rotations, scales = UsdSkel.DecomposeTransforms(transforms)
            animation.CreateJointsAttr(skeleton.GetJointsAttr().Get())
            animation.CreateTranslationsAttr(translations)
            animation.CreateRotationsAttr(rotations)
            animation.CreateScalesAttr(scales)
            UsdSkel.BindingAPI(skeleton).CreateAnimationSourceRel().SetTargets([animation.GetPath()])
        self._bindings.append(path)

    def restore(self) -> None:
        """Remove the preview and restore any prior session animation bindings."""
        if self._stage:
            session = self._stage.GetSessionLayer()
            with Usd.EditContext(self._stage, session), Sdf.ChangeBlock():
                for path in self._bindings:
                    prim = self._stage.GetPrimAtPath(path.GetPrimPath())
                    if prim:
                        prim.RemoveProperty(path.name)
                        if self._backup.GetPropertyAtPath(path):
                            Sdf.CopySpec(self._backup, path, session, path)
                self._stage.RemovePrim(self._scope)
        self._bindings.clear()
        self._backup = Sdf.Layer.CreateAnonymous()
