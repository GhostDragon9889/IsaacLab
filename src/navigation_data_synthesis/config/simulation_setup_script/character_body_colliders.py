# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configure IRA standing, body capsules and robot contacts from attached USD settings."""

import math
from collections.abc import Callable
from functools import partial

import carb.settings
import omni.anim.behavior.core as bh
import omni.kit.app
from isaacsim.replicator.agent.core.character.runtime_state import StopRuntimeState
from omni.behavior.scripting.core import BehaviorScript
from omni.metropolis.pipeline.agent import Agent, AgentsManager
from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics

from .character_initial_pose import InitialStandingPose
from .robot_visuals import RobotVisuals
from .robot_yield import RobotYield

_NAV_BLOCKED_TIMEOUT_SETTING = "/exts/omni.anim.behavior.core/navBlockedTimeout"


class CharacterBodyColliders(BehaviorScript):
    """Apply scene contact and navigation settings without altering source assets."""

    def on_init(self) -> None:
        """Remember original dimensions and settings while the layer is attached."""
        self._original_dimensions: dict[Sdf.Path, tuple[float, float]] = {}
        self._original_avoidance: dict[Sdf.Path, tuple[bool, bool]] = {}
        self._original_trigger_enabled: dict[Sdf.Path, bool] = {}
        self._original_nav_blocked_timeout: float | None = None
        self._standing_bindings: list[tuple[StopRuntimeState, Callable]] = []
        self._robot_visuals: RobotVisuals | None = None
        self._robot_yield: RobotYield | None = None
        self._runtime_agents: dict[Sdf.Path, Agent] = {}
        self._playing = False
        self._applying = False
        self._initial_pose = (
            InitialStandingPose(self.stage, self.prim) if self.prim.GetAttribute("stationary_action").Get() else None
        )
        self._preview_pending = self._initial_pose is not None
        self._preview_update = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._update_initial_pose, name="character_initial_standing_pose")
        )
        self._notice = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, self.stage)

    def on_play(self) -> None:
        """Apply capsule sizing and the optional blocked-navigation timeout [s]."""
        # Play also fires after Pause; keep the existing helpers and held targets.
        if self._playing:
            return
        self._preview_pending = False
        self._applying = True
        try:
            if self._initial_pose is not None:
                self._initial_pose.restore()
        finally:
            self._applying = False
        timeout = self.prim.GetAttribute("nav_blocked_timeout").Get()
        if timeout is not None:
            if timeout <= 0.0:
                raise ValueError("nav_blocked_timeout must be greater than 0 seconds")
            settings = carb.settings.get_settings()
            if self._original_nav_blocked_timeout is None:
                self._original_nav_blocked_timeout = settings.get(_NAV_BLOCKED_TIMEOUT_SETTING)
            settings.set(_NAV_BLOCKED_TIMEOUT_SETTING, timeout)
        self._playing = True
        self._original_avoidance.clear()
        self._robot_visuals = RobotVisuals(self.stage, self.prim)
        self._robot_yield = RobotYield(self.stage, self.prim)
        for root in self.prim.GetRelationship("character_root").GetTargets():
            prim = self.stage.GetPrimAtPath(root)
            if prim:
                for descendant in Usd.PrimRange.AllPrims(prim):
                    if descendant.GetParent().GetName() == "Physics":
                        if descendant.GetName() == "BodyCollider":
                            self._resize_capsule(descendant.GetPath())
                        elif descendant.GetName() == "AvoidanceTrigger":
                            self._configure_avoidance_trigger(descendant.GetPath())

    def on_update(self, current_time: float, delta_time: float) -> None:
        """Configure each runtime agent once after IRA initializes it.

        Args:
            current_time: Elapsed playback time [s].
            delta_time: Update interval [s].
        """
        roots = self.prim.GetRelationship("character_root").GetTargets()
        for agent in AgentsManager.get_instance().get_runtime_agent_instances():
            path = agent.prim_path
            if self._runtime_agents.get(path) is agent or not any(path.HasPrefix(root) for root in roots):
                continue
            native_agent = bh.acquire_interface().get_agent(str(path))
            if native_agent is None:
                continue
            self._resize_capsule(path.AppendPath("Physics/BodyCollider"))
            self._configure_avoidance_trigger(path.AppendPath("Physics/AvoidanceTrigger"))
            self._runtime_agents[path] = agent
            self._configure_standing(agent, native_agent)
            self._original_avoidance.setdefault(
                path,
                (native_agent.is_obstacle_avoidance_enabled(), native_agent.is_auto_avoidance_enabled()),
            )
            obstacle_avoidance = self.prim.GetAttribute("obstacle_avoidance_enabled").Get()
            auto_avoidance = self.prim.GetAttribute("auto_avoidance_enabled").Get()
            if obstacle_avoidance is not None:
                native_agent.set_obstacle_avoidance_enabled(obstacle_avoidance)
            if auto_avoidance is not None:
                native_agent.set_auto_avoidance_enabled(auto_avoidance)
            if self._robot_yield is not None:
                self._robot_yield.configure_agent(agent)

    def on_pause(self) -> None:
        """Preserve active patrol progress across SDK runtime-agent replacement."""
        if self._robot_yield is not None:
            self._robot_yield.remember_patrols()

    def on_stop(self) -> None:
        """Restore navigation settings and keep capsule sizes for the next playback."""
        self._playing = False
        self._close_robot_helpers()
        self._original_avoidance.clear()
        self._runtime_agents.clear()
        self._restore_navigation_timeout()
        self._preview_pending = self._initial_pose is not None

    def on_destroy(self) -> None:
        """Restore modified settings when the settings layer is removed."""
        self._restore_navigation_timeout()
        self._close_robot_helpers()
        self._notice.Revoke()
        self._preview_update.unsubscribe()
        if self._initial_pose is not None:
            self._initial_pose.restore()
        if self._playing:
            for path, (obstacle_avoidance, auto_avoidance) in self._original_avoidance.items():
                agent = bh.acquire_interface().get_agent(str(path))
                if agent is not None:
                    agent.set_obstacle_avoidance_enabled(obstacle_avoidance)
                    agent.set_auto_avoidance_enabled(auto_avoidance)
        self._original_avoidance.clear()
        self._runtime_agents.clear()
        self._restore_dimensions()
        if self.stage:
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                for path, enabled in self._original_trigger_enabled.items():
                    prim = self.stage.GetPrimAtPath(path)
                    if prim:
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(enabled)
        self._original_trigger_enabled.clear()

    def _close_robot_helpers(self) -> None:
        for state, original in self._standing_bindings:
            state.perform_task = original
        self._standing_bindings.clear()
        if self._robot_yield is not None:
            self._robot_yield.close()
            self._robot_yield = None
        if self._robot_visuals is not None:
            self._robot_visuals.close()
            self._robot_visuals = None

    def _configure_standing(self, agent: Agent, native: bh.IBehaviorAgent) -> None:
        action = self.prim.GetAttribute("stationary_action").Get()
        if not action:
            return
        if action not in native.get_custom_action_names():
            raise ValueError(f"Stationary action {action!r} is missing from the character motion library")
        retained = []
        for state, original in self._standing_bindings:
            if state.prim.GetPath().HasPrefix(agent.prim_path):
                state.perform_task = original
            else:
                retained.append((state, original))
        self._standing_bindings = retained
        states = [*agent.routines, *(state for trigger in agent.triggers for state in trigger.behaviors)]
        for state in states:
            if isinstance(state, StopRuntimeState):
                original = state.perform_task
                self._standing_bindings.append((state, original))
                state.perform_task = partial(self._perform_standing, original, partial(native.custom_action, action))

    @staticmethod
    def _perform_standing(original, standing, task_fn, duration=-1.0, *args, **kwargs):
        return original(task_fn=standing, duration=duration, *args, **kwargs)

    def _restore_navigation_timeout(self) -> None:
        if self._original_nav_blocked_timeout is not None:
            carb.settings.get_settings().set(_NAV_BLOCKED_TIMEOUT_SETTING, self._original_nav_blocked_timeout)
            self._original_nav_blocked_timeout = None

    def _configure_avoidance_trigger(self, path: Sdf.Path) -> None:
        roots = self.prim.GetRelationship("character_root").GetTargets()
        if not any(path.HasPrefix(root) for root in roots):
            return
        prim = self.stage.GetPrimAtPath(path)
        if not prim:
            return
        attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
        if not attr or not attr.HasAuthoredValueOpinion():
            return
        disable = all(
            self.prim.GetAttribute(name).Get() is False
            for name in ("obstacle_avoidance_enabled", "auto_avoidance_enabled")
        )
        if disable:
            enabled = False
            self._original_trigger_enabled.setdefault(path, attr.Get())
        else:
            enabled = self._original_trigger_enabled.get(path, attr.Get())
        if attr.Get() != enabled:
            self._applying = True
            try:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    attr.Set(enabled)
            finally:
                self._applying = False

    def _resize_capsule(self, path: Sdf.Path) -> None:
        if self._applying:
            return
        radius_scale = self.prim.GetAttribute("body_radius_scale").Get()
        if radius_scale is None or not math.isfinite(radius_scale) or radius_scale <= 0.0:
            raise ValueError("body_radius_scale must be finite and greater than 0")
        roots = self.prim.GetRelationship("character_root").GetTargets()
        if not any(path.HasPrefix(root) for root in roots):
            return
        capsule = UsdGeom.Capsule.Get(self.stage, path)
        if not capsule or not all(
            attr.HasAuthoredValueOpinion() for attr in (capsule.GetRadiusAttr(), capsule.GetHeightAttr())
        ):
            return
        dimensions = (capsule.GetRadiusAttr().Get(), capsule.GetHeightAttr().Get())
        original_radius, original_height = self._original_dimensions.setdefault(path, dimensions)
        radius = original_radius * radius_scale
        # Capsule height excludes its hemispheres; keep the top and bottom fixed.
        height = original_height + 2.0 * (original_radius - radius)
        if height < 0.0:
            raise ValueError("body_radius_scale must keep the radius at most half the total capsule height")
        if dimensions != (radius, height):
            self._applying = True
            try:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    capsule.GetRadiusAttr().Set(radius)
                    capsule.GetHeightAttr().Set(height)
            finally:
                self._applying = False

    def _on_objects_changed(self, notice: Usd.Notice.ObjectsChanged, sender: Usd.Stage) -> None:
        if not self._playing and not self._applying and self._initial_pose is not None:
            roots = self.prim.GetRelationship("character_root").GetTargets()
            if any(
                path.HasPrefix(root) or root.HasPrefix(path)
                for path in (*notice.GetResyncedPaths(), *notice.GetChangedInfoOnlyPaths())
                for root in roots
            ):
                self._preview_pending = True
        if self._playing and not self._applying:
            for path in (*notice.GetResyncedPaths(), *notice.GetChangedInfoOnlyPaths()):
                prim_path = path.GetPrimPath()
                if prim_path.GetParentPath().name == "Physics":
                    if prim_path.name == "BodyCollider":
                        self._resize_capsule(prim_path)
                    elif prim_path.name == "AvoidanceTrigger":
                        self._configure_avoidance_trigger(prim_path)

    def _update_initial_pose(self, event) -> None:
        if self._playing or not self._preview_pending or self._initial_pose is None:
            return
        self._preview_pending = False
        self._applying = True
        try:
            self._initial_pose.apply()
        finally:
            self._applying = False

    def _restore_dimensions(self) -> None:
        if self.stage:
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                for path, (radius, height) in self._original_dimensions.items():
                    capsule = UsdGeom.Capsule.Get(self.stage, path)
                    if capsule:
                        capsule.GetRadiusAttr().Set(radius)
                        capsule.GetHeightAttr().Set(height)
        self._original_dimensions.clear()
