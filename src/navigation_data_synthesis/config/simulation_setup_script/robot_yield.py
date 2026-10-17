# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Let IRA patrols wait for robots and pedestrians using inexpensive footprint checks."""

import math
from collections.abc import Callable, Generator
from functools import partial

import omni.anim.behavior.core as bh
import omni.physx
from isaacsim.replicator.agent.core.character.runtime_state import PatrolRuntimeState
from omni.metropolis.pipeline.agent import Agent
from pxr import Sdf, Usd, UsdGeom


class RobotYield:
    """Keep patrol targets through slowdown, standing, and resuming near a robot.

    The settings prim selects a rigid body with ``robot_yield_body``. Its parent's
    visual bounds define the robot footprint. Clearance, stopping distance, and
    slowdown distance are configured in [m]. Robot poses come directly from PhysX.
    Optional ``pedestrian_yield_clearance`` [m] also checks the configured agents'
    live positions and enlarged body radii without enabling scene-wide avoidance.
    Optional ``robot_yield_min_walk_speed`` [m/s] provides a floor for slowed
    patrols, without exceeding the original patrol speed.
    """

    def __init__(self, stage: Usd.Stage, settings: Usd.Prim) -> None:
        """Read the robot footprint and optional waiting settings.

        Args:
            stage: Active simulation stage.
            settings: Prim carrying the robot relationship and distance settings.
        """
        self._stage = stage
        self._bindings: list[tuple[PatrolRuntimeState, Callable]] = []
        self._stationary_action = settings.GetAttribute("stationary_action").Get()
        self._pending: dict[Sdf.Path, tuple[list, list[str]]] = {}
        self._agents: dict[Sdf.Path, tuple[bh.IBehaviorAgent, float]] = {}
        self._pedestrian_clearance = settings.GetAttribute("pedestrian_yield_clearance").Get()
        targets = settings.GetRelationship("robot_yield_body").GetTargets()
        self._body_path = str(targets[0]) if targets else ""
        if not self._body_path:
            return
        body = stage.GetPrimAtPath(self._body_path)
        if not body:
            raise ValueError(f"Robot waiting body does not exist: {self._body_path}")
        unit = UsdGeom.GetStageMetersPerUnit(stage)
        minimum_walk_speed = settings.GetAttribute("robot_yield_min_walk_speed").Get()
        minimum_walk_speed = 0.0 if minimum_walk_speed is None else minimum_walk_speed
        if not math.isfinite(minimum_walk_speed) or minimum_walk_speed < 0.0:
            raise ValueError("robot_yield_min_walk_speed must be finite and nonnegative")
        self._minimum_walk_speed = minimum_walk_speed / unit
        if self._pedestrian_clearance is not None:
            if not math.isfinite(self._pedestrian_clearance) or self._pedestrian_clearance < 0.0:
                raise ValueError("pedestrian_yield_clearance must be finite and nonnegative")
            self._pedestrian_clearance /= unit
        self._standing_speed = 0.02 / unit
        self._clearance = settings.GetAttribute("robot_yield_clearance").Get() / unit
        self._stop_distance = settings.GetAttribute("robot_yield_stop_distance").Get() / unit
        self._slow_distance = settings.GetAttribute("robot_yield_slow_distance").Get() / unit
        self._hysteresis = 0.15 / unit
        if not self._clearance >= 0.0 or not 0.0 < self._stop_distance < self._slow_distance:
            raise ValueError("Robot yield distances must satisfy clearance >= 0 and 0 < stop < slow")
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bounds = cache.ComputeWorldBound(body.GetParent())
        bounds.Transform(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse())
        box = bounds.ComputeAlignedRange()
        self._low, self._high = box.GetMin(), box.GetMax()
        self._physics = omni.physx.get_physx_interface()

    def configure_agent(self, agent: Agent) -> None:
        """Attach waiting to this agent's patrols while preserving other behaviors.

        Args:
            agent: Initialized IRA runtime agent.
        """
        if not self._body_path:
            return
        native = bh.acquire_interface().get_agent(str(agent.prim_path))
        if native is None:
            return
        retained = []
        for state, original in self._bindings:
            if state.prim.GetPath().HasPrefix(agent.prim_path):
                state.perform_task = original
            else:
                retained.append((state, original))
        self._bindings = retained
        body = UsdGeom.Capsule.Get(self._stage, agent.prim_path.AppendPath("Physics/BodyCollider"))
        radius = body.GetRadiusAttr().Get()
        self._agents[agent.prim_path] = (native, radius)
        states = [*agent.routines, *(state for trigger in agent.triggers for state in trigger.behaviors)]
        for state in states:
            if isinstance(state, PatrolRuntimeState):
                original = state.perform_task
                self._bindings.append((state, original))
                state.perform_task = partial(self._perform_task, state, original, radius)
        for trigger in agent.triggers:
            for index, state in enumerate(trigger.behaviors):
                remaining = self._pending.pop(state.prim.GetPath(), None)
                if remaining is not None:
                    trigger.on_update = partial(self._resume_trigger, trigger, trigger.on_update, index, remaining)
                    # SDK 6.1 recreates runtime agents at Pause; fire the new trigger once.
                    trigger._trigger.trigger()

    def remember_patrols(self) -> None:
        """Remember remaining patrol waypoints before SDK runtime agents are replaced."""
        for state, _ in self._bindings:
            # Native action IDs may already be cleared when Pause is dispatched.
            if state.get_current_target() is not None and state.get_current_task_id() is not None:
                index = state._current_target_idx
                self._pending[state.prim.GetPath()] = (
                    list(state.path_points[index:]) if state.path_points else [],
                    list(state.target_prims[index:]) if state.target_prims else [],
                )

    def _resume_trigger(self, trigger, original, index, remaining, delta_time):
        state = trigger.behaviors[index]
        points, prims = state.path_points, state.target_prims
        state.path_points, state.target_prims = remaining
        try:
            for behavior in trigger.behaviors[index:]:
                result = behavior.on_update(delta_time)
                if isinstance(result, Generator):
                    yield from result
        finally:
            state.path_points, state.target_prims = points, prims
            trigger.on_update = original

    def close(self) -> None:
        """Release the adaptations when runtime agents are stopped or removed."""
        for state, original in self._bindings:
            state.perform_task = original
        self._bindings.clear()
        self._pending.clear()
        self._agents.clear()

    def _distance(self, native, radius: float, heading=None) -> float:
        distance = self._robot_distance(native, radius, heading)
        if self._pedestrian_clearance is None:
            return distance
        origin = native.get_world_translation()
        heading = native.get_facing_direction() if heading is None else heading
        length = math.hypot(heading[0], heading[1])
        if length < 1e-6:
            return distance
        direction = (heading[0] / length, heading[1] / length)
        for other, other_radius in self._agents.values():
            if other.get_prim_path() == native.get_prim_path():
                continue
            position = other.get_world_translation()
            if position[2] > origin[2] + native.get_height() or origin[2] > position[2] + other.get_height():
                continue
            dx, dy = position[0] - origin[0], position[1] - origin[1]
            along = dx * direction[0] + dy * direction[1]
            across = dx * direction[1] - dy * direction[0]
            padding = radius + other_radius + self._pedestrian_clearance
            if abs(across) >= padding:
                continue
            half_chord = math.sqrt(padding * padding - across * across)
            if along + half_chord >= 0.0:
                distance = min(distance, max(0.0, along - half_chord))
        return distance

    def _robot_distance(self, native, radius: float, heading=None) -> float:
        pose = self._physics.get_rigidbody_transformation(self._body_path)
        if not pose["ret_val"]:
            return math.inf
        position, rotation = pose["position"], pose["rotation"]
        origin = native.get_world_translation()
        if origin[2] + native.get_height() < position[2] + self._low[2] or origin[2] > position[2] + self._high[2]:
            return math.inf
        heading = native.get_facing_direction() if heading is None else heading
        yaw = math.atan2(
            2.0 * (rotation[3] * rotation[2] + rotation[0] * rotation[1]),
            1.0 - 2.0 * (rotation[1] ** 2 + rotation[2] ** 2),
        )
        cosine, sine = math.cos(yaw), math.sin(yaw)
        dx, dy = origin[0] - position[0], origin[1] - position[1]
        local = (cosine * dx + sine * dy, -sine * dx + cosine * dy)
        direction = (cosine * heading[0] + sine * heading[1], -sine * heading[0] + cosine * heading[1])
        enter, leave = 0.0, math.inf
        padding = radius + self._clearance
        for axis in range(2):
            low, high = self._low[axis] - padding, self._high[axis] + padding
            if abs(direction[axis]) < 1e-6:
                if not low <= local[axis] <= high:
                    return math.inf
            else:
                a, b = (low - local[axis]) / direction[axis], (high - local[axis]) / direction[axis]
                enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
        return enter if enter <= leave else math.inf

    def _idle(self, state, original, ready) -> Generator[None, None, None]:
        native = state.get_bh_agent()
        if self._stationary_action:
            # A custom animation does not cancel the previous navigation target.
            braking = original(task_fn=native.idle)
            next(braking, None)
            braking_id = state.get_current_task_id()
            try:
                yield
                while state.get_current_task_id() == braking_id:
                    velocity = native.get_linear_velocity(True)
                    if math.hypot(velocity[0], velocity[1]) <= self._standing_speed:
                        break
                    yield
            finally:
                if state.get_current_task_id() == braking_id:
                    native.cancel_task(braking_id)
                braking.close()
            if state.get_current_task_id() is None:
                return
        standing = partial(native.custom_action, self._stationary_action) if self._stationary_action else native.idle
        generator = original(task_fn=standing)
        next(generator, None)
        task_id = state.get_current_task_id()
        try:
            while state.get_current_task_id() is not None and not ready():
                yield
        finally:
            if state.get_current_task_id() == task_id:
                native.cancel_task(task_id)
            generator.close()

    def _perform_task(self, state, original, radius, task_fn, duration=-1.0, *args, **kwargs):
        native = state.get_bh_agent()
        speed = native.get_speed()
        slow_speed = min(speed, max(speed * 0.4, self._minimum_walk_speed))
        slow = False
        generator = None
        try:
            while True:
                distance = self._distance(native, radius)
                if distance < self._stop_distance:
                    heading = tuple(native.get_facing_direction())
                    yield from self._idle(
                        state,
                        original,
                        lambda: self._distance(native, radius, heading) > self._stop_distance + self._hysteresis,
                    )
                    if state.get_current_task_id() is None:
                        return
                    continue
                slow = distance < self._slow_distance + (self._hysteresis if slow else 0.0)
                native.set_speed(slow_speed if slow else speed)
                generator = original(task_fn=task_fn, duration=duration, *args, **kwargs)
                next(generator, None)
                task_id = state.get_current_task_id()
                if task_id is None or task_id == bh.BEHAVIOR_TASK_ID_INVALID:
                    return
                interrupted = False
                while task_id is not None and native.is_task_running(task_id):
                    distance = self._distance(native, radius)
                    new_slow = distance < self._slow_distance + (self._hysteresis if slow else 0.0)
                    if distance < self._stop_distance or new_slow != slow:
                        native.cancel_task(task_id)
                        interrupted = True
                        break
                    yield
                generator.close()
                generator = None
                if state.get_current_task_id() is None:
                    return
                if native.get_task_status(task_id) == bh.BehaviorTaskStatus.SUCCEEDED:
                    return
                if not interrupted:
                    # Keep IRA's handling of unrelated invalid/unreachable targets.
                    return
        finally:
            if generator is not None:
                generator.close()
            native.set_speed(speed)
