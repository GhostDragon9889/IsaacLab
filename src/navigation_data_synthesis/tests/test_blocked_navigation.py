# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Check that case_1 pedestrians resume after the real robot clears their target.

Run with ``uv run python src/navigation_data_synthesis/tests/test_blocked_navigation.py``.
This standalone integration test requires the project's local Isaac Sim assets and a GPU.
"""

import argparse
import asyncio
import json
import math
import time
import traceback
from pathlib import Path


def _record_robot_visuals(robot_root):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    meshes = [
        prim
        for prim in Usd.PrimRange(robot_root)
        if prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    cache = UsdGeom.XformCache()
    records = []
    for mesh in meshes:
        body = mesh.GetParent()
        while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if body:
            records.append(
                (
                    mesh.GetPath(),
                    str(body.GetPath()),
                    cache.GetLocalToWorldTransform(mesh) * cache.GetLocalToWorldTransform(body).GetInverse(),
                    UsdGeom.Mesh(mesh).GetFaceVertexCountsAttr().Get(),
                    UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()[0].GetPath(),
                )
            )
    return meshes, records


def _isolate_character_group(stage, selected: str, groups: dict) -> None:
    from pxr import Usd

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for name in groups:
            if name != selected:
                stage.GetPrimAtPath("/World/Characters/" + name).SetActive(False)


def _move_robot(robot, position: list[float], yaw: float | None = None) -> None:
    """Set the robot root position [m] and optional yaw [rad]."""
    import numpy as np

    transform = robot.get_root_transforms().copy()
    transform[0, :2] = position
    if yaw is not None:
        transform[0, 3:7] = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
    indices = np.array([0], dtype=np.uint32)
    robot.set_root_transforms(transform, indices)
    robot.set_root_velocities(np.zeros((1, 6), dtype=np.float32), indices)


def main() -> None:
    """Run the two blocked-path regressions and preserve failures at Kit shutdown."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("logs/blocked_navigation"))
    parser.add_argument("--experience", default="", help="Optional Kit experience path for matching the GUI runtime.")
    parser.add_argument("--groups", nargs="+", default=["walker_02", "walker_03"])
    parser.add_argument("--block-seconds", type=float, default=15.0, help="Time before clearing the robot [s].")
    parser.add_argument("--stepping", choices=("fixed", "variable"), default="fixed")
    parser.add_argument("--motion", choices=("park", "cross", "side"), default="park")
    parser.add_argument("--resume-seconds", type=float, default=8.0, help="Playback after robot clearance [s].")
    parser.add_argument("--min-fps", type=float, default=0.0, help="Minimum measured walking FPS, or zero to omit.")
    parser.add_argument("--expect-yield", action="store_true", help="Require standing with the patrol target retained.")
    parser.add_argument(
        "--isolate-group", action="store_true", help="Disable other groups to test direct robot waiting."
    )
    parser.add_argument("--pause-before-clear", action="store_true", help="Pause and resume before clearing the robot.")
    args, _ = parser.parse_known_args()
    args.output.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "width": 640,
            "height": 360,
            "limit_cpu_threads": 8,
            "multi_gpu": False,
            "extra_args": ["--/persistent/renderer/startupMessageDisplayed=true"],
        },
        experience=args.experience,
    )
    import yaml

    import carb.eventdispatcher
    import carb.settings
    import omni.physics.tensors as tensors
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.replicator.agent.core")
    app.update()
    from test_character_contact_boundary import _measure_character_boundary

    import omni.anim.behavior.core as bh
    from isaacsim.replicator.agent.core import api as ira
    from omni.metropolis.pipeline.agent import AgentsManager
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    config = Path(__file__).resolve().parents[1] / "config/third_floor/third_floor_case_1.yaml"
    groups = yaml.safe_load(config.read_text())["isaacsim.replicator.agent"]["character"]["groups"]
    timeline = omni.timeline.get_timeline_interface()
    settings = carb.settings.get_settings()
    timeout_key = "/exts/omni.anim.behavior.core/navBlockedTimeout"
    original_timeout = settings.get(timeout_key)
    report = {"runs": [], "passed": False}
    task_results = []
    robot = None
    frame_times = []
    waiting_frames = []
    waiting_distances = []
    target_distances = []

    def task_ended(event):
        native = bh.acquire_interface().get_agent(str(event["agent"]))
        if native and native.get_task_name(event["task_id"]) == "MoveTo":
            task_results.append(
                {
                    "agent": str(event["agent"]),
                    "status": int(event["task_status"]),
                    "error": int(event["task_error"]),
                    "time": timeline.get_current_time(),
                }
            )

    subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
        observer_name="blocked_navigation_regression",
        event_name=bh.GLOBAL_EVENT_BEHAVIOR_TASK_ENDED,
        on_event=task_ended,
    )

    def advance_until(target_time):
        deadline = time.monotonic() + 240
        while timeline.get_current_time() < target_time:
            assert time.monotonic() < deadline, "Playback timed out"
            started = time.perf_counter()
            app.update()
            current_time = timeline.get_current_time()
            frame_times.append((current_time, time.perf_counter() - started))
            arrival_start = args.block_seconds - 4.0 if args.motion == "cross" else args.block_seconds
            if current_time >= arrival_start:
                for instance in AgentsManager.get_instance().get_runtime_agent_instances():
                    if instance.group == group:
                        target_distances.append(math.dist(instance.get_world_position(), target))
            motion_start = args.block_seconds - 4.0
            if robot is not None and args.motion == "cross" and motion_start <= current_time < args.block_seconds:
                fraction = min(1.0, (current_time - motion_start) / 4.0)
                _move_robot(robot, [target[0], target[1] + 0.3 - 1.9 * fraction])
            if args.expect_yield and 2.0 < current_time < args.block_seconds:
                from isaacsim.replicator.agent.core.character.runtime_state import PatrolRuntimeState

                for instance in AgentsManager.get_instance().get_runtime_agent_instances():
                    standing_name = collider_settings.GetAttribute("stationary_action").Get() or "Idle"
                    if instance.group != group or instance.get_current_task_name() not in ("Idle", standing_name):
                        continue
                    states = [*instance.routines, *(s for trigger in instance.triggers for s in trigger.behaviors)]
                    if any(
                        isinstance(s, PatrolRuntimeState)
                        and s.get_current_target() is not None
                        and s.get_current_task_id() == instance.get_bh_agent().get_action_task_id()
                        for s in states
                    ):
                        waiting_frames.append(current_time)
                        waiting_distances.append(math.dist(instance.get_world_position(), target))

    try:
        for group in args.groups:
            print(f"BLOCKED_NAVIGATION_START {group}", flush=True)
            task_results.clear()
            frame_times.clear()
            waiting_frames.clear()
            waiting_distances.clear()
            target_distances.clear()
            target = groups[group]["triggers"][0]["time_trigger"]["behavior"][0]["patrol"]["path_points"][-1]
            assert ira.load_config_file(str(config)), "IRA rejected case_1"
            future = asyncio.ensure_future(ira.setup_simulation())
            deadline = time.monotonic() + 240
            while not future.done():
                assert time.monotonic() < deadline, "Scene setup timed out"
                app.update()
            future.result()
            stage = omni.usd.get_context().get_stage()
            collider_settings = stage.GetPrimAtPath("/World/CharacterBodyColliderSettings")
            robot_root = stage.GetPrimAtPath("/World/EVT_EXP")
            original_meshes, visual_records = _record_robot_visuals(robot_root)
            original_collisions = [
                prim.GetPath() for prim in Usd.PrimRange(robot_root) if prim.HasAPI(UsdPhysics.CollisionAPI)
            ]
            original_bodies = [
                prim.GetPath() for prim in Usd.PrimRange(robot_root) if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ]
            if args.isolate_group:
                _isolate_character_group(stage, group, groups)
            settings.set("/app/player/useFixedTimeStepping", args.stepping == "fixed")
            settings.set("/app/runLoops/main/rateLimitEnabled", False)
            timeline.set_current_time(0)
            timeline.play()
            timeline.commit()
            advance_until(0.5)
            physics = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
            physics.set_subspace_roots("/")
            robot = physics.create_articulation_view("/World/EVT_EXP/base_link")
            assert robot.count == 1, "Expected the scene's EVT robot"
            if args.motion == "side":
                _move_robot(robot, [0.3, 0.4], yaw=0.0)
            elif group == "walker_03":
                _move_robot(robot, [target[0], target[1] + 0.3])
            advance_until(args.block_seconds)
            agent_ids_before_pause = [
                id(instance) for instance in AgentsManager.get_instance().get_runtime_agent_instances()
            ]
            if args.pause_before_clear:
                timeline.pause()
                timeline.commit()
                for _ in range(3):
                    app.update()
                timeline.play()
                timeline.commit()
                advance_until(args.block_seconds + 0.1)
            agents = {agent.group: agent for agent in AgentsManager.get_instance().get_runtime_agent_instances()}
            agent = agents[group]
            native = agent.get_bh_agent()
            blocked_position = list(agent.get_world_position())
            run = {
                "group": group,
                "isolated": args.isolate_group,
                "agents_before_clear": {
                    name: {"position_m": list(instance.get_world_position()), "task": instance.get_current_task_name()}
                    for name, instance in agents.items()
                },
                "blocked_position_m": blocked_position,
                "task_before_clear": agent.get_current_task_name(),
                "timeout_during_play_s": settings.get(timeout_key),
                "agents_recreated_by_pause": agent_ids_before_pause
                != [id(instance) for instance in AgentsManager.get_instance().get_runtime_agent_instances()],
                "checks": {
                    "really_blocked": math.dist(blocked_position, target) > 0.2
                    or (args.motion == "cross" and max(waiting_distances, default=0.0) > 0.2),
                    "configured_timeout_is_active": settings.get(timeout_key)
                    == collider_settings.GetAttribute("nav_blocked_timeout").Get(),
                },
            }
            run["contact_boundary"] = _measure_character_boundary(agent)
            run["checks"].update(run["contact_boundary"]["checks"])
            if args.pause_before_clear:
                run["checks"]["pause_did_not_duplicate_rendering"] = (
                    sum(prim.GetName().startswith("RenderVisuals") for prim in robot_root.GetChildren()) == 1
                )
                run["checks"]["pause_did_not_duplicate_agents"] = len(agents) == len(
                    AgentsManager.get_instance().get_runtime_agent_instances()
                )
            if collider_settings.GetAttribute("separate_robot_visuals").Get():
                copies = list(stage.GetPrimAtPath("/World/EVT_EXP/RenderVisuals").GetChildren())
                visual_errors = []
                geometry_preserved = len(copies) == len(visual_records)
                materials_preserved = geometry_preserved
                for copy, (source, body_path, local, faces, material) in zip(copies, visual_records):
                    pose = omni.physx.get_physx_interface().get_rigidbody_transformation(body_path)
                    rotation = pose["rotation"]
                    transform = Gf.Transform()
                    transform.SetTranslation(Gf.Vec3d(*pose["position"]))
                    transform.SetRotation(
                        Gf.Rotation(Gf.Quatd(rotation[3], Gf.Vec3d(*[rotation[i] for i in range(3)])))
                    )
                    expected = local * transform.GetMatrix()
                    actual = UsdGeom.Xformable(copy).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    visual_errors.append(max(abs(expected[i][j] - actual[i][j]) for i in range(4) for j in range(4)))
                    geometry_preserved &= UsdGeom.Mesh(copy).GetFaceVertexCountsAttr().Get() == faces
                    geometry_preserved &= not stage.GetPrimAtPath(source).IsA(UsdGeom.Mesh)
                    materials_preserved &= (
                        UsdShade.MaterialBindingAPI(copy).ComputeBoundMaterial()[0].GetPath() == material
                    )
                run["render_meshes"] = len(copies)
                run["visual_transform_max_error"] = max(visual_errors, default=math.inf)
                run["checks"]["visual_geometry_preserved"] = geometry_preserved
                run["checks"]["visual_materials_preserved"] = materials_preserved
                run["checks"]["visuals_follow_physics"] = run["visual_transform_max_error"] < 1e-4
                run["checks"]["robot_collisions_preserved"] = original_collisions == [
                    prim.GetPath() for prim in Usd.PrimRange(robot_root) if prim.HasAPI(UsdPhysics.CollisionAPI)
                ]
                run["checks"]["robot_rigid_bodies_preserved"] = original_bodies == [
                    prim.GetPath() for prim in Usd.PrimRange(robot_root) if prim.HasAPI(UsdPhysics.RigidBodyAPI)
                ]
            if args.motion == "side":
                _move_robot(robot, [0.65, -0.45], yaw=math.pi / 2.0)
            else:
                _move_robot(robot, [5.0, 5.0])
            advance_until(args.block_seconds + args.resume_seconds)
            run["robot_position_after_clear_m"] = list(
                omni.physx.get_physx_interface().get_rigidbody_transformation("/World/EVT_EXP/base_link")["position"]
            )
            run["final_position_m"] = list(agent.get_world_position())
            run["target_distance_m"] = math.dist(run["final_position_m"], target)
            run["closest_target_distance_m"] = min(target_distances, default=math.inf)
            results = [result["status"] for result in task_results if result["agent"] == str(agent.prim_path)]
            run["move_to_statuses"] = results
            run["checks"]["resumed_and_reached_target"] = run["closest_target_distance_m"] < 0.1
            run["checks"]["move_to_succeeded"] = int(bh.BehaviorTaskStatus.SUCCEEDED) in results
            run["checks"]["move_to_not_abandoned"] = not any(
                result["status"] == int(bh.BehaviorTaskStatus.FAILED)
                and result["error"] != int(bh.BehaviorTaskError.CANCELED)
                and result["time"] > 2.0
                and result["agent"] == str(agent.prim_path)
                for result in task_results
            )
            walking_times = [elapsed for instant, elapsed in frame_times if 1.0 <= instant < 6.0]
            run["walking_fps"] = len(walking_times) / sum(walking_times)
            if args.min_fps:
                run["checks"]["walking_fps_budget"] = run["walking_fps"] >= args.min_fps
            if args.expect_yield:
                run["waiting_frames"] = len(waiting_frames)
                run["waiting_duration_s"] = len(set(round(instant * 60) for instant in waiting_frames)) / 60.0
                run["checks"]["wait_preserves_patrol_target"] = run["waiting_duration_s"] > 1.0
            for name in ("obstacle_avoidance", "auto_avoidance"):
                run["checks"][name + "_preserved"] = getattr(native, "is_" + name + "_enabled")() == (
                    collider_settings.GetAttribute(name + "_enabled").Get()
                )
            body = UsdGeom.Capsule.Get(stage, agent.prim_path.AppendPath("Physics/BodyCollider"))
            run["checks"]["body_collision_preserved"] = UsdPhysics.CollisionAPI(body).GetCollisionEnabledAttr().Get()
            radius = body.GetRadiusAttr().Get()
            timeline.stop()
            timeline.commit()
            for _ in range(3):
                app.update()
            run["checks"]["timeout_restored_after_stop"] = settings.get(timeout_key) == original_timeout
            run["checks"]["visuals_restored_after_stop"] = all(
                stage.GetPrimAtPath(mesh.GetPath()).IsA(UsdGeom.Mesh) for mesh in original_meshes
            ) and not stage.GetPrimAtPath("/World/EVT_EXP/RenderVisuals")
            robot = None
            timeline.set_current_time(0)
            timeline.play()
            timeline.commit()
            advance_until(0.5)
            replay_agent = next(
                instance
                for instance in AgentsManager.get_instance().get_runtime_agent_instances()
                if instance.group == group
            )
            replay_body = UsdGeom.Capsule.Get(stage, replay_agent.prim_path.AppendPath("Physics/BodyCollider"))
            run["checks"]["replay_capsule_did_not_shrink"] = math.isclose(replay_body.GetRadiusAttr().Get(), radius)
            run["checks"]["replay_visual_count_preserved"] = len(
                stage.GetPrimAtPath("/World/EVT_EXP/RenderVisuals").GetChildren()
            ) == len(visual_records)
            timeline.stop()
            timeline.commit()
            for _ in range(3):
                app.update()
            run["checks"]["replay_timeout_restored"] = settings.get(timeout_key) == original_timeout
            report["runs"].append(run)
            print("BLOCKED_NAVIGATION_RUN " + json.dumps(run), flush=True)
            (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
            robot = physics = None
        report["passed"] = all(all(run["checks"].values()) for run in report["runs"])
    except Exception as error:
        report["error"] = repr(error)
        traceback.print_exc()
    finally:
        timeline.stop()
        del subscription
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print("BLOCKED_NAVIGATION_RESULT " + json.dumps(report), flush=True)
        app.close(exit_code=0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
