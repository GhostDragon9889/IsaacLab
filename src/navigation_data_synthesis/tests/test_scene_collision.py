# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Check pedestrian wall clearance and motor-driven robot contact in case_1.

Run with ``uv run python src/navigation_data_synthesis/tests/test_scene_collision.py``.
Requires the project's local Isaac Sim assets and a GPU.
``--controller gamepad`` replays sampled axes after the device readers; physical gamepad acquisition is not covered.
"""

import argparse
import asyncio
import json
import math
import os
import time
import traceback
from contextlib import ExitStack
from itertools import combinations
from pathlib import Path


class _GamepadReplay:
    """Replay sampled axes while retaining the asset's mixer, guards and controller."""

    def __init__(self, stage) -> None:
        import omni.graph.core as og
        from pxr import Usd

        self._graph = "/World/EVT_EXP/Gamepad"
        self._stage = stage
        assert stage.GetPrimAtPath(self._graph).IsActive(), "The authored Gamepad graph must remain active"
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            og.Controller.edit(
                self._graph,
                {
                    og.Controller.Keys.CREATE_NODES: [
                        (f"ReplayAxis{i}", "omni.graph.nodes.ConstantFloat") for i in range(3)
                    ]
                    + [(f"ReplayPressed{i}", "omni.graph.nodes.ConstantBool") for i in range(3)]
                },
            )
            sources = {
                f"{self._graph}/Read{i}.outputs:{name}": f"{self._graph}/Replay{kind}{i}.inputs:value"
                for i in range(3)
                for name, kind in (("value", "Axis"), ("isPressed", "Pressed"))
            }
            for node in list(stage.GetPrimAtPath(self._graph).GetChildren()):
                for attr in node.GetAttributes():
                    for source in list(attr.GetConnections()):
                        if str(source) in sources:
                            og.Controller.disconnect(str(source), str(attr.GetPath()))
                            og.Controller.connect(sources[str(source)], str(attr.GetPath()))
        self.set_axes((0.0, 0.0, 0.0))

    def set_axes(self, axes: tuple[float, float, float]) -> None:
        import omni.graph.core as og
        from pxr import Usd

        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            for index, value in enumerate(axes):
                og.Controller.set(og.Controller.attribute(f"{self._graph}/ReplayAxis{index}.inputs:value"), value)
                og.Controller.set(
                    og.Controller.attribute(f"{self._graph}/ReplayPressed{index}.inputs:value"), abs(value) > 0.06
                )

    def output(self):
        import omni.graph.core as og

        return og.Controller.get(og.Controller.attribute(f"{self._graph}/Safe.outputs:result"))


def _configure_drive(stage, controller: str) -> _GamepadReplay | None:
    from pxr import Usd

    if controller == "gamepad":
        return _GamepadReplay(stage)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath("/World/EVT_EXP/Gamepad").SetActive(False)
    return None


def _set_drive(robot, targets, wheel_ids, indices, replay, active: bool, axes) -> None:
    if replay is not None:
        replay.set_axes(axes if active else (0.0, 0.0, 0.0))
    else:
        targets[0, wheel_ids] = -120.0 if active else 0.0
        robot.set_dof_velocity_targets(targets, indices)


def _check_gamepad_commands(app, replay, robot, wheel_ids) -> dict:
    import numpy as np

    if replay is None:
        return {}
    samples = []
    for label, axes, enabled in [
        ("neutral", (0.0, 0.0, 0.0), True),
        ("forward", (1.0, 0.0, 0.0), True),
        ("backward", (-1.0, 0.0, 0.0), True),
        ("left", (0.0, -1.0, 0.0), True),
        ("right", (0.0, 1.0, 0.0), True),
        ("turn_left", (0.0, 0.0, -1.0), True),
        ("turn_right", (0.0, 0.0, 1.0), True),
        ("mixed_limit", (1.0, 1.0, 1.0), True),
        ("released", (0.0, 0.0, 0.0), True),
        ("invalid_nan", (math.nan, 0.0, 0.0), False),
        ("held_after_invalid", (-1.0, 0.0, 0.0), False),
        ("neutral_rearm", (0.0, 0.0, 0.0), True),
        ("backward_rearmed", (-1.0, 0.0, 0.0), True),
        ("invalid_range", (2.0, 0.0, 0.0), False),
        ("held_after_range", (1.0, 0.0, 0.0), False),
        ("neutral_final", (0.0, 0.0, 0.0), True),
    ]:
        for _ in range(6):
            replay.set_axes(axes)
            app.update()
        # Independent mecanum inverse kinematics: r = 0.075 m, half-length + half-width = 0.33 m.
        vx, vy, yaw = 0.5 * axes[0], -0.5 * axes[1], -0.6 * axes[2]
        expected = (
            np.clip(
                np.array([vx - vy - 0.33 * yaw, vx + vy + 0.33 * yaw, vx + vy - 0.33 * yaw, vx - vy + 0.33 * yaw])
                / 0.075,
                -15.0,
                15.0,
            )
            if enabled
            else np.zeros(4)
        )
        actual = robot.get_dof_velocity_targets()[0, wheel_ids]
        samples.append(
            {
                "label": label,
                "expected_rad_s": expected.tolist(),
                "actual_rad_s": actual.tolist(),
                "mapping_passed": bool(np.allclose(actual, expected, atol=1e-5)),
                "controller_passed": bool(np.allclose(actual, replay.output(), atol=1e-5)),
                "bounded": bool(np.isfinite(actual).all() and np.abs(actual).max() <= 15.00001),
            }
        )
    return {
        "input_boundary": "sampled_axis_outputs; physical_device_reader_not_emulated",
        "samples": samples,
        "checks": {
            "gamepad_mapping_and_rearm": all(sample["mapping_passed"] for sample in samples),
            "gamepad_output_reaches_articulation": all(sample["controller_passed"] for sample in samples),
            "gamepad_wheel_targets_finite_and_bounded": all(sample["bounded"] for sample in samples),
        },
    }


def _robot_drive_cases(gamepad: bool, washstation_sweep: bool = False, labels: list[str] | None = None) -> list[tuple]:
    cases = [
        ("north_wall", (-1.5, 0.9), 270, "/World/Collision/Walls", 1, 1, 1.6, (-1.0, 0.0, 0.0)),
        ("washstation", (-1.45, 0.65), 0, "/World/Details/WashStation/", 0, -1, -2.065, (-1.0, 0.0, 0.0)),
        ("partition", (-1.5, -0.75), 180, "/World/Cubicles/Partitions/West", 0, 1, -0.995, (-1.0, 0.0, 0.0)),
        ("door", (-2.95, -0.70), 90, "/World/Meshes/Assets/Door_0000/Geom/", 1, -1, None, (-1.0, 0.0, 0.0)),
    ]
    if gamepad:
        cases.append(
            ("washstation_lateral", (-1.45, 0.65), 270, "/World/Details/WashStation/", 0, -1, -2.065, (0.0, 1.0, 0.0))
        )
        cases.append(
            ("washstation_handle", (-1.45, 1.07), 0, "/World/Details/WashStation/", 0, -1, -2.065, (-1.0, 0.0, 0.0))
        )
        cases.extend(
            [
                (
                    "washstation_south",
                    (-2.35, -0.90),
                    270,
                    "/World/Details/WashStation/",
                    1,
                    1,
                    -0.27,
                    (-1.0, 0.0, 0.0),
                ),
                (
                    "washstation_south_lateral",
                    (-2.35, -0.90),
                    180,
                    "/World/Details/WashStation/",
                    1,
                    1,
                    -0.27,
                    (0.0, 1.0, 0.0),
                ),
                (
                    "washstation_corner",
                    (-1.5, -0.75),
                    0,
                    "/World/Details/WashStation/",
                    0,
                    -1,
                    -2.065,
                    (-1.0, -1.0, 0.0),
                ),
            ]
        )
        if washstation_sweep:
            for yaw in (45, 90, 135, 180, 225, 315):
                angle = math.radians(yaw)
                cases.append(
                    (
                        f"washstation_yaw_{yaw}",
                        (-1.45, 0.65),
                        yaw,
                        "/World/Details/WashStation/",
                        0,
                        -1,
                        -2.065,
                        (-math.cos(angle), -math.sin(angle), 0.0),
                    )
                )
    if labels:
        unknown = set(labels) - {case[0] for case in cases}
        if unknown:
            raise ValueError(f"Drive cases unavailable with this controller: {sorted(unknown)}")
        cases = [case for case in cases if case[0] in labels]
    return cases


def _washstation_visual_overlap(stage) -> dict | None:
    """Find live robot vertices more than 3 mm inside the cabinet, counter or handles."""
    import numpy as np

    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache()
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    meshes = []
    for mesh in Usd.PrimRange(stage.GetPrimAtPath("/World/EVT_EXP/RenderVisuals")):
        if mesh.IsA(UsdGeom.Mesh):
            meshes.append((mesh, bounds.ComputeWorldBound(mesh).ComputeAlignedRange()))
    for name in ("Cabinet", "Counter", "Handle0", "Handle1"):
        prim = stage.GetPrimAtPath("/World/Details/WashStation/" + name)
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim).GetInverse())
        scale = np.linalg.norm(np.asarray(cache.GetLocalToWorldTransform(prim))[:3, :3], axis=1)
        object_bounds = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        for mesh, mesh_bounds in meshes:
            if any(
                object_bounds.GetMin()[i] >= mesh_bounds.GetMax()[i]
                or mesh_bounds.GetMin()[i] >= object_bounds.GetMax()[i]
                for i in range(3)
            ):
                continue
            world = np.asarray(cache.GetLocalToWorldTransform(mesh))
            points = np.asarray(UsdGeom.Mesh(mesh).GetPointsAttr().Get(), dtype=np.float64)
            points = points @ world[:3, :3] + world[3, :3]
            local = points @ matrix[:3, :3] + matrix[3, :3]
            if prim.IsA(UsdGeom.Cube):
                depths = (UsdGeom.Cube(prim).GetSizeAttr().Get() * 0.5 - np.abs(local)) * scale
                depths = depths.min(axis=1)
            else:
                cylinder = UsdGeom.Cylinder(prim)
                assert cylinder.GetAxisAttr().Get() == "Z"
                depths = np.minimum(
                    cylinder.GetRadiusAttr().Get() - np.linalg.norm(local[:, :2], axis=1),
                    cylinder.GetHeightAttr().Get() * 0.5 - np.abs(local[:, 2]),
                )
            index = int(np.argmax(depths))
            if depths[index] > 0.003:
                return {
                    "object": str(prim.GetPath()),
                    "robot_mesh": str(mesh.GetPath()),
                    "depth_m": float(depths[index]),
                    "point_m": points[index].tolist(),
                }
    return None


def _is_wall_shape(path: str) -> bool:
    return path == "/World/Collision/Walls" or "/Partitions/" in path or "/Partition_" in path


def _record_pair_clearances(stage, agents, now: float, records: dict) -> None:
    from pxr import UsdGeom

    for first, second in combinations(agents, 2):
        radii = [
            UsdGeom.Capsule.Get(stage, a.prim_path.AppendPath("Physics/BodyCollider")).GetRadiusAttr().Get()
            for a in (first, second)
        ]
        distance = math.dist(list(first.get_world_position())[:2], list(second.get_world_position())[:2])
        key = "/".join(sorted((first.group, second.group)))
        clearance = distance - sum(radii)
        if clearance < records.get(key, {}).get("minimum_gap_m", math.inf):
            records[key] = {
                "minimum_gap_m": clearance,
                "time_s": now,
                "positions_m": [list(a.get_world_position()) for a in (first, second)],
            }


def _record_robot_contacts(headers, data, run: dict | None, now: float) -> None:
    from pxr import PhysicsSchemaTools

    if run is None:
        return
    for header in headers:
        actors = [str(PhysicsSchemaTools.intToSdfPath(actor)) for actor in (header.actor0, header.actor1)]
        if "/World/EVT_EXP/base_link" not in actors or not any(actor.startswith(run["target"]) for actor in actors):
            continue
        if header.num_contact_data:
            run.setdefault("first_contact_time_s", now)
            run["contacts"] += header.num_contact_data
            for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                run["min_separation_m"] = min(run["min_separation_m"], data[index].separation)


def _prepare_robot_visuals(stage, cleanup: ExitStack, enabled: bool) -> None:
    if not enabled:
        return
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from navigation_data_synthesis.config.simulation_setup_script.robot_visuals import RobotVisuals

    cleanup.callback(RobotVisuals(stage, stage.GetPrimAtPath("/World/CharacterBodyColliderSettings")).close)


def _check_collision_proxies(stage) -> dict:
    import numpy as np

    import carb
    import omni.physx
    from pxr import Gf, Usd, UsdGeom

    physics = omni.physx.get_physx_interface()
    query = omni.physx.get_physx_scene_query_interface()
    records = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/EVT_EXP")):
        if prim.GetParent().GetName() != "PhysicsCollisionProxies":
            continue
        body = prim.GetParent().GetParent()
        pose = physics.get_rigidbody_transformation(str(body.GetPath()))
        assert pose["ret_val"], f"Missing body for proxy {prim.GetPath()}"
        transform = Gf.Transform()
        transform.SetTranslation(Gf.Vec3d(*pose["position"]))
        rotation = pose["rotation"]
        transform.SetRotation(Gf.Rotation(Gf.Quatd(rotation[3], Gf.Vec3d(rotation[0], rotation[1], rotation[2]))))
        if prim.IsA(UsdGeom.Cube):
            center = UsdGeom.Xformable(prim).GetLocalTransformation().ExtractTranslation()
        else:
            center = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=np.float64).mean(axis=0)
        center = transform.GetMatrix().Transform(Gf.Vec3d(*center))
        hits = []

        def collect(hit):
            if hit.collision == str(prim.GetPath()):
                hits.append(hit.distance)
            return True

        query.raycast_all(carb.Float3(center[0] + 1.0, center[1], center[2]), carb.Float3(-1.0, 0.0, 0.0), 2.0, collect)
        records[str(prim.GetPath())] = bool(hits)
    return {"shapes": records, "passed": bool(records) and all(records.values())}


def _visible_chassis_clearance(stage, label, axis, sign, plane) -> float | None:
    import numpy as np

    from pxr import Usd, UsdGeom

    visual = stage.GetPrimAtPath("/World/EVT_EXP/RenderVisuals/mesh_0")
    if not visual or plane is None:
        return None
    points = np.asarray(UsdGeom.Mesh(visual).GetPointsAttr().Get(), dtype=np.float64)
    matrix = np.asarray(UsdGeom.Xformable(visual).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    points = points @ matrix[:3, :3] + matrix[3, :3]
    points = points[(points[:, 2] > 0.25) & (points[:, 2] < 0.75)]
    if label.startswith("washstation"):
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        cabinet = bounds.ComputeWorldBound(
            stage.GetPrimAtPath("/World/Details/WashStation/Cabinet")
        ).ComputeAlignedRange()
        perpendicular = 1 - axis
        points = points[
            (points[:, perpendicular] >= cabinet.GetMin()[perpendicular])
            & (points[:, perpendicular] <= cabinet.GetMax()[perpendicular])
        ]
    if not len(points):
        return None
    surface = float(points[:, axis].max() if sign > 0 else points[:, axis].min())
    return sign * (plane - surface)


def _visible_wall_penetration(agent, query) -> dict | None:
    import numpy as np
    from test_character_contact_boundary import _skinned_world_points

    import carb

    points = _skinned_world_points(agent)
    origin = np.asarray(list(agent.get_world_position()))
    angles = np.arange(8) * math.pi / 4
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    for height in np.arange(0.1, agent.get_bh_agent().get_height() + 0.05, 0.1):
        band = points[np.abs(points[:, 2] - origin[2] - height) < 0.05]
        if not len(band):
            continue
        for index in np.argmax((band[:, :2] - origin[:2]) @ directions.T, axis=0):
            vertex = band[index]
            start = np.array((origin[0], origin[1], vertex[2]))
            delta = vertex - start
            length = float(np.linalg.norm(delta))
            if length <= 0.003:
                continue
            hits = []

            def collect(hit):
                if _is_wall_shape(hit.collision) and hit.distance < length - 0.003:
                    hits.append(
                        {"collider": hit.collision, "wall_point_m": list(hit.position), "vertex_m": vertex.tolist()}
                    )
                return True

            query.raycast_all(carb.Float3(*start), carb.Float3(*(delta / length)), length, collect)
            if hits:
                return hits[0]
    return None


def _sample_standing_pose(agent, samples: dict) -> None:
    import carb

    if agent.group != "walker_01":
        return
    position = list(agent.get_world_position())
    joint, rotation = carb.Float3(), carb.Float4()
    assert agent.get_bh_agent().get_joint_world_transform(30, joint, rotation)
    samples["positions"].append(position)
    samples["joint_offsets"].append([joint[i] - position[i] for i in range(3)])


def _standing_metrics(samples: dict) -> dict:
    import numpy as np

    positions = np.asarray(samples["positions"])
    return {
        "maximum_drift_m": float(np.linalg.norm(positions - positions[0], axis=1).max()),
        "joint_motion_m": float(np.ptp(np.asarray(samples["joint_offsets"]), axis=0).max()),
    }


def main() -> None:
    """Run real PhysX overlap queries and wheel-drive wall contact checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("logs/scene_collision"))
    parser.add_argument("--experience", default="", help="Optional Kit experience path for matching the GUI runtime.")
    parser.add_argument(
        "--visuals-before-physics", action="store_true", help="Initialize render separation before loading physics."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config/third_floor/third_floor_case_1.yaml",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="Pedestrian playback duration [s].")
    parser.add_argument("--pedestrians-only", action="store_true", help="Skip the robot drive checks.")
    parser.add_argument("--robot-only", action="store_true", help="Skip pedestrian playback checks.")
    parser.add_argument(
        "--controller", choices=("wheels", "gamepad"), default="wheels", help="Direct wheels or Gamepad axis replay."
    )
    parser.add_argument("--washstation-sweep", action="store_true", help="Check six more chassis orientations.")
    parser.add_argument("--drive-cases", nargs="+", help="Run only these named robot contact cases.")
    parser.add_argument("--contact-seconds", type=float, default=2.0, help="Drive after first contact [s].")
    parser.add_argument(
        "--block-passage-seconds",
        type=float,
        default=0.0,
        help="Park the robot in the shared passage before clearing [s].",
    )
    args, _ = parser.parse_known_args()
    args.output.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {"headless": True, "width": 640, "height": 360, "limit_cpu_threads": 8, "multi_gpu": False},
        experience=args.experience,
    )
    import numpy as np
    import yaml

    import carb
    import carb.settings
    import omni.physics.tensors as tensors
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import PhysxSchema, Usd, UsdGeom

    enable_extension("isaacsim.replicator.agent.core")
    app.update()
    from isaacsim.replicator.agent.core import api as ira
    from omni.metropolis.pipeline.agent import AgentsManager

    timeline = omni.timeline.get_timeline_interface()
    query = omni.physx.get_physx_scene_query_interface()
    report = {"passed": False, "checks": {}}
    report["runtime"] = {"isaac_path": os.environ.get("ISAAC_PATH"), "experience": args.experience or "default_python"}
    robot = None
    subscription = None
    visual_cleanup = ExitStack()
    indices = np.array([0], dtype=np.uint32)
    try:
        assert ira.load_config_file(str(args.config.resolve())), "IRA rejected configuration"
        future = asyncio.ensure_future(ira.setup_simulation())
        deadline = time.monotonic() + 240
        while not future.done():
            assert time.monotonic() < deadline, "Scene setup timed out"
            app.update()
        future.result()
        stage = omni.usd.get_context().get_stage()
        _prepare_robot_visuals(stage, visual_cleanup, args.visuals_before_physics)
        report["visuals_before_physics"] = args.visuals_before_physics
        replay = _configure_drive(stage, args.controller)
        report["controller"] = args.controller
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            PhysxSchema.PhysxContactReportAPI.Apply(
                stage.GetPrimAtPath("/World/EVT_EXP/base_link")
            ).CreateThresholdAttr(0.0)
        carb.settings.get_settings().set("/app/player/useFixedTimeStepping", False)
        timeline.play()
        timeline.commit()
        while timeline.get_current_time() < 0.75:
            app.update()
        physics = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
        physics.set_subspace_roots("/")
        robot = physics.create_articulation_view("/World/EVT_EXP/base_link")
        assert robot.count == 1
        report["proxy_geometry"] = _check_collision_proxies(stage)
        report["checks"]["proxy_shapes_queryable"] = report["proxy_geometry"]["passed"]
        home = robot.get_root_transforms().copy()

        def park(x, y):
            transform = home.copy()
            transform[0, :2] = (x, y)
            robot.set_root_transforms(transform, indices)
            robot.set_root_velocities(np.zeros((1, 6), dtype=np.float32), indices)

        blocked = args.block_passage_seconds > 0.0
        park(0.3, 0.4) if blocked else park(10.0, 10.0)
        overlaps = {}
        capsule_overlaps = {}
        next_skin_check = {}
        positions = {}
        standing_samples = {"positions": [], "joint_offsets": []}
        frame_times = []
        pair_clearances = {}
        deadline = time.monotonic() + 240
        while timeline.get_current_time() < (0.75 if args.robot_only else args.duration):
            assert time.monotonic() < deadline, "Playback timed out"
            started = time.monotonic()
            app.update()
            now = timeline.get_current_time()
            if blocked and now >= args.block_passage_seconds:
                park(10.0, 10.0)
                blocked = False
            if 1.0 <= now <= 6.0:
                frame_times.append(time.monotonic() - started)
            agents = AgentsManager.get_instance().get_runtime_agent_instances()
            _record_pair_clearances(stage, agents, now, pair_clearances)
            for agent in agents:
                _sample_standing_pose(agent, standing_samples)
                position = list(agent.get_world_position())
                positions[agent.group] = position
                radius = (
                    UsdGeom.Capsule.Get(stage, agent.prim_path.AppendPath("Physics/BodyCollider")).GetRadiusAttr().Get()
                )
                walls = []

                def collect(hit):
                    if _is_wall_shape(hit.collision):
                        walls.append(hit.collision)
                    return True

                # A horizontal capsule cross section, with 5 mm tolerance for contact skin.
                query.overlap_sphere(radius - 0.005, carb.Float3(position[0], position[1], position[2] + 0.7), collect)
                if walls:
                    record = capsule_overlaps.setdefault(
                        agent.group, {"first_time_s": now, "position_m": position, "samples": 0}
                    )
                    record["samples"] += 1
                    if agent.group not in overlaps and now >= next_skin_check.get(agent.group, 0.0):
                        next_skin_check[agent.group] = now + 0.25
                        penetration = _visible_wall_penetration(agent, query)
                        if penetration:
                            overlaps[agent.group] = {"time_s": now, "position_m": position, **penetration}
        report["wall_overlaps"] = overlaps
        report["capsule_overlaps"] = capsule_overlaps
        report["last_positions_m"] = positions
        report["walking_fps"] = len(frame_times) / sum(frame_times) if frame_times else 0.0
        report["pedestrian_pair_clearances"] = pair_clearances
        if not args.robot_only:
            report["checks"]["pedestrian_bodies_separated"] = all(
                pair["minimum_gap_m"] >= -0.02 for pair in pair_clearances.values()
            )
            report["checks"]["pedestrians_clear_of_walls"] = not overlaps
            report["standing"] = _standing_metrics(standing_samples)
            report["checks"]["standing_position_retained"] = report["standing"]["maximum_drift_m"] < 0.01
            report["checks"]["standing_animation_continues"] = report["standing"]["joint_motion_m"] > 0.001
        groups = yaml.safe_load(args.config.read_text())["isaacsim.replicator.agent"]["character"]["groups"]
        report["target_distances_m"] = {}
        for name in () if args.robot_only else ("walker_02", "walker_03"):
            target = groups[name]["triggers"][0]["time_trigger"]["behavior"][0]["patrol"]["path_points"][-1]
            distance = math.dist(positions[name], target)
            report["target_distances_m"][name] = distance
            report["checks"][name + "_reached_target"] = distance < 0.2
        print("SCENE_PEDESTRIAN_RESULT " + json.dumps(report), flush=True)
        if args.pedestrians_only:
            report["passed"] = all(report["checks"].values())
            return

        timeline.stop()
        timeline.commit()
        for _ in range(3):
            app.update()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            stage.GetPrimAtPath("/World/Characters").SetActive(False)
        timeline.set_current_time(0.0)
        timeline.set_end_time(180.0)
        timeline.play()
        timeline.commit()
        while timeline.get_current_time() < 0.75:
            app.update()
        physics = tensors.create_simulation_view("numpy", stage_id=omni.usd.get_context().get_stage_id())
        physics.set_subspace_roots("/")
        robot = physics.create_articulation_view("/World/EVT_EXP/base_link")
        home = robot.get_root_transforms().copy()

        targets = robot.get_dof_velocity_targets().copy()
        home_joints = robot.get_dof_positions().copy()
        names = robot.get_metatype(0).dof_names
        wheel_ids = [names.index(f"wheel_{corner}_joint") for corner in ("fl", "fr", "rl", "rr")]
        report["gamepad"] = _check_gamepad_commands(app, replay, robot, wheel_ids)
        report["checks"].update(report["gamepad"].get("checks", {}))
        active_run = None

        def on_contact(headers, data):
            _record_robot_contacts(headers, data, active_run, timeline.get_current_time())

        subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        report["wheel_drive"] = []
        # Each start has room for the chassis. A movable door may open after contact.
        for label, start, yaw, target, axis, sign, plane, axes in _robot_drive_cases(
            replay is not None, args.washstation_sweep, args.drive_cases
        ):
            _set_drive(robot, targets, wheel_ids, indices, replay, False, axes)
            transform = home.copy()
            transform[0, :2] = start
            angle = math.radians(yaw) / 2
            transform[0, 3:] = (0.0, 0.0, math.sin(angle), math.cos(angle))
            robot.set_root_transforms(transform, indices)
            robot.set_dof_positions(home_joints, indices)
            robot.set_root_velocities(np.zeros((1, 6), dtype=np.float32), indices)
            robot.set_dof_velocities(np.zeros_like(robot.get_dof_velocities()), indices)
            for _ in range(10):
                app.update()
            origin = robot.get_root_transforms()[0, :3].copy()
            active_run = {
                "label": label,
                "target": target,
                "contacts": 0,
                "min_separation_m": 0.0,
                "start_m": origin.tolist(),
            }
            report["wheel_drive"].append(active_run)
            deadline = time.monotonic() + 120
            end_time = timeline.get_current_time() + 24.0
            next_visual_check = timeline.get_current_time()
            while timeline.get_current_time() < end_time:
                assert time.monotonic() < deadline, "Wheel drive timed out"
                _set_drive(robot, targets, wheel_ids, indices, replay, True, axes)
                app.update()
                if label.startswith("washstation") and timeline.get_current_time() >= next_visual_check:
                    next_visual_check = timeline.get_current_time() + 0.25
                    if not active_run.get("visual_overlap"):
                        active_run["visual_overlap"] = _washstation_visual_overlap(stage)
                if (
                    timeline.get_current_time()
                    > active_run.get("first_contact_time_s", math.inf) + args.contact_seconds
                ):
                    break
            active_run["end_m"] = robot.get_root_transforms()[0, :3].tolist()
            active_run["wheel_targets_rad_s"] = robot.get_dof_velocity_targets()[0, wheel_ids].tolist()
            active_run["checks"] = {
                "wheel_commands_moved_robot": sign * (active_run["end_m"][axis] - float(origin[axis])) > 0.01,
                "actual_environment_contact": active_run["contacts"] > 0,
                "no_deep_physical_penetration": active_run["min_separation_m"] >= -0.025,
            }
            clearance = _visible_chassis_clearance(stage, label, axis, sign, plane)
            if clearance is not None:
                active_run["visual_clearance_m"] = clearance
                active_run["checks"]["visible_chassis_clear_of_surface"] = clearance >= 0.005
            if label.startswith("washstation"):
                if not active_run.get("visual_overlap"):
                    active_run["visual_overlap"] = _washstation_visual_overlap(stage)
                active_run["checks"]["robot_visuals_clear_of_washstation"] = active_run["visual_overlap"] is None
            print("SCENE_WHEEL_RESULT " + json.dumps(active_run), flush=True)
            active_run = None
        _set_drive(robot, targets, wheel_ids, indices, replay, False, (0.0, 0.0, 0.0))
        report["checks"]["robot_environment_contact"] = all(
            all(run["checks"].values()) for run in report["wheel_drive"]
        )
        report["passed"] = all(report["checks"].values())
    except Exception as error:
        report["error"] = repr(error)
        traceback.print_exc()
    finally:
        if subscription is not None:
            subscription.unsubscribe()
        timeline.stop()
        visual_cleanup.close()
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print("SCENE_COLLISION_RESULT " + json.dumps(report), flush=True)
        app.close(exit_code=0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
