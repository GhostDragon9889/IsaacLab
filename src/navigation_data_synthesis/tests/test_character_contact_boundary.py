# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Check initial arms-down poses and live PhysX pedestrian contact boundaries.

Run with ``uv run python src/navigation_data_synthesis/tests/test_character_contact_boundary.py``.
Requires the project's local Isaac Sim assets and a GPU.
"""

import argparse
import asyncio
import json
import math
import time
import traceback
from pathlib import Path


def _skinned_world_points(agent):
    import numpy as np

    import carb
    from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

    native = agent.get_bh_agent()
    skeleton = next(UsdSkel.Skeleton(p) for p in Usd.PrimRange(agent.prim) if p.IsA(UsdSkel.Skeleton))
    cache = UsdGeom.XformCache()
    scale = Gf.Transform(cache.GetLocalToWorldTransform(skeleton.GetPrim())).GetScale()
    transforms = []
    for index, bind in enumerate(skeleton.GetBindTransformsAttr().Get()):
        translation, rotation = carb.Float3(), carb.Float4()
        assert native.get_joint_world_transform(index, translation, rotation), f"Missing live joint {index}"
        transform = Gf.Transform()
        transform.SetScale(scale)
        transform.SetTranslation(Gf.Vec3d(*translation))
        transform.SetRotation(Gf.Rotation(Gf.Quatd(rotation[3], Gf.Vec3d(rotation[0], rotation[1], rotation[2]))))
        transforms.append(bind.GetInverse() * transform.GetMatrix())
    skeleton_cache = UsdSkel.Cache()
    skeleton_cache.Populate(UsdSkel.Root(agent.prim), Usd.PrimDefaultPredicate)
    parts = []
    for mesh in (p for p in Usd.PrimRange(agent.prim) if p.IsA(UsdGeom.Mesh)):
        points = Vt.Vec3fArray(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
        skinning = skeleton_cache.GetSkinningQuery(mesh)
        if skinning:
            assert skinning.ComputeSkinnedPoints(Vt.Matrix4dArray(transforms), points)
            parts.append(np.asarray(points, dtype=np.float64))
        else:
            matrix = np.asarray(cache.GetLocalToWorldTransform(mesh))
            parts.append(np.asarray(points) @ matrix[:3, :3] + matrix[3, :3])
    return np.concatenate(parts)


def _measure_character_boundary(agent):
    import numpy as np

    import carb
    import omni.physx
    from pxr import UsdGeom

    native = agent.get_bh_agent()
    origin = np.asarray(list(native.get_world_translation()))
    height = native.get_height()
    path = agent.prim_path.AppendPath("Physics/BodyCollider")
    capsule = UsdGeom.Capsule.Get(agent.prim.GetStage(), path)
    points = _skinned_world_points(agent)
    # The central/lower body is where the EVT chassis can touch a standing person.
    band = points[(points[:, 2] > origin[2] + height * 0.25) & (points[:, 2] < origin[2] + height * 0.60)]
    assert len(band), "No visible vertices in the body contact band"
    visible_radius = float(np.linalg.norm(band[:, :2] - origin[:2], axis=1).max())
    boundaries = []
    query = omni.physx.get_physx_scene_query_interface()
    for axis in range(3):
        for sign in (-1, 1):
            start = origin.copy()
            start[2] += height * 0.5
            start[axis] += 2.0 * sign
            direction = [0.0, 0.0, 0.0]
            direction[axis] = -sign
            hits = []

            def collect(hit):
                if hit.rigid_body == str(path):
                    hits.append((hit.distance, list(hit.position)))
                return True

            query.raycast_all(carb.Float3(*start), carb.Float3(*direction), 4.0, collect)
            assert hits, f"PhysX missed {path} along axis {axis}, sign {sign}"
            boundaries.append(min(hits)[1][axis])
    radii = [float(abs(boundaries[i] - origin[i // 2])) for i in range(4)]
    return {
        "radius_m": capsule.GetRadiusAttr().Get(),
        "physical_radii_m": radii,
        "physical_height_m": abs(boundaries[5] - boundaries[4]),
        "native_height_m": height,
        "visible_contact_band_radius_m": visible_radius,
        "checks": {
            "physical_boundary_matches_usd": all(
                math.isclose(radius, capsule.GetRadiusAttr().Get(), abs_tol=0.002) for radius in radii
            ),
            "contact_covers_body_envelope": min(radii) + 0.005 >= visible_radius,
            "physical_height_preserved": math.isclose(abs(boundaries[5] - boundaries[4]), height, abs_tol=0.002),
        },
    }


def _measure_initial_poses(stage):
    from pxr import Usd, UsdGeom, UsdSkel

    characters = {}
    for group in stage.GetPrimAtPath("/World/Characters").GetChildren():
        if not group.GetName().startswith("walker_"):
            continue
        skeleton = next(UsdSkel.Skeleton(p) for p in Usd.PrimRange(group) if p.IsA(UsdSkel.Skeleton))
        root = skeleton.GetPrim()
        while not root.IsA(UsdSkel.Root):
            root = root.GetParent()
        cache = UsdSkel.Cache()
        cache.Populate(UsdSkel.Root(root), Usd.PrimDefaultPredicate)
        query = cache.GetSkelQuery(skeleton)
        world = UsdGeom.Xformable(skeleton).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        positions = {
            str(name).rsplit("/", 1)[-1]: (matrix * world).ExtractTranslation()
            for name, matrix in zip(query.GetJointOrder(), query.ComputeJointSkelTransforms(Usd.TimeCode.Default()))
        }
        drops = {}
        for side in ("L", "R"):
            shoulder, hand = positions[side + "_Upperarm"], positions[side + "_Hand"]
            drops[side] = (shoulder[2] - hand[2]) / (shoulder - hand).GetLength()
        characters[group.GetName()] = {
            "arm_drop_fractions": drops,
            "checks": {side + "_arm_down": drop > math.cos(math.radians(30)) for side, drop in drops.items()},
        }
    return characters


def main() -> None:
    """Check stopped poses and real contact geometry across repeated playback."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("logs/character_contact_boundary"))
    parser.add_argument("--experience", default="", help="Optional Kit experience path for matching the GUI runtime.")
    parser.add_argument("--cases", nargs="+", default=["third_floor_case_1", "third_floor_case_3"])
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
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.replicator.agent.core")
    app.update()
    from isaacsim.replicator.agent.core import api as ira
    from omni.metropolis.pipeline.agent import AgentsManager

    timeline = omni.timeline.get_timeline_interface()
    report = {"passed": False, "runs": [], "initial_poses": []}
    try:
        for case in args.cases:
            config = Path(__file__).resolve().parents[1] / "config/third_floor" / (case + ".yaml")
            assert ira.load_config_file(str(config)), f"IRA rejected {case}"
            future = asyncio.ensure_future(ira.setup_simulation())
            deadline = time.monotonic() + 240
            while not future.done():
                assert time.monotonic() < deadline, "Scene setup timed out"
                app.update()
            future.result()
            for _ in range(3):
                app.update()
            report["initial_poses"].append(
                {
                    "case": case,
                    "cycle": "setup",
                    "characters": _measure_initial_poses(omni.usd.get_context().get_stage()),
                }
            )
            for cycle in ("initial", "replay"):
                timeline.set_current_time(0)
                timeline.play()
                timeline.commit()
                deadline = time.monotonic() + 120
                while timeline.get_current_time() < 0.75:
                    assert time.monotonic() < deadline, "Playback timed out"
                    app.update()
                agents = AgentsManager.get_instance().get_runtime_agent_instances()
                assert len(agents) == (3 if case.endswith("1") else 5), "Characters missing"
                run = {"case": case, "cycle": cycle, "characters": {}}
                for agent in agents:
                    run["characters"][agent.group] = _measure_character_boundary(agent)
                report["runs"].append(run)
                print("CONTACT_BOUNDARY_RUN " + json.dumps(run), flush=True)
                timeline.stop()
                timeline.commit()
                for _ in range(3):
                    app.update()
                report["initial_poses"].append(
                    {
                        "case": case,
                        "cycle": "stop_" + cycle,
                        "characters": _measure_initial_poses(omni.usd.get_context().get_stage()),
                    }
                )
        report["passed"] = all(
            all(character["checks"].values())
            for run in [*report["runs"], *report["initial_poses"]]
            for character in run["characters"].values()
        )
    except Exception as error:
        report["error"] = repr(error)
        traceback.print_exc()
    finally:
        timeline.stop()
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print("CONTACT_BOUNDARY_RESULT " + json.dumps(report), flush=True)
        app.close(exit_code=0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
