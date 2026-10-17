# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for visual-material manager terms."""

import inspect
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
import torch

import isaaclab.envs.mdp.events as events
import isaaclab.sim as sim_utils
from isaaclab.assets import VisualMaterialCfg
from isaaclab.envs.mdp.events import randomize_visual_color
from isaaclab.envs.mdp.visual_events import randomize_visual_material, randomize_visual_shape
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.renderers.render_context import RenderContext


class _RenderContext:
    def __init__(self):
        self.calls = []

    def write_visual_materials(self, materials, channels, env_ids):
        self.calls.append((materials, channels, env_ids))


class _Scene:
    num_envs = 4

    def __init__(self, materials):
        self.materials = dict(zip(("body", "legs"), materials, strict=True))

    def __getitem__(self, name):
        return self.materials[name]


def test_functional_color_randomization_does_not_create_graph_rng(monkeypatch):
    """Functional colors work without a graph and follow Replicator's seed changes."""
    seed = [17]
    written = []
    prim = SimpleNamespace(IsInstanceable=lambda: False)

    def graph_rng(*args, **kwargs):
        raise AssertionError("functional randomization must not create an OmniGraph RNG")

    rep = ModuleType("omni.replicator.core")
    rep.__file__ = "/extensions/omni.replicator.core-1.13.36/omni/replicator/core/__init__.py"
    rep.get_global_seed = lambda: seed[0]
    rep.rng = SimpleNamespace(ReplicatorRNG=graph_rng)
    rep.functional = SimpleNamespace(
        get=SimpleNamespace(prims=lambda **kwargs: [prim]),
        create_batch=SimpleNamespace(material=lambda **kwargs: [prim]),
        modify=SimpleNamespace(attribute=lambda prims, name, values: written.append(values)),
    )
    omni = ModuleType("omni")
    replicator = ModuleType("omni.replicator")
    omni.replicator = replicator
    replicator.core = rep
    carb = ModuleType("carb")
    tokens = ModuleType("carb.tokens")
    carb.tokens = tokens
    tokens.get_tokens_interface = lambda: SimpleNamespace(resolve=lambda path: path)
    for module in (omni, replicator, rep, carb, tokens):
        monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setattr(sim_utils, "enable_extension", lambda name: None)
    monkeypatch.setattr(events, "get_settings_manager", lambda: SimpleNamespace(get=lambda path: seed[0]))
    env = SimpleNamespace(
        cfg=SimpleNamespace(scene=SimpleNamespace(replicate_physics=False)),
        scene={"cube": SimpleNamespace(cfg=SimpleNamespace(prim_path="/Cube"))},
        sim=SimpleNamespace(stage=object()),
    )
    cfg = EventTermCfg(
        func=randomize_visual_color,
        mode="prestartup",
        params={
            "asset_cfg": SceneEntityCfg("cube"),
            "mesh_name": "mesh",
            "event_name": "color",
            "colors": {channel: (0.0, 1.0) for channel in ("r", "g", "b")},
        },
    )
    term = randomize_visual_color(cfg, env)
    for current_seed in (17, 17, 23, 17):
        seed[0] = current_seed
        term(env, None, **cfg.params)
    assert written[0].shape == (1, 3)
    assert np.all((written[0] >= 0.0) & (written[0] <= 1.0))
    assert not np.array_equal(written[0], written[1])
    assert not np.array_equal(written[0], written[2])
    np.testing.assert_array_equal(written[0], written[3])


def test_all_environment_slice_samples_one_gpu_row_per_material_and_environment() -> None:
    materials = [
        SimpleNamespace(
            cfg=VisualMaterialCfg(prim_path=f"/World/envs/env_0/Materials/{name}", spawn=sim_utils.PreviewSurfaceCfg()),
            is_per_env=True,
            channels=("color",),
        )
        for name in ("body", "legs")
    ]
    render_context = _RenderContext()
    env = SimpleNamespace(
        scene=_Scene(materials),
        sim=SimpleNamespace(render_context=render_context),
        device="cpu",
        num_envs=4,
    )
    cfg = EventTermCfg(
        func=randomize_visual_material,
        mode="reset",
        params={
            "materials": [SceneEntityCfg("body"), SceneEntityCfg("legs")],
            "channels": {"color": ((0.25, 0.5, 0.75), (0.25, 0.5, 0.75))},
        },
    )

    term = randomize_visual_material(cfg, env)
    term(env, slice(None), **cfg.params)

    written_materials, channels, env_ids = render_context.calls[0]
    assert written_materials == materials
    assert env_ids is None
    assert channels["color"].shape == (2, 4, 3)
    torch.testing.assert_close(channels["color"], torch.tensor([0.25, 0.5, 0.75]).expand(2, 4, 3))


def test_shape_backend_follows_only_active_render_consumers() -> None:
    cases = (
        (("newton_gl",), (), "physx", "newton"),
        ((), ("newton_warp",), "physx", "newton"),
    )
    for visualizers, renderers, physics, expected in cases:
        sim = SimpleNamespace(
            physics_manager=physics,
            resolve_visualizer_types=lambda visualizers=visualizers: list(visualizers),
            render_context=SimpleNamespace(renderer_types=renderers),
        )
        assert randomize_visual_shape._get_backend(None, SimpleNamespace(sim=sim)) == expected

    for visualizers, renderer_types in ((("kit",), ()), (("newton_gl",), ("isaac_rtx",))):
        sim = SimpleNamespace(
            resolve_visualizer_types=lambda visualizers=visualizers: list(visualizers),
            render_context=SimpleNamespace(renderer_types=renderer_types),
        )
        with pytest.raises(NotImplementedError, match="no per-shape visual storage"):
            randomize_visual_shape._get_backend(None, SimpleNamespace(sim=sim))

    selector = inspect.getsource(randomize_visual_shape._get_backend)
    assert "physics_manager" not in selector
    assert "FactoryBase._get_backend" not in selector

    unsupported_env = SimpleNamespace(
        sim=SimpleNamespace(resolve_visualizer_types=lambda: [], render_context=SimpleNamespace(renderer_types=()))
    )
    with pytest.raises(NotImplementedError, match="no per-shape visual storage"):
        randomize_visual_shape(None, unsupported_env)


def test_render_context_reports_registered_renderer_types() -> None:
    context = RenderContext()
    context._renderer_entries = [  # noqa: SLF001 - isolate the read-only capability query
        (SimpleNamespace(renderer_type="newton_warp"), object()),
        (SimpleNamespace(renderer_type="ovrtx"), object()),
    ]
    assert context.renderer_types == ("newton_warp", "ovrtx")
