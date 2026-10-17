# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for manager-based RL environments."""

from __future__ import annotations

import importlib
import sys
from types import ModuleType, SimpleNamespace

import gymnasium as gym
import numpy as np
import pytest

from isaaclab.envs import DirectMARLEnv, DirectRLEnv, ManagerBasedEnv, ManagerBasedRLEnv

pytestmark = pytest.mark.unit


@pytest.mark.parametrize("env_class", [ManagerBasedEnv, DirectRLEnv, DirectMARLEnv])
@pytest.mark.parametrize("stage_attached", [False, True])
@pytest.mark.parametrize("requested_seed", [17, -1])
def test_environment_seed_supports_unattached_stages(monkeypatch, env_class, stage_attached, requested_seed):
    """Environment seeding must share the resolved seed without requiring a USD stage."""
    settings = {}
    graph_seed = []
    resolved_seed = 123 if requested_seed == -1 else requested_seed

    def set_global_seed(seed):
        if seed < 0:
            raise ValueError("Seed must be a non-negative integer")
        settings["/omni/replicator/globalSeed"] = seed
        if not stage_attached:
            raise ValueError("Unable to retrieve replicator graph")
        graph_seed.append(seed)

    omni = ModuleType("omni")
    omni.usd = ModuleType("omni.usd")
    omni.usd.get_context = lambda: SimpleNamespace(get_stage=lambda: object() if stage_attached else None)
    omni.replicator = ModuleType("omni.replicator")
    omni.replicator.core = ModuleType("omni.replicator.core")
    omni.replicator.core.set_global_seed = set_global_seed
    for module in (omni, omni.usd, omni.replicator, omni.replicator.core):
        monkeypatch.setitem(sys.modules, module.__name__, module)

    module = importlib.import_module(env_class.__module__)
    monkeypatch.setattr(module, "configure_seed", lambda seed: 123 if seed == -1 else seed)
    monkeypatch.setattr(
        module,
        "get_settings_manager",
        lambda: SimpleNamespace(set=settings.__setitem__),
        raising=False,
    )

    assert env_class.seed(requested_seed) == resolved_seed
    assert settings["/omni/replicator/globalSeed"] == resolved_seed
    assert graph_seed == ([resolved_seed] if stage_attached else [])


def _make_env_with_policy_obs_terms(
    num_envs: int,
    terms: list[tuple[str, tuple[int, ...], tuple[float, float] | None]],
) -> ManagerBasedRLEnv:
    """Build an uninitialized env whose observation manager stubs drive space setup.

    Args:
        num_envs: Number of vectorized environments.
        terms: Non-concatenated policy observation terms as ``(name, shape, clip)``
            where ``clip`` is ``(low, high)`` or ``None``.

    Returns:
        An uninitialized :class:`~isaaclab.envs.ManagerBasedRLEnv` ready for
        :meth:`~isaaclab.envs.ManagerBasedRLEnv._configure_gym_env_spaces`.
    """
    term_names = [name for name, _, _ in terms]
    term_dims = [shape for _, shape, _ in terms]
    term_cfgs = [SimpleNamespace(clip=clip) for _, _, clip in terms]

    env = object.__new__(ManagerBasedRLEnv)
    env._is_closed = True
    env.scene = SimpleNamespace(num_envs=num_envs)
    env.observation_manager = SimpleNamespace(
        active_terms={"policy": term_names},
        group_obs_concatenate={"policy": False},
        group_obs_dim={"policy": term_dims},
        _group_obs_term_cfgs={"policy": term_cfgs},
    )
    env.action_manager = SimpleNamespace(action_term_dim=[0])
    return env


def test_non_concatenated_obs_groups_contain_all_terms():
    """Non-concatenated observation groups expose every term in the Dict space (issue #3133).

    Before the fix, only the last term in each non-concatenated group would be present
    in the observation space Dict. This test ensures all terms are correctly included.
    """
    terms = [
        ("image", (4, 5, 3), (0.0, 1.0)),
        ("matrix", (2, 3), (-2.0, 2.0)),
        ("vector", (3,), None),
    ]
    env = _make_env_with_policy_obs_terms(num_envs=2, terms=terms)
    ManagerBasedRLEnv._configure_gym_env_spaces(env)

    assert isinstance(env.observation_space, gym.spaces.Dict)
    policy_space = env.observation_space.spaces["policy"]
    assert isinstance(policy_space, gym.spaces.Dict)

    expected_policy_terms = ["image", "matrix", "vector"]
    assert list(policy_space.spaces) == expected_policy_terms
    for term_name in expected_policy_terms:
        assert isinstance(policy_space.spaces[term_name], gym.spaces.Box)


def test_obs_space_follows_clip_constraint():
    """Observation space bounds reflect the clip constraint on each non-concatenated term."""
    terms = [
        ("vector", (3,), None),
        ("matrix", (2, 3), (-2.0, 2.0)),
        ("image", (4, 5, 3), (0.0, 1.0)),
    ]
    expected_shapes = {
        "vector": (2, 3),
        "matrix": (2, 2, 3),
        "image": (2, 4, 5, 3),
    }
    term_clips = {name: clip for name, _, clip in terms}
    env = _make_env_with_policy_obs_terms(num_envs=2, terms=terms)
    ManagerBasedRLEnv._configure_gym_env_spaces(env)

    for group_name, group_space in env.observation_space.spaces.items():
        assert isinstance(group_space, gym.spaces.Dict)
        for term_name, term_space in group_space.spaces.items():
            clip = term_clips[term_name]
            low = -np.inf if clip is None else clip[0]
            high = np.inf if clip is None else clip[1]
            assert isinstance(term_space, gym.spaces.Box), (
                f"Expected Box space for {term_name} in {group_name}, got {type(term_space)}"
            )
            assert term_space.shape == expected_shapes[term_name]
            assert np.all(term_space.low == low)
            assert np.all(term_space.high == high)
