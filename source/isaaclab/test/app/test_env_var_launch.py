# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import sys

import pytest
from standalone_script_cases import run_until_ready

import isaaclab.app.app_launcher as app_launcher_module
from isaaclab.app import AppLauncher

pytestmark = pytest.mark.integration


def test_livestream_launch_with_env_vars(monkeypatch):
    """Test launching with environment variables."""
    monkeypatch.setenv("LIVESTREAM", "1")
    monkeypatch.setenv("HEADLESS", "1")
    code = """
from isaaclab.app import AppLauncher
from isaaclab.app.settings_manager import get_settings_manager
launcher = AppLauncher()
assert get_settings_manager().get("/app/window/enabled") is False
assert launcher._livestream == 1
assert launcher._headless is True
assert "omni.kit.livestream.app" in launcher._livestream_args
print("LIVESTREAM_READY", flush=True)
launcher.app.close()
"""
    result = run_until_ready([sys.executable, "-c", code], "LIVESTREAM_READY")
    assert result.ready and result.returncode == 0 and not result.fatal_patterns, result.output


def _resolve_kit_args(
    monkeypatch: pytest.MonkeyPatch, launcher_args: dict, argv: list[str] | None = None
) -> tuple[list[str], list[str]]:
    monkeypatch.setattr(sys, "argv", ["pytest", *(argv or [])])
    launcher = AppLauncher.__new__(AppLauncher)
    launcher._xr = False
    launcher._resolve_kit_args(launcher_args)
    return sys.argv[1:], launcher._kit_args


@pytest.mark.parametrize("env_value, expected", [("0", "false"), ("1", "true")])
def test_fabric_gpu_interop_env_adds_override(monkeypatch: pytest.MonkeyPatch, env_value: str, expected: str):
    monkeypatch.setenv(app_launcher_module._FABRIC_GPU_INTEROP_ENV, env_value)

    kit_args, _ = _resolve_kit_args(monkeypatch, {})

    assert kit_args == [
        f"--/physics/fabricUseGPUInterop={expected}",
        "--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false",
    ]


def test_fabric_gpu_interop_env_rejects_invalid_value(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv(app_launcher_module._FABRIC_GPU_INTEROP_ENV, "false")

    with pytest.raises(ValueError, match="Expected: 0 or 1"):
        _resolve_kit_args(monkeypatch, {})


def test_explicit_kit_setting_takes_precedence(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv(app_launcher_module._FABRIC_GPU_INTEROP_ENV, "0")
    explicit_arg = "--/physics/fabricUseGPUInterop=true"

    kit_args, resolved_args = _resolve_kit_args(monkeypatch, {}, [explicit_arg])

    assert kit_args == [
        explicit_arg,
        "--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false",
    ]
    assert resolved_args == ["--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false"]


def test_explicit_kit_args_setting_takes_precedence(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv(app_launcher_module._FABRIC_GPU_INTEROP_ENV, "0")
    explicit_arg = "--/physics/fabricUseGPUInterop=true"

    kit_args, resolved_args = _resolve_kit_args(monkeypatch, {"kit_args": explicit_arg})

    assert kit_args == [
        explicit_arg,
        "--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false",
    ]
    assert resolved_args == [
        explicit_arg,
        "--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false",
    ]


def test_simulation_manager_default_callbacks_disabled(monkeypatch: pytest.MonkeyPatch):
    kit_args, resolved_args = _resolve_kit_args(monkeypatch, {})

    assert kit_args == ["--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false"]
    assert resolved_args == ["--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=false"]


def test_explicit_simulation_manager_callback_setting_takes_precedence(monkeypatch: pytest.MonkeyPatch):
    explicit_arg = "--/exts/isaacsim.core.simulation_manager/enable_default_callbacks=true"

    kit_args, resolved_args = _resolve_kit_args(monkeypatch, {"kit_args": explicit_arg})

    assert kit_args == [explicit_arg]
    assert resolved_args == [explicit_arg]
