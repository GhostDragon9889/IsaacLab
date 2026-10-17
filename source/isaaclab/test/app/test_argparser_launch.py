# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
import sys

import pytest
from standalone_script_cases import run_until_ready

from isaaclab.app import AppLauncher

pytestmark = pytest.mark.integration


def test_livestream_launch_with_argparser():
    """Test launching with argparser arguments."""
    # Kit fast shutdown exits Python; keep pytest and its report writer in the parent.
    code = """
import argparse
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
for name in AppLauncher._APPLAUNCHER_CFG_INFO.keys() - {"headless", "enable_cameras"}:
    assert parser._option_string_actions[f"--{name}"]
launcher = AppLauncher(parser.parse_args(["--livestream", "1"]))
assert launcher._livestream == 1
assert launcher._headless is True
print("LIVESTREAM_READY", flush=True)
launcher.app.close()
"""
    result = run_until_ready([sys.executable, "-c", code], "LIVESTREAM_READY")
    assert result.ready and result.returncode == 0 and not result.fatal_patterns, result.output


def test_visualizer_alias_parsing():
    """Test that --viz alias maps to visualizer values."""
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)

    args = parser.parse_args(["--viz", "kit,newton"])
    assert args.visualizer == ["kit", "newton_gl"]
    assert args.visualizer_explicit is True


@pytest.mark.parametrize("deprecated_arg", ["--headless", "--enable_cameras"])
def test_deprecated_render_flags_are_rejected(deprecated_arg: str):
    """Test that removed render flags are rejected by the parser."""
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)

    with pytest.raises(SystemExit):
        parser.parse_args([deprecated_arg])


def test_help_on_parser_with_required_positionals(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture):
    """Launcher arguments reach the help output of a script that takes required positionals.

    ``add_app_launcher_args`` probes the command line to check for name collisions. That probe
    exits when a required argument is missing, which is the case for every tool script invoked
    with ``--help``, so it must not take the parser down before the arguments are added.
    """
    monkeypatch.setattr("sys.argv", ["convert_urdf.py", "--help"])
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("output")

    AppLauncher.add_app_launcher_args(parser)

    # the probe's own usage line must not leak to stderr ahead of the real help output
    assert capsys.readouterr().err == ""
    assert "--visualizer" in parser._option_string_actions
    assert "--device" in parser._option_string_actions

    with pytest.raises(SystemExit) as excinfo:
        parser.parse_args(["--help"])
    assert excinfo.value.code == 0
    assert "app_launcher arguments" in capsys.readouterr().out


@pytest.mark.parametrize("value", ["none", "None"])
def test_visualizer_none_parsing(value: str):
    parser = argparse.ArgumentParser()
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args(["--viz", value])
    assert args.visualizer is None
    assert args.visualizer_explicit is True
