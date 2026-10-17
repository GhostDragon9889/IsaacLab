# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for editor path discovery without starting the simulator."""

import json
import pathlib
from importlib.machinery import PathFinder

import pytest

from isaaclab.utils.editor import read_isaacsim_extra_paths, resolve_isaacsim_dir, setup_editor

pytestmark = pytest.mark.unit


def test_resolve_isaacsim_dir_accepts_pip_install_without_editor_settings(tmp_path):
    """A packaged SDK can be recognized without binary-install editor metadata."""
    sdk = tmp_path / "isaacsim"
    for name in ("apps", "exts", "kit"):
        (sdk / name).mkdir(parents=True)

    assert resolve_isaacsim_dir(tmp_path, str(sdk)) == sdk


def test_read_isaacsim_extra_paths_discovers_pip_extension_roots(tmp_path):
    """Discover extension roots and Carbonite bindings when no settings file ships."""
    relative_paths = (
        "kit/kernel/py",
        "exts/isaacsim.simulation_app",
        "exts/isaacsim.replicator.agent.core",
        "extsDeprecated/isaacsim.core.utils",
        "extscache/omni.usd-1.0",
        "kit/exts/omni.kit.example",
        "kit/extscore/omni.client",
    )
    expected = {tmp_path / path for path in relative_paths}
    for path in expected:
        path.mkdir(parents=True)
    (tmp_path / "exts/README.txt").write_text("Not an extension directory.")

    assert set(read_isaacsim_extra_paths(tmp_path)) == expected


@pytest.mark.parametrize("sdk_layout", ["site-packages/isaacsim", "standalone"])
def test_setup_editor_merges_sdk_namespace_sources(tmp_path, monkeypatch, sdk_layout):
    """The SDK bootstrap and split extension sources must be reachable in one package."""
    sdk = tmp_path / sdk_layout
    namespace = sdk / "exts/isaacsim.ros2.control/isaacsim"
    namespace.mkdir(parents=True)
    (namespace / "__init__.py").write_text("")
    bootstrap = sdk if sdk_layout == "site-packages/isaacsim" else sdk / "python_packages/isaacsim"
    bootstrap.mkdir(parents=True, exist_ok=True)
    sdk_init = bootstrap / "__init__.py"
    sdk_init.write_text("SimulationApp = object\n")
    for name in ("apps", "kit"):
        (sdk / name).mkdir()
    modules = {
        "exts/isaacsim.core/isaacsim/core/api.py": "class World: ...\n",
        "extsDeprecated/isaacsim.core.utils/isaacsim/core/utils/extensions.py": "def enable_extension(): ...\n",
        "extscache/isaacsim.replicator.agent.core/isaacsim/replicator/agent/core/api.py": "def load_config(): ...\n",
    }
    for relative_path, content in modules.items():
        module = sdk / relative_path
        module.parent.mkdir(parents=True, exist_ok=True)
        module.write_text(content)
    monkeypatch.setattr("isaaclab.utils.editor.find_isaaclab_package_paths", lambda: [])

    setup_editor(tmp_path, str(sdk))
    paths = json.loads((tmp_path / "pyrightconfig.json").read_text())["extraPaths"]
    spec = PathFinder.find_spec("isaacsim", [str(tmp_path / path) for path in paths])

    assert spec is not None
    assert spec.origin is not None
    assert pathlib.Path(spec.origin).resolve() == sdk_init
    package = pathlib.Path(spec.origin).parent
    for relative_path in modules:
        module_path = relative_path.split("/isaacsim/", 1)[1]
        assert (package / module_path).resolve() == sdk / relative_path


def test_read_isaacsim_extra_paths_preserves_sdk_settings(tmp_path):
    """Use an SDK's own path list when editor metadata is available."""
    settings = tmp_path / ".vscode/settings.json"
    settings.parent.mkdir()
    settings.write_text(json.dumps({"python.analysis.extraPaths": ["exts/custom", str(tmp_path / "absolute")]}))
    (tmp_path / "exts/unlisted").mkdir(parents=True)

    assert read_isaacsim_extra_paths(tmp_path) == [tmp_path / "exts/custom", tmp_path / "absolute"]
