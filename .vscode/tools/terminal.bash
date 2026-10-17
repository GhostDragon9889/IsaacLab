# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Load user aliases and shell configuration before activating the project environment.
if [[ -r "$HOME/.bashrc" ]]; then
    source "$HOME/.bashrc"
fi

if [[ "${CONDA_DEFAULT_ENV:-}" == "base" ]] && declare -F conda >/dev/null; then
    conda deactivate
fi

_isaaclab_workspace="$(builtin cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -r "$_isaaclab_workspace/env_isaaclab/bin/activate" ]]; then
    source "$_isaaclab_workspace/env_isaaclab/bin/activate"
    export UV_PROJECT_ENVIRONMENT="$_isaaclab_workspace/env_isaaclab"
else
    printf 'Isaac Lab environment not found: %s\n' "$_isaaclab_workspace/env_isaaclab" >&2
fi
unset _isaaclab_workspace

# A custom rcfile needs manual VS Code shell integration.
if [[ "${TERM_PROGRAM:-}" == "vscode" && -z "${VSCODE_SHELL_INTEGRATION:-}" ]] && command -v code >/dev/null 2>&1; then
    _isaaclab_shell_integration="$(command code --locate-shell-integration-path bash 2>/dev/null)"
    if [[ -r "$_isaaclab_shell_integration" ]]; then
        source "$_isaaclab_shell_integration"
    fi
    unset _isaaclab_shell_integration
fi
