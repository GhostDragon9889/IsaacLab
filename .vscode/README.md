# 项目 VS Code 使用说明

从 `/project/EXP/IsaacLab` 打开工作区，使用 `env_isaaclab/bin/python3` 解释器。
工作区推荐扩展包括 Python、Pylance、Python Debugger、Ruff、YAML、Even Better TOML 和 autoDocstring。

| 操作 | 入口 | 行为 |
| --- | --- | --- |
| 查看导入源码 | F12 或 Ctrl+单击 | 支持 Isaac Lab、Isaac Sim、Omni 及项目 `src` 中的模块 |
| Python 格式化及导入整理 | 保存文件 | 使用 Ruff 和项目 `pyproject.toml` |
| YAML / TOML 格式化 | 保存文件 | YAML 使用 2 空格缩进，`.kit` 按 TOML 编辑 |
| 运行当前 Python 文件 | 任务 `Python: Run Current File (uv)` | 使用项目环境和工作区根目录 |
| 调试当前 Python 文件 | F5 → `Python: Current File` | 支持断点、单步、变量检查及进入第三方 Python 源码 |
| 调试导航启动脚本 | F5 → `Navigation: Spawn Simulation` | 启动固定的 `spawn_simulation_sence.py`，使用脚本本身指定的场景和配置 |
| 调试当前 pytest 文件 | F5 → `Python: SinglePytest` | 在当前测试文件中设置断点 |
| 编辑器检查 | 测试资源管理器，或默认测试任务 | 默认发现编辑器和项目元数据检查，不启动仿真 |
| 运行当前 pytest 文件 | 任务 `Python: Test Current File (uv)` | 适用于真正的 pytest 测试文件 |
| 完整仿真测试入口 | 任务 `Isaac Lab: Full Tests (isolated processes)` 或 F5 → `Python: ALL Pytest` | 使用项目 `tools` 调度器，为测试文件创建独立进程 |
| 完整仓库格式检查 | 任务 `run_formatter` | 执行项目 pre-commit，可能格式化仓库文件 |
| 检查解释器与工作目录 | 任务 `Python: Check Environment (uv)` | 输出 Python、环境目录、cwd 和 uv 环境 |
| 重新生成编辑器配置 | 任务 `setup_python_env` | 更新解释器和 SDK 源码路径 |

测试资源管理器可手动刷新、运行或调试默认的检查。复杂仿真测试通过独立进程任务执行。
导航调试入口不会根据当前打开的 YAML 自动切换场景；它遵循启动脚本的逻辑。
训练与回放入口保留在调试列表中；回放需要已有的训练检查点。

`.vscode/python.env` 为 Python 测试与调试提供项目 `src` 路径，并关闭无关 pytest 插件的自动加载。
终端和任务使用 `uv run --no-sync`，复用现有环境。

Linux 新终端默认使用 `IsaacLab` 配置：先加载用户 `.bashrc`，退出自动激活的 Conda
`base`，再激活项目 `env_isaaclab`。新建终端与拆分终端会显示 `(env_isaaclab)`，直接
运行 `python` 即使用项目解释器；不依赖先打开 Python 文件。用户 `.bashrc` 无需修改。
更新设置后关闭旧终端并新建终端；需要手动选择时，在终端下拉菜单选择 `IsaacLab`。

`Python: Debugger Attach` 连接本机 `127.0.0.1:3000` 上已启动的 debugpy 服务。
F5 使用 Python Debugger 扩展自带的 debugpy；终端中的 `python -m debugpy` 则需要该解释器安装 debugpy。
附加调试时，如需在 `.python-libs` 中设置断点，也应给被调试的目标进程设置 `PYDEVD_RESOLVE_SYMLINKS=1`。

修改共享设置时编辑 `tools/settings.template.json`；修改共享调试入口时编辑 `tools/launch.template.json`。
重新生成配置会保留已有的 `launch.json`；如需更新已有调试入口，应将模板中的对应配置合并到该文件。
机器相关的 `settings.json`、`launch.json`、根目录 `pyrightconfig.json` 和 `.python-libs` 已忽略。
`.python-libs` 是指向 SDK 源文件的编辑器索引，不加入运行时 `PYTHONPATH`。
`python.env` 中的 `PYDEVD_RESOLVE_SYMLINKS=1` 让调试器识别这些源码链接上的断点。
`.taplo.toml` 将 `.kit` 加入 TOML 工具处理范围；YAML/TOML 默认使用本地语法校验，不自动下载 Schema。

二进制 `.usd` 文件使用 Isaac Sim 编辑；文本 `.usda` 可以在 VS Code 中查看。
