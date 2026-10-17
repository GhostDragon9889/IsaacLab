# IRA 场景数值覆盖

[third_floor_cpu.usda](/project/EXP/Sence/Sences/IRA_Validated/third_floor_cpu.usda) 通过 USD sublayer 引用原三楼场景，保留原始几何、关节、质量、材料及 120 Hz。正式验证使用 CPU PGS、MBP、最小迭代 16/4、稳定化和原场景 22 个刚体的 0.1 m/s 穿透恢复上限。原始场景文件没有改写。

在 Isaac Sim 中打开该文件，再引用 [机器人](/project/EXP/Sence/Robot/EVT-EXP.usd)，使用 IRA 配置行人。机器人控制方式及 GUI ScriptNode 提示处理见 [使用说明](/project/EXP/Sence/Robot/README.md)。该场景覆盖无需外部启动脚本。

二楼、YOHO、NewMetro 的正式验证保留各自原始场景物理设置。完整 8 组 IRA 测试见 [测试报告](/project/EXP/Sence/Robot/diagnostics/ira_sences_20261008/README.md)。

本目录的 `third_floor_gpu_buffers.usda` 属于未通过的诊断候选，不用于正式运行。扩大 GPU 缓冲没有解决已观察到的失稳。
