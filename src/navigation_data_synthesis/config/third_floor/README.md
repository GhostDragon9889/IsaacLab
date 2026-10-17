# 三楼行人配置

在 IRA 中加载 `third_floor_case_1.yaml`，执行 **Set Up Simulation** 后播放。

`third_floor_case_3.yaml` 是五名行人原地待机的场景。两个场景的 `stop.time_range`
设为 `[60.0, 60.0]`，覆盖当前 `simulation_duration: 60.0`。
`standing_motion_library.usda` 引用原始动作库，并将中性站姿注册为 `Standing` 自定义动作。
设置层中的 `stationary_action = "Standing"` 让 `stop` 和机器人堵路等待播放该动作，
通过原生 `custom_action` 的默认忽略根动画模式保持位置，骨骼动画继续播放。
配置该字段后，Set Up Simulation 完成、尚未播放时也显示双臂下垂的初始站姿，
手臂距竖直方向约 10°，为手掌与躯干保留间隙。该预览使用当前 Human 骨架的重定向标签，
只在 session layer 临时绑定动画，不修改角色资产、骨骼静止姿态或蒙皮绑定。
播放前移除预览，由原生动作控制；停止后恢复下垂姿势，暂停时保留当前动作。
原始动作库和 SDK 文件无需修改。移除该字段可恢复原生 `Idle`；在当前场景中，
原生 `Idle` 的待机位移可能把行人的外观带入隔墙，仅延长待机时长不能消除此问题。

`case_1` 将 `walker_01` 的待机点移入空闲隔间，为增大后的身体碰撞体留出主通道。
`walker_02` 的初始待机点也移入空闲隔间，避免尚未出发时堵住 `walker_03` 的必经通道。
`walker_03` 的出生点远离右墙，并用转角路点经过隔墙端部；`walker_02` 通过主通道路点避开小便池隔板，在第 10 s 开始巡逻，
避免两者同时占用狭窄通路。最终目的地保持不变。

`character_body_colliders.usda` 的 `body_radius_scale = 2.30` 将 `/World/Characters`
下行人的身体胶囊碰撞体半径设为 SDK 原始胶囊半径的 2.3 倍，覆盖当前自然站姿的
中下部身体及手臂范围。`case_1` 和 `case_3` 共用此设置。
`case_1` 中三个行人的原始半径约为 0.140～0.146 m，当前约为 0.321～0.336 m。
相比此前的 2.50 倍，半径减小 8%，每侧约减少 2.8～2.9 cm；胶囊总高度保持不变。
原设置 `0.70` 只有约 0.10 m，明显小于身体外观，会在出现可见重叠后才实际接触。

设为 `1.0` 使用 SDK 原始半径；有效值为有限正数，且缩放后的半径不能超过胶囊总高度的一半。
该参数是碰撞尺寸比例，角色外观保持原尺寸。单胶囊仍是近似接触形状，不逐个模拟手指和肢体；
大幅摆臂、跨步等姿态不保证每个外观顶点都位于胶囊内。

此设置保持胶囊总高度、中心和角色外观。它通过本项目的
`config/simulation_setup_script/character_body_colliders.py` 行为脚本在物理形状初始化时应用，支持反复播放，
不会累积缩放。修改设置文件后，重新执行 **Set Up Simulation**。

当前采用“减速、等待、让路后继续”的模式，关闭 SDK 的高开销实时避让和自动闪避：

```usda
custom bool obstacle_avoidance_enabled = false
custom bool auto_avoidance_enabled = false
custom string stationary_action = "Standing"
custom rel robot_yield_body = </World/EVT_EXP/base_link>
custom double robot_yield_clearance = 0.04
custom double pedestrian_yield_clearance = 0.08
custom double robot_yield_stop_distance = 0.35
custom double robot_yield_slow_distance = 1.0
custom double robot_yield_min_walk_speed = 0.5
custom bool separate_robot_visuals = true
```

脚本保留 NavMesh 路径规划和身体物理碰撞，同时禁用 `Physics/AvoidanceTrigger`
的碰撞检测，避免继续生成自动避让的重叠触发事件。行人不主动绕开运行时障碍物；
静态墙体仍由烘焙的 NavMesh 约束路径。
NavMesh 约束导航位置，身体胶囊和动画外观还需要转角净空；配置路点时应运行下面的场景碰撞测试。

设置层中的 `EVTRobotContactMargins` 对当前 EVT 资产的车身和机械臂碰撞体设置
4 cm 接触检测距离（`physxCollision:contactOffset`）、2 cm 静止接触余量（`physxCollision:restOffset`），
给墙体、洗手台等表面保留间隙。当前场景 USD 单位为 m，要求 `0 <= restOffset < contactOffset`。
轮子和滚子沿用资产接触参数，保留原来的落地高度。覆盖层在场景加载时生效，支持停止重播，
不修改机器人资产文件；替换机器人资产时需重新检查该层中列出的碰撞体路径。

`character_body_colliders.usda` 同时加载 `robot_collision_proxies.usda`，使用独立的
25 个物理凸包（底盘 16 个，其余连杆共 9 个），合计 705 个顶点、约 1310 个三角形。
它们由当前 EVT 网格通过 SDK 的 `request_convex_collision_representation` 接口烘焙，
按连杆局部坐标保存；源资产及其 SHA-256 记录在该文件元数据中。
原 CAD 节点的重复碰撞在设置层关闭，新凸包沿用上述接触余量。
轮子、滚子、关节、刚体及质量和惯量属性保持原值。
代理的 purpose 为 `proxy`，且不可见；高精显示模型保持不变。更换或修改机器人几何时需重新生成该代理层。

`config/simulation_setup_script/robot_yield.py` 使用 PhysX 的机器人实时位姿和矩形占地范围检测前方通路。
占地范围来自机器人初始外观包围盒，并加上行人的身体半径和 `clearance`。
`robot_yield_clearance = 0.04` 是身体半径之外的额外 4 cm 余量，不改变物理碰撞体。
此前 8 cm 的余量会在机器人靠隔间墙转向停放、通路仍有约 7 cm 身体净空时持续判定受阻。
减小该余量后，有足够净空的行人可以继续；机器人仍占据路线时仍会等待。
距离参数单位均为 m：距该范围 1.0 m 内降至巡逻速度的 40%，
并通过 `robot_yield_min_walk_speed` [m/s] 将减速后的行走速度限制在至少 0.5 m/s，
且不会超过原巡逻速度。
0.35 m 内进入自然站立等待。机器人让开、前方距离超过 0.50 m 后，恢复原巡逻目标。
等待过程保留巡逻步骤，不会因为 `MoveTo` 结束而跳过目标。
检测针对关系指定的一台机器人；更换机器人时更新 `robot_yield_body`。

当前角色与动作库组合在约 0.27 m/s 下可出现近站姿平移，walker_02 在 0.4 m/s 下也可出现
持续低幅腿部动作的平移。原 40% 减速会将本场景
0.5～1.0 m/s 的巡逻速度降至 0.2～0.4 m/s。0.5 m/s 下限用于避开本轮测得的
低速异常区间，并已对照机器人逐渐让路的场景；它不是所有角色或动作库的通用速度保证。
移除该字段或设为 `0` 可恢复原来的 40% 减速。显式配置低于此下限的巡逻速度仍保持原值，
需要另行验证对应动作库的低速步态。

`pedestrian_yield_clearance` [m] 将同一设置层管理的其他行人也纳入减速与等待检测，
范围来自双方实际身体胶囊半径及此余量，并使用实时位置；不扫描场景三角网格。
单靠 IRA 的身体胶囊不能保证导航中的行人互相阻挡，关闭原生避让后需要此接近保护。
它复用上述停止、降速距离和恢复滞回，保留巡逻目标；移除此字段可关闭行人间等待。
此模式以等待代替超车或会车绕行，路线和待机点应为其他行人留出通路。

当前 SDK 在暂停后恢复播放时会重建 IRA 运行时代理。脚本按行为的 USD 路径保存并
恢复未完成的触发式巡逻剩余路径及后续步骤，避免一次性触发任务在恢复播放后丢失。
此适配基于项目当前的 Isaac Sim 6.1 IRA 运行时；停止后再次播放仍按 YAML 重新开始。

当前机器人有约 161 万个显示三角形。测试发现，仅关闭两个避让开关仍存在明显的
近距离开销，因此 `config/simulation_setup_script/robot_visuals.py` 在播放时将纯显示网格引用到机器人根节点的
`RenderVisuals` 下，避免它们作为刚体子网格进入运行时导航处理。
每次物理步结束后同步显示位姿，并用 `Sdf.ChangeBlock` 合并提交各网格的变换，
减少 USD 通知触发的重复处理。实际碰撞使用上述独立凸包，机器人原始资产文件保持不变；
停止播放时恢复原显示网格，显示分离修改仅存在于 USD session layer。
脚本不会搬移任何仍被启用的祖先碰撞节点使用的网格，避免显示分离早于物理初始化时丢失碰撞。
旧版本只检查网格自身与子节点的碰撞 API，会把 CAD 碰撞节点下的网格改成 Xform；
物理已加载时的缓存可能掩盖该问题，但之后初始化物理会导致底盘穿入高于车轮的柜体。
播放期间显示网格的 prim 路径会变化；依赖具体网格路径的标注器需要使用新路径。
不再需要此优化时，将 `separate_robot_visuals` 设为 `false`。

本机测试采用 RTX 4080 SUPER、640×360、无界面渲染。
实际 FPS 还取决于窗口分辨率、传感器、视角和其他运行负载。
恢复独立凸包后，独立 SDK 的 Full 场景本轮测得约 20 FPS，低于此前约 32 FPS；
当前配置尚未达到下面命令的 30 FPS 验收门槛。
批量显示同步的交替对照中，五名待机场景从约 15.0～15.4 FPS 提升至 17.3～17.4 FPS，
显示同步回调耗时减少约 60%；该结果含诊断监听器开销，不替代具体场景的完整性能验收。
关闭原生避让未消除凸包对应的运行时障碍处理开销；简单包围盒会扩大局部边界，
精细方块组合又显著降低 FPS，因此当前保留精确凸包，不能将旧版性能结果用于本版验收。

开关使用本机 SDK 的 `IBehaviorAgent.set_obstacle_avoidance_enabled()` 和
`set_auto_avoidance_enabled()`，参见 [Behavior API Reference](https://docs.omniverse.nvidia.com/kit/docs/behavior-simulation/110.1.1/behavior-simulation/api.html#obstacle-avoidance)。

IRA 的 YAML `colliders` 字段用于额外触发区域，不用于修改自动生成的身体碰撞体。

保留 `nav_blocked_timeout = 60.0` 作为其他原生导航任务的受阻超时设置 [s]。
脚本只在播放期间修改此 SDK 设置，在停止播放或移除脚本时恢复原值。
机器人等待逻辑使用站立动作保留巡逻步骤，因此等待本身不会消耗 `MoveTo` 的受阻超时。

使用真实场景机器人验证两名行人受阻后恢复行走：

```bash
uv run python src/navigation_data_synthesis/tests/test_blocked_navigation.py --expect-yield --isolate-group --stepping variable --min-fps 30 --pause-before-clear --block-seconds 30
```

测试分别用机器人阻挡 `walker_02`、`walker_03` 的目标区域，在仿真第 30 s 移开机器人，
检查持续等待、让路后到达原目标、行走任务成功、FPS、身体碰撞、机器人刚体和碰撞体、
显示网格及材质、物理与显示同步，以及停止与再次播放的恢复行为。
`--motion cross` 增加机器人穿过行走路线的场景，`--block-seconds` 设置让路时间 [s]。
`--motion side --block-seconds 15 --resume-seconds 15` 检查机器人从通道移至隔间墙侧、
转向 90° 后保持近距离停放的场景；`--resume-seconds` 设置让路后的观察时长 [s]。
`--pause-before-clear` 同时检查暂停后恢复播放时，巡逻任务和显示网格不会丢失或重复。
`--isolate-group` 在测试中暂时禁用其他行人，确保被测行人直接面对机器人，而不是排在另一名等待行人后方。
保留全部行人时省略此参数及 `--expect-yield`，检查多人排队后恢复到达；跟随者可能仍处于未完成的 `MoveTo`。
等待先用原生 `idle` 停止导航，再切换站立动画，避免前一条导航目标继续推动站立动作。

检查八名行人的实际 PhysX 接触边界、当前骨骼姿态的身体网格范围，以及停止重播一致性：

```bash
uv run python src/navigation_data_synthesis/tests/test_character_contact_boundary.py
```

该测试通过六个方向的 PhysX 射线测量实际胶囊，同时用实时关节变换重建蒙皮网格，
检查 25%～60% 身高范围内的外观包络。它不会用未动画的绑定姿态代替当前身体位置。

检查行人待机位置、动画连续性、外观穿墙与最终目的地，以及机器人持续轮速命令下的环境接触：

```bash
uv run python src/navigation_data_synthesis/tests/test_scene_collision.py
```

机器人分别驶向固定墙、洗手台、隔墙和活动门，测试读取真实 PhysX 接触数据和车身外观间隙。
行人测试逐帧检查两两身体胶囊间距（允许 2 cm 接触误差），并要求两名巡逻行人到达原目标。
行人测试先用胶囊重叠筛选，再对实时蒙皮顶点与墙面做射线检查，避免把保守胶囊重叠直接当成外观穿墙。
可用 `--pedestrians-only` 或 `--robot-only` 单独复测。默认 `--controller wheels` 直接驱动轮关节。

通过资产内 `/World/EVT_EXP/Gamepad` 的原混合、限幅、保护和 `ArticulationController` 节点验证接触：

```bash
uv run python src/navigation_data_synthesis/tests/test_scene_collision.py --robot-only --controller gamepad
```

此模式在设备读取节点的轴值输出处回放输入，原驱动图保持激活，额外检查横移驶近洗手台。
洗手台检查覆盖柜体正面、把手位置、南侧端面及拐角；每 0.25 s 检查全车显示顶点
是否进入柜体、台面或把手的实体内部超过 3 mm，检查不是仅针对底盘中部的最终位置。
可用 `--washstation-sweep --contact-seconds 10` 扩展底盘朝向并在首次接触后持续驱动 10 s，
或用 `--drive-cases washstation_south washstation_south_lateral washstation_corner` 单独复测端面与拐角。
正向与倒退驶近柜体可用 `--washstation-sweep --drive-cases washstation washstation_yaw_180 --contact-seconds 10` 复测。
加上 `--visuals-before-physics` 可强制先分离显示、再创建物理形状，检查上述初始化顺序回归。
场景测试还逐个从真实 PhysX 查询独立凸包，避免仅检查 USD 上的碰撞 API 是否存在。
16 组输入覆盖前后、左右横移、转向、混合满摇杆、松手归零、非法轴值停车及回中后重新使能。
当前资产的前后及横移速度指令上限为 0.5 m/s，转向指令上限为 0.6 rad/s，轮速目标限制为 ±15 rad/s。
测试同时核对原图输出与 PhysX articulation 中实际接收的轮速目标。
回放节点和连接修改只写入测试的 USD session layer，不保存到资产文件。
无界面环境下的虚拟手柄未能被原生设备读取节点采集，因此此模式不覆盖实体手柄硬件采集和断连事件。

检查机器人在主通道挡路 20 s、行人排队以及机器人移开后的到达情况：

```bash
uv run python src/navigation_data_synthesis/tests/test_scene_collision.py --pedestrians-only --duration 55 --block-passage-seconds 20
```

`test_scene_collision.py`、`test_blocked_navigation.py` 和 `test_character_contact_boundary.py`
均接受 `--experience <同一 SDK 中的 .kit 路径>`，
用于匹配 Kit 启动配置；该参数本身不切换 Python 或原生 SDK。
