# Tianji IO Teleop Bridge

天机 Marvin M6 CCS 双臂机器人与艾欧智能 IO TeleXperience 遥操作设备之间的 ROS 2 Humble 桥接节点。

本项目完成以下工作：

- 从 Marvin SDK 读取双臂 14 个关节的位置、速度和扰动估计，并在同一条
  `/io_teleop/joint_states` 消息的 `position`/`velocity`/`effort` 中发布；
- 订阅艾欧设备发布的 `/io_teleop/joint_cmd`，按关节名称解析双臂目标角；
- 经过显式授权、阻抗模式确认、初始对齐和多层安全检查后，以关节阻抗模式控制机器人；
- 提供独立的一键回初始姿态服务，按同一套阻抗参数和限速平滑移动，完成后退出到空闲模式；
- 可选地根据关节扰动估计和运动学模型计算左右臂法兰处的六维等效力/力矩，
  该功能默认关闭；
- 发布控制状态和逐关节对齐诊断，并提供终端对齐面板。
- 提供独立只读的双臂力估计 HUD：消费现有 `joint_states`，以30 Hz发布JPEG压缩图像；
  默认使用力圆环，保留六轴条形图调试模式；方向未确认时仅显示合力大小。
  人工方向观察可使用 `config/force_hud_direction_preview.yaml`，显式显示未标定箭头，
  不代表方向已确认；步骤与注意事项见 [HUD说明](docs/FORCE_HUD.md#未标定方向预览人工方向观察)。
  插孔任务使用 `config/force_hud_insertion_preview.yaml`：固定任务坐标旋转、接触侧箭头、
  轴向力大小、显示死区和分臂手动去零；当前场景人工方向验证已通过，仍保留未完整标定提示，见
  [插孔任务视图](docs/FORCE_HUD.md#10-插孔任务视图固定孔口坐标系)。
  不建立机器人控制连接、不改变原遥操作状态机。见 [HUD使用与验证指南](docs/FORCE_HUD.md)。

> **安全提示**：`config/bridge.yaml` 默认禁止所有运动。`config/bridge_both_test.yaml`
> 是已经打开部分人工确认门的实机联调配置（当前只启用右臂），不是无人值守生产配置。首次调试应清空机器人周围区域、
> 准备物理急停，并由熟悉天机控制器的人员在场。

## 1. 系统数据流

```text
Marvin SDK feedback
  ├─ joint position/velocity + m_EST_Joint_Force
  │    └───────────────────────────────────> /io_teleop/joint_states
  └─ joint position + disturbance + model ─> ~/left|right/estimated_cartesian_disturbance
                                            (default disabled)

IO TeleXperience
  └─ /io_teleop/joint_cmd
       └─ name mapping → validation → alignment → slew limiting → Marvin joint target API

~/move_to_initial_pose
  └─ configured initial pose → safety checks → slew limiting → Marvin joint target API
```

Marvin A 臂对应左臂，B 臂对应右臂。标准关节顺序为：

```text
Joint1_L ... Joint7_L, Joint1_R ... Joint7_R
```

收到的命令按 `JointState.name` 映射，不依赖输入数组顺序。SDK 使用角度和角度每秒，ROS 接口使用弧度和弧度每秒。

## 2. ROS 接口

以下私有名称中的 `~` 展开为 `/tianji_marvin_teleop_bridge`。

| 方向 | 名称 | 类型 | 默认频率 | 说明 |
|---|---|---|---:|---|
| 发布 | `/io_teleop/joint_states` | `sensor_msgs/msg/JointState` | 默认100 Hz；当前联调300 Hz | 双臂14关节反馈；`effort` 为 `m_EST_Joint_Force` |
| 订阅 | `/io_teleop/joint_cmd` | `sensor_msgs/msg/JointState` | 艾欧约100 Hz | 双臂目标角；共享话题中的手指消息会被忽略 |
| 可选发布 | `~/left/estimated_cartesian_disturbance` | `geometry_msgs/msg/WrenchStamped` | 跟随 `publish_rate_hz` | 左臂法兰处计算得到的等效六维扰动力；默认关闭 |
| 可选发布 | `~/right/estimated_cartesian_disturbance` | `geometry_msgs/msg/WrenchStamped` | 跟随 `publish_rate_hz` | 右臂法兰处计算得到的等效六维扰动力；默认关闭 |
| 发布 | `~/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray` | 10 Hz | 汇总状态和14个逐关节对齐状态 |
| 发布 | `~/control_status` | `std_msgs/msg/String` | 1 Hz | 状态机、安全门、计数器和阻塞原因 |
| 服务 | `~/enable_control` | `std_srvs/srv/SetBool` | 按需 | 显式启用或关闭控制 |
| 服务 | `~/move_to_initial_pose` | `std_srvs/srv/Trigger` | 按需 | 从 `MONITOR` 异步移动到配置的初始姿态，完成后退出到IDLE |

`/io_teleop/joint_cmd` 同时承载艾欧左右手手指消息：

- 完全不含天机手臂关节名称的消息计入 `ignored_non_arm`，静默忽略；
- 只包含部分天机手臂关节的消息属于畸形手臂指令，会被拒绝；
- 有效手臂指令必须恰好包含全部14个标准关节名称和14个有限位置值；
- 等待对齐期间允许目标与实机位置相差较大；进入 `ACTIVE` 后才启用相邻指令跳变限制。

## 3. 控制状态机

```text
MONITOR
  ├─ enable_control=true
       → PREPARING_PARAMETERS
       → CONFIGURING_IMPEDANCE
       → WAITING_FOR_ALIGNMENT
       → ACTIVE ←→ SOFT_LIMIT_HOLD
           ├─ 输入无效/超时、反馈暂不可用、SDK暂忙超时 → COMMAND_WAIT ─重新对齐→ ACTIVE
           └─ 停用/其他保护 → HOLD → IDLE → MONITOR
  └─ move_to_initial_pose
       → PREPARING_PARAMETERS
       → CONFIGURING_IMPEDANCE
       → RETURNING_TO_INITIAL_POSE
       → HOLD → IDLE → MONITOR
       任一不可恢复错误 ───────→ FAULT
```

控制启用过程如下：

1. `enable_control=true` 通过四个显式运动门、SDK连接、反馈新鲜度和机器人健康检查；
2. 节点设置并读回确认关节阻抗 K/D、速度比例和加速度比例；
3. 节点发送纯模式切换报文，只包含 `ARM_STATE_TORQ` 和 `ARM_IMP_JOINT`，不包含关节位置目标；
4. 控制器回显确认关节阻抗模式后，节点进入 `WAITING_FOR_ALIGNMENT`；
5. 此时不要求 TeleBox 已经发送手臂指令，可以一直等待，而且不会下发关节目标；
6. 有效目标进入安全限位范围，并与启用手臂反馈连续对齐指定帧数后进入 `ACTIVE`；
7. 进入 `ACTIVE` 后才第一次发送经过速度/加速度限制的遥操作目标。

默认配置要求20个连续对齐帧。`bridge.yaml` 的初始容差为 `0.05 rad`，双臂联调配置为
`0.25 rad`。无新鲜命令会把连续对齐计数清零，但在 `WAITING_FOR_ALIGNMENT` 中不会退出阻抗模式。

进入 `ACTIVE` 后，下列情况会进入保持/停止路径：

- 手臂指令被拒绝或有效指令超过 `command_timeout_sec` 未更新：进入 `COMMAND_WAIT`，不自动IDLE；
- 遥操作中反馈读取失败、反馈帧停止更新或本地快照过期：进入 `COMMAND_WAIT`，不主动请求IDLE；
- 关节目标包SDK暂忙超过重试期限：进入 `COMMAND_WAIT`，不主动请求IDLE；
- 反馈含非有限值或机器人报告错误：仍走故障停止；
- 实际位置相对已发送目标的跟随误差过大；
- SDK控制报文发送失败；
- 操作员调用 `enable_control=false`。

目标进入关节软限位时不会立刻退出遥操作，而是进入 `SOFT_LIMIT_HOLD`：默认冻结所有启用手臂，
目标回到更大的释放裕量、重新对齐并稳定指定帧数后自动恢复 `ACTIVE`。机器人故障和急停错误不会由本节点自动清除。

`COMMAND_WAIT` 处理遥操作输入/反馈暂不可用以及SDK暂忙超时：保持阻抗模式，不发送新的目标或模式报文，
由控制器保持**最后成功提交的目标**，不追踪最后收到但未实际提交的目标。
保持目标不等于立即停止在实际位置，机械臂可能仍向最后目标运动。若尚未提交过任何目标，
等待期间也不会为了“保持”而额外发送位置包。
等待不受 `hold_duration_sec` 限制；进入等待时结束本轮SDK暂忙重试，不再因旧期限届满而请求IDLE。
反馈不可用期间禁止目标下发，不使用旧反馈恢复控制，也不自动清除机器人故障或重新使能。
持续失去反馈时只能保证桥接不主动发送IDLE，**不能确认机器人实际保持状态**；应暂停操作并检查通信，
必要时使用实机急停。控制器自己的安全保护不受本节点控制。

等待期间仍校验14关节名称、有限值和硬限位，不再用旧的接受目标作跳变基准。
新目标必须在 `soft_limit_release_margin_rad` 内，并且每个启用关节与实时反馈之差不超过
`command_recovery_alignment_rad`（默认0.10 rad，约5.73°）。连续检查通过
`alignment_required_frames`（默认20）条新的有效指令才自动恢复；重复检查同一缓存指令不累计。
无效、超时或未对齐会打断连续计数；恢复从保持目标开始重新限速，不直接跳向新目标。
若等待由反馈异常触发，先重新读取并校验反馈，确认双臂反馈帧都相对异常时发生更新，
丢弃反馈中断期间收到的命令，再开始对新命令累计对齐。机器人错误、模式丢失、
非有限反馈和跟随误差保护仍保留；SDK暂忙恢复不通过空报文探测，只在重新对齐后受限尝试下发，
若仍暂忙则跳过更新，期限届满后再次等待。
对齐面板在COMMAND_WAIT显示恢复容差、限位裕量和计数。纯手指消息仍忽略，不触发等待或计数重置。
操作员调用 `enable_control=false` 仍进入普通HOLD并请求IDLE；若要回初始姿态，请先停用并等到MONITOR。
以上“不主动IDLE”的恢复策略仅用于已进入阻抗遥操作的运行/等待阶段，
不改变参数或模式设置失败、回初始姿态、手动停用以及节点退出的原有停止处理。

`move_to_initial_pose` 与遥操作互斥，只允许从 `MONITOR` 调用。服务返回成功表示任务已被接受，
不表示机器人已经到位。节点会先配置并确认关节阻抗模式，再从当前反馈位置开始按软件速度和加速度
限制移动；执行期间不会跟随艾欧目标。到位并连续稳定后，节点短暂保持当前位置、请求IDLE并回到
`MONITOR`。之后如需遥操作，仍须单独调用 `enable_control=true` 并完成对齐。

## 4. 扰动力估计

### 4.1 关节扰动力

`/io_teleop/joint_states` 在同一条消息中发布以下三个14维数组：

- `position`：双臂关节角，单位 rad；
- `velocity`：双臂关节速度，单位 rad/s；
- `effort`：Marvin `m_EST_Joint_Force`，按相同关节名称和索引对应。

三个数组统一使用 `Joint1_L ... Joint7_L, Joint1_R ... Joint7_R` 顺序，并来自同一个 Marvin
反馈帧。这里的 `effort` 是控制器的关节扰动估计，不是电机实际反馈扭矩，也不是末端力传感器测量值。
如果某一帧扰动估计包含非有限值，节点仍会发布该帧的名称、位置和速度，但将 `effort` 留空；不会发布部分长度的数组。

### 4.2 末端六维等效力

该功能默认关闭：节点不创建左右臂 `WrenchStamped` 发布器，也不执行运动学换算。
需要时将 `publish_cartesian_disturbance` 设为 `true` 并重启节点。

机器人当前没有安装末端六维力传感器，因此左右臂 `WrenchStamped` 由以下信息计算：

- 当前SDK关节角（度）；
- 同一反馈帧的 `m_EST_Joint_Force`；
- M6 CCS 4.0 运动学配置；
- Marvin `FX_Robot_JntTau2EETau()`。

对应关系为 `tau = J(q)^T wrench`。当前使用单位工具变换，结果表示法兰处等效力，`force` 使用 N，
`torque` 使用 N·m。由于坐标系名称和符号约定尚未最终确认，两个消息的 `frame_id` 默认留空。

这些数据可能包含静态偏置、摩擦、模型误差和噪声，不能等同于标定力传感器。运动学奇异或输入非有限时，
只跳过对应手臂当前一帧的 Wrench，不影响关节状态、关节扰动力或控制状态机。

## 5. 配置文件

新增左右臂工具参数服务（**节点默认及 `bridge.yaml` 仅预览，联调配置可真实上传**）：
`~/configure_left_tool`、`~/configure_right_tool`。默认参数按供应商JSON的 `current_tool` 选择，
`bridge.yaml` 为 `tool_config_dry_run: true`；`bridge_both_test.yaml` 和
`bridge_right_retreat_test.yaml` 当前为 `false`，服务在其他准入条件通过后会真实上传。
启动不会自动上传。参数明细、离线预览及真实下发代码的审核说明见
[工具配置审核文档](docs/TOOL_CONFIGURATION_REVIEW.md)。这两个服务不自动改变人工 `tool_dynamics_confirmed` 标志。

### `config/bridge.yaml`

安全默认配置：

```yaml
allow_motion: false
sdk_control_compatibility_confirmed: false
tool_dynamics_confirmed: false
enabled_arms: "both"
retreat_duration_sec: 10.0
```

即使连接机器人，也只能读取反馈、发布状态和检查命令，不会发送模式、阻抗、空闲或关节目标报文。

正式配置集中包含左右臂阻抗参数，以及两个独立的 10 cm 后退服务。运动能力和人工确认仍默认关闭；
启用后才可控制。`velocity_ratio` / `acceleration_ratio` 配置为50，与已验证的联调配置一致。

### `config/bridge_both_test.yaml`

实机联调配置（沿用历史文件名，当前仅启用右臂）：

```yaml
allow_motion: false
sdk_control_compatibility_confirmed: true
tool_dynamics_confirmed: true
enabled_arms: "right"
tool_config_dry_run: false
initial_alignment_tolerance_rad: 0.25
joint_limit_safety_margin_rad: 0.005
soft_limit_release_margin_rad: 0.015
publish_rate_hz: 300.0
control_rate_hz: 100.0
```

配置文件中的 `allow_motion` 仍为 `false`，必须在启动命令中显式传入 `allow_motion:=true` 才能开放运动能力。
两个 `*_confirmed` 参数是人工确认标志，不会替代实际的SDK兼容性和工具负载检查。
其中两个限位裕量和300 Hz状态发布是当前联调专用设置；控制状态机及SDK目标下发上限仍为
100 Hz。需要双臂控制时，必须核对两侧条件并显式将本机配置的 `enabled_arms` 改为 `both`；
该选项不影响双臂14关节状态发布。安全默认配置
`bridge.yaml` 仍使用 `0.01 rad` 和 `0.03 rad`。URDF硬限位以及
`joint_limit_tolerance_rad` 均未放宽。

## 6. 参数说明

以下默认值来自 `bridge.yaml`。

### 通信、话题与频率

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_ip` | `192.168.1.190` | Marvin控制器IP |
| `state_topic` | `/io_teleop/joint_states` | 艾欧订阅的关节状态话题 |
| `command_topic` | `/io_teleop/joint_cmd` | 艾欧发布的共享控制话题 |
| `publish_cartesian_disturbance` | `false` | 是否创建并发布左右臂末端六维力估计 |
| `left_cartesian_disturbance_topic` | `~/left/estimated_cartesian_disturbance` | 左臂六维扰动力话题 |
| `right_cartesian_disturbance_topic` | `~/right/estimated_cartesian_disturbance` | 右臂六维扰动力话题 |
| `left_cartesian_disturbance_frame_id` | 空 | 左臂Wrench坐标系；确认后再填写 |
| `right_cartesian_disturbance_frame_id` | 空 | 右臂Wrench坐标系；确认后再填写 |
| `kinematics_config_file` | 空 | 空值使用安装包内校验过的 `ccs_m6_40.MvKDCfg` |
| `alignment_diagnostics_topic` | `~/alignment_status` | 对齐诊断话题 |
| `publish_rate_hz` | `100.0` | SDK反馈读取及 `/io_teleop/joint_states` 发布频率；允许最高1000 Hz |
| `control_rate_hz` | `100.0` | 控制状态机及SDK目标下发频率；允许最高200 Hz且不得高于发布频率 |
| `alignment_diagnostics_rate_hz` | `10.0` | 对齐诊断频率 |
| `reconnect_period_sec` | `2.0` | SDK连接失败后的重试周期 |
| `connect_on_start` | `true` | 节点启动后是否连接机器人 |
| `subscribe_joint_commands` | `true` | 是否订阅艾欧控制话题 |

状态发布和控制下发已经解耦。提高 `publish_rate_hz` 不会同步提高阻抗目标报文频率；这可避免
500 Hz状态测试放大SDK `OnClearSet` 竞争。需要注意，高发布频率不等于控制器产生了同频率的
独立测量帧：如果Marvin反馈帧更新较慢，ROS消息会以更高频率重复最新有效反馈。

### 显式运动门

| 参数 | 默认值 | 说明 |
|---|---|---|
| `allow_motion` | `false` | 总运动能力开关；启动参数会覆盖配置文件 |
| `sdk_control_compatibility_confirmed` | `false` | 人工确认SDK与控制器控制接口兼容 |
| `tool_dynamics_confirmed` | `false` | 人工确认工具/负载动力学配置 |
| `enabled_arms` | `none`（正式配置为`both`） | `none`、`left`、`right` 或 `both` |

四项必须同时允许，`enable_control=true` 才可能成功。连接、反馈和机器人健康检查仍独立生效。

### 对齐、超时与限位

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `feedback_timeout_sec` | `0.2` | 反馈新鲜度上限 |
| `command_timeout_sec` | `0.15` | `ACTIVE` 后有效手臂命令超时 |
| `command_recovery_alignment_rad` | `0.10` | COMMAND_WAIT恢复时每个启用关节与反馈最大偏差，不超过跟随误差阈值 |
| `command_reference_timeout_sec` | `0.2` | 运行中相邻命令参考有效期 |
| `max_command_step_rad` | `0.05` | `ACTIVE/SOFT_LIMIT_HOLD` 单消息最大跳变 |
| `initial_alignment_tolerance_rad` | `0.05` | 初次对齐每个启用关节最大误差 |
| `alignment_required_frames` | `20` | 初次对齐检查次数；COMMAND_WAIT恢复则需检查通过同样数量的新有效指令 |
| `joint_limit_tolerance_rad` | `0.0001` | URDF硬限位数值容差 |
| `joint_limit_safety_margin_rad` | `0.01` | 进入软限位保持的内缩裕量 |
| `soft_limit_release_margin_rad` | `0.03` | 从软限位保持恢复所需裕量 |
| `soft_limit_recovery_alignment_rad` | `0.10` | 软限位恢复对齐容差 |
| `soft_limit_recovery_frames` | `20` | 软限位恢复连续帧数 |
| `soft_limit_allow_other_arm_motion` | `false` | `false` 冻结全部启用手臂；`true` 只冻结触发侧 |
| `max_following_error_rad` | `0.25` | 已发送目标与反馈最大跟随误差 |
| `mode_transition_timeout_sec` | `2.0` | 参数或模式回显确认超时 |
| `sdk_packet_settle_sec` | `0.25` | 两个SDK设置报文之间的最小稳定时间 |
| `sdk_target_busy_timeout_sec` | `0.1` | 从首次关节目标包暂忙开始的重试期限（秒）；遥操作到期转COMMAND_WAIT而非IDLE，不因重复暂忙延长 |
| `hold_duration_sec` | `0.5` | 进入空闲模式前的短时保持时长 |
| `initial_pose_deg` | 14维数组 | 一键回初始姿态目标，顺序为左臂7轴后右臂7轴，单位deg |
| `initial_pose_tolerance_rad` | `0.02` | 判定到达初始姿态的逐关节最大误差 |
| `initial_pose_required_frames` | `20` | 到位后要求的连续稳定反馈帧数 |
| `initial_pose_timeout_sec` | `45.0` | 回初始姿态运动阶段超时 |

### 阻抗和软件限速

| 参数 | 默认值 | 说明 |
|---|---|---|
| `left_joint_stiffness` | `[2.5,2.5,2.5,2.0,1.5,1.2,1.0]` | 左臂 K，单位 N·m/deg |
| `right_joint_stiffness` | 同左臂 | 右臂 K |
| `left_joint_damping` | `[0.35,0.35,0.35,0.30,0.25,0.25,0.25]` | 左臂无量纲阻尼比 |
| `right_joint_damping` | 同左臂 | 右臂无量纲阻尼比 |
| `velocity_ratio` | `10` | Marvin速度百分比限制 |
| `acceleration_ratio` | `10` | Marvin加速度百分比限制 |
| `max_command_velocity_rad_s` | 14个 `0.25` | ROS侧逐关节速度限制 |
| `max_command_acceleration_rad_s2` | 14个 `0.50` | ROS侧逐关节加速度限制 |

这些低刚度参数是受控联调的起始候选值，不代表已经完成生产标定。

### QoS与SDK校验

| 参数 | 默认值 | 说明 |
|---|---|---|
| `state_qos_reliability` | `reliable` | 状态发布可靠性 |
| `command_qos_reliability` | `best_effort` | 命令订阅可靠性，与艾欧发布端兼容 |
| `state_qos_depth` | `10` | 状态队列深度 |
| `command_qos_depth` | `1` | 只保留最新目标，避免回放积压命令 |
| `expected_sdk_version` | `100343013` | 运行时要求的SDK版本 |
| `strict_sdk_version` | `true` | 版本不一致时是否拒绝启动 |

## 7. 在新电脑上从零部署

### 7.1 环境和外部文件

已验证的构建环境为 Ubuntu 22.04 x86-64、ROS 2 Humble、C++17 和 Python 3.10。
供应商的两个 `.so` 为 x86-64 ELF，因此不能直接在 ARM 工控机上使用。ROS 2 Humble
的系统安装请按[ROS 2 Humble Ubuntu 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
完成，建议安装 `ros-humble-ros-base` 或 `ros-humble-desktop`。

本仓库不分发天机SDK，新电脑必须从天机获取与当前机器人版本匹配的 `DEMO_C++`
目录。最稳妥的方式是复制完整目录；构建至少需要以下文件：

```text
MarvinSDK.h  FxRobot.h  PointSet.h  FxRtCSDef.h  FxType.h  FXDG.h  FXKineCommon.h
libMarvinSDK.so  libKine.so  ccs_m6_40.MvKDCfg
```

当前仓库只接受以下已验证文件：

| 文件 | SHA-256 |
|---|---|
| `libMarvinSDK.so` | `47ab2ab6e35cf899eae3978dfb50ee219e05b10f72aedd6bc9ba6a338a8b4011` |
| `libKine.so` | `f1512f8e440b53614b8d61adc27b217cab1b97d8bb9a80e32e00c880eee73397` |
| `ccs_m6_40.MvKDCfg` | `ce20b1a80974c3ed58cdac4973152625096c8ad93a779e5fd4108c26ce19bd52` |

SDK版本或机器人型号变更时不要绕过校验，应先重新完成ABI、控制模式和运动学回归验证。

### 7.2 安装构建工具

ROS 2 安装完成后执行：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake \
  python3-rosdep python3-colcon-common-extensions python3-tk

source /opt/ros/humble/setup.bash
```

`rosdep` 在整台电脑上只初始化一次。如果 `/etc/ros/rosdep/sources.list.d/20-default.list`
已存在，跳过第一条：

```bash
sudo rosdep init
rosdep update
```

### 7.3 克隆仓库并放置SDK

```bash
mkdir -p "$HOME/tianji_ws/src" "$HOME/tianji_vendor"
git clone --branch codex/force-feedback-hud https://github.com/Shuteng-0608/tianji_io_teleop_bridge.git \
  "$HOME/tianji_ws/src/tianji_io_teleop_bridge"

# 将供应商的完整 DEMO_C++ 目录复制到新电脑后，指向它：
export MARVIN_SDK_ROOT="$HOME/tianji_vendor/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++"
```

确认架构和校验值：

```bash
file "$MARVIN_SDK_ROOT/libMarvinSDK.so" "$MARVIN_SDK_ROOT/libKine.so"

cd "$MARVIN_SDK_ROOT"
sha256sum libMarvinSDK.so libKine.so ccs_m6_40.MvKDCfg
```

`file` 应显示 `ELF 64-bit ... x86-64`，三个校验值应与上表一致。

### 7.4 安装ROS依赖并构建

如果终端处在 Conda 环境中，建议先执行 `conda deactivate`，避免 CMake 选中 Conda Python。

```bash
cd "$HOME/tianji_ws"
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y

colcon build \
  --packages-select tianji_io_teleop_bridge \
  --symlink-install \
  --cmake-args \
    -DMARVIN_SDK_ROOT="$MARVIN_SDK_ROOT" \
    -DPython3_EXECUTABLE=/usr/bin/python3

source "$HOME/tianji_ws/install/setup.bash"
ros2 pkg executables tianji_io_teleop_bridge
```

CMake会检查SDK文件完整性和SHA-256，构建时将运行所需的 `.so` 和机型配置复制到
`install/`。以后每个新终端至少需要：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"
```

### 7.5 网络和DDS发现

使用额外电脑时，将机器人主控、该电脑和TeleBox可变IP网口接入同一交换机。
三者必须使用同一子网内互不冲突的静态IP。例如机器人为 `192.168.1.190/24`时，
电脑可设为未占用的 `192.168.1.x/24`；不要盲目照搬示例地址，TeleBox IP以艾欧配置为准。

```bash
ip -br address
ip route get 192.168.1.190
ping -c 3 192.168.1.190
```

ROS 2 DDS 还要求本机与TeleBox使用相同的 `ROS_DOMAIN_ID`、允许多播/本地UDP通信，且
`ROS_LOCALHOST_ONLY` 不能为 `1`。若艾欧未指定域，双方都不设置 `ROS_DOMAIN_ID` 即使用默认域0。
艾欧的网络与话题规约可参考
[TeleXperience 机器人适配文档](https://io-ai.tech/telexperience/docs/AdaptationRequirements/)。

```bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
ros2 node list
ros2 topic info -v /io_teleop/joint_cmd
```

`ping` 失败也可能是对端禁用ICMP；最终以SDK连接日志和ROS 2端点发现结果为准。

## 8. 配置、验收与启动

### 8.1 准备本机配置

不要直接把实机私有参数提交到仓库。从安全默认配置复制一份本机配置：

```bash
mkdir -p "$HOME/tianji_config"
cp "$HOME/tianji_ws/src/tianji_io_teleop_bridge/config/bridge.yaml" \
  "$HOME/tianji_config/bridge_robot.yaml"
```

至少确认 `robot_ip`、`enabled_arms`、阻抗参数、关节限速和两个人工确认门。
末端六维力估计当前保持 `publish_cartesian_disturbance: false`。

### 8.2 阶段A：不连接机器人的ROS自检

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"

ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  config_file:="$HOME/tianji_config/bridge_robot.yaml" \
  connect_on_start:=false \
  allow_motion:=false
```

另开终端检查接口：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"

ros2 node info /tianji_marvin_teleop_bridge
ros2 param get /tianji_marvin_teleop_bridge allow_motion
ros2 param get /tianji_marvin_teleop_bridge publish_cartesian_disturbance
```

预期两个参数均为 `false`，且ROS图中不出现独立关节扰动话题或左右臂
`estimated_cartesian_disturbance` 话题。

### 8.3 阶段B：连接机器人，仅发布状态

Marvin SDK连接是独占的，启动前关闭 MarvinPlatform 和其他SDK客户端。机器人工作区仍应清空，
但此阶段必须保持 `allow_motion:=false`。

```bash
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  config_file:="$HOME/tianji_config/bridge_robot.yaml" \
  allow_motion:=false
```

另开终端验证：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"

ros2 topic hz /io_teleop/joint_states
ros2 topic echo /io_teleop/joint_states --once
ros2 topic echo /io_teleop/joint_states --field effort --once
```

`name`、`position`、`velocity`、`effort` 应都为14项，并且逐索引对应：

```text
0..6   Joint1_L .. Joint7_L
7..13  Joint1_R .. Joint7_R
```

### 8.4 阶段C：确认TeleBox的14关节指令

在不开启机器人运动的情况下，可以先确认艾欧话题、名称和频率：

```bash
ros2 topic info -v /io_teleop/joint_cmd

ros2 topic echo /io_teleop/joint_cmd \
  --qos-reliability best_effort \
  --filter 'len(m.name) == 14 and set(m.name) == {"Joint1_L","Joint2_L","Joint3_L","Joint4_L","Joint5_L","Joint6_L","Joint7_L","Joint1_R","Joint2_R","Joint3_R","Joint4_R","Joint5_R","Joint6_R","Joint7_R"}' \
  --once

ros2 topic hz /io_teleop/joint_cmd \
  --filter 'len(m.name) == 14 and set(m.name) == {"Joint1_L","Joint2_L","Joint3_L","Joint4_L","Joint5_L","Joint6_L","Joint7_L","Joint1_R","Joint2_R","Joint3_R","Joint4_R","Joint5_R","Joint6_R","Joint7_R"}'
```

共享话题中可能同时有手指消息，因此总话题频率不等于14关节手臂指令频率。
如果使用 `--field position`，过滤表达式中的 `m` 已经是位置数组，不能再访问 `m.name`。

### 8.5 可选：一键回到初始姿态

> 该服务会真实控制机器人运动。调用前必须清空工作区、准备物理急停，并确认机器人能够沿路径
> 安全运动。当前实现只约束关节目标、速度和加速度，不进行双臂或环境碰撞规划。

当前双臂目标为：

```text
左臂： 90, -75, -90, -90, 0, 0, 0 deg
右臂：-90, -75,  90, -90, 0, 0, 0 deg
```

节点必须使用 `allow_motion:=true` 启动并处于 `MONITOR`。调用：

```bash
ros2 service call \
  /tianji_marvin_teleop_bridge/move_to_initial_pose \
  std_srvs/srv/Trigger \
  "{}"
```

另一个终端观察异步执行状态：

```bash
ros2 topic echo /tianji_marvin_teleop_bridge/control_status
```

正常状态顺序为：

```text
PREPARING_PARAMETERS → CONFIGURING_IMPEDANCE
→ RETURNING_TO_INITIAL_POSE → HOLD → MONITOR
```

日志出现 `Initial pose reached ... holding briefly before requesting idle` 表示已经到位。
如果当时正在遥操作，先调用 `enable_control=false`，等待状态回到 `MONITOR` 后再调用本服务。

### 8.6 阶段D：双臂阻抗遥操作联调

> 这一阶段会让机器人进入关节阻抗模式并在对齐后执行TeleBox目标。必须清空工作区、
> 确认工具/负载参数、准备物理急停，并由熟悉天机控制器的人员在场。

将 `bridge_both_test.yaml` 复制到本机配置目录，核对 `robot_ip`、机器人版本、负载和所有限制后启动。
当前仓库版本为 `enabled_arms: right`；以下流程若要控制双臂，须先编辑复制后的配置为 `both`，
不要仅根据文件名判断控制范围。`tool_config_dry_run: false` 表示工具配置服务可真实上传，
仅需预览时先改成 `true`。

```bash
cp "$HOME/tianji_ws/src/tianji_io_teleop_bridge/config/bridge_both_test.yaml" \
  "$HOME/tianji_config/bridge_both_test.yaml"

ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  config_file:="$HOME/tianji_config/bridge_both_test.yaml" \
  allow_motion:=true
```

另开终端观察状态和对齐面板：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"

ros2 topic echo /tianji_marvin_teleop_bridge/control_status
ros2 run tianji_io_teleop_bridge alignment_dashboard
```

显式启用控制：

```bash
ros2 service call \
  /tianji_marvin_teleop_bridge/enable_control \
  std_srvs/srv/SetBool \
  "{data: true}"
```

调用成功后会配置阻抗参数并切换为关节阻抗模式，但在对齐完成前不会发送位置目标。
对齐面板中绿色 `ALIGNED` 表示已进入容差，红色 `ADJUST` 需调整操作端，黄色 `LIMIT`
表示目标进入关节限位安全裕量，灰色 `OFF` 表示该臂未启用。

正常日志顺序：

```text
Impedance parameter packet sent; robot control mode is unchanged
Impedance parameters confirmed; pure mode-switch packet sent without a joint target
Joint impedance mode confirmed; no joint target sent; waiting for alignment
Alignment confirmed in joint impedance mode; teleoperation control is ACTIVE
```

只有最后一行出现后，节点才开始发送艾欧目标。关闭控制：

```bash
ros2 service call \
  /tianji_marvin_teleop_bridge/enable_control \
  std_srvs/srv/SetBool \
  "{data: false}"
```

### 8.7 实时关节位置与限位窗口（只读）

先安全结束当前运行，并启动新版桥接，使新增诊断字段生效。无需开启遥操作，
`allow_motion:=false` 时也能观察实机位置。另开图形桌面终端：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"
ros2 run tianji_io_teleop_bridge joint_limit_dashboard
```

窗口约10 Hz刷新，左右臂分别显示7个关节：

- 蓝圆：实机反馈位置；橙三角：**桥接已接受的有效目标**，不是被拒绝的原始指令。
- 简洁bar视图：每行仅保留关节名、实机角度、位置/目标标记和模型范围端点，全部为角度。
- 红线是触发边界，绿色区间是更靠内的解除保持范围；刻度按该关节模型上下界线性绘制。
- 不显示长段说明、逐关节调整建议和稳定计数；窗口标题保留运行状态及过期提示。
- 反馈或目标不新鲜时隐藏对应标记；灰条表示没有新鲜反馈，不能据此判断实机已停止。

数据只来自桥接的 `~/alignment_status`，范围直接取自桥接模型和运行参数，不在窗口内另写一套限位。
诊断超过0.5秒未收到、消息时间戳过旧/明显超前，或桥接标记反馈无效时，窗口会提示等待/过期。
跨电脑运行须保持ROS网络、时钟同步一致。此工具不会创建控制发布器、调用使能服务或连接Marvin SDK；
关闭窗口不等于停止机器人。小窗口自动改为单列，可用右侧滚动条查看所有关节。

不启动ROS或机器人即可预览合成示例（窗口明确标注演示数据）：

```bash
ros2 run tianji_io_teleop_bridge joint_limit_dashboard --demo
```

缺少Tk依赖时安装 `sudo apt install python3-tk`；没有图形桌面时使用原来的
`ros2 run tianji_io_teleop_bridge alignment_dashboard` 终端面板。
若窗口提示“限位数据缺失”，请重启新版桥接，不要凭旧数据调整实机。

### 8.8 后退3 cm：仅正逆解检查服务（不会运动）

独立进程只链接 `libKine`，不链接 `libMarvinSDK`，不创建目标发布器或使能客户端。
只订阅 `/io_teleop/joint_states` 的新鲜14关节反馈；桥接可以用 `allow_motion:=false`。
不需要停用/重启已有桥接，也不会改变其控制状态。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"
ros2 run tianji_io_teleop_bridge retreat_check_node
```

另一个终端分别检查左臂、右臂：

```bash
ros2 service call /tianji_retreat_check/retreat_left std_srvs/srv/Trigger '{}'
ros2 service call /tianji_retreat_check/retreat_right std_srvs/srv/Trigger '{}'
```

依据用户初步确认的轴向：右臂 +Y向上、+X向前、+Z向右；左臂 +Y向下、+X向前、+Z向左。
两臂各自在 **SDK机械臂基坐标系** 沿 -X 计算30 mm，Y/Z和旋转矩阵不变。
这是假定轴向后的数值检查，不能代替实机方向验证。当前工具变换为空，计算点是法兰中心，
不是已标定的工具TCP。服务不下发工具参数，也不需要工具动力学辨识结果来做纯运动学检查。

算法先检查起始关节，再对0–30 mm的31个采样位置求逆解（以上一解为参考），
检查SDK可达性/奇异/关节越限标志、桥接模型的关节范围、相邻解不超过3°，
并用正解验证误差不超过0.1 mm和0.05°。默认规划裕量0.005 rad；若桥接配置不同，
启动此节点时用 `--ros-args -p joint_limit_safety_margin_rad:=实际值` 显式保持一致，
此独立节点不会自动读取运行中桥接的参数。

`success=true` **只表示采样运动学检查通过，绝不表示已运动或允许执行**。
返回文本包含 `NO_MOTION`、起点/终点XYZ（mm）、最终逆解关节角（deg）、最大位置/旋转残差、
最大关节步长、相对规划限位的最小余量；失败会注明原因和路径点。
无反馈、超时、名字重复/缺失、非有限位置或明显过旧/超前时间戳会拒绝。
本功能尚无碰撞检查、连续轨迹安全证明、动态约束或电机执行路径。

## 9. 诊断与常见问题

### `MARVIN_SDK_ROOT is unset` 或SDK文件缺失

重新确认 `MARVIN_SDK_ROOT` 指向包含 `.h`、`.so` 和 `ccs_m6_40.MvKDCfg` 的同一个
`DEMO_C++` 目录，然后使用 `-DMARVIN_SDK_ROOT="$MARVIN_SDK_ROOT"` 重新构建。

### SDK checksum mismatch

本地SDK与已验证版本不同。不要为了通过构建而直接替换CMake中的校验值；先与天机确认
SDK、控制器系统版本和M6 CCS 4.0机型配置是否匹配。

### `Package 'tianji_io_teleop_bridge' not found`

当前终端没有加载工作空间：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"
```

### 看不到TeleBox节点或 `/io_teleop/joint_cmd`

按顺序检查交换机物理链路、静态IP/子网掩码、`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、本机防火墙和
DDS多播。`ros2 topic info -v /io_teleop/joint_cmd` 应能显示艾欧发布端。

### 旧话题仍然存在或同名节点出现多次

通常是旧版节点仍在运行。停止所有旧进程，在新终端重新 `source` 工作空间后再启动。
默认配置下，独立关节扰动话题和左右臂末端 `WrenchStamped` 话题都不应出现。

### `refused: allow_motion is false`

启动命令没有显式传入 `allow_motion:=true`。不要在只监测场景中打开它。

### `SDK/controller control compatibility is not confirmed`

当前配置中的 `sdk_control_compatibility_confirmed` 为 `false`。默认 `bridge.yaml` 会有意触发该拒绝；
受控联调可使用 `bridge_both_test.yaml`，并核对 `enabled_arms`（当前为 `right`，不是 `both`）。

### 一直停在 `WAITING_FOR_ALIGNMENT`

```bash
ros2 topic echo /tianji_marvin_teleop_bridge/control_status
ros2 run tianji_io_teleop_bridge alignment_dashboard
```

重点查看 `alignment_block_reason`、`alignment_max_joint`、`alignment_error_rad`、
`aligned_joints` 和 `aligned_frames`。没有新鲜14关节命令时会显示 `WAITING_FOR_COMMAND`。

### 进入 `HOLD`

查看节点错误日志和 `control_status.reason`。普通 `HOLD` 完成后需要再次显式启用；
`SOFT_LIMIT_HOLD` 在目标回到释放裕量并稳定后可以自动恢复。

如果显示 `COMMAND_WAIT`，表示输入被拒绝/超时、反馈暂不可用或SDK暂忙超过重试期限，
节点不主动退出阻抗模式，等待反馈恢复和新指令重新对齐，
不是普通HOLD的“保持0.5秒后下使能”。查看拒绝原因及对齐面板，无需重复调用enable_control。

### `OnClearSet failed`

SDK前一个设置包可能仍在处理。节点会按 `sdk_packet_settle_sec` 分隔参数包与模式包，并对空闲请求进行有限重试。
同时确认没有其他 Marvin SDK 客户端占用控制器连接。

关节目标发送单独区分 `OnClearSet` 暂忙：跳过当前更新，下个控制周期重新检查安全条件，
用最新有效目标重新限速；不阻塞、不积压旧目标、不覆盖SDK待发送包。
未成功提交时不推进限速器的位置/速度状态，也不更新上次成功提交目标和时间。
普通HOLD目标也使用同一期限，但不会继续跟随遥操作目标。

`sdk_target_busy_timeout_sec: 0.1` 为 **100 ms**，从首次暂忙计时；重复暂忙不会重置期限。
期限届满后停止本轮目标重试：遥操作进入COMMAND_WAIT，保持最后提交目标，不请求IDLE。
重新对齐后才能开始下一轮下发；普通HOLD、回初始姿态仍按原故障停止流程处理。
这是软件暂忙窗口，**不是“距上次控制器收到指令”的时限**；实际控制器指令间隔还包含
此前控制周期、传输与调度延迟。用户已向天机确认：不发新目标时保持最后目标，不因超时退出。
100 ms仍是桥接工程重试窗口，不是已认证的安全值；也不保证非实时系统在恰好100 ms时进入等待。
参数须为正有限数，且不能超过指令或反馈超时；修改YAML后重启节点生效。
状态发布频率独立配置（当前联调为300 Hz）；上述恢复机制不取消限位、反馈/指令超时检查。
控制检查移到状态话题发布之前，避免本次状态发布阻塞使本次控制快照过期；仍使用单线程执行器，
发布阻塞或系统调度仍可能延迟下一次回调，并非实时性保证。
控制按本次处理**完成时间加一个周期**排期，
不再按旧的固定相位赶进度。`control_rate_hz: 100.0` 表示控制频率上限，实际频率可能因
SDK耗时和状态定时器粒度略低于100 Hz；错过周期不补发，恢复后仍限速，不追赶历史目标。
参数包、模式包和其他SDK发送错误仍走原故障流程。`submitted` 仅表示SDK接受提交，不代表控制器确认收到。

发生超时日志为 `joint target retry deadline exceeded`。其中 `elapsed_since_first_busy_ms`
只表示自首次暂忙返回以来的经过时间，**不能证明SDK持续忙了这么久**；`busy_responses`
才是本轮实际观察到的暂忙次数。故障日志保留当时的时序快照，不需要根据停机后的值反推。
暂忙/恢复日志限流到每秒最多各一条；累计计数不受日志限流影响。

下列时序字段也在 `/tianji_marvin_teleop_bridge/control_status` 中每秒发布（单位ms，未观察到为-1）：

- `timer_gap_ms` / `max_timer_gap_ms`：反馈回调入口间隔及最大值。
- `callback_age_ms`：当前回调已执行的时间，包含被系统调度暂停的时间。
- `feedback_sdk_ms` / `max_feedback_sdk_ms`：反馈SDK调用的最近/最大耗时。
- `target_sdk_ms` / `max_target_sdk_ms`：目标SDK调用的最近/最大耗时。
- `target_attempt_gap_ms`、`last_target_attempt_age_ms`：最近两次目标尝试间隔及距最近尝试的时间。
- `last_target_submit_age_ms`：距最近成功提交目标的时间，不是控制器确认时间；恢复日志中显示的是前一次提交年龄。
- `target_busy_total`：累计暂忙返回次数。

最大值和累计计数在SDK重连时清零。上述观测能帮助区分回调停顿与SDK调用耗时，
但调用耗时也包含线程被暂停的时间，不能单凭它证明SDK内部卡顿。

### 末端六维力不为零或波动

这是模型反算值而非传感器真值。静态偏置和噪声是预期现象；使用前应按姿态采集基线，并验证坐标轴、符号和灵敏度。

## 10. 测试

```bash
cd "$HOME/tianji_ws"
source /opt/ros/humble/setup.bash
source "$HOME/tianji_ws/install/setup.bash"

colcon test --packages-select tianji_io_teleop_bridge
colcon test-result --test-result-base build/tianji_io_teleop_bridge/test_results --verbose
```

测试覆盖关节名称映射、position/velocity/effort逐索引对应、单位换算、输入拒绝、限位与恢复滞回、
速度/加速度限制、控制状态门、扰动力消息映射，
以及左右臂运动学的 `tau ↔ wrench` 回归验证。
`test_target_send` 使用替身SDK（不链接机器人控制库）覆盖暂忙分类、缓冲区不覆盖、
限速状态回滚、SDK暂忙到期转等待、快照过期/读失败/反馈帧停更时不发IDLE、
双臂反馈恢复后重新对齐、手动停用和真实故障仍退出，以及只读模式不下发任何控制报文。

## 11. 项目结构

```text
config/     安全默认配置和双臂联调配置
include/    关节模型、控制门、SDK与运动学适配器接口
launch/     ROS 2 launch文件
scripts/    对齐终端面板
src/        ROS节点、Marvin SDK适配和运动学计算
test/       GTest单元与回归测试
```

## License

本项目代码使用 Apache-2.0 许可证。Marvin SDK、运动学库及其配置文件属于外部供应商依赖，遵循其各自许可。
## 左右臂后退 10 cm 服务（正式配置）

`/tianji_marvin_teleop_bridge/retreat_left_10cm` 和 `/tianji_marvin_teleop_bridge/retreat_right_10cm`（`std_srvs/srv/Trigger`）是**真实运动服务**，分别控制左臂、右臂。固定在对应 SDK 臂基座坐标系沿 −X 平移 **100 mm**，保持法兰 Y/Z 和旋转矩阵不变。两臂均按“+X 向前”，不是相机或工具自身坐标系。原 `/tianji_retreat_check/retreat_left`、`retreat_right` 仍是只计算30 mm、不运动的检查服务。

只允许在 MONITOR、指定臂在 `enabled_arms` 范围内、反馈新鲜且该臂静止时调用。正式配置 `enabled_arms: both`、`subscribe_joint_commands: true` 可直接使用，不必切换为单臂配置。流程：仅准备被选中臂的阻抗 → 从实际位置规划一次 −X 100 mm 路径 → 平滑发送该臂目标 → 短暂保持 → 请求IDLE并回到MONITOR。规划检查 1001 个路径点的 IK、关节安全裕量和 FK 残差；规划失败则保持、不发送路径目标。**不需要先调用 enable_control=true，也不需要开启艾欧遥操作。**

单次操作独占控制：遥操作、回初始姿态或另一臂后退正在执行/保持时，新的后退请求会被拒绝；后退期间收到的艾欧命令不参与执行。另一臂不会被下发阻抗参数、模式或关节目标，后退的手动退出和故障IDLE重试也仅针对被选中臂。状态话题的 `enabled_arms=both` 是配置允许范围，`control_arms=left/right` 是本次实际控制范围。

执行改为连续下发：`retreat_duration_sec: 10.0` 表示 100 mm 名义用时 10 秒，平均约 10 mm/s。采用五次平滑曲线起步、减速停止；理想 10 秒轨迹的中段峰值约 18.75 mm/s。不再每 0.1 mm 停等 100 ms，也不再经过逐点清零速度的限速器。1001 个点按平滑进度规划，沿 SDK 基座 −X，Y/Z 和朝向保持不变；控制频率上限仍为 100 Hz。

执行前检查全部采样关节目标的速度、加速度（含起止边界），分别不超过 0.20 rad/s、0.40 rad/s²，若配置的关节限速更低则取更低值。需要时自动延长发送周期，启动日志显示计算用时。SDK 忙或调度延迟不会跳过路径点、不会追赶补发，因此实际用时可能更长，通信抖动也会影响实机顺滑性。该检查约束的是规划采样轨迹，不是实测电机速度或加速度保证。

继续保留关节限位、反馈有效性、SDK 持续忙保护，以及 `max_following_error_rad`（默认 0.25 rad）。发送超时取“30 秒”和“计算用时的 3 倍”中的较大值；超时停止推进、保持最后目标，不自动 IDLE。控制器明确故障、模式丢失等仍执行原有故障处理。左右臂共用 `retreat_duration_sec`，调整后重启节点，数值越大越慢。原右臂专用配置仍可兼容使用，但只允许右臂服务。

正常下发结束后进入 `HOLD`，按 `hold_duration_sec`（默认0.5秒）短暂保持最后一个已发送目标，然后仅向本次后退的手臂请求 `IDLE`，请求发送成功后自动回到 `MONITOR`。不再需要额外调用 `enable_control=false`，可以继续请求另一臂或同一臂后退。若IDLE暂忙，保留单臂范围重试，成功后再回到MONITOR；失败期间不接受新运动。重试耗尽则保留故障，需要人工检查。

日志仍明确说明“目标序列已发送，未验证实际到位”，随后显示 `left/right retreat finished: IDLE request sent; returned to MONITOR`（延迟重试成功时包含 `delayed`）。这里依据SDK发送成功返回MONITOR，没有增加IDLE反馈回读确认或精确到位判定。目标朝向保持不变，但实机运动精度取决于阻抗刚度、负载补偿和跟随能力。**正常结束会自动退出阻抗，可能卸力，请确保机械臂及负载可安全退出。**

运动中因限位、反馈失效等异常进入的 `RETREAT_HOLD` 不适用上述正常收尾，仍保持最后目标、等待人工处理；控制器明确故障继续沿用故障退出逻辑。

这是采样运动学检查，**没有碰撞检测**，也不能保证实机姿态绝对不变。首次测试须空载或正确配置负载、远离人和障碍物、准备实体急停；阻抗保持不等于机械锁止。关闭发送只保留最后下发目标，不保证瞬时测量位置不再变化。

1. 安全停止原桥接节点，确保没有其他程序占用 Marvin 控制连接；保持待运动臂静止，确认后方及整个臂运动范围净空。
2. 每次机器人断电重启后，先用厂商方式下发实际工具动力学参数。编辑 `config/bridge.yaml`，确认 SDK 兼容和双臂工具配置后才把 `sdk_control_compatibility_confirmed`、`tool_dynamics_confirmed` 改为 `true`。这两个标志只是人工确认，**不会下发工具参数**。保持 `enabled_arms: both`，允许保留 `subscribe_joint_commands: true`。现有 `bridge_both_test.yaml` 也已包含后退配置，可继续使用；不自动修改其中已有的联调参数。
3. 编译并启动（以下工作空间路径按实际安装调整）：

```bash
cd /home/stw/tianji_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tianji_io_teleop_bridge
source install/setup.bash
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/bridge.yaml \
  allow_motion:=true
```

4. 新终端观察状态：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 topic echo /tianji_marvin_teleop_bridge/control_status
```

5. 确认安全后，在另一个已 source 的终端**只选择一条调用**：

```bash
ros2 service call /tianji_marvin_teleop_bridge/retreat_left_10cm std_srvs/srv/Trigger '{}'
```

或右臂：

```bash
ros2 service call /tianji_marvin_teleop_bridge/retreat_right_10cm std_srvs/srv/Trigger '{}'
```

`success=True` 仅表示受理，不表示路径已经规划或实机到位。看到 `left/right retreat target sequence sent: -X 100 mm; actual arrival not verified` 后，等待短暂保持和IDLE发送；确认状态回到 `MONITOR`，即可调用下一次后退，无需手动退出。若是异常保持或故障，需先检查原因再人工处理。每次调用均从新的当前位置再退 100 mm，不是回到固定绝对位置。

主动退出阻抗（可能卸力，先确保机械臂和负载可安全退出）沿用：

```bash
ros2 service call /tianji_marvin_teleop_bridge/enable_control std_srvs/srv/SetBool '{data: false}'
```

紧急情况使用实体急停，不依赖 ROS 服务。程序退出后的机器人行为仍以厂商控制器为准。
