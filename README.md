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

> **安全提示**：`config/bridge.yaml` 默认禁止所有运动。`config/bridge_both_test.yaml`
> 是已经打开部分人工确认门的双臂联调配置，不是无人值守生产配置。首次调试应清空机器人周围区域、
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
| 发布 | `/io_teleop/joint_states` | `sensor_msgs/msg/JointState` | 默认100 Hz；联调500 Hz | 双臂14关节反馈；`effort` 为 `m_EST_Joint_Force` |
| 订阅 | `/io_teleop/joint_cmd` | `sensor_msgs/msg/JointState` | 艾欧约100 Hz | 双臂目标角；共享话题中的手指消息会被忽略 |
| 可选发布 | `~/left/estimated_cartesian_disturbance` | `geometry_msgs/msg/WrenchStamped` | 100 Hz | 左臂法兰处计算得到的等效六维扰动力；默认关闭 |
| 可选发布 | `~/right/estimated_cartesian_disturbance` | `geometry_msgs/msg/WrenchStamped` | 100 Hz | 右臂法兰处计算得到的等效六维扰动力；默认关闭 |
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
                    └──────────→ HOLD → MONITOR
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

- 有效手臂指令超过 `command_timeout_sec` 未更新；
- Marvin反馈停止更新、含非有限值或机器人报告错误；
- 实际位置相对已发送目标的跟随误差过大；
- SDK控制报文发送失败；
- 操作员调用 `enable_control=false`。

目标进入关节软限位时不会立刻退出遥操作，而是进入 `SOFT_LIMIT_HOLD`：默认冻结所有启用手臂，
目标回到更大的释放裕量、重新对齐并稳定指定帧数后自动恢复 `ACTIVE`。机器人故障和急停错误不会由本节点自动清除。

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

### `config/bridge.yaml`

安全默认配置：

```yaml
allow_motion: false
sdk_control_compatibility_confirmed: false
tool_dynamics_confirmed: false
enabled_arms: "none"
```

即使连接机器人，也只能读取反馈、发布状态和检查命令，不会发送模式、阻抗、空闲或关节目标报文。

### `config/bridge_both_test.yaml`

双臂实机联调配置：

```yaml
allow_motion: false
sdk_control_compatibility_confirmed: true
tool_dynamics_confirmed: true
enabled_arms: "both"
initial_alignment_tolerance_rad: 0.25
joint_limit_safety_margin_rad: 0.005
soft_limit_release_margin_rad: 0.015
publish_rate_hz: 500.0
control_rate_hz: 100.0
```

配置文件中的 `allow_motion` 仍为 `false`，必须在启动命令中显式传入 `allow_motion:=true` 才能开放运动能力。
两个 `*_confirmed` 参数是人工确认标志，不会替代实际的SDK兼容性和工具负载检查。
其中两个限位裕量和500 Hz状态发布是当前双臂联调专用设置；控制状态机及SDK目标下发仍保持
100 Hz。安全默认配置
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
| `enabled_arms` | `none` | `none`、`left`、`right` 或 `both` |

四项必须同时允许，`enable_control=true` 才可能成功。连接、反馈和机器人健康检查仍独立生效。

### 对齐、超时与限位

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `feedback_timeout_sec` | `0.2` | 反馈新鲜度上限 |
| `command_timeout_sec` | `0.15` | `ACTIVE` 后有效手臂命令超时 |
| `command_reference_timeout_sec` | `0.2` | 运行中相邻命令参考有效期 |
| `max_command_step_rad` | `0.05` | `ACTIVE/SOFT_LIMIT_HOLD` 单消息最大跳变 |
| `initial_alignment_tolerance_rad` | `0.05` | 初次对齐每个启用关节最大误差 |
| `alignment_required_frames` | `20` | 初次对齐连续帧数 |
| `joint_limit_tolerance_rad` | `0.0001` | URDF硬限位数值容差 |
| `joint_limit_safety_margin_rad` | `0.01` | 进入软限位保持的内缩裕量 |
| `soft_limit_release_margin_rad` | `0.03` | 从软限位保持恢复所需裕量 |
| `soft_limit_recovery_alignment_rad` | `0.10` | 软限位恢复对齐容差 |
| `soft_limit_recovery_frames` | `20` | 软限位恢复连续帧数 |
| `soft_limit_allow_other_arm_motion` | `false` | `false` 冻结全部启用手臂；`true` 只冻结触发侧 |
| `max_following_error_rad` | `0.25` | 已发送目标与反馈最大跟随误差 |
| `mode_transition_timeout_sec` | `2.0` | 参数或模式回显确认超时 |
| `sdk_packet_settle_sec` | `0.25` | 两个SDK设置报文之间的最小稳定时间 |
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
  python3-rosdep python3-colcon-common-extensions

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
git clone https://github.com/Shuteng-0608/tianji_io_teleop_bridge.git \
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

将 `bridge_both_test.yaml` 复制到本机配置目录，核对 `robot_ip`、机器人版本、负载和所有限制后启动：

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
双臂受控联调使用 `bridge_both_test.yaml`。

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

### `OnClearSet failed`

SDK前一个设置包可能仍在处理。节点会按 `sdk_packet_settle_sec` 分隔参数包与模式包，并对空闲请求进行有限重试。
同时确认没有其他 Marvin SDK 客户端占用控制器连接。

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
