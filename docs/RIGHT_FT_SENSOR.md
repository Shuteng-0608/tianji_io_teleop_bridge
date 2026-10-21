# 右臂末端六维力传感器记录

完整可复制命令见 [清零 标定与逐条采集操作流程](RIGHT_FT_RUNBOOK.md)。

右臂 B 已接六维力传感器。桥接节点发布独立的 `sensor_msgs/msg/JointState`，
用于艾欧支持的 `name + effort` 记录格式；六项是测量通道，不是机械臂关节。
自 2026-10-16 起，此话题直接发布**工具重力补偿和软件去皮完成后的数据**。
话题名和消息格式不变，不新增原始/补偿力话题或传感器 `WrenchStamped`。
现有十四关节位置、速度、反馈关节力矩和关节外力矩估计仍分别从各自 SDK 字段读取。

## 消息约定

默认话题：`/io_teleop/right/ft_sensor`。

```yaml
header:
  stamp: 与同一份 DCSS 反馈生成的 joint_states 相同
  frame_id: right_ft_measurement
name: [Fx, Fy, Fz, Tx, Ty, Tz]
position: []
velocity: []
effort: [0.01, -0.02, 0.01, 0.0001, -0.0002, 0.0001]  # 示意值，已补偿并去皮
```

`effort[0:3]` 单位 N，`effort[3:6]` 单位 N·m，均表达在传感器测量坐标系、以传感器测量原点为参考。
厂家轴向一致性已由操作者确认；当前安装旋转为单位阵，力和力矩符号均为 +1。
`header.stamp` 是主机读取同一份 DCSS 反馈的 ROS 时间，不是传感器硬件采样时刻。

## 每次开机的流程

每次机器人开机都按本轮重新准备：无工具清零 → 安装当前工具 → 独立多姿态标定 →
自动覆盖固定标定文件 → 启动桥接加载该文件 → 起始姿态软件去皮 → 遥操作采集。
固定路径为 `/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml`，`auto` 只在真实数据验证通过后更新；
各轮采样和标定结果继续保留。本轮失败时不能使用固定位置残留的旧结果。
桥接仍需通过参数指定文件，不会扫描目录挑选“最新文件”；未指定路径或文件无效会在连接机器人前报错。
桥接只在启动时读取标定文件，覆盖固定文件不会改变正在运行的补偿参数；历史去皮值不会自动加载。

## 在线处理及手动去皮

```text
raw_si = SDK 六通道 / 10000
compensated = signs * raw_si - [m*g_sensor(q), com × (m*g_sensor(q))] - calibration_bias
session_tare = 静止窗口内 compensated 的均值
published = compensated - session_tare
```

每帧使用同一份右臂反馈的七轴实际角度做本地法兰 FK，工具参数来自独立力传标定文件，
不使用控制器的关节动力学工具配置。安装偏置已经包含在 calibration_bias 中，不额外扣旧 X/Z 参考值。
去皮窗口始终积累尚未扣 session_tare 的 compensated，重复手动去皮会替换去皮值，不累计扣除。
程序不会调用传感器硬件清零，也不会因补偿或去皮改变控制模式、发送运动指令。

1. 每条录制前回到采集起始姿态，保持采集时的控制模式，工具悬空、无接触、静止至少 5 秒。
2. 执行一次软件去皮：

   ```bash
   source /home/stw/tianji_ws/install/setup.bash
   ros2 service call /tianji_marvin_teleop_bridge/right/zero_ft_sensor std_srvs/srv/Trigger '{}'
   ```

3. 返回 `success: true` 后，下一份新鲜反馈开始在 `/io_teleop/right/ft_sensor` 发布补偿去皮后的六轴数据。
   艾欧继续选择该话题和 `sensor_msgs/JointState`，采集 `effort` 即可。

启动及 SDK 重连后均需手动去皮。成功前不发布六维力消息，不会混入原始值或伪造零值；其余关节话题照常发布。
去皮采用最近完整 5 秒（可配置）的窗口，无需阻塞反馈线程等待；条件不足时服务返回原因，保持静止后重试。
最低 200 个不同控制器帧、最大帧间隔为 feedback_timeout_sec；低速标志为真，
各轴速度 ≤0.2 deg/s、位置跨度 ≤0.15 deg；力标准差 ≤0.1 N、力矩标准差 ≤0.005 N·m。
不按残余力大小拒绝去皮，不以标定 RMS 阈值阻止流程联调。稳定接触不能自动识别，由操作者确认无接触。
去皮后偏置保持固定，不会在遥操作中自动追零。
每条录制结束并回到起始姿态后，重新执行软件去皮，成功后再开始下一条；新值替换旧值，不累计扣除。

去皮记录先写入文件，保存成功才生效；磁盘写入失败保持之前的去皮状态。服务响应及现有 alignment_status
给出记录路径。每份 YAML 包含完整标定参数快照、六轴去皮值、均值关节角、窗口起止时间、控制模式、样本量、
标准差和计算公式。重启不会自动沿用旧去皮值。项目配置保存在 `/home/stw/ft_calibration_runs/teleop_tares`。
更换工具或硬件清零后需更新对应标定文件。单姿态去皮不能保证所有姿态都无残余，也不消除运动惯性载荷。

## HUD 使用同一路数据

默认 HUD 已直接订阅 `/io_teleop/right/ft_sensor`，显示右臂补偿去皮后的数据，不重新估计或单独去皮。
完成桥接去皮后运行 `ros2 launch tianji_io_teleop_bridge force_hud.launch.py`。
图像话题仍是 `/tianji_force_hud/image/compressed`。详见 [HUD 使用说明](FORCE_HUD.md)。

## 参数与启动

| 参数 | 代码默认 | 项目桥接 YAML | 说明 |
|---|---|---|---|
| `publish_right_ft_sensor` | `false` | `true` | 启用右臂六维力配置及发布 |
| `right_ft_sensor_topic` | `/io_teleop/right/ft_sensor` | 同左 | 独立 JointState 话题 |
| `right_ft_sensor_frame_id` | 空，取标定文件值 | `right_ft_measurement` | 显式配置必须与标定文件一致 |
| `right_ft_calibration_file` | 空，启用力传时必须指定 | 同左 | 本轮标定更新的固定文件，或本轮 `result/calibration.yaml` 绝对路径 |
| `right_ft_tare_directory` | 空，使用 `$HOME/.ros/tianji_ft_tares` | `/home/stw/ft_calibration_runs/teleop_tares` | 软件去皮记录目录 |
| `right_ft_tare_window_sec` | `5.0` | `5.0` | 最近连续静止窗口长度，秒 |

这些参数仅启动时设置。`bridge.yaml`、`bridge_both_test.yaml`、`bridge_right_retreat_test.yaml`
均开启右臂传感器，需要显式指定本轮标定文件；不依赖 `enabled_arms` 或运动许可。直接运行可执行程序而不加载 YAML 时，需显式开启。

```bash
source /home/stw/tianji_ws/install/setup.bash
# 本轮自动标定通过且固定文件更新成功后使用；也可在桥接 YAML 中设置该参数。
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  allow_motion:=false \
  right_ft_calibration_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml
```

艾欧新增自定义记录话题 `/io_teleop/right/ft_sensor`，消息类型选择 `sensor_msgs/JointState`。
ROS 2 命令行显示的完整类型名为 `sensor_msgs/msg/JointState`。

```bash
ros2 topic echo /io_teleop/right/ft_sensor --once
ros2 topic hz /io_teleop/right/ft_sensor
ros2 topic echo /tianji_marvin_teleop_bridge/alignment_status
```

SDK 使用独占本机 UDP 4730 端口，启动前释放其他天机客户端的连接。
当前普通桥接 YAML 反馈频率为 300 Hz，右臂退让配置为 100 Hz。
六维力仅在右臂反馈帧变化时发布，故不会以新时间戳重复发送同一 SDK 帧。
SDK 帧更新只能证明控制器反馈更新，不能替代传感器自身的样本序号或硬件故障位。

## 数据通道与诊断

连接初始化（含重新建立连接）时调用一次 `OnSetUserSpcfData_B(216)`。
此调用会等待 SDK 响应，放在控制开始前；采集循环不重复发送通道设置命令。
切换的是遥测通道，不要求机器人进入力控模式。

- 通道回显：`m_Out[1].m_EST_Joint_Firc[0] == 216`。
- 传感器数据：`m_Out[1].m_EST_Joint_Firc_Dot[0..5]`。
- 关节力矩：`m_FB_Joint_SToq`；关节外力矩估计：`m_EST_Joint_Force`。

`~/alignment_status` 中的 `tianji_teleop/right_ft_sensor` 报告通道、最后发布距今时间、单位和坐标系。
错误通道、NaN/Inf、反馈停更时不发布伪造的零值或旧值。
合法零读数也经过相同补偿，不因原始为零而绕过计算。等待软件去皮为 WARN；断开或发布数据超时为 STALE；错误通道、FK 失败或非有限值为 ERROR。
通道设置失败不妨碍现有关节数据采集；需要检查 SDK/控制器并重新建立连接后重试。
若其他客户端改了通道，节点暂停传感器发布并报告 `CHANNEL_NOT_216`，不会在运动期间争抢通道。

## SDK 版本

此次使用 SDK `100343014`，实测控制器 `100343024`；厂商接口按 `/1000` 的版本族判断兼容。
项目保留严格运行时版本和构建 SHA-256 校验，更新为当前 SDK 及配套运动学库。
旧构建目录若缓存了此前校验值，重新配置时清除相应缓存：

```bash
cd /home/stw/tianji_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tianji_io_teleop_bridge --symlink-install --cmake-args \
  -DMARVIN_SDK_ROOT=/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++ \
  -UMARVIN_SDK_EXPECTED_SHA256 -UMARVIN_KINEMATICS_EXPECTED_SHA256
```

构建仍将当前文件与源码中的固定哈希比较。这次六维力读取验收不包含实机运动验收。

## 补偿与去皮版本验证（2026-10-16）

- 编译安装成功。4 个相关测试套件共 107 项通过（首次接入时）：FT 补偿 7 项、桥接/控制回归 92 项、运动学 4 项、后退计算 4 项。
- 18 个标定/验证姿态与两次后续复测，共 20 个实测姿态离线回放，与 Python 独立计算结果逐轴误差不超过 1e-6。
- 假 SDK 测试确认：去皮前不发布，成功后原话题发布补偿数据，与关节消息同时间戳；不同控制器帧才发布；
  无效通道/非有限值不发布，断线重连后去皮失效；软件去皮不触发 SDK 运动或模式写入。
- 合成物理载荷覆盖姿态变化、质心力矩、安装旋转、符号、重复去皮、保存失败和静止窗口检查。
- 这些验证没有连接机器人。新的补偿发布链路和艾欧最终录制仍需实机确认。

## 历史原始发布版本验证（2026-10-13）

- `colcon build` 成功；16 组测试通过，`colcon test-result` 汇总 208 tests，0 errors/failures/skipped。
- 新增测试覆盖六值缩放、空位置/速度数组、同帧时间戳、错误通道、重复帧、非有限值、
  有效零读数、过期诊断、重连配置、配置失败及与现有关节话题共同运行 300 Hz。
- 实机节点使用 `allow_motion=false`、`subscribe_joint_commands=false`、`enabled_arms=none`，
  约 5 秒收到 1501 条传感器消息，实测约 300.16 Hz。
- 1501 条传感器消息均与关节状态、校正后外力矩估计、原始外力矩估计三路找到相同时间戳。
- 末条 `effort=[3.0366,0.9322,1.1190,0.1882,-0.1990,0.0222]`，通道回显 216。
  未出现传感器 WrenchStamped 话题。
- `frame_id` 暂空，因此诊断为 `FRAME_ID_UNSET`；测试结束正常退出并释放 SDK 连接。
- 本轮实测 ROS 发布链路；艾欧端对空位置/速度数组的支持由用户确认，艾欧端最终落盘尚未现场验收。
