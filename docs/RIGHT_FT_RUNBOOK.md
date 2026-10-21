# 右臂末端力传清零、标定与采集操作指令

日常操作只需两条命令：无工具时运行 `right_ft_zero.py`，安装工具后运行
`right_ft_calibration.py auto`。不需要目录变量、复制配置或填写 zero-note。
两个命令可以在不同终端执行，不依赖上一次终端的目录变量。
使用 Conda 时先 `conda activate tianji`，安装 `numpy`、`PyYAML`；离线测试另需 `pytest`。
独立清零和标定无需 ROS。SDK 默认位于当前用户的 `~/TJ_FX_ROBOT_CONTRL_SDK`，
模型路径中的 `~` 自动展开，固定结果路径相对于配置文件目录解析。
迁移到其他用户名或仓库位置不再需要修改这三个默认路径；下面桥接/HUD 部分的
`/home/stw` 是原 PC 的 ROS 工作区示例，新 PC 使用时改为自己的工作区路径。

## 1 无工具 SDK 清零

完成预热后，传感器保留安装、接线正常，受力侧无工具、无接触；右臂七轴零位静止，
处于 IDLE 或 Position 模式。断开天机界面、遥操作桥接及其他 SDK 客户端。

```bash
python -u "$HOME/tianji_ws/src/tianji_io_teleop_bridge/scripts/right_ft_zero.py"
```

执行命令即连接右臂并开始无工具清零，无需额外参数或交互确认。
程序先读取约 2 秒，调用一次右臂传感器 SDK 清零，等待约 1 秒，再验证约 5 秒。
终端打印清零前后的六轴均值、标准差及 `zero_verified: true/false`，随后释放 SDK。
不发送运动或模式切换指令。默认不创建目录、不保存原始采样或清零记录。

只有 `zero_verified: true` 且退出码为 0 才继续。失败时先处理终端报告的问题；
若清零已尝试或被中断，终端会显示 `reset_attempted`，程序不自动重试。
需留档时可选加 `--output /绝对路径/新的清零目录`，已有目录不会被覆盖。

## 2 安装工具

清零通过后安装并固定最终工具和线缆，保持右臂七轴零位，工具无外部接触。
安装后不要再次执行 SDK 硬件清零。不需要复制配置或生成本轮配置文件。

## 3 自动重力标定

以下命令会真实控制右臂运动。使用已经审核的 18 个姿态，J1～J5 固定零位，
只改变 J6/J7；Position 纯位置模式，速度和加速度均为 10%。
保持工具、线缆和环境与审核时一致，其他 SDK 客户端保持断开。

```bash
python -u "$HOME/tianji_ws/src/tianji_io_teleop_bridge/scripts/right_ft_calibration.py" auto
```

执行 `auto` 即启动已审核流程，不再要求 `--connect`、`--poses-reviewed`、
`--zero-note`、`--config`、`--poses`、`--current-file` 或 `--output`。
这些参数仍可用于显式覆盖或兼容旧命令。
默认使用 `config/right_ft_calibration_robot.yaml` 和 `config/right_ft_calibration_poses.yaml`，
零点备注由程序提供简短默认值；不再依赖清零记录文件，也不自动寻找历史零点。
操作者负责按“无工具清零成功 → 安装工具 → auto”的顺序执行。

程序自动在 `~/ft_calibration_runs` 创建唯一工作目录，保存采样并用于拟合、验证，
无需手动管理目录。每次重试自动使用新目录；先处理失败原因并恢复七轴零位后再运行。
标定期间保留静止性、反馈新鲜度、腕部组合限位及 10% 速度/加速度检查。

## 4 结果与下一步

成功时终端显示“验证通过”和“固定标定文件已更新”，并打印质量和质心。
固定标定文件自动覆盖到：

```text
/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml
```

当前验证阈值为力矢量 RMS ≤0.20 N、力矩矢量 RMS ≤0.01 N·m。
失败或中断不会更新固定文件，旧文件不能当作本轮成功结果。
完成后右臂保持最后目标 `[0, 0, 0, 0, 0, -25, -35]°`，不会自动回零，SDK 自动释放。
之后启动桥接加载新结果；每条录制前在起始位无接触静止后执行软件去皮。

## 5 启动遥操作桥接和实时补偿

确认本轮标定及固定文件更新成功后执行，桥接持续占用该终端；此命令不依赖前面终端的变量：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash

test -s /home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml && \
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/bridge_both_test.yaml \
  right_ft_calibration_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml \
  connect_on_start:=true \
  allow_motion:=true
```

文件检查失败时不会启动桥接。该配置当前为**仅右臂、关节阻抗遥操作、速度和加速度 30%/30%**，
与自动标定的 Position 10%/10% 是两个阶段。启动桥接不会自动开始跟随，需第 7 步启用。
补偿直接作用于 `/io_teleop/right/ft_sensor`，软件去皮成功前该话题不发布数据。
桥接只在启动时加载标定文件；更新固定文件不会热切换正在运行的桥接。

## 6 启动方向 HUD 和操作员窗口

终端二启动原来的插孔方向界面：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash

ros2 launch tianji_io_teleop_bridge force_hud.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/force_hud_insertion_preview.yaml
```

终端三打开本地图像窗口：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
/usr/bin/python3 /opt/ros/humble/lib/rqt_image_view/rqt_image_view
```

刷新后选择 `/tianji_force_hud/image/compressed`，移到操作员屏幕并最大化。
箭头表示接触侧，与横向外力方向相反；横向力不超过 0.5 N 时不显示箭头。
中心圆点和轴向数值显示轴向力。沿用原固定孔口参考，通过同帧关节姿态实时转换传感器方向。
去皮前显示等待数据，去皮后自动更新。

## 7 启用遥操作并到达采集起始姿态

艾欧或 TeleBox 按原方式开启操作端指令发布。终端四执行：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash

ros2 service call /tianji_marvin_teleop_bridge/enable_control \
  std_srvs/srv/SetBool '{data: true}'
```

完成主从对齐，确认状态为 `ACTIVE`，再由操作员移到采集起始姿态：

```bash
ros2 topic echo /tianji_marvin_teleop_bridge/control_status
```

查看后 Ctrl+C 结束 echo，不会停止桥接。起始姿态由操作员决定，无需回七轴零位才能软件去皮。

## 8 每条采集前的软件去皮

**每条数据都执行：停止上一条录制 → 回到起始姿态 → 无接触静止 → 软件去皮成功 → 开始下一条录制。**
保持采集时的控制模式，工具悬空且手不接触工具；保持机器人与操作端静止至少 5 秒。
终端四执行：

```bash
sleep 6
ros2 service call /tianji_marvin_teleop_bridge/right/zero_ft_sensor \
  std_srvs/srv/Trigger '{}'
```

必须看到 `success: true` 才开始录制；若返回 false，根据 message 处理后保持静止重试。
新去皮值替换旧值，不累计扣除。每条采集中固定该值，重力补偿继续逐帧计算。
该服务同时作用于记录话题与 HUD，不调用 HUD 的 `zero_force` 或 SDK 硬件清零。
每次成功去皮记录保存在 `/home/stw/ft_calibration_runs/teleop_tares`，响应包含确切文件路径。

可检查一条消息：

```bash
ros2 topic echo /io_teleop/right/ft_sensor --once
```

艾欧记录配置：

| 内容 | 话题 | 消息类型 |
|---|---|---|
| 补偿并去皮后的末端六维力 | `/io_teleop/right/ft_sensor` | `sensor_msgs/JointState` |
| 操作员 HUD 图像 | `/tianji_force_hud/image/compressed` | `sensor_msgs/CompressedImage` |

六维力记录 `effort`，顺序为 `Fx, Fy, Fz, Tx, Ty, Tz`，前三项 N、后三项 N·m。
记录数据保持传感器坐标系；HUD 为方向显示额外旋转坐标并平滑，不改动记录值。
原有关节状态和关节外力估计的录制配置继续保留。

## 9 结束采集

先停止艾欧录制，再在终端四关闭遥操作：

```bash
ros2 service call /tianji_marvin_teleop_bridge/enable_control \
  std_srvs/srv/SetBool '{data: false}'
```

等桥接回到 `MONITOR`，再 Ctrl+C 退出 HUD、桥接并关闭查看窗口。
机器人再次开机时从第 1 步重新开始。

## 常见中断与恢复

- **SDK 连接被占用**：退出其他桥接、上位机、标定或读力程序，再运行所需的一个 SDK 客户端。
- **无工具清零失败**：检查接线、静止状态和前后数据；保留失败记录。SDK 返回 true 不能替代实际读数检查。
- **标定目录已存在**：重新执行第 4 步，生成新时间戳目录；不覆盖原始数据。
- **标定未生成 calibration.yaml**：查看同目录 `result/REPORT.md`、`report.json` 或终端报错，不能用上一轮文件代替。
- **软件去皮失败**：检查工具无接触且连续静止，待窗口完整后重试；软件不会自动判断稳定接触是否存在。
- **桥接重启或 SDK 重连**：本次软件去皮失效，静止后重做；若机器人本身断电重启，则重新执行整套流程。
- **只重启 HUD**：不会清除桥接去皮值；下一条数据采集前仍按第 8 步重新去皮。
- **工具重装或硬件再次清零**：原标定零点不再可直接沿用，应重新完成对应的装配和标定流程。

更多算法和姿态细节见 [独立标定说明](RIGHT_FT_CALIBRATION.md)，在线消息约定见
[末端力传发布说明](RIGHT_FT_SENSOR.md)，方向显示见 [HUD 说明](FORCE_HUD.md)。
