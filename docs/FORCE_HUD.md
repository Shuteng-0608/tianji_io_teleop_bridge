# 右臂末端力传 HUD

HUD 默认直接显示 `/io_teleop/right/ft_sensor` 中已经完成工具重力补偿和本次软件去皮的数据，
与艾欧记录使用同一数据源。只显示右臂，左臂没有安装末端力传时不会生成左臂零值或冒充实测。
HUD 只订阅 ROS 数据并发布图像/显示诊断，不建立 SDK 连接、不调用机器人控制或传感器清零接口。

## 数据流程

```text
本轮独立标定结果 + 同帧实际关节角 + SDK 六维力
  → 桥接节点：实时重力/标定偏置补偿 + 固定的软件去皮
  → /io_teleop/right/ft_sensor [JointState, Fx Fy Fz Tx Ty Tz, N/Nm]
      ├→ 艾欧正式数据记录
      └→ force_hud_node：显示坐标转换（方向模式）、平滑和绘制
           ├→ /tianji_force_hud/image/compressed [100 Hz JPEG]
           └→ /tianji_force_hud/display_values [显示数值及原测量时间戳]
```

默认不启动 `force_hud_estimator_node`，不订阅关节扰动或 `/estimates`，断流后也不会回退到关节估计。
节点内部不重新缩放、重力补偿或去皮。六通道按名称重排，保持输入数据的单位、符号和传感器参考点。

插孔方向配置另外订阅 `/io_teleop/joint_states`，严格匹配同一个 DCSS 样本的时间戳，
本地 `libKine.so` 算出右臂法兰姿态后，把传感器力旋转到 SDK 基座系，再投影到原固定孔口轴。
该过程不连接控制器、不使用关节 effort 估计力，也不改变记录话题。

## 启动

每次机器人开机先完成本轮独立标定。桥接启动时显式传入本轮 `calibration.yaml`，
每条采集前回到起始姿态，无接触静止至少 5 秒，重新调用桥接软件去皮服务。
完整命令见 [操作流程](RIGHT_FT_RUNBOOK.md)。
桥接节点已经运行时不要重复启动桥接。

```bash
source /home/stw/tianji_ws/install/setup.bash
ros2 service call /tianji_marvin_teleop_bridge/right/zero_ft_sensor std_srvs/srv/Trigger '{}'
```

去皮返回 `success: true` 后，启动或重启 HUD：

```bash
source /home/stw/tianji_ws/install/setup.bash
ros2 launch tianji_io_teleop_bridge force_hud.launch.py
```

若原 HUD 正在运行，先在其终端 Ctrl-C，避免两个进程向同一图像话题发布。
HUD 启动命令不启动桥接、不改变遥操作模式，也无需再次加载标定文件。
在桥接去皮完成前可以先启动 HUD，此时显示等待数据。

```bash
ros2 topic echo /io_teleop/right/ft_sensor --once
ros2 topic hz /tianji_force_hud/image/compressed
ros2 topic echo /tianji_force_hud/display_values --once
```

艾欧图像端点仍为 `/tianji_force_hud/image/compressed`，类型 `sensor_msgs/CompressedImage`，
JPEG、1280×720、目标 100 Hz，默认 BEST_EFFORT/VOLATILE。力数据继续记录原 `/io_teleop/right/ft_sensor`。
图像为独立画布，不覆盖原始相机。

## 操作员插孔方向视图

停止原 HUD 后，用用户原来的命令启动带方向箭头的界面；桥接保持运行。
只重启 HUD 不会清除去皮值，但每条采集开始前仍须按流程重新软件去皮：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 launch tianji_io_teleop_bridge force_hud.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/force_hud_insertion_preview.yaml
```

另开本机桌面终端显示窗口（系统 Python 避免 Conda 与 ROS Python 版本冲突）：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
/usr/bin/python3 /opt/ros/humble/lib/rqt_image_view/rqt_image_view
```

刷新并选择 `/tianji_force_hud/image/compressed`，将窗口移到操作员屏幕并最大化。
已打开的查看器会直接收到新画面。

此配置恢复原来的右臂孔口三轴矩阵、`contact_side` 语义、横向箭头、接触侧圆弧和轴向力圆点：

```text
F_base = R_base_flange(q_same_stamp) × R_flange_sensor × F_sensor
F_task = external_force_sign × Rz(-task_yaw) × A_hole × F_base
arrow = -F_task.xy                  # 接触侧，与横向外力方向相反
center_circle = abs(F_task.z)       # 轴向符号由颜色/数值表达
```

横向和轴向显示死区仍为 0.5 N；去皮后无接触接近零时没有箭头是正常的。
先旋转每帧数据，再在固定基座系平滑，因此手腕旋转不会把旧姿态的传感器分量直接混入新姿态。
力矩只旋转坐标轴，参考点仍为传感器测量原点，未平移到法兰或工具尖端。

孔口参考沿用 [原方向配置及现场验证记录](HOLE_REFERENCE_2026-09-10.md)，保持原有 preview 标记，
不把这次软件验证视为新场景的方向确认。若孔口、相机视角或基座安装改变，需重新核对方向。
当前传感器到法兰旋转为厂家确认的单位阵。

## 配置和坐标

`config/force_hud.yaml`、`force_hud_insertion_preview.yaml`、`force_hud_direction_preview.yaml`
均默认使用右臂末端力传。通过 `config_file:=/绝对路径/配置.yaml` 选择配置，参数修改后重启 HUD。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `input_source` | `right_ft_sensor` | 数据来源；launch 的同名参数覆盖 YAML |
| `right_ft_topic` | `/io_teleop/right/ft_sensor` | 已补偿并去皮的六通道 JointState |
| `right_ft_frame` | `right_ft_measurement` | 必须与输入 frame_id 一致，填写名称不会执行坐标变换 |
| `right_ft_display_frame` | 与 `right_ft_frame` 相同 | 插孔配置为 `right_sdk_base`：启用同帧关节反馈与本地 FK |
| `joint_state_topic` | `/io_teleop/joint_states` | 方向模式使用 position，按右臂关节名称重排；单位 rad |
| `right_ft_sensor_to_flange_rotation` | 单位阵，9 项行优先 | 将传感器分量转为法兰分量的安装旋转，与当前厂家确认一致 |
| `display_arms` | `right` | 只有右臂有末端力传；选择 both 时左侧明确显示无数据 |
| `display_mode` | `ring` | 合力圆环；改为 `bars` 显示 Fx/Fy/Fz/Tx/Ty/Tz 六轴数值 |
| `smoothing_time_constant_sec` | `0.08` | 仅 HUD 低通平滑，设为 0 可显示未经平滑的输入 |
| `input_timeout_sec` | `0.2` | 数据接收、源时间戳及重复帧超时 |
| `future_tolerance_sec` | `0.05` | 源时钟超前容差 |
| `image_rate_hz` | `100.0` | HUD 图像发布频率，独立于源话题频率 |
| `image_qos_reliability` | `best_effort` | 接收端只支持 reliable 时显式修改 |
| `force_display_range` / `torque_display_range` | `20.0` / `2.0` | 图形量程，不截断记录值 |

界面标为 `FT SENSOR`，参考点为传感器测量原点。力矩不会自动平移到法兰或工具尖端。
现有记录话题保持传感器坐标系。默认 `force_hud.yaml` 显示合力大小；
`force_hud_insertion_preview.yaml` 经过上述实时坐标转换后使用原孔口方向映射。
`force_hud_direction_preview.yaml` 仍未启用方向；操作员插孔界面请选择 insertion 配置。

## 去皮与断流行为

软件去皮统一由桥接 `right/zero_ft_sensor` 完成，同时影响记录数据和 HUD 输入。
HUD 的旧 `zero_force`/`clear_force_zero` 服务在末端力传模式返回说明，不产生第二份显示偏置，
即使配置 `enable_display_force_zero: true` 也不会对末端力传重复去皮。

输入六轴缺失、名称重复、非有限值、坐标系不符、过期/未来时间戳会使显示无效。
断流或源时间戳持续重复超过 0.2 秒后隐藏数值，不维持旧数值、不填零、不切换到关节估计。
新鲜有效数据恢复后重置显示滤波，不将断流前的历史平滑到新数据中。
插孔模式还要求力和关节 position 的时间戳完全相同；任一流中断或无法配对超过 0.2 秒，
数值和箭头一起失效，不用最近姿态替代、不回退到未旋转的力。单条匹配样本的关节/FK 无效时立即清除。

`display_values` 保留图像生成时间和 `source_stamp_ns`，记录数据来源、处理状态及传感器参考点。
其中 `raw_Fx..raw_Tz` 指 HUD 显示坐标系中未平滑的值（见 `wrench_frame`），
已经经过桥接补偿和去皮，不是 SDK 原始计数；`filtered_*` 是图像实际使用的值。
`sensor_Fx..sensor_Tz` 保留传感器坐标系下收到的六轴值，`measurement_frame` 指明该坐标系。
方向模式还保存 `joint_source_stamp_ns`、`sensor_to_display_rotation_row_major`、
`display_to_task_rotation_row_major` 和完整的 `source_to_task_rotation_row_major`，用于复核方向计算。
内部兼容字段 Mx/My/Mz 与 Tx/Ty/Tz 数值相同。
HUD 平滑不改变 `/io_teleop/right/ft_sensor` 的记录值。

## 离线验证

```bash
cd /home/stw/tianji_ws/src/tianji_io_teleop_bridge
PYTHONPATH=scripts /usr/bin/python3 -m pytest -q \
  test/test_ft_hud.py test/test_force_hud_core.py test/test_insertion_hud.py
source /home/stw/tianji_ws/install/setup.bash
/usr/bin/python3 test/ft_hud_pipeline_probe.py
/usr/bin/python3 test/ft_hud_pipeline_probe.py --insertion
```

模拟话题使用隔离 ROS 域、localhost 和随机话题名；不启动桥接，不连接机器人。
验证六轴映射、源时间戳、旧关节/估计输入不参与显示、无二次去皮、异常/断流隐藏与恢复、JPEG 解码及图像频率。
方向转换测试覆盖同帧配对、名称/单位重排、传感器安装旋转、手腕运动下固定孔口力方向保持，
以及关节姿态缺失、错时、无效时箭头清除。所有模拟测试都不启动桥接或建立控制 SDK 连接；
尚需实机及艾欧接收端验证。

历史关节估计实现与试验记录保存在 [旧 HUD 说明](FORCE_HUD_JOINT_ESTIMATE_LEGACY.md)。
如需显式调试旧数据源，可使用 `input_source:=joint_estimate`；默认末端力传模式不会自动启用它。
