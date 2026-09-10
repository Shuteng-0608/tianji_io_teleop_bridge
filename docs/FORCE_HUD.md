# 真机估计力 HUD（只读基础版）

本功能用于数据采集时向操作员显示左右臂估计力，不是六维力传感器，也不是安全保护。
不调用 `enable_control`、初始姿态或任何机器人控制API；HUD的launch不会启动桥接节点。
不修改原始相机图像或数据集。支持桥接节点以 `allow_motion:=false` 运行时独立观察。

## 1. 实现范围与数据含义

```text
已有 Marvin 桥接节点
  /io_teleop/joint_states [position(rad), effort(m_EST_Joint_Force)]
       ↓ 最新样本，按 Joint1_L..Joint7_L / Joint1_R..Joint7_R 名称重排
  force_hud_estimator_node [独立进程，仅链接libKine，默认100 Hz]
       ↓ /tianji_force_hud/estimates [双臂原始六维估计 + 有效性 + 源时间戳]
  force_hud_node [独立进程，按时间常数滤波]
       ├─ /tianji_force_hud/image/compressed [30 Hz JPEG]
       └─ /tianji_force_hud/display_values [同图像时间戳的显示数值/来源]
```

沿用仿真HUD的操作员专用显示思路，但不引入MuJoCo、Vision Pro、WebRTC依赖。
默认 `display_mode: ring`：左右臂各一个力圆环；保留 `bars` 六轴条形图用于调试。
圆环在方向映射未确认时只显示合力大小，**圆周进度不是受力方向**；面积和圆周进度随合力增加。
轴向/横向分解必须通过下述双重确认后才启用，不照搬仿真的插入轴、SAFE/JAM标签和力阈值。

输出按 `Fx,Fy,Fz,Mx,My,Mz` 排列。估计沿用 `FX_Robot_JntTau2EETau`，参考点是**法兰**，
没有转换到工具尖端。参考点不等于分量表达坐标系。单位/轴向/符号未确认前标为
`UNVERIFIED`，N*/Nm*只是待核实的SDK单位约定，不表示已经定量标定。

默认不做自动归零、重力二次补偿、TCP/相机变换、方向引导或相机叠加。
这些操作需确认SDK定义、负载、工具几何及任务轴后再实施。

## 2. 构建

先按主README完成Ubuntu 22.04、ROS 2 Humble及匹配的Marvin SDK配置。新增依赖包括
Eigen、Python OpenCV/NumPy和ament pytest；`rosdep`会根据package.xml安装。

```bash
cd /home/stw/tianji_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --packages-select tianji_io_teleop_bridge \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DMARVIN_SDK_ROOT=/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++
source install/setup.bash
```

新电脑需将工作空间与SDK路径改为实际路径。Python节点使用系统Python，不需要Conda。

## 3. 只读启动流程

如果桥接节点已经运行，不要再启动第二个桥接节点；只执行步骤2。

步骤1：需要时单独启动桥接的只读状态发布（会连接机器人，但不会下发运动指令）：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py allow_motion:=false
```

保持原桥接配置 `publish_cartesian_disturbance: false`，HUD自己计算，不需要开启原有Wrench话题。
HUD启动命令不控制桥接节点已经具有的运动权限；如果原桥接以true运行，该权限仍然存在。

步骤2：新终端启动HUD：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 launch tianji_io_teleop_bridge force_hud.launch.py
```

步骤3：查看图像端点、发布频率及诊断：

```bash
ros2 topic info -v /tianji_force_hud/image/compressed
ros2 topic hz /tianji_force_hud/image/compressed
ros2 topic echo /tianji_force_hud/estimates --once
ros2 topic echo /tianji_force_hud/display_values --once
```

将艾欧接收端配置到 `/tianji_force_hud/image/compressed`，消息类型
`sensor_msgs/msg/CompressedImage`，默认JPEG、1280×720、30 Hz、BEST_EFFORT/VOLATILE。
不要直接echo整幅图像的data数组。图像是独立不透明画布，**不是透明叠加层**。
若艾欧只支持RELIABLE，修改本机HUD配置为 `image_qos_reliability: reliable` 并重启。
需要双方处于相同ROS_DOMAIN_ID，并允许DDS网络通信；不能沿用离线测试的localhost限制。

停止HUD：在HUD终端Ctrl-C，不会停止桥接或改变机器人控制模式。

## 4. 配置

复制 `config/force_hud.yaml` 到受gitignore保护的 `config/local/force_hud.yaml` 后修改；
不要直接修改供应商尚未确认的物理约定。通过以下方式加载本机配置：

```bash
ros2 launch tianji_io_teleop_bridge force_hud.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/local/force_hud.yaml
```

参数按文件中的 `tianji_force_estimator` / `tianji_force_hud` 节点分组。参数为启动时只读，
修改需重启HUD；更改一个节点的estimate话题时，另一个节点也必须同步修改。

| 参数 | 默认值 | 作用 |
|---|---|---|
| state_topic | /io_teleop/joint_states | 必须是effort含关节扰动估计的状态，不是目标指令或电机总扭矩 |
| estimate_topic | /tianji_force_hud/estimates | 两进程之间的双臂原始估计与质量报告 |
| kinematics_config_file | 空 | 使用包内校验过的ccs_m6_40模型 |
| estimate_rate_hz | 100.0 | 计算/报告频率，上限500；不改变桥接发布频率 |
| input_timeout_sec | 0.2 | 两节点均检查输入/源时间戳超时；不是控制保护参数 |
| future_tolerance_sec | 0.05 | 容许的源时钟超前量，需保持电脑时钟一致 |
| jacobian_length_scale_m | 0.1 | 条件数计算的长度归一化参数，依赖待验证的SDK单位约定 |
| max_scaled_condition | 200.0 | 数值病态拒绝阈值，不是标定后的可信度 |
| max_relative_residual | 0.5 | 关节扰动无法由末端等效力解释时拒绝显示 |
| max_force_norm / max_torque_norm | 1000.0 / 100.0 | 异常数值上限，不是安全/接触阈值 |
| wrench_convention_confirmed | false | 仅在核实单位、表达坐标系、符号后改true；不证明测力精度 |
| left_wrench_frame / right_wrench_frame | 空 | 记录真实SDK输出的表达坐标系；填写名称不会执行TF变换 |
| image_topic | /tianji_force_hud/image/compressed | 操作员JPEG图像 |
| image_rate_hz | 30.0 | 允许1–60 Hz；等待/失效时仍发布状态画面 |
| image_width / image_height | 1280 / 720 | 允许640–1920 / 360–1080；推荐16:9 |
| jpeg_quality | 80 | 允许1–100，影响清晰度和带宽 |
| image_qos_reliability | best_effort | 可选reliable；两者均depth=1、volatile |
| smoothing_time_constant_sec | 0.08 | 六分量统一低通时间常数；0禁用，仅影响显示 |
| force_display_range / torque_display_range | 20.0 / 2.0 | 圆环/条形图满量程；超过显示RANGE提示，数值不截断 |
| display_mode | ring | ring操作员圆环视图；bars保留完整六轴调试图 |
| ring_show_torque | false | 圆环顶部额外显示合力矩，不显示力矩方向 |
| trend_window_sec | 0.8 | 圆环合力变化窗口；显示Delta而不是N/s，预热不足显示-- |
| left/right_direction_mapping_confirmed | false | 每臂独立确认固定显示/任务轴的映射 |
| left/right_direction_frame | 空 | 各臂轴向量所处的wrench表达坐标系，必须与估计消息一致 |
| left/right_screen_right_axis | [1.0,0.0,0.0] | 屏幕向右轴，在direction_frame中表达；默认只是占位值 |
| left/right_screen_up_axis | [0.0,1.0,0.0] | 屏幕向上轴，绘制时正确转换为图像y向下 |
| left/right_axial_axis | [0.0,0.0,1.0] | 任务轴，正方向由现场定义；默认只是占位值 |

### 圆环与条形图切换

在本机配置的 `tianji_force_hud.ros__parameters` 下设置 `display_mode: ring` 或
`display_mode: bars`，重启HUD即可。原先没有此参数的配置也默认使用ring。

没有完成方向确认时：中心数字、圆盘面积和外圆周进度都代表 **合力模长**，显示
`MAGNITUDE ONLY / DIRECTION LOCKED`，不出现横向力箭头或“轴向力”标签。

完成确认后：箭头表示横向受力方向/大小，中心圆表示轴向力大小，青色表示轴向正、琥珀色表示负，
并显示带符号轴向值；**颜色表示符号，不表示安全或危险**。箭头始终表示受力，不是操作方向建议。
`Delta |F|` 是近期滤波后合力模长变化，正值表示增大，不自动解释为卡滞。
失效时清空数值、箭头和填充，保留灰色圆环与原因；恢复后滤波和趋势重新建立。

每条臂启用方向显示同时要求：

1. 估计消息的 `convention_confirmed=true`，即天机单位/符号/表达坐标系已经核实。
2. 该臂 `*_direction_mapping_confirmed=true`，`*_direction_frame` 与消息的wrench_frame完全相同。
3. 屏幕右、屏幕上和任务轴三向量均为有限的三维单位向量，且相互正交。

此实现是 **固定轴投影**：分别计算 `dot(F, right/up/axial)`。不要直接把默认占位轴确认为真。
如果相机或工具在wrench坐标系内转动，这些固定轴通常不能继续代表相机/工具方向；
需要后续加入带时间戳的TF变换。在此之前保持方向确认关闭，或使用明确的固定任务平面。
仅填写frame名称不会执行任何坐标变换。不支持任意斜视相机的透视投影。

## 5. 数值质量、失效与时间戳

- 输入必须包含恰好14个唯一标准名称、14个position和14个effort，按名称同时重排两数组，
  再将rad转换为运动学API要求的deg。缺失effort不是零扰动，而是无效输入。
- SDK失败、非有限数据、病态雅可比、高残差、异常幅值均显示无效；左右臂独立判断。
- 条件数来自雅可比平移行除以特征长度后的SVD；残差为
  `||J^T w - tau|| / max(||tau||, 0.1)`。这是数值诊断，不能排除摩擦、重力模型误差或连杆接触。
- 源时间戳和本机单调时钟都参与超时检查；重复源时间戳不反复滤波，不延长有效数据寿命。
  输入断流或估计器退出后，显示进程继续发布STALE画面，旧数字被隐藏。
- `/estimates` 使用 **JointState原始时间戳**，不因定时重发刷新；两个手臂有各自的
  `wrench_frame` 字段，数组header.frame_id为空。
- JPEG的header.stamp是 **HUD画布生成时间**，不是测力时间。它不是物理相机，frame_id为空。
  `/display_values` 与该图像使用同一header，并包含 `source_stamp_ns`、`source_age_sec`。
- 桥接源stamp目前是反馈读取/发布时间，不是硬件采样时刻；500 Hz状态不保证500个独立硬件帧。
- 0.08秒滤波存在响应滞后；幅值上限检查发生在滤波之前。尚无短时峰值保持，短于显示周期的
  冲击可能在画面中不明显，原始数值必须另行记录，不能依赖HUD识别快速碰撞。

中间消息为 `diagnostic_msgs/msg/DiagnosticArray`：恰好两个status，name是left/right。
schema_version=1，包含reference_point、wrench_frame、convention_confirmed；计算成功时有
raw_Fx/Fy/Fz/Mx/My/Mz、scaled_condition、relative_residual。level为WARN表示未核实约定，
ERROR表示不得显示为有效力。数值检查拒绝时可能保留raw字段用于审计，但显示端必须遵从level。

## 6. 数据记录

建议保存原始joint_states、/estimates、/display_values、原始相机及操作指令。
`/display_values`保存原始和滤波分量、display_mode、direction_active、显示量程及趋势；
方向开启时还保存投影的screen_right/screen_up/axial分量，便于追溯显示结果。
HUD图像可选录，不覆盖原始相机。
另行保存使用的YAML、SDK/模型版本和后续标定数据。不将估计数据标为传感器真值。

## 7. 无硬件测试

```bash
cd /home/stw/tianji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select tianji_io_teleop_bridge
colcon test-result --verbose

/usr/bin/python3 src/tianji_io_teleop_bridge/test/hud_pipeline_probe.py --duration 60
# 可选：回归原条形图模式
/usr/bin/python3 src/tianji_io_teleop_bridge/test/hud_pipeline_probe.py --duration 5 --display-mode bars
```

probe在自己的子进程内设置localhost-only和测试ROS域，并使用随机唯一的 `/hud_probe_*` 话题。
只启动估计器/显示器，向隔离话题发布500 Hz合成状态，检查100 Hz估计和30 HzJPEG、解码、
缺失effort、断流、恢复及估计器退出。不会向 `/io_teleop/joint_states` 或控制话题发布。
合成姿态与零力只验证软件链路，不代表真实机器人姿态或力。输出JSON、日志和有效/失效截图
保存到自动生成的 `/tmp/tianji_hud_probe_*` 目录；使用 `--output-dir` 可指定结果目录。

尚需现场验收：开启HUD前后对比桥接500 Hz状态/100 Hz控制链路、运行十分钟以上，
检查CPU/带宽、头显实际刷新率与端到端延迟；经现场授权后再做已知方向/幅值的接触验证。
只读HUD联调本身不需要调用enable_control。

## 8. 待确认与下一阶段

### 未标定方向预览（人工方向观察）

使用独立 `config/force_hud_direction_preview.yaml`，不要将任何 confirmed 参数改为 true 来试箭头。
此配置保持约定及映射未确认，仅显式打开左右臂 `*_direction_preview`。
正常 `force_hud.yaml` 仍关闭预览；帧名必须匹配，质量检查和断流失效仍生效。
`left_sdk_base` / `right_sdk_base` 是各臂 SDK 模型基座的标签，不是相机坐标，也不建立 TF。
默认 +Fx→右、+Fy→上、+Fz→轴向正值；这是观察用映射，不保证对应现场推力方向。
画面标注 `UNCALIBRATED PREVIEW`，诊断保持 WARN，并记录 `direction_preview=true`。

先停止旧 HUD（不要重复启动），保持桥接仅作反馈读取，然后运行：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 launch tianji_io_teleop_bridge force_hud.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/force_hud_direction_preview.yaml
```

先无接触观察基线。现场确认机器人姿态、支撑、模式允许轻微接触后，再单臂逐方向轻微加载并释放，
不要推动连杆、冲击、强扭、靠近夹点或为得到箭头而增大外力。只读模式并不代表机器人适合手推。
固定相机和姿态，记录画面右/左/上/下/远离/靠近相机六方向、箭头变化及轴向符号，左右臂分别测试。
显示未做自动去零，要比较加载与释放之间的变化，而不是仅看某一时刻箭头。
纯轴向输入可能只有中心圆大小/颜色及 Axial 数字变化；它不一定产生平面箭头。
无效/STALE 时停止实验并检查诊断，不要放宽质量阈值。结束后停掉预览，正常启动即可恢复方向锁定。

1. 天机：扰动估计补偿内容、模式依赖、单位/符号、SDK雅可比及六维输出的坐标约定。
2. 现场：工具负载、质心、法兰到TCP、无接触基线和不同姿态误差；之后再决定零偏采集服务。
3. 艾欧：JPEG格式、QoS、分辨率、话题、镜像/旋转、独立窗口或叠加方式及延迟。
4. 任务：插入轴、相机外参、固定映射是否适用；核实后再开启已提供的受力圆环方向显示。

本版不改变任何机器人限位、超时、阻抗或控制门；显示颜色不承担安全判定。

## 9. 开发期离线基准（2026-09-07）

初版bars视图：Legion-Y9000P本机，合成状态源，1280×720/JPEG80，采样60秒，非实机/头显测试：

| 链路 | 实测平均频率 | P99消息间隔 | 最大间隔 |
|---|---:|---:|---:|
| Python合成状态源（设置500 Hz） | 496.300 Hz | 2.95 ms | 7.55 ms |
| 独立六维估计报告 | 99.998 Hz | 11.73 ms | 15.30 ms |
| JPEG HUD | 30.000 Hz | 36.43 ms | 42.02 ms |

图像生成时间戳到本机订阅回调的P99约14.52 ms，不包含源采样、滤波和艾欧显示延迟。
平均JPEG约86.4 KB，30 Hz约2.59 MB/s（20.7 Mbps，未含DDS协议开销）。
这不是硬实时保证；无线链路或额外相机流下需要重新测量。

圆环更新后本包自动化结果：57项测试，0错误、0失败、0跳过（包含ROS链路测试；不含工作空间其他包的结果）。
覆盖Humble诊断消息byte类型兼容、双臂重排、滤波、缺失/非有限数据、时间异常、
奇异姿态、重复关节名称、JPEG解码、断流/恢复及估计器退出。未执行任何实机运动测试。
新增覆盖ring/bars模式、双重方向确认、左右臂独立投影、屏幕y轴符号、无效数据清除箭头、
不合法/非正交轴拒绝、趋势预热与恢复。方向视图只用合成数据验证，未填入任何实机标定结果。

圆环版另测60秒：1800帧，平均29.999 Hz，P99间隔37.66 ms，最大44.07 ms；
估计报告100.004 Hz。平均JPEG约70.3 KB，本机生成到订阅回调P99约19.19 ms。
旧bars模式另测5秒，平均29.997 Hz，全部失效/恢复检查通过。以上均为隔离合成数据测试，
不包含艾欧网络/头显延迟，也不证明实机力方向已经标定。

## 10. 插孔任务视图：固定孔口坐标系

### 显示含义与坐标转换

`force_hud_insertion_preview.yaml` 使用 `ring_semantics: contact_side`。
圆环代表固定孔口截面，X 向右、Y 向上、Z 从孔口朝向机器人，插入方向为 -Z。
这是右手坐标约定；不是同时规定 X 向右、Y 向上、Z 朝孔内的镜像坐标。
箭头及环上扇区表示**疑似接触侧**，不是 peg 受到的力方向，也不是移动指令。
单侧接触主导时，侧向箭头为 `(-Fx_task, -Fy_task)`；右孔壁反力向左，因此箭头向右。
两侧同时接触或存在弯矩时，合力可能抵消，不能用短箭头判定无碰撞或定位全部接触点。

固定基准轴矩阵 `A` 的三行分别为 `*_screen_right_axis`、`*_screen_up_axis`、
`*_axial_axis`，均表达在相应臂的 SDK 基座坐标中。
本模式要求单位、正交、右手（det=+1），拒绝用单轴反射假装坐标旋转。
原始力方向预览仍兼容原有的正交显示映射。

```text
F_task = external_force_sign * Rz(-task_yaw_deg) * A * (F_filtered_base - bias_base)
arrow  = (-F_task.x, -F_task.y)
```

`task_yaw_deg` 表示任务轴相对固定基准轴绕已对齐的 +Z 旋转多少度。
例如参数 +90°，基准力 (1,0,0) 变成任务力 (0,-1,0)，接触侧箭头朝上。
它不是实时腕部 yaw，不跟随机械臂姿态更新。必须先将 Z 对齐孔轴，才能只调这一角度。
`external_force_sign` 只能取 +1 或 -1，用于将 SDK 符号统一为“环境作用于 peg 的力”，
与坐标旋转、接触侧箭头的反号分开处理。+1 只是待现场验证的初值。

**2026-09-10 已将插孔配置的单位阵替换为当前参考姿态计算的候选轴，yaw=0；
它仍不是完整孔口标定。** 数据、假设及矩阵见 [参考采集记录](HOLE_REFERENCE_2026-09-10.md)。
仍使用未标定预览、约定及映射确认均为 false，保留帧名、时间及数值质量门。
需要现场验证两臂基座相对墙面/孔的方向 A；可用对孔参考姿态的 FK、peg 安装关系和
孔口上方方向确定。SDK FK 不知道墙面外参，本版不自动捕获参考姿态，也不声称已标定。
当前仅转换力向量方向，不将法兰力矩平移到 peg 尖端，原始六维估计保持不变。

### 启动与调节参数

先停止旧 HUD，不重复向同一图像话题发布。保持桥接只读，无需调用 enable_control。

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS \
ros2 launch tianji_io_teleop_bridge force_hud.launch.py \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/force_hud_insertion_preview.yaml
```

异步发布仅应用于此次 HUD 进程，不能保证远端 30 Hz；网络接收仍需单独测量。
话题保持 `/tianji_force_hud/image/compressed`，JPEG、1280×720、目标 30 Hz。
配置参数是启动期只读，修改 YAML 后重启 HUD，无需重启桥接。

| 参数 | 初值 | 作用 |
|---|---|---|
| ring_semantics | contact_side | 插孔预览用接触侧；force 恢复原受力箭头 |
| left/right_task_yaw_deg | 0.0 | 各臂固定基准对齐后的孔轴旋转角，单位度 |
| left/right_external_force_sign | 1.0 | SDK 到环境作用力的符号，需验证 |
| lateral_display_deadzone | 0.5 | 横向力范数不超过此值时隐藏箭头，不改变数值 |
| axial_display_deadzone | 0.5 | 轴向力绝对值不超过此值时不扩大中心圆 |
| force_display_range | 20.0 | 箭头/中心圆满量程，不是安全限值 |
| zero_window_sec | 1.0 | 手动去零取最近这段有效数据的均值 |
| zero_max_std | 0.3 | 去零窗口单分量最大标准差，非接触判据 |

数值单位继承估计力单位（名义 N，未确认时显示 N*）。0.5/0.3 是可调初值，不是标定结果。
中心圆面积随 `|Fz_task|` 增大，数字仍显示带符号轴向力；正负配色不是风险等级。
死区仅影响可视图形，诊断及数字不删去小分量；它不能证明未接触。
插孔模式不显示方向力矩，原 bars 模式和估计器输出不变。

### 手动显示去零

去零只修改 HUD 的显示基线，不改关节状态、SDK 估计、机器人控制或已记录的 raw/filtered 值。
每臂独立，启动时无零点，重启后不保存。先确保末端无外部接触、保持参考姿态至少 1 秒：

```bash
ros2 service call /tianji_force_hud/left/zero_force std_srvs/srv/Trigger '{}'
ros2 service call /tianji_force_hud/right/zero_force std_srvs/srv/Trigger '{}'
```

调用立即检查近期窗口并返回成功/失败，不阻塞图像线程等采样。
至少需要 20 个不同时间戳样本，覆盖 80% 窗口；无效、过期、样本不足或波动大时拒绝。
**稳定接触也可能通过数值稳定检查，所以不能靠在孔壁上去零。**
节点不能凭力稳定确认静止/无接触，操作员必须确认。
去零取原始估计的基座 XYZ 均值，再从显示用低通滤波力中扣除；滤波过渡可能需要短暂稳定。
不会随插入过程自动更新零点。改变姿态、工具负载或参考场景后应清除并在无接触状态重采，
单次零点不是随姿态变化的重力/摩擦模型。

```bash
ros2 service call /tianji_force_hud/left/clear_force_zero std_srvs/srv/Trigger '{}'
ros2 service call /tianji_force_hud/right/clear_force_zero std_srvs/srv/Trigger '{}'
```

帧/单位约定确认状态变化、检测到源时间回退或连续有效样本发生过大时间间隔时清除零点；
无效消息会清空待采样窗口，原零点可能保留，但无效期间绝不显示旧箭头。
显示诊断记录零点时间戳/基座偏置、完整 source-to-task 旋转矩阵、符号、yaw、死区、
投影分量与实际箭头分量，便于还原操作员看到的含义。`raw_*` 和 `filtered_*` 仍是去零前源坐标值。

### 离线验证

```bash
PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_force_hud_core.py test/test_insertion_hud.py
/usr/bin/python3 test/hud_pipeline_probe.py --insertion --duration 30
```

测试使用隔离 ROS 域与唯一话题，只有合成状态发布、运动学估计和显示，不连接机器人。
覆盖旋转顺序、接触侧正负号、拒绝反射、死区、去零稳定窗口/左右臂隔离/清除、失效隐藏箭头，
以及 JPEG 链路、分臂去零服务和断流时拒绝去零。现场验收仍需实际孔口轴标定和力符号验证。

2026-09-10 本次验证：整包 124 项测试通过（0 错误/失败/跳过，含工作区已有的其他功能测试）。
插孔模式隔离合成数据运行 30 秒、900 帧：29.998 Hz，P99 图像间隔 37.66 ms，最大 42.34 ms；
估计报告 99.998 Hz。上述是本机离线链路结果，不是艾欧网络/头显帧率，也不是实机接触定位精度。

提交前另将仅含本次 HUD 改动的暂存区快照导出到独立目录，完成干净编译及全部 76 项测试，
0 错误/失败/跳过。该快照不包含工作区另行开发的控制恢复/限位窗口改动，验证 HUD 提交可独立构建。
当前场景人工方向验证已由用户确认通过，记录见上述参考采集文档；仍不等同于力值精度标定。
