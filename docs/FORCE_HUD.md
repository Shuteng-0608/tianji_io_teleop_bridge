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

沿用仿真HUD的操作员专用面板思路，但不引入MuJoCo、Vision Pro、WebRTC依赖。
此基础版显示左右臂六个有符号分量、合力/合力矩、数据年龄、数值质量检查和失效提示。
不复制仿真的插入轴、方向箭头、SAFE/JAM标签、40/80/100 N阈值。

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
| force_display_range / torque_display_range | 20.0 / 2.0 | 正负条形图满量程；超过显示RANGE，数值不截断 |

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
`/display_values`保存原始和滤波分量，便于追溯操作员看到的数值；HUD图像可选录，不覆盖原始相机。
另行保存使用的YAML、SDK/模型版本和后续标定数据。不将估计数据标为传感器真值。

## 7. 无硬件测试

```bash
cd /home/stw/tianji_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select tianji_io_teleop_bridge
colcon test-result --verbose

/usr/bin/python3 src/tianji_io_teleop_bridge/test/hud_pipeline_probe.py --duration 60
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

1. 天机：扰动估计补偿内容、模式依赖、单位/符号、SDK雅可比及六维输出的坐标约定。
2. 现场：工具负载、质心、法兰到TCP、无接触基线和不同姿态误差；之后再决定零偏采集服务。
3. 艾欧：JPEG格式、QoS、分辨率、话题、镜像/旋转、独立窗口或叠加方式及延迟。
4. 任务：插入轴、相机外参、箭头表示受力还是调整方向；验证后再添加仿真式圆环引导。

本版不改变任何机器人限位、超时、阻抗或控制门；显示颜色不承担安全判定。

## 9. 开发期离线基准（2026-09-07）

Legion-Y9000P本机，合成状态源，1280×720/JPEG80，采样60秒，非实机/头显测试：

| 链路 | 实测平均频率 | P99消息间隔 | 最大间隔 |
|---|---:|---:|---:|
| Python合成状态源（设置500 Hz） | 496.300 Hz | 2.95 ms | 7.55 ms |
| 独立六维估计报告 | 99.998 Hz | 11.73 ms | 15.30 ms |
| JPEG HUD | 30.000 Hz | 36.43 ms | 42.02 ms |

图像生成时间戳到本机订阅回调的P99约14.52 ms，不包含源采样、滤波和艾欧显示延迟。
平均JPEG约86.4 KB，30 Hz约2.59 MB/s（20.7 Mbps，未含DDS协议开销）。
这不是硬实时保证；无线链路或额外相机流下需要重新测量。

本包自动化结果：44项测试，0错误、0失败、0跳过（包含ROS链路测试；不含工作空间其他包的结果）。
覆盖Humble诊断消息byte类型兼容、双臂重排、滤波、缺失/非有限数据、时间异常、
奇异姿态、重复关节名称、JPEG解码、断流/恢复及估计器退出。未执行任何实机运动测试。
