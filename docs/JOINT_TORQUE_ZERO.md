# 逐关节去皮与数据记录

桥接在 C++ 中处理每条臂的 7 个关节残余偏置：

```text
bias[j] = mean(无接触静止窗口内的 m_EST_Joint_Force[j])
tau_corrected[j] = m_EST_Joint_Force[j] - bias[j]
wrench = FX_Robot_JntTau2EETau(q_actual, tau_corrected)
```

左右臂分别估计，不合并为一个末端偏置。重复去皮始终从控制器原始估计取均值，
覆盖旧偏置，不累计扣减。运动过程中保持偏置固定，不把真实接触力自动归零。
根据厂家确认，控制器已做动力学补偿；这里仅去残余偏置，不再扣除重力，也不添加阻抗力矩。

## 使用步骤

重新构建后，在合适的停机时机重启桥接和 HUD，使用新程序。
先完成工具配置及计划使用的控制模式切换；在该模式下使待去皮机械臂静止、无接触且无人施力，
保持至少 1 秒，再调用所需服务。服务只修改本地遥测偏置，不发送任何机器人控制指令。

```bash
# 只对右臂去皮（仅采集右臂时推荐）
ros2 service call /tianji_marvin_teleop_bridge/right/zero_joint_torque std_srvs/srv/Trigger '{}'

# 双臂去皮：两侧都要静止且无接触，全部通过才一起更新
ros2 service call /tianji_marvin_teleop_bridge/both/zero_joint_torque std_srvs/srv/Trigger '{}'

# 查看每侧 active、zero_stamp_ns、revision 及 7 个偏置
ros2 topic echo /tianji_marvin_teleop_bridge/joint_torque_zero_status --once

# 清除右臂偏置，恢复控制器原始估计
ros2 service call /tianji_marvin_teleop_bridge/right/clear_joint_torque_zero std_srvs/srv/Trigger '{}'
```

上述服务的 `right` 均可换成 `left` 或 `both`。必须检查 `success: true`。
失败时旧偏置不变；双臂失败不会只更新一侧。
HUD 继续用已有的低通滤波，去皮后等待约 0.5 秒让画面稳定，再开始采集。

原 `/tianji_force_hud/*/zero_force` 只做末端显示去零，现默认禁用，调用会提示新接口。
`clear_force_zero` 仍可清除旧显示基线。`enable_display_force_zero: true` 仅供旧显示流程调试；
正式采集保持 false，避免画面又额外扣除一个数据记录中不存在的末端基线。
HUD 中的 `DISPLAY ZERO` 指旧显示基线，逐关节去皮状态以桥接 `joint_torque_zero_status` 为准。

## 记录哪些数据

右臂实时监测窗口（只读，7 个关节，数值均保留三位小数）：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 run tianji_io_teleop_bridge right_joint_torque_dashboard
# 可选：查看去偏置前的控制器估计
ros2 run tianji_io_teleop_bridge right_joint_torque_dashboard --raw
# 可选：离线界面预览，不连接 ROS
ros2 run tianji_io_teleop_bridge right_joint_torque_dashboard --demo
```

默认订阅 `/tianji_marvin_teleop_bridge/estimated_joint_disturbance`，
`--raw` 订阅 `/io_teleop/joint_states_raw`；不把主状态话题的反馈关节扭矩当作外力。
显示的是关节力矩估计（N·m），不是末端力（N）。数字不做额外低通滤波；三位小数只影响显示。
每侧条形默认范围 ±2.000 N·m，可用 `--range 5.0` 修改，范围不是安全阈值。
超范围时条形截断、数字仍显示完整值；断流、时间戳过期或无效 effort 显示“—”，不保留旧数值。
窗口显示右臂去皮状态，不发送控制或去皮请求，也不改变记录数据。不需要重启桥接即可打开。

| 默认话题 | 内容 |
|---|---|
| `/io_teleop/joint_states` | position/velocity 不变，effort 为 `m_FB_Joint_SToq` 反馈关节扭矩，不做扰动去偏置 |
| `/tianji_marvin_teleop_bridge/estimated_joint_disturbance` | 同帧position/velocity，effort 为逐关节去偏置后的 `m_EST_Joint_Force`，供HUD与正式扰动力数据使用 |
| `/io_teleop/joint_states_raw` | 原始 m_EST_Joint_Force，供复核；默认名称为 state_topic 加 `_raw` |
| `/tianji_marvin_teleop_bridge/joint_torque_bias` | 14 个关节名及实际扣除的 effort 偏置，position/velocity 留空 |
| `/tianji_marvin_teleop_bridge/joint_torque_zero_status` | 左右臂是否已去皮、时间、修订号及偏置；变化时及每秒发布，支持 transient-local |
| `/tianji_force_hud/estimates` | 从校正关节力矩计算的六维力；raw_Fx 等字段意为未低通滤波，已包含关节去皮 |
| `/tianji_force_hud/display_values` | 同一估计的滤波显示值及来源时间戳 |

以上三个 JointState 状态话题及偏置话题每帧使用相同 header.stamp，可按时间戳验证
`joint_states_raw.effort - joint_torque_bias.effort = estimated_joint_disturbance.effort`。
桥接可选的 `estimated_cartesian_disturbance` 也使用相同偏置。数据记录端不要再扣一次。
原始扰动估计非有限时，raw/corrected 两个扰动话题的 effort 都留空，不能当作零力；
主状态话题的反馈扭矩独立判断是否有限。

```bash
ros2 bag record -o joint_zero_session \
  /io_teleop/joint_states /io_teleop/joint_states_raw \
  /tianji_marvin_teleop_bridge/estimated_joint_disturbance \
  /tianji_marvin_teleop_bridge/joint_torque_bias \
  /tianji_marvin_teleop_bridge/joint_torque_zero_status \
  /tianji_force_hud/estimates /tianji_force_hud/display_values
```

此命令只示范受力链路；按采集任务加入相机和指令话题。
本仓库没有最终数据集写入程序：订阅标准 joint_states 的记录端收到的是反馈关节扭矩，
不是去偏置扰动；要保存后者，必须让记录端另外订阅 estimated_joint_disturbance 的 effort。
已录制的数据不会被本功能追溯改写。

启动时偏置为零、active=false，此时标准话题仍等于原始估计；必须在服务成功且目标臂 active=true 后
开始正式采集。采集期间不要重新去皮或切换模式，否则偏置会变化；可结束当前片段再重新标定。
进程重启、SDK 重新连接、对应臂控制状态/模式或反馈中的工具参数改变都会清除旧零点。
通过桥接发送工具参数（包括发送失败但结果不确定）也会清除对应臂零点。
普通位置变化不会清除偏置。短暂反馈中断清除采样窗口，保留已确定偏置。

## 判定参数

桥接启动参数（修改后重启）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| joint_zero_window_sec | 1.0 s | 最近采样窗口；至少 20 个不同控制器帧，跨度至少为窗口的 80% |
| joint_zero_max_std_nm | 0.05 | 每个关节原始估计力矩的最大标准差，沿用 SDK 力矩单位 |
| joint_zero_max_velocity_rad_s | 0.01 | 每个关节静止速度上限 |
| joint_zero_max_position_range_rad | 0.005 | 窗口内每个关节角的最大变化范围 |
| feedback_timeout_sec | 0.2 s | 最新独立帧及连续采样的最大间隔 |

阈值是可调初值。重复 SDK 帧不计数，非有限数据、运动、反馈失败会清除采样窗口；
旧数据、样本不足、姿态漂移、力矩波动过大或工具配置未确认时拒绝采集新零点。
稳定窗口不能判断是否有恒定接触力，需操作员确认无接触。

固定关节偏置只能去除近似固定的残余零偏，不能消除随姿态、速度、加速度变化的模型误差、
摩擦、温漂或雅可比病态放大。它不会保证所有无接触运动中的六维力都严格为零。

### 排查 unstable torque/position

拒绝消息会输出 `failed`、`torque_std_nm`、`position_range_rad`、各自阈值及窗口样本数/跨度。
`failed=torque_std` 表示力矩标准差过大；`failed=position_range` 表示关节位置漂移；
二者同时超限会同时列出。这里比较的是波动量，不是平均力矩的绝对值。
只报告首个不通过的关节，不能据此判断其余关节是否都通过。

2026-09-22 在当前桥接运行期间只读订阅了约 6 秒右臂原始反馈（约 300 Hz）。
其中五个连续 1 秒窗口的 J1 力矩标准差为 0.094～0.110 N·m，位置范围约 0.000001 rad，
最大绝对速度不超过 0.002 rad/s：这些观测支持原 0.05 N·m 阈值低于当时的静止力矩波动。
它们是 ROS 发布样本的统计，不是失败调用当时的内部去重窗口，也不能证明无接触。

因此仅将 `bridge_both_test.yaml` 的联调初值改为 `joint_zero_max_std_nm: 0.15`、
`joint_zero_window_sec: 2.0`；默认配置及速度/位置限制不变。
加长窗口有助于稳定均值，不意味着原始数据的标准差会降低。
新配置和详细错误信息需要重启桥接后生效；没有在线更改参数或自动重试去皮。
请在实际采集模式下、确认无接触且静止至少 2 秒后重试，阈值仍需现场验证。
