# 工具参数服务：配置、准入与回读审核

本说明记录实现的准入与回读机制；离线测试不代表实际工具动力学已验证。
节点默认及 `bridge.yaml` 为 `tool_config_dry_run: true`，仅预览；当前
`bridge_both_test.yaml` 和 `bridge_right_retreat_test.yaml` 为 `false`，允许服务在其他
准入条件满足后真实下发。三个配置启动时均不会自动上传。不要将联调配置当成纯预览配置。

## 默认参数来源

正式/联调 YAML 中的 `tools_config_file` 指向：
`/home/stw/TJ_FX_ROBOT_CONTRL_SDK/MarvinPlatform_EN/tools_cfg.json`。
每次调用读取文件的 `current_tool`，不是默认选择 tool-1，也不把左右臂套用同一组参数。
读取失败、当前工具缺失、键重复、数组长度错误、非有限值等会拒绝，不自动使用其他工具。

审核时的选择与数值：

| ROS服务 | SDK / JSON | 当前工具 |
|---|---|---|
| `~/configure_left_tool` | A / arm0 | tool-2 |
| `~/configure_right_tool` | B / arm1 | tool-1 |

两臂 kine 均为 `[0,0,0,0,0,0]`。

```text
左臂 dyn = [0.195,-22.133,-42.953,13.967,0.003,0,0,0.008,0,0.008]
右臂 dyn = [0.166,-47.89,-3.741,28.536,0.001,0,0,0.016,0,0.004]
```

按天机 `c++_doc_contrl.md` 第14节及平台 `ui_EN.py`：

- kine：`X,Y,Z,A,B,C`，平移毫米、旋转角度（度）。
- dyn：`M,mx,my,mz,Ixx,Ixy,Ixz,Iyy,Iyz,Izz`。
- 所有数据原样传递，不转弧度、不转米、不更改符号，也不重排惯量。
- 长度、有限值、float可表达范围及非负质量检查不能代替动力学辨识和实际负载确认。动力学各项单位、物理含义及辨识结果有效性仍应按供应商确认，不能因服务返回成功就认为模型已经准确。

仓库 `config/tools_cfg.json` 是上述原文件的完整内容快照（仅多了文件末尾换行）。新电脑可以将
`tools_config_file` **显式设为空字符串**选用快照，或者指定自己的文件路径。指定非空路径不存在时不会静默回退。

## 默认行为与审核方法

三个参数均为启动时只读，修改 YAML 后重启生效：

```yaml
tools_config_file: "/home/stw/TJ_FX_ROBOT_CONTRL_SDK/MarvinPlatform_EN/tools_cfg.json"
tool_config_dry_run: true
tool_config_timeout_sec: 2.0
```

`tool_config_dry_run=true` 时，两个 Trigger 服务只校验和显示参数，`success=True` 的含义是
**预览成功，未上传**。无需机器人连接，不检查机器人状态，不调用 OnClearSet/OnSetTool/OnSetSend，
不修改已有的 `tool_dynamics_confirmed` 标志。启动节点也不会自动配置工具。

如需自行预览，请先确认不与正在执行的实机任务冲突。以下启动方式不连接机器人，并锁定运动：

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  right_ft_calibration_file:=/home/stw/ft_calibration_runs/本轮采集目录/result/calibration.yaml \
  config_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/bridge.yaml \
  connect_on_start:=false allow_motion:=false
```

上述标定路径需替换为本轮结果；只预览工具配置时也可在桥接 YAML 中设置 `publish_right_ft_sensor: false`，无需该路径。

另一个已 source 的终端可分别预览：

```bash
ros2 service call /tianji_marvin_teleop_bridge/configure_left_tool std_srvs/srv/Trigger '{}'
ros2 service call /tianji_marvin_teleop_bridge/configure_right_tool std_srvs/srv/Trigger '{}'
```

返回必须含 `DRY_RUN: no SDK write`，并展示臂、工具名、全部16个数值及路径。
这不是提交到机器人的操作；本次实现过程中没有执行上述真实节点启动/服务调用。

## 审核通过后真实下发路径的代码行为

真实上传需要显式设置 `tool_config_dry_run=false`、`allow_motion=true`、
`sdk_control_compatibility_confirmed=true`，请求臂还需包含在 `enabled_arms` 内。
`tool_dynamics_confirmed` 不作为上传前置条件，避免“先配置才能确认、先确认才能配置”的循环。

每次真正上传必须满足：

1. MONITOR，无运动、IDLE重试或其他工具上传事务。
2. 双臂反馈有限且新鲜，双臂控制器都是健康的 IDLE；各关节速度不超过0.5度/秒。
3. `OnClearSet` → 对应的 `OnSetTool_A` **或** `OnSetTool_B` → `OnSetSend`，同一个包包含 kine 和 dyn。
   不调用另一臂设置函数，不切换控制模式、不发送关节目标、不主动发送IDLE。
4. SDK忙直接拒绝，不覆盖已有包。发送成功返回 `SENT_PENDING_ECHO`，仅表示已提交。
5. 在现有反馈定时器中非阻塞检查：等待发送后至少 `sdk_packet_settle_sec`、反馈帧推进，
   并比对该臂 `m_In[].m_ToolKine[6]` 和 `m_In[].m_ToolDyn[10]`。按SDK float转换比较，绝对容差1e-6。
6. 匹配后发布 `VERIFIED`。失败/超时发布 `UNCONFIRMED`，不自动重发、回滚或切模式。
   待确认期间和结果不确定时，遥操作使能、回初始姿态及后退均被拒绝；重新配置失败的臂并验证成功后才能解除。
   各臂不确定状态分别记录，右臂成功不会清除左臂的失败状态。

状态话题（可靠、保留最近一条）为 `/tianji_marvin_teleop_bridge/tool_configuration_status`：

```bash
ros2 topic echo /tianji_marvin_teleop_bridge/tool_configuration_status \
  --qos-durability transient_local --once
```

只有状态中的 `VERIFIED` 表示参数回读一致；仍不代表实际负载辨识准确。`control_status` 另含
`tool_config_pending` / `tool_config_uncertain`。发送失败可能无法判断控制器是否已经应用，因此不会假称“完全未生效”。
不要用重启节点绕过失败审核。工具在机器人断电后需重新下发，代码不会自动重发。

## 与现有功能的边界

- 沿用桥接节点的唯一 SDK 连接和单线程执行器，不另开控制连接，不用会阻塞等待的 `SetTool()`便利函数。
- 配置服务不会自动将人工参数 `tool_dynamics_confirmed` 设为true；实际确认后仍由操作员设置并重启。
- 不修改进程内 libKine 的工具矩阵。已有六维力估计和后退规划继续以**法兰**为参考，避免悄悄变成TCP。
  当前文件的kine均为零；未来若采用非零TCP，需另外审核坐标与力矩参考点，不能混用。
- 依赖新增 `yaml-cpp`，可通过 `rosdep install` 或 Ubuntu 的 `libyaml-cpp-dev` 安装。

## 主要代码位置

- `src/tool_configuration.cpp`：读取 current_tool、数组校验、报告、float回读比对。
- `src/marvin_sdk_adapter.cpp` 的 `configure_tool`：仅单臂工具参数打包。
- `src/marvin_teleop_bridge_node.cpp` 的 `on_configure_tool` / `process_tool_upload`：预览、准入及非阻塞确认。
- `test/test_tool_configuration.cpp` / `test/test_target_send.cpp`：原值映射、零写入预览、左右臂隔离、忙碌、回读、超时等离线测试。
