# 只读获取左臂当前末端法兰位姿

先读取实际关节角，再执行正运动学。左臂为 SDK A 臂，运动学索引为 0。
计算结果是 **SDK 左臂基座到法兰** 的变换，不是整机/world 坐标，也不是工具尖端。

## 已有 ROS 反馈时

1. 订阅 `/io_teleop/joint_states`，按名称提取 `Joint1_L` 到 `Joint7_L` 的 `position`。
2. 检查时间戳、有限值和反馈新鲜度；使用实际状态而非 `/io_teleop/joint_cmd`。
3. ROS 角度为 rad，乘 `180/pi` 转成 SDK 所需的 deg。
4. 按 `src/marvin_kinematics_adapter.cpp` 加载 `ccs_m6_40.MvKDCfg`，初始化左臂模型、移除本地工具变换。
5. 调用 `FX_Robot_Kine_FK(0, q_deg, T)`。`T[0..2][3]` 单位为 mm，发布 ROS 位姿时除以 1000；左上 3×3 为旋转矩阵，可转换为 `geometry_msgs/PoseStamped` 的四元数。

这条路径只需订阅反馈及本地 `libKine`，不需要连接控制 SDK。`frame_id` 应填写实际确认的基座 TF 名称。

## 本次直接读取示例

本次 ROS 话题发布者数量为 0，因此使用 `scripts/read_left_pose.cpp` 建立临时 SDK 反馈连接。
程序只调用连接、读取和释放接口 `OnLinkTo`、`OnGetBuf`、`OnRelease`；不调用模式、使能、目标、工具上传、清错或停止接口。
`FX_Robot_Tool_Rmv` 仅设置本进程的数学模型。程序要求左臂帧计数至少变化两次，读取超时则不输出位姿。
时间是主机读到反馈的时间，非控制器硬件采样时间。

在本机编译：

```bash
g++ -std=c++17 -Wall -Wextra -O2 \
  -I/usr/include/eigen3 \
  -I/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++ \
  /home/stw/tianji_ws/src/tianji_io_teleop_bridge/scripts/read_left_pose.cpp \
  -L/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++ \
  -Wl,-rpath,/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++ \
  -lMarvinSDK -lKine -pthread -o /tmp/tianji_read_left_pose
```

在没有桥接节点或其他 SDK 客户端占用本机连接时运行；已有反馈发布者时优先使用上面的 ROS 路径：

```bash
/tmp/tianji_read_left_pose /home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++/ccs_m6_40.MvKDCfg
```

已采集示例（快照，不代表运行后始终保持此位置）：

```text
host_capture_time: 2026-09-20T14:30:01+0800
host_capture_unix_ms: 1789885801433
left_feedback_frame: 376182
joint_degrees: [89.949447632, -89.785629272, -90.020858765, -90.004966736, -0.043807983, 0.118219756, -0.043773651]
position_m: [0.427270420, 0.304781196, 0.175724469]
quaternion_xyzw: [-0.499627977, 0.500568622, -0.498882777, 0.500918071]
rpy_degrees: [-90.022581663, 0.170521166, -89.937314775]
```

RPY 约定：`R = Rz(yaw) * Ry(pitch) * Rx(roll)`，不要直接当成其他欧拉角顺序或供应商 ABC。
姿态表示法兰坐标轴在左臂基座坐标系中的方向。

如需工具尖端 TCP，使用已标定的固定工具变换：

```text
T_base_tcp = T_base_flange * T_flange_tcp
```

若需要世界坐标，则再左乘 `T_world_base`；所有平移分量必须使用相同单位。
此次没有应用工具偏移或世界外参。
