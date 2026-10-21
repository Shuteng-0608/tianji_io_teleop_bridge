# 独立右臂末端六维力工具标定

可直接复制的完整开机与逐条采集指令见 [操作流程](RIGHT_FT_RUNBOOK.md)。
日常直接执行 `right_ft_calibration.py auto`，默认使用 `config/right_ft_calibration_robot.yaml`；
无需复制配置、绑定清零记录或填写备注，输出目录自动创建。清零命令默认仅终端报告。
旧的显式参数示例仍可使用；执行 auto 前由操作者完成无工具清零和工具安装。

这是直接使用天机 SDK 的独立 Python 工具，不依赖 ROS、JointState 或现有发布节点。
`auto` 使用右臂 Position 纯位置模式，速度和加速度均为控制器关节限制的 **10%**，
从七轴零位自动按姿态表顺序走完 18 点、采样、保存、拟合和验证。保留 `collect` 手动调姿采集入口。
两个入口都不会执行力传清零或上传工具参数。
当前流程已完成多轮实机 18 点采集；开发回归测试使用离线模型和模拟反馈。

## 先审核姿态

- [姿态审核表](right_ft_calibration_review/REVIEW.md)：12 个标定点、6 个验证点，逐点列出 J6/J7 及限位余量。
- [角度范围图](right_ft_calibration_review/poses.png)、[CSV](right_ft_calibration_review/poses.csv)。
- 实际使用的姿态文件：`config/right_ft_calibration_poses.yaml`。可以编辑；每次启动会重新检查。

角度是 **SDK 的绝对关节角，单位度**。按操作者要求，自动流程每个目标的 J1～J5 都为零，
只改变 J6/J7。`collect` 手动入口仍可记录操作者选定的前五轴位置。

当前模型的约束：

```text
-60° <= J6 <= 60°
|J7| <= min(90°, 110.5° - 1.025 * |J6|)
```

系数来自 `CommonConfig/ccs_m6_40.MvKDCfg`；象限选取依据
`kinematicsSDK/FxRobot.cpp` 中的 `lmtj67_pp/np/nn/pn` 分支。
程序复现 SDK 在 J6 > 1°、J6 < -1° 时的分段检查；当前系数在中间区间与上式等价。
模型、控制库、运动学库及 Python DCSS 声明均校验固定 SHA-256，升级后需重新审查。

选点：J6=-40°、+40° 时，J7 取 -50°、-20°、+20°、+50°；
J6=0° 时，J7 取 -75°、-25°、+25°、+75°。
六个验证点为 J6∈{-25°,0°,+25°} 与 J7∈{-35°,+35°} 的组合。
全部点的 J6 边界余量至少 20°；固定 J6 时的 J7 边界余量至少 15°。
这个“角度余量”不是工具与其他物体之间的几何间隙。

操作者已手动逐点验证 18 个姿态。自动顺序保持姿态文件的 C01～C12、V01～V06 顺序，
启动时检查七轴零位、各目标及相邻目标关节角直线插值的腕部约束；实际运动中持续检查反馈。
这些检查不包含工具、线缆和环境的几何碰撞，也不保证控制器实际插值方式。
安装最终工具后，应按相同顺序确认点间路径。程序不新增过渡姿态，也不自动回零。
七轴零位下离线覆盖检查：力模型秩 4、条件数约 2.487；力矩模型秩 6、条件数约 6.690。
实际拟合仍依据反馈角度检查可辨识性。

## 依赖和离线查看

使用系统 `/usr/bin/python3`，依赖 NumPy、PyYAML；绘图另需 Matplotlib（可选）。
当前系统已具备这些依赖。不要使用缺少 NumPy 的其他 Python 解释器。
无需 source ROS，也无需先编译本 ROS 包。

```bash
cd /home/stw/tianji_ws/src/tianji_io_teleop_bridge
/usr/bin/python3 scripts/right_ft_calibration.py --help
/usr/bin/python3 scripts/right_ft_calibration.py plan
```

重新导出审核材料（完全离线，只读配置）：

```bash
/usr/bin/python3 scripts/right_ft_calibration.py plan --output /tmp/right_ft_pose_review
```

可用 `analyze --base-deg 0 0 0 0 0` 离线检查自动流程的重力方向覆盖。
此命令只加载本地 `libKine.so`，不加载控制库，不连接机器人。
输出力模型秩（应为 4）、力矩模型秩（应为 6）及归一化条件数（默认要求均不超过 30）。
覆盖检查不通过时 analyze 返回退出码 2；实际采集连接后的预检查也会拒绝继续采样。
检查依赖正确的基座重力方向。固定的传感器安装旋转不改变此项秩/条件数检查。

## 你启动自动标定

1. 使用“无工具清零 → 安装最终工具 → 标定”的零点流程。传感器受力侧的转接件和夹具都属于待标定负载。
   程序不会替你清零；机器人重启后应确认实际零点状态，不能假定此前清零必然保留。
2. 安装最终工具并将七轴置于零位，保持静止。启动时右臂应为 IDLE 或 Position。
   工具不得接触外物，采样时手要离开工具。
3. 释放桥接节点、天机上位机等其他客户端的 SDK 连接。本机 UDP 4730 为 SDK 独占。
   自动程序运行期间不要由其他客户端更改模式、目标或限速。
4. 若要自动拟合，先补齐下文的坐标、符号及实际基座重力方向配置。
   通用模板仍保留未确认项；本机已确认约定在 `right_ft_calibration_robot.yaml` 中。
   程序会在连接前检查并拒绝未配置的自动拟合。
5. 使用下面命令。程序自动创建新的工作目录，无需手动准备。

```bash
/usr/bin/python3 -u /home/stw/tianji_ws/src/tianji_io_teleop_bridge/scripts/right_ft_calibration.py auto
```

命令启动后连接机器人并自动执行已审核的姿态及路径，无需每点回车。
若只希望自动走点保存原始数据、稍后再补坐标/符号并拟合，可在同一命令后增加 `--collect-only`。
`auto` 默认连接；手动 `collect` 仍需显式给出 `--connect --poses-reviewed`，其他命令完全离线。

自动流程：

- 先只读 1 秒，确认七轴在零位容差内且静止，通道 216、反馈帧正常更新，力传不是全零或六通道完全冻结。
- 同一命令包设定速度/加速度 10%/10%、锁定当前实际关节位置、切换右臂 Position（状态 1）。
  模式切换时允许小幅收敛，实际关节位置偏离锁定位置不得超过 0.15°；模式、两项百分比、目标回读及静止反馈均确认后进入下一步。
- 首个及后续每个标定点下发前，最多等待 `automatic.mode_timeout_s`（默认 5 秒），
  要求速度 ≤0.2°/s 且低速标志有效，连续保持 `sampling.settle_s`（默认 1 秒）。
  短暂速度尖峰会重新计时；持续不静止则超时停止，不下发下一目标。
  等待期间目标改变、位置偏移超过 0.15°、模式/限速变化或反馈断流仍立即中止。
- 每个目标为 `[0,0,0,0,0,J6,J7]`，不控制左臂。到位误差和关节速度均满足条件才开始静止采样。
- 单点运动超时 60 秒，模式切换超时 5 秒，目标回读超时 1 秒。运行中持续检查 Position 模式和 10%/10% 设置。
- 关节速度 ≤0.2°/s、姿态正确并稳定 1 秒后，连续采样至少 2 秒。
- J6/J7 到位容差 1°；前五轴相对零位容差 0.5°；静止期间关节漂移 ≤0.15°。
- 每个点至少 200 个不同控制器反馈帧。轮询 400 Hz，实际样本率取决于 SDK 反馈。
- 连续窗口内反馈间隔不能超过 0.2 秒；同一帧不重复计数。
- 单轴力标准差 ≤0.10 N，单轴力矩标准差 ≤0.005 N·m。
- 到位后 30 秒内没有合格连续窗口即中止，不自动重试或继续走点。
- 断流/重复帧超过 0.2 秒、模式/百分比变化、目标被改写、关节约束不满足、控制器错误或非有限反馈都会中止。
- 六个原始力传通道同时保持精确不变超过 2 秒也会中止；这是保守筛查，不能替代传感器硬件状态检查。

速度、加速度设置固定为本次要求的 10%，配置若改成其他值会拒绝自动运行；
百分比相对于控制器设定的关节运动上限，不是 10°/s 或 10°/s²。
其余时间和采样阈值可在 `config/right_ft_calibration.yaml` 调整。
Ctrl+C、SIGTERM 或异常发生在控制开始后，会请求右臂停止，再等待静止反馈验证，随后释放连接。
停止请求和是否验证成功写入 `session.json`；断网等情况下无法保证远端收到停止指令，终端会明确提示现场确认。
预检查失败不会发送运动或停止命令。
每点尝试保存完整原始 JSONL（包括移动过程）；合格静止窗口另存一份，`session.json` 在每点后更新。
日志写入失败也会先执行停止处理，不依赖再次写盘成功。
本版不跨连接恢复采集，避免把不同零点/装配的数据拼成一次标定。
完整计划为 12+6 点；至少 10 个有效标定点和 4 个独立验证点才允许拟合。
完成后停留在 V06，不发送回零或 IDLE 指令；释放 SDK 连接后进行离线拟合。
结果写入本次目录的 `result/`，只有验证通过才生成 `calibration.yaml`。
断开连接后的实际保持行为由控制器固件和现场设置决定。

原有手动流程：将命令中的 `auto` 换成 `collect`，由操作者调姿后逐点回车；输入 `q` 结束。
`collect` 只读反馈、配置右臂遥测通道，不发送运动、模式或停止命令，完成后使用独立 `fit`。

## 坐标与符号必须明确

自动完整流程要求提前确认下列配置；`auto --collect-only` 和手动 `collect` 可先保存原始数据再离线拟合。
当前厂家确认 SDK 测量通道与法兰轴同向，本机模板的安装旋转为单位阵，力和力矩符号为 +1，
基座重力为 `[0,-9.81,0]`。此前的非单位轴向假设已被该确认取代，不用于本流程。
单位缩放继续使用原始值除以 10000，分别得到 N 和 N·m。

`config/right_ft_calibration.yaml`：

- `sensor.R_flange_sensor`：3×3 正交旋转矩阵，定义 `v_flange = R_flange_sensor @ v_sensor`。
  传感器坐标系沿 SDK 返回的 Fx/Fy/Fz 三个通道方向定义；力矩通道应使用同一右手坐标约定。
  若实际通道排序不同，需要先明确并修改解码映射，不能靠更改 frame 名称修正。
- `sensor.force_sign`、`sensor.torque_sign`：各为 +1 或 -1，分别把力、力矩统一到
  `F = m*g + bF`、`tau = com × (m*g) + bTau` 的受力符号约定。
  确认实际轴向/符号后设置 `sensor.axes_confirmed: true`，并填写 `frame_id`。
- `gravity.base_m_s2`：默认 null 使用模型中的右臂基座重力 `[0,-9.81,0]` m/s²。
  核对实际基座安装方向后设置 `base_direction_confirmed: true`；必要时改成实际向量。

传感器测量原点的定义也必须明确。输出质心相对于该原点，表达在传感器坐标系，单位 m；
此处不使用天机面向关节动力学的工具质量/质心参数。

## 离线拟合和验证

```bash
cd /home/stw/tianji_ws/src/tianji_io_teleop_bridge
/usr/bin/python3 scripts/right_ft_calibration.py \
  --config config/right_ft_calibration_robot.yaml fit \
  /home/stw/ft_calibration_runs/right_tool_01 \
  --output /home/stw/ft_calibration_runs/right_tool_01_result
```

拟合完全离线，只加载本地运动学库。可以通过主命令前的 `--config /路径/配置.yaml` 使用独立配置副本。
软件重新校验原始数据哈希、通道、单位、静止条件、目标角度、前五轴固定条件和腕部余量；
由每帧实际七轴关节角做 FK，计算采样窗口的平均重力向量。

力模型拟合质量与三个力偏置；力矩模型拟合一阶质量矩 `h=m*com` 与三个力矩偏置，
然后通过 `com=h/m` 求质心。使用归一化重力向量检查条件数，按姿态等权拟合。
重力方向覆盖不足、质量非正、数据无效时明确失败；不会靠填零生成参数。

预留的 V01～V06 只做验证，不参与拟合。初始验收标准：

- 验证姿态的力矢量 RMS ≤0.20 N。
- 验证姿态的力矩矢量 RMS ≤0.01 N·m。

生成 `report.json`、`REPORT.md`，包含逐轴 RMS、最大绝对残差、逐姿态残差、秩/条件数。
只有通过验证才输出 `calibration.yaml`；未通过时保留诊断报告，退出码为 2。
合成测试数据仅输出 `synthetic_calibration.yaml`，不会冒充真实标定结果。

`auto` 在真实数据验证通过后，将结果原子覆盖到配置中的 `current_calibration_file`；
本机模板指向 `/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml`，
与桥接 YAML 在同一目录。可用 `--current-file /固定路径.yaml` 优先指定位置；配置未指定时默认放入程序的配置目录。
各轮 `result/calibration.yaml` 和原始采样继续保留。仅采样、采集失败、验证失败、合成数据均不更新固定文件。
`fit` 默认仅离线复算，不更新固定文件；确认要使用该轮结果时，可显式加
`--current-file /home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml`。

输出包含质量、传感器系质心、六维残余偏置、坐标/符号约定、模型哈希和本次硬件零点记录。
独立标定程序不会将参数写入机器人或自动切换发布节点的配置。在线桥接通过 `right_ft_calibration_file`
加载结果，并在原六维力话题发布重力补偿与软件去皮后的数据，见 [RIGHT_FT_SENSOR.md](RIGHT_FT_SENSOR.md)。
每次机器人开机都重新完成独立标定；固定文件更新成功后，启动桥接时可一直使用
`right_ft_calibration_file:=/home/stw/tianji_ws/src/tianji_io_teleop_bridge/config/right_ft_calibration_current.yaml`。
本轮失败时不能继续使用固定位置残留的旧结果。桥接只在启动时加载文件，运行中不热切换参数。

静态标定无法消除快速运动时的惯性力，也无法自动区分稳定接触力和真实偏置。
控制器帧号变化不能证明传感器自身每帧都更新；没有额外硬件状态位时无法完全检测传感器冻结。

## 离线开发验证

```bash
cd /home/stw/tianji_ws/src/tianji_io_teleop_bridge
PYTHONPATH=scripts /usr/bin/python3 -m pytest -q test/test_ft_calibration.py
```

测试禁止网络 socket 和真实控制库加载；仅允许本地运动学库。
覆盖 J6/J7 四象限约束、边界余量、独立验证、合成参数恢复、非单位安装旋转、正负号、
重复帧、断流、运动/漂移、噪声、数据篡改及模拟 SDK 连接释放。
离线测试包含模拟 SDK 的 18 姿态自动移动→到位→采集→保存→自动拟合流程，
检查右臂 Position 目标及 10%/10% 命令、启动预检查、模式/限速/目标变化、断流、冻结、
运动/采样超时、Ctrl+C、停止确认失败，以及已有数据保存和连接释放。
这证明离线逻辑通过测试，不代表姿态已通过整机碰撞验收或真实传感器精度验收。
