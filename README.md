# Tianji IO Teleop Bridge

ROS 2 Humble bridge between IO TeleXperience and the Tianji Marvin M6 CCS dual-arm robot.

## Version 0.2.0 safety boundary

The code contains a safety-gated joint-impedance control path, but the installed default
configuration is still monitor-only. In `config/bridge.yaml` all of these gates default to a
non-motion value:

```yaml
allow_motion: false
sdk_control_compatibility_confirmed: false
tool_dynamics_confirmed: false
enabled_arms: "none"
```

With `allow_motion=false`, the node cannot call Marvin mode switching, impedance parameter,
idle, or joint target APIs. The enable service refuses the request before any SDK control call.
Starting the node only connects for feedback, publishes `/io_teleop/joint_states`, validates
arm commands, and publishes status.

Do not remove the locks until Tianji confirms controller/SDK control compatibility and the
tool dynamics configuration. A robot fault, including emergency-stop error 13, is never
cleared automatically.

## ROS interfaces

- Publishes `/io_teleop/joint_states` (`sensor_msgs/msg/JointState`) at 100 Hz.
- Publishes `~/estimated_joint_disturbance` (`sensor_msgs/msg/JointState`) at the same rate.
  Its `name` is the canonical 14-joint order and its `effort` is Marvin
  `m_EST_Joint_Force`; `position` and `velocity` are empty.
- Publishes `~/left/estimated_cartesian_disturbance` and
  `~/right/estimated_cartesian_disturbance` (`geometry_msgs/msg/WrenchStamped`) at the
  same rate. Force xyz maps from `m_EST_Cart_FN[0..2]` and torque xyz from `[3..5]`.
- Subscribes `/io_teleop/joint_cmd` (`sensor_msgs/msg/JointState`) with best-effort, depth-1
  latest-sample QoS.
- Publishes `~/control_status` (`std_msgs/msg/String`) once per second.
- Publishes `~/alignment_status` (`diagnostic_msgs/msg/DiagnosticArray`) at 10 Hz with one
  summary and one entry for every arm joint.
- Provides `~/enable_control` (`std_srvs/srv/SetBool`). The safe default always rejects enable.

The shared command topic also carries IO left/right finger messages. Messages containing no
Tianji arm joint name are counted as `ignored_non_arm` and silently ignored. A production arm
command must contain exactly all 14 Tianji joints; a malformed message containing any Tianji
arm name is rejected and reported.

Before control becomes active, every structurally valid, in-hard-limit 14-joint command is kept
as an alignment candidate even when it is far from feedback. The candidate is never sent to the
robot until the single alignment gate passes. This avoids the previous hidden startup rejection
threshold. Once control is `ACTIVE` or `SOFT_LIMIT_HOLD`, the configured per-message jump check
is enforced again.

## Joint mapping and units

Canonical ROS order:

```text
Joint1_L ... Joint7_L, Joint1_R ... Joint7_R
```

Marvin arm A maps to the left arm and arm B maps to the right arm. SDK position and velocity
use degrees and degrees/second; ROS uses radians and radians/second. Incoming commands are
mapped by name rather than relying on input order.

The disturbance topics are copied read-only from the same validated `DCSS` feedback frame and
share its ROS timestamp. They add no SDK call and do not participate in command validation or
the control state machine. A non-finite disturbance field suppresses only its own message and
does not suppress joint-state feedback or stop control. The SDK documentation available here
does not define the `m_EST_Cart_FN` reference frame, so both `WrenchStamped.header.frame_id`
values intentionally default to empty. Set `left_cartesian_disturbance_frame_id` and
`right_cartesian_disturbance_frame_id` only after Tianji confirms the frames, axes, signs, and
units.

Inspect the raw estimates with:

```bash
ros2 topic echo /tianji_marvin_teleop_bridge/estimated_joint_disturbance --once
ros2 topic echo /tianji_marvin_teleop_bridge/left/estimated_cartesian_disturbance --once
ros2 topic echo /tianji_marvin_teleop_bridge/right/estimated_cartesian_disturbance --once
```

URDF limits in `joint_model.hpp` were copied from the IO-supplied
`Marvin_M6-S-CCS-696-V4.0_Base_and_Stand_Asm.urdf`.

## Prepared joint-impedance path

After all external confirmations and explicit operator authorization, the state machine is
designed to progress through:

```text
MONITOR -> PREPARING_PARAMETERS -> WAITING_FOR_ALIGNMENT
                                      -> CONFIGURING_IMPEDANCE -> ACTIVE
                                                                <-> SOFT_LIMIT_HOLD
                                                                -> HOLD -> MONITOR
                                                                -> FAULT
```

An explicit enable request first writes and verifies K/D plus motion limits without changing
the robot's current control mode. It then requires 20 consecutive aligned arm commands by
default. Initial target error must remain within 0.05 rad for every joint. Only after alignment
does it enter Marvin torque state 3 with impedance type 1 while
holding measured position, verifies the controller's echoed mode and K/D values, and only then
allows target following. Entering the configurable joint-limit safety margin no longer exits
impedance control: it freezes the configured arms at measured position in `SOFT_LIMIT_HOLD`.
The default freezes both enabled arms to preserve dual-arm coordination. Control resumes
automatically only after every enabled target has the larger release clearance, the frozen
arms are realigned, and the condition remains stable for the configured number of frames.
A valid-arm-command timeout, feedback timeout, robot error, following error, hard-limit command
rejection, or SDK failure still enters the fail-stop path. Reconnection never re-arms.
The impedance parameter packet and mode-switch packet are separated by a non-blocking 250 ms
settle interval because Marvin's SDK rejects `OnClearSet()` while its previous packet is busy.
If an idle request encounters the same busy condition, it is retried after the settle interval.

The default soft-limit hysteresis is:

```yaml
joint_limit_safety_margin_rad: 0.01
soft_limit_release_margin_rad: 0.03
soft_limit_recovery_alignment_rad: 0.10
soft_limit_recovery_frames: 20
soft_limit_allow_other_arm_motion: false
```

`control_status` reports `state=SOFT_LIMIT_HOLD`, the frozen arms, recovery-frame count, and
the triggering joint with its target and safe interval. Setting
`soft_limit_allow_other_arm_motion=true` freezes only the arm that reached a soft limit; this
is not the default because independent motion can break coordinated dual-arm tasks.

## Operator alignment dashboard

Run the bridge in one terminal and the dashboard in another:

```bash
source /opt/ros/humble/setup.bash
source /home/stw/tianji_ws/install/setup.bash
ros2 run tianji_io_teleop_bridge alignment_dashboard
```

The color terminal display shows all 14 joints with command angle, robot feedback angle,
absolute error, and an error/tolerance bar:

- green `ALIGNED`: enabled joint is within the configured alignment tolerance;
- red `ADJUST`: move the operator-side joint toward the robot pose;
- yellow `LIMIT`: command is inside the URDF joint-limit safety margin;
- gray `OFF`: that arm is not enabled.

The header reports the largest-error joint and the consecutive alignment-frame counter. The
corresponding structured topic can also be recorded in a rosbag:

```bash
ros2 bag record /tianji_marvin_teleop_bridge/alignment_status
```

The low-stiffness starting candidates are deliberately softer than the Marvin SDK demo:

```yaml
left_joint_stiffness:  [2.5, 2.5, 2.5, 2.0, 1.5, 1.2, 1.0]
left_joint_damping:    [0.35, 0.35, 0.35, 0.30, 0.25, 0.25, 0.25]
right_joint_stiffness: [2.5, 2.5, 2.5, 2.0, 1.5, 1.2, 1.0]
right_joint_damping:   [0.35, 0.35, 0.35, 0.30, 0.25, 0.25, 0.25]
```

K is in N*m/degree and D is a dimensionless damping ratio. These are candidates for controlled
commissioning, not validated production gains. They are never applied with the default locks.
The software additionally limits each target to 0.25 rad/s and 0.50 rad/s^2 by default.

## SDK dependency

The build defaults to the validated local SDK directory:

```text
/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++
```

CMake requires SDK version `100343013` and checks SHA-256
`47ab2ab6e35cf899eae3978dfb50ee219e05b10f72aedd6bc9ba6a338a8b4011`.
The shared library is installed into the ROS workspace install prefix.

## Build and test

```bash
cd /home/stw/tianji_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tianji_io_teleop_bridge --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon test --packages-select tianji_io_teleop_bridge
colcon test-result --verbose
```

## Safe runs

SDK/ROS smoke test without opening the exclusive robot connection:

```bash
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  connect_on_start:=false allow_motion:=false
```

Feedback and command-monitoring run, still incapable of control:

```bash
ros2 launch tianji_io_teleop_bridge marvin_teleop_bridge.launch.py \
  allow_motion:=false
ros2 topic echo /tianji_marvin_teleop_bridge/control_status
```

Do not run MarvinPlatform or another Marvin SDK client at the same time; the connection is
exclusive. No motion-enabling procedure is documented until the remaining Tianji confirmations
and staged commissioning checklist are complete.
