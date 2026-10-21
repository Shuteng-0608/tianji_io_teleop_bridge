#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "MarvinSDK.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tianji_io_teleop_bridge/control_gate.hpp"
#include "tianji_io_teleop_bridge/disturbance_messages.hpp"
#include "tianji_io_teleop_bridge/joint_model.hpp"
#include "tianji_io_teleop_bridge/joint_torque_zero.hpp"
#include "tianji_io_teleop_bridge/ft_compensation.hpp"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"
#include "tianji_io_teleop_bridge/marvin_sdk_adapter.hpp"
#include "tianji_io_teleop_bridge/send_timing.hpp"
#include "tianji_io_teleop_bridge/retreat_check.hpp"

namespace tianji_io_teleop_bridge
{

enum class ImpedanceConfigurationPhase
{
  kNone,
  kWaitingForParameterEcho,
  kWaitingForModeEcho,
  kWaitingForPositionLimitEcho
};

enum class ControlPurpose
{
  kNone,
  kTeleoperation,
  kInitialPose,
  kRetreatRight,
  kRetreatLeft
};

inline const char * control_purpose_name(ControlPurpose purpose)
{
  switch (purpose) {
    case ControlPurpose::kNone:
      return "NONE";
    case ControlPurpose::kTeleoperation:
      return "TELEOPERATION";
    case ControlPurpose::kInitialPose:
      return "INITIAL_POSE";
    case ControlPurpose::kRetreatRight:
      return "RETREAT_RIGHT_10CM";
    case ControlPurpose::kRetreatLeft:
      return "RETREAT_LEFT_10CM";
  }
  return "UNKNOWN";
}

inline const char * impedance_phase_name(ImpedanceConfigurationPhase phase)
{
  switch (phase) {
    case ImpedanceConfigurationPhase::kNone:
      return "NONE";
    case ImpedanceConfigurationPhase::kWaitingForParameterEcho:
      return "WAITING_FOR_PARAMETER_ECHO";
    case ImpedanceConfigurationPhase::kWaitingForModeEcho:
      return "WAITING_FOR_MODE_ECHO";
    case ImpedanceConfigurationPhase::kWaitingForPositionLimitEcho:
      return "WAITING_FOR_POSITION_LIMIT_ECHO";
  }
  return "UNKNOWN";
}

class MarvinTeleopBridgeNode : public rclcpp::Node
{
  friend class TargetSendTest;  // Offline fault-injection tests; SDK is replaced with stubs.
public:
  explicit MarvinTeleopBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tianji_marvin_teleop_bridge", options), control_gate_(20)
  {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.1.190");
    state_topic_ = declare_parameter<std::string>(
      "state_topic", "/io_teleop/joint_states");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/io_teleop/joint_cmd");
    left_cartesian_disturbance_topic_ = declare_parameter<std::string>(
      "left_cartesian_disturbance_topic", "~/left/estimated_cartesian_disturbance");
    right_cartesian_disturbance_topic_ = declare_parameter<std::string>(
      "right_cartesian_disturbance_topic", "~/right/estimated_cartesian_disturbance");
    left_cartesian_disturbance_frame_id_ = declare_parameter<std::string>(
      "left_cartesian_disturbance_frame_id", "");
    right_cartesian_disturbance_frame_id_ = declare_parameter<std::string>(
      "right_cartesian_disturbance_frame_id", "");
    kinematics_config_file_ = declare_parameter<std::string>(
      "kinematics_config_file", "");
    publish_cartesian_disturbance_ = declare_parameter<bool>(
      "publish_cartesian_disturbance", false);
    alignment_diagnostics_topic_ = declare_parameter<std::string>(
      "alignment_diagnostics_topic", "~/alignment_status");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    alignment_diagnostics_rate_hz_ = declare_parameter<double>(
      "alignment_diagnostics_rate_hz", 10.0);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 100.0);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.2);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.15);
    command_recovery_alignment_rad_ = declare_parameter<double>("command_recovery_alignment_rad", 0.1);
    command_reference_timeout_sec_ = declare_parameter<double>(
      "command_reference_timeout_sec", 0.2);
    max_command_step_rad_ = declare_parameter<double>("max_command_step_rad", 0.05);
    initial_alignment_tolerance_rad_ = declare_parameter<double>(
      "initial_alignment_tolerance_rad", 0.05);
    joint_limit_tolerance_rad_ = declare_parameter<double>(
      "joint_limit_tolerance_rad", 1.0e-4);
    joint_limit_safety_margin_rad_ = declare_parameter<double>(
      "joint_limit_safety_margin_rad", 0.01);
    soft_limit_release_margin_rad_ = declare_parameter<double>(
      "soft_limit_release_margin_rad", 0.03);
    soft_limit_recovery_alignment_rad_ = declare_parameter<double>(
      "soft_limit_recovery_alignment_rad", 0.10);
    max_following_error_rad_ = declare_parameter<double>("max_following_error_rad", 0.25);
    retreat_duration_sec_ = declare_parameter<double>("retreat_duration_sec", 10.0);
    rcl_interfaces::msg::ParameterDescriptor tool_parameter_descriptor;
    tool_parameter_descriptor.read_only = true;
    publish_right_ft_sensor_ = declare_parameter<bool>(
      "publish_right_ft_sensor", false, tool_parameter_descriptor);
    right_ft_sensor_topic_ = declare_parameter<std::string>(
      "right_ft_sensor_topic", "/io_teleop/right/ft_sensor", tool_parameter_descriptor);
    right_ft_sensor_frame_id_ = declare_parameter<std::string>(
      "right_ft_sensor_frame_id", "", tool_parameter_descriptor);
    right_ft_calibration_file_ = declare_parameter<std::string>(
      "right_ft_calibration_file", "", tool_parameter_descriptor);
    right_ft_tare_directory_ = declare_parameter<std::string>(
      "right_ft_tare_directory", "", tool_parameter_descriptor);
    right_ft_compensation_.window_sec = declare_parameter<double>(
      "right_ft_tare_window_sec", 5.0, tool_parameter_descriptor);
    right_ft_compensation_.max_gap_sec = feedback_timeout_sec_;
    validate_positive_finite(right_ft_compensation_.window_sec, "right_ft_tare_window_sec");
    teleop_soft_limits_enabled_ = declare_parameter<bool>(
      "teleop_soft_limits_enabled", false, tool_parameter_descriptor);
    teleop_joint_limit_extension_rad_ = declare_parameter<double>(
      "teleop_joint_limit_extension_rad", 0.02, tool_parameter_descriptor);
    teleop_control_mode_ = parse_joint_control_mode(declare_parameter<std::string>(
      "teleop_control_mode", "joint_impedance", tool_parameter_descriptor));
    tools_config_file_ = declare_parameter<std::string>("tools_config_file", "", tool_parameter_descriptor);
    tool_config_dry_run_ = declare_parameter<bool>("tool_config_dry_run", true, tool_parameter_descriptor);
    tool_config_timeout_sec_ = declare_parameter<double>("tool_config_timeout_sec", 2.0, tool_parameter_descriptor);
    raw_state_topic_ = declare_parameter<std::string>(
      "raw_state_topic", state_topic_ + "_raw", tool_parameter_descriptor);
    estimated_joint_disturbance_topic_ = declare_parameter<std::string>(
      "estimated_joint_disturbance_topic", "~/estimated_joint_disturbance",
      tool_parameter_descriptor);
    auto & zero = joint_torque_zero_.settings;
    zero.window_sec = declare_parameter<double>("joint_zero_window_sec", 1.0, tool_parameter_descriptor);
    zero.max_std_nm = declare_parameter<double>("joint_zero_max_std_nm", 0.05, tool_parameter_descriptor);
    zero.max_velocity_rad_s = declare_parameter<double>(
      "joint_zero_max_velocity_rad_s", 0.01, tool_parameter_descriptor);
    zero.max_position_range_rad = declare_parameter<double>(
      "joint_zero_max_position_range_rad", 0.005, tool_parameter_descriptor);
    zero.max_gap_sec = feedback_timeout_sec_;
    validate_positive_finite(zero.window_sec, "joint_zero_window_sec");
    validate_positive_finite(zero.max_std_nm, "joint_zero_max_std_nm");
    validate_positive_finite(zero.max_velocity_rad_s, "joint_zero_max_velocity_rad_s");
    validate_positive_finite(zero.max_position_range_rad, "joint_zero_max_position_range_rad");
    mode_transition_timeout_sec_ = declare_parameter<double>(
      "mode_transition_timeout_sec", 2.0);
    sdk_packet_settle_sec_ = declare_parameter<double>("sdk_packet_settle_sec", 0.25);
    sdk_target_busy_timeout_sec_ = declare_parameter<double>("sdk_target_busy_timeout_sec", 0.1);
    hold_duration_sec_ = declare_parameter<double>("hold_duration_sec", 0.5);
    initial_pose_tolerance_rad_ = declare_parameter<double>(
      "initial_pose_tolerance_rad", 0.02);
    initial_pose_timeout_sec_ = declare_parameter<double>("initial_pose_timeout_sec", 45.0);
    initial_pose_velocity_ratio_ = declare_parameter<int>(
      "initial_pose_velocity_ratio", 10, tool_parameter_descriptor);
    initial_pose_acceleration_ratio_ = declare_parameter<int>(
      "initial_pose_acceleration_ratio", 10, tool_parameter_descriptor);
    reconnect_period_sec_ = declare_parameter<double>("reconnect_period_sec", 2.0);
    alignment_required_frames_ = declare_parameter<int>("alignment_required_frames", 20);
    soft_limit_recovery_frames_ = declare_parameter<int>("soft_limit_recovery_frames", 20);
    initial_pose_required_frames_ = declare_parameter<int>("initial_pose_required_frames", 20);
    expected_sdk_version_ = declare_parameter<int64_t>("expected_sdk_version", 100343014);
    connect_on_start_ = declare_parameter<bool>("connect_on_start", true);
    subscribe_joint_commands_ = declare_parameter<bool>("subscribe_joint_commands", true);
    strict_sdk_version_ = declare_parameter<bool>("strict_sdk_version", true);
    soft_limit_allow_other_arm_motion_ = declare_parameter<bool>(
      "soft_limit_allow_other_arm_motion", false);

    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    sdk_control_compatibility_confirmed_ = declare_parameter<bool>(
      "sdk_control_compatibility_confirmed", false);
    tool_dynamics_confirmed_ = declare_parameter<bool>("tool_dynamics_confirmed", false);
    enabled_arms_text_ = declare_parameter<std::string>("enabled_arms", "none");

    impedance_settings_.velocity_ratio = declare_parameter<int>("velocity_ratio", 10);
    impedance_settings_.acceleration_ratio = declare_parameter<int>("acceleration_ratio", 10);
    left_stiffness_ = declare_parameter<std::vector<double>>(
      "left_joint_stiffness", {2.5, 2.5, 2.5, 2.0, 1.5, 1.2, 1.0});
    left_damping_ = declare_parameter<std::vector<double>>(
      "left_joint_damping", {0.35, 0.35, 0.35, 0.30, 0.25, 0.25, 0.25});
    right_stiffness_ = declare_parameter<std::vector<double>>(
      "right_joint_stiffness", {2.5, 2.5, 2.5, 2.0, 1.5, 1.2, 1.0});
    right_damping_ = declare_parameter<std::vector<double>>(
      "right_joint_damping", {0.35, 0.35, 0.35, 0.30, 0.25, 0.25, 0.25});
    initial_pose_deg_ = declare_parameter<std::vector<double>>(
      "initial_pose_deg",
      {90.0, -75.0, -90.0, -90.0, 0.0, 0.0, 0.0,
        -90.0, -75.0, 90.0, -90.0, 0.0, 0.0, 0.0});

    state_qos_reliability_ = declare_parameter<std::string>(
      "state_qos_reliability", "reliable");
    command_qos_reliability_ = declare_parameter<std::string>(
      "command_qos_reliability", "best_effort");
    state_qos_depth_ = declare_parameter<int>("state_qos_depth", 10);
    command_qos_depth_ = declare_parameter<int>("command_qos_depth", 1);

    validate_parameters();
    if (publish_right_ft_sensor_ && right_ft_calibration_file_.empty()) {
      throw std::invalid_argument(
              "right_ft_calibration_file is required: specify calibration.yaml from this "
              "robot startup's calibration run; previous runs are never selected automatically");
    }
    validate_sdk();
    if (publish_cartesian_disturbance_ || publish_right_ft_sensor_) {
      initialize_kinematics();
    }
    if (publish_right_ft_sensor_) {
      const auto share = ament_index_cpp::get_package_share_directory("tianji_io_teleop_bridge");
      right_ft_compensation_.load(right_ft_calibration_file_, kinematics_config_file_,
        share + "/marvin_kinematics/ccs_m6_40.MvKDCfg");
      if (right_ft_sensor_frame_id_.empty()) {
        right_ft_sensor_frame_id_ = right_ft_compensation_.frame_id();
      } else if (right_ft_sensor_frame_id_ != right_ft_compensation_.frame_id()) {
        throw std::invalid_argument("right_ft_sensor_frame_id differs from calibration frame");
      }
      if (right_ft_tare_directory_.empty()) {
        const char * user_home = std::getenv("HOME");
        if (!user_home) {throw std::invalid_argument("Set right_ft_tare_directory");}
        right_ft_tare_directory_ = std::string(user_home) + "/.ros/tianji_ft_tares";
      }
    }
    enabled_arms_ = parse_enabled_arms(enabled_arms_text_);
    copy_control_arrays();
    control_gate_ = ControlGate(
      static_cast<std::size_t>(alignment_required_frames_),
      static_cast<std::size_t>(soft_limit_recovery_frames_));

    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      state_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
    estimated_joint_disturbance_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      estimated_joint_disturbance_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
    raw_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      raw_state_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
    if (std::string(raw_state_publisher_->get_topic_name()) == state_publisher_->get_topic_name() ||
      std::string(estimated_joint_disturbance_publisher_->get_topic_name()) ==
      state_publisher_->get_topic_name() ||
      std::string(estimated_joint_disturbance_publisher_->get_topic_name()) ==
      raw_state_publisher_->get_topic_name())
    {
      throw std::invalid_argument("state, estimated disturbance and raw disturbance topics must differ");
    }
    torque_bias_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_torque_bias", make_qos(state_qos_reliability_, state_qos_depth_));
    if (publish_right_ft_sensor_) {
      right_ft_sensor_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        right_ft_sensor_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
      const std::string topic = right_ft_sensor_publisher_->get_topic_name();
      if (topic == state_publisher_->get_topic_name() ||
        topic == estimated_joint_disturbance_publisher_->get_topic_name() ||
        topic == raw_state_publisher_->get_topic_name() ||
        topic == torque_bias_publisher_->get_topic_name() ||
        topic == get_node_topics_interface()->resolve_topic_name(command_topic_))
      {
        throw std::invalid_argument("right FT sensor topic must be separate from joint topics");
      }
      right_ft_zero_service_ = create_service<std_srvs::srv::Trigger>("~/right/zero_ft_sensor",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {on_zero_ft_sensor(response);});
      RCLCPP_INFO(get_logger(), "Right FT compensated recording on %s; waiting for manual software tare; effort=[Fx,Fy,Fz,Tx,Ty,Tz], N/Nm",
        topic.c_str());
    }
    torque_zero_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/joint_torque_zero_status", rclcpp::QoS(1).reliable().transient_local());
    const std::array<std::string, 3> zero_sides{"left", "right", "both"};
    for (std::size_t side = 0; side < zero_sides.size(); ++side) {
      const std::array<bool, 2> selected{side != 1, side != 0};
      for (std::size_t clear = 0; clear < 2; ++clear) {
        torque_zero_services_.push_back(create_service<std_srvs::srv::Trigger>(
          "~/" + zero_sides[side] + (clear ? "/clear_joint_torque_zero" : "/zero_joint_torque"),
          [this, selected, clear](const std_srvs::srv::Trigger::Request::SharedPtr,
          std_srvs::srv::Trigger::Response::SharedPtr response) {
            on_zero_joint_torque(selected, clear != 0, response);
          }));
      }
    }
    publish_torque_zero_status();
    if (publish_cartesian_disturbance_) {
      left_cartesian_disturbance_publisher_ =
        create_publisher<geometry_msgs::msg::WrenchStamped>(
        left_cartesian_disturbance_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
      right_cartesian_disturbance_publisher_ =
        create_publisher<geometry_msgs::msg::WrenchStamped>(
        right_cartesian_disturbance_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
    }
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/control_status", rclcpp::QoS(1).reliable().durability_volatile());
    tool_status_publisher_ = create_publisher<std_msgs::msg::String>(
      "~/tool_configuration_status", rclcpp::QoS(1).reliable().transient_local());
    for (std::size_t arm = 0; arm < 2; ++arm) {
      tool_services_[arm] = create_service<std_srvs::srv::Trigger>(
        arm == 0 ? "~/configure_left_tool" : "~/configure_right_tool",
        [this, arm](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {on_configure_tool(arm, response);});
    }
    publish_tool_status("NOT_UPLOADED: no automatic tool configuration; dry_run=" +
      std::string(tool_config_dry_run_ ? "true" : "false"));
    alignment_diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      alignment_diagnostics_topic_, rclcpp::QoS(1).reliable().durability_volatile());

    if (subscribe_joint_commands_) {
      command_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        command_topic_, make_qos(command_qos_reliability_, command_qos_depth_),
        std::bind(&MarvinTeleopBridgeNode::on_joint_command, this, std::placeholders::_1));
    }
    control_service_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_control",
      std::bind(
        &MarvinTeleopBridgeNode::on_enable_control, this,
        std::placeholders::_1, std::placeholders::_2));
    initial_pose_service_ = create_service<std_srvs::srv::Trigger>(
      "~/move_to_initial_pose",
      std::bind(
        &MarvinTeleopBridgeNode::on_move_to_initial_pose, this,
        std::placeholders::_1, std::placeholders::_2));
    retreat_service_ = create_service<std_srvs::srv::Trigger>("~/retreat_right_10cm",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {on_retreat(1, response);});
    retreat_left_service_ = create_service<std_srvs::srv::Trigger>("~/retreat_left_10cm",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {on_retreat(0, response);});

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(timer_period, std::bind(&MarvinTeleopBridgeNode::on_timer, this));
    next_connect_attempt_ = std::chrono::steady_clock::now();
    last_status_publish_ = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    last_alignment_diagnostics_publish_ =
      std::chrono::steady_clock::now() - std::chrono::seconds(2);

    if (!allow_motion_) {
      RCLCPP_WARN(
        get_logger(),
        "Motion lock is ON (allow_motion=false): no mode, impedance, idle, or joint command "
        "will be sent to Marvin");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Motion capability was explicitly allowed, but enable_control service and all safety "
        "gates are still required");
    }
    RCLCPP_INFO(
      get_logger(),
      "Publishing %s at %.1f Hz; control update ceiling %.1f Hz (completion-paced); monitoring %s with "
      "latest-sample QoS",
      state_topic_.c_str(), publish_rate_hz_, control_rate_hz_, command_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Teleoperation mode=%s (startup-only); initial pose and retreat use joint_impedance",
      joint_control_mode_name(teleop_control_mode_));
    RCLCPP_INFO(
      get_logger(), "Teleoperation joint limits=%s; URDF extension=%.4f rad per boundary "
      "(plus %.4f rad numerical tolerance)",
      teleop_soft_limits_enabled_ ? "inset soft limits" : "backup guard only",
      teleop_soft_limits_enabled_ ? 0.0 : teleop_joint_limit_extension_rad_,
      joint_limit_tolerance_rad_);
    RCLCPP_INFO(
      get_logger(), "Publishing Marvin feedback joint torque in %s effort and bias-corrected "
      "estimated joint disturbance in %s effort",
      state_topic_.c_str(), estimated_joint_disturbance_publisher_->get_topic_name());
    if (publish_cartesian_disturbance_) {
      RCLCPP_INFO(
        get_logger(), "Cartesian disturbance publishing enabled on %s and %s",
        left_cartesian_disturbance_topic_.c_str(),
        right_cartesian_disturbance_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Cartesian disturbance publishing disabled (publish_cartesian_disturbance=false)");
    }
  }

  ~MarvinTeleopBridgeNode() override
  {
    disconnect_robot();
  }

private:
  static void validate_positive_finite(double value, const std::string & name)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
  }

  static void validate_vector(
    const std::vector<double> & values, std::size_t expected_size,
    double lower, double upper, const std::string & name)
  {
    if (values.size() != expected_size) {
      throw std::invalid_argument(
              name + " must contain exactly " + std::to_string(expected_size) +
              " values");
    }
    for (double value : values) {
      if (!std::isfinite(value) || value < lower || value > upper) {
        throw std::invalid_argument(name + " contains an out-of-range or non-finite value");
      }
    }
  }

  void validate_parameters()
  {
    validate_positive_finite(publish_rate_hz_, "publish_rate_hz");
    if (publish_rate_hz_ > 1000.0) {
      throw std::invalid_argument("publish_rate_hz must not exceed 1000");
    }
    validate_positive_finite(control_rate_hz_, "control_rate_hz");
    if (control_rate_hz_ > 200.0) {
      throw std::invalid_argument("control_rate_hz must not exceed 200");
    }
    if (control_rate_hz_ > publish_rate_hz_) {
      throw std::invalid_argument("control_rate_hz must not exceed publish_rate_hz");
    }
    validate_positive_finite(
      alignment_diagnostics_rate_hz_, "alignment_diagnostics_rate_hz");
    if (alignment_diagnostics_rate_hz_ > publish_rate_hz_) {
      throw std::invalid_argument(
              "alignment_diagnostics_rate_hz must not exceed publish_rate_hz");
    }
    validate_positive_finite(feedback_timeout_sec_, "feedback_timeout_sec");
    validate_positive_finite(command_timeout_sec_, "command_timeout_sec");
    validate_positive_finite(command_recovery_alignment_rad_, "command_recovery_alignment_rad");
    if (command_recovery_alignment_rad_ > max_following_error_rad_) {
      throw std::invalid_argument("command_recovery_alignment_rad must not exceed following-error threshold");
    }
    validate_positive_finite(command_reference_timeout_sec_, "command_reference_timeout_sec");
    validate_positive_finite(max_command_step_rad_, "max_command_step_rad");
    validate_positive_finite(
      initial_alignment_tolerance_rad_, "initial_alignment_tolerance_rad");
    validate_positive_finite(
      soft_limit_release_margin_rad_, "soft_limit_release_margin_rad");
    validate_positive_finite(
      soft_limit_recovery_alignment_rad_, "soft_limit_recovery_alignment_rad");
    validate_positive_finite(max_following_error_rad_, "max_following_error_rad");
    validate_positive_finite(retreat_duration_sec_, "retreat_duration_sec");
    validate_positive_finite(tool_config_timeout_sec_, "tool_config_timeout_sec");
    if (tool_config_timeout_sec_ <= sdk_packet_settle_sec_) {
      throw std::invalid_argument("tool_config_timeout_sec must exceed sdk_packet_settle_sec");
    }
    validate_positive_finite(mode_transition_timeout_sec_, "mode_transition_timeout_sec");
    validate_positive_finite(sdk_packet_settle_sec_, "sdk_packet_settle_sec");
    validate_positive_finite(sdk_target_busy_timeout_sec_, "sdk_target_busy_timeout_sec");
    if (sdk_target_busy_timeout_sec_ > command_timeout_sec_ ||
      sdk_target_busy_timeout_sec_ > feedback_timeout_sec_)
    {
      throw std::invalid_argument(
              "sdk_target_busy_timeout_sec must not exceed command/feedback timeout");
    }
    validate_positive_finite(hold_duration_sec_, "hold_duration_sec");
    validate_positive_finite(initial_pose_tolerance_rad_, "initial_pose_tolerance_rad");
    validate_positive_finite(initial_pose_timeout_sec_, "initial_pose_timeout_sec");
    validate_positive_finite(reconnect_period_sec_, "reconnect_period_sec");
    if (!std::isfinite(joint_limit_tolerance_rad_) || joint_limit_tolerance_rad_ < 0.0 ||
      !std::isfinite(joint_limit_safety_margin_rad_) || joint_limit_safety_margin_rad_ < 0.0)
    {
      throw std::invalid_argument("joint limit tolerance and margin must be finite and non-negative");
    }
    if (!std::isfinite(teleop_joint_limit_extension_rad_) || teleop_joint_limit_extension_rad_ < 0.0) {
      throw std::invalid_argument("teleop_joint_limit_extension_rad must be finite and non-negative");
    }
    if (soft_limit_release_margin_rad_ <= joint_limit_safety_margin_rad_) {
      throw std::invalid_argument(
              "soft_limit_release_margin_rad must be greater than "
              "joint_limit_safety_margin_rad");
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (2.0 * soft_limit_release_margin_rad_ >=
        upper_limits_rad()[index] - lower_limits_rad()[index])
      {
        throw std::invalid_argument(
                "soft_limit_release_margin_rad leaves no valid range for " +
                joint_names()[index]);
      }
    }
    if (alignment_required_frames_ <= 0) {
      throw std::invalid_argument("alignment_required_frames must be positive");
    }
    if (soft_limit_recovery_frames_ <= 0) {
      throw std::invalid_argument("soft_limit_recovery_frames must be positive");
    }
    if (initial_pose_required_frames_ <= 0) {
      throw std::invalid_argument("initial_pose_required_frames must be positive");
    }
    if (impedance_settings_.velocity_ratio < 1 || impedance_settings_.velocity_ratio > 100 ||
      impedance_settings_.acceleration_ratio < 1 ||
      impedance_settings_.acceleration_ratio > 100)
    {
      throw std::invalid_argument("velocity_ratio and acceleration_ratio must be in [1, 100]");
    }
    if (initial_pose_velocity_ratio_ < 1 || initial_pose_velocity_ratio_ > 100 ||
      initial_pose_acceleration_ratio_ < 1 || initial_pose_acceleration_ratio_ > 100)
    {
      throw std::invalid_argument(
              "initial_pose_velocity_ratio and initial_pose_acceleration_ratio must be in [1, 100]");
    }
    if (state_qos_depth_ <= 0 || command_qos_depth_ <= 0) {
      throw std::invalid_argument("QoS depths must be positive");
    }
    if (state_topic_.empty() || command_topic_.empty() || alignment_diagnostics_topic_.empty())
    {
      throw std::invalid_argument("topic names must not be empty");
    }
    if (publish_cartesian_disturbance_ &&
      (left_cartesian_disturbance_topic_.empty() || right_cartesian_disturbance_topic_.empty()))
    {
      throw std::invalid_argument(
              "Cartesian disturbance topic names must not be empty when publishing is enabled");
    }
    (void)parse_ipv4(robot_ip_);
    (void)parse_enabled_arms(enabled_arms_text_);
    validate_vector(left_stiffness_, kArmDof, 0.0, 22.0, "left_joint_stiffness");
    validate_vector(right_stiffness_, kArmDof, 0.0, 22.0, "right_joint_stiffness");
    validate_vector(left_damping_, kArmDof, 0.0, 1.0, "left_joint_damping");
    validate_vector(right_damping_, kArmDof, 0.0, 1.0, "right_joint_damping");
    validate_vector(initial_pose_deg_, kJointCount, -360.0, 360.0, "initial_pose_deg");
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double position_rad = degrees_to_radians(initial_pose_deg_[index]);
      if (position_rad < lower_limits_rad()[index] + joint_limit_safety_margin_rad_ ||
        position_rad > upper_limits_rad()[index] - joint_limit_safety_margin_rad_)
      {
        throw std::invalid_argument(
                "initial_pose_deg is outside the configured safe range for " +
                joint_names()[index]);
      }
    }
  }

  void validate_sdk()
  {
    int byte_order = -1;
    const int compatibility = CheckSDKTypeCompat(&byte_order);
    if (compatibility < 0) {
      throw std::runtime_error(
              "Marvin SDK ABI compatibility check failed: " + std::to_string(compatibility));
    }
    const long sdk_version = OnGetSDKVersion();
    if (sdk_version != expected_sdk_version_) {
      const std::string message = "Marvin SDK version is " + std::to_string(sdk_version) +
        ", expected " + std::to_string(expected_sdk_version_);
      if (strict_sdk_version_) {
        throw std::runtime_error(message);
      }
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "Marvin SDK ABI check passed (byte_order=%d), version=%ld",
      byte_order, sdk_version);
  }

  void initialize_kinematics()
  {
    if (kinematics_config_file_.empty()) {
      kinematics_config_file_ =
        ament_index_cpp::get_package_share_directory("tianji_io_teleop_bridge") +
        "/marvin_kinematics/ccs_m6_40.MvKDCfg";
    }
    if (!kinematics_.initialize(kinematics_config_file_)) {
      throw std::runtime_error(
              "Marvin kinematics initialization failed: " + kinematics_.last_error());
    }
    RCLCPP_INFO(
      get_logger(), "Marvin dual-arm kinematics initialized from %s",
      kinematics_config_file_.c_str());
  }

  static std::array<unsigned char, 4> parse_ipv4(const std::string & ip)
  {
    if (std::count(ip.begin(), ip.end(), '.') != 3) {
      throw std::invalid_argument("robot_ip is not a valid IPv4 address: " + ip);
    }
    std::array<unsigned char, 4> octets{};
    std::stringstream stream(ip);
    std::string token;
    for (std::size_t index = 0; index < octets.size(); ++index) {
      if (!std::getline(stream, token, '.') || token.empty()) {
        throw std::invalid_argument("robot_ip is not a valid IPv4 address: " + ip);
      }
      std::size_t parsed_characters = 0;
      int value = 0;
      try {
        value = std::stoi(token, &parsed_characters);
      } catch (const std::exception &) {
        throw std::invalid_argument("robot_ip is not a valid IPv4 address: " + ip);
      }
      if (parsed_characters != token.size() || value < 0 || value > 255) {
        throw std::invalid_argument("robot_ip is not a valid IPv4 address: " + ip);
      }
      octets[index] = static_cast<unsigned char>(value);
    }
    if (stream.rdbuf()->in_avail() != 0) {
      throw std::invalid_argument("robot_ip is not a valid IPv4 address: " + ip);
    }
    return octets;
  }

  static EnabledArms parse_enabled_arms(const std::string & value)
  {
    if (value == "none") {
      return EnabledArms::kNone;
    }
    if (value == "left") {
      return EnabledArms::kLeft;
    }
    if (value == "right") {
      return EnabledArms::kRight;
    }
    if (value == "both") {
      return EnabledArms::kBoth;
    }
    throw std::invalid_argument("enabled_arms must be one of: none, left, right, both");
  }

  void copy_control_arrays()
  {
    std::copy(
      left_stiffness_.begin(), left_stiffness_.end(),
      impedance_settings_.left_stiffness.begin());
    std::copy(
      left_damping_.begin(), left_damping_.end(),
      impedance_settings_.left_damping.begin());
    std::copy(
      right_stiffness_.begin(), right_stiffness_.end(),
      impedance_settings_.right_stiffness.begin());
    std::copy(
      right_damping_.begin(), right_damping_.end(),
      impedance_settings_.right_damping.begin());
    std::transform(
      initial_pose_deg_.begin(), initial_pose_deg_.end(), initial_pose_rad_.begin(),
      [](double value) {return degrees_to_radians(value);});
  }

  rclcpp::QoS make_qos(const std::string & reliability, int depth) const
  {
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(depth)));
    if (reliability == "reliable") {
      qos.reliable();
    } else if (reliability == "best_effort") {
      qos.best_effort();
    } else {
      throw std::invalid_argument(
              "QoS reliability must be 'reliable' or 'best_effort', got: " + reliability);
    }
    qos.durability_volatile();
    return qos;
  }

  bool connect_robot()
  {
    RCLCPP_INFO(get_logger(), "Connecting to Marvin robot at %s", robot_ip_.c_str());
    if (!sdk_.connect(parse_ipv4(robot_ip_))) {
      RCLCPP_ERROR(get_logger(), "Marvin SDK connection failed: %s", sdk_.last_error().c_str());
      return false;
    }
    have_feedback_ = false;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      joint_torque_zero_.clear(arm);
      torque_zero_context_[arm].reset();
      torque_zero_stamp_[arm] = 0;
    }
    publish_torque_zero_status();
    frames_initialized_ = false;
    frame_advanced_once_.fill(false);
    last_feedback_advance_.fill(std::chrono::steady_clock::now());
    control_pacer_.reset();
    send_timing_ = SendTiming{};
    target_send_guard_.reset();
    last_submitted_target_rad_.reset();
    feedback_recovery_frames_.reset();
    control_engaged_ = false;
    retreat_control_arms_.reset();
    control_purpose_ = ControlPurpose::kNone;
    initial_pose_stable_frames_ = 0;
    idle_retry_pending_ = false;
    impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
    clear_soft_limit_state();
    control_gate_.disarm("SDK connected; explicit enable required");
    retreat_completion_pending_ = false;
    right_ft_last_frame_.reset();
    right_ft_last_publish_.reset();
    right_ft_compensation_.reset();
    right_ft_channel_ = 0.0;
    right_ft_status_ = "WAITING_FOR_CHANNEL_216";
    if (publish_right_ft_sensor_ && !sdk_.select_right_ft_sensor()) {
      right_ft_status_ = "CHANNEL_SETUP_FAILED";
      RCLCPP_ERROR(get_logger(), "Right FT telemetry setup failed: %s", sdk_.last_error().c_str());
    }
    RCLCPP_INFO(get_logger(), "Marvin SDK connection established");
    return true;
  }

  void disconnect_robot()
  {
    right_ft_compensation_.reset();
    if (!sdk_.connected()) {
      return;
    }
    if (allow_motion_ && control_engaged_) {
      if (!sdk_.request_idle(control_arms())) {
        RCLCPP_ERROR(
          get_logger(), "Best-effort idle request failed during shutdown: %s",
          sdk_.last_error().c_str());
      }
      control_engaged_ = false;
      retreat_control_arms_.reset();
    }
    if (!sdk_.disconnect()) {
      RCLCPP_ERROR(get_logger(), "Marvin SDK release failed: %s", sdk_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Marvin SDK connection released");
    }
  }

  void on_timer()
  {
    process_tool_upload(nullptr); // Timeout also runs when no fresh feedback arrives.
    const auto now_steady = std::chrono::steady_clock::now();
    send_timing_.timer_started(now_steady);
    publish_alignment_diagnostics_if_due(now_steady);
    if (!sdk_.connected()) {
      if (connect_on_start_ && now_steady >= next_connect_attempt_) {
        if (!connect_robot()) {
          next_connect_attempt_ = now_steady +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(reconnect_period_sec_));
        }
      }
      publish_status_if_due(now_steady);
      return;
    }

    process_idle_retry(now_steady);

    DCSS feedback{};
    const auto read_started = std::chrono::steady_clock::now();
    const bool read_ok = sdk_.read_feedback(feedback);
    const auto read_finished = std::chrono::steady_clock::now();
    send_timing_.feedback_sdk_ms = SendTiming::ms(read_finished, read_started);
    send_timing_.max_feedback_sdk_ms = std::max(
      send_timing_.max_feedback_sdk_ms, send_timing_.feedback_sdk_ms);
    if (!read_ok) {
      clear_torque_zero_windows();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Marvin feedback read failed: %s",
        sdk_.last_error().c_str());
      if (control_engaged_) {
        pause_teleoperation_or_fault("feedback read failed", true);
      }
      publish_status_if_due(now_steady);
      return;
    }
    if (!feedback_is_finite(feedback)) {
      clear_torque_zero_windows();
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Rejected non-finite Marvin joint feedback");
      if (control_engaged_) {
        fault_and_idle("non-finite feedback");
      }
      publish_status_if_due(now_steady);
      return;
    }
    // A reported controller fault is not a recoverable telemetry interruption.
    if (control_engaged_ && !robot_healthy_for_enabled_arms(feedback)) {
      clear_torque_zero_windows();
      fault_and_idle("robot reported an error in feedback");
      publish_status_if_due(std::chrono::steady_clock::now());
      return;
    }
    if (!feedback_frames_are_fresh(feedback, read_finished)) {
      clear_torque_zero_windows();
      if (control_engaged_) {
        pause_teleoperation_or_fault("feedback frame timeout", true);
      }
      publish_status_if_due(now_steady);
      return;
    }

    latest_dcss_ = feedback;
    last_feedback_receive_ = read_finished;
    report_robot_errors(feedback);
    const auto feedback_stamp = now();
    const JointArray feedback_rad = feedback_positions(feedback);
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_feedback_rad_ = feedback_rad;
      have_feedback_ = true;
    }
    process_tool_upload(&feedback);
    // Check on every feedback tick, even when the next control update is not due.
    if (!control_engaged_) {
      target_send_guard_.reset();
    } else if (target_send_guard_.expired(
        std::chrono::steady_clock::now(), sdk_target_busy_timeout_sec_))
    {
      pause_teleoperation_or_fault("joint target retry deadline exceeded");
    }
    const auto control_now = std::chrono::steady_clock::now();
    if (control_pacer_.due(control_now)) {
      // Do not use the callback-entry time for command freshness after a slow read.
      if (control_engaged_ && SendTiming::ms(control_now, read_finished) >
        feedback_timeout_sec_ * 1000.0)
      {
        pause_teleoperation_or_fault("feedback snapshot expired before control update", true);
      } else {
        update_control(feedback, feedback_rad, control_now);
      }
      control_pacer_.completed(std::chrono::steady_clock::now(), 1.0 / control_rate_hz_);
    }
    // Telemetry serialization/publishing must not age this tick's control snapshot.
    // This remains a single-threaded executor: a blocked publisher can delay the NEXT tick.
    publish_feedback(feedback, feedback_stamp);
    if (publish_cartesian_disturbance_) {
      // Keep the existing control path ahead of the optional telemetry serialization.
      publish_cartesian_disturbance_estimates(feedback, feedback_stamp);
    }
    publish_status_if_due(std::chrono::steady_clock::now());
  }

  bool feedback_frames_are_fresh(
    const DCSS & feedback, std::chrono::steady_clock::time_point now_steady)
  {
    const std::array<int32_t, 2> frames = {
      feedback.m_Out[0].m_OutFrameSerial, feedback.m_Out[1].m_OutFrameSerial};
    if (!frames_initialized_) {
      last_frames_ = frames;
      frames_initialized_ = true;
      for (std::size_t arm = 0; arm < 2; ++arm) {
        if (frames[arm] != 0) {
          frame_advanced_once_[arm] = true;
          last_feedback_advance_[arm] = now_steady;
        }
      }
    } else {
      for (std::size_t arm = 0; arm < 2; ++arm) {
        if (frames[arm] != last_frames_[arm]) {
          last_frames_[arm] = frames[arm];
          frame_advanced_once_[arm] = true;
          last_feedback_advance_[arm] = now_steady;
        }
        const double stale_seconds = std::chrono::duration<double>(
          now_steady - last_feedback_advance_[arm]).count();
        if (stale_seconds > feedback_timeout_sec_) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Marvin arm %s feedback frame has not advanced for %.3f seconds",
            arm == 0 ? "A/left" : "B/right", stale_seconds);
          return false;
        }
      }
    }
    if (!frame_advanced_once_[0] || !frame_advanced_once_[1]) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for both Marvin arm feedback frames to advance before publishing");
      return false;
    }
    return true;
  }

  static bool feedback_is_finite(const DCSS & feedback)
  {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        if (!std::isfinite(feedback.m_Out[arm].m_FB_Joint_Pos[joint]) ||
          !std::isfinite(feedback.m_Out[arm].m_FB_Joint_Vel[joint]))
        {
          return false;
        }
      }
    }
    return true;
  }

  void report_robot_errors(const DCSS & feedback)
  {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (feedback.m_State[arm].m_CurState == ARM_STATE_ERROR ||
        feedback.m_State[arm].m_ERRCode != 0)
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Arm %s reports state=%d error_code=%d; no automatic error clearing is performed",
          arm == 0 ? "A/left" : "B/right",
          feedback.m_State[arm].m_CurState, feedback.m_State[arm].m_ERRCode);
      }
    }
  }

  static JointArray feedback_positions(const DCSS & feedback)
  {
    JointArray positions_rad{};
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      positions_rad[joint] = degrees_to_radians(feedback.m_Out[0].m_FB_Joint_Pos[joint]);
      positions_rad[kArmDof + joint] = degrees_to_radians(
        feedback.m_Out[1].m_FB_Joint_Pos[joint]);
    }
    return positions_rad;
  }

  static double torque_zero_time()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void clear_torque_zero_windows()
  {
    for (std::size_t arm = 0; arm < 2; ++arm) {joint_torque_zero_.clear_window(arm);}
    right_ft_compensation_.clear_window();
  }

  void publish_torque_zero_status()
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    for (std::size_t arm = 0; arm < 2; ++arm) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = arm == 0 ? "left" : "right";
      status.level = joint_torque_zero_.active(arm) ? status.OK : status.WARN;
      status.message = joint_torque_zero_.active(arm) ? "JOINT_ZERO_ACTIVE" : "JOINT_ZERO_NOT_SET";
      const auto add = [&status](const std::string & key, const std::string & value) {
          diagnostic_msgs::msg::KeyValue entry;
          entry.key = key; entry.value = value; status.values.push_back(entry);
        };
      add("active", joint_torque_zero_.active(arm) ? "true" : "false");
      add("revision", std::to_string(joint_torque_zero_.revision(arm)));
      add("zero_stamp_ns", std::to_string(torque_zero_stamp_[arm]));
      add("measured_torque_topic", state_publisher_->get_topic_name());
      add("corrected_topic", estimated_joint_disturbance_publisher_->get_topic_name());
      add("raw_topic", raw_state_publisher_->get_topic_name());
      add("bias_topic", torque_bias_publisher_->get_topic_name());
      for (std::size_t j = 0; j < kArmDof; ++j) {
        std::ostringstream value;
        value << std::setprecision(17) << joint_torque_zero_.bias()[arm * kArmDof + j];
        add("bias_joint" + std::to_string(j + 1), value.str());
      }
      message.status.push_back(status);
    }
    torque_zero_status_publisher_->publish(message);
  }

  void observe_joint_torque(const DCSS & feedback)
  {
    const double time = torque_zero_time();
    bool changed = false;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      std::array<double, 17> context{};
      context[0] = feedback.m_State[arm].m_CurState;
      std::copy_n(feedback.m_In[arm].m_ToolKine, 6, context.begin() + 1);
      std::copy_n(feedback.m_In[arm].m_ToolDyn, 10, context.begin() + 7);
      if (torque_zero_context_[arm] && context != *torque_zero_context_[arm]) {
        joint_torque_zero_.clear(arm);
        torque_zero_stamp_[arm] = 0;
        changed = true;
      }
      torque_zero_context_[arm] = context;
      if (feedback.m_State[arm].m_CurState == ARM_STATE_ERROR ||
        feedback.m_State[arm].m_ERRCode != 0 || tool_configuration_uncertain_arms_[arm] ||
        (tool_upload_ && tool_upload_->arm == arm) ||
        !std::all_of(context.begin(), context.end(), [](double v) {return std::isfinite(v);}))
      {
        joint_torque_zero_.clear_window(arm);
        continue;
      }
      JointTorqueZero::ArmArray q{}, dq{}, tau{};
      for (std::size_t j = 0; j < kArmDof; ++j) {
        q[j] = degrees_to_radians(feedback.m_Out[arm].m_FB_Joint_Pos[j]);
        dq[j] = degrees_to_radians(feedback.m_Out[arm].m_FB_Joint_Vel[j]);
        tau[j] = feedback.m_Out[arm].m_EST_Joint_Force[j];
      }
      joint_torque_zero_.observe(arm, time, feedback.m_Out[arm].m_OutFrameSerial, q, dq, tau);
    }
    if (changed) {publish_torque_zero_status();}
  }

  void on_zero_joint_torque(const std::array<bool, 2> & selected, bool clear,
    const std_srvs::srv::Trigger::Response::SharedPtr & response)
  {
    response->success = false;
    if (clear) {
      for (std::size_t arm = 0; arm < 2; ++arm) {
        if (selected[arm]) {joint_torque_zero_.clear(arm); torque_zero_stamp_[arm] = 0;}
      }
      response->success = true;
      response->message = "joint torque bias cleared; selected effort returns to raw from next feedback";
    } else {
      if (!sdk_.connected() || !have_feedback_) {
        response->message = "refused: live controller feedback required";
        return;
      }
      response->success = joint_torque_zero_.capture(selected, torque_zero_time(), response->message);
      if (!response->success) {return;}
      const auto stamp = now().nanoseconds();
      for (std::size_t arm = 0; arm < 2; ++arm) {
        if (selected[arm]) {torque_zero_stamp_[arm] = stamp;}
      }
    }
    publish_torque_zero_status();
  }

  void publish_feedback(const DCSS & feedback, const rclcpp::Time & stamp)
  {
    observe_joint_torque(feedback);
    const JointArray positions_rad = feedback_positions(feedback);
    JointArray velocities_rad_s{};
    JointArray feedback_joint_torque{};
    JointArray joint_disturbance{};
    bool feedback_joint_torque_finite = true;
    bool joint_disturbance_finite = true;
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      velocities_rad_s[joint] = degrees_to_radians(
        feedback.m_Out[0].m_FB_Joint_Vel[joint]);
      velocities_rad_s[kArmDof + joint] = degrees_to_radians(
        feedback.m_Out[1].m_FB_Joint_Vel[joint]);
      feedback_joint_torque[joint] = feedback.m_Out[0].m_FB_Joint_SToq[joint];
      feedback_joint_torque[kArmDof + joint] = feedback.m_Out[1].m_FB_Joint_SToq[joint];
      feedback_joint_torque_finite = feedback_joint_torque_finite &&
        std::isfinite(feedback_joint_torque[joint]) &&
        std::isfinite(feedback_joint_torque[kArmDof + joint]);
      joint_disturbance[joint] = feedback.m_Out[0].m_EST_Joint_Force[joint];
      joint_disturbance[kArmDof + joint] = feedback.m_Out[1].m_EST_Joint_Force[joint];
      joint_disturbance_finite = joint_disturbance_finite &&
        std::isfinite(joint_disturbance[joint]) &&
        std::isfinite(joint_disturbance[kArmDof + joint]);
    }
    const auto corrected = joint_torque_zero_.correct(joint_disturbance);
    if (!feedback_joint_torque_finite) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Published Marvin joint state without effort because feedback joint torque was non-finite");
    }
    if (!joint_disturbance_finite) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Published Marvin estimated joint disturbance without effort because its source was "
        "non-finite");
    }
    state_publisher_->publish(make_joint_feedback_message(
      positions_rad, velocities_rad_s, feedback_joint_torque, stamp, feedback_joint_torque_finite));
    estimated_joint_disturbance_publisher_->publish(make_joint_feedback_message(
      positions_rad, velocities_rad_s, corrected, stamp, joint_disturbance_finite));
    raw_state_publisher_->publish(make_joint_feedback_message(
      positions_rad, velocities_rad_s, joint_disturbance, stamp, joint_disturbance_finite));
    sensor_msgs::msg::JointState bias;
    bias.header.stamp = stamp;
    bias.name.assign(joint_names().begin(), joint_names().end());
    bias.effort.assign(joint_torque_zero_.bias().begin(), joint_torque_zero_.bias().end());
    torque_bias_publisher_->publish(bias);  // Same stamp as raw and corrected for exact replay.
    publish_right_ft_sensor(feedback, stamp);
  }

  void on_zero_ft_sensor(const std_srvs::srv::Trigger::Response::SharedPtr & response)
  {
    response->success = false;
    if (!sdk_.connected() || !have_feedback_) {
      response->message = "Live right-arm feedback required for software tare";
      return;
    }
    FtCompensation::Capture candidate;
    if (!right_ft_compensation_.candidate(torque_zero_time(), candidate, response->message)) {return;}
    try {
      const auto path = right_ft_compensation_.save_and_apply(candidate, right_ft_tare_directory_,
        now().nanoseconds(), right_ft_sensor_publisher_->get_topic_name());
      response->success = true;
      response->message = "Software tare saved: " + path + "; compensated+tared data starts on the next fresh frame";
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & e) {
      response->message = std::string("Software tare unchanged: ") + e.what();
    }
  }

  void publish_right_ft_sensor(const DCSS & feedback, const rclcpp::Time & stamp)
  {
    if (!right_ft_sensor_publisher_) {return;}
    const auto & out = feedback.m_Out[1];
    // Never stamp the same controller frame as a new measurement, including invalid frames.
    if (right_ft_last_frame_ && *right_ft_last_frame_ == out.m_OutFrameSerial) {return;}
    right_ft_last_frame_ = out.m_OutFrameSerial;
    right_ft_channel_ = out.m_EST_Joint_Firc[0];
    if (right_ft_channel_ != 216.0) {
      right_ft_compensation_.clear_window();
      right_ft_status_ = "CHANNEL_NOT_216";
      return;
    }
    if (feedback.m_State[1].m_ERRCode != 0) {
      right_ft_compensation_.clear_window();
      right_ft_status_ = "ROBOT_ERROR";
      return;
    }
    FtCompensation::Wrench measured{};
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.header.frame_id = right_ft_sensor_frame_id_;
    // Recording compatibility channels, not robot joint names. SDK order is preserved.
    message.name = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
    for (std::size_t i = 0; i < message.name.size(); ++i) {
      const double value = out.m_EST_Joint_Firc_Dot[i];
      if (!std::isfinite(value)) {
        right_ft_compensation_.clear_window();
        right_ft_status_ = "NONFINITE_SENSOR_DATA";
        return;
      }
      measured[i] = value * 0.0001;  // Tianji scale: 10000 counts per N or Nm.
    }
    FtCompensation::Joints q{}, velocity{};
    std::copy_n(out.m_FB_Joint_Pos, 7, q.begin());
    std::copy_n(out.m_FB_Joint_Vel, 7, velocity.begin());
    std::array<std::array<double, 3>, 3> rotation{};
    if (!kinematics_.flange_rotation(1, q, rotation)) {
      right_ft_compensation_.clear_window();
      right_ft_status_ = "FK_UNAVAILABLE";
      return;
    }
    try {
      Eigen::Matrix3d r;
      for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {r(i, j) = rotation[i][j];}
      }
      const auto compensated = right_ft_compensation_.compensate(measured, r);
      right_ft_compensation_.observe(torque_zero_time(), out.m_OutFrameSerial, stamp.nanoseconds(),
        feedback.m_State[1].m_CurState, out.m_LowSpdFlag == 1, q, velocity, compensated);
      if (!right_ft_compensation_.active()) {
        right_ft_status_ = "WAITING_FOR_SOFTWARE_TARE";
        return;
      }
      const auto corrected = right_ft_compensation_.corrected(compensated);
      message.effort.assign(corrected.begin(), corrected.end());
    } catch (const std::exception & e) {
      right_ft_compensation_.clear_window();
      right_ft_status_ = "COMPENSATION_FAILED";
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Right FT: %s", e.what());
      return;
    }
    right_ft_sensor_publisher_->publish(message);
    right_ft_last_publish_ = std::chrono::steady_clock::now();
    right_ft_status_ = "OK";
  }

  diagnostic_msgs::msg::DiagnosticStatus right_ft_sensor_diagnostic(
    std::chrono::steady_clock::time_point now_steady) const
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "tianji_teleop/right_ft_sensor";
    status.hardware_id = robot_ip_;
    const double age = right_ft_last_publish_ ?
      std::chrono::duration<double>(now_steady - *right_ft_last_publish_).count() : -1.0;
    status.message = right_ft_status_;
    status.level = status.ERROR;
    if (!sdk_.connected()) {
      status.message = "DISCONNECTED";
      status.level = status.STALE;
    } else if (right_ft_status_ == "OK") {
      if (age < 0.0 || age > feedback_timeout_sec_) {
        status.message = "NO_FRESH_SENSOR_FEEDBACK";
        status.level = status.STALE;
      } else {
        status.level = right_ft_sensor_frame_id_.empty() ? status.WARN : status.OK;
        status.message = right_ft_sensor_frame_id_.empty() ? "FRAME_ID_UNSET" : "OK";
      }
    } else if (right_ft_status_ == "WAITING_FOR_SOFTWARE_TARE") {
      status.level = status.WARN;
    }
    status.values = {
      diagnostic_value("topic", right_ft_sensor_topic_),
      diagnostic_value("channel", std::to_string(right_ft_channel_)),
      diagnostic_value("last_sample_age_sec", std::to_string(age)),
      diagnostic_value("frame_id", right_ft_sensor_frame_id_),
      diagnostic_value("components", "Fx,Fy,Fz,Tx,Ty,Tz"),
      diagnostic_value("units", "N,N,N,Nm,Nm,Nm"),
      diagnostic_value("sdk_scale", "0.0001")};
    status.values.push_back(diagnostic_value("processing", "gravity+calibration_bias+session_tare"));
    status.values.push_back(diagnostic_value("calibration_file", right_ft_calibration_file_));
    status.values.push_back(diagnostic_value("software_tare_active", right_ft_compensation_.active() ? "true" : "false"));
    status.values.push_back(diagnostic_value("tare_record", right_ft_compensation_.record_file()));
    return status;
  }

  void publish_cartesian_disturbance_estimates(
    const DCSS & feedback, const rclcpp::Time & stamp)
  {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      std::array<double, kArmDof> positions_deg{};
      std::array<double, kArmDof> arm_joint_disturbance{};
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        positions_deg[joint] = feedback.m_Out[arm].m_FB_Joint_Pos[joint];
        arm_joint_disturbance[joint] = feedback.m_Out[arm].m_EST_Joint_Force[joint] -
          joint_torque_zero_.bias()[arm * kArmDof + joint];
      }

      CartesianDisturbance cartesian_disturbance{};
      if (kinematics_.joint_disturbance_to_wrench(
          arm, positions_deg, arm_joint_disturbance, cartesian_disturbance))
      {
        const auto & publisher = arm == 0 ?
          left_cartesian_disturbance_publisher_ : right_cartesian_disturbance_publisher_;
        const auto & frame_id = arm == 0 ?
          left_cartesian_disturbance_frame_id_ : right_cartesian_disturbance_frame_id_;
        publisher->publish(
          make_cartesian_disturbance_message(
            cartesian_disturbance, stamp, frame_id));
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Skipped Marvin %s-arm wrench estimate: %s",
          arm == 0 ? "left" : "right", kinematics_.last_error().c_str());
      }
    }
  }

  // Keep the selected scope through HOLD/FAULT/idle retries, even after purpose
  // is cleared. The untouched arm must never receive shutdown/idle packets.
  EnabledArms control_arms() const {return retreat_control_arms_.value_or(enabled_arms_);}
  const char * retreat_arm_name() const {return retreat_arm_ == 0 ? "left" : "right";}

  bool robot_healthy_for_enabled_arms(const DCSS & feedback) const
  {
    const auto healthy = [&feedback](std::size_t arm) {
        return feedback.m_State[arm].m_CurState != ARM_STATE_ERROR &&
               feedback.m_State[arm].m_ERRCode == 0;
      };
    return (!left_enabled(control_arms()) || healthy(0)) &&
           (!right_enabled(control_arms()) || healthy(1));
  }

  JointImpedanceSettings control_impedance_settings() const
  {
    auto settings = impedance_settings_;
    if (control_purpose_ == ControlPurpose::kInitialPose) {
      settings.velocity_ratio = initial_pose_velocity_ratio_;
      settings.acceleration_ratio = initial_pose_acceleration_ratio_;
    }
    return settings;
  }

  bool impedance_parameters_verified(const DCSS & feedback) const
  {
    const auto settings = control_impedance_settings();
    const auto verified = [&settings, &feedback](std::size_t arm) {
        const auto & expected_k = arm == 0 ?
          settings.left_stiffness : settings.right_stiffness;
        const auto & expected_d = arm == 0 ?
          settings.left_damping : settings.right_damping;
        if (feedback.m_In[arm].m_Joint_Vel_Ratio != settings.velocity_ratio ||
          feedback.m_In[arm].m_Joint_Acc_Ratio != settings.acceleration_ratio)
        {
          return false;
        }
        for (std::size_t joint = 0; joint < kArmDof; ++joint) {
          if (std::abs(feedback.m_In[arm].m_Joint_K[joint] - expected_k[joint]) > 1.0e-4 ||
            std::abs(feedback.m_In[arm].m_Joint_D[joint] - expected_d[joint]) > 1.0e-4)
          {
            return false;
          }
        }
        return true;
      };
    return (!left_enabled(control_arms()) || verified(0)) &&
           (!right_enabled(control_arms()) || verified(1));
  }

  bool impedance_mode_verified(const DCSS & feedback) const
  {
    const auto mode_verified = [&feedback](std::size_t arm) {
        return feedback.m_State[arm].m_CurState == ARM_STATE_TORQ &&
               feedback.m_In[arm].m_ImpType == ARM_IMP_JOINT;
      };
    return impedance_parameters_verified(feedback) &&
           (!left_enabled(control_arms()) || mode_verified(0)) &&
           (!right_enabled(control_arms()) || mode_verified(1));
  }

  bool position_control_selected() const
  {
    return operation_control_mode_ == JointControlMode::kPosition;
  }

  bool position_switch_stationary(const DCSS & feedback) const
  {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!(arm == 0 ? left_enabled(control_arms()) : right_enabled(control_arms()))) {continue;}
      // Match the vendor SetJointMode low-speed prerequisite and also reject bad velocities.
      if (feedback.m_Out[arm].m_LowSpdFlag != 1) {return false;}
      for (const auto velocity : feedback.m_Out[arm].m_FB_Joint_Vel) {
        if (!std::isfinite(velocity) || std::abs(velocity) > 0.5) {return false;}
      }
    }
    return true;
  }

  bool control_parameters_verified(const DCSS & feedback) const
  {
    if (!position_control_selected()) {return impedance_parameters_verified(feedback);}
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!(arm == 0 ? left_enabled(control_arms()) : right_enabled(control_arms()))) {continue;}
      if (feedback.m_In[arm].m_Joint_Vel_Ratio != impedance_settings_.velocity_ratio ||
        feedback.m_In[arm].m_Joint_Acc_Ratio != impedance_settings_.acceleration_ratio)
      {return false;}
    }
    return true;
  }

  bool control_mode_verified(const DCSS & feedback) const
  {
    if (!position_control_selected()) {return impedance_mode_verified(feedback);}
    return position_mode_limits_verified(feedback) &&
           (!left_enabled(control_arms()) || feedback.m_State[0].m_CurState == ARM_STATE_POSITION) &&
           (!right_enabled(control_arms()) || feedback.m_State[1].m_CurState == ARM_STATE_POSITION);
  }

  bool position_mode_limits_verified(const DCSS & feedback) const
  {
    // The controller reports 29/29 after being commanded 30/30 in position mode.
    // Accept only a one-point downward readback: a higher or larger-mismatch
    // limit still blocks alignment and target submission.
    const auto matches = [](int actual, int requested) {
        return actual >= std::max(1, requested - 1) && actual <= requested;
      };
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!(arm == 0 ? left_enabled(control_arms()) : right_enabled(control_arms()))) {continue;}
      if (!matches(feedback.m_In[arm].m_Joint_Vel_Ratio,
          impedance_settings_.velocity_ratio) ||
        !matches(feedback.m_In[arm].m_Joint_Acc_Ratio,
          impedance_settings_.acceleration_ratio))
      {return false;}
    }
    return true;
  }

  bool position_mode_echoed(const DCSS & feedback) const
  {
    return (!left_enabled(control_arms()) ||
           feedback.m_State[0].m_CurState == ARM_STATE_POSITION) &&
           (!right_enabled(control_arms()) ||
           feedback.m_State[1].m_CurState == ARM_STATE_POSITION);
  }

  std::string describe_control_mode_echo(const DCSS & feedback) const
  {
    const auto settings = control_impedance_settings();
    std::ostringstream stream;
    stream << "expected_mode=" << joint_control_mode_name(operation_control_mode_) <<
      " expected_velocity_ratio=" << settings.velocity_ratio <<
      " expected_acceleration_ratio=" << settings.acceleration_ratio;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!(arm == 0 ? left_enabled(control_arms()) : right_enabled(control_arms()))) {continue;}
      stream << " " << (arm == 0 ? "left" : "right") << "_state=" <<
        feedback.m_State[arm].m_CurState <<
        " " << (arm == 0 ? "left" : "right") << "_velocity_ratio=" <<
        feedback.m_In[arm].m_Joint_Vel_Ratio <<
        " " << (arm == 0 ? "left" : "right") << "_acceleration_ratio=" <<
        feedback.m_In[arm].m_Joint_Acc_Ratio;
    }
    return stream.str();
  }

  static JointArray radians_to_degrees(const JointArray & radians)
  {
    JointArray degrees{};
    std::transform(
      radians.begin(), radians.end(), degrees.begin(),
      [](double value) {return tianji_io_teleop_bridge::radians_to_degrees(value);});
    return degrees;
  }

  double max_enabled_delta(const JointArray & first, const JointArray & second) const
  {
    double maximum = 0.0;
    const auto inspect_arm = [&first, &second, &maximum](std::size_t offset) {
        for (std::size_t joint = 0; joint < kArmDof; ++joint) {
          maximum = std::max(
            maximum, std::abs(first[offset + joint] - second[offset + joint]));
        }
      };
    if (left_enabled(control_arms())) {
      inspect_arm(0);
    }
    if (right_enabled(control_arms())) {
      inspect_arm(kArmDof);
    }
    return maximum;
  }

  std::array<bool, 2> enabled_arm_mask() const
  {
    return {left_enabled(control_arms()), right_enabled(control_arms())};
  }

  static bool any_arm_selected(const std::array<bool, 2> & arms)
  {
    return arms[0] || arms[1];
  }

  static std::string arm_mask_name(const std::array<bool, 2> & arms)
  {
    if (arms[0] && arms[1]) {
      return "both";
    }
    if (arms[0]) {
      return "left";
    }
    if (arms[1]) {
      return "right";
    }
    return "none";
  }

  bool enabled_arms_are_safe(const JointLimitMarginResult & result) const
  {
    return result.valid &&
           (!left_enabled(control_arms()) || result.arm_safe[0]) &&
           (!right_enabled(control_arms()) || result.arm_safe[1]);
  }

  bool enabled_targets_within_margin(const JointArray & positions, double margin_rad) const
  {
    return enabled_arms_are_safe(inspect_joint_limit_margin(positions, margin_rad));
  }

  double teleop_limit_extension() const
  {
    return teleop_soft_limits_enabled_ ? 0.0 :
           teleop_joint_limit_extension_rad_ + joint_limit_tolerance_rad_;
  }

  bool enabled_teleop_targets_within_limits(
    const JointArray & positions, bool recovering = false) const
  {
    const double margin = teleop_soft_limits_enabled_ ?
      (recovering ? soft_limit_release_margin_rad_ : joint_limit_safety_margin_rad_) : 0.0;
    return enabled_arms_are_safe(inspect_joint_limit_margin(
      positions, margin, teleop_limit_extension()));
  }

  static double max_arm_mask_delta(
    const JointArray & first, const JointArray & second,
    const std::array<bool, 2> & arms)
  {
    double maximum = 0.0;
    for (std::size_t arm = 0; arm < arms.size(); ++arm) {
      if (!arms[arm]) {
        continue;
      }
      const std::size_t offset = arm * kArmDof;
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        maximum = std::max(
          maximum, std::abs(first[offset + joint] - second[offset + joint]));
      }
    }
    return maximum;
  }

  static void copy_selected_arms(
    JointArray & destination, const JointArray & source,
    const std::array<bool, 2> & arms)
  {
    for (std::size_t arm = 0; arm < arms.size(); ++arm) {
      if (arms[arm]) {
        std::copy_n(
          source.begin() + arm * kArmDof, kArmDof,
          destination.begin() + arm * kArmDof);
      }
    }
  }

  std::string describe_limit_violations(
    const JointArray & positions, const JointLimitMarginResult & result,
    double margin_rad) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4);
    bool first = true;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const std::size_t arm = index / kArmDof;
      if ((arm == 0 && !left_enabled(control_arms())) ||
        (arm == 1 && !right_enabled(control_arms())) || result.joint_safe[index])
      {
        continue;
      }
      if (!first) {
        stream << "; ";
      }
      stream << joint_names()[index] << " target=" << positions[index] <<
        " safe=[" << lower_limits_rad()[index] + margin_rad << "," <<
        upper_limits_rad()[index] - margin_rad << "]";
      first = false;
    }
    return first ? "unknown enabled joint" : stream.str();
  }

  JointArray hold_disabled_arms_at_feedback(
    const JointArray & command, const JointArray & feedback) const
  {
    JointArray enabled_command = command;
    if (!left_enabled(control_arms())) {
      std::copy_n(feedback.begin(), kArmDof, enabled_command.begin());
    }
    if (!right_enabled(control_arms())) {
      std::copy_n(
        feedback.begin() + kArmDof, kArmDof, enabled_command.begin() + kArmDof);
    }
    return enabled_command;
  }

  bool submit_joint_target(const JointArray & target_rad)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    // Do not send a late recovery target after the deadline has already elapsed.
    if (target_send_guard_.expired(now_steady, sdk_target_busy_timeout_sec_)) {
      pause_teleoperation_or_fault("joint target retry deadline exceeded");
      return false;
    }
    const auto target_deg = radians_to_degrees(target_rad);
    const auto sdk_started = std::chrono::steady_clock::now();
    send_timing_.target_started(sdk_started);
    const auto result = sdk_.send_joint_targets(control_arms(), target_deg);
    const auto sdk_finished = std::chrono::steady_clock::now();
    send_timing_.target_sdk_ms = SendTiming::ms(sdk_finished, sdk_started);
    send_timing_.max_target_sdk_ms = std::max(
      send_timing_.max_target_sdk_ms, send_timing_.target_sdk_ms);
    if (result == TargetSendResult::kBusy) {
      const bool first_busy = !target_send_guard_.busy();
      target_send_guard_.observe_busy(sdk_finished);
      ++send_timing_.busy_total;
      if (first_busy) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Joint target SDK busy; skipping this update (retry deadline %.0f ms); %s",
          sdk_target_busy_timeout_sec_ * 1000.0, send_timing_.describe(sdk_finished).c_str());
      }
      return false;
    }
    if (result == TargetSendResult::kError) {
      fault_and_idle("joint target send failed: " + sdk_.last_error());
      return false;
    }
    if (target_send_guard_.busy()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Joint target submitted again %.1f ms after first busy response / %zu busy responses; %s",
        target_send_guard_.age_sec(sdk_finished) * 1000.0,
        target_send_guard_.attempts(), send_timing_.describe(sdk_finished).c_str());
    }
    send_timing_.target_submitted = sdk_finished;
    last_submitted_target_rad_ = target_rad;
    target_send_guard_.reset();
    return true;
  }

  bool send_position_target(
    const JointArray & desired, std::chrono::steady_clock::time_point now_steady)
  {
    // Submit the caller's position unchanged, as in the SDK examples, and track
    // it only after the SDK accepts the packet.
    if (!submit_joint_target(desired)) {
      return false;
    }
    last_control_update_ = now_steady;
    last_sent_target_rad_ = desired;
    return true;
  }

  void enter_soft_limit_hold(
    const JointArray & command, const JointArray & feedback,
    const JointLimitMarginResult & limit_result)
  {
    const auto enabled = enabled_arm_mask();
    std::array<bool, 2> triggering_arms{
      enabled[0] && !limit_result.arm_safe[0],
      enabled[1] && !limit_result.arm_safe[1]};
    soft_limit_frozen_arms_ = soft_limit_allow_other_arm_motion_ ? triggering_arms : enabled;
    soft_limit_hold_target_rad_ = last_sent_target_rad_;
    copy_selected_arms(soft_limit_hold_target_rad_, feedback, soft_limit_frozen_arms_);
    last_sent_target_rad_ = soft_limit_hold_target_rad_;
    soft_limit_trigger_detail_ = describe_limit_violations(
      command, limit_result, joint_limit_safety_margin_rad_);
    control_gate_.enter_soft_limit_hold(
      "soft joint limit; frozen arms=" + arm_mask_name(soft_limit_frozen_arms_) +
      "; " + soft_limit_trigger_detail_);
    RCLCPP_WARN(
      get_logger(),
      "Entering SOFT_LIMIT_HOLD without leaving %s control: frozen arms=%s; %s",
      joint_control_mode_name(operation_control_mode_),
      arm_mask_name(soft_limit_frozen_arms_).c_str(), soft_limit_trigger_detail_.c_str());
  }

  void freeze_new_soft_limit_arms(
    const JointArray & command, const JointArray & feedback,
    const JointLimitMarginResult & limit_result)
  {
    if (!soft_limit_allow_other_arm_motion_) {
      return;
    }
    const auto enabled = enabled_arm_mask();
    std::array<bool, 2> newly_frozen{
      enabled[0] && !limit_result.arm_safe[0] && !soft_limit_frozen_arms_[0],
      enabled[1] && !limit_result.arm_safe[1] && !soft_limit_frozen_arms_[1]};
    if (!any_arm_selected(newly_frozen)) {
      return;
    }
    copy_selected_arms(soft_limit_hold_target_rad_, feedback, newly_frozen);
    for (std::size_t arm = 0; arm < newly_frozen.size(); ++arm) {
      soft_limit_frozen_arms_[arm] = soft_limit_frozen_arms_[arm] || newly_frozen[arm];
    }
    JointArray reset_target = last_sent_target_rad_;
    copy_selected_arms(reset_target, soft_limit_hold_target_rad_, newly_frozen);
    last_sent_target_rad_ = reset_target;
    soft_limit_trigger_detail_ = describe_limit_violations(
      command, limit_result, joint_limit_safety_margin_rad_);
    control_gate_.enter_soft_limit_hold(
      "soft joint limit; frozen arms=" + arm_mask_name(soft_limit_frozen_arms_) +
      "; " + soft_limit_trigger_detail_);
    RCLCPP_WARN(
      get_logger(), "Additional arm frozen by soft joint limit: frozen arms=%s; %s",
      arm_mask_name(soft_limit_frozen_arms_).c_str(), soft_limit_trigger_detail_.c_str());
  }

  void clear_soft_limit_state()
  {
    soft_limit_frozen_arms_.fill(false);
    soft_limit_trigger_detail_.clear();
  }

  void update_control(
    const DCSS & feedback, const JointArray & feedback_rad,
    std::chrono::steady_clock::time_point now_steady)
  {
    const ControlState state = control_gate_.state();
    if (state == ControlState::kMonitor || state == ControlState::kFault) {
      return;
    }
    if (!allow_motion_) {
      control_gate_.enter_fault("internal invariant: motion lock is active");
      return;
    }

    std::optional<std::chrono::steady_clock::time_point> command_time;
    JointArray command{};
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      command_time = last_valid_command_time_;
      command = last_valid_command_rad_;
    }
    const bool command_fresh = command_time.has_value() &&
      std::chrono::duration<double>(now_steady - command_time.value()).count() <=
      command_timeout_sec_;
    const bool robot_healthy = robot_healthy_for_enabled_arms(feedback);

    if (state == ControlState::kPreparingParameters) {
      if (control_purpose_ == ControlPurpose::kNone) {
        fault_and_idle("no control purpose selected while preparing control parameters");
        return;
      }
      if (!robot_healthy) {
        return;
      }
      if (impedance_configuration_phase_ == ImpedanceConfigurationPhase::kNone) {
        if (position_control_selected() && !position_switch_stationary(feedback)) {
          control_purpose_ = ControlPurpose::kNone;
          control_gate_.disarm("position mode refused: enabled arms must be stationary; re-enable when ready");
          RCLCPP_WARN(get_logger(), "%s", control_gate_.reason().c_str());
          return;  // No mode or target packet was submitted.
        }
        const auto settings = control_impedance_settings();
        const bool configured = position_control_selected() ?
          sdk_.configure_position_parameters(control_arms(), impedance_settings_.velocity_ratio,
            impedance_settings_.acceleration_ratio) :
          sdk_.configure_joint_impedance_parameters(control_arms(), settings);
        if (!configured) {
          fault_and_idle("control parameter configuration failed: " + sdk_.last_error());
          return;
        }
        transition_started_ = now_steady;
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kWaitingForParameterEcho;
        RCLCPP_WARN(
          get_logger(),
          "%s parameter packet sent (velocity=%d%%, acceleration=%d%%); robot control mode is unchanged",
          joint_control_mode_name(operation_control_mode_),
          settings.velocity_ratio, settings.acceleration_ratio);
        return;
      }
      const double phase_age = std::chrono::duration<double>(
        now_steady - transition_started_).count();
      if (phase_age > mode_transition_timeout_sec_) {
        fault_and_idle("timed out waiting for control parameter confirmation");
        return;
      }
      if (phase_age >= sdk_packet_settle_sec_ && control_parameters_verified(feedback)) {
        if (position_control_selected() && !position_switch_stationary(feedback)) {
          impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
          control_purpose_ = ControlPurpose::kNone;
          control_gate_.disarm("position mode refused: arm moved during preparation; re-enable when stationary");
          RCLCPP_WARN(get_logger(), "%s", control_gate_.reason().c_str());
          return;  // Only speed parameters were sent; never switch a moving arm.
        }
        // Switching control mode can change robot behavior even though this packet contains
        // no position target. Mark it engaged first so every partial failure requests idle.
        control_engaged_ = true;
        const bool switched = position_control_selected() ? sdk_.enter_position_mode(control_arms()) :
          sdk_.enter_joint_impedance_mode(control_arms());
        if (!switched) {
          fault_and_idle("control mode request failed: " + sdk_.last_error());
          return;
        }
        transition_started_ = now_steady;
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kWaitingForModeEcho;
        control_gate_.mark_configuring(position_control_selected());
        RCLCPP_WARN(
          get_logger(),
          "%s parameters confirmed; pure mode-switch packet sent without a joint target",
          joint_control_mode_name(operation_control_mode_));
      }
      return;
    }

    if (state == ControlState::kWaitingForAlignment) {
      if (!robot_healthy) {
        begin_hold("robot error while waiting for alignment", feedback_rad,
          now_steady);
        return;
      }
      if (!control_mode_verified(feedback)) {
        fault_and_idle("selected control mode lost while waiting for alignment"); return;
      }
      // Explicit enable authorizes impedance preparation independently of TeleBox availability.
      // A missing/stale command simply keeps this state unaligned; no target is sent.
      const bool aligned = command_fresh &&
        enabled_teleop_targets_within_limits(command) &&
        max_enabled_delta(command, feedback_rad) <= initial_alignment_tolerance_rad_;
      if (!control_gate_.observe_alignment(aligned)) {
        return;
      }

      clear_soft_limit_state();
      // No target has been sent since the mode switch. Use feedback as the
      // following-error reference until the first direct teleoperation submission.
      last_sent_target_rad_ = feedback_rad;
      last_control_update_ = now_steady;
      control_gate_.mark_active();
      RCLCPP_WARN(
        get_logger(),
        "Alignment confirmed in %s mode; teleoperation control is ACTIVE",
        joint_control_mode_name(operation_control_mode_));
      return;
    }

    if (state == ControlState::kConfiguringImpedance || state == ControlState::kConfiguringPosition) {
      if (!robot_healthy) {
        begin_hold("robot error while entering control mode", feedback_rad, now_steady);
        return;
      }
      const double phase_age = std::chrono::duration<double>(
        now_steady - transition_started_).count();
      if (position_control_selected() &&
        impedance_configuration_phase_ == ImpedanceConfigurationPhase::kWaitingForModeEcho &&
        position_mode_echoed(feedback) && !position_mode_limits_verified(feedback) &&
        phase_age >= sdk_packet_settle_sec_)
      {
        // The controller may change speed/acceleration ratios while entering position mode.
        // As in Marvin's SetJointMode already-in-position branch, set and verify them again
        // before alignment or any joint target is permitted.
        if (!sdk_.configure_position_parameters(control_arms(), impedance_settings_.velocity_ratio,
            impedance_settings_.acceleration_ratio))
        {
          fault_and_idle("position-mode speed limit reconfiguration failed: " + sdk_.last_error());
          return;
        }
        transition_started_ = now_steady;
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kWaitingForPositionLimitEcho;
        RCLCPP_WARN(
          get_logger(), "Position mode echoed with a speed limit mismatch beyond the safe readback range; reapplying limits before alignment: %s",
          describe_control_mode_echo(feedback).c_str());
        return;
      }
      if (impedance_configuration_phase_ ==
        ImpedanceConfigurationPhase::kWaitingForPositionLimitEcho &&
        !position_mode_echoed(feedback))
      {
        fault_and_idle("position mode lost while confirming speed limits; " +
          describe_control_mode_echo(feedback));
        return;
      }
      const bool mode_ready =
        impedance_configuration_phase_ == ImpedanceConfigurationPhase::kWaitingForModeEcho &&
        control_mode_verified(feedback);
      const bool position_limits_ready =
        impedance_configuration_phase_ == ImpedanceConfigurationPhase::kWaitingForPositionLimitEcho &&
        phase_age >= sdk_packet_settle_sec_ && control_mode_verified(feedback);
      if (mode_ready || position_limits_ready)
      {
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
        if (control_purpose_ == ControlPurpose::kRetreatRight ||
          control_purpose_ == ControlPurpose::kRetreatLeft)
        {
          // Plan once from the actual pose after mode confirmation. Never reuse a
          // pre-switch path whose start may have shifted under soft impedance.
          ArmAngles q{};
          for (int j = 0; j < 7; ++j) {q[j] = tianji_io_teleop_bridge::radians_to_degrees(feedback_rad[j + 7 * retreat_arm_]);}
          retreat_plan_ = check_retreat(retreat_arm_, q, joint_limit_safety_margin_rad_, 100.0, 1000, true);
          if (!retreat_plan_.success) {
            begin_retreat_hold("post-mode planning failed: " + retreat_plan_.reason); return;
          }
          retreat_period_sec_ = std::max(
            retreat_duration_sec_ / (retreat_plan_.path_deg.size() - 1),
            1.0 / control_rate_hz_);
          last_sent_target_rad_ = feedback_rad;
          last_control_update_ = now_steady;
          retreat_started_ = now_steady;
          retreat_next_ = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(retreat_period_sec_));
          retreat_index_ = 1;
          control_gate_.mark_retreating();
          RCLCPP_WARN(get_logger(), "%s retreat ACTIVE: -X 100 mm, smooth start/stop; planned %.2f s, control ceiling %.1f Hz",
            retreat_arm_name(), retreat_period_sec_ * 1000, control_rate_hz_);
        } else if (control_purpose_ == ControlPurpose::kInitialPose) {
          initial_pose_started_ = now_steady;
          initial_pose_stable_frames_ = 0;
          last_sent_target_rad_ = feedback_rad;
          last_control_update_ = now_steady;
          control_gate_.mark_returning_to_initial_pose();
          RCLCPP_WARN(
            get_logger(),
            "Joint impedance mode confirmed; moving enabled arms to the configured initial pose");
        } else if (control_purpose_ == ControlPurpose::kTeleoperation) {
          control_gate_.mark_waiting_for_alignment();
          RCLCPP_WARN(
            get_logger(),
            "%s mode confirmed; no joint target sent; waiting for alignment",
            joint_control_mode_name(operation_control_mode_));
        } else {
          fault_and_idle("no control purpose selected after control mode confirmation");
        }
        return;
      }
      if (phase_age > mode_transition_timeout_sec_) {
        fault_and_idle("timed out waiting for selected control mode/limits confirmation; " +
          describe_control_mode_echo(feedback));
      }
      return;
    }

    if (state == ControlState::kRetreatHold) {
      if (!robot_healthy || !impedance_mode_verified(feedback)) {
        fault_and_idle("robot error/mode loss during retreat hold");
      }
      return;  // No new targets and no automatic teleoperation resumption.
    }
    if (state == ControlState::kRetreating) {
      if (!robot_healthy || !impedance_mode_verified(feedback)) {
        fault_and_idle("robot error/mode loss during retreat"); return;
      }
      if (!enabled_targets_within_margin(feedback_rad, joint_limit_safety_margin_rad_)) {
        begin_retreat_hold("measured joint position entered safety margin"); return;
      }
      if (std::chrono::duration<double>(now_steady - retreat_started_).count() >
        std::max(30.0, retreat_period_sec_ * 1000 * 3.0))
      {
        begin_retreat_hold("retreat send timeout; retaining last submitted target"); return;
      }
      // Reuse the bridge's joint-following bound rather than imposing a separate
      // precision Cartesian servo criterion on a compliant joint controller.
      if (max_enabled_delta(last_sent_target_rad_, feedback_rad) > max_following_error_rad_) {
        begin_retreat_hold("retreat exceeded max_following_error_rad; retaining last submitted target"); return;
      }
      if (now_steady < retreat_next_) {return;}
      JointArray desired = feedback_rad;
      for (int j = 0; j < 7; ++j) {
        desired[j + 7 * retreat_arm_] = degrees_to_radians(retreat_plan_.path_deg[retreat_index_][j]);
      }
      if (!enabled_targets_within_margin(desired, joint_limit_safety_margin_rad_)) {
        begin_retreat_hold("retreat target violates joint margin"); return;
      }
      // Send each planned waypoint unchanged at the configured trajectory cadence.
      if (submit_joint_target(desired)) {
        last_sent_target_rad_ = desired;
        last_control_update_ = now_steady;
        ++retreat_index_;
        if (retreat_index_ >= retreat_plan_.path_deg.size()) {
          control_purpose_ = ControlPurpose::kNone;
          hold_target_rad_ = last_sent_target_rad_;
          hold_started_ = std::chrono::steady_clock::now();
          // Preserve the selected arm scope until IDLE succeeds, including retries.
          retreat_completion_pending_ = true;
          const std::string reason = std::string(retreat_arm_name()) +
            " retreat target sequence sent: -X 100 mm; actual arrival not verified; holding briefly before idle";
          control_gate_.enter_hold(reason);
          RCLCPP_INFO(get_logger(), "%s", reason.c_str());
          return;
        }
        // Completion paced: a delayed/busy send never causes a catch-up burst.
        retreat_next_ = std::max(now_steady, std::chrono::steady_clock::now()) +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(retreat_period_sec_));
      }
      return;
    }
    if (state == ControlState::kReturningToInitialPose) {
      if (!robot_healthy) {
        begin_hold("robot reported an error while returning to initial pose", feedback_rad,
          now_steady);
        return;
      }
      if (std::chrono::duration<double>(now_steady - initial_pose_started_).count() >
        initial_pose_timeout_sec_)
      {
        begin_hold("initial pose motion timed out", feedback_rad, now_steady);
        return;
      }
      if (!impedance_mode_verified(feedback)) {
        fault_and_idle("joint impedance mode lost while returning to initial pose");
        return;
      }
      if (!enabled_targets_within_margin(
          initial_pose_rad_, joint_limit_safety_margin_rad_))
      {
        fault_and_idle("configured initial pose entered the joint-limit safety margin");
        return;
      }

      // This is a point move: submit the final position once, as in the SDK
      // example, then wait for measured arrival. Distance to the final point
      // during the move is not a following error against a streamed reference.
      if (!last_submitted_target_rad_) {
        const JointArray desired = hold_disabled_arms_at_feedback(initial_pose_rad_, feedback_rad);
        if (!send_position_target(desired, now_steady)) {
          return;
        }
      }
      if (max_enabled_delta(initial_pose_rad_, feedback_rad) <= initial_pose_tolerance_rad_) {
        ++initial_pose_stable_frames_;
      } else {
        initial_pose_stable_frames_ = 0;
      }
      if (initial_pose_stable_frames_ >=
        static_cast<std::size_t>(initial_pose_required_frames_))
      {
        complete_initial_pose_motion(feedback_rad, now_steady);
      }
      return;
    }

    if (state == ControlState::kActive) {
      if (!robot_healthy) {
        begin_hold("robot reported an error", feedback_rad, now_steady);
        return;
      }
      if (!control_mode_verified(feedback)) {
        fault_and_idle("selected control mode lost during teleoperation"); return;
      }
      if (!command_fresh) {
        begin_command_wait("valid arm command timed out");
        return;
      }
      if (max_enabled_delta(last_sent_target_rad_, feedback_rad) > max_following_error_rad_) {
        begin_hold("following error exceeded safety threshold", feedback_rad, now_steady);
        return;
      }
      // Backup bounds were checked on receipt. Do not impose a second, tighter
      // URDF-inset gate on targets already limited by IO.
      if (teleop_soft_limits_enabled_) {
        const auto limit_result = inspect_joint_limit_margin(
          command, joint_limit_safety_margin_rad_);
        if (!enabled_arms_are_safe(limit_result)) {
          enter_soft_limit_hold(command, feedback_rad, limit_result);
          return;
        }
      }

      const JointArray desired = hold_disabled_arms_at_feedback(command, feedback_rad);
      (void)send_position_target(desired, now_steady);
      return;
    }

    if (state == ControlState::kSoftLimitHold) {
      if (!robot_healthy) {
        begin_hold("robot reported an error during soft-limit hold", feedback_rad, now_steady);
        return;
      }
      if (!control_mode_verified(feedback)) {
        fault_and_idle("selected control mode lost during soft-limit hold"); return;
      }
      if (!command_fresh) {
        begin_command_wait("valid arm command timed out during soft-limit hold");
        return;
      }
      if (max_enabled_delta(last_sent_target_rad_, feedback_rad) > max_following_error_rad_) {
        begin_hold(
          "following error exceeded safety threshold during soft-limit hold",
          feedback_rad, now_steady);
        return;
      }

      const auto limit_result = inspect_joint_limit_margin(
        command, joint_limit_safety_margin_rad_);
      if (!enabled_arms_are_safe(limit_result)) {
        freeze_new_soft_limit_arms(command, feedback_rad, limit_result);
      }

      const bool command_has_release_clearance = enabled_targets_within_margin(
        command, soft_limit_release_margin_rad_);
      const bool command_is_realigned = max_arm_mask_delta(
        command, feedback_rad, soft_limit_frozen_arms_) <=
        soft_limit_recovery_alignment_rad_;
      if (control_gate_.observe_soft_limit_recovery(
          command_has_release_clearance && command_is_realigned))
      {
        const std::string previously_frozen = arm_mask_name(soft_limit_frozen_arms_);
        clear_soft_limit_state();
        control_gate_.mark_active();
        RCLCPP_WARN(
          get_logger(),
          "Soft joint limit cleared after %d stable frames; resumed ACTIVE from frozen arms=%s",
          soft_limit_recovery_frames_, previously_frozen.c_str());
        return;
      }

      JointArray desired = hold_disabled_arms_at_feedback(command, feedback_rad);
      copy_selected_arms(desired, soft_limit_hold_target_rad_, soft_limit_frozen_arms_);
      (void)send_position_target(desired, now_steady);
      return;
    }

    if (state == ControlState::kCommandWait) {
      if (!robot_healthy || !control_mode_verified(feedback)) {
        fault_and_idle("robot error or selected control mode changed while waiting for commands");
        return;
      }
      if (feedback_recovery_frames_) {
        if (feedback.m_Out[0].m_OutFrameSerial == (*feedback_recovery_frames_)[0] ||
          feedback.m_Out[1].m_OutFrameSerial == (*feedback_recovery_frames_)[1])
        {
          (void)control_gate_.observe_alignment(false);
          return;
        }
        feedback_recovery_frames_.reset();
        // Commands received during the feedback outage must not count towards recovery.
        last_recovery_command_count_ = accepted_command_count_;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          last_valid_command_time_.reset();
        }
        (void)control_gate_.observe_alignment(false);
        RCLCPP_WARN(get_logger(),
          "Both feedback frames advanced again; waiting for fresh commands to realign");
        return;
      }
      if (command_wait_target_rad_ &&
        max_enabled_delta(*command_wait_target_rad_, feedback_rad) > max_following_error_rad_)
      {
        begin_hold("following error exceeded safety threshold while waiting for commands",
          feedback_rad, now_steady);
        return;
      }
      const bool aligned = command_fresh &&
        enabled_teleop_targets_within_limits(command, true) &&
        max_enabled_delta(command, feedback_rad) <= command_recovery_alignment_rad_;
      if (!aligned) {
        (void)control_gate_.observe_alignment(false);
        last_recovery_command_count_ = accepted_command_count_;
        return;
      }
      // A single cached message must not accumulate enough checks to resume.
      if (accepted_command_count_ == last_recovery_command_count_) {
        return;
      }
      last_recovery_command_count_ = accepted_command_count_;
      if (control_gate_.observe_alignment(true)) {
        const auto start = command_wait_target_rad_.value_or(feedback_rad);
        last_sent_target_rad_ = start;
        last_control_update_ = now_steady;
        clear_soft_limit_state();
        control_gate_.mark_active();
        RCLCPP_WARN(get_logger(), "Fresh commands realigned; resuming ACTIVE from held target");
      }
      // No target or mode packets during COMMAND_WAIT. Controller retains its last target.
      return;
    }

    if (state == ControlState::kHold) {
      if (!control_engaged_) {
        control_gate_.disarm("hold completed; explicit re-enable required");
        return;
      }
      const double hold_age = std::chrono::duration<double>(now_steady - hold_started_).count();
      if (robot_healthy && hold_age < hold_duration_sec_) {
        (void)submit_joint_target(hold_target_rad_);
        return;
      }
      if (!sdk_.request_idle(control_arms())) {
        control_gate_.enter_fault("idle request failed after hold: " + sdk_.last_error());
        RCLCPP_ERROR(get_logger(), "%s", control_gate_.reason().c_str());
        schedule_idle_retry();
        return;
      } else {
        control_gate_.disarm("hold completed; explicit re-enable required");
        if (retreat_completion_pending_) {
          RCLCPP_INFO(get_logger(), "%s retreat finished: IDLE request sent; returned to MONITOR", retreat_arm_name());
          retreat_completion_pending_ = false;
        }
      }
      control_engaged_ = false;
      retreat_control_arms_.reset();
    }
  }

  void begin_command_wait(const std::string & reason)
  {
    const bool already_waiting = control_gate_.state() == ControlState::kCommandWait;
    command_wait_target_rad_ = last_submitted_target_rad_;
    // The retry window limits active retries, not how long the controller may hold.
    target_send_guard_.reset();
    clear_soft_limit_state();
    last_recovery_command_count_ = accepted_command_count_;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_valid_command_time_.reset();
    }
    control_gate_.enter_command_wait(reason);
    if (already_waiting) {
      return;
    }
    RCLCPP_WARN(get_logger(),
      "Entering COMMAND_WAIT: %s; keeping %s and last submitted target; no automatic idle",
      reason.c_str(), joint_control_mode_name(operation_control_mode_));
  }

  void pause_teleoperation_or_fault(const std::string & reason, bool feedback_unavailable = false)
  {
    const auto state = control_gate_.state();
    if (state == ControlState::kRetreating || state == ControlState::kRetreatHold) {
      if (state != ControlState::kRetreatHold) {begin_retreat_hold(reason);}
      return;
    }
    // Do not turn a manual stop, a configuration failure or a return-to-pose task
    // into automatically resumable teleoperation.
    const bool can_wait = control_engaged_ && !idle_retry_pending_ &&
      (state == ControlState::kActive || state == ControlState::kSoftLimitHold ||
      state == ControlState::kCommandWait || state == ControlState::kWaitingForAlignment);
    if (!can_wait) {
      fault_and_idle(reason);
      return;
    }
    std::string detail = reason;
    if (target_send_guard_.busy()) {
      detail += "; elapsed_since_first_busy_ms=" + std::to_string(
        target_send_guard_.age_sec(std::chrono::steady_clock::now()) * 1000.0) +
        " busy_responses=" + std::to_string(target_send_guard_.attempts());
    }
    detail += "; " + send_timing_.describe(std::chrono::steady_clock::now());
    if (feedback_unavailable && !feedback_recovery_frames_) {
      feedback_recovery_frames_ = last_frames_;
    }
    begin_command_wait(detail);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Teleoperation paused without idle: %s", detail.c_str());
  }

  void begin_hold(
    const std::string & reason, const JointArray & feedback_rad,
    std::chrono::steady_clock::time_point now_steady)
  {
    retreat_completion_pending_ = false;
    feedback_recovery_frames_.reset();
    clear_soft_limit_state();
    control_purpose_ = ControlPurpose::kNone;
    initial_pose_stable_frames_ = 0;
    hold_target_rad_ = feedback_rad;
    hold_started_ = now_steady;
    control_gate_.enter_hold(reason);
    RCLCPP_ERROR(get_logger(), "Entering HOLD: %s", reason.c_str());
  }

  void complete_initial_pose_motion(
    const JointArray & feedback_rad, std::chrono::steady_clock::time_point now_steady)
  {
    control_purpose_ = ControlPurpose::kNone;
    initial_pose_stable_frames_ = 0;
    hold_target_rad_ = feedback_rad;
    hold_started_ = now_steady;
    control_gate_.enter_hold("initial pose reached; holding briefly before idle");
    RCLCPP_INFO(
      get_logger(),
      "Initial pose reached for %d stable frames; holding briefly before requesting idle",
      initial_pose_required_frames_);
  }

  void fault_and_idle(const std::string & reason)
  {
    retreat_completion_pending_ = false;
    feedback_recovery_frames_.reset();
    std::string detail = reason;
    if (target_send_guard_.busy()) {
      detail += "; elapsed_since_first_busy_ms=" + std::to_string(
        target_send_guard_.age_sec(std::chrono::steady_clock::now()) * 1000.0) +
        " busy_responses=" + std::to_string(target_send_guard_.attempts());
    }
    detail += "; " + send_timing_.describe(std::chrono::steady_clock::now());
    target_send_guard_.reset();
    clear_soft_limit_state();
    control_purpose_ = ControlPurpose::kNone;
    initial_pose_stable_frames_ = 0;
    control_gate_.enter_fault(detail);
    impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
    RCLCPP_ERROR(get_logger(), "Control fault: %s", detail.c_str());
    if (allow_motion_ && control_engaged_ && sdk_.connected() && !idle_retry_pending_) {
      if (!sdk_.request_idle(control_arms())) {
        RCLCPP_ERROR(
          get_logger(), "Best-effort idle request also failed: %s", sdk_.last_error().c_str());
        schedule_idle_retry();
      } else {
        control_engaged_ = false;
        retreat_control_arms_.reset();
      }
    }
  }

  void schedule_idle_retry()
  {
    idle_retry_pending_ = true;
    idle_retry_attempts_ = 0;
    next_idle_retry_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(sdk_packet_settle_sec_));
  }

  void process_idle_retry(std::chrono::steady_clock::time_point now_steady)
  {
    if (!idle_retry_pending_ || !control_engaged_ || now_steady < next_idle_retry_) {
      return;
    }
    ++idle_retry_attempts_;
    if (sdk_.request_idle(control_arms())) {
      if (retreat_completion_pending_) {
        control_gate_.disarm("retreat completed; delayed idle sent; explicit re-enable required");
        retreat_completion_pending_ = false;
        RCLCPP_INFO(get_logger(), "%s retreat finished: delayed IDLE request sent; returned to MONITOR", retreat_arm_name());
      }
      idle_retry_pending_ = false;
      control_engaged_ = false;
      retreat_control_arms_.reset();
      RCLCPP_WARN(get_logger(), "Delayed idle retry succeeded");
      return;
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Delayed idle retry failed: %s",
      sdk_.last_error().c_str());
    if (idle_retry_attempts_ >= 20) {
      idle_retry_pending_ = false;
      RCLCPP_FATAL(
        get_logger(),
        "Unable to send idle after 20 retries; use the physical emergency stop and inspect "
        "the controller state");
      return;
    }
    next_idle_retry_ = now_steady +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(sdk_packet_settle_sec_));
  }

  void on_joint_command(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    std::optional<JointArray> reference;
    const ControlState state = control_gate_.state();
    const bool motion_is_following =
      state == ControlState::kActive || state == ControlState::kSoftLimitHold;
    if (motion_is_following) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (last_valid_command_time_.has_value() &&
        std::chrono::duration<double>(
          now_steady - last_valid_command_time_.value()).count() <=
        command_reference_timeout_sec_)
      {
        reference = last_valid_command_rad_;
      } else if (have_feedback_) {
        reference = latest_feedback_rad_;
      }
    }

    const auto evaluation = evaluate_joint_command(
      *message, reference, max_command_step_rad_, joint_limit_tolerance_rad_ +
      (teleop_soft_limits_enabled_ ? 0.0 : teleop_joint_limit_extension_rad_));
    if (evaluation.disposition == CommandDisposition::kIgnoredNonArm) {
      ++ignored_non_arm_count_;
      return;
    }
    if (evaluation.disposition == CommandDisposition::kRejected) {
      ++rejected_command_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected Tianji arm joint_cmd (%lu rejected): %s",
        static_cast<unsigned long>(rejected_command_count_), evaluation.reason.c_str());
      if (motion_is_following) {
        begin_command_wait("rejected arm command: " + evaluation.reason);
      } else if (state == ControlState::kCommandWait) {
        (void)control_gate_.observe_alignment(false);
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_valid_command_time_.reset();
      }
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_valid_command_rad_ = evaluation.positions_rad;
      last_valid_command_time_ = now_steady;
    }
    ++accepted_command_count_;
    if (state == ControlState::kCommandWait &&
      (!have_feedback_ ||
      max_enabled_delta(evaluation.positions_rad, latest_feedback_rad_) > command_recovery_alignment_rad_ ||
      !enabled_teleop_targets_within_limits(evaluation.positions_rad, true)))
    {
      (void)control_gate_.observe_alignment(false);
    }
  }

  void on_enable_control(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (!request->data) {
      if (idle_retry_pending_) {
        response->success = true;
        response->message = "control faulted; delayed idle retry is already pending";
        return;
      }
      if (control_engaged_ && have_feedback_) {
        JointArray feedback{};
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          feedback = latest_feedback_rad_;
        }
        begin_hold("operator requested disable", feedback, std::chrono::steady_clock::now());
        response->message = "disable accepted; holding briefly before idle";
      } else {
        control_purpose_ = ControlPurpose::kNone;
        initial_pose_stable_frames_ = 0;
        control_gate_.disarm("operator requested disable");
        retreat_completion_pending_ = false;
        retreat_control_arms_.reset();
        response->message = "control disabled";
      }
      response->success = true;
      return;
    }

    if (tool_upload_ || tool_configuration_uncertain()) {
      response->message = "refused: tool upload pending/unconfirmed; verify tool configuration first";
    } else if (!allow_motion_) {
      response->message = "refused: allow_motion is false";
    } else if (control_gate_.state() != ControlState::kMonitor || control_engaged_) {
      response->message = "refused: control is not in MONITOR; disable or resolve the fault first";
    } else if (!sdk_control_compatibility_confirmed_) {
      response->message = "refused: SDK/controller control compatibility is not confirmed";
    } else if (!tool_dynamics_confirmed_) {
      response->message = "refused: tool dynamics configuration is not confirmed";
    } else if (enabled_arms_ == EnabledArms::kNone) {
      response->message = "refused: enabled_arms is none";
    } else if (!sdk_.connected() || !have_feedback_ ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_feedback_receive_).count() >
      feedback_timeout_sec_)
    {
      response->message = "refused: fresh Marvin feedback is unavailable";
    } else if (!robot_healthy_for_enabled_arms(latest_dcss_)) {
      response->message = "refused: an enabled arm reports an error";
    } else if (teleop_control_mode_ == JointControlMode::kPosition &&
      !position_switch_stationary(latest_dcss_))
    {
      response->message = "refused: position mode requires stationary enabled arms (SDK low-speed flag and <=0.5 deg/s)";
    } else {
      operation_control_mode_ = teleop_control_mode_;
      impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
      clear_soft_limit_state();
      control_purpose_ = ControlPurpose::kTeleoperation;
      last_submitted_target_rad_.reset();
      feedback_recovery_frames_.reset();
      initial_pose_stable_frames_ = 0;
      control_gate_.request_enable();
      response->success = true;
      response->message =
        "enable accepted; preparing parameters, then switching mode before alignment";
      return;
    }
    response->success = false;
  }

  void begin_retreat_hold(const std::string & reason)
  {
    retreat_completion_pending_ = false;
    target_send_guard_.reset();
    clear_soft_limit_state();
    control_gate_.enter_retreat_hold(reason);
    RCLCPP_WARN(get_logger(), "RETREAT_HOLD: %s; no automatic idle or teleoperation", reason.c_str());
  }

  void publish_tool_status(const std::string & message)
  {
    std_msgs::msg::String status;
    status.data = message;
    tool_status_publisher_->publish(status);
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  bool tool_configuration_uncertain() const
  {
    return tool_configuration_uncertain_arms_[0] || tool_configuration_uncertain_arms_[1];
  }

  void finish_tool_upload(bool verified, const std::string & reason)
  {
    const auto details = describe_tool_configuration(tool_upload_->arm, tool_upload_->config);
    tool_configuration_uncertain_arms_[tool_upload_->arm] = !verified;
    tool_upload_.reset();
    publish_tool_status((verified ? "VERIFIED: " : "UNCONFIRMED: ") + reason + " " + details);
  }

  void process_tool_upload(const DCSS * feedback)
  {
    if (!tool_upload_) {return;}
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - tool_upload_->started).count();
    if (age > tool_config_timeout_sec_) {
      finish_tool_upload(false, "tool feedback timeout; no rollback or automatic retry; motion blocked");
      return;
    }
    if (!feedback) {return;}
    const auto arm = tool_upload_->arm;
    if (feedback->m_State[arm].m_CurState != ARM_STATE_IDLE || feedback->m_State[arm].m_ERRCode != 0) {
      finish_tool_upload(false, "controller no longer healthy/IDLE; motion blocked"); return;
    }
    if (age >= sdk_packet_settle_sec_ &&
      feedback->m_Out[arm].m_OutFrameSerial != tool_upload_->frame &&
      tool_configuration_matches(tool_upload_->config, feedback->m_In[arm].m_ToolKine,
        feedback->m_In[arm].m_ToolDyn))
    {finish_tool_upload(true, "fresh controller parameter echo matched; manual tool_dynamics_confirmed unchanged");}
  }

  void on_configure_tool(std::size_t arm, std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->success = false;
    ToolConfiguration config;
    std::string path = tools_config_file_;
    try {
      if (path.empty()) {
        path = ament_index_cpp::get_package_share_directory("tianji_io_teleop_bridge") + "/config/tools_cfg.json";
      }
      config = load_tool_configuration(path, arm); // Read current_tool on each call.
    } catch (const std::exception & error) {
      response->message = std::string("refused: tool file invalid: ") + error.what(); return;
    }
    const std::string details = describe_tool_configuration(arm, config) + " source=" + path;
    if (tool_config_dry_run_) {
      response->success = true;
      response->message = "DRY_RUN: no SDK write; " + details;
      if (!tool_upload_) {publish_tool_status(response->message);}
      return;
    }
    if (!allow_motion_ || !sdk_control_compatibility_confirmed_ ||
      !(arm == 0 ? left_enabled(enabled_arms_) : right_enabled(enabled_arms_)))
    {
      response->message = "refused: require allow_motion, SDK compatibility and enabled arm"; return;
    }
    if (control_gate_.state() != ControlState::kMonitor || control_engaged_ ||
      idle_retry_pending_ || tool_upload_)
    {response->message = "refused: require MONITOR with no active operation or pending packet"; return;}
    const auto now_steady = std::chrono::steady_clock::now();
    if (!sdk_.connected() || !have_feedback_ || !feedback_is_finite(latest_dcss_) ||
      std::chrono::duration<double>(now_steady - last_feedback_receive_).count() > feedback_timeout_sec_)
    {response->message = "refused: fresh finite feedback required"; return;}
    for (std::size_t a = 0; a < 2; ++a) {
      if (!frame_advanced_once_[a] ||
        std::chrono::duration<double>(now_steady - last_feedback_advance_[a]).count() > feedback_timeout_sec_ ||
        latest_dcss_.m_State[a].m_CurState != ARM_STATE_IDLE || latest_dcss_.m_State[a].m_ERRCode != 0)
      {response->message = "refused: both arms must have fresh healthy IDLE feedback"; return;}
      for (std::size_t j = 0; j < 7; ++j) {
        if (std::abs(latest_dcss_.m_Out[a].m_FB_Joint_Vel[j]) > 0.5) {
          response->message = "refused: both arms must be stationary"; return;
        }
      }
    }
    const auto sent = sdk_.configure_tool(arm, config);
    if (sent == TargetSendResult::kBusy) {
      response->message = "refused: " + sdk_.last_error(); return;
    }
    tool_configuration_uncertain_arms_[arm] = true; // Sending is not confirmation, including send errors.
    joint_torque_zero_.clear(arm);
    torque_zero_stamp_[arm] = 0;
    publish_torque_zero_status();
    if (sent != TargetSendResult::kSubmitted) {
      response->message = "UNCONFIRMED: " + sdk_.last_error() + "; motion blocked; " + details;
      publish_tool_status(response->message); return;
    }
    tool_upload_ = ToolUpload{arm, config, std::chrono::steady_clock::now(),
      latest_dcss_.m_Out[arm].m_OutFrameSerial};
    response->success = true;
    response->message = "SENT_PENDING_ECHO: not yet verified; " + details;
    publish_tool_status(response->message);
  }

  void on_retreat(std::size_t arm, std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->success = false;
    if (tool_upload_ || tool_configuration_uncertain()) {
      response->message = "refused: tool upload pending/unconfirmed"; return;
    }
    if (!allow_motion_ || !sdk_control_compatibility_confirmed_ || !tool_dynamics_confirmed_) {
      response->message = "refused: motion permission, SDK compatibility and tool dynamics confirmation required";
      return;
    }
    if (arm > 1 || !(arm == 0 ? left_enabled(enabled_arms_) : right_enabled(enabled_arms_)) ||
      control_gate_.state() != ControlState::kMonitor || control_engaged_ || idle_retry_pending_)
    {
      response->message = "refused: requested arm must be enabled, with MONITOR and no active control or pending idle";
      return;
    }
    const auto fresh = [this, arm]() {
        const auto now = std::chrono::steady_clock::now();
        if (!sdk_.connected() || !have_feedback_ || !feedback_is_finite(latest_dcss_) ||
          std::chrono::duration<double>(now - last_feedback_receive_).count() > feedback_timeout_sec_)
        {return false;}
        for (std::size_t arm = 0; arm < 2; ++arm) {
          if (!frame_advanced_once_[arm] ||
            std::chrono::duration<double>(now - last_feedback_advance_[arm]).count() > feedback_timeout_sec_)
          {return false;}
        }
        return latest_dcss_.m_State[arm].m_CurState != ARM_STATE_ERROR &&
               latest_dcss_.m_State[arm].m_ERRCode == 0;
      };
    if (!fresh()) {response->message = "refused: fresh healthy feedback required"; return;}
    for (std::size_t j = 0; j < 7; ++j) {
      if (std::abs(latest_dcss_.m_Out[arm].m_FB_Joint_Vel[j]) > 0.5) {
        response->message = "refused: requested arm must be stationary (<0.5 deg/s)"; return;
      }
    }
    try {
      if (!kinematics_.initialized()) {initialize_kinematics();}
    } catch (const std::exception & error) {
      response->message = std::string("refused: ") + error.what(); return;
    }
    const auto limits = inspect_joint_limit_margin(latest_feedback_rad_, joint_limit_safety_margin_rad_);
    if (!fresh() || !limits.valid || !limits.arm_safe[arm]) {
      response->message = "refused: feedback expired or current joints outside planning margin"; return;
    }
    impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
    clear_soft_limit_state();
    target_send_guard_.reset();
    last_submitted_target_rad_.reset();
    retreat_arm_ = arm;
    retreat_control_arms_ = arm == 0 ? EnabledArms::kLeft : EnabledArms::kRight;
    control_purpose_ = arm == 0 ? ControlPurpose::kRetreatLeft : ControlPurpose::kRetreatRight;
    operation_control_mode_ = JointControlMode::kJointImpedance;
    control_gate_.request_enable();
    response->success = true;
    response->message = std::string("accepted: prepare impedance, plan from current pose, then send ") +
      retreat_arm_name() + "-arm -X 100 mm path; not collision checked";
  }

  void on_move_to_initial_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    if (tool_upload_ || tool_configuration_uncertain()) {
      response->success = false;
      response->message = "refused: tool upload pending/unconfirmed"; return;
    }
    if (!allow_motion_) {
      response->message = "refused: allow_motion is false";
    } else if (control_gate_.state() != ControlState::kMonitor || control_engaged_) {
      response->message =
        "refused: control is not in MONITOR; disable teleoperation or resolve the fault first";
    } else if (!sdk_control_compatibility_confirmed_) {
      response->message = "refused: SDK/controller control compatibility is not confirmed";
    } else if (!tool_dynamics_confirmed_) {
      response->message = "refused: tool dynamics configuration is not confirmed";
    } else if (enabled_arms_ == EnabledArms::kNone) {
      response->message = "refused: enabled_arms is none";
    } else if (!sdk_.connected() || !have_feedback_ ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_feedback_receive_).count() >
      feedback_timeout_sec_)
    {
      response->message = "refused: fresh Marvin feedback is unavailable";
    } else if (!robot_healthy_for_enabled_arms(latest_dcss_)) {
      response->message = "refused: an enabled arm reports an error";
    } else {
      impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
      clear_soft_limit_state();
      control_purpose_ = ControlPurpose::kInitialPose;
      operation_control_mode_ = JointControlMode::kJointImpedance;
      last_submitted_target_rad_.reset();
      initial_pose_stable_frames_ = 0;
      control_gate_.request_enable();
      response->success = true;
      response->message =
        "accepted: preparing joint impedance, then moving to the configured initial pose";
      return;
    }
    response->success = false;
  }

  static diagnostic_msgs::msg::KeyValue diagnostic_value(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue result;
    result.key = key;
    result.value = value;
    return result;
  }

  double displayed_alignment_tolerance() const
  {
    if (control_gate_.state() == ControlState::kSoftLimitHold) {
      return soft_limit_recovery_alignment_rad_;
    }
    return control_gate_.state() == ControlState::kCommandWait ?
           command_recovery_alignment_rad_ : initial_alignment_tolerance_rad_;
  }

  double displayed_alignment_margin() const
  {
    if (displayed_backup_limits()) {
      return 0.0;
    }
    return (control_gate_.state() == ControlState::kCommandWait ||
           control_gate_.state() == ControlState::kSoftLimitHold) ?
           soft_limit_release_margin_rad_ : joint_limit_safety_margin_rad_;
  }

  bool displayed_backup_limits() const
  {
    return !teleop_soft_limits_enabled_ &&
           control_purpose_ != ControlPurpose::kInitialPose &&
           control_purpose_ != ControlPurpose::kRetreatLeft &&
           control_purpose_ != ControlPurpose::kRetreatRight;
  }

  double displayed_limit_extension() const
  {
    return displayed_backup_limits() ? teleop_limit_extension() : 0.0;
  }

  std::size_t displayed_stable_frames() const
  {
    return control_gate_.state() == ControlState::kSoftLimitHold ?
           control_gate_.soft_limit_recovery_frames() : control_gate_.aligned_frames();
  }

  std::size_t displayed_required_frames() const
  {
    return control_gate_.state() == ControlState::kSoftLimitHold ?
           static_cast<std::size_t>(soft_limit_recovery_frames_) :
           static_cast<std::size_t>(alignment_required_frames_);
  }

  static std::string format_decimal(double value, int precision = 5)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  void publish_alignment_diagnostics_if_due(
    std::chrono::steady_clock::time_point now_steady)
  {
    if (std::chrono::duration<double>(
        now_steady - last_alignment_diagnostics_publish_).count() <
      1.0 / alignment_diagnostics_rate_hz_)
    {
      return;
    }
    last_alignment_diagnostics_publish_ = now_steady;

    JointArray command{};
    JointArray feedback{};
    std::optional<std::chrono::steady_clock::time_point> command_time;
    bool have_feedback = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      command = last_valid_command_rad_;
      feedback = latest_feedback_rad_;
      command_time = last_valid_command_time_;
      have_feedback = have_feedback_;
    }
    const bool command_fresh = command_time.has_value() &&
      std::chrono::duration<double>(now_steady - command_time.value()).count() <=
      command_timeout_sec_;
    const bool feedback_fresh = have_feedback && !feedback_recovery_frames_ && sdk_.connected() &&
      std::chrono::duration<double>(now_steady - last_feedback_receive_).count() <=
      feedback_timeout_sec_;
    const auto limit_result = inspect_joint_limit_margin(
      command, displayed_alignment_margin(), displayed_limit_extension());
    const auto enabled = enabled_arm_mask();

    std::size_t aligned_count = 0;
    std::size_t enabled_count = 0;
    double maximum_error = -1.0;
    std::size_t maximum_error_joint = 0;
    bool all_limits_safe = true;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (!enabled[index / kArmDof]) {
        continue;
      }
      ++enabled_count;
      const double error = std::abs(command[index] - feedback[index]);
      if (command_fresh && feedback_fresh && error > maximum_error) {
        maximum_error = error;
        maximum_error_joint = index;
      }
      const bool limit_safe = command_fresh && limit_result.valid &&
        limit_result.joint_safe[index];
      all_limits_safe = all_limits_safe && limit_safe;
      if (command_fresh && feedback_fresh && limit_safe &&
        error <= displayed_alignment_tolerance())
      {
        ++aligned_count;
      }
    }

    std::string block_reason = "ALIGNED";
    uint8_t summary_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (feedback_recovery_frames_) {
      block_reason = "WAITING_FOR_FEEDBACK";
      summary_level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else if (!command_fresh) {
      block_reason = "WAITING_FOR_COMMAND";
      summary_level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else if (!feedback_fresh) {
      block_reason = "WAITING_FOR_FEEDBACK";
      summary_level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    } else if (!all_limits_safe) {
      block_reason = "WAITING_FOR_SAFE_LIMIT";
      summary_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (aligned_count != enabled_count) {
      block_reason = "WAITING_FOR_ALIGNMENT";
      summary_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if ((control_gate_.state() == ControlState::kWaitingForAlignment ||
      control_gate_.state() == ControlState::kCommandWait ||
      control_gate_.state() == ControlState::kSoftLimitHold) &&
      displayed_stable_frames() < displayed_required_frames())
    {
      block_reason = "WAITING_FOR_STABLE_FRAMES";
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus summary;
    summary.level = summary_level;
    summary.name = "tianji_teleop/alignment_summary";
    summary.message = block_reason;
    summary.hardware_id = robot_ip_;
    summary.values.push_back(
      diagnostic_value(
        "control_state", control_state_name(control_gate_.state())));
    summary.values.push_back(diagnostic_value("enabled_arms", enabled_arms_text_));
    summary.values.push_back(diagnostic_value(
      "joint_limit_policy", displayed_backup_limits() ? "backup" : "soft"));
    summary.values.push_back(diagnostic_value("control_reason", control_gate_.reason()));
    summary.values.push_back(diagnostic_value("aligned_joints", std::to_string(aligned_count)));
    summary.values.push_back(diagnostic_value("enabled_joints", std::to_string(enabled_count)));
    summary.values.push_back(
      diagnostic_value(
        "aligned_frames", std::to_string(displayed_stable_frames())));
    summary.values.push_back(
      diagnostic_value(
        "required_frames", std::to_string(displayed_required_frames())));
    summary.values.push_back(
      diagnostic_value(
        "tolerance_rad", format_decimal(displayed_alignment_tolerance())));
    summary.values.push_back(
      diagnostic_value(
        "command_fresh", command_fresh ? "true" : "false"));
    summary.values.push_back(
      diagnostic_value(
        "feedback_fresh", feedback_fresh ? "true" : "false"));
    if (maximum_error >= 0.0) {
      summary.values.push_back(
        diagnostic_value(
          "maximum_error_joint", joint_names()[maximum_error_joint]));
      summary.values.push_back(
        diagnostic_value(
          "maximum_error_rad", format_decimal(maximum_error)));
      summary.values.push_back(
        diagnostic_value(
          "maximum_error_deg", format_decimal(
            tianji_io_teleop_bridge::radians_to_degrees(maximum_error), 2)));
    }
    array.status.push_back(summary);

    for (std::size_t index = 0; index < kJointCount; ++index) {
      diagnostic_msgs::msg::DiagnosticStatus joint_status;
      joint_status.name = "tianji_teleop/alignment/" + joint_names()[index];
      joint_status.hardware_id = robot_ip_;
      const bool joint_enabled = enabled[index / kArmDof];
      const double signed_error = command[index] - feedback[index];
      const double absolute_error = std::abs(signed_error);
      const bool limit_safe = limit_result.valid && limit_result.joint_safe[index];
      const bool aligned = joint_enabled && command_fresh && feedback_fresh && limit_safe &&
        absolute_error <= displayed_alignment_tolerance();

      if (!joint_enabled) {
        joint_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        joint_status.message = "DISABLED";
      } else if (!command_fresh || !feedback_fresh) {
        joint_status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        joint_status.message = !command_fresh ? "NO_FRESH_COMMAND" : "NO_FRESH_FEEDBACK";
      } else if (!limit_safe) {
        joint_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        joint_status.message = "OUTSIDE_SAFE_LIMIT";
      } else if (!aligned) {
        joint_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        joint_status.message = "NOT_ALIGNED";
      } else {
        joint_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        joint_status.message = "ALIGNED";
      }

      joint_status.values.push_back(diagnostic_value("joint_name", joint_names()[index]));
      joint_status.values.push_back(
        diagnostic_value(
          "enabled", joint_enabled ? "true" : "false"));
      joint_status.values.push_back(
        diagnostic_value(
          "aligned", aligned ? "true" : "false"));
      joint_status.values.push_back(
        diagnostic_value(
          "limit_safe", limit_safe ? "true" : "false"));
      joint_status.values.push_back(
        diagnostic_value(
          "command_rad", format_decimal(command[index])));
      joint_status.values.push_back(
        diagnostic_value(
          "feedback_rad", format_decimal(feedback[index])));
      joint_status.values.push_back(
        diagnostic_value(
          "signed_error_rad", format_decimal(signed_error)));
      joint_status.values.push_back(
        diagnostic_value(
          "absolute_error_rad", format_decimal(absolute_error)));
      joint_status.values.push_back(
        diagnostic_value(
          "command_deg", format_decimal(
            tianji_io_teleop_bridge::radians_to_degrees(command[index]), 2)));
      joint_status.values.push_back(
        diagnostic_value(
          "feedback_deg", format_decimal(
            tianji_io_teleop_bridge::radians_to_degrees(feedback[index]), 2)));
      joint_status.values.push_back(
        diagnostic_value(
          "absolute_error_deg", format_decimal(
            tianji_io_teleop_bridge::radians_to_degrees(absolute_error), 2)));
      joint_status.values.push_back(
        diagnostic_value(
          "tolerance_rad", format_decimal(displayed_alignment_tolerance())));
      joint_status.values.push_back(
        diagnostic_value(
          "safe_lower_rad",
          format_decimal(lower_limits_rad()[index] - displayed_limit_extension() + displayed_alignment_margin())));
      joint_status.values.push_back(
        diagnostic_value(
          "safe_upper_rad",
          format_decimal(upper_limits_rad()[index] + displayed_limit_extension() - displayed_alignment_margin())));
      // Explicit display-only ranges, sourced from the same model/parameters as control.
      joint_status.values.push_back(diagnostic_value(
          "hard_lower_rad", format_decimal(lower_limits_rad()[index])));
      joint_status.values.push_back(diagnostic_value(
          "hard_upper_rad", format_decimal(upper_limits_rad()[index])));
      joint_status.values.push_back(diagnostic_value(
          "joint_limit_policy", displayed_backup_limits() ? "backup" : "soft"));
      joint_status.values.push_back(diagnostic_value(
          "guard_lower_rad", format_decimal(lower_limits_rad()[index] - displayed_limit_extension())));
      joint_status.values.push_back(diagnostic_value(
          "guard_upper_rad", format_decimal(upper_limits_rad()[index] + displayed_limit_extension())));
      joint_status.values.push_back(diagnostic_value(
          "soft_lower_rad", format_decimal(lower_limits_rad()[index] - displayed_limit_extension() +
          (displayed_backup_limits() ? 0.0 : joint_limit_safety_margin_rad_))));
      joint_status.values.push_back(diagnostic_value(
          "soft_upper_rad", format_decimal(upper_limits_rad()[index] + displayed_limit_extension() -
          (displayed_backup_limits() ? 0.0 : joint_limit_safety_margin_rad_))));
      joint_status.values.push_back(diagnostic_value(
          "release_lower_rad", format_decimal(lower_limits_rad()[index] - displayed_limit_extension() +
          (displayed_backup_limits() ? 0.0 : soft_limit_release_margin_rad_))));
      joint_status.values.push_back(diagnostic_value(
          "release_upper_rad", format_decimal(upper_limits_rad()[index] + displayed_limit_extension() -
          (displayed_backup_limits() ? 0.0 : soft_limit_release_margin_rad_))));
      array.status.push_back(joint_status);
    }
    if (publish_right_ft_sensor_) {
      array.status.push_back(right_ft_sensor_diagnostic(now_steady));
    }
    alignment_diagnostics_publisher_->publish(array);
  }

  void publish_status_if_due(std::chrono::steady_clock::time_point now_steady)
  {
    if (std::chrono::duration<double>(now_steady - last_status_publish_).count() < 1.0) {
      return;
    }
    last_status_publish_ = now_steady;
    publish_torque_zero_status();
    double command_age = -1.0;
    double alignment_error = -1.0;
    std::string alignment_max_joint = "none";
    std::size_t aligned_joints = 0;
    std::size_t enabled_joints = 0;
    bool alignment_limits_safe = false;
    JointArray command{};
    JointArray feedback{};
    bool have_feedback = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      command = last_valid_command_rad_;
      feedback = latest_feedback_rad_;
      have_feedback = have_feedback_;
      if (last_valid_command_time_.has_value()) {
        command_age = std::chrono::duration<double>(
          now_steady - last_valid_command_time_.value()).count();
      }
    }
    const bool command_fresh = command_age >= 0.0 && command_age <= command_timeout_sec_;
    const bool feedback_fresh = have_feedback && !feedback_recovery_frames_ && sdk_.connected() &&
      std::chrono::duration<double>(now_steady - last_feedback_receive_).count() <=
      feedback_timeout_sec_;
    if (command_fresh && feedback_fresh && enabled_arms_ != EnabledArms::kNone) {
      const auto enabled = enabled_arm_mask();
      const auto limits = inspect_joint_limit_margin(
        command, displayed_alignment_margin(), displayed_limit_extension());
      alignment_limits_safe = enabled_arms_are_safe(limits);
      for (std::size_t index = 0; index < kJointCount; ++index) {
        if (!enabled[index / kArmDof]) {
          continue;
        }
        ++enabled_joints;
        const double error = std::abs(command[index] - feedback[index]);
        if (error > alignment_error) {
          alignment_error = error;
          alignment_max_joint = joint_names()[index];
        }
        if (limits.joint_safe[index] && error <= displayed_alignment_tolerance()) {
          ++aligned_joints;
        }
      }
    }
    std::string alignment_block_reason = "ALIGNED";
    if (feedback_recovery_frames_) {
      alignment_block_reason = "WAITING_FOR_FEEDBACK";
    } else if (!command_fresh) {
      alignment_block_reason = "WAITING_FOR_COMMAND";
    } else if (!feedback_fresh) {
      alignment_block_reason = "WAITING_FOR_FEEDBACK";
    } else if (!alignment_limits_safe) {
      alignment_block_reason = "WAITING_FOR_SAFE_LIMIT";
    } else if (aligned_joints != enabled_joints) {
      alignment_block_reason = "WAITING_FOR_ALIGNMENT";
    } else if ((control_gate_.state() == ControlState::kWaitingForAlignment ||
      control_gate_.state() == ControlState::kCommandWait ||
      control_gate_.state() == ControlState::kSoftLimitHold) &&
      displayed_stable_frames() < displayed_required_frames())
    {
      alignment_block_reason = "WAITING_FOR_STABLE_FRAMES";
    }
    std::ostringstream stream;
    stream << "state=" << control_state_name(control_gate_.state()) <<
      " reason=\"" << control_gate_.reason() << "\"" <<
      " motion_lock=" << (allow_motion_ ? "off" : "on") <<
      " enabled_arms=" << enabled_arms_text_ <<
      " control_arms=" << arm_mask_name(enabled_arm_mask()) <<
      " tool_config_pending=" << (tool_upload_ ? "true" : "false") <<
      " tool_config_uncertain=" << (tool_configuration_uncertain() ? "true" : "false") <<
      " soft_limit_frozen_arms=" << arm_mask_name(soft_limit_frozen_arms_) <<
      " soft_limit_recovery_frames=" << control_gate_.soft_limit_recovery_frames() <<
      " config_phase=" << impedance_phase_name(impedance_configuration_phase_) <<
      " control_purpose=" << control_purpose_name(control_purpose_) <<
      " teleop_control_mode=" << joint_control_mode_name(teleop_control_mode_) <<
      " teleop_soft_limits_enabled=" << (teleop_soft_limits_enabled_ ? "true" : "false") <<
      " teleop_joint_limit_extension_rad=" << teleop_joint_limit_extension_rad_ <<
      " operation_control_mode=" << joint_control_mode_name(operation_control_mode_) <<
      " actual_left_state=" << (have_feedback_ ? latest_dcss_.m_State[0].m_CurState : -1) <<
      " actual_right_state=" << (have_feedback_ ? latest_dcss_.m_State[1].m_CurState : -1) <<
      " actual_left_imp_type=" << (have_feedback_ ? latest_dcss_.m_In[0].m_ImpType : -1) <<
      " actual_right_imp_type=" << (have_feedback_ ? latest_dcss_.m_In[1].m_ImpType : -1) <<
      " actual_left_velocity_ratio=" << (have_feedback_ ? latest_dcss_.m_In[0].m_Joint_Vel_Ratio : -1) <<
      " actual_left_acceleration_ratio=" << (have_feedback_ ? latest_dcss_.m_In[0].m_Joint_Acc_Ratio : -1) <<
      " actual_right_velocity_ratio=" << (have_feedback_ ? latest_dcss_.m_In[1].m_Joint_Vel_Ratio : -1) <<
      " actual_right_acceleration_ratio=" << (have_feedback_ ? latest_dcss_.m_In[1].m_Joint_Acc_Ratio : -1) <<
      " initial_pose_stable_frames=" << initial_pose_stable_frames_ <<
      " sdk_connected=" << (sdk_.connected() ? "true" : "false") <<
      " aligned_frames=" << control_gate_.aligned_frames() <<
      " aligned_joints=" << aligned_joints << "/" << enabled_joints <<
      " alignment_block_reason=" << alignment_block_reason <<
      " alignment_max_joint=" << alignment_max_joint <<
      " alignment_error_rad=" << std::fixed << std::setprecision(4) << alignment_error <<
      " command_age_sec=" << std::fixed << std::setprecision(3) << command_age <<
      " accepted=" << accepted_command_count_ <<
      " ignored_non_arm=" << ignored_non_arm_count_ <<
      " rejected_arm=" << rejected_command_count_ <<
      " sdk_target_busy_timeout_sec=" << sdk_target_busy_timeout_sec_ <<
      " " << send_timing_.describe(std::chrono::steady_clock::now());
    std_msgs::msg::String status;
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  std::string robot_ip_;
  bool publish_right_ft_sensor_{false};
  std::string right_ft_sensor_topic_;
  std::string right_ft_sensor_frame_id_;
  std::string right_ft_calibration_file_, right_ft_tare_directory_;
  FtCompensation right_ft_compensation_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr right_ft_zero_service_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_ft_sensor_publisher_;
  std::optional<int32_t> right_ft_last_frame_;
  std::optional<std::chrono::steady_clock::time_point> right_ft_last_publish_;
  double right_ft_channel_{0.0};
  std::string right_ft_status_{"NOT_CONNECTED"};
  JointControlMode teleop_control_mode_{JointControlMode::kJointImpedance};
  // Latch for the entire operation, including HOLD/FAULT where purpose can be cleared.
  JointControlMode operation_control_mode_{JointControlMode::kJointImpedance};
  JointTorqueZero joint_torque_zero_;
  // Controller mode and tool kinematics/dynamics define the calibration context.
  std::array<std::optional<std::array<double, 17>>, 2> torque_zero_context_;
  std::array<int64_t, 2> torque_zero_stamp_{};
  std::string raw_state_topic_;
  std::string estimated_joint_disturbance_topic_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr estimated_joint_disturbance_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_bias_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr torque_zero_status_publisher_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> torque_zero_services_;
  std::string tools_config_file_;
  bool tool_config_dry_run_{true};
  std::array<bool, 2> tool_configuration_uncertain_arms_{};
  double tool_config_timeout_sec_{2.0};
  struct ToolUpload
  {
    std::size_t arm;
    ToolConfiguration config;
    std::chrono::steady_clock::time_point started;
    int32_t frame;
  };
  std::optional<ToolUpload> tool_upload_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tool_status_publisher_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, 2> tool_services_;
  std::string state_topic_;
  std::string command_topic_;
  std::string left_cartesian_disturbance_topic_;
  std::string right_cartesian_disturbance_topic_;
  std::string left_cartesian_disturbance_frame_id_;
  std::string right_cartesian_disturbance_frame_id_;
  std::string kinematics_config_file_;
  std::string alignment_diagnostics_topic_;
  std::string state_qos_reliability_;
  std::string command_qos_reliability_;
  std::string enabled_arms_text_;
  double publish_rate_hz_{100.0};
  double alignment_diagnostics_rate_hz_{10.0};
  double control_rate_hz_{100.0};
  double feedback_timeout_sec_{0.2};
  double command_timeout_sec_{0.15};
  double command_reference_timeout_sec_{0.2};
  double max_command_step_rad_{0.05};
  double initial_alignment_tolerance_rad_{0.05};
  double joint_limit_tolerance_rad_{1.0e-4};
  bool teleop_soft_limits_enabled_{false};
  double teleop_joint_limit_extension_rad_{0.02};
  double joint_limit_safety_margin_rad_{0.01};
  double soft_limit_release_margin_rad_{0.03};
  double soft_limit_recovery_alignment_rad_{0.10};
  double max_following_error_rad_{0.25};
  double retreat_duration_sec_{10.0}, retreat_period_sec_{0.01};
  double mode_transition_timeout_sec_{2.0};
  double sdk_packet_settle_sec_{0.25};
  double sdk_target_busy_timeout_sec_{0.1};
  double command_recovery_alignment_rad_{0.1};
  double hold_duration_sec_{0.5};
  double initial_pose_tolerance_rad_{0.02};
  double initial_pose_timeout_sec_{45.0};
  double reconnect_period_sec_{2.0};
  int alignment_required_frames_{20};
  int soft_limit_recovery_frames_{20};
  int initial_pose_required_frames_{20};
  int initial_pose_velocity_ratio_{10};
  int initial_pose_acceleration_ratio_{10};
  int64_t expected_sdk_version_{100343014};
  int state_qos_depth_{10};
  int command_qos_depth_{1};
  bool connect_on_start_{true};
  bool subscribe_joint_commands_{true};
  bool strict_sdk_version_{true};
  bool publish_cartesian_disturbance_{false};
  bool soft_limit_allow_other_arm_motion_{false};
  bool allow_motion_{false};
  bool sdk_control_compatibility_confirmed_{false};
  bool tool_dynamics_confirmed_{false};
  bool have_feedback_{false};
  bool frames_initialized_{false};
  bool control_engaged_{false};
  bool idle_retry_pending_{false};

  std::vector<double> left_stiffness_;
  std::vector<double> left_damping_;
  std::vector<double> right_stiffness_;
  std::vector<double> right_damping_;
  std::vector<double> initial_pose_deg_;
  EnabledArms enabled_arms_{EnabledArms::kNone};
  JointImpedanceSettings impedance_settings_;
  JointArray initial_pose_rad_{};
  RetreatCheck retreat_plan_;
  std::size_t retreat_arm_{1};
  std::optional<EnabledArms> retreat_control_arms_;
  bool retreat_completion_pending_{false};
  std::chrono::steady_clock::time_point retreat_started_{}, retreat_next_{};
  std::size_t retreat_index_{0};
  JointArray latest_feedback_rad_{};
  JointArray last_valid_command_rad_{};
  JointArray last_sent_target_rad_{};
  JointArray hold_target_rad_{};
  JointArray soft_limit_hold_target_rad_{};
  DCSS latest_dcss_{};

  std::array<bool, 2> frame_advanced_once_{};
  std::array<bool, 2> soft_limit_frozen_arms_{};
  std::array<int32_t, 2> last_frames_{};
  std::array<std::chrono::steady_clock::time_point, 2> last_feedback_advance_{};
  std::chrono::steady_clock::time_point next_connect_attempt_{};
  std::chrono::steady_clock::time_point transition_started_{};
  std::chrono::steady_clock::time_point hold_started_{};
  std::chrono::steady_clock::time_point initial_pose_started_{};
  CompletionPacer control_pacer_;
  SendTiming send_timing_;
  std::chrono::steady_clock::time_point last_control_update_{};
  std::chrono::steady_clock::time_point last_status_publish_{};
  std::chrono::steady_clock::time_point last_alignment_diagnostics_publish_{};
  std::chrono::steady_clock::time_point last_feedback_receive_{};
  std::chrono::steady_clock::time_point next_idle_retry_{};
  std::optional<std::chrono::steady_clock::time_point> last_valid_command_time_;

  ControlGate control_gate_;
  ImpedanceConfigurationPhase impedance_configuration_phase_{
    ImpedanceConfigurationPhase::kNone};
  ControlPurpose control_purpose_{ControlPurpose::kNone};
  TargetSendGuard target_send_guard_;
  std::optional<JointArray> last_submitted_target_rad_;
  std::optional<JointArray> command_wait_target_rad_;
  std::optional<std::array<int32_t, 2>> feedback_recovery_frames_;
  std::uint64_t last_recovery_command_count_{0};
  MarvinSdkAdapter sdk_;
  MarvinKinematicsAdapter kinematics_;
  std::mutex data_mutex_;
  std::uint64_t accepted_command_count_{0};
  std::uint64_t ignored_non_arm_count_{0};
  std::uint64_t rejected_command_count_{0};
  int idle_retry_attempts_{0};
  std::size_t initial_pose_stable_frames_{0};
  std::string soft_limit_trigger_detail_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    left_cartesian_disturbance_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
    right_cartesian_disturbance_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    alignment_diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr initial_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retreat_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retreat_left_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tianji_io_teleop_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tianji_io_teleop_bridge::MarvinTeleopBridgeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("tianji_marvin_teleop_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
