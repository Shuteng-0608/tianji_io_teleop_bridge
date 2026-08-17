#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tianji_io_teleop_bridge/control_gate.hpp"
#include "tianji_io_teleop_bridge/disturbance_messages.hpp"
#include "tianji_io_teleop_bridge/joint_model.hpp"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"
#include "tianji_io_teleop_bridge/marvin_sdk_adapter.hpp"

namespace tianji_io_teleop_bridge
{

enum class ImpedanceConfigurationPhase
{
  kNone,
  kWaitingForParameterEcho,
  kWaitingForModeEcho
};

inline const char * impedance_phase_name(ImpedanceConfigurationPhase phase)
{
  switch (phase) {
    case ImpedanceConfigurationPhase::kNone:
      return "NONE";
    case ImpedanceConfigurationPhase::kWaitingForParameterEcho:
      return "WAITING_FOR_PARAMETER_ECHO";
    case ImpedanceConfigurationPhase::kWaitingForModeEcho:
      return "WAITING_FOR_MODE_ECHO";
  }
  return "UNKNOWN";
}

class MarvinTeleopBridgeNode : public rclcpp::Node
{
public:
  MarvinTeleopBridgeNode()
  : Node("tianji_marvin_teleop_bridge"), control_gate_(20)
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
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.2);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.15);
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
    mode_transition_timeout_sec_ = declare_parameter<double>(
      "mode_transition_timeout_sec", 2.0);
    sdk_packet_settle_sec_ = declare_parameter<double>("sdk_packet_settle_sec", 0.25);
    hold_duration_sec_ = declare_parameter<double>("hold_duration_sec", 0.5);
    reconnect_period_sec_ = declare_parameter<double>("reconnect_period_sec", 2.0);
    alignment_required_frames_ = declare_parameter<int>("alignment_required_frames", 20);
    soft_limit_recovery_frames_ = declare_parameter<int>("soft_limit_recovery_frames", 20);
    expected_sdk_version_ = declare_parameter<int64_t>("expected_sdk_version", 100343013);
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
    max_velocity_ = declare_parameter<std::vector<double>>(
      "max_command_velocity_rad_s", std::vector<double>(kJointCount, 0.25));
    max_acceleration_ = declare_parameter<std::vector<double>>(
      "max_command_acceleration_rad_s2", std::vector<double>(kJointCount, 0.50));

    state_qos_reliability_ = declare_parameter<std::string>(
      "state_qos_reliability", "reliable");
    command_qos_reliability_ = declare_parameter<std::string>(
      "command_qos_reliability", "best_effort");
    state_qos_depth_ = declare_parameter<int>("state_qos_depth", 10);
    command_qos_depth_ = declare_parameter<int>("command_qos_depth", 1);

    validate_parameters();
    validate_sdk();
    if (publish_cartesian_disturbance_) {
      initialize_kinematics();
    }
    enabled_arms_ = parse_enabled_arms(enabled_arms_text_);
    copy_control_arrays();
    control_gate_ = ControlGate(
      static_cast<std::size_t>(alignment_required_frames_),
      static_cast<std::size_t>(soft_limit_recovery_frames_));

    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      state_topic_, make_qos(state_qos_reliability_, state_qos_depth_));
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
      get_logger(), "Publishing %s at %.1f Hz; monitoring %s with latest-sample QoS",
      state_topic_.c_str(), publish_rate_hz_, command_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Publishing Marvin estimated joint disturbance in %s effort",
      state_topic_.c_str());
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
    if (publish_rate_hz_ > 200.0) {
      throw std::invalid_argument("publish_rate_hz must not exceed 200");
    }
    validate_positive_finite(
      alignment_diagnostics_rate_hz_, "alignment_diagnostics_rate_hz");
    if (alignment_diagnostics_rate_hz_ > publish_rate_hz_) {
      throw std::invalid_argument(
              "alignment_diagnostics_rate_hz must not exceed publish_rate_hz");
    }
    validate_positive_finite(feedback_timeout_sec_, "feedback_timeout_sec");
    validate_positive_finite(command_timeout_sec_, "command_timeout_sec");
    validate_positive_finite(command_reference_timeout_sec_, "command_reference_timeout_sec");
    validate_positive_finite(max_command_step_rad_, "max_command_step_rad");
    validate_positive_finite(
      initial_alignment_tolerance_rad_, "initial_alignment_tolerance_rad");
    validate_positive_finite(
      soft_limit_release_margin_rad_, "soft_limit_release_margin_rad");
    validate_positive_finite(
      soft_limit_recovery_alignment_rad_, "soft_limit_recovery_alignment_rad");
    validate_positive_finite(max_following_error_rad_, "max_following_error_rad");
    validate_positive_finite(mode_transition_timeout_sec_, "mode_transition_timeout_sec");
    validate_positive_finite(sdk_packet_settle_sec_, "sdk_packet_settle_sec");
    validate_positive_finite(hold_duration_sec_, "hold_duration_sec");
    validate_positive_finite(reconnect_period_sec_, "reconnect_period_sec");
    if (!std::isfinite(joint_limit_tolerance_rad_) || joint_limit_tolerance_rad_ < 0.0 ||
      !std::isfinite(joint_limit_safety_margin_rad_) || joint_limit_safety_margin_rad_ < 0.0)
    {
      throw std::invalid_argument("joint limit tolerance and margin must be finite and non-negative");
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
    if (impedance_settings_.velocity_ratio < 1 || impedance_settings_.velocity_ratio > 100 ||
      impedance_settings_.acceleration_ratio < 1 ||
      impedance_settings_.acceleration_ratio > 100)
    {
      throw std::invalid_argument("velocity_ratio and acceleration_ratio must be in [1, 100]");
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
    validate_vector(left_stiffness_, kArmDof, 0.0, 12.0, "left_joint_stiffness");
    validate_vector(right_stiffness_, kArmDof, 0.0, 12.0, "right_joint_stiffness");
    validate_vector(left_damping_, kArmDof, 0.0, 1.0, "left_joint_damping");
    validate_vector(right_damping_, kArmDof, 0.0, 1.0, "right_joint_damping");
    validate_vector(max_velocity_, kJointCount, 1.0e-6, 10.0, "max_command_velocity_rad_s");
    validate_vector(
      max_acceleration_, kJointCount, 1.0e-6, 50.0,
      "max_command_acceleration_rad_s2");
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
    std::copy(max_velocity_.begin(), max_velocity_.end(), max_velocity_rad_s_.begin());
    std::copy(
      max_acceleration_.begin(), max_acceleration_.end(), max_acceleration_rad_s2_.begin());
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
    frames_initialized_ = false;
    frame_advanced_once_.fill(false);
    last_feedback_advance_.fill(std::chrono::steady_clock::now());
    control_engaged_ = false;
    idle_retry_pending_ = false;
    impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
    clear_soft_limit_state();
    control_gate_.disarm("SDK connected; explicit enable required");
    RCLCPP_INFO(get_logger(), "Marvin SDK connection established");
    return true;
  }

  void disconnect_robot()
  {
    if (!sdk_.connected()) {
      return;
    }
    if (allow_motion_ && control_engaged_) {
      if (!sdk_.request_idle(enabled_arms_)) {
        RCLCPP_ERROR(
          get_logger(), "Best-effort idle request failed during shutdown: %s",
          sdk_.last_error().c_str());
      }
      control_engaged_ = false;
    }
    if (!sdk_.disconnect()) {
      RCLCPP_ERROR(get_logger(), "Marvin SDK release failed: %s", sdk_.last_error().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Marvin SDK connection released");
    }
  }

  void on_timer()
  {
    const auto now_steady = std::chrono::steady_clock::now();
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
    if (!sdk_.read_feedback(feedback)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Marvin feedback read failed: %s",
        sdk_.last_error().c_str());
      if (control_engaged_) {
        fault_and_idle("feedback read failed");
      }
      publish_status_if_due(now_steady);
      return;
    }
    if (!feedback_is_finite(feedback)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Rejected non-finite Marvin joint feedback");
      if (control_engaged_) {
        fault_and_idle("non-finite feedback");
      }
      publish_status_if_due(now_steady);
      return;
    }
    if (!feedback_frames_are_fresh(feedback, now_steady)) {
      if (control_engaged_) {
        fault_and_idle("feedback frame timeout");
      }
      publish_status_if_due(now_steady);
      return;
    }

    latest_dcss_ = feedback;
    last_feedback_receive_ = now_steady;
    report_robot_errors(feedback);
    const auto feedback_stamp = now();
    const JointArray feedback_rad = publish_feedback(feedback, feedback_stamp);
    update_control(feedback, feedback_rad, now_steady);
    if (publish_cartesian_disturbance_) {
      // Keep the existing control path ahead of the optional telemetry serialization.
      publish_cartesian_disturbance_estimates(feedback, feedback_stamp);
    }
    publish_status_if_due(now_steady);
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

  JointArray publish_feedback(const DCSS & feedback, const rclcpp::Time & stamp)
  {
    JointArray positions_rad{};
    JointArray velocities_rad_s{};
    JointArray joint_disturbance{};
    bool joint_disturbance_finite = true;
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      positions_rad[joint] = degrees_to_radians(feedback.m_Out[0].m_FB_Joint_Pos[joint]);
      positions_rad[kArmDof + joint] = degrees_to_radians(
        feedback.m_Out[1].m_FB_Joint_Pos[joint]);
      velocities_rad_s[joint] = degrees_to_radians(
        feedback.m_Out[0].m_FB_Joint_Vel[joint]);
      velocities_rad_s[kArmDof + joint] = degrees_to_radians(
        feedback.m_Out[1].m_FB_Joint_Vel[joint]);
      joint_disturbance[joint] = feedback.m_Out[0].m_EST_Joint_Force[joint];
      joint_disturbance[kArmDof + joint] = feedback.m_Out[1].m_EST_Joint_Force[joint];
      joint_disturbance_finite = joint_disturbance_finite &&
        std::isfinite(joint_disturbance[joint]) &&
        std::isfinite(joint_disturbance[kArmDof + joint]);
    }
    const auto message = make_joint_feedback_message(
      positions_rad, velocities_rad_s, joint_disturbance, stamp, joint_disturbance_finite);
    if (!joint_disturbance_finite) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Published Marvin joint state without effort because estimated disturbance was "
        "non-finite");
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_feedback_rad_ = positions_rad;
      have_feedback_ = true;
    }
    state_publisher_->publish(message);
    return positions_rad;
  }

  void publish_cartesian_disturbance_estimates(
    const DCSS & feedback, const rclcpp::Time & stamp)
  {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      std::array<double, kArmDof> positions_deg{};
      std::array<double, kArmDof> arm_joint_disturbance{};
      for (std::size_t joint = 0; joint < kArmDof; ++joint) {
        positions_deg[joint] = feedback.m_Out[arm].m_FB_Joint_Pos[joint];
        arm_joint_disturbance[joint] = feedback.m_Out[arm].m_EST_Joint_Force[joint];
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

  bool robot_healthy_for_enabled_arms(const DCSS & feedback) const
  {
    const auto healthy = [&feedback](std::size_t arm) {
        return feedback.m_State[arm].m_CurState != ARM_STATE_ERROR &&
               feedback.m_State[arm].m_ERRCode == 0;
      };
    return (!left_enabled(enabled_arms_) || healthy(0)) &&
           (!right_enabled(enabled_arms_) || healthy(1));
  }

  bool impedance_parameters_verified(const DCSS & feedback) const
  {
    const auto verified = [this, &feedback](std::size_t arm) {
        const auto & expected_k = arm == 0 ?
          impedance_settings_.left_stiffness : impedance_settings_.right_stiffness;
        const auto & expected_d = arm == 0 ?
          impedance_settings_.left_damping : impedance_settings_.right_damping;
        if (feedback.m_In[arm].m_Joint_Vel_Ratio != impedance_settings_.velocity_ratio ||
          feedback.m_In[arm].m_Joint_Acc_Ratio != impedance_settings_.acceleration_ratio)
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
    return (!left_enabled(enabled_arms_) || verified(0)) &&
           (!right_enabled(enabled_arms_) || verified(1));
  }

  bool impedance_mode_verified(const DCSS & feedback) const
  {
    const auto mode_verified = [&feedback](std::size_t arm) {
        return feedback.m_State[arm].m_CurState == ARM_STATE_TORQ &&
               feedback.m_In[arm].m_ImpType == ARM_IMP_JOINT;
      };
    return impedance_parameters_verified(feedback) &&
           (!left_enabled(enabled_arms_) || mode_verified(0)) &&
           (!right_enabled(enabled_arms_) || mode_verified(1));
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
    if (left_enabled(enabled_arms_)) {
      inspect_arm(0);
    }
    if (right_enabled(enabled_arms_)) {
      inspect_arm(kArmDof);
    }
    return maximum;
  }

  std::array<bool, 2> enabled_arm_mask() const
  {
    return {left_enabled(enabled_arms_), right_enabled(enabled_arms_)};
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
           (!left_enabled(enabled_arms_) || result.arm_safe[0]) &&
           (!right_enabled(enabled_arms_) || result.arm_safe[1]);
  }

  bool enabled_targets_within_margin(const JointArray & positions, double margin_rad) const
  {
    return enabled_arms_are_safe(inspect_joint_limit_margin(positions, margin_rad));
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
      if ((arm == 0 && !left_enabled(enabled_arms_)) ||
        (arm == 1 && !right_enabled(enabled_arms_)) || result.joint_safe[index])
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
    if (!left_enabled(enabled_arms_)) {
      std::copy_n(feedback.begin(), kArmDof, enabled_command.begin());
    }
    if (!right_enabled(enabled_arms_)) {
      std::copy_n(
        feedback.begin() + kArmDof, kArmDof, enabled_command.begin() + kArmDof);
    }
    return enabled_command;
  }

  bool send_slew_limited_target(
    const JointArray & desired, std::chrono::steady_clock::time_point now_steady)
  {
    double dt_sec = 1.0 / publish_rate_hz_;
    if (last_control_update_.time_since_epoch().count() != 0) {
      dt_sec = std::clamp(
        std::chrono::duration<double>(now_steady - last_control_update_).count(),
        1.0 / (publish_rate_hz_ * 2.0), 2.0 / publish_rate_hz_);
    }
    last_control_update_ = now_steady;
    const JointArray limited = slew_limiter_.update(
      desired, max_velocity_rad_s_, max_acceleration_rad_s2_, dt_sec);
    if (!sdk_.send_joint_targets(enabled_arms_, radians_to_degrees(limited))) {
      fault_and_idle("joint target send failed: " + sdk_.last_error());
      return false;
    }
    last_sent_target_rad_ = limited;
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
    slew_limiter_.reset(soft_limit_hold_target_rad_);
    last_sent_target_rad_ = soft_limit_hold_target_rad_;
    soft_limit_trigger_detail_ = describe_limit_violations(
      command, limit_result, joint_limit_safety_margin_rad_);
    control_gate_.enter_soft_limit_hold(
      "soft joint limit; frozen arms=" + arm_mask_name(soft_limit_frozen_arms_) +
      "; " + soft_limit_trigger_detail_);
    RCLCPP_WARN(
      get_logger(),
      "Entering SOFT_LIMIT_HOLD without leaving impedance control: frozen arms=%s; %s",
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
    slew_limiter_.reset(reset_target);
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
      if (!robot_healthy) {
        return;
      }
      if (impedance_configuration_phase_ == ImpedanceConfigurationPhase::kNone) {
        if (!sdk_.configure_joint_impedance_parameters(enabled_arms_, impedance_settings_)) {
          fault_and_idle("impedance parameter configuration failed: " + sdk_.last_error());
          return;
        }
        transition_started_ = now_steady;
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kWaitingForParameterEcho;
        RCLCPP_WARN(
          get_logger(),
          "Impedance parameter packet sent; robot control mode is unchanged");
        return;
      }
      const double phase_age = std::chrono::duration<double>(
        now_steady - transition_started_).count();
      if (phase_age > mode_transition_timeout_sec_) {
        fault_and_idle("timed out waiting for impedance parameter confirmation");
        return;
      }
      if (phase_age >= sdk_packet_settle_sec_ && impedance_parameters_verified(feedback)) {
        // Switching control mode can change robot behavior even though this packet contains
        // no position target. Mark it engaged first so every partial failure requests idle.
        control_engaged_ = true;
        if (!sdk_.enter_joint_impedance_mode(enabled_arms_)) {
          fault_and_idle("impedance mode request failed: " + sdk_.last_error());
          return;
        }
        transition_started_ = now_steady;
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kWaitingForModeEcho;
        control_gate_.mark_configuring();
        RCLCPP_WARN(
          get_logger(),
          "Impedance parameters confirmed; pure mode-switch packet sent without a joint target");
      }
      return;
    }

    if (state == ControlState::kWaitingForAlignment) {
      if (!robot_healthy) {
        begin_hold("robot error while waiting for alignment in impedance mode", feedback_rad,
          now_steady);
        return;
      }
      // Explicit enable authorizes impedance preparation independently of TeleBox availability.
      // A missing/stale command simply keeps this state unaligned; no target is sent.
      const bool aligned = command_fresh &&
        enabled_targets_within_margin(command, joint_limit_safety_margin_rad_) &&
        max_enabled_delta(command, feedback_rad) <= initial_alignment_tolerance_rad_;
      if (!control_gate_.observe_alignment(aligned)) {
        return;
      }

      clear_soft_limit_state();
      // No target has been sent since the mode switch. Initialize the limiter from the latest
      // measured pose so the first teleoperation target remains velocity/acceleration limited.
      slew_limiter_.reset(feedback_rad);
      last_sent_target_rad_ = feedback_rad;
      last_control_update_ = now_steady;
      control_gate_.mark_active();
      RCLCPP_WARN(
        get_logger(),
        "Alignment confirmed in joint impedance mode; teleoperation control is ACTIVE");
      return;
    }

    if (state == ControlState::kConfiguringImpedance) {
      if (!robot_healthy) {
        begin_hold("robot error while entering impedance mode", feedback_rad, now_steady);
        return;
      }
      const double phase_age = std::chrono::duration<double>(
        now_steady - transition_started_).count();
      if (impedance_configuration_phase_ ==
        ImpedanceConfigurationPhase::kWaitingForModeEcho && impedance_mode_verified(feedback))
      {
        impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
        control_gate_.mark_waiting_for_alignment();
        RCLCPP_WARN(
          get_logger(),
          "Joint impedance mode confirmed; no joint target sent; waiting for alignment");
        return;
      }
      if (phase_age > mode_transition_timeout_sec_) {
        fault_and_idle("timed out waiting for joint impedance mode confirmation");
      }
      return;
    }

    if (state == ControlState::kActive) {
      if (!robot_healthy) {
        begin_hold("robot reported an error", feedback_rad, now_steady);
        return;
      }
      if (!command_fresh) {
        begin_hold("valid arm command timed out", feedback_rad, now_steady);
        return;
      }
      if (max_enabled_delta(last_sent_target_rad_, feedback_rad) > max_following_error_rad_) {
        begin_hold("following error exceeded safety threshold", feedback_rad, now_steady);
        return;
      }
      const auto limit_result = inspect_joint_limit_margin(
        command, joint_limit_safety_margin_rad_);
      if (!enabled_arms_are_safe(limit_result)) {
        enter_soft_limit_hold(command, feedback_rad, limit_result);
        return;
      }

      const JointArray desired = hold_disabled_arms_at_feedback(command, feedback_rad);
      (void)send_slew_limited_target(desired, now_steady);
      return;
    }

    if (state == ControlState::kSoftLimitHold) {
      if (!robot_healthy) {
        begin_hold("robot reported an error during soft-limit hold", feedback_rad, now_steady);
        return;
      }
      if (!command_fresh) {
        begin_hold("valid arm command timed out during soft-limit hold", feedback_rad, now_steady);
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
        slew_limiter_.reset(last_sent_target_rad_);
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
      (void)send_slew_limited_target(desired, now_steady);
      return;
    }

    if (state == ControlState::kHold) {
      if (!control_engaged_) {
        control_gate_.disarm("hold completed; explicit re-enable required");
        return;
      }
      const double hold_age = std::chrono::duration<double>(now_steady - hold_started_).count();
      if (robot_healthy && hold_age < hold_duration_sec_) {
        if (!sdk_.send_joint_targets(enabled_arms_, radians_to_degrees(hold_target_rad_))) {
          fault_and_idle("failed to send hold target: " + sdk_.last_error());
        }
        return;
      }
      if (!sdk_.request_idle(enabled_arms_)) {
        control_gate_.enter_fault("idle request failed after hold: " + sdk_.last_error());
        RCLCPP_ERROR(get_logger(), "%s", control_gate_.reason().c_str());
        schedule_idle_retry();
        return;
      } else {
        control_gate_.disarm("hold completed; explicit re-enable required");
      }
      control_engaged_ = false;
    }
  }

  void begin_hold(
    const std::string & reason, const JointArray & feedback_rad,
    std::chrono::steady_clock::time_point now_steady)
  {
    clear_soft_limit_state();
    hold_target_rad_ = feedback_rad;
    hold_started_ = now_steady;
    control_gate_.enter_hold(reason);
    RCLCPP_ERROR(get_logger(), "Entering HOLD: %s", reason.c_str());
  }

  void fault_and_idle(const std::string & reason)
  {
    clear_soft_limit_state();
    control_gate_.enter_fault(reason);
    impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
    RCLCPP_ERROR(get_logger(), "Control fault: %s", reason.c_str());
    if (allow_motion_ && control_engaged_ && sdk_.connected() && !idle_retry_pending_) {
      if (!sdk_.request_idle(enabled_arms_)) {
        RCLCPP_ERROR(
          get_logger(), "Best-effort idle request also failed: %s", sdk_.last_error().c_str());
        schedule_idle_retry();
      } else {
        control_engaged_ = false;
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
    if (sdk_.request_idle(enabled_arms_)) {
      idle_retry_pending_ = false;
      control_engaged_ = false;
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
      *message, reference, max_command_step_rad_, joint_limit_tolerance_rad_);
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
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_valid_command_rad_ = evaluation.positions_rad;
      last_valid_command_time_ = now_steady;
    }
    ++accepted_command_count_;
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
        control_gate_.disarm("operator requested disable");
        response->message = "control disabled";
      }
      response->success = true;
      return;
    }

    if (!allow_motion_) {
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
    } else {
      impedance_configuration_phase_ = ImpedanceConfigurationPhase::kNone;
      clear_soft_limit_state();
      control_gate_.request_enable();
      response->success = true;
      response->message =
        "enable accepted; preparing parameters, then switching mode before alignment";
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
    const bool feedback_fresh = have_feedback && sdk_.connected() &&
      std::chrono::duration<double>(now_steady - last_feedback_receive_).count() <=
      feedback_timeout_sec_;
    const auto limit_result = inspect_joint_limit_margin(
      command, joint_limit_safety_margin_rad_);
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
        error <= initial_alignment_tolerance_rad_)
      {
        ++aligned_count;
      }
    }

    std::string block_reason = "ALIGNED";
    uint8_t summary_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (!command_fresh) {
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
    } else if (control_gate_.state() == ControlState::kWaitingForAlignment &&
      control_gate_.aligned_frames() < static_cast<std::size_t>(alignment_required_frames_))
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
    summary.values.push_back(diagnostic_value("aligned_joints", std::to_string(aligned_count)));
    summary.values.push_back(diagnostic_value("enabled_joints", std::to_string(enabled_count)));
    summary.values.push_back(
      diagnostic_value(
        "aligned_frames", std::to_string(control_gate_.aligned_frames())));
    summary.values.push_back(
      diagnostic_value(
        "required_frames", std::to_string(alignment_required_frames_)));
    summary.values.push_back(
      diagnostic_value(
        "tolerance_rad", format_decimal(initial_alignment_tolerance_rad_)));
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
        absolute_error <= initial_alignment_tolerance_rad_;

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
          "tolerance_rad", format_decimal(initial_alignment_tolerance_rad_)));
      joint_status.values.push_back(
        diagnostic_value(
          "safe_lower_rad",
          format_decimal(lower_limits_rad()[index] + joint_limit_safety_margin_rad_)));
      joint_status.values.push_back(
        diagnostic_value(
          "safe_upper_rad",
          format_decimal(upper_limits_rad()[index] - joint_limit_safety_margin_rad_)));
      array.status.push_back(joint_status);
    }
    alignment_diagnostics_publisher_->publish(array);
  }

  void publish_status_if_due(std::chrono::steady_clock::time_point now_steady)
  {
    if (std::chrono::duration<double>(now_steady - last_status_publish_).count() < 1.0) {
      return;
    }
    last_status_publish_ = now_steady;
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
    const bool feedback_fresh = have_feedback && sdk_.connected() &&
      std::chrono::duration<double>(now_steady - last_feedback_receive_).count() <=
      feedback_timeout_sec_;
    if (command_fresh && feedback_fresh && enabled_arms_ != EnabledArms::kNone) {
      const auto enabled = enabled_arm_mask();
      const auto limits = inspect_joint_limit_margin(command, joint_limit_safety_margin_rad_);
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
        if (limits.joint_safe[index] && error <= initial_alignment_tolerance_rad_) {
          ++aligned_joints;
        }
      }
    }
    std::string alignment_block_reason = "ALIGNED";
    if (!command_fresh) {
      alignment_block_reason = "WAITING_FOR_COMMAND";
    } else if (!feedback_fresh) {
      alignment_block_reason = "WAITING_FOR_FEEDBACK";
    } else if (!alignment_limits_safe) {
      alignment_block_reason = "WAITING_FOR_SAFE_LIMIT";
    } else if (aligned_joints != enabled_joints) {
      alignment_block_reason = "WAITING_FOR_ALIGNMENT";
    } else if (control_gate_.state() == ControlState::kWaitingForAlignment &&
      control_gate_.aligned_frames() < static_cast<std::size_t>(alignment_required_frames_))
    {
      alignment_block_reason = "WAITING_FOR_STABLE_FRAMES";
    }
    std::ostringstream stream;
    stream << "state=" << control_state_name(control_gate_.state()) <<
      " reason=\"" << control_gate_.reason() << "\"" <<
      " motion_lock=" << (allow_motion_ ? "off" : "on") <<
      " enabled_arms=" << enabled_arms_text_ <<
      " soft_limit_frozen_arms=" << arm_mask_name(soft_limit_frozen_arms_) <<
      " soft_limit_recovery_frames=" << control_gate_.soft_limit_recovery_frames() <<
      " config_phase=" << impedance_phase_name(impedance_configuration_phase_) <<
      " sdk_connected=" << (sdk_.connected() ? "true" : "false") <<
      " aligned_frames=" << control_gate_.aligned_frames() <<
      " aligned_joints=" << aligned_joints << "/" << enabled_joints <<
      " alignment_block_reason=" << alignment_block_reason <<
      " alignment_max_joint=" << alignment_max_joint <<
      " alignment_error_rad=" << std::fixed << std::setprecision(4) << alignment_error <<
      " command_age_sec=" << std::fixed << std::setprecision(3) << command_age <<
      " accepted=" << accepted_command_count_ <<
      " ignored_non_arm=" << ignored_non_arm_count_ <<
      " rejected_arm=" << rejected_command_count_;
    std_msgs::msg::String status;
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  std::string robot_ip_;
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
  double feedback_timeout_sec_{0.2};
  double command_timeout_sec_{0.15};
  double command_reference_timeout_sec_{0.2};
  double max_command_step_rad_{0.05};
  double initial_alignment_tolerance_rad_{0.05};
  double joint_limit_tolerance_rad_{1.0e-4};
  double joint_limit_safety_margin_rad_{0.01};
  double soft_limit_release_margin_rad_{0.03};
  double soft_limit_recovery_alignment_rad_{0.10};
  double max_following_error_rad_{0.25};
  double mode_transition_timeout_sec_{2.0};
  double sdk_packet_settle_sec_{0.25};
  double hold_duration_sec_{0.5};
  double reconnect_period_sec_{2.0};
  int alignment_required_frames_{20};
  int soft_limit_recovery_frames_{20};
  int64_t expected_sdk_version_{100343013};
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
  std::vector<double> max_velocity_;
  std::vector<double> max_acceleration_;
  EnabledArms enabled_arms_{EnabledArms::kNone};
  JointImpedanceSettings impedance_settings_;
  JointArray max_velocity_rad_s_{};
  JointArray max_acceleration_rad_s2_{};
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
  std::chrono::steady_clock::time_point last_control_update_{};
  std::chrono::steady_clock::time_point last_status_publish_{};
  std::chrono::steady_clock::time_point last_alignment_diagnostics_publish_{};
  std::chrono::steady_clock::time_point last_feedback_receive_{};
  std::chrono::steady_clock::time_point next_idle_retry_{};
  std::optional<std::chrono::steady_clock::time_point> last_valid_command_time_;

  ControlGate control_gate_;
  ImpedanceConfigurationPhase impedance_configuration_phase_{
    ImpedanceConfigurationPhase::kNone};
  JointSlewLimiter slew_limiter_;
  MarvinSdkAdapter sdk_;
  MarvinKinematicsAdapter kinematics_;
  std::mutex data_mutex_;
  std::uint64_t accepted_command_count_{0};
  std::uint64_t ignored_non_arm_count_{0};
  std::uint64_t rejected_command_count_{0};
  int idle_retry_attempts_{0};
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
