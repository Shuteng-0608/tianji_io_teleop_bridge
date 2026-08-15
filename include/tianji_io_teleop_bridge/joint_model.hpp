#ifndef TIANJI_IO_TELEOP_BRIDGE__JOINT_MODEL_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__JOINT_MODEL_HPP_

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>

#include "sensor_msgs/msg/joint_state.hpp"

namespace tianji_io_teleop_bridge
{

constexpr std::size_t kArmDof = 7;
constexpr std::size_t kJointCount = 14;
constexpr double kPi = 3.14159265358979323846;

using JointArray = std::array<double, kJointCount>;

inline const std::array<std::string, kJointCount> & joint_names()
{
  static const std::array<std::string, kJointCount> names = {
    "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L",
    "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R"
  };
  return names;
}

inline const JointArray & lower_limits_rad()
{
  static const JointArray limits = {
    -1.5708, -1.6760, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708,
    -3.1067, -1.6760, 0.0000, -2.5307, -3.1067, -1.0472, -1.5708
  };
  return limits;
}

inline const JointArray & upper_limits_rad()
{
  static const JointArray limits = {
    3.1067, 0.0000, 0.0000, 0.1000, 3.1067, 1.0472, 1.5708,
    1.5708, 0.0000, 3.1067, 0.1000, 3.1067, 1.0472, 1.5708
  };
  return limits;
}

inline double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

inline double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

struct CommandValidationResult
{
  bool valid{false};
  JointArray positions_rad{};
  std::string reason;
};

enum class CommandDisposition
{
  kIgnoredNonArm,
  kAccepted,
  kRejected
};

struct CommandEvaluationResult
{
  CommandDisposition disposition{CommandDisposition::kRejected};
  JointArray positions_rad{};
  std::string reason;
};

struct JointLimitMarginResult
{
  bool valid{false};
  std::array<bool, 2> arm_safe{};
  std::array<bool, kJointCount> joint_safe{};
  std::optional<std::size_t> first_violating_joint;
};

inline CommandValidationResult validate_joint_command(
  const sensor_msgs::msg::JointState & message,
  const std::optional<JointArray> & reference_rad,
  double max_delta_rad,
  double limit_tolerance_rad)
{
  CommandValidationResult result;

  if (message.name.size() != kJointCount) {
    result.reason = "expected exactly 14 joint names";
    return result;
  }
  if (message.position.size() != message.name.size()) {
    result.reason = "position array size must equal name array size";
    return result;
  }
  if (!std::isfinite(max_delta_rad) || max_delta_rad <= 0.0) {
    result.reason = "max_delta_rad must be finite and positive";
    return result;
  }
  if (!std::isfinite(limit_tolerance_rad) || limit_tolerance_rad < 0.0) {
    result.reason = "limit_tolerance_rad must be finite and non-negative";
    return result;
  }

  std::unordered_map<std::string, std::size_t> canonical_indices;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    canonical_indices.emplace(joint_names()[index], index);
  }

  std::array<bool, kJointCount> seen{};
  for (std::size_t input_index = 0; input_index < message.name.size(); ++input_index) {
    const auto canonical = canonical_indices.find(message.name[input_index]);
    if (canonical == canonical_indices.end()) {
      result.reason = "unknown joint name: " + message.name[input_index];
      return result;
    }

    const std::size_t output_index = canonical->second;
    if (seen[output_index]) {
      result.reason = "duplicate joint name: " + message.name[input_index];
      return result;
    }

    const double position = message.position[input_index];
    if (!std::isfinite(position)) {
      result.reason = "non-finite position for joint: " + message.name[input_index];
      return result;
    }

    result.positions_rad[output_index] = position;
    seen[output_index] = true;
  }

  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!seen[index]) {
      result.reason = "missing joint: " + joint_names()[index];
      return result;
    }

    const double position = result.positions_rad[index];
    if (
      position < lower_limits_rad()[index] - limit_tolerance_rad ||
      position > upper_limits_rad()[index] + limit_tolerance_rad)
    {
      std::ostringstream stream;
      stream << "joint limit violation for " << joint_names()[index] << ": " << position;
      result.reason = stream.str();
      return result;
    }

    if (reference_rad.has_value()) {
      const double delta = std::abs(position - reference_rad.value()[index]);
      if (delta > max_delta_rad) {
        std::ostringstream stream;
        stream << "command jump too large for " << joint_names()[index] << ": " << delta;
        result.reason = stream.str();
        return result;
      }
    }
  }

  result.valid = true;
  return result;
}

inline bool is_arm_joint_name(const std::string & name)
{
  return std::find(joint_names().begin(), joint_names().end(), name) != joint_names().end();
}

inline CommandEvaluationResult evaluate_joint_command(
  const sensor_msgs::msg::JointState & message,
  const std::optional<JointArray> & reference_rad,
  double max_delta_rad,
  double limit_tolerance_rad)
{
  CommandEvaluationResult result;
  const bool contains_arm_joint = std::any_of(
    message.name.begin(), message.name.end(),
    [](const std::string & name) {return is_arm_joint_name(name);});

  if (!contains_arm_joint) {
    result.disposition = CommandDisposition::kIgnoredNonArm;
    result.reason = "message contains no Tianji arm joints";
    return result;
  }

  const auto validation = validate_joint_command(
    message, reference_rad, max_delta_rad, limit_tolerance_rad);
  if (!validation.valid) {
    result.disposition = CommandDisposition::kRejected;
    result.reason = validation.reason;
    return result;
  }

  result.disposition = CommandDisposition::kAccepted;
  result.positions_rad = validation.positions_rad;
  return result;
}

inline double max_abs_delta(const JointArray & first, const JointArray & second)
{
  double maximum = 0.0;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    maximum = std::max(maximum, std::abs(first[index] - second[index]));
  }
  return maximum;
}

inline JointLimitMarginResult inspect_joint_limit_margin(
  const JointArray & positions, double margin_rad)
{
  JointLimitMarginResult result;
  if (!std::isfinite(margin_rad) || margin_rad < 0.0) {
    return result;
  }
  result.valid = true;
  result.arm_safe.fill(true);
  result.joint_safe.fill(true);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (
      positions[index] < lower_limits_rad()[index] + margin_rad ||
      positions[index] > upper_limits_rad()[index] - margin_rad)
    {
      result.arm_safe[index / kArmDof] = false;
      result.joint_safe[index] = false;
      if (!result.first_violating_joint.has_value()) {
        result.first_violating_joint = index;
      }
    }
  }
  return result;
}

inline bool within_safe_joint_limits(const JointArray & positions, double margin_rad)
{
  const auto result = inspect_joint_limit_margin(positions, margin_rad);
  return result.valid && result.arm_safe[0] && result.arm_safe[1];
}

class JointSlewLimiter
{
public:
  void reset(const JointArray & positions)
  {
    positions_ = positions;
    velocities_.fill(0.0);
    initialized_ = true;
  }

  JointArray update(
    const JointArray & desired,
    const JointArray & max_velocity_rad_s,
    const JointArray & max_acceleration_rad_s2,
    double dt_sec)
  {
    if (!initialized_) {
      reset(desired);
      return positions_;
    }
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      return positions_;
    }

    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double max_velocity = std::max(0.0, max_velocity_rad_s[index]);
      const double max_acceleration = std::max(0.0, max_acceleration_rad_s2[index]);
      const double error = desired[index] - positions_[index];
      const double requested_velocity = std::clamp(
        error / dt_sec, -max_velocity, max_velocity);
      const double velocity_delta = std::clamp(
        requested_velocity - velocities_[index],
        -max_acceleration * dt_sec,
        max_acceleration * dt_sec);
      const double next_velocity = std::clamp(
        velocities_[index] + velocity_delta, -max_velocity, max_velocity);
      const double step = next_velocity * dt_sec;

      if (std::abs(step) >= std::abs(error)) {
        positions_[index] = desired[index];
        velocities_[index] = 0.0;
      } else {
        positions_[index] += step;
        velocities_[index] = next_velocity;
      }
    }
    return positions_;
  }

private:
  bool initialized_{false};
  JointArray positions_{};
  JointArray velocities_{};
};

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__JOINT_MODEL_HPP_
