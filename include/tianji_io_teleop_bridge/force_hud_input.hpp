#ifndef TIANJI_IO_TELEOP_BRIDGE__FORCE_HUD_INPUT_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__FORCE_HUD_INPUT_HPP_

#include <cmath>
#include <string>
#include <unordered_map>

#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace tianji_io_teleop_bridge
{
// Telemetry validation is intentionally independent of command/enable gates.
inline bool parse_force_hud_input(
  const sensor_msgs::msg::JointState & msg, JointArray & q_deg, JointArray & tau,
  std::string & reason)
{
  if (msg.name.size() != kJointCount || msg.position.size() != kJointCount ||
    msg.effort.size() != kJointCount)
  {
    reason = "expected 14 names, positions and estimated efforts";
    return false;
  }
  std::unordered_map<std::string, std::size_t> indices;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!indices.emplace(msg.name[i], i).second) {
      reason = "duplicate joint name";
      return false;
    }
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto found = indices.find(joint_names()[i]);
    if (found == indices.end()) {
      reason = "missing joint: " + joint_names()[i];
      return false;
    }
    const auto j = found->second;
    if (!std::isfinite(msg.position[j]) || !std::isfinite(msg.effort[j]) ||
      std::abs(msg.position[j]) > 2.0 * kPi)
    {
      reason = "non-finite effort/position or implausible angle (expected rad)";
      return false;
    }
    q_deg[i] = radians_to_degrees(msg.position[j]);
    tau[i] = msg.effort[j];
  }
  reason.clear();
  return true;
}
}  // namespace tianji_io_teleop_bridge
#endif
