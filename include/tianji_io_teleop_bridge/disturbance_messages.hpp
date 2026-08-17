#ifndef TIANJI_IO_TELEOP_BRIDGE__DISTURBANCE_MESSAGES_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__DISTURBANCE_MESSAGES_HPP_

#include <array>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace tianji_io_teleop_bridge
{

using CartesianDisturbance = std::array<double, 6>;

inline sensor_msgs::msg::JointState make_joint_feedback_message(
  const JointArray & positions,
  const JointArray & velocities,
  const JointArray & joint_disturbance,
  const builtin_interfaces::msg::Time & stamp,
  bool include_effort = true)
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(joint_names().begin(), joint_names().end());
  message.position.assign(positions.begin(), positions.end());
  message.velocity.assign(velocities.begin(), velocities.end());
  if (include_effort) {
    message.effort.assign(joint_disturbance.begin(), joint_disturbance.end());
  }
  return message;
}

inline geometry_msgs::msg::WrenchStamped make_cartesian_disturbance_message(
  const CartesianDisturbance & cartesian_disturbance,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.wrench.force.x = cartesian_disturbance[0];
  message.wrench.force.y = cartesian_disturbance[1];
  message.wrench.force.z = cartesian_disturbance[2];
  message.wrench.torque.x = cartesian_disturbance[3];
  message.wrench.torque.y = cartesian_disturbance[4];
  message.wrench.torque.z = cartesian_disturbance[5];
  return message;
}

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__DISTURBANCE_MESSAGES_HPP_
