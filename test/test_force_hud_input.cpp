#include <algorithm>
#include <limits>
#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/force_hud_input.hpp"

namespace bridge = tianji_io_teleop_bridge;

static sensor_msgs::msg::JointState sample()
{
  sensor_msgs::msg::JointState result;
  result.name.assign(bridge::joint_names().begin(), bridge::joint_names().end());
  for (std::size_t i = 0; i < bridge::kJointCount; ++i) {
    result.position.push_back(bridge::degrees_to_radians(i + 1));
    result.effort.push_back(100.0 + i);
  }
  return result;
}

TEST(ForceHudInput, ReordersBothPositionAndEffortByName)
{
  auto msg = sample();
  std::reverse(msg.name.begin(), msg.name.end());
  std::reverse(msg.position.begin(), msg.position.end());
  std::reverse(msg.effort.begin(), msg.effort.end());
  bridge::JointArray q{}, tau{};
  std::string reason;
  ASSERT_TRUE(bridge::parse_force_hud_input(msg, q, tau, reason));
  for (std::size_t i = 0; i < bridge::kJointCount; ++i) {
    EXPECT_NEAR(q[i], i + 1, 1e-10);
    EXPECT_EQ(tau[i], 100.0 + i);
  }
}

TEST(ForceHudInput, RejectsMissingEffortAndBadNames)
{
  bridge::JointArray q{}, tau{};
  std::string reason;
  auto msg = sample();
  msg.effort.clear();
  EXPECT_FALSE(bridge::parse_force_hud_input(msg, q, tau, reason));
  msg = sample();
  msg.name[7] = msg.name[0];
  EXPECT_FALSE(bridge::parse_force_hud_input(msg, q, tau, reason));
  msg = sample();
  msg.name[7] = "lh_FFJ4";
  EXPECT_FALSE(bridge::parse_force_hud_input(msg, q, tau, reason));
  msg = sample();
  msg.position.pop_back();
  EXPECT_FALSE(bridge::parse_force_hud_input(msg, q, tau, reason));
}

TEST(ForceHudInput, RejectsNonFiniteAndDegreesMistakenForRadians)
{
  bridge::JointArray q{}, tau{};
  std::string reason;
  auto msg = sample();
  msg.effort[13] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(bridge::parse_force_hud_input(msg, q, tau, reason));
  msg = sample();
  msg.position[0] = 90.0;
  EXPECT_FALSE(bridge::parse_force_hud_input(msg, q, tau, reason));
}
