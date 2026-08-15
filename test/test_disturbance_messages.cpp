#include <array>

#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/disturbance_messages.hpp"

namespace tianji_io_teleop_bridge
{
namespace
{

TEST(DisturbanceMessages, PreservesCanonicalJointOrderAndValues)
{
  JointArray disturbance{};
  for (std::size_t index = 0; index < disturbance.size(); ++index) {
    disturbance[index] = static_cast<double>(index) + 0.25;
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 345;

  const auto message = make_joint_disturbance_message(disturbance, stamp);

  ASSERT_EQ(message.name.size(), kJointCount);
  ASSERT_EQ(message.effort.size(), kJointCount);
  EXPECT_TRUE(message.position.empty());
  EXPECT_TRUE(message.velocity.empty());
  EXPECT_EQ(message.name.front(), "Joint1_L");
  EXPECT_EQ(message.name[kArmDof], "Joint1_R");
  EXPECT_EQ(message.name.back(), "Joint7_R");
  EXPECT_DOUBLE_EQ(message.effort.front(), 0.25);
  EXPECT_DOUBLE_EQ(message.effort[kArmDof], 7.25);
  EXPECT_DOUBLE_EQ(message.effort.back(), 13.25);
  EXPECT_EQ(message.header.stamp.sec, 12);
  EXPECT_EQ(message.header.stamp.nanosec, 345U);
}

TEST(DisturbanceMessages, MapsCartesianForceAndTorqueWithoutReordering)
{
  const CartesianDisturbance disturbance{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 98;
  stamp.nanosec = 765;

  const auto message = make_cartesian_disturbance_message(
    disturbance, stamp, "confirmed_frame");

  EXPECT_EQ(message.header.frame_id, "confirmed_frame");
  EXPECT_EQ(message.header.stamp.sec, 98);
  EXPECT_EQ(message.header.stamp.nanosec, 765U);
  EXPECT_DOUBLE_EQ(message.wrench.force.x, 1.0);
  EXPECT_DOUBLE_EQ(message.wrench.force.y, 2.0);
  EXPECT_DOUBLE_EQ(message.wrench.force.z, 3.0);
  EXPECT_DOUBLE_EQ(message.wrench.torque.x, 4.0);
  EXPECT_DOUBLE_EQ(message.wrench.torque.y, 5.0);
  EXPECT_DOUBLE_EQ(message.wrench.torque.z, 6.0);
}

}  // namespace
}  // namespace tianji_io_teleop_bridge
