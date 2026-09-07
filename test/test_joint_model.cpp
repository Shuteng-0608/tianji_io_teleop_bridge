#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace bridge = tianji_io_teleop_bridge;

namespace
{

sensor_msgs::msg::JointState make_valid_message()
{
  sensor_msgs::msg::JointState message;
  message.name.assign(bridge::joint_names().begin(), bridge::joint_names().end());
  message.position = {
    0.0, -0.5, -0.5, -0.5, 0.0, 0.0, 0.0,
    0.0, -0.5, 0.5, -0.5, 0.0, 0.0, 0.0};
  return message;
}

}  // namespace

TEST(JointModel, ConvertsDegreesAndRadians)
{
  EXPECT_NEAR(bridge::degrees_to_radians(180.0), bridge::kPi, 1.0e-12);
  EXPECT_NEAR(bridge::radians_to_degrees(bridge::kPi / 2.0), 90.0, 1.0e-12);
}

TEST(JointModel, ConfiguredDualArmInitialPoseIsInsideCommissioningSafetyMargin)
{
  const bridge::JointArray initial_pose_rad = {
    bridge::degrees_to_radians(90.0), bridge::degrees_to_radians(-75.0),
    bridge::degrees_to_radians(-90.0), bridge::degrees_to_radians(-90.0),
    0.0, 0.0, 0.0,
    bridge::degrees_to_radians(-90.0), bridge::degrees_to_radians(-75.0),
    bridge::degrees_to_radians(90.0), bridge::degrees_to_radians(-90.0),
    0.0, 0.0, 0.0};

  const auto limits = bridge::inspect_joint_limit_margin(initial_pose_rad, 0.005);
  ASSERT_TRUE(limits.valid);
  EXPECT_TRUE(limits.arm_safe[0]);
  EXPECT_TRUE(limits.arm_safe[1]);
  EXPECT_TRUE(bridge::within_safe_joint_limits(initial_pose_rad, 0.005));
}

TEST(JointModel, ReordersCommandByJointName)
{
  auto message = make_valid_message();
  std::reverse(message.name.begin(), message.name.end());
  std::reverse(message.position.begin(), message.position.end());

  const auto result = bridge::validate_joint_command(message, std::nullopt, 0.2, 1.0e-4);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_DOUBLE_EQ(result.positions_rad[1], -0.5);
  EXPECT_DOUBLE_EQ(result.positions_rad[9], 0.5);
}

TEST(JointModel, RejectsMissingJoint)
{
  auto message = make_valid_message();
  message.name.pop_back();
  message.position.pop_back();
  const auto result = bridge::validate_joint_command(message, std::nullopt, 0.2, 1.0e-4);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("14"), std::string::npos);
}

TEST(JointModel, RejectsDuplicateAndUnknownJointNames)
{
  auto duplicate = make_valid_message();
  duplicate.name[13] = duplicate.name[12];
  auto result = bridge::validate_joint_command(duplicate, std::nullopt, 0.2, 1.0e-4);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("duplicate"), std::string::npos);

  auto unknown = make_valid_message();
  unknown.name[13] = "Joint8_R";
  result = bridge::validate_joint_command(unknown, std::nullopt, 0.2, 1.0e-4);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("unknown"), std::string::npos);
}

TEST(JointModel, RejectsNonFiniteAndOutOfLimitPositions)
{
  auto non_finite = make_valid_message();
  non_finite.position[0] = std::numeric_limits<double>::quiet_NaN();
  auto result = bridge::validate_joint_command(non_finite, std::nullopt, 0.2, 1.0e-4);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("non-finite"), std::string::npos);

  auto out_of_limit = make_valid_message();
  out_of_limit.position[9] = -0.01;
  result = bridge::validate_joint_command(out_of_limit, std::nullopt, 0.2, 1.0e-4);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("limit"), std::string::npos);
}

TEST(JointModel, RejectsExcessiveCommandJump)
{
  auto message = make_valid_message();
  bridge::JointArray reference{};
  reference[1] = -0.5;
  reference[2] = -0.5;
  reference[3] = -0.5;
  reference[8] = -0.5;
  reference[9] = 0.5;
  reference[10] = -0.5;
  message.position[0] = 0.051;

  const auto result = bridge::validate_joint_command(message, reference, 0.05, 1.0e-4);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("jump"), std::string::npos);
}

TEST(JointModel, AcceptsUnalignedCandidateWhenNoRunningReferenceIsProvided)
{
  auto message = make_valid_message();
  message.position[0] = 1.0;
  bridge::JointArray feedback{};

  const auto startup_candidate = bridge::validate_joint_command(
    message, std::nullopt, 0.05, 1.0e-4);
  EXPECT_TRUE(startup_candidate.valid) << startup_candidate.reason;

  const auto running_command = bridge::validate_joint_command(
    message, feedback, 0.05, 1.0e-4);
  EXPECT_FALSE(running_command.valid);
  EXPECT_NE(running_command.reason.find("jump"), std::string::npos);
}

TEST(JointModel, AcceptsCommandAtConfiguredLimits)
{
  auto message = make_valid_message();
  message.position.assign(bridge::lower_limits_rad().begin(), bridge::lower_limits_rad().end());
  const auto result = bridge::validate_joint_command(message, std::nullopt, 0.2, 1.0e-4);
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(JointModel, IgnoresMessagesWithoutTianjiArmJoints)
{
  sensor_msgs::msg::JointState hand_message;
  hand_message.name = {"lh_FFJ4", "lh_FFJ3", "lh_THJ1"};
  hand_message.position = {0.1, 0.2, 0.3};

  const auto result = bridge::evaluate_joint_command(
    hand_message, std::nullopt, 0.2, 1.0e-4);
  EXPECT_EQ(result.disposition, bridge::CommandDisposition::kIgnoredNonArm);
}

TEST(JointModel, RejectsPartialMessagesThatContainArmJoints)
{
  sensor_msgs::msg::JointState partial;
  partial.name = {"Joint1_L"};
  partial.position = {0.0};

  const auto result = bridge::evaluate_joint_command(partial, std::nullopt, 0.2, 1.0e-4);
  EXPECT_EQ(result.disposition, bridge::CommandDisposition::kRejected);
}

TEST(JointModel, AcceptsCompleteArmMessageThroughClassifier)
{
  const auto result = bridge::evaluate_joint_command(
    make_valid_message(), std::nullopt, 0.2, 1.0e-4);
  EXPECT_EQ(result.disposition, bridge::CommandDisposition::kAccepted);
}

TEST(JointModel, ReportsSoftLimitViolationsPerArmAndJoint)
{
  bridge::JointArray positions{};
  positions = {
    0.0, -0.5, -0.5, -0.5, 0.0, 0.0, 0.0,
    0.0, -0.5, 0.5, -0.5, 0.0, 0.0, 0.0};

  auto result = bridge::inspect_joint_limit_margin(positions, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.arm_safe[0]);
  EXPECT_TRUE(result.arm_safe[1]);
  EXPECT_FALSE(result.first_violating_joint.has_value());

  positions[2] = -0.005;
  positions[9] = 0.005;
  result = bridge::inspect_joint_limit_margin(positions, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.arm_safe[0]);
  EXPECT_FALSE(result.arm_safe[1]);
  EXPECT_FALSE(result.joint_safe[2]);
  EXPECT_FALSE(result.joint_safe[9]);
  ASSERT_TRUE(result.first_violating_joint.has_value());
  EXPECT_EQ(result.first_violating_joint.value(), 2U);
}

TEST(JointModel, SoftLimitMarginsSupportRecoveryHysteresis)
{
  bridge::JointArray positions{
    0.0, -0.5, -0.5, -0.5, 0.0, 0.0, 0.0,
    0.0, -0.5, 0.5, -0.5, 0.0, 0.0, 0.0};

  positions[9] = 0.02;
  EXPECT_TRUE(bridge::within_safe_joint_limits(positions, 0.01));
  EXPECT_FALSE(bridge::within_safe_joint_limits(positions, 0.03));

  positions[9] = 0.04;
  EXPECT_TRUE(bridge::within_safe_joint_limits(positions, 0.01));
  EXPECT_TRUE(bridge::within_safe_joint_limits(positions, 0.03));

  EXPECT_FALSE(bridge::inspect_joint_limit_margin(positions, -0.01).valid);
}

TEST(JointModel, SlewLimiterBoundsAccelerationAndVelocity)
{
  bridge::JointArray start{};
  bridge::JointArray desired{};
  bridge::JointArray max_velocity{};
  bridge::JointArray max_acceleration{};
  desired.fill(1.0);
  max_velocity.fill(0.25);
  max_acceleration.fill(0.50);

  bridge::JointSlewLimiter limiter;
  limiter.reset(start);
  const auto first = limiter.update(desired, max_velocity, max_acceleration, 0.01);
  EXPECT_NEAR(first[0], 0.00005, 1.0e-12);

  auto output = first;
  for (int iteration = 0; iteration < 100; ++iteration) {
    const auto previous = output;
    output = limiter.update(desired, max_velocity, max_acceleration, 0.01);
    EXPECT_LE(output[0] - previous[0], 0.0025 + 1.0e-12);
    EXPECT_LE(output[0], desired[0]);
  }
}
