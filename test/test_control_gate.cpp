#include <string>

#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/control_gate.hpp"

namespace bridge = tianji_io_teleop_bridge;

TEST(ControlGate, StartsInMonitorAndRequiresConsecutiveAlignment)
{
  bridge::ControlGate gate(3);
  EXPECT_EQ(gate.state(), bridge::ControlState::kMonitor);

  gate.request_enable();
  EXPECT_EQ(gate.state(), bridge::ControlState::kPreparingParameters);
  EXPECT_FALSE(gate.observe_alignment(true));
  gate.mark_configuring();
  EXPECT_EQ(gate.state(), bridge::ControlState::kConfiguringImpedance);
  gate.mark_waiting_for_alignment();
  EXPECT_EQ(gate.state(), bridge::ControlState::kWaitingForAlignment);
  EXPECT_FALSE(gate.observe_alignment(true));
  EXPECT_FALSE(gate.observe_alignment(false));
  EXPECT_FALSE(gate.observe_alignment(true));
  EXPECT_FALSE(gate.observe_alignment(true));
  EXPECT_TRUE(gate.observe_alignment(true));
}

TEST(ControlGate, DoesNotAutoResumeAfterHoldOrFault)
{
  bridge::ControlGate gate(1);
  gate.request_enable();
  gate.mark_configuring();
  gate.mark_waiting_for_alignment();
  ASSERT_TRUE(gate.observe_alignment(true));
  gate.mark_active();
  EXPECT_EQ(gate.state(), bridge::ControlState::kActive);

  gate.enter_hold("timeout");
  EXPECT_EQ(gate.state(), bridge::ControlState::kHold);
  EXPECT_FALSE(gate.observe_alignment(true));

  gate.enter_fault("send failed");
  EXPECT_EQ(gate.state(), bridge::ControlState::kFault);
  EXPECT_FALSE(gate.observe_alignment(true));

  gate.disarm();
  EXPECT_EQ(gate.state(), bridge::ControlState::kMonitor);
}

TEST(ControlGate, RepresentsInitialPoseMotionExplicitly)
{
  bridge::ControlGate gate(1);
  gate.request_enable();
  gate.mark_configuring();
  gate.mark_returning_to_initial_pose();

  EXPECT_EQ(gate.state(), bridge::ControlState::kReturningToInitialPose);
  EXPECT_STREQ(
    bridge::control_state_name(gate.state()), "RETURNING_TO_INITIAL_POSE");
  EXPECT_FALSE(gate.observe_alignment(true));

  gate.enter_hold("initial pose reached");
  EXPECT_EQ(gate.state(), bridge::ControlState::kHold);
}

TEST(ControlGate, SoftLimitHoldRequiresConsecutiveRecoveryFrames)
{
  bridge::ControlGate gate(1, 3);
  gate.request_enable();
  gate.mark_configuring();
  gate.mark_waiting_for_alignment();
  ASSERT_TRUE(gate.observe_alignment(true));
  gate.mark_active();

  gate.enter_soft_limit_hold("Joint3_R near lower limit");
  EXPECT_EQ(gate.state(), bridge::ControlState::kSoftLimitHold);
  EXPECT_FALSE(gate.observe_soft_limit_recovery(true));
  EXPECT_EQ(gate.soft_limit_recovery_frames(), 1U);
  EXPECT_FALSE(gate.observe_soft_limit_recovery(false));
  EXPECT_EQ(gate.soft_limit_recovery_frames(), 0U);
  EXPECT_FALSE(gate.observe_soft_limit_recovery(true));
  EXPECT_FALSE(gate.observe_soft_limit_recovery(true));
  EXPECT_TRUE(gate.observe_soft_limit_recovery(true));

  gate.mark_active();
  EXPECT_EQ(gate.state(), bridge::ControlState::kActive);
  EXPECT_EQ(gate.soft_limit_recovery_frames(), 0U);
}
