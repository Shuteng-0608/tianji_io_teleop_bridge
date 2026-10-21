#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/joint_torque_zero.hpp"

namespace tianji_io_teleop_bridge
{
namespace
{
using Arm = JointTorqueZero::ArmArray;
void feed(JointTorqueZero & zero, std::size_t arm, double end = 1.0, double offset = 1.0)
{
  for (int i = 0; i <= 100; ++i) {
    Arm tau{};
    for (std::size_t j = 0; j < kArmDof; ++j) {tau[j] = offset + j * 0.1;}
    zero.observe(arm, end - 1.0 + i * 0.01, i, Arm{}, Arm{}, tau);
  }
}
}

TEST(JointTorqueZero, PerJointBiasAndRepeatedCaptureUseRawNotCorrected)
{
  JointTorqueZero zero;
  std::string reason;
  JointArray raw{};
  for (std::size_t j = 0; j < kJointCount; ++j) {raw[j] = 1.0 + (j % 7) * 0.1;}
  EXPECT_EQ(zero.correct(raw), raw);
  feed(zero, 0); feed(zero, 1, 1.0, -2.0);
  ASSERT_TRUE(zero.capture({true, false}, 1.0, reason)) << reason;
  for (int j = 0; j < 7; ++j) {
    EXPECT_NEAR(zero.correct(raw)[j], 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(zero.correct(raw)[7 + j], raw[7 + j]);
  }
  ASSERT_TRUE(zero.capture({true, true}, 1.0, reason));
  const auto bias = zero.bias();
  ASSERT_TRUE(zero.capture({true, true}, 1.0, reason));
  EXPECT_EQ(zero.bias(), bias);
  raw[0] += 3.0;  // Contact after calibration must remain, even at a new posture.
  EXPECT_NEAR(zero.correct(raw)[0], 3.0, 1e-12);
  zero.clear(0);
  EXPECT_FALSE(zero.active(0)); EXPECT_TRUE(zero.active(1));
  EXPECT_DOUBLE_EQ(zero.correct(raw)[0], raw[0]);
}

TEST(JointTorqueZero, BothCaptureIsAtomicAndFailureKeepsOldBias)
{
  JointTorqueZero zero;
  std::string reason;
  feed(zero, 0); feed(zero, 1);
  ASSERT_TRUE(zero.capture({true, true}, 1.0, reason));
  auto old = zero.bias();
  feed(zero, 0, 3.0, 4.0);
  EXPECT_FALSE(zero.capture({true, true}, 3.0, reason));
  EXPECT_EQ(zero.bias(), old);
  EXPECT_NE(reason.find("right"), std::string::npos);
}

TEST(JointTorqueZero, RejectsRepeatedFramesAndExpiredData)
{
  JointTorqueZero zero;
  std::string reason;
  for (int i = 0; i < 150; ++i) {
    zero.observe(0, i * 0.01, 1, Arm{}, Arm{}, Arm{});
  }
  EXPECT_FALSE(zero.capture({true, false}, 1.49, reason));
  feed(zero, 0, 3.0);
  EXPECT_FALSE(zero.capture({true, false}, 3.21, reason));
  zero.observe(0, 4.0, 200, Arm{}, Arm{}, Arm{}); // Gap resets history.
  EXPECT_FALSE(zero.capture({true, false}, 4.0, reason));
}

TEST(JointTorqueZero, RejectsMotionDriftNoiseAndInvalidSamples)
{
  for (int kind = 0; kind < 4; ++kind) {
    JointTorqueZero zero;
    for (int i = 0; i <= 100; ++i) {
      Arm q{}, dq{}, tau{};
      if (kind == 0) {dq[2] = 0.02;}
      if (kind == 1) {q[2] = i * 0.0001;}
      if (kind == 2) {tau[2] = i % 2 ? 1.0 : -1.0;}
      if (kind == 3 && i == 90) {tau[2] = std::numeric_limits<double>::quiet_NaN();}
      zero.observe(0, i * 0.01, i, q, dq, tau);
    }
    std::string reason;
    EXPECT_FALSE(zero.capture({true, false}, 1.0, reason)) << kind;
    EXPECT_FALSE(zero.active(0));
  }
}
}  // namespace tianji_io_teleop_bridge
