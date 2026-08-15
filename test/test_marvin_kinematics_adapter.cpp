#include <array>
#include <cmath>

#include "FxRobot.h"
#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"

namespace tianji_io_teleop_bridge
{
namespace
{

TEST(MarvinKinematicsAdapter, RejectsMissingConfiguration)
{
  MarvinKinematicsAdapter adapter;
  EXPECT_FALSE(adapter.initialize("/definitely/not/a/marvin/config.MvKDCfg"));
  EXPECT_FALSE(adapter.initialized());
}

TEST(MarvinKinematicsAdapter, RoundTripsWrenchThroughBothArmJacobians)
{
  MarvinKinematicsAdapter adapter;
  ASSERT_TRUE(adapter.initialize(TIANJI_TEST_KINEMATICS_CONFIG)) << adapter.last_error();

  const std::array<std::array<double, kArmDof>, 2> positions_deg{{
    {{30.0, -55.0, 25.0, -80.0, 35.0, 30.0, 15.0}},
    {{-30.0, -55.0, -25.0, -80.0, -35.0, 30.0, -15.0}}
  }};
  const CartesianDisturbance expected_wrench{8.0, -4.0, 6.0, 0.8, -0.5, 0.3};

  for (std::size_t arm = 0; arm < 2; ++arm) {
    Vect7 mutable_positions{};
    std::copy(
      positions_deg[arm].begin(), positions_deg[arm].end(), mutable_positions);
    FX_Jacobi jacobian{};
    ASSERT_EQ(
      FX_Robot_Kine_Jacb(static_cast<FX_INT32L>(arm), mutable_positions, &jacobian),
      FX_TRUE);

    std::array<double, kArmDof> joint_disturbance{};
    for (std::size_t joint = 0; joint < kArmDof; ++joint) {
      for (std::size_t axis = 0; axis < expected_wrench.size(); ++axis) {
        joint_disturbance[joint] +=
          jacobian.m_Jcb[axis][joint] * expected_wrench[axis];
      }
    }

    CartesianDisturbance computed_wrench{};
    ASSERT_TRUE(
      adapter.joint_disturbance_to_wrench(
        arm, positions_deg[arm], joint_disturbance, computed_wrench)) <<
      adapter.last_error();
    for (std::size_t axis = 0; axis < expected_wrench.size(); ++axis) {
      EXPECT_NEAR(computed_wrench[axis], expected_wrench[axis], 1.0e-8);
    }
  }
}

TEST(MarvinKinematicsAdapter, RejectsInvalidArmAndNonFiniteInput)
{
  MarvinKinematicsAdapter adapter;
  ASSERT_TRUE(adapter.initialize(TIANJI_TEST_KINEMATICS_CONFIG)) << adapter.last_error();
  std::array<double, kArmDof> positions_deg{};
  std::array<double, kArmDof> joint_disturbance{};
  CartesianDisturbance wrench{};

  EXPECT_FALSE(
    adapter.joint_disturbance_to_wrench(2, positions_deg, joint_disturbance, wrench));
  positions_deg[3] = std::nan("");
  EXPECT_FALSE(
    adapter.joint_disturbance_to_wrench(0, positions_deg, joint_disturbance, wrench));
}

TEST(MarvinKinematicsAdapter, ComputesObservedCommissioningPose)
{
  MarvinKinematicsAdapter adapter;
  ASSERT_TRUE(adapter.initialize(TIANJI_TEST_KINEMATICS_CONFIG)) << adapter.last_error();
  const std::array<std::array<double, kArmDof>, 2> positions_deg{{
    {{91.1482544, -76.3175583, -89.5921631, -89.0354538, -3.5460572, -0.0307755,
      -0.6391159}},
    {{-92.5033112, -74.6600342, 89.3552780, -88.7921600, -0.8604630, 1.3009117,
      -0.1074463}}
  }};
  const std::array<std::array<double, kArmDof>, 2> disturbance{{
    {{0.0133366, -0.0631399, -0.0865060, 0.1835609, -0.2024711, 0.5055825,
      -0.1236015}},
    {{-0.5953137, 0.4416444, -0.0731883, 0.0866165, -0.0973934, 0.0307739,
      -0.0022100}}
  }};

  for (std::size_t arm = 0; arm < 2; ++arm) {
    CartesianDisturbance wrench{};
    ASSERT_TRUE(
      adapter.joint_disturbance_to_wrench(
        arm, positions_deg[arm], disturbance[arm], wrench)) << adapter.last_error();
    for (double value : wrench) {
      EXPECT_TRUE(std::isfinite(value));
    }
  }
}

}  // namespace
}  // namespace tianji_io_teleop_bridge
