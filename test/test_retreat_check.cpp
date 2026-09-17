#include <cmath>
#include <iostream>
#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"
#include "tianji_io_teleop_bridge/retreat_check.hpp"
using namespace tianji_io_teleop_bridge;

TEST(RetreatCheck, BothArmsAtInitialPose)
{
  MarvinKinematicsAdapter model;
  ASSERT_TRUE(model.initialize(TIANJI_TEST_KINEMATICS_CONFIG));
  for (std::size_t arm = 0; arm < 2; ++arm) {
    const ArmAngles q = arm == 0 ? ArmAngles{90,-75,-90,-90,0,0,0} :
      ArmAngles{-90,-75,90,-90,0,0,0};
    const auto result = check_retreat(arm, q, 0.005);
    std::cout << retreat_report(arm, result) << std::endl;
    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_EQ(result.path_deg.size(), 31U);
    EXPECT_NEAR(result.solved[0][3] - result.start[0][3], -30, 0.1);
    EXPECT_NEAR(result.solved[1][3], result.start[1][3], 0.1);
    EXPECT_NEAR(result.solved[2][3], result.start[2][3], 0.1);
    EXPECT_LE(result.max_rotation_error_deg, 0.05);
    EXPECT_LE(result.max_step_deg, 3);
    EXPECT_GE(result.min_limit_clearance_deg, 0);
  }
}
TEST(RetreatCheck, RejectsInvalidInputsAndOutsideLimits)
{
  MarvinKinematicsAdapter model;
  ASSERT_TRUE(model.initialize(TIANJI_TEST_KINEMATICS_CONFIG));
  ArmAngles q{90,-75,-90,-90,0,0,0};
  EXPECT_FALSE(check_retreat(2, q, 0.005).success);
  EXPECT_FALSE(check_retreat(0, q, -1).success);
  q[0] = std::nan("");
  EXPECT_FALSE(check_retreat(0, q, 0.005).success);
  q[0] = 200;
  EXPECT_FALSE(check_retreat(0, q, 0.005).success);
}

TEST(RetreatCheck, RejectsPathThatCrossesLimitDespiteValidStart)
{
  MarvinKinematicsAdapter model;
  ASSERT_TRUE(model.initialize(TIANJI_TEST_KINEMATICS_CONFIG));
  const auto result = check_retreat(0, ArmAngles{90,-75,-90,-144.6,0,0,0}, 0.005);
  std::cout << retreat_report(0, result) << std::endl;
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.path_deg.empty());
  EXPECT_NE(result.reason.find("waypoint"), std::string::npos);
}

TEST(RetreatCheck, SmoothHundredMillimetersRespectsSampledRatesAndPose)
{
  MarvinKinematicsAdapter model;
  ASSERT_TRUE(model.initialize(TIANJI_TEST_KINEMATICS_CONFIG));
  const auto plan = check_retreat(1, ArmAngles{-90,-60,90,-90,0,0,0}, 0.005, 100, 1000, true);
  ASSERT_TRUE(plan.success) << plan.reason;
  ArmAngles velocity{}, acceleration{};
  velocity.fill(0.20);
  acceleration.fill(0.40);
  const double dt = retreat_sample_period(plan.path_deg, 10.0, velocity, acceleration);
  std::cout << "Smooth retreat planned duration: " << dt * 1000 << " s\n";
  EXPECT_GE(dt, 0.01);
  EXPECT_LT(dt, 0.02);
  EXPECT_NEAR(plan.solved[0][3] - plan.start[0][3], -100, 0.1);
  EXPECT_LE(plan.max_rotation_error_deg, 0.05);
  for (std::size_t j = 0; j < 7; ++j) {
    double previous_velocity = 0;
    for (std::size_t i = 1; i < plan.path_deg.size(); ++i) {
      const double v = (plan.path_deg[i][j] - plan.path_deg[i - 1][j]) * std::acos(-1) / 180 / dt;
      EXPECT_LE(std::abs(v), velocity[j] + 1e-9);
      EXPECT_LE(std::abs(v - previous_velocity) / dt, acceleration[j] + 1e-9);
      previous_velocity = v;
    }
    EXPECT_LE(std::abs(previous_velocity) / dt, acceleration[j] + 1e-9);
  }
  PoseMatrix first{}, middle{}, penultimate{};
  ASSERT_TRUE(retreat_forward(1, plan.path_deg[1], first));
  ASSERT_TRUE(retreat_forward(1, plan.path_deg[500], middle));
  ASSERT_TRUE(retreat_forward(1, plan.path_deg[999], penultimate));
  EXPECT_LT(std::abs(first[0][3] - plan.start[0][3]), 0.001);
  EXPECT_LT(std::abs(penultimate[0][3] - plan.target[0][3]), 0.001);
  EXPECT_NEAR(middle[0][3] - plan.start[0][3], -50, 0.1);
  // An aggressive requested duration is stretched, never bypassing rate limits.
  EXPECT_GT(retreat_sample_period(plan.path_deg, 0.1, velocity, acceleration), 0.0001);
  EXPECT_THROW(retreat_sample_period(plan.path_deg, 0, velocity, acceleration), std::invalid_argument);
}
