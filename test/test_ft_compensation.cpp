#include <filesystem>
#include <fstream>
#include <chrono>
#include <limits>
#include <Eigen/Geometry>
#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/ft_compensation.hpp"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"

namespace tianji_io_teleop_bridge
{
namespace
{
class FtCompensationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path = std::filesystem::temp_directory_path() / ("ft-comp-test-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path);
    calibration = YAML::LoadFile(std::string(FT_TEST_SOURCE) + "/test/fixtures/right_ft_calibration_20261016_03.yaml");
    calibration["parameters"]["mass_kg"] = 2.;
    calibration["parameters"]["com_sensor_m"] = std::vector<double>{.01, .02, .03};
    calibration["parameters"]["force_bias_N"] = std::vector<double>{.4, .1, 1.2};
    calibration["parameters"]["torque_bias_Nm"] = std::vector<double>{.01, .02, .03};
    calibration["gravity_base_m_s2"] = std::vector<double>{0, -10, 0};
    load();
  }
  void TearDown() override {std::filesystem::remove_all(path);}
  void load()
  {
    std::ofstream file(path / "cal.yaml");
    file << calibration;
    file.close();
    ft.load((path / "cal.yaml").string(), FT_TEST_MODEL, FT_TEST_MODEL);
  }
  void fill(const FtCompensation::Wrench & residual = {.2, .1, .8, .003, .005, .016})
  {
    for (int i = 0; i <= 510; ++i) {
      ft.observe(.01 * i, i, 10000000LL * i, 1, true, {}, {}, residual);
    }
  }
  FtCompensation::Capture capture()
  {
    FtCompensation::Capture c;
    std::string reason;
    EXPECT_TRUE(ft.candidate(5.101, c, reason)) << reason;
    return c;
  }
  FtCompensation ft;
  YAML::Node calibration;
  std::filesystem::path path;
};

TEST_F(FtCompensationTest, GravityTorqueAndBiasAreSubtractedOnceAtDifferentOrientations)
{
  const auto zero = ft.compensate({.4, -19.9, 1.2, .61, .02, -.17}, Eigen::Matrix3d::Identity());
  for (double x : zero) {EXPECT_NEAR(x, 0, 1e-12);}
  Eigen::Matrix3d r;
  r << 1, 0, 0, 0, 0, -1, 0, 1, 0;  // Gravity in sensor frame becomes +Z.
  const auto force = ft.compensate({3.4, .1, 21.2, .41, -.18, .03}, r);
  EXPECT_NEAR(force[0], 3, 1e-12);
  for (std::size_t i = 1; i < 6; ++i) {EXPECT_NEAR(force[i], 0, 1e-12);}
}

TEST_F(FtCompensationTest, SensorMountAndSignsAreAppliedInTheDeclaredDirection)
{
  calibration["sensor"]["R_flange_sensor"] = std::vector<std::vector<double>>{{1,0,0},{0,0,-1},{0,1,0}};
  calibration["sensor"]["force_sign"] = -1;
  calibration["sensor"]["torque_sign"] = -1;
  load();
  const auto zero = ft.compensate({-.4, -.1, -21.2, -.41, .18, -.03}, Eigen::Matrix3d::Identity());
  for (double x : zero) {EXPECT_NEAR(x, 0, 1e-12);}
}

TEST_F(FtCompensationTest, CaptureReplacesSessionBiasAndPersistenceContainsFullCalibration)
{
  EXPECT_THROW(ft.corrected({}), std::runtime_error);
  fill();
  const auto c = capture();
  const auto file = ft.save_and_apply(c, path.string(), 100, "/io_teleop/right/ft_sensor");
  ASSERT_TRUE(ft.active());
  const auto record = YAML::LoadFile(file);
  EXPECT_DOUBLE_EQ(record["calibration_snapshot"]["parameters"]["mass_kg"].as<double>(), 2.);
  EXPECT_DOUBLE_EQ(record["software_tare_N_Nm"][2].as<double>(), .8);
  EXPECT_GE(record["span_sec"].as<double>(), 5.);
  EXPECT_FALSE(record["hardware_zero_called"].as<bool>());
  for (double x : ft.corrected(c.bias)) {EXPECT_NEAR(x, 0, 1e-12);}
  ft.save_and_apply(c, path.string(), 101, "/io_teleop/right/ft_sensor");
  EXPECT_EQ(ft.bias(), c.bias);  // No second subtraction of the old session tare.
  const auto old_bias = ft.bias();
  EXPECT_THROW(ft.save_and_apply(c, (path / "cal.yaml" / "bad").string(), 102, "topic"), std::exception);
  EXPECT_EQ(ft.bias(), old_bias);
  auto with_contact = c.bias;
  with_contact[0] += 4;
  EXPECT_NEAR(ft.corrected(with_contact)[0], 4, 1e-12);
  ft.reset();
  EXPECT_FALSE(ft.active());
  EXPECT_THROW(ft.corrected({}), std::runtime_error);
}

TEST_F(FtCompensationTest, LargeStableResidualIsAllowedWithoutChangingCalibration)
{
  fill({2, 3, 4, .1, .2, .3});
  const auto c = capture();
  EXPECT_DOUBLE_EQ(c.bias[2], 4);
}

TEST_F(FtCompensationTest, RejectsStaleDuplicateMovingUnstableAndChangedModeWindows)
{
  const FtCompensation::Wrench residual{.2, .1, .8, .003, .005, .016};
  for (int scenario = 0; scenario < 7; ++scenario) {
    ft.reset();
    for (int i = 0; i <= 510; ++i) {
      FtCompensation::Joints q{}, v{};
      auto w = residual;
      if (scenario == 2) {v[0] = .3;}
      if (scenario == 3) {q[0] = i * .001;}
      if (scenario == 4) {w[0] += i % 2;}
      ft.observe(.01 * i, scenario == 1 ? 0 : i, i, scenario == 5 && i > 400 ? 3 : 1,
        scenario != 6, q, v, w);
    }
    FtCompensation::Capture c;
    std::string reason;
    EXPECT_FALSE(ft.candidate(scenario == 0 ? 6. : 5.1, c, reason)) << scenario;
  }
  ft.reset();
  fill();
  ft.clear_window();
  FtCompensation::Capture c;
  std::string reason;
  EXPECT_FALSE(ft.candidate(5.101, c, reason));
}

TEST_F(FtCompensationTest, RejectsBadCalibrationAndFk)
{
  calibration["parameters"]["mass_kg"] = -1.;
  EXPECT_THROW(load(), std::invalid_argument);
  calibration["parameters"]["mass_kg"] = 2.;
  calibration["sensor"]["R_flange_sensor"][0][0] = -1.;
  EXPECT_THROW(load(), std::invalid_argument);
  calibration["sensor"]["R_flange_sensor"][0][0] = 1.;
  calibration["model_sha256"] = "wrong";
  EXPECT_THROW(load(), std::invalid_argument);
  calibration["model_sha256"] = "ce20b1a80974c3ed58cdac4973152625096c8ad93a779e5fd4108c26ce19bd52";
  load();
  EXPECT_THROW(ft.compensate({}, Eigen::Matrix3d::Zero()), std::runtime_error);
  FtCompensation::Wrench invalid{};
  invalid[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ft.compensate(invalid, Eigen::Matrix3d::Identity()), std::runtime_error);
}

TEST(FtCompensationReplay, MatchesIndependentPythonResultsForAllCalibrationAndObservedPoses)
{
  FtCompensation ft;
  ft.load(std::string(FT_TEST_SOURCE) + "/test/fixtures/right_ft_calibration_20261016_03.yaml", FT_TEST_MODEL, FT_TEST_MODEL);
  MarvinKinematicsAdapter kine;
  ASSERT_TRUE(kine.initialize(FT_TEST_MODEL));
  const auto data = YAML::LoadFile(std::string(FT_TEST_SOURCE) + "/test/fixtures/right_ft_compensation_replay.yaml");
  ASSERT_EQ(data["poses"].size(), 20U);
  for (const auto & pose : data["poses"]) {
    FtCompensation::Joints q{};
    FtCompensation::Wrench measured{};
    for (std::size_t i = 0; i < 7; ++i) {q[i] = pose["joint_deg"][i].as<double>();}
    for (std::size_t i = 0; i < 6; ++i) {measured[i] = pose["measured_si"][i].as<double>();}
    std::array<std::array<double, 3>, 3> rotation{};
    ASSERT_TRUE(kine.flange_rotation(1, q, rotation));
    Eigen::Matrix3d r;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {r(i,j) = rotation[i][j];}
    }
    const auto result = ft.compensate(measured, r);
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_NEAR(result[i], pose["expected_compensated_si"][i].as<double>(), 1e-6) << pose["id"];
    }
  }
}
}  // namespace
}  // namespace tianji_io_teleop_bridge
