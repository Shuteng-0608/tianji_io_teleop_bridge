#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

#include "FxRobot.h"

namespace tianji_io_teleop_bridge
{

bool MarvinKinematicsAdapter::initialize(const std::string & config_file)
{
  initialized_ = false;
  last_error_.clear();
  if (config_file.empty()) {
    return fail("kinematics configuration path is empty");
  }
  if (!std::filesystem::is_regular_file(config_file)) {
    return fail("kinematics configuration does not exist: " + config_file);
  }

  FX_INT32L robot_types[2] = {0, 0};
  FX_DOUBLE gravity[2][3] = {{0}};
  FX_DOUBLE dh[2][8][4] = {{{0}}};
  FX_DOUBLE limits[2][7][4] = {{{0}}};
  FX_DOUBLE joint_67_interference[2][4][3] = {{{0}}};
  FX_DOUBLE mass[2][7] = {{0}};
  FX_DOUBLE center_of_mass[2][7][3] = {{{0}}};
  FX_DOUBLE inertia[2][7][6] = {{{0}}};

  std::vector<char> mutable_path(config_file.begin(), config_file.end());
  mutable_path.push_back('\0');
  if (LOADMvCfg(
      mutable_path.data(), robot_types, gravity, dh, limits,
      joint_67_interference, mass, center_of_mass, inertia) == FX_FALSE)
  {
    return fail("LOADMvCfg failed for: " + config_file);
  }

  FX_LOG_SWITCH(0);
  for (std::size_t arm = 0; arm < 2; ++arm) {
    const FX_INT32L serial = static_cast<FX_INT32L>(arm);
    if (FX_Robot_Init_Type(serial, robot_types[arm]) == FX_FALSE) {
      return fail("FX_Robot_Init_Type failed for arm " + std::to_string(arm));
    }
    if (FX_Robot_Init_Kine(serial, dh[arm]) == FX_FALSE) {
      return fail("FX_Robot_Init_Kine failed for arm " + std::to_string(arm));
    }
    if (FX_Robot_Init_Lmt(
        serial, limits[arm], joint_67_interference[arm]) == FX_FALSE)
    {
      return fail("FX_Robot_Init_Lmt failed for arm " + std::to_string(arm));
    }
    // No calibrated TCP transform is available yet. Estimate the equivalent wrench at
    // each arm's flange and keep the ROS frame_id empty until frame names are confirmed.
    if (FX_Robot_Tool_Rmv(serial) == FX_FALSE) {
      return fail("FX_Robot_Tool_Rmv failed for arm " + std::to_string(arm));
    }
  }

  initialized_ = true;
  return true;
}

bool MarvinKinematicsAdapter::joint_disturbance_to_wrench(
  std::size_t arm_index,
  const std::array<double, kArmDof> & positions_deg,
  const std::array<double, kArmDof> & joint_disturbance,
  CartesianDisturbance & wrench)
{
  if (!initialized_) {
    return fail("wrench conversion requested before kinematics initialization");
  }
  if (arm_index >= 2) {
    return fail("wrench conversion requested with invalid arm index");
  }
  const auto finite = [](double value) {return std::isfinite(value);};
  if (!std::all_of(positions_deg.begin(), positions_deg.end(), finite) ||
    !std::all_of(joint_disturbance.begin(), joint_disturbance.end(), finite))
  {
    return fail("wrench conversion input contains a non-finite value");
  }

  Vect7 mutable_positions{};
  Vect7 mutable_disturbance{};
  Vect6 computed_wrench{};
  std::copy(positions_deg.begin(), positions_deg.end(), mutable_positions);
  std::copy(joint_disturbance.begin(), joint_disturbance.end(), mutable_disturbance);
  if (FX_Robot_JntTau2EETau(
      static_cast<FX_INT32L>(arm_index), mutable_positions,
      mutable_disturbance, computed_wrench) == FX_FALSE)
  {
    return fail(
      "FX_Robot_JntTau2EETau failed (singular or ill-conditioned Jacobian) for arm " +
      std::to_string(arm_index));
  }
  if (!std::all_of(std::begin(computed_wrench), std::end(computed_wrench), finite)) {
    return fail("wrench conversion produced a non-finite result");
  }

  std::copy(std::begin(computed_wrench), std::end(computed_wrench), wrench.begin());
  last_error_.clear();
  return true;
}

bool MarvinKinematicsAdapter::fail(const std::string & message)
{
  last_error_ = message;
  return false;
}

}  // namespace tianji_io_teleop_bridge
