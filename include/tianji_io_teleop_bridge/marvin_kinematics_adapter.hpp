#ifndef TIANJI_IO_TELEOP_BRIDGE__MARVIN_KINEMATICS_ADAPTER_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__MARVIN_KINEMATICS_ADAPTER_HPP_

#include <array>
#include <cstddef>
#include <string>

#include "tianji_io_teleop_bridge/disturbance_messages.hpp"
#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace tianji_io_teleop_bridge
{

class MarvinKinematicsAdapter
{
public:
  bool initialize(const std::string & config_file);
  bool flange_rotation(std::size_t arm_index, const std::array<double, kArmDof> & positions_deg,
    std::array<std::array<double, 3>, 3> & rotation);
  bool joint_disturbance_to_wrench(
    std::size_t arm_index,
    const std::array<double, kArmDof> & positions_deg,
    const std::array<double, kArmDof> & joint_disturbance,
    CartesianDisturbance & wrench);

  bool initialized() const {return initialized_;}
  const std::string & last_error() const {return last_error_;}

private:
  bool fail(const std::string & message);

  bool initialized_{false};
  std::string last_error_;
};

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__MARVIN_KINEMATICS_ADAPTER_HPP_
