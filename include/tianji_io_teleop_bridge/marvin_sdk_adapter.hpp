#ifndef TIANJI_IO_TELEOP_BRIDGE__MARVIN_SDK_ADAPTER_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__MARVIN_SDK_ADAPTER_HPP_

#include <array>
#include <string>

#include "MarvinSDK.h"
#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace tianji_io_teleop_bridge
{

enum class EnabledArms
{
  kNone,
  kLeft,
  kRight,
  kBoth
};

inline bool left_enabled(EnabledArms arms)
{
  return arms == EnabledArms::kLeft || arms == EnabledArms::kBoth;
}

inline bool right_enabled(EnabledArms arms)
{
  return arms == EnabledArms::kRight || arms == EnabledArms::kBoth;
}

struct JointImpedanceSettings
{
  std::array<double, kArmDof> left_stiffness{};
  std::array<double, kArmDof> left_damping{};
  std::array<double, kArmDof> right_stiffness{};
  std::array<double, kArmDof> right_damping{};
  int velocity_ratio{10};
  int acceleration_ratio{10};
};

class MarvinSdkAdapter
{
public:
  bool connect(const std::array<unsigned char, 4> & ip);
  bool disconnect();
  bool read_feedback(DCSS & feedback);

  bool configure_joint_impedance_parameters(
    EnabledArms arms,
    const JointImpedanceSettings & settings);
  bool enter_joint_impedance_mode(
    EnabledArms arms, const JointArray & hold_positions_deg);
  bool send_joint_targets(EnabledArms arms, const JointArray & positions_deg);
  bool request_idle(EnabledArms arms);

  bool connected() const {return connected_;}
  const std::string & last_error() const {return last_error_;}

private:
  bool fail(const std::string & message);
  void discard_pending_packet();

  bool connected_{false};
  std::string last_error_;
};

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__MARVIN_SDK_ADAPTER_HPP_
