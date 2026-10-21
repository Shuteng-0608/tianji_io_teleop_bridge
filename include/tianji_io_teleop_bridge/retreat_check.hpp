#ifndef TIANJI_IO_TELEOP_BRIDGE__RETREAT_CHECK_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__RETREAT_CHECK_HPP_

#include <array>
#include <string>
#include <vector>

namespace tianji_io_teleop_bridge
{
using ArmAngles = std::array<double, 7>;
using PoseMatrix = std::array<std::array<double, 4>, 4>;
struct RetreatCheck
{
  bool success{false};
  std::string reason;
  PoseMatrix start{}, target{}, solved{};  // SDK arm base frame; translation in mm.
  std::vector<ArmAngles> path_deg;
  double max_position_error_mm{0}, max_rotation_error_deg{0}, max_step_deg{0};
  double min_limit_clearance_deg{180};
  double distance_mm{30};
};

// Requires initialized libKine in this process; never uses the control SDK.
bool retreat_forward(std::size_t arm, const ArmAngles & q, PoseMatrix & pose);
RetreatCheck check_retreat(std::size_t arm, const ArmAngles & start_deg, double margin_rad,
  double distance_mm = 30.0, int segments = 30, bool smooth = false);
std::string retreat_report(std::size_t arm, const RetreatCheck & result);
}  // namespace tianji_io_teleop_bridge
#endif
