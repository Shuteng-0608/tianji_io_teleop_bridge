#ifndef TIANJI_IO_TELEOP_BRIDGE__TOOL_CONFIGURATION_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__TOOL_CONFIGURATION_HPP_

#include <array>
#include <string>

namespace tianji_io_teleop_bridge
{
struct ToolConfiguration
{
  std::string profile;
  std::array<double, 6> kine{};  // SDK XYZABC: mm, degrees.
  std::array<double, 10> dyn{};  // SDK M,mx,my,mz,Ixx,Ixy,Ixz,Iyy,Iyz,Izz; unmodified.
};
bool valid_tool_configuration(const ToolConfiguration & config);
ToolConfiguration load_tool_configuration(const std::string & path, std::size_t arm);
std::string describe_tool_configuration(std::size_t arm, const ToolConfiguration & config);
bool tool_configuration_matches(const ToolConfiguration & config,
  const float * kine, const float * dyn);
}  // namespace tianji_io_teleop_bridge
#endif
