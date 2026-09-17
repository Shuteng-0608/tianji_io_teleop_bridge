#include "tianji_io_teleop_bridge/tool_configuration.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace tianji_io_teleop_bridge
{
bool valid_tool_configuration(const ToolConfiguration & config)
{
  const auto finite_float = [](double x) {
      return std::isfinite(x) && std::abs(x) <= std::numeric_limits<float>::max();
    };
  for (const double x : config.kine) {if (!finite_float(x)) {return false;}}
  for (const double x : config.dyn) {if (!finite_float(x)) {return false;}}
  // Do not reinterpret the manufacturer's identified inertia coefficients.
  return config.dyn[0] >= 0;
}

static YAML::Node required(const YAML::Node & map, const std::string & key)
{
  if (!map.IsMap()) {throw std::invalid_argument("expected map for " + key);}
  YAML::Node found;
  int count = 0;
  for (const auto & item : map) {
    if (item.first.as<std::string>() == key) {found = item.second; ++count;}
  }
  if (count != 1) {throw std::invalid_argument("missing/duplicate key: " + key);}
  return found;
}

ToolConfiguration load_tool_configuration(const std::string & path, std::size_t arm)
{
  if (arm > 1) {throw std::invalid_argument("invalid tool arm");}
  const auto root = YAML::LoadFile(path); // JSON is a YAML subset; no generated defaults.
  const auto key = "arm" + std::to_string(arm);
  ToolConfiguration result;
  result.profile = required(required(root, "current_tool"), key).as<std::string>();
  const auto selected = required(required(root, key), result.profile);
  const auto kine = required(selected, "kine"), dyn = required(selected, "dyn");
  if (!kine.IsSequence() || kine.size() != 6 || !dyn.IsSequence() || dyn.size() != 10) {
    throw std::invalid_argument("tool must have exactly 6 kine and 10 dyn values");
  }
  for (std::size_t j = 0; j < 6; ++j) {result.kine[j] = kine[j].as<double>();}
  for (std::size_t j = 0; j < 10; ++j) {result.dyn[j] = dyn[j].as<double>();}
  if (!valid_tool_configuration(result)) {throw std::invalid_argument("invalid numeric tool parameters");}
  return result;
}

std::string describe_tool_configuration(std::size_t arm, const ToolConfiguration & config)
{
  std::ostringstream out;
  out << std::setprecision(12) << "arm=" << (arm == 0 ? "left/A/arm0" : "right/B/arm1") <<
    " profile=" << config.profile << " kine=[";
  for (std::size_t j = 0; j < 6; ++j) {out << (j ? "," : "") << config.kine[j];}
  out << "] dyn=[";
  for (std::size_t j = 0; j < 10; ++j) {out << (j ? "," : "") << config.dyn[j];}
  return out.str() + "]";
}

bool tool_configuration_matches(const ToolConfiguration & config, const float * kine, const float * dyn)
{
  if (!valid_tool_configuration(config)) {return false;}
  for (std::size_t j = 0; j < 6; ++j) {
    if (!std::isfinite(kine[j]) || std::abs(kine[j] - static_cast<float>(config.kine[j])) > 1e-6f) {return false;}
  }
  for (std::size_t j = 0; j < 10; ++j) {
    if (!std::isfinite(dyn[j]) || std::abs(dyn[j] - static_cast<float>(config.dyn[j])) > 1e-6f) {return false;}
  }
  return true;
}
}  // namespace tianji_io_teleop_bridge
