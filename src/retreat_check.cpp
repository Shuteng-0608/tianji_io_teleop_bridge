#include "tianji_io_teleop_bridge/retreat_check.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include "FxRobot.h"
#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace tianji_io_teleop_bridge
{
bool retreat_forward(std::size_t arm, const ArmAngles & q, PoseMatrix & pose)
{
  if (arm > 1 || !std::all_of(q.begin(), q.end(), [](double x) {return std::isfinite(x);})) {
    return false;
  }
  Vect7 joints{};
  std::copy(q.begin(), q.end(), joints);
  Matrix4 matrix{};
  if (!FX_Robot_Kine_FK(arm, joints, matrix)) {return false;}
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (!std::isfinite(matrix[i][j])) {return false;}
      pose[i][j] = matrix[i][j];
    }
  }
  return true;
}

RetreatCheck check_retreat(std::size_t arm, const ArmAngles & start_deg, double margin_rad,
  double distance_mm, int segments, bool smooth)
{
  RetreatCheck out;
  out.distance_mm = distance_mm;
  if (arm > 1 || !std::isfinite(margin_rad) || margin_rad < 0 ||
    !std::isfinite(distance_mm) || distance_mm <= 0 || distance_mm > 100 ||
    segments < 1 || segments > 1000)
  {
    out.reason = "invalid arm or limit margin"; return out;
  }
  auto limits_ok = [&](const ArmAngles & q) {
      for (std::size_t j = 0; j < 7; ++j) {
        const auto index = arm * 7 + j;
        const double clearance = std::min(
          q[j] - radians_to_degrees(lower_limits_rad()[index] + margin_rad),
          radians_to_degrees(upper_limits_rad()[index] - margin_rad) - q[j]);
        if (!std::isfinite(q[j]) || clearance < 0) {return false;}
        out.min_limit_clearance_deg = std::min(out.min_limit_clearance_deg, clearance);
      }
      return true;
    };
  if (!limits_ok(start_deg)) {out.reason = "start joint outside planning limits"; return out;}
  if (!retreat_forward(arm, start_deg, out.start)) {out.reason = "start FK failed"; return out;}
  out.target = out.start;
  out.target[0][3] -= distance_mm;  // Both arm bases +X forward.
  out.path_deg.push_back(start_deg);
  for (int step = 0; step <= segments; ++step) {
    PoseMatrix goal = out.start;
    const double u = static_cast<double>(step) / segments;
    // Quintic time scaling: zero first and second derivatives at both ends.
    const double progress = smooth ? u * u * u * (10.0 + u * (-15.0 + 6.0 * u)) : u;
    goal[0][3] -= distance_mm * progress;  // Rotation and Y/Z unchanged.
    FX_InvKineSolvePara solve{};
    for (int i = 0; i < 4; ++i) {
      std::copy(goal[i].begin(), goal[i].end(), solve.m_Input_IK_TargetTCP[i]);
    }
    std::copy(out.path_deg.back().begin(), out.path_deg.back().end(), solve.m_Input_IK_RefJoint);
    solve.m_Input_IK_ZSPType = 0;
    const auto reject = [&](const std::string & reason) {
        out.reason = "waypoint " + std::to_string(step) + "/" + std::to_string(segments) + ": " + reason;
      };
    if (!FX_Robot_Kine_IK(arm, &solve) || solve.m_Output_IsOutRange ||
      solve.m_Output_IsJntExd || solve.m_OutPut_Result_Num <= 0)
    {reject("IK unreachable or SDK joint limit failure"); return out;}
    for (int j = 0; j < 7; ++j) {
      if (solve.m_Output_IsDeg[j] || solve.m_Output_JntExdTags[j]) {
        reject("SDK singularity/limit flag at joint " + std::to_string(j + 1)); return out;
      }
    }
    ArmAngles q{};
    std::copy_n(solve.m_Output_RetJoint, 7, q.begin());
    if (!limits_ok(q)) {reject("outside bridge planning limits"); return out;}
    for (int j = 0; j < 7; ++j) {
      out.max_step_deg = std::max(out.max_step_deg, std::abs(q[j] - out.path_deg.back()[j]));
    }
    if (out.max_step_deg > 3.0) {reject("joint step exceeds 3 degrees"); return out;}
    if (!retreat_forward(arm, q, out.solved)) {reject("verification FK failed"); return out;}
    double position_error2 = 0, rotation_dot = 0;
    for (int i = 0; i < 3; ++i) {
      position_error2 += std::pow(out.solved[i][3] - goal[i][3], 2);
      for (int j = 0; j < 3; ++j) {rotation_dot += out.solved[i][j] * goal[i][j];}
    }
    out.max_position_error_mm = std::max(out.max_position_error_mm, std::sqrt(position_error2));
    out.max_rotation_error_deg = std::max(out.max_rotation_error_deg,
      radians_to_degrees(std::acos(std::clamp((rotation_dot - 1.0) / 2.0, -1.0, 1.0))));
    if (out.max_position_error_mm > 0.1 || out.max_rotation_error_deg > 0.05) {
      reject("FK residual exceeds 0.1 mm / 0.05 degrees"); return out;
    }
    if (step > 0) {out.path_deg.push_back(q);}
  }
  out.success = true;
  out.reason = "sampled kinematic check passed; not collision checked; NO MOTION";
  return out;
}

double retreat_sample_period(const std::vector<ArmAngles> & path_deg, double duration_sec,
  const ArmAngles & velocity_limits, const ArmAngles & acceleration_limits)
{
  if (path_deg.size() < 2 || !std::isfinite(duration_sec) || duration_sec <= 0) {
    throw std::invalid_argument("invalid retreat duration or path");
  }
  double period = duration_sec / (path_deg.size() - 1);
  for (std::size_t j = 0; j < 7; ++j) {
    if (!std::isfinite(velocity_limits[j]) || velocity_limits[j] <= 0 ||
      !std::isfinite(acceleration_limits[j]) || acceleration_limits[j] <= 0)
    {throw std::invalid_argument("invalid retreat joint rate limits");}
    double previous_step = 0.0;
    for (std::size_t i = 0; i < path_deg.size(); ++i) {
      if (!std::isfinite(path_deg[i][j])) {throw std::invalid_argument("nonfinite retreat path");}
      if (i == 0) {continue;}
      const double step = degrees_to_radians(path_deg[i][j] - path_deg[i - 1][j]);
      period = std::max(period, std::abs(step) / velocity_limits[j]);
      period = std::max(period, std::sqrt(std::abs(step - previous_step) / acceleration_limits[j]));
      previous_step = step;
    }
    period = std::max(period, std::sqrt(std::abs(previous_step) / acceleration_limits[j]));
  }
  return period;
}

std::string retreat_report(std::size_t arm, const RetreatCheck & out)
{
  std::ostringstream s;
  s << std::fixed << std::setprecision(6) << "NO_MOTION arm=" << (arm == 0 ? "left" : "right")
    << " frame=SDK_arm_base flange_offset_mm=[-" << std::setprecision(0) << out.distance_mm
    << ",0,0] reason=" << std::setprecision(6) << out.reason;
  if (!out.path_deg.empty()) {
    auto print = [&](const char * name, const auto & values) {
        s << " " << name << "=[";
        for (std::size_t i = 0; i < values.size(); ++i) {s << (i ? "," : "") << values[i];}
        s << "]";
      };
    print("start_xyz_mm", std::array<double, 3>{out.start[0][3], out.start[1][3], out.start[2][3]});
    print("target_xyz_mm", std::array<double, 3>{out.target[0][3], out.target[1][3], out.target[2][3]});
    print("start_q_deg", out.path_deg.front());
    print("last_verified_q_deg", out.path_deg.back());
    if (out.success) {
      print("verified_end_xyz_mm", std::array<double, 3>{out.solved[0][3], out.solved[1][3], out.solved[2][3]});
    }
    s << " checked_path_points=" << out.path_deg.size()
      << " max_position_error_mm=" << out.max_position_error_mm
      << " max_rotation_error_deg=" << out.max_rotation_error_deg
      << " max_joint_step_deg=" << out.max_step_deg
      << " min_clearance_to_planning_limit_deg=" << out.min_limit_clearance_deg;
  }
  return s.str();
}
}  // namespace tianji_io_teleop_bridge
