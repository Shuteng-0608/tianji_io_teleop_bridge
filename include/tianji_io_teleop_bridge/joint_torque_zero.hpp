#ifndef TIANJI_IO_TELEOP_BRIDGE__JOINT_TORQUE_ZERO_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__JOINT_TORQUE_ZERO_HPP_

#include <deque>
#include <cstdint>
#include <iomanip>
#include "tianji_io_teleop_bridge/joint_model.hpp"

namespace tianji_io_teleop_bridge
{

// Residual external-torque offset only. No gravity or impedance compensation here.
// A stationary, stable window cannot establish absence of contact: the operator must.
class JointTorqueZero
{
public:
  struct Settings
  {
    double window_sec{1.0};
    double max_gap_sec{0.2};
    double max_velocity_rad_s{0.01};
    double max_position_range_rad{0.005};
    double max_std_nm{0.05};
    std::size_t min_samples{20};
  };
  using ArmArray = std::array<double, kArmDof>;
  struct Sample
  {
    double time;
    int32_t frame;
    ArmArray q, tau;
  };

  Settings settings;

  void observe(std::size_t arm, double time, int32_t frame,
    const ArmArray & q, const ArmArray & dq, const ArmArray & tau)
  {
    auto & samples = samples_[arm];
    const bool repeated = last_frames_[arm] && *last_frames_[arm] == frame;
    last_frames_[arm] = frame;
    for (std::size_t j = 0; j < kArmDof; ++j) {
      if (!std::isfinite(q[j]) || !std::isfinite(dq[j]) || !std::isfinite(tau[j]) ||
        std::abs(dq[j]) > settings.max_velocity_rad_s)
      {
        samples.clear();
        return;
      }
    }
    if (!samples.empty()) {
      if (time < samples.back().time || time - samples.back().time > settings.max_gap_sec) {
        samples.clear();
      }
    }
    if (repeated) {return;}  // A repeated SDK frame never refreshes freshness, even after a gap.
    samples.push_back({time, frame, q, tau});
    while (!samples.empty() && time - samples.front().time > settings.window_sec) {
      samples.pop_front();
    }
  }

  // Validate every requested arm before changing any bias. Always average RAW tau.
  bool capture(const std::array<bool, 2> & selected, double time, std::string & reason)
  {
    auto candidate = bias_;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (!selected[arm]) {continue;}
      const auto & samples = samples_[arm];
      const std::string side = arm == 0 ? "left: " : "right: ";
      if (samples.size() < settings.min_samples || time < samples.back().time ||
        time - samples.back().time > settings.max_gap_sec ||
        samples.back().time - samples.front().time < 0.8 * settings.window_sec)
      {
        reason = side + "need a fresh stationary window of distinct controller frames";
        return false;
      }
      for (std::size_t j = 0; j < kArmDof; ++j) {
        double mean = 0.0, m2 = 0.0, count = 0.0;
        double qmin = samples.front().q[j], qmax = qmin;
        for (const auto & sample : samples) {
          const double delta = sample.tau[j] - mean;
          mean += delta / ++count;
          m2 += delta * (sample.tau[j] - mean);
          qmin = std::min(qmin, sample.q[j]);
          qmax = std::max(qmax, sample.q[j]);
        }
        const double torque_std = std::sqrt(std::max(0.0, m2 / count));
        const double position_range = qmax - qmin;
        const bool invalid = !std::isfinite(mean) || !std::isfinite(m2);
        const bool torque_unstable = torque_std > settings.max_std_nm;
        const bool position_unstable = position_range > settings.max_position_range_rad;
        if (invalid || torque_unstable || position_unstable)
        {
          std::ostringstream detail;
          detail << std::setprecision(6) << side << "unstable torque/position at joint " <<
            j + 1 << "; failed=" << (invalid ? "nonfinite_statistics" :
            (torque_unstable && position_unstable ? "torque_std,position_range" :
            (torque_unstable ? "torque_std" : "position_range"))) <<
            "; torque_std_nm=" << torque_std << " (limit=" << settings.max_std_nm <<
            "); position_range_rad=" << position_range << " (limit=" <<
            settings.max_position_range_rad << "); samples=" << samples.size() <<
            "; span_sec=" << samples.back().time - samples.front().time;
          reason = detail.str();
          return false;
        }
        candidate[arm * kArmDof + j] = mean;
      }
    }
    bias_ = candidate;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (selected[arm]) {active_[arm] = true; ++revision_[arm];}
    }
    reason = "joint torque bias captured; corrected effort and wrench apply from next feedback";
    return true;
  }

  void clear_window(std::size_t arm) {samples_[arm].clear();}
  void clear(std::size_t arm)
  {
    clear_window(arm);
    last_frames_[arm].reset();
    std::fill_n(bias_.begin() + arm * kArmDof, kArmDof, 0.0);
    active_[arm] = false;
    ++revision_[arm];
  }
  JointArray correct(const JointArray & raw) const
  {
    auto corrected = raw;
    for (std::size_t j = 0; j < kJointCount; ++j) {corrected[j] -= bias_[j];}
    return corrected;
  }
  const JointArray & bias() const {return bias_;}
  bool active(std::size_t arm) const {return active_[arm];}
  uint64_t revision(std::size_t arm) const {return revision_[arm];}

private:
  std::array<std::deque<Sample>, 2> samples_;
  std::array<std::optional<int32_t>, 2> last_frames_;
  JointArray bias_{};
  std::array<bool, 2> active_{};
  std::array<uint64_t, 2> revision_{};
};

}  // namespace tianji_io_teleop_bridge
#endif  // TIANJI_IO_TELEOP_BRIDGE__JOINT_TORQUE_ZERO_HPP_
