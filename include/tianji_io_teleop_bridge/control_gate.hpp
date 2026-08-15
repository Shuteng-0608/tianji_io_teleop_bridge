#ifndef TIANJI_IO_TELEOP_BRIDGE__CONTROL_GATE_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__CONTROL_GATE_HPP_

#include <cstddef>
#include <string>

namespace tianji_io_teleop_bridge
{

enum class ControlState
{
  kMonitor,
  kPreparingParameters,
  kConfiguringImpedance,
  kWaitingForAlignment,
  kActive,
  kSoftLimitHold,
  kHold,
  kFault
};

inline const char * control_state_name(ControlState state)
{
  switch (state) {
    case ControlState::kMonitor:
      return "MONITOR";
    case ControlState::kPreparingParameters:
      return "PREPARING_PARAMETERS";
    case ControlState::kConfiguringImpedance:
      return "CONFIGURING_IMPEDANCE";
    case ControlState::kWaitingForAlignment:
      return "WAITING_FOR_ALIGNMENT";
    case ControlState::kActive:
      return "ACTIVE";
    case ControlState::kSoftLimitHold:
      return "SOFT_LIMIT_HOLD";
    case ControlState::kHold:
      return "HOLD";
    case ControlState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

class ControlGate
{
public:
  explicit ControlGate(
    std::size_t required_alignment_frames,
    std::size_t required_soft_limit_recovery_frames = 20)
  : required_alignment_frames_(required_alignment_frames),
    required_soft_limit_recovery_frames_(required_soft_limit_recovery_frames)
  {
  }

  void request_enable()
  {
    state_ = ControlState::kPreparingParameters;
    aligned_frames_ = 0;
    reason_ = "preparing impedance parameters without switching control mode";
  }

  void mark_waiting_for_alignment()
  {
    state_ = ControlState::kWaitingForAlignment;
    aligned_frames_ = 0;
    reason_ = "joint impedance active; waiting for fresh aligned command";
  }

  bool observe_alignment(bool ready)
  {
    if (state_ != ControlState::kWaitingForAlignment) {
      return false;
    }
    aligned_frames_ = ready ? aligned_frames_ + 1 : 0;
    return aligned_frames_ >= required_alignment_frames_;
  }

  void mark_configuring()
  {
    state_ = ControlState::kConfiguringImpedance;
    reason_ = "waiting for Marvin impedance mode confirmation";
  }

  void mark_active()
  {
    state_ = ControlState::kActive;
    soft_limit_recovery_frames_ = 0;
    reason_ = "control active";
  }

  void enter_soft_limit_hold(const std::string & reason)
  {
    state_ = ControlState::kSoftLimitHold;
    soft_limit_recovery_frames_ = 0;
    reason_ = reason;
  }

  bool observe_soft_limit_recovery(bool ready)
  {
    if (state_ != ControlState::kSoftLimitHold) {
      return false;
    }
    soft_limit_recovery_frames_ = ready ? soft_limit_recovery_frames_ + 1 : 0;
    return soft_limit_recovery_frames_ >= required_soft_limit_recovery_frames_;
  }

  void enter_hold(const std::string & reason)
  {
    state_ = ControlState::kHold;
    reason_ = reason;
  }

  void enter_fault(const std::string & reason)
  {
    state_ = ControlState::kFault;
    reason_ = reason;
  }

  void disarm(const std::string & reason = "control disabled")
  {
    state_ = ControlState::kMonitor;
    aligned_frames_ = 0;
    soft_limit_recovery_frames_ = 0;
    reason_ = reason;
  }

  ControlState state() const {return state_;}
  std::size_t aligned_frames() const {return aligned_frames_;}
  std::size_t soft_limit_recovery_frames() const {return soft_limit_recovery_frames_;}
  const std::string & reason() const {return reason_;}

private:
  std::size_t required_alignment_frames_{1};
  std::size_t required_soft_limit_recovery_frames_{1};
  std::size_t aligned_frames_{0};
  std::size_t soft_limit_recovery_frames_{0};
  ControlState state_{ControlState::kMonitor};
  std::string reason_{"monitor only"};
};

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__CONTROL_GATE_HPP_
