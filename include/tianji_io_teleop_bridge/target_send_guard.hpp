#ifndef TIANJI_IO_TELEOP_BRIDGE__TARGET_SEND_GUARD_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__TARGET_SEND_GUARD_HPP_

#include <chrono>
#include <cstddef>
#include <optional>

namespace tianji_io_teleop_bridge
{

enum class TargetSendResult {kSubmitted, kBusy, kError};

// Steady-clock deadline, not a retry count. Repeated busy responses never extend it.
class TargetSendGuard
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe_busy(TimePoint now)
  {
    if (!busy_since_) {
      busy_since_ = now;
    }
    ++attempts_;
  }

  bool busy() const {return busy_since_.has_value();}
  std::size_t attempts() const {return attempts_;}
  double age_sec(TimePoint now) const
  {
    return busy_since_ ? std::chrono::duration<double>(now - *busy_since_).count() : 0.0;
  }
  bool expired(TimePoint now, double timeout_sec) const
  {
    return busy() && age_sec(now) >= timeout_sec;
  }
  void reset()
  {
    busy_since_.reset();
    attempts_ = 0;
  }

private:
  std::optional<TimePoint> busy_since_;
  std::size_t attempts_{0};
};

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__TARGET_SEND_GUARD_HPP_
