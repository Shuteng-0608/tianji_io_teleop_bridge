#ifndef TIANJI_IO_TELEOP_BRIDGE__SEND_TIMING_HPP_
#define TIANJI_IO_TELEOP_BRIDGE__SEND_TIMING_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace tianji_io_teleop_bridge
{

// No fixed-phase catch-up: the next cycle is due only a full period after completion.
class CompletionPacer
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  bool due(TimePoint now) const {return !next_ || now >= *next_;}
  void completed(TimePoint now, double period_sec)
  {
    next_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period_sec));
  }
  void reset() {next_.reset();}
private:
  std::optional<TimePoint> next_;
};

// Observation only: elapsed time since a busy response is NOT continuous SDK busy time.
struct SendTiming
{
  using TimePoint = std::chrono::steady_clock::time_point;
  static double ms(TimePoint end, TimePoint start)
  {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }
  void timer_started(TimePoint now)
  {
    if (timer_start) {
      timer_gap_ms = ms(now, *timer_start);
      max_timer_gap_ms = std::max(max_timer_gap_ms, timer_gap_ms);
    }
    timer_start = now;
  }
  void target_started(TimePoint now)
  {
    if (target_start) {
      target_attempt_gap_ms = ms(now, *target_start);
    }
    target_start = now;
  }
  std::string describe(TimePoint now) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) <<
      "timer_gap_ms=" << timer_gap_ms << " max_timer_gap_ms=" << max_timer_gap_ms <<
      " callback_age_ms=" << (timer_start ? ms(now, *timer_start) : -1.0) <<
      " feedback_sdk_ms=" << feedback_sdk_ms << " max_feedback_sdk_ms=" << max_feedback_sdk_ms <<
      " target_sdk_ms=" << target_sdk_ms << " max_target_sdk_ms=" << max_target_sdk_ms <<
      " target_attempt_gap_ms=" << target_attempt_gap_ms <<
      " last_target_attempt_age_ms=" << (target_start ? ms(now, *target_start) : -1.0) <<
      " last_target_submit_age_ms=" << (target_submitted ? ms(now, *target_submitted) : -1.0) <<
      " target_busy_total=" << busy_total;
    return out.str();
  }
  std::optional<TimePoint> timer_start, target_start, target_submitted;
  double timer_gap_ms{-1.0}, max_timer_gap_ms{0.0};
  double feedback_sdk_ms{-1.0}, max_feedback_sdk_ms{0.0};
  double target_sdk_ms{-1.0}, max_target_sdk_ms{0.0}, target_attempt_gap_ms{-1.0};
  std::uint64_t busy_total{0};
};

}  // namespace tianji_io_teleop_bridge

#endif  // TIANJI_IO_TELEOP_BRIDGE__SEND_TIMING_HPP_
