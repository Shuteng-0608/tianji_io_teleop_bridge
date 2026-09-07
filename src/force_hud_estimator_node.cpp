#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include "FxRobot.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tianji_io_teleop_bridge/force_hud_input.hpp"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"

namespace tianji_io_teleop_bridge
{
using Status = diagnostic_msgs::msg::DiagnosticStatus;
using Report = diagnostic_msgs::msg::DiagnosticArray;
using Steady = std::chrono::steady_clock;

class ForceHudEstimator : public rclcpp::Node
{
public:
  ForceHudEstimator() : Node("tianji_force_estimator")
  {
    const auto topic = parameter<std::string>("state_topic", "/io_teleop/joint_states");
    const auto output = parameter<std::string>("estimate_topic", "/tianji_force_hud/estimates");
    auto config = parameter<std::string>("kinematics_config_file", "");
    const double rate = parameter<double>("estimate_rate_hz", 100.0);
    timeout_ = parameter<double>("input_timeout_sec", 0.2);
    future_tolerance_ = parameter<double>("future_tolerance_sec", 0.05);
    length_scale_ = parameter<double>("jacobian_length_scale_m", 0.1);
    max_condition_ = parameter<double>("max_scaled_condition", 200.0);
    max_residual_ = parameter<double>("max_relative_residual", 0.5);
    max_force_ = parameter<double>("max_force_norm", 1000.0);
    max_torque_ = parameter<double>("max_torque_norm", 100.0);
    confirmed_ = parameter<bool>("wrench_convention_confirmed", false);
    frames_[0] = parameter<std::string>("left_wrench_frame", "");
    frames_[1] = parameter<std::string>("right_wrench_frame", "");
    for (const double value : {rate, timeout_, future_tolerance_, length_scale_,
        max_condition_, max_residual_, max_force_, max_torque_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("HUD rates, timeouts and quality limits must be finite/positive");
      }
    }
    if (rate > 500.0 || (confirmed_ && (frames_[0].empty() || frames_[1].empty()))) {
      throw std::invalid_argument("rate exceeds 500 Hz or confirmed convention has empty frames");
    }
    if (config.empty()) {
      config = ament_index_cpp::get_package_share_directory("tianji_io_teleop_bridge") +
        "/marvin_kinematics/ccs_m6_40.MvKDCfg";
    }
    if (!kinematics_.initialize(config)) {
      throw std::runtime_error(kinematics_.last_error());
    }
    publisher_ = create_publisher<Report>(output, rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        latest_ = msg;
        received_ = Steady::now();
        ++generation_;
      });
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Read-only force estimator at %.1f Hz; no robot connection or control APIs", rate);
  }

private:
  template<class T> T parameter(const std::string & key, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(key, value, descriptor);
  }

  static void add(Status & status, const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    status.values.push_back(pair);
  }

  static void number(Status & status, const std::string & key, double value)
  {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    add(status, key, stream.str());
  }

  Status base_status(std::size_t arm) const
  {
    Status status;
    status.name = arm == 0 ? "left" : "right";
    status.hardware_id = "Marvin_M6_estimate_only";
    status.level = Status::ERROR;
    add(status, "schema_version", "1");
    add(status, "reference_point", "flange");
    add(status, "wrench_frame", frames_[arm]);
    add(status, "convention_confirmed", confirmed_ ? "true" : "false");
    return status;
  }

  Status estimate(std::size_t arm, const JointArray & q, const JointArray & tau)
  {
    auto result = base_status(arm);
    std::array<double, kArmDof> arm_q{}, arm_tau{};
    std::copy_n(q.begin() + arm * kArmDof, kArmDof, arm_q.begin());
    std::copy_n(tau.begin() + arm * kArmDof, kArmDof, arm_tau.begin());
    CartesianDisturbance wrench{};
    if (!kinematics_.joint_disturbance_to_wrench(arm, arm_q, arm_tau, wrench)) {
      result.message = "SDK estimate failed";
      add(result, "detail", kinematics_.last_error());
      return result;
    }
    FX_Jacobi sdk_jacobian{};
    if (FX_Robot_Kine_Jacb(static_cast<FX_INT32L>(arm), arm_q.data(), &sdk_jacobian) == FX_FALSE) {
      result.message = "Jacobian unavailable";
      return result;
    }
    Eigen::Matrix<double, 6, 7> jacobian;
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 7; ++col) {
        jacobian(row, col) = sdk_jacobian.m_Jcb[row][col];
      }
    }
    if (!jacobian.allFinite()) {
      result.message = "non-finite Jacobian";
      return result;
    }
    auto scaled = jacobian;
    scaled.topRows<3>() /= length_scale_;
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(scaled);
    const auto singular = svd.singularValues();
    const double condition = singular(5) > 1e-12 ? singular(0) / singular(5) :
      std::numeric_limits<double>::infinity();
    const Eigen::Map<const Eigen::Matrix<double, 6, 1>> w(wrench.data());
    const Eigen::Map<const Eigen::Matrix<double, 7, 1>> t(arm_tau.data());
    const double residual = (jacobian.transpose() * w - t).norm() / std::max(t.norm(), 0.1);
    number(result, "scaled_condition", condition);
    number(result, "relative_residual", residual);
    // Raw values remain available for audit even when quality checks reject the estimate.
    const std::array<std::string, 6> axes{"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
    for (std::size_t i = 0; i < 6; ++i) {number(result, "raw_" + axes[i], wrench[i]);}
    if (!std::isfinite(condition) || condition > max_condition_) {
      result.message = "ill-conditioned Jacobian";
    } else if (!std::isfinite(residual) || residual > max_residual_) {
      result.message = "joint disturbance not explained by end wrench";
    } else if (w.head<3>().norm() > max_force_ || w.tail<3>().norm() > max_torque_) {
      result.message = "estimate exceeds plausibility cap";
    } else {
      result.level = confirmed_ ? Status::OK : Status::WARN;
      result.message = confirmed_ ? "ESTIMATED (not safety-rated)" : "UNVERIFIED units/frame/sign";
    }
    return result;
  }

  void tick()
  {
    std::string reason;
    Report report;
    if (!latest_) {
      reason = "WAITING for joint_states";
    } else {
      report.header = latest_->header;
      // The array contains two wrench frames, not the input JointState frame.
      report.header.frame_id.clear();
      const rclcpp::Time stamp(latest_->header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (stamp.nanoseconds() <= 0 || age < -future_tolerance_) {
        reason = "invalid/future source stamp";
      } else if (age > timeout_ ||
        std::chrono::duration<double>(Steady::now() - received_).count() > timeout_)
      {
        reason = "STALE joint_states";
      } else if (generation_ == processed_) {
        publisher_->publish(cached_);
        return;
      } else {
        JointArray q{}, tau{};
        if (parse_force_hud_input(*latest_, q, tau, reason)) {
          report.status = {estimate(0, q, tau), estimate(1, q, tau)};
        }
      }
    }
    if (!reason.empty()) {
      for (std::size_t arm = 0; arm < 2; ++arm) {
        auto status = base_status(arm);
        status.message = reason;
        report.status.push_back(status);
      }
    }
    cached_ = report;
    processed_ = generation_;
    publisher_->publish(report);
  }

  MarvinKinematicsAdapter kinematics_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_;
  Steady::time_point received_{};
  Report cached_;
  uint64_t generation_{0}, processed_{0};
  double timeout_, future_tolerance_, length_scale_, max_condition_, max_residual_;
  double max_force_, max_torque_;
  bool confirmed_;
  std::array<std::string, 2> frames_;
  rclcpp::Publisher<Report>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace tianji_io_teleop_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<tianji_io_teleop_bridge::ForceHudEstimator>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("tianji_force_estimator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
