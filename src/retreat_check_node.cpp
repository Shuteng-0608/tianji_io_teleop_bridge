#include <chrono>
#include <optional>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tianji_io_teleop_bridge/marvin_kinematics_adapter.hpp"
#include "tianji_io_teleop_bridge/retreat_check.hpp"

namespace tianji_io_teleop_bridge
{
class RetreatCheckNode : public rclcpp::Node
{
public:
  RetreatCheckNode() : Node("tianji_retreat_check")
  {
    auto config = declare_parameter<std::string>("kinematics_config_file", "");
    margin_ = declare_parameter<double>("joint_limit_safety_margin_rad", 0.005);
    if (!std::isfinite(margin_) || margin_ < 0 || margin_ > 0.1) {
      throw std::invalid_argument("planning margin must be in [0,0.1] rad");
    }
    if (config.empty()) {
      config = ament_index_cpp::get_package_share_directory("tianji_io_teleop_bridge") +
        "/marvin_kinematics/ccs_m6_40.MvKDCfg";
    }
    if (!kinematics_.initialize(config)) {throw std::runtime_error(kinematics_.last_error());}
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("state_topic", "/io_teleop/joint_states"),
      rclcpp::QoS(1).best_effort(), [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        const auto validated = validate_joint_command(*msg, std::nullopt, 1.0, 0.0);
        const double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (!validated.valid || age < -0.05 || age > 0.2 ||
          (stamp_ && rclcpp::Time(msg->header.stamp).nanoseconds() <= *stamp_))
        {q_.reset(); return;}
        q_ = validated.positions_rad;
        stamp_ = rclcpp::Time(msg->header.stamp).nanoseconds();
        received_ = std::chrono::steady_clock::now();
      });
    for (std::size_t arm = 0; arm < 2; ++arm) {
      services_[arm] = create_service<std_srvs::srv::Trigger>(
        arm == 0 ? "~/retreat_left" : "~/retreat_right",
        [this, arm](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (!q_ || !stamp_ ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() > 0.2 ||
            (now().nanoseconds() - *stamp_) / 1e9 > 0.2 ||
            (now().nanoseconds() - *stamp_) / 1e9 < -0.05)
          {response->message = "NO_MOTION: fresh valid 14-joint feedback unavailable"; return;}
          ArmAngles q{};
          for (int j = 0; j < 7; ++j) {q[j] = radians_to_degrees((*q_)[arm * 7 + j]);}
          const auto result = check_retreat(arm, q, margin_);
          response->success = result.success;
          response->message = retreat_report(arm, result);
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        });
    }
    RCLCPP_INFO(get_logger(), "COMPUTATION ONLY: libKine, no motor SDK. Both arms retreat -X 30 mm.");
  }
private:
  MarvinKinematicsAdapter kinematics_;
  double margin_{};
  std::optional<JointArray> q_;
  std::optional<int64_t> stamp_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, 2> services_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<tianji_io_teleop_bridge::RetreatCheckNode>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("retreat_check"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
