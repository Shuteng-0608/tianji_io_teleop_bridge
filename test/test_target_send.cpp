#include <cstdlib>
#include <filesystem>
#include <thread>
#include "gtest/gtest.h"

// Exercise the production node without linking libMarvinSDK or spinning its executor.
#define main unused_bridge_main
#include "../src/marvin_teleop_bridge_node.cpp"
#undef main

namespace fake_sdk
{
bool busy = false;
bool stage_ok = true;
bool send_ok = true;
bool feedback_ok = true;
bool advance_feedback_frames = false;
bool ft_select_ok = true;
bool ft_channel_echo = true;
int ft_selections = 0;
int clears = 0;
int stages = 0;
int sends = 0;
int idles = 0;
int kd_writes = 0;
int imp_type_writes = 0;
std::array<int, 2> modes{{-1, -1}};
std::array<int, 2> speed_writes{};
std::array<int, 2> velocity_ratios{}, acceleration_ratios{};
std::array<int, 2> writes{};
std::array<int, 2> tool_writes{};
std::array<tianji_io_teleop_bridge::ToolConfiguration, 2> tool_values{};
int read_delay_ms = 0;
int send_delay_ms = 0;
DCSS feedback_value{};
std::chrono::steady_clock::time_point send_finished{};
std::array<double, 7> left{}, right{};
void reset()
{
  busy = false;
  stage_ok = send_ok = feedback_ok = true;
  advance_feedback_frames = false;
  ft_select_ok = ft_channel_echo = true;
  ft_selections = 0;
  clears = stages = sends = idles = 0;
  kd_writes = imp_type_writes = 0;
  modes.fill(-1);
  speed_writes.fill(0);
  velocity_ratios.fill(0);
  acceleration_ratios.fill(0);
  writes.fill(0);
  tool_writes.fill(0);
  tool_values = {};
  read_delay_ms = send_delay_ms = 0;
  send_finished = {};
  feedback_value = DCSS{};
  feedback_value.m_Out[0].m_OutFrameSerial = 1;
  feedback_value.m_Out[1].m_OutFrameSerial = 1;
  left.fill(0);
  right.fill(0);
}
}  // namespace fake_sdk

extern "C"
{
bool OnLinkTo(unsigned char, unsigned char, unsigned char, unsigned char) {return true;}
bool OnRelease() {return true;}
int CheckSDKTypeCompat(int *order) {*order = 0; return 0;}
long OnGetSDKVersion() {return 100343014;}
bool OnSetUserSpcfData_B(long channel)
{
  ++fake_sdk::ft_selections;
  if (channel != 216) {return false;}
  if (fake_sdk::ft_select_ok && fake_sdk::ft_channel_echo) {
    fake_sdk::feedback_value.m_Out[1].m_EST_Joint_Firc[0] = channel;
  }
  return fake_sdk::ft_select_ok;
}
bool OnGetBuf(DCSS *feedback)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(fake_sdk::read_delay_ms));
  if (fake_sdk::advance_feedback_frames) {
    ++fake_sdk::feedback_value.m_Out[0].m_OutFrameSerial;
    ++fake_sdk::feedback_value.m_Out[1].m_OutFrameSerial;
  }
  *feedback = fake_sdk::feedback_value;
  return fake_sdk::feedback_ok;
}
bool OnClearSet() {++fake_sdk::clears; return !fake_sdk::busy;}
bool OnSetJointCmdPos_A(double *q)
{
  ++fake_sdk::writes[0];
  ++fake_sdk::stages;
  std::copy_n(q, 7, fake_sdk::left.begin());
  return fake_sdk::stage_ok;
}
bool OnSetJointCmdPos_B(double *q)
{
  ++fake_sdk::writes[1];
  ++fake_sdk::stages;
  std::copy_n(q, 7, fake_sdk::right.begin());
  return fake_sdk::stage_ok;
}
bool OnSetSend()
{
  ++fake_sdk::sends;
  std::this_thread::sleep_for(std::chrono::milliseconds(fake_sdk::send_delay_ms));
  fake_sdk::send_finished = std::chrono::steady_clock::now();
  return fake_sdk::send_ok;
}
bool OnSetTargetState_A(int state) {++fake_sdk::writes[0]; fake_sdk::modes[0] = state; if (state == ARM_STATE_IDLE) {++fake_sdk::idles;} return fake_sdk::stage_ok;}
bool OnSetTargetState_B(int state) {++fake_sdk::writes[1]; fake_sdk::modes[1] = state; if (state == ARM_STATE_IDLE) {++fake_sdk::idles;} return fake_sdk::stage_ok;}
bool OnSetImpType_A(int) {++fake_sdk::writes[0]; ++fake_sdk::imp_type_writes; return true;}
bool OnSetImpType_B(int) {++fake_sdk::writes[1]; ++fake_sdk::imp_type_writes; return true;}
bool OnSetJointLmt_A(int velocity, int acceleration)
{
  ++fake_sdk::writes[0]; ++fake_sdk::speed_writes[0];
  fake_sdk::velocity_ratios[0] = velocity;
  fake_sdk::acceleration_ratios[0] = acceleration;
  return fake_sdk::stage_ok;
}
bool OnSetJointLmt_B(int velocity, int acceleration)
{
  ++fake_sdk::writes[1]; ++fake_sdk::speed_writes[1];
  fake_sdk::velocity_ratios[1] = velocity;
  fake_sdk::acceleration_ratios[1] = acceleration;
  return fake_sdk::stage_ok;
}
bool OnSetJointKD_A(double *, double *) {++fake_sdk::writes[0]; ++fake_sdk::kd_writes; return true;}
bool OnSetJointKD_B(double *, double *) {++fake_sdk::writes[1]; ++fake_sdk::kd_writes; return true;}
bool OnSetTool_A(double *kine, double *dyn)
{
  ++fake_sdk::writes[0]; ++fake_sdk::tool_writes[0];
  std::copy_n(kine, 6, fake_sdk::tool_values[0].kine.begin());
  std::copy_n(dyn, 10, fake_sdk::tool_values[0].dyn.begin());
  return fake_sdk::stage_ok;
}
bool OnSetTool_B(double *kine, double *dyn)
{
  ++fake_sdk::writes[1]; ++fake_sdk::tool_writes[1];
  std::copy_n(kine, 6, fake_sdk::tool_values[1].kine.begin());
  std::copy_n(dyn, 10, fake_sdk::tool_values[1].dyn.begin());
  return fake_sdk::stage_ok;
}
}

namespace tianji_io_teleop_bridge
{
using namespace std::chrono_literals;

TEST(TargetSendGuard, DeadlineDoesNotSlideAndResetStartsANewEpisode)
{
  TargetSendGuard guard;
  const TargetSendGuard::TimePoint t{};
  EXPECT_FALSE(guard.expired(t + 1s, 0.03));
  guard.observe_busy(t);
  guard.observe_busy(t + 20ms);
  EXPECT_EQ(guard.attempts(), 2U);
  EXPECT_FALSE(guard.expired(t + 29ms, 0.03));
  EXPECT_TRUE(guard.expired(t + 30ms, 0.03));
  guard.reset();
  EXPECT_FALSE(guard.busy());
  EXPECT_EQ(guard.attempts(), 0U);
  guard.observe_busy(t + 1s);
  EXPECT_FALSE(guard.expired(t + 1010ms, 0.03));
}

TEST(TargetSendAdapter, BusyDoesNotStageDiscardOrSendAndRecoveryMapsBothArms)
{
  fake_sdk::reset();
  MarvinSdkAdapter sdk;
  JointArray q{};
  EXPECT_EQ(sdk.send_joint_targets(EnabledArms::kBoth, q), TargetSendResult::kError);
  ASSERT_TRUE(sdk.connect({127, 0, 0, 1}));  // Stub only: no network.
  EXPECT_EQ(sdk.send_joint_targets(EnabledArms::kNone, q), TargetSendResult::kError);
  fake_sdk::busy = true;
  EXPECT_EQ(sdk.send_joint_targets(EnabledArms::kBoth, q), TargetSendResult::kBusy);
  EXPECT_EQ(fake_sdk::clears, 1);
  EXPECT_EQ(fake_sdk::stages, 0);
  EXPECT_EQ(fake_sdk::sends, 0);
  fake_sdk::busy = false;
  for (std::size_t i = 0; i < q.size(); ++i) {q[i] = static_cast<double>(i);}
  EXPECT_EQ(sdk.send_joint_targets(EnabledArms::kBoth, q), TargetSendResult::kSubmitted);
  EXPECT_TRUE(sdk.last_error().empty());
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(fake_sdk::left[i], q[i]);
    EXPECT_DOUBLE_EQ(fake_sdk::right[i], q[i + 7]);
  }
  fake_sdk::stage_ok = false;
  EXPECT_EQ(sdk.send_joint_targets(EnabledArms::kBoth, q), TargetSendResult::kError);
  fake_sdk::stage_ok = true;
  fake_sdk::send_ok = false;
  EXPECT_EQ(sdk.send_joint_targets(EnabledArms::kBoth, q), TargetSendResult::kError);
}

TEST(PositionAdapter, SelectsOnlyRequestedArmsWithoutKdImpedanceTypeOrTargets)
{
  for (const auto arms : {EnabledArms::kLeft, EnabledArms::kRight, EnabledArms::kBoth}) {
    fake_sdk::reset();
    MarvinSdkAdapter sdk;
    EXPECT_FALSE(sdk.enter_position_mode(arms));
    ASSERT_TRUE(sdk.connect({127, 0, 0, 1}));
    EXPECT_FALSE(sdk.configure_position_parameters(arms, 0, 10));
    EXPECT_FALSE(sdk.enter_position_mode(EnabledArms::kNone));
    fake_sdk::busy = true;
    EXPECT_FALSE(sdk.configure_position_parameters(arms, 10, 10));
    EXPECT_FALSE(sdk.enter_position_mode(arms));
    EXPECT_EQ(fake_sdk::sends, 0);
    EXPECT_EQ(fake_sdk::writes, (std::array<int, 2>{0, 0}));
    fake_sdk::busy = false;
    ASSERT_TRUE(sdk.configure_position_parameters(arms, 10, 20));
    ASSERT_TRUE(sdk.enter_position_mode(arms));
    EXPECT_EQ(fake_sdk::modes[0], left_enabled(arms) ? ARM_STATE_POSITION : -1);
    EXPECT_EQ(fake_sdk::modes[1], right_enabled(arms) ? ARM_STATE_POSITION : -1);
    EXPECT_EQ(fake_sdk::speed_writes[0], left_enabled(arms) ? 1 : 0);
    EXPECT_EQ(fake_sdk::speed_writes[1], right_enabled(arms) ? 1 : 0);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::kd_writes, 0);
    EXPECT_EQ(fake_sdk::imp_type_writes, 0);
    EXPECT_EQ(fake_sdk::sends, 2);
    fake_sdk::stage_ok = false;
    EXPECT_FALSE(sdk.enter_position_mode(arms));
    EXPECT_FALSE(sdk.configure_position_parameters(arms, 10, 20));
    EXPECT_EQ(fake_sdk::sends, 2);
    fake_sdk::stage_ok = true;
    fake_sdk::send_ok = false;
    EXPECT_FALSE(sdk.enter_position_mode(arms));
    EXPECT_FALSE(sdk.configure_position_parameters(arms, 10, 20));
  }
}

TEST(CompletionPacer, DelayedCyclesNeverCatchUpOrRetryBeforeAFullPeriod)
{
  CompletionPacer pacer;
  const CompletionPacer::TimePoint t{};
  EXPECT_TRUE(pacer.due(t));
  pacer.completed(t + 9ms, 0.01);
  EXPECT_FALSE(pacer.due(t + 10ms));  // Old fixed phase would execute again after only 1 ms.
  EXPECT_FALSE(pacer.due(t + 18ms));
  EXPECT_TRUE(pacer.due(t + 19ms));
  // A long pause skips all missed slots; it does not schedule a burst on recovery.
  EXPECT_TRUE(pacer.due(t + 87ms));
  pacer.completed(t + 90ms, 0.01);
  EXPECT_FALSE(pacer.due(t + 99ms));
  EXPECT_TRUE(pacer.due(t + 100ms));
  pacer.reset();
  EXPECT_TRUE(pacer.due(t));
}

TEST(SendTiming, DistinguishesTimerGapSdkDurationAndSubmissionAge)
{
  SendTiming timing;
  const SendTiming::TimePoint t{};
  EXPECT_NE(timing.describe(t).find("last_target_submit_age_ms=-1.000"), std::string::npos);
  timing.timer_started(t);
  timing.target_started(t + 1ms);
  timing.target_submitted = t + 2ms;
  timing.timer_started(t + 68ms);
  timing.target_started(t + 69ms);
  timing.target_sdk_ms = 0.005;
  const auto description = timing.describe(t + 70ms);
  EXPECT_DOUBLE_EQ(timing.timer_gap_ms, 68.0);
  EXPECT_DOUBLE_EQ(timing.target_attempt_gap_ms, 68.0);
  EXPECT_NE(description.find("last_target_submit_age_ms=68.000"), std::string::npos);
  EXPECT_NE(description.find("target_sdk_ms=0.005"), std::string::npos);
}

class TargetSendTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_DOMAIN_ID", "231", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    fake_sdk::reset();
    // Existing soft-hold tests explicitly exercise the optional legacy policy.
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("teleop_soft_limits_enabled", true)});
    node = std::make_shared<MarvinTeleopBridgeNode>(options);
    node->timer_->cancel();
    ASSERT_TRUE(node->sdk_.connect({127, 0, 0, 1}));
    node->enabled_arms_ = EnabledArms::kBoth;
    node->allow_motion_ = true;
    node->control_engaged_ = true;
    node->control_gate_.mark_active();
  }
  void TearDown() override {node.reset();}

  void check_backup_limit_parameters()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("connect_on_start", false)});
    auto backup_node = std::make_shared<MarvinTeleopBridgeNode>(options);
    backup_node->timer_->cancel();
    EXPECT_FALSE(backup_node->teleop_soft_limits_enabled_);
    EXPECT_DOUBLE_EQ(backup_node->teleop_joint_limit_extension_rad_, 0.02);
    EXPECT_FALSE(backup_node->set_parameter(
      rclcpp::Parameter("teleop_soft_limits_enabled", true)).successful);
    EXPECT_FALSE(backup_node->set_parameter(
      rclcpp::Parameter("teleop_joint_limit_extension_rad", 0.01)).successful);
    for (const double extension : {-0.01, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
    {
      options.parameter_overrides({rclcpp::Parameter("connect_on_start", false),
        rclcpp::Parameter("teleop_joint_limit_extension_rad", extension)});
      EXPECT_THROW(std::make_shared<MarvinTeleopBridgeNode>(options), std::invalid_argument);
    }
    EXPECT_EQ(fake_sdk::sends, 0);
  }

  void check_backup_forwards_boundaries(JointControlMode mode, double extension)
  {
    node->teleop_soft_limits_enabled_ = false;
    node->teleop_joint_limit_extension_rad_ = extension;
    node->operation_control_mode_ = mode;
    node->control_purpose_ = ControlPurpose::kTeleoperation;
    node->max_command_step_rad_ = 10.0;
    node->have_feedback_ = true;
    auto feedback = wait_feedback();
    if (mode == JointControlMode::kPosition) {
      for (auto & state : feedback.m_State) {state.m_CurState = ARM_STATE_POSITION;}
    }
    // Both URDF endpoints and the exact outer guard must reach the SDK unchanged.
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      for (const double extra : {0.0, node->teleop_limit_extension()}) {
        for (const double endpoint : {lower_limits_rad()[joint] - extra,
          upper_limits_rad()[joint] + extra})
        {
          auto target = node->initial_pose_rad_;
          target[joint] = endpoint;
          node->latest_feedback_rad_ = target;
          node->last_sent_target_rad_ = target;
          receive(target);
          const int before = fake_sdk::sends;
          node->update_control(feedback, target, std::chrono::steady_clock::now());
          ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
          EXPECT_EQ(fake_sdk::sends, before + 1);
          EXPECT_EQ(node->last_sent_target_rad_, target);
          EXPECT_TRUE(node->enabled_teleop_targets_within_limits(target, true));
        }
      }
    }
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_backup_rejects_outside_guard()
  {
    node->teleop_soft_limits_enabled_ = false;
    node->max_command_step_rad_ = 10.0;
    const auto pose = node->initial_pose_rad_;
    node->have_feedback_ = true;
    node->latest_feedback_rad_ = pose;
    node->last_sent_target_rad_ = pose;
    ASSERT_TRUE(node->submit_joint_target(pose));
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      for (const double endpoint : {
        lower_limits_rad()[joint] - node->teleop_limit_extension() - 0.001,
        upper_limits_rad()[joint] + node->teleop_limit_extension() + 0.001})
      {
        node->control_gate_.mark_active();
        auto target = pose;
        target[joint] = endpoint;
        const int before = fake_sdk::sends;
        receive(target);
        ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
        EXPECT_NE(node->control_gate_.reason().find("joint limit violation"), std::string::npos);
        EXPECT_EQ(node->command_wait_target_rad_, pose);
        node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
        EXPECT_EQ(fake_sdk::sends, before);
      }
    }
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_backup_previously_frozen_targets()
  {
    node->teleop_soft_limits_enabled_ = false;
    node->enabled_arms_ = EnabledArms::kRight;
    auto feedback_pose = node->initial_pose_rad_;
    feedback_pose[8] = -1.666;
    node->latest_feedback_rad_ = feedback_pose;
    node->have_feedback_ = true;
    node->last_sent_target_rad_ = feedback_pose;
    for (const double logged_target : {-1.6715, -1.6726}) {
      auto target = feedback_pose;
      target[8] = logged_target;
      receive(target);
      node->update_control(wait_feedback(), feedback_pose, std::chrono::steady_clock::now());
      ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
      EXPECT_DOUBLE_EQ(fake_sdk::right[1], radians_to_degrees(logged_target));
    }
    EXPECT_EQ(fake_sdk::sends, 2);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_backup_keeps_input_validation()
  {
    node->teleop_soft_limits_enabled_ = false;
    const auto pose = node->initial_pose_rad_;
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    node->last_sent_target_rad_ = pose;
    receive(pose);
    auto jumped = pose;
    jumped[6] += 0.06;
    receive(jumped);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_NE(node->control_gate_.reason().find("jump"), std::string::npos);
    auto nonfinite = pose;
    nonfinite[8] = std::numeric_limits<double>::quiet_NaN();
    receive(nonfinite);
    EXPECT_FALSE(node->last_valid_command_time_);
    auto malformed = std::make_shared<sensor_msgs::msg::JointState>();
    malformed->name = {"Joint1_R"};
    malformed->position = {0.0};
    node->on_joint_command(malformed);
    EXPECT_FALSE(node->last_valid_command_time_);
    EXPECT_EQ(node->rejected_command_count_, 3U);
    EXPECT_EQ(fake_sdk::sends, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_backup_aligns_and_recovers_at_urdf_boundary()
  {
    node->teleop_soft_limits_enabled_ = false;
    node->control_purpose_ = ControlPurpose::kTeleoperation;
    auto pose = node->initial_pose_rad_;
    pose[8] = lower_limits_rad()[8];
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    node->last_sent_target_rad_ = pose;
    node->control_gate_.mark_waiting_for_alignment();
    for (int i = 0; i < 20; ++i) {
      receive(pose);
      node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    }
    ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
    ASSERT_TRUE(node->submit_joint_target(pose));
    node->begin_command_wait("test timeout at URDF boundary");
    EXPECT_DOUBLE_EQ(node->displayed_alignment_margin(), 0.0);
    for (int i = 0; i < 19; ++i) {
      receive(pose);
      node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    }
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    receive(pose);
    node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::sends, 1);  // Alignment/recovery do not submit targets.
    // Extension applies only to teleoperation, never automatic initial-pose/retreat checks.
    pose[8] -= 0.005;
    EXPECT_TRUE(node->enabled_teleop_targets_within_limits(pose));
    EXPECT_FALSE(node->enabled_targets_within_margin(pose, node->joint_limit_safety_margin_rad_));
    for (const auto purpose : {ControlPurpose::kInitialPose, ControlPurpose::kRetreatLeft,
      ControlPurpose::kRetreatRight})
    {
      node->control_purpose_ = purpose;
      EXPECT_DOUBLE_EQ(node->displayed_limit_extension(), 0.0);
      EXPECT_DOUBLE_EQ(node->displayed_alignment_margin(), node->joint_limit_safety_margin_rad_);
    }
  }

  void check_mode_parameter()
  {
    EXPECT_EQ(node->teleop_control_mode_, JointControlMode::kJointImpedance);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("teleop_control_mode", "position")).successful);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("teleop_control_mode", "position"),
      rclcpp::Parameter("connect_on_start", false)});
    auto position_node = std::make_shared<MarvinTeleopBridgeNode>(options);
    position_node->timer_->cancel();
    EXPECT_EQ(position_node->teleop_control_mode_, JointControlMode::kPosition);
    EXPECT_EQ(position_node->operation_control_mode_, JointControlMode::kJointImpedance);
    EXPECT_EQ(fake_sdk::sends, 0);
    options.parameter_overrides({rclcpp::Parameter("teleop_control_mode", "positoin")});
    EXPECT_THROW(std::make_shared<MarvinTeleopBridgeNode>(options), std::invalid_argument);
  }

  void prepare_position_monitor(EnabledArms arms = EnabledArms::kBoth)
  {
    node->teleop_control_mode_ = JointControlMode::kPosition;
    node->control_engaged_ = false;
    node->control_gate_.disarm();
    node->enabled_arms_ = arms;
    node->sdk_control_compatibility_confirmed_ = node->tool_dynamics_confirmed_ = true;
    node->have_feedback_ = true;
    node->latest_feedback_rad_ = node->initial_pose_rad_;
    node->latest_dcss_ = wait_feedback();
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    node->frame_advanced_once_.fill(true);
    node->last_feedback_advance_.fill(node->last_feedback_receive_);
    for (int arm = 0; arm < 2; ++arm) {
      node->latest_dcss_.m_State[arm].m_CurState = ARM_STATE_IDLE;
      node->latest_dcss_.m_Out[arm].m_LowSpdFlag = 1;
    }
  }

  void check_position_start_and_recovery(EnabledArms arms)
  {
    prepare_position_monitor(arms);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    node->on_enable_control(request, response);
    ASSERT_TRUE(response->success);
    ASSERT_EQ(node->operation_control_mode_, JointControlMode::kPosition);
    const auto pose = node->initial_pose_rad_;
    auto feedback = node->latest_dcss_;
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    EXPECT_EQ(fake_sdk::sends, 1);
    node->transition_started_ -= 300ms;
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kConfiguringPosition);
    EXPECT_EQ(fake_sdk::sends, 2);
    EXPECT_EQ(fake_sdk::stages, 0);
    // Old torque state is not accepted, even though speed ratios match.
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kConfiguringPosition);
    for (int arm = 0; arm < 2; ++arm) {
      feedback.m_State[arm].m_CurState = ARM_STATE_POSITION;
      // Observed on hardware: setting 30% can read back as 29% in position mode.
      feedback.m_In[arm].m_Joint_Vel_Ratio = node->impedance_settings_.velocity_ratio - 1;
      feedback.m_In[arm].m_Joint_Acc_Ratio = node->impedance_settings_.acceleration_ratio - 1;
      feedback.m_In[arm].m_ImpType = 99;
      feedback.m_In[arm].m_Joint_K[0] = std::numeric_limits<float>::quiet_NaN();
    }
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    ASSERT_EQ(node->control_gate_.state(), ControlState::kWaitingForAlignment);
    EXPECT_EQ(fake_sdk::sends, 2);  // A one-point lower echo needs no extra packet.
    EXPECT_EQ(fake_sdk::kd_writes, 0);
    EXPECT_EQ(fake_sdk::imp_type_writes, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
    for (int i = 0; i < 20; ++i) {
      receive(pose);
      node->update_control(feedback, pose, std::chrono::steady_clock::now());
    }
    ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
    auto target = pose;
    const int joint = left_enabled(arms) ? 6 : 13;
    target[joint] += 0.02;
    receive(target);
    node->update_control(feedback, pose, std::chrono::steady_clock::now() + 10ms);
    ASSERT_TRUE(node->last_submitted_target_rad_.has_value());
    EXPECT_DOUBLE_EQ(node->last_sent_target_rad_[joint], target[joint]);
    const auto retained = node->last_submitted_target_rad_;
    node->last_valid_command_time_.reset();
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    const int sends = fake_sdk::sends;
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    EXPECT_EQ(fake_sdk::sends, sends);
    EXPECT_EQ(node->last_submitted_target_rad_, retained);
    for (int i = 0; i < 20; ++i) {
      receive(pose);
      node->update_control(feedback, pose, std::chrono::steady_clock::now());
    }
    EXPECT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::kd_writes, 0);
    if (!left_enabled(arms)) {EXPECT_EQ(fake_sdk::writes[0], 0);}
    if (!right_enabled(arms)) {EXPECT_EQ(fake_sdk::writes[1], 0);}
    // Disable keeps the operation's position identity until the normal IDLE path completes.
    request->data = false;
    node->on_enable_control(request, response);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(node->operation_control_mode_, JointControlMode::kPosition);
    node->update_control(feedback, pose, std::chrono::steady_clock::now() + 1s);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kMonitor);
    EXPECT_EQ(fake_sdk::idles, (left_enabled(arms) ? 1 : 0) + (right_enabled(arms) ? 1 : 0));
  }

  void check_position_rejections()
  {
    prepare_position_monitor();
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    node->latest_dcss_.m_Out[1].m_LowSpdFlag = 0;
    node->on_enable_control(request, response);
    EXPECT_FALSE(response->success);
    node->latest_dcss_.m_Out[1].m_LowSpdFlag = 1;
    node->latest_dcss_.m_Out[0].m_FB_Joint_Vel[0] = 1.0;
    node->on_enable_control(request, response);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(fake_sdk::sends, 0);
    node->latest_dcss_.m_Out[0].m_FB_Joint_Vel[0] = 0;
    node->on_enable_control(request, response);
    ASSERT_TRUE(response->success);
    node->update_control(node->latest_dcss_, node->initial_pose_rad_, std::chrono::steady_clock::now());
    EXPECT_EQ(fake_sdk::sends, 1);
    node->transition_started_ -= 300ms;
    node->latest_dcss_.m_Out[1].m_LowSpdFlag = 0;
    node->update_control(node->latest_dcss_, node->initial_pose_rad_, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kMonitor);
    EXPECT_EQ(fake_sdk::sends, 1);
    EXPECT_EQ(fake_sdk::modes, (std::array<int, 2>{-1, -1}));
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_position_soft_hold_and_busy()
  {
    prepare_position_monitor();
    node->operation_control_mode_ = JointControlMode::kPosition;
    node->control_engaged_ = true;
    node->control_gate_.mark_active();
    const auto pose = node->initial_pose_rad_;
    auto feedback = wait_feedback();
    for (int arm = 0; arm < 2; ++arm) {feedback.m_State[arm].m_CurState = ARM_STATE_POSITION;}
    node->last_sent_target_rad_ = pose;
    node->max_command_step_rad_ = 2.0;
    auto outside_margin = pose;
    outside_margin[1] = lower_limits_rad()[1] + 0.001;
    receive(outside_margin);
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    ASSERT_EQ(node->control_gate_.state(), ControlState::kSoftLimitHold);
    EXPECT_EQ(fake_sdk::idles, 0);
    node->last_control_update_ = std::chrono::steady_clock::now() - 10ms;
    node->update_control(feedback, pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->last_sent_target_rad_, pose);
    for (int i = 0; i < 20; ++i) {
      receive(pose);
      node->update_control(feedback, pose, std::chrono::steady_clock::now());
    }
    ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
    auto target = pose;
    target[6] += 0.01;
    receive(target);
    fake_sdk::busy = true;
    node->update_control(feedback, pose, std::chrono::steady_clock::now() + 10ms);
    EXPECT_EQ(node->last_sent_target_rad_, pose);
    node->target_send_guard_.reset();
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now() - 1s);
    node->update_control(feedback, pose, std::chrono::steady_clock::now() + 20ms);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::kd_writes, 0);
    EXPECT_EQ(fake_sdk::imp_type_writes, 0);
  }

  void check_position_echo_timeout(bool parameters)
  {
    prepare_position_monitor();
    node->operation_control_mode_ = JointControlMode::kPosition;
    node->control_purpose_ = ControlPurpose::kTeleoperation;
    node->impedance_configuration_phase_ = parameters ?
      ImpedanceConfigurationPhase::kWaitingForParameterEcho : ImpedanceConfigurationPhase::kWaitingForModeEcho;
    if (parameters) {node->control_gate_.request_enable();}
    else {node->control_gate_.mark_configuring(true);}
    node->transition_started_ = std::chrono::steady_clock::now() - 3s;
    node->update_control(node->latest_dcss_, node->initial_pose_rad_, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_position_limits_after_mode_switch(bool confirm_limits)
  {
    prepare_position_monitor(EnabledArms::kRight);
    node->operation_control_mode_ = JointControlMode::kPosition;
    node->control_purpose_ = ControlPurpose::kTeleoperation;
    node->control_engaged_ = true;
    node->control_gate_.mark_configuring(true);
    node->impedance_configuration_phase_ = ImpedanceConfigurationPhase::kWaitingForModeEcho;
    node->transition_started_ = std::chrono::steady_clock::now() - 300ms;
    auto feedback = wait_feedback();
    feedback.m_State[0].m_CurState = ARM_STATE_IDLE;
    feedback.m_State[1].m_CurState = ARM_STATE_POSITION;
    feedback.m_In[1].m_Joint_Vel_Ratio = 100;
    feedback.m_In[1].m_Joint_Acc_Ratio = 80;

    node->update_control(feedback, node->initial_pose_rad_, std::chrono::steady_clock::now());
    ASSERT_EQ(node->control_gate_.state(), ControlState::kConfiguringPosition);
    EXPECT_EQ(node->impedance_configuration_phase_,
      ImpedanceConfigurationPhase::kWaitingForPositionLimitEcho);
    EXPECT_EQ(fake_sdk::sends, 1);
    EXPECT_EQ(fake_sdk::speed_writes, (std::array<int, 2>{0, 1}));
    EXPECT_EQ(fake_sdk::modes, (std::array<int, 2>{-1, -1}));
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
    node->update_control(feedback, node->initial_pose_rad_, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kConfiguringPosition);
    EXPECT_EQ(fake_sdk::sends, 1);  // No repeated speed packet or joint target.

    node->transition_started_ = std::chrono::steady_clock::now() -
      (confirm_limits ? 300ms : 3s);
    if (confirm_limits) {
      feedback.m_In[1].m_Joint_Vel_Ratio = node->impedance_settings_.velocity_ratio;
      feedback.m_In[1].m_Joint_Acc_Ratio = node->impedance_settings_.acceleration_ratio;
    }
    node->update_control(feedback, node->initial_pose_rad_, std::chrono::steady_clock::now());
    if (confirm_limits) {
      EXPECT_EQ(node->control_gate_.state(), ControlState::kWaitingForAlignment);
      EXPECT_EQ(fake_sdk::idles, 0);
    } else {
      EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
      EXPECT_NE(node->control_gate_.reason().find("right_state=1"), std::string::npos);
      EXPECT_NE(node->control_gate_.reason().find("right_velocity_ratio=100"), std::string::npos);
      EXPECT_EQ(fake_sdk::idles, 1);
    }
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_position_ratio_bounds()
  {
    prepare_position_monitor(EnabledArms::kRight);
    node->operation_control_mode_ = JointControlMode::kPosition;
    auto feedback = wait_feedback();
    feedback.m_State[0].m_CurState = ARM_STATE_IDLE;
    feedback.m_State[1].m_CurState = ARM_STATE_POSITION;
    const int requested = node->impedance_settings_.velocity_ratio;
    feedback.m_In[1].m_Joint_Vel_Ratio = requested - 1;
    EXPECT_TRUE(node->control_mode_verified(feedback));
    feedback.m_In[1].m_Joint_Vel_Ratio = requested - 2;
    EXPECT_FALSE(node->control_mode_verified(feedback));
    feedback.m_In[1].m_Joint_Vel_Ratio = requested + 1;
    EXPECT_FALSE(node->control_mode_verified(feedback));
    feedback.m_In[1].m_Joint_Vel_Ratio = requested;
    feedback.m_In[1].m_Joint_Acc_Ratio = node->impedance_settings_.acceleration_ratio - 2;
    EXPECT_FALSE(node->control_mode_verified(feedback));
    feedback.m_In[1].m_Joint_Acc_Ratio = node->impedance_settings_.acceleration_ratio + 1;
    EXPECT_FALSE(node->control_mode_verified(feedback));
    feedback.m_In[1].m_Joint_Acc_Ratio = node->impedance_settings_.acceleration_ratio - 1;
    EXPECT_TRUE(node->control_mode_verified(feedback));
    node->impedance_settings_.velocity_ratio = 1;
    feedback.m_In[1].m_Joint_Vel_Ratio = 0;
    EXPECT_FALSE(node->control_mode_verified(feedback));
  }

  void check_position_mode_loss(ControlState state)
  {
    prepare_position_monitor();
    node->operation_control_mode_ = JointControlMode::kPosition;
    node->control_engaged_ = true;
    if (state == ControlState::kActive) {node->control_gate_.mark_active();}
    if (state == ControlState::kWaitingForAlignment) {node->control_gate_.mark_waiting_for_alignment();}
    if (state == ControlState::kSoftLimitHold) {node->control_gate_.enter_soft_limit_hold("test");}
    if (state == ControlState::kCommandWait) {node->control_gate_.enter_command_wait("test");}
    node->update_control(wait_feedback(), node->initial_pose_rad_, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_initial_pose_ratio_parameters()
  {
    EXPECT_EQ(node->initial_pose_velocity_ratio_, 10);
    EXPECT_EQ(node->initial_pose_acceleration_ratio_, 10);
    for (const auto * name : {"initial_pose_velocity_ratio", "initial_pose_acceleration_ratio"}) {
      EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(name, 20)).successful);
      for (const int invalid : {0, 101}) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("connect_on_start", false),
          rclcpp::Parameter(name, invalid)});
        EXPECT_THROW(std::make_shared<MarvinTeleopBridgeNode>(options), std::invalid_argument);
      }
    }
    EXPECT_EQ(fake_sdk::sends, 0);
  }

  void check_initial_pose_ratios(JointControlMode teleop_mode, bool custom)
  {
    node->control_engaged_ = false;
    node.reset();
    fake_sdk::reset();
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("connect_on_start", false),
      rclcpp::Parameter("velocity_ratio", 30),
      rclcpp::Parameter("acceleration_ratio", 40)};
    const int velocity = custom ? 7 : 10;
    const int acceleration = custom ? 9 : 10;
    if (custom) {
      parameters.emplace_back("initial_pose_velocity_ratio", velocity);
      parameters.emplace_back("initial_pose_acceleration_ratio", acceleration);
    }
    options.parameter_overrides(parameters);
    node = std::make_shared<MarvinTeleopBridgeNode>(options);
    node->timer_->cancel();
    ASSERT_TRUE(node->sdk_.connect({127, 0, 0, 1}));
    node->allow_motion_ = true;
    prepare_position_monitor();
    node->teleop_control_mode_ = teleop_mode;
    const auto pose = node->initial_pose_rad_;
    auto feedback = node->latest_dcss_;
    const auto update = [&]() {
        node->update_control(feedback, pose, std::chrono::steady_clock::now());
      };
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->on_move_to_initial_pose(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    ASSERT_TRUE(response->success) << response->message;
    update();
    EXPECT_EQ(fake_sdk::velocity_ratios, (std::array<int, 2>{velocity, velocity}));
    EXPECT_EQ(fake_sdk::acceleration_ratios, (std::array<int, 2>{acceleration, acceleration}));
    EXPECT_EQ(node->impedance_settings_.velocity_ratio, 30);
    EXPECT_EQ(node->impedance_settings_.acceleration_ratio, 40);
    node->transition_started_ -= 300ms;
    update();  // Old teleoperation speed echo must not permit a mode switch.
    EXPECT_EQ(node->control_gate_.state(), ControlState::kPreparingParameters);
    for (auto & arm : feedback.m_In) {arm.m_Joint_Vel_Ratio = velocity;}
    update();  // Acceleration must match independently, too.
    EXPECT_EQ(node->control_gate_.state(), ControlState::kPreparingParameters);
    EXPECT_EQ(fake_sdk::sends, 1);
    for (auto & arm : feedback.m_In) {arm.m_Joint_Acc_Ratio = acceleration;}
    update();
    ASSERT_EQ(node->control_gate_.state(), ControlState::kConfiguringImpedance);
    EXPECT_EQ(fake_sdk::stages, 0);
    for (auto & arm : feedback.m_State) {arm.m_CurState = ARM_STATE_TORQ;}
    update();
    ASSERT_EQ(node->control_gate_.state(), ControlState::kReturningToInitialPose);
    for (int i = 0; i < node->initial_pose_required_frames_; ++i) {update();}
    ASSERT_EQ(node->control_gate_.state(), ControlState::kHold);
    node->update_control(feedback, pose, node->hold_started_ + 1s);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kMonitor);
    EXPECT_FALSE(node->control_engaged_);

    node->latest_dcss_ = feedback;
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    auto enable = std::make_shared<std_srvs::srv::SetBool::Request>();
    enable->data = true;
    auto enabled = std::make_shared<std_srvs::srv::SetBool::Response>();
    node->on_enable_control(enable, enabled);
    ASSERT_TRUE(enabled->success) << enabled->message;
    update();
    EXPECT_EQ(fake_sdk::velocity_ratios, (std::array<int, 2>{30, 30}));
    EXPECT_EQ(fake_sdk::acceleration_ratios, (std::array<int, 2>{40, 40}));
    node->transition_started_ -= 300ms;
    update();  // The previous return-to-pose echo is no longer sufficient.
    EXPECT_EQ(node->control_gate_.state(), ControlState::kPreparingParameters);
    for (auto & arm : feedback.m_In) {
      arm.m_Joint_Vel_Ratio = 30;
      arm.m_Joint_Acc_Ratio = 40;
    }
    update();
    EXPECT_EQ(node->control_gate_.state(), teleop_mode == JointControlMode::kPosition ?
      ControlState::kConfiguringPosition : ControlState::kConfiguringImpedance);
  }

  void check_initial_pose_mode_independent()
  {
    prepare_position_monitor();
    node->operation_control_mode_ = JointControlMode::kPosition; // Previous operation.
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->on_move_to_initial_pose(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(node->operation_control_mode_, JointControlMode::kJointImpedance);
    node->update_control(node->latest_dcss_, node->initial_pose_rad_, std::chrono::steady_clock::now());
    EXPECT_EQ(fake_sdk::kd_writes, 2);
  }

  void check_direct_teleoperation(JointControlMode mode)
  {
    const auto pose = node->initial_pose_rad_;
    node->operation_control_mode_ = mode;
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    node->last_sent_target_rad_ = pose;
    auto feedback = wait_feedback();
    if (mode == JointControlMode::kPosition) {
      for (auto & state : feedback.m_State) {state.m_CurState = ARM_STATE_POSITION;}
    }
    auto t = std::chrono::steady_clock::now();
    JointArray target = pose;
    for (double offset : {0.001, 0.002, 0.003}) {
      target[6] = pose[6] + offset;
      target[13] = pose[13] - offset;
      receive(target);
      t += 10ms;
      node->update_control(feedback, pose, t);
      ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
      EXPECT_EQ(node->last_sent_target_rad_, target);
      ASSERT_TRUE(node->last_submitted_target_rad_);
      EXPECT_EQ(*node->last_submitted_target_rad_, target);
      EXPECT_NEAR(fake_sdk::left[6], radians_to_degrees(target[6]), 1e-12);
      EXPECT_NEAR(fake_sdk::right[6], radians_to_degrees(target[13]), 1e-12);
    }
    const auto submitted = node->last_submitted_target_rad_;
    const auto last_update = node->last_control_update_;
    const auto last_sent = node->last_sent_target_rad_;
    target[6] = pose[6] + 0.004;
    receive(target);
    fake_sdk::busy = true;
    t += 10ms;
    node->update_control(feedback, pose, t);
    EXPECT_EQ(node->last_submitted_target_rad_, submitted);
    EXPECT_EQ(node->last_sent_target_rad_, last_sent);
    EXPECT_EQ(node->last_control_update_, last_update);
    // A reversal received while busy replaces the unsent sample without a catch-up ramp.
    target[6] = pose[6] - 0.001;
    target[13] = pose[13] + 0.001;
    receive(target);
    fake_sdk::busy = false;
    t += 10ms;
    node->update_control(feedback, pose, t);
    EXPECT_EQ(node->last_sent_target_rad_, target);
    EXPECT_EQ(*node->last_submitted_target_rad_, target);
    EXPECT_EQ(node->last_control_update_, t);
    EXPECT_FALSE(node->target_send_guard_.busy());
    EXPECT_NEAR(fake_sdk::left[6], radians_to_degrees(target[6]), 1e-12);
    EXPECT_NEAR(fake_sdk::right[6], radians_to_degrees(target[13]), 1e-12);
  }

  void check_direct_soft_limit_hold()
  {
    const auto pose = node->initial_pose_rad_;
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    node->last_sent_target_rad_ = pose;
    node->soft_limit_allow_other_arm_motion_ = true;
    node->max_command_step_rad_ = 2.0;
    auto target = pose;
    target[1] = lower_limits_rad()[1] + 0.001;
    target[13] += 0.02;
    receive(target);
    auto t = std::chrono::steady_clock::now();
    node->update_control(wait_feedback(), pose, t);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kSoftLimitHold);
    EXPECT_TRUE(node->soft_limit_frozen_arms_[0]);
    EXPECT_FALSE(node->soft_limit_frozen_arms_[1]);
    node->update_control(wait_feedback(), pose, t + 10ms);
    auto expected = target;
    std::copy_n(pose.begin(), kArmDof, expected.begin());
    EXPECT_EQ(node->last_sent_target_rad_, expected);
    EXPECT_EQ(*node->last_submitted_target_rad_, expected);
    EXPECT_NEAR(fake_sdk::left[1], radians_to_degrees(pose[1]), 1e-12);
    EXPECT_NEAR(fake_sdk::right[6], radians_to_degrees(target[13]), 1e-12);
  }

  void check_initial_pose_direct(int scenario)
  {
    const auto pose = node->initial_pose_rad_;
    auto start = pose;
    start[6] += 0.6;
    start[13] -= 0.6;
    ASSERT_GT(node->max_enabled_delta(start, pose), node->max_following_error_rad_);
    node->control_gate_.disarm();
    node->control_engaged_ = false;
    node->sdk_control_compatibility_confirmed_ = node->tool_dynamics_confirmed_ = true;
    node->have_feedback_ = true;
    node->latest_feedback_rad_ = start;
    node->latest_dcss_ = wait_feedback();
    auto t = std::chrono::steady_clock::now();
    node->last_feedback_receive_ = t;
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->on_move_to_initial_pose(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    ASSERT_TRUE(response->success) << response->message;
    node->update_control(node->latest_dcss_, start, t);
    t += 300ms;
    node->update_control(node->latest_dcss_, start, t);
    t += 100ms;
    node->update_control(node->latest_dcss_, start, t);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kReturningToInitialPose);
    ASSERT_FALSE(node->last_submitted_target_rad_);
    const auto previous_update = node->last_control_update_;
    fake_sdk::busy = true;
    t += 10ms;
    node->update_control(node->latest_dcss_, start, t);
    EXPECT_FALSE(node->last_submitted_target_rad_);
    EXPECT_EQ(node->last_sent_target_rad_, start);
    EXPECT_EQ(node->last_control_update_, previous_update);
    EXPECT_EQ(fake_sdk::idles, 0);
    fake_sdk::busy = false;
    t += 10ms;
    node->update_control(node->latest_dcss_, start, t);
    ASSERT_TRUE(node->last_submitted_target_rad_);
    EXPECT_EQ(*node->last_submitted_target_rad_, pose);
    EXPECT_EQ(node->last_sent_target_rad_, pose);
    EXPECT_EQ(node->last_control_update_, t);
    EXPECT_FALSE(node->target_send_guard_.busy());
    EXPECT_NEAR(fake_sdk::left[6], radians_to_degrees(pose[6]), 1e-12);
    EXPECT_NEAR(fake_sdk::right[6], radians_to_degrees(pose[13]), 1e-12);
    const int stages = fake_sdk::stages;
    // A distant endpoint is expected during a point move, and is sent just once.
    t += 10ms;
    node->update_control(node->latest_dcss_, start, t);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kReturningToInitialPose);
    EXPECT_EQ(fake_sdk::stages, stages);
    if (scenario == 1) {
      node->update_control(node->latest_dcss_, start, t + 50s);
      EXPECT_EQ(node->control_gate_.state(), ControlState::kHold);
    } else if (scenario == 2) {
      node->latest_dcss_.m_State[0].m_CurState = ARM_STATE_IDLE;
      node->update_control(node->latest_dcss_, start, t + 10ms);
      EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    } else {
      for (int i = 0; i < node->initial_pose_required_frames_; ++i) {
        t += 10ms;
        node->update_control(node->latest_dcss_, pose, t);
      }
      EXPECT_EQ(node->control_gate_.state(), ControlState::kHold);
      EXPECT_EQ(node->hold_target_rad_, pose);
    }
    EXPECT_EQ(fake_sdk::stages, stages);
  }

  void check_expired(bool feedback_tick)
  {
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now() - 101ms);
    // Even if SDK is free now, no recovery target may be submitted after expiry.
    fake_sdk::busy = false;
    if (feedback_tick) {
      node->control_pacer_.completed(std::chrono::steady_clock::now(), 1.0);
      node->on_timer();
    } else {
      EXPECT_FALSE(node->submit_joint_target(JointArray{}));
    }
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_TRUE(node->control_engaged_);
    EXPECT_FALSE(node->target_send_guard_.busy());
    node->update_control(wait_feedback(), JointArray{}, std::chrono::steady_clock::now());
    EXPECT_EQ(fake_sdk::stages, 0);  // Still requires fresh aligned commands.
  }

  void check_hard_error()
  {
    fake_sdk::send_ok = false;
    EXPECT_FALSE(node->submit_joint_target(JointArray{}));
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_TRUE(node->idle_retry_pending_);
  }

  void check_continuous_busy()
  {
    fake_sdk::busy = true;
    EXPECT_FALSE(node->submit_joint_target(JointArray{}));
    EXPECT_FALSE(node->submit_joint_target(JointArray{}));
    EXPECT_EQ(node->target_send_guard_.attempts(), 2U);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::stages, 0);
    node->target_send_guard_.reset();
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now() - 101ms);
    EXPECT_FALSE(node->submit_joint_target(JointArray{}));
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_FALSE(node->idle_retry_pending_);
    EXPECT_EQ(fake_sdk::stages, 0);
    fake_sdk::busy = false;
    node->process_idle_retry(std::chrono::steady_clock::now() + 1s);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_TRUE(node->control_engaged_);
    EXPECT_FALSE(node->idle_retry_pending_);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
  }

  void check_stale_command()
  {
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now());
    node->last_valid_command_time_.reset();
    node->update_control(wait_feedback(), JointArray{}, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(node->control_gate_.reason(), "valid arm command timed out");
    node->update_control(wait_feedback(), JointArray{}, std::chrono::steady_clock::now() + 2s);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_feedback_failure()
  {
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now());
    fake_sdk::feedback_ok = false;
    node->on_timer();
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_TRUE(node->feedback_recovery_frames_.has_value());
    EXPECT_FALSE(node->target_send_guard_.busy());
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_invalid_timeout()
  {
    EXPECT_NO_THROW(node->validate_parameters());
    for (const double value : {0.0, -0.01, std::nan(""), 0.16}) {
      node->sdk_target_busy_timeout_sec_ = value;
      EXPECT_THROW(node->validate_parameters(), std::invalid_argument);
    }
  }

  void check_68ms_recovery()
  {
    ASSERT_DOUBLE_EQ(node->sdk_target_busy_timeout_sec_, 0.1);
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now() - 68ms);
    ASSERT_TRUE(node->submit_joint_target(JointArray{}));
    EXPECT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_TRUE(node->send_timing_.target_submitted.has_value());
  }

  void check_pacing_after_sdk_delay()
  {
    node->control_gate_.enter_hold("test");
    node->hold_started_ = std::chrono::steady_clock::now();
    fake_sdk::send_delay_ms = 3;
    node->on_timer();
    ASSERT_EQ(fake_sdk::stages, 2);
    EXPECT_FALSE(node->control_pacer_.due(fake_sdk::send_finished + 9ms));
    EXPECT_GE(node->send_timing_.target_sdk_ms, 3.0);
    EXPECT_GE(node->send_timing_.max_target_sdk_ms, 3.0);
    EXPECT_TRUE(node->control_pacer_.due(std::chrono::steady_clock::now() + 11ms));
  }

  void check_command_freshness_after_slow_read()
  {
    fake_sdk::feedback_value = wait_feedback();  // Controller remains in the selected mode.
    fake_sdk::feedback_value.m_Out[0].m_OutFrameSerial = 1;
    fake_sdk::feedback_value.m_Out[1].m_OutFrameSerial = 1;
    node->command_timeout_sec_ = 0.01;
    node->last_valid_command_time_ = std::chrono::steady_clock::now();
    fake_sdk::read_delay_ms = 20;
    node->on_timer();
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(node->control_gate_.reason(), "valid arm command timed out");
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_GE(node->send_timing_.feedback_sdk_ms, 20.0);
  }

  void check_fault_diagnostic_snapshot()
  {
    const auto now = std::chrono::steady_clock::now();
    node->send_timing_.timer_started(now - 68ms);
    node->send_timing_.timer_started(now);
    node->target_send_guard_.observe_busy(now - 101ms);
    node->submit_joint_target(JointArray{});
    const auto & reason = node->control_gate_.reason();
    EXPECT_NE(reason.find("elapsed_since_first_busy_ms="), std::string::npos);
    EXPECT_NE(reason.find("busy_responses=1"), std::string::npos);
    EXPECT_NE(reason.find("timer_gap_ms=68.000"), std::string::npos);
    EXPECT_EQ(reason.find("SDK busy observed for"), std::string::npos);
  }

  DCSS wait_feedback()
  {
    DCSS feedback{};
    for (int arm = 0; arm < 2; ++arm) {
      feedback.m_State[arm].m_CurState = ARM_STATE_TORQ;
      feedback.m_In[arm].m_ImpType = ARM_IMP_JOINT;
      feedback.m_In[arm].m_Joint_Vel_Ratio = node->impedance_settings_.velocity_ratio;
      feedback.m_In[arm].m_Joint_Acc_Ratio = node->impedance_settings_.acceleration_ratio;
      const auto & k = arm == 0 ? node->impedance_settings_.left_stiffness : node->impedance_settings_.right_stiffness;
      const auto & d = arm == 0 ? node->impedance_settings_.left_damping : node->impedance_settings_.right_damping;
      std::copy(k.begin(), k.end(), feedback.m_In[arm].m_Joint_K);
      std::copy(d.begin(), d.end(), feedback.m_In[arm].m_Joint_D);
    }
    return feedback;
  }

  void receive(const JointArray & q)
  {
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    msg->name.assign(joint_names().begin(), joint_names().end());
    msg->position.assign(q.begin(), q.end());
    node->on_joint_command(msg);
  }

  void check_rejected_then_realigned()
  {
    const auto pose = node->initial_pose_rad_;
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    node->last_sent_target_rad_ = pose;
    ASSERT_TRUE(node->submit_joint_target(pose));
    receive(pose);
    auto jumped = pose;
    jumped[6] += 0.06;
    receive(jumped);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    ASSERT_EQ(*node->command_wait_target_rad_, pose);
    // New input bypasses the OLD accepted-target jump reference, but must realign to feedback.
    auto far = pose;
    far[6] += 0.2;
    receive(far);
    node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.aligned_frames(), 0U);
    receive(jumped);  // 0.06 > old jump threshold, but < recovery alignment threshold.
    node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.aligned_frames(), 1U);
    for (int i = 0; i < 30; ++i) {
      node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    }
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(node->control_gate_.aligned_frames(), 1U);  // Cached sample never counts again.
    auto fingers = std::make_shared<sensor_msgs::msg::JointState>();
    fingers->name = {"lh_FFJ1"};
    fingers->position = {0.0};
    node->on_joint_command(fingers);
    EXPECT_EQ(node->control_gate_.aligned_frames(), 1U);
    EXPECT_DOUBLE_EQ(node->displayed_alignment_tolerance(), 0.1);
    EXPECT_DOUBLE_EQ(node->displayed_alignment_margin(), node->soft_limit_release_margin_rad_);
    auto malformed = std::make_shared<sensor_msgs::msg::JointState>();
    malformed->name = {"Joint1_L"};
    node->on_joint_command(malformed);
    EXPECT_EQ(node->control_gate_.aligned_frames(), 0U);
    EXPECT_FALSE(node->last_valid_command_time_);
    for (int i = 0; i < 20; ++i) {
      receive(jumped);
      node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    }
    ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::stages, 2);  // Only the original target, no packets while waiting.
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(node->last_sent_target_rad_, pose);
    const auto t = node->last_control_update_ + 10ms;
    node->update_control(wait_feedback(), pose, t);
    EXPECT_EQ(node->last_sent_target_rad_, jumped);
  }

  void check_wait_holds_submitted_not_unsent()
  {
    const auto pose = node->initial_pose_rad_;
    ASSERT_TRUE(node->submit_joint_target(pose));
    node->last_sent_target_rad_ = pose;
    node->last_sent_target_rad_[6] += 0.03;  // A soft-limit transition may stage an unsent target.
    node->control_gate_.enter_soft_limit_hold("test");
    node->last_valid_command_time_.reset();
    node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(*node->command_wait_target_rad_, pose);
    node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now() + 5s);
    EXPECT_EQ(fake_sdk::stages, 2);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_wait_operator_disable()
  {
    node->begin_command_wait("test");
    node->have_feedback_ = true;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    request->data = false;
    node->on_enable_control(request, response);
    EXPECT_TRUE(response->success);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kHold);
    node->update_control(wait_feedback(), JointArray{}, std::chrono::steady_clock::now() + 1s);
    EXPECT_EQ(fake_sdk::idles, 2);
    EXPECT_FALSE(node->control_engaged_);
  }

  void check_wait_robot_fault()
  {
    node->begin_command_wait("test");
    auto feedback = wait_feedback();
    feedback.m_State[0].m_ERRCode = 1;
    node->update_control(feedback, JointArray{}, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::idles, 2);
  }

  void check_wait_feedback_failure()
  {
    node->begin_command_wait("test");
    fake_sdk::feedback_ok = false;
    node->on_timer();
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_snapshot_expiry()
  {
    // Force expiry between read completion and the actual control check, without sleeping.
    // First nonzero feedback frames initialize freshness independently of this interval.
    node->feedback_timeout_sec_ = 1e-12;
    node->on_timer();
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_NE(node->control_gate_.reason().find("feedback snapshot expired"), std::string::npos);
    EXPECT_TRUE(node->feedback_recovery_frames_);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_TRUE(node->control_engaged_);
  }

  void check_feedback_outage_and_recovery()
  {
    const auto pose = node->initial_pose_rad_;
    ASSERT_TRUE(node->submit_joint_target(pose));
    node->last_sent_target_rad_ = pose;
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    node->last_frames_ = {10, 20};
    node->frames_initialized_ = true;
    node->frame_advanced_once_.fill(true);
    node->last_feedback_advance_.fill(std::chrono::steady_clock::now() - 1s);
    fake_sdk::feedback_value = wait_feedback();
    fake_sdk::feedback_value.m_Out[0].m_OutFrameSerial = 10;
    fake_sdk::feedback_value.m_Out[1].m_OutFrameSerial = 20;
    for (std::size_t j = 0; j < kArmDof; ++j) {
      fake_sdk::feedback_value.m_Out[0].m_FB_Joint_Pos[j] = radians_to_degrees(pose[j]);
      fake_sdk::feedback_value.m_Out[1].m_FB_Joint_Pos[j] = radians_to_degrees(pose[j + kArmDof]);
    }
    for (int i = 0; i < 25; ++i) {
      receive(pose);
      node->on_timer();
    }
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    EXPECT_EQ(*node->command_wait_target_rad_, pose);
    EXPECT_EQ(node->control_gate_.aligned_frames(), 0U);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::stages, 2);  // Only the original submission.
    // Only one arm advancing cannot reopen the gate.
    auto recovered = fake_sdk::feedback_value;
    recovered.m_Out[0].m_OutFrameSerial = 11;
    receive(pose);
    node->update_control(recovered, pose, std::chrono::steady_clock::now());
    EXPECT_TRUE(node->feedback_recovery_frames_);
    EXPECT_EQ(node->control_gate_.aligned_frames(), 0U);
    recovered.m_Out[1].m_OutFrameSerial = 21;
    node->update_control(recovered, pose, std::chrono::steady_clock::now());
    EXPECT_FALSE(node->feedback_recovery_frames_);
    EXPECT_FALSE(node->last_valid_command_time_);  // Discard commands from the outage.
    for (int i = 0; i < 19; ++i) {
      receive(pose);
      node->update_control(recovered, pose, std::chrono::steady_clock::now());
    }
    EXPECT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    receive(pose);
    node->update_control(recovered, pose, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::stages, 2);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_pause_cannot_override_other_states(ControlState state)
  {
    if (state == ControlState::kHold) {
      node->control_gate_.enter_hold("operator disable");
    } else if (state == ControlState::kReturningToInitialPose) {
      node->control_gate_.mark_returning_to_initial_pose();
    } else {
      node->control_gate_.mark_configuring();
    }
    node->pause_teleoperation_or_fault("test unavailable feedback", true);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::idles, 2);
  }

  void check_disable_during_feedback_outage()
  {
    node->have_feedback_ = true;
    fake_sdk::feedback_ok = false;
    node->on_timer();
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    request->data = false;
    node->on_enable_control(request, response);
    ASSERT_TRUE(response->success);
    node->on_timer();
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::idles, 2);
    EXPECT_FALSE(node->control_engaged_);
    EXPECT_EQ(fake_sdk::stages, 0);  // No hold target derived from stale feedback.
  }

  void check_bad_feedback_still_faults(bool nonfinite)
  {
    node->begin_command_wait("test");
    fake_sdk::feedback_value = wait_feedback();
    if (nonfinite) {
      fake_sdk::feedback_value.m_Out[0].m_FB_Joint_Pos[0] = std::nan("");
    } else {
      fake_sdk::feedback_value.m_State[0].m_ERRCode = 1;
    }
    node->on_timer();
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::idles, 2);
  }

  void check_busy_wait_recovery()
  {
    const auto pose = node->initial_pose_rad_;
    ASSERT_TRUE(node->submit_joint_target(pose));
    node->last_sent_target_rad_ = pose;
    node->latest_feedback_rad_ = pose;
    node->have_feedback_ = true;
    fake_sdk::busy = true;
    node->target_send_guard_.observe_busy(std::chrono::steady_clock::now() - 101ms);
    ASSERT_FALSE(node->submit_joint_target(pose));
    ASSERT_EQ(node->control_gate_.state(), ControlState::kCommandWait);
    auto next = pose;
    next[6] += 0.05;
    const int clears_before_wait = fake_sdk::clears;
    for (int i = 0; i < 20; ++i) {
      receive(next);
      node->update_control(wait_feedback(), pose, std::chrono::steady_clock::now());
    }
    EXPECT_EQ(fake_sdk::clears, clears_before_wait);  // No SDK probing in WAIT.
    ASSERT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::stages, 2);
    // If the SDK is still busy, the newly aligned target still cannot be staged.
    node->update_control(wait_feedback(), pose, node->last_control_update_ + 10ms);
    EXPECT_EQ(fake_sdk::stages, 2);
    EXPECT_EQ(node->last_sent_target_rad_, pose);
    fake_sdk::busy = false;
    node->update_control(wait_feedback(), pose, node->last_control_update_ + 20ms);
    EXPECT_EQ(fake_sdk::stages, 4);
    EXPECT_EQ(node->last_sent_target_rad_, next);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_monitor_remains_read_only()
  {
    node->allow_motion_ = false;
    node->control_engaged_ = false;
    node->control_gate_.disarm();
    fake_sdk::feedback_ok = false;
    node->on_timer();
    fake_sdk::feedback_ok = true;
    node->on_timer();
    EXPECT_EQ(fake_sdk::clears, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(node->control_gate_.state(), ControlState::kMonitor);
    EXPECT_TRUE(node->have_feedback_);
  }

  void check_soft_limit_diagnostics_use_release_conditions()
  {
    node->control_gate_.enter_soft_limit_hold("test");
    node->soft_limit_recovery_alignment_rad_ = 0.07;
    node->soft_limit_recovery_frames_ = 12;
    node->control_gate_.observe_soft_limit_recovery(true);
    EXPECT_DOUBLE_EQ(node->displayed_alignment_tolerance(), 0.07);
    EXPECT_DOUBLE_EQ(node->displayed_alignment_margin(), node->soft_limit_release_margin_rad_);
    EXPECT_EQ(node->displayed_stable_frames(), 1U);
    EXPECT_EQ(node->displayed_required_frames(), 12U);
    node->begin_command_wait("test");
    EXPECT_DOUBLE_EQ(node->displayed_alignment_tolerance(), node->command_recovery_alignment_rad_);
    EXPECT_EQ(node->displayed_stable_frames(), 0U);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
  }

  void check_limit_diagnostics_mapping(bool backup = false)
  {
    node->teleop_soft_limits_enabled_ = !backup;
    node->latest_feedback_rad_ = node->initial_pose_rad_;
    node->have_feedback_ = true;
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    receive(node->initial_pose_rad_);
    if (backup) {node->begin_command_wait("test"); receive(node->initial_pose_rad_);}
    else {node->control_gate_.enter_soft_limit_hold("test");}
    std::optional<diagnostic_msgs::msg::DiagnosticArray> captured;
    auto sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      node->alignment_diagnostics_publisher_->get_topic_name(), rclcpp::SensorDataQoS(),
      [&captured](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {captured = *msg;});
    for (int attempt = 0; attempt < 100 && !captured; ++attempt) {
      node->last_alignment_diagnostics_publish_ = {};
      node->publish_alignment_diagnostics_if_due(std::chrono::steady_clock::now());
      rclcpp::spin_some(node);  // Timer cancelled, SDK is stubbed, isolated ROS domain.
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_TRUE(captured);
    ASSERT_EQ(captured->status.size(), 15U);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto & status = captured->status[i + 1];
      auto value = [&status](const std::string & key) {
          for (const auto & item : status.values) {
            if (item.key == key) {return item.value;}
          }
          return std::string{};
        };
      EXPECT_EQ(value("joint_name"), joint_names()[i]);
      EXPECT_NEAR(std::stod(value("hard_lower_rad")), lower_limits_rad()[i], 0.00001);
      EXPECT_NEAR(std::stod(value("hard_upper_rad")), upper_limits_rad()[i], 0.00001);
      EXPECT_NEAR(std::stod(value("soft_lower_rad")),
        lower_limits_rad()[i] + (backup ? -node->teleop_limit_extension() :
        node->joint_limit_safety_margin_rad_), 0.00001);
      EXPECT_NEAR(std::stod(value("release_upper_rad")),
        upper_limits_rad()[i] - (backup ? -node->teleop_limit_extension() :
        node->soft_limit_release_margin_rad_), 0.00001);
      EXPECT_NEAR(std::stod(value("safe_lower_rad")),
        lower_limits_rad()[i] + (backup ? -node->teleop_limit_extension() :
        node->soft_limit_release_margin_rad_), 0.00001);
      EXPECT_EQ(value("joint_limit_policy"), backup ? "backup" : "soft");
      EXPECT_NEAR(std::stod(value("guard_lower_rad")),
        lower_limits_rad()[i] - (backup ? node->teleop_limit_extension() : 0.0), 0.00001);
      EXPECT_NEAR(std::stod(value("feedback_rad")), node->initial_pose_rad_[i], 0.00001);
    }
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_wait_mode_loss()
  {
    node->begin_command_wait("test");
    auto feedback = wait_feedback();
    feedback.m_State[1].m_CurState = ARM_STATE_IDLE;
    node->update_control(feedback, JointArray{}, std::chrono::steady_clock::now());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
    EXPECT_EQ(fake_sdk::idles, 2);
  }
  void check_retreat(int scenario, std::size_t arm = 1)
  {
    node->teleop_control_mode_ = JointControlMode::kPosition;
    node->operation_control_mode_ = JointControlMode::kPosition;
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->on_retreat(arm, response);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(fake_sdk::sends, 0);
    node->control_gate_ = ControlGate(20);
    node->control_engaged_ = false;
    node->sdk_control_compatibility_confirmed_ = true;
    node->tool_dynamics_confirmed_ = true;
    node->enabled_arms_ = EnabledArms::kRight;
    node->subscribe_joint_commands_ = false;
    node->have_feedback_ = true;
    node->latest_dcss_ = wait_feedback();
    const ArmAngles start = arm == 0 ? ArmAngles{90, -60, -90, -90, 0, 0, 0} :
      ArmAngles{-90, -60, 90, -90, 0, 0, 0};
    for (std::size_t j = 0; j < 7; ++j) {
      node->latest_dcss_.m_Out[arm].m_FB_Joint_Pos[j] = start[j];
      node->latest_feedback_rad_[j + 7 * arm] = degrees_to_radians(start[j]);
    }
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    node->last_feedback_advance_.fill(node->last_feedback_receive_);
    node->frame_advanced_once_.fill(true);
    node->allow_motion_ = false;
    node->on_retreat(arm, response);
    EXPECT_FALSE(response->success);
    node->allow_motion_ = true;
    node->enabled_arms_ = arm == 0 ? EnabledArms::kRight : EnabledArms::kLeft;
    node->on_retreat(arm, response);
    EXPECT_FALSE(response->success);
    // Formal dual-arm profile permits subscriptions, but execution is exclusive.
    node->enabled_arms_ = EnabledArms::kBoth;
    node->subscribe_joint_commands_ = true;
    node->last_feedback_receive_ -= 1s;
    node->on_retreat(arm, response);
    EXPECT_FALSE(response->success);
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    EXPECT_EQ(fake_sdk::sends, 0);
    node->on_retreat(arm, response);
    ASSERT_TRUE(response->success) << response->message;
    EXPECT_EQ(node->operation_control_mode_, JointControlMode::kJointImpedance);
    EXPECT_EQ(fake_sdk::sends, 0); // Service only arms; planning waits for mode echo.
    EXPECT_TRUE(node->retreat_plan_.path_deg.empty());
    node->on_retreat(arm, response);
    EXPECT_FALSE(response->success); // No duplicate trajectories.
    node->on_retreat(1 - arm, response);
    EXPECT_FALSE(response->success); // Opposite arm cannot run concurrently.
    auto enable = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto enable_response = std::make_shared<std_srvs::srv::SetBool::Response>();
    enable->data = true;
    node->on_enable_control(enable, enable_response);
    EXPECT_FALSE(enable_response->success);
    auto t = std::chrono::steady_clock::now();
    node->latest_dcss_.m_State[1 - arm].m_CurState = ARM_STATE_IDLE;
    node->latest_dcss_.m_In[1 - arm].m_ImpType = 0;
    node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t);
    t += 300ms;
    node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kConfiguringImpedance);
    ASSERT_TRUE(node->control_engaged_);
    const int prepared_sends = fake_sdk::sends;
    EXPECT_EQ(prepared_sends, 2);
    t += 100ms;
    // Modest mode-switch drift must use the new actual start, not reject or replay
    // a pre-switch path (the former special bound was only 0.005 rad).
    node->latest_feedback_rad_[7 * arm] += 0.01;
    node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t);
    ASSERT_EQ(node->control_gate_.state(), ControlState::kRetreating);
    ASSERT_EQ(node->retreat_plan_.path_deg.size(), 1001U);
    EXPECT_NEAR(node->retreat_plan_.path_deg.front()[0], start[0] + radians_to_degrees(0.01), 1e-9);
    EXPECT_NEAR(node->retreat_plan_.target[0][3] - node->retreat_plan_.start[0][3], -100, 1e-9);
    EXPECT_DOUBLE_EQ(node->retreat_period_sec_, 0.01); // 1000 intervals in the configured 10 s.
    receive(node->latest_feedback_rad_); // Incoming teleop cannot override retreat.
    EXPECT_EQ(node->control_gate_.state(), ControlState::kRetreating);
    EXPECT_EQ(fake_sdk::sends, prepared_sends);
    node->retreat_next_ = t + 100ms;
    node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t + 99ms);
    EXPECT_EQ(fake_sdk::sends, prepared_sends);
    node->retreat_next_ = t;
    if (scenario == 1) {
      fake_sdk::busy = true;
      node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t + 10ms);
      EXPECT_EQ(node->retreat_index_, 1U);
      EXPECT_EQ(fake_sdk::stages, 0);
      node->pause_teleoperation_or_fault("injected feedback loss", true);
      EXPECT_EQ(node->control_gate_.state(), ControlState::kRetreatHold);
      EXPECT_EQ(fake_sdk::sends, prepared_sends);
    } else if (scenario == 5) {
      fake_sdk::send_ok = false;
      node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t + 10ms);
      ASSERT_EQ(node->control_gate_.state(), ControlState::kFault);
      ASSERT_TRUE(node->idle_retry_pending_);
      EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
      EXPECT_EQ(node->control_arms(), arm == 0 ? EnabledArms::kLeft : EnabledArms::kRight);
      fake_sdk::send_ok = true;
      node->process_idle_retry(std::chrono::steady_clock::now() + 2s);
      EXPECT_FALSE(node->control_engaged_);
      EXPECT_EQ(node->control_arms(), EnabledArms::kBoth);
      EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
      return;
    } else if (scenario == 3 || scenario == 4) {
      auto unsafe = node->latest_feedback_rad_;
      if (scenario == 3) {unsafe[7 * arm] += node->max_following_error_rad_ + 0.01;}
      else {unsafe[7 * arm + 1] = 0.0;} // Right joint 2 is outside the planning margin.
      node->update_control(node->latest_dcss_, unsafe, t + 10ms);
      EXPECT_EQ(node->control_gate_.state(), ControlState::kRetreatHold);
      EXPECT_EQ(fake_sdk::sends, prepared_sends);
    } else {
      const auto started = t;
      for (int tick = 1; tick < 1100 && node->control_gate_.state() == ControlState::kRetreating; ++tick) {
        t = std::max(t, node->retreat_next_);
        if (tick > 1) {
          const int previous_sends = fake_sdk::sends;
          node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t - 1ns);
          EXPECT_EQ(fake_sdk::sends, previous_sends); // No early sends/100 ms dwell.
        }
        // Simulated controller tracks the last submitted position, not the desired endpoint.
        node->latest_feedback_rad_ = node->last_sent_target_rad_;
        if (scenario == 2) {
          // A compliant arm can lag >2 mm / 1 degree without violating the
          // bridge's shared joint-following bound. Do not claim actual arrival.
          node->latest_feedback_rad_[7 * arm] -= 0.04;
        }
        node->update_control(node->latest_dcss_, node->latest_feedback_rad_, t);
      }
      ASSERT_EQ(node->control_gate_.state(), ControlState::kHold);
      EXPECT_LT(std::chrono::duration<double>(t - started).count(), 20.0);
      EXPECT_NE(node->control_gate_.reason().find("target sequence sent: -X 100 mm"), std::string::npos);
      EXPECT_NE(node->control_gate_.reason().find("actual arrival not verified"), std::string::npos);
      EXPECT_EQ(arm == 0 ? fake_sdk::right : fake_sdk::left, ArmAngles{});
      EXPECT_GT(fake_sdk::sends, 0);
      PoseMatrix actual{};
      ASSERT_TRUE(retreat_forward(arm, arm == 0 ? fake_sdk::left : fake_sdk::right, actual));
      EXPECT_NEAR(actual[0][3], node->retreat_plan_.target[0][3], 0.1);
      EXPECT_NEAR(actual[1][3], node->retreat_plan_.target[1][3], 0.1);
      EXPECT_NEAR(actual[2][3], node->retreat_plan_.target[2][3], 0.1);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          EXPECT_NEAR(actual[i][j], node->retreat_plan_.start[i][j], 0.001);
        }
      }
      EXPECT_TRUE(node->retreat_completion_pending_);
      EXPECT_EQ(node->hold_target_rad_, node->last_sent_target_rad_);
      const auto hold_started = node->hold_started_;
      node->update_control(wait_feedback(), node->latest_feedback_rad_, hold_started + 100ms);
      EXPECT_EQ(fake_sdk::idles, 0);
      EXPECT_EQ(node->control_gate_.state(), ControlState::kHold);
      EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
      // Exercise immediate IDLE success and temporary busy -> delayed success.
      fake_sdk::busy = scenario == 2;
      node->update_control(wait_feedback(), node->latest_feedback_rad_, hold_started + 600ms);
      if (scenario == 2) {
        EXPECT_EQ(node->control_gate_.state(), ControlState::kFault);
        ASSERT_TRUE(node->idle_retry_pending_);
        EXPECT_TRUE(node->retreat_completion_pending_);
        node->on_retreat(1 - arm, response);
        EXPECT_FALSE(response->success);
        fake_sdk::busy = false;
        node->process_idle_retry(std::chrono::steady_clock::now() + 2s);
      }
      EXPECT_EQ(node->control_gate_.state(), ControlState::kMonitor);
      EXPECT_FALSE(node->control_engaged_);
      EXPECT_FALSE(node->retreat_completion_pending_);
      EXPECT_EQ(node->control_arms(), EnabledArms::kBoth);
      EXPECT_EQ(fake_sdk::idles, 1);
      EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
      // The other arm can now be requested without manually disabling again.
      const ArmAngles next = arm == 1 ? ArmAngles{90,-60,-90,-90,0,0,0} :
        ArmAngles{-90,-60,90,-90,0,0,0};
      for (std::size_t j = 0; j < 7; ++j) {
        node->latest_feedback_rad_[j + (1 - arm) * 7] = degrees_to_radians(next[j]);
        node->latest_dcss_.m_Out[1 - arm].m_FB_Joint_Pos[j] = next[j];
      }
      node->last_feedback_receive_ = std::chrono::steady_clock::now();
      node->last_feedback_advance_.fill(node->last_feedback_receive_);
      node->on_retreat(1 - arm, response);
      EXPECT_TRUE(response->success) << response->message;
      EXPECT_EQ(node->control_arms(), arm == 1 ? EnabledArms::kLeft : EnabledArms::kRight);
      return;
    }
    const int sends = fake_sdk::sends;
    node->update_control(wait_feedback(), node->latest_feedback_rad_, t + 1s);
    EXPECT_EQ(fake_sdk::sends, sends);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_TRUE(node->control_engaged_);
    EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
    EXPECT_GT(fake_sdk::writes[arm], 0);
    // Explicit stop and its delayed retry must also touch ONLY the selected arm.
    fake_sdk::busy = false;
    enable->data = false;
    node->on_enable_control(enable, enable_response);
    ASSERT_TRUE(enable_response->success);
    fake_sdk::busy = true;
    node->update_control(wait_feedback(), node->latest_feedback_rad_, std::chrono::steady_clock::now() + 1s);
    ASSERT_TRUE(node->idle_retry_pending_);
    EXPECT_EQ(node->control_arms(), arm == 0 ? EnabledArms::kLeft : EnabledArms::kRight);
    fake_sdk::busy = false;
    node->process_idle_retry(std::chrono::steady_clock::now() + 2s);
    EXPECT_FALSE(node->control_engaged_);
    EXPECT_EQ(node->control_arms(), EnabledArms::kBoth);
    EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
    EXPECT_EQ(fake_sdk::idles, 1);
  }

  void check_tool_config(std::size_t arm, int scenario)
  {
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->allow_motion_ = false;
    node->on_configure_tool(arm, response);
    ASSERT_TRUE(response->success) << response->message;
    EXPECT_NE(response->message.find("DRY_RUN: no SDK write"), std::string::npos);
    EXPECT_EQ(fake_sdk::clears, 0);
    EXPECT_EQ(fake_sdk::sends, 0);
    EXPECT_FALSE(node->tool_upload_);
    EXPECT_FALSE(node->kinematics_.initialized());
    node->tool_config_dry_run_ = false; // Test-only access, never change a running real node.
    node->control_gate_ = ControlGate(20);
    node->control_engaged_ = false;
    node->sdk_control_compatibility_confirmed_ = true;
    node->tool_dynamics_confirmed_ = false; // No circular requirement, no automatic confirmation.
    node->latest_dcss_ = DCSS{};
    node->have_feedback_ = true;
    node->frame_advanced_once_.fill(true);
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    node->last_feedback_advance_.fill(node->last_feedback_receive_);
    node->on_configure_tool(arm, response);
    EXPECT_FALSE(response->success); // Motion lock is still false.
    node->allow_motion_ = true;
    node->latest_dcss_.m_State[1 - arm].m_CurState = ARM_STATE_TORQ;
    node->on_configure_tool(arm, response);
    EXPECT_FALSE(response->success);
    node->latest_dcss_.m_State[1 - arm].m_CurState = ARM_STATE_IDLE;
    node->latest_dcss_.m_Out[arm].m_FB_Joint_Vel[0] = 1;
    node->on_configure_tool(arm, response);
    EXPECT_FALSE(response->success);
    node->latest_dcss_.m_Out[arm].m_FB_Joint_Vel[0] = 0;
    node->control_gate_.mark_active();
    node->on_configure_tool(arm, response);
    EXPECT_FALSE(response->success);
    node->control_gate_ = ControlGate(20);
    node->last_feedback_receive_ -= 1s;
    node->on_configure_tool(arm, response);
    EXPECT_FALSE(response->success);
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    EXPECT_EQ(fake_sdk::clears, 0);
    fake_sdk::busy = true;
    node->on_configure_tool(arm, response);
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(node->tool_configuration_uncertain());
    EXPECT_EQ(fake_sdk::tool_writes[arm], 0);
    fake_sdk::busy = false;
    if (scenario == 2) {fake_sdk::send_ok = false;}
    if (scenario == 3) {node->tool_configuration_uncertain_arms_[1 - arm] = true;}
    node->on_configure_tool(arm, response);
    EXPECT_EQ(fake_sdk::tool_writes[arm], 1);
    EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
    EXPECT_EQ(fake_sdk::stages, 0); // No joint target, mode or idle request.
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::writes[arm], 1);
    EXPECT_TRUE(node->tool_configuration_uncertain());
    EXPECT_EQ(node->control_gate_.state(), ControlState::kMonitor);
    if (scenario == 2) {
      EXPECT_FALSE(response->success);
      EXPECT_FALSE(node->tool_upload_);
      return;
    }
    ASSERT_TRUE(response->success);
    ASSERT_TRUE(node->tool_upload_);
    const auto expected = load_tool_configuration(
      ament_index_cpp::get_package_share_directory("tianji_io_teleop_bridge") + "/config/tools_cfg.json", arm);
    EXPECT_EQ(fake_sdk::tool_values[arm].kine, expected.kine);
    EXPECT_EQ(fake_sdk::tool_values[arm].dyn, expected.dyn);
    EXPECT_EQ(expected.profile, arm == 0 ? "tool-2" : "tool-1");
    auto enable = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto enabled = std::make_shared<std_srvs::srv::SetBool::Response>();
    enable->data = true;
    node->on_enable_control(enable, enabled);
    EXPECT_FALSE(enabled->success);
    node->on_retreat(arm, response);
    EXPECT_FALSE(response->success);
    node->on_move_to_initial_pose(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    EXPECT_FALSE(response->success);
    node->on_configure_tool(1 - arm, response);
    EXPECT_FALSE(response->success); // No overlapping SDK packet transactions.
    EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
    if (scenario == 1) {
      node->tool_upload_->started -= 3s;
      node->process_tool_upload(nullptr);
      EXPECT_FALSE(node->tool_upload_);
      EXPECT_TRUE(node->tool_configuration_uncertain());
      node->on_retreat(arm, response);
      EXPECT_FALSE(response->success);
      EXPECT_EQ(fake_sdk::sends, 1);
      EXPECT_EQ(fake_sdk::idles, 0);
      return;
    }
    auto echo = node->latest_dcss_;
    std::copy(expected.kine.begin(), expected.kine.end(), echo.m_In[arm].m_ToolKine);
    std::copy(expected.dyn.begin(), expected.dyn.end(), echo.m_In[arm].m_ToolDyn);
    node->tool_upload_->started -= 300ms;
    node->process_tool_upload(&echo);
    EXPECT_TRUE(node->tool_upload_); // Same feedback frame cannot verify upload.
    ++echo.m_Out[arm].m_OutFrameSerial;
    echo.m_In[arm].m_ToolDyn[9] = std::numeric_limits<float>::quiet_NaN();
    node->process_tool_upload(&echo);
    EXPECT_TRUE(node->tool_upload_);
    echo.m_In[arm].m_ToolDyn[9] = static_cast<float>(expected.dyn[9]);
    node->process_tool_upload(&echo);
    EXPECT_FALSE(node->tool_upload_);
    EXPECT_EQ(node->tool_configuration_uncertain(), scenario == 3);
    EXPECT_FALSE(node->tool_configuration_uncertain_arms_[arm]);
    EXPECT_FALSE(node->tool_dynamics_confirmed_);
    EXPECT_FALSE(node->kinematics_.initialized());
    EXPECT_EQ(fake_sdk::sends, 1);
    EXPECT_EQ(fake_sdk::writes[1 - arm], 0);
  }

  void check_joint_zero_pipeline()
  {
    using JS = sensor_msgs::msg::JointState;
    using Wrench = geometry_msgs::msg::WrenchStamped;
    using Report = diagnostic_msgs::msg::DiagnosticArray;
    JS::SharedPtr measured, corrected, raw, bias;
    Wrench::SharedPtr wrench;
    Report::SharedPtr zero_status;
    auto measured_sub = node->create_subscription<JS>(node->state_topic_, 10,
      [&measured](JS::SharedPtr m) {measured = m;});
    auto corrected_sub = node->create_subscription<JS>(node->estimated_joint_disturbance_topic_, 10,
      [&corrected](JS::SharedPtr m) {corrected = m;});
    auto raw_sub = node->create_subscription<JS>(node->raw_state_topic_, 10,
      [&raw](JS::SharedPtr m) {raw = m;});
    auto bias_sub = node->create_subscription<JS>("~/joint_torque_bias", 10,
      [&bias](JS::SharedPtr m) {bias = m;});
    auto status_sub = node->create_subscription<Report>("~/joint_torque_zero_status", 10,
      [&zero_status](Report::SharedPtr m) {zero_status = m;});
    node->initialize_kinematics();
    node->left_cartesian_disturbance_publisher_ = node->create_publisher<Wrench>("~/zero_test_left", 10);
    node->right_cartesian_disturbance_publisher_ = node->create_publisher<Wrench>("~/zero_test_right", 10);
    auto wrench_sub = node->create_subscription<Wrench>("~/zero_test_right", 10,
      [&wrench](Wrench::SharedPtr m) {wrench = m;});
    DCSS feedback{};
    const JointTorqueZero::ArmArray qdeg{30, -55, 25, -80, 35, 30, 15};
    for (std::size_t arm = 0; arm < 2; ++arm) {
      feedback.m_State[arm].m_CurState = ARM_STATE_TORQ;
      for (std::size_t j = 0; j < kArmDof; ++j) {
        feedback.m_Out[arm].m_FB_Joint_Pos[j] = qdeg[j];
        feedback.m_Out[arm].m_FB_Joint_SToq[j] = 20.0 + arm * 7 + j;
        feedback.m_Out[arm].m_EST_Joint_Force[j] = 1.0 + arm * 7 + j;
      }
    }
    node->observe_joint_torque(feedback); // Establish tool/mode context.
    node->clear_torque_zero_windows();
    const double end = node->torque_zero_time();
    for (std::size_t arm = 0; arm < 2; ++arm) {
      for (int i = 0; i <= 100; ++i) {
        JointTorqueZero::ArmArray q{}, tau{};
        for (std::size_t j = 0; j < kArmDof; ++j) {
          q[j] = degrees_to_radians(qdeg[j]); tau[j] = 1.0 + arm * 7 + j;
        }
        node->joint_torque_zero_.observe(arm, end - 1.0 + i * 0.01, i + 1, q, {}, tau);
      }
    }
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    node->have_feedback_ = true;
    node->on_zero_joint_torque({true, true}, false, response);
    ASSERT_TRUE(response->success) << response->message;
    const auto captured = node->joint_torque_zero_.bias();
    node->on_zero_joint_torque({true, true}, false, response);
    ASSERT_TRUE(response->success);
    EXPECT_EQ(node->joint_torque_zero_.bias(), captured);
    // Change pose after capture and introduce external torque on right joint 1.
    feedback.m_Out[1].m_FB_Joint_Pos[0] += 5.0;
    feedback.m_Out[1].m_EST_Joint_Force[0] += 2.0;
    const auto stamp = node->now();
    for (int attempt = 0; attempt < 100 && !(measured && corrected && raw && bias && wrench && zero_status); ++attempt) {
      node->publish_feedback(feedback, stamp);
      node->publish_cartesian_disturbance_estimates(feedback, stamp);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(measured && corrected && raw && bias && wrench && zero_status);
    ASSERT_EQ(zero_status->status.size(), 2U);
    for (const auto & item : zero_status->status) {
      std::unordered_map<std::string, std::string> values;
      for (const auto & entry : item.values) {values[entry.key] = entry.value;}
      EXPECT_EQ(values["measured_torque_topic"], node->state_publisher_->get_topic_name());
      EXPECT_EQ(values["corrected_topic"],
        node->estimated_joint_disturbance_publisher_->get_topic_name());
    }
    EXPECT_EQ(measured->header.stamp, corrected->header.stamp);
    EXPECT_EQ(corrected->header.stamp, raw->header.stamp);
    EXPECT_EQ(corrected->header.stamp, bias->header.stamp);
    EXPECT_EQ(measured->name, corrected->name);
    EXPECT_EQ(measured->position, corrected->position);
    EXPECT_EQ(measured->velocity, corrected->velocity);
    EXPECT_EQ(corrected->position, raw->position);
    EXPECT_EQ(corrected->velocity, raw->velocity);
    ASSERT_EQ(measured->effort.size(), 14U);
    ASSERT_EQ(corrected->effort.size(), 14U);
    for (std::size_t j = 0; j < kJointCount; ++j) {
      EXPECT_DOUBLE_EQ(measured->effort[j], 20.0 + j);
      EXPECT_NEAR(corrected->effort[j], j == 7 ? 2.0 : 0.0, 1e-12);
      EXPECT_NEAR(raw->effort[j] - bias->effort[j], corrected->effort[j], 1e-12);
    }
    auto actual_q = qdeg;
    actual_q[0] += 5.0;
    JointTorqueZero::ArmArray tau{}; tau[0] = 2.0;
    CartesianDisturbance expected{};
    ASSERT_TRUE(node->kinematics_.joint_disturbance_to_wrench(1, actual_q, tau, expected));
    EXPECT_NEAR(wrench->wrench.force.x, expected[0], 1e-10);
    EXPECT_NEAR(wrench->wrench.force.y, expected[1], 1e-10);
    EXPECT_NEAR(wrench->wrench.force.z, expected[2], 1e-10);
    EXPECT_NEAR(wrench->wrench.torque.x, expected[3], 1e-10);
    EXPECT_NEAR(wrench->wrench.torque.y, expected[4], 1e-10);
    EXPECT_NEAR(wrench->wrench.torque.z, expected[5], 1e-10);
    feedback.m_In[1].m_ToolDyn[0] = 2.0;
    node->observe_joint_torque(feedback);
    EXPECT_FALSE(node->joint_torque_zero_.active(1));
    EXPECT_TRUE(node->joint_torque_zero_.active(0));
    feedback.m_State[0].m_CurState = ARM_STATE_IDLE;
    node->observe_joint_torque(feedback);
    EXPECT_FALSE(node->joint_torque_zero_.active(0));
    node->on_zero_joint_torque({true, true}, true, response);
    EXPECT_TRUE(response->success);
    EXPECT_EQ(node->joint_torque_zero_.bias(), JointArray{});
    EXPECT_EQ(fake_sdk::sends, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::writes, (std::array<int, 2>{0, 0}));
  }

  static std::string ft_test_calibration()
  {
    return std::string(FT_TEST_SOURCE) + "/test/fixtures/right_ft_calibration_20261016_03.yaml";
  }

  void check_ft_requires_explicit_calibration()
  {
    for (const std::string & path : {std::string{}, std::string{"/missing/this_run/calibration.yaml"}}) {
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter("connect_on_start", false),
        rclcpp::Parameter("publish_right_ft_sensor", true),
        rclcpp::Parameter("right_ft_calibration_file", path)});
      EXPECT_THROW(std::make_shared<MarvinTeleopBridgeNode>(options), std::exception);
    }
    EXPECT_EQ(fake_sdk::ft_selections, 0);
    EXPECT_EQ(fake_sdk::sends, 0);
  }

  void prime_ft_tare(const std::shared_ptr<MarvinTeleopBridgeNode> & target,
    const FtCompensation::Wrench & residual = {})
  {
    const auto now = target->torque_zero_time();
    for (int i = 0; i <= 510; ++i) {
      target->right_ft_compensation_.observe(now - 5.1 + .01 * i, -1000 + i,
        target->now().nanoseconds(), 1, true, {}, {}, residual);
    }
    target->have_feedback_ = true;
    const auto directory = std::filesystem::temp_directory_path() /
      ("tianji-ft-node-test-" + std::to_string(target->now().nanoseconds()));
    target->right_ft_tare_directory_ = directory.string();
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    target->on_zero_ft_sensor(response);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_TRUE(target->right_ft_compensation_.active());
    ASSERT_TRUE(std::filesystem::is_regular_file(target->right_ft_compensation_.record_file()));
    const auto record = YAML::LoadFile(target->right_ft_compensation_.record_file());
    EXPECT_EQ(record["topic"].as<std::string>(), target->right_ft_sensor_publisher_->get_topic_name());
    EXPECT_FALSE(record["hardware_zero_called"].as<bool>());
    EXPECT_DOUBLE_EQ(record["software_tare_N_Nm"][2].as<double>(), residual[2]);
    std::filesystem::remove_all(directory);
  }

  void check_read_only_300hz_telemetry(bool with_ft = false)
  {
    using JS = sensor_msgs::msg::JointState;
    using Clock = std::chrono::steady_clock;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("allow_motion", false),
      rclcpp::Parameter("connect_on_start", false),
      rclcpp::Parameter("subscribe_joint_commands", false),
      rclcpp::Parameter("publish_right_ft_sensor", with_ft),
      rclcpp::Parameter("right_ft_calibration_file", ft_test_calibration()),
      rclcpp::Parameter("right_ft_sensor_topic", "/telemetry_probe/ft"),
      rclcpp::Parameter("publish_rate_hz", 300.0),
      rclcpp::Parameter("state_topic", "/telemetry_probe/joint_states"),
      rclcpp::Parameter("estimated_joint_disturbance_topic", "/telemetry_probe/disturbance"),
      rclcpp::Parameter("raw_state_topic", "/telemetry_probe/disturbance_raw")});
    auto safe_node = std::make_shared<MarvinTeleopBridgeNode>(options);
    ASSERT_TRUE(safe_node->connect_robot()); // Stub only: no network.
    if (with_ft) {prime_ft_tare(safe_node);}
    fake_sdk::advance_feedback_frames = true;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      for (std::size_t j = 0; j < kArmDof; ++j) {
        fake_sdk::feedback_value.m_Out[arm].m_FB_Joint_SToq[j] = 20.0 + arm * 7 + j;
        fake_sdk::feedback_value.m_Out[arm].m_EST_Joint_Force[j] = 1.0 + arm * 7 + j;
      }
    }
    fake_sdk::feedback_value.m_Out[1].m_EST_Joint_Firc_Dot[0] = 24756.0f;
    std::array<std::vector<Clock::time_point>, 4> arrivals;
    std::array<rclcpp::Subscription<JS>::SharedPtr, 4> subscriptions;
    const std::array<std::string, 4> topics{
      "/telemetry_probe/joint_states", "/telemetry_probe/disturbance",
      "/telemetry_probe/disturbance_raw", "/telemetry_probe/ft"};
    const std::size_t stream_count = with_ft ? 4 : 3;
    for (std::size_t stream = 0; stream < stream_count; ++stream) {
      subscriptions[stream] = safe_node->create_subscription<JS>(topics[stream], 10,
        [&, stream](JS::SharedPtr message) {
          arrivals[stream].push_back(Clock::now());
          if (stream == 3) {
            EXPECT_EQ(message->name.size(), 6U);
            EXPECT_TRUE(message->position.empty());
            EXPECT_TRUE(message->velocity.empty());
            ASSERT_EQ(message->effort.size(), 6U);
            EXPECT_NEAR(message->effort[0], 2.4756 - 0.4226312057938369, 1e-6);
            return;
          }
          EXPECT_EQ(message->name.size(), kJointCount);
          EXPECT_EQ(message->position.size(), kJointCount);
          ASSERT_EQ(message->effort.size(), kJointCount);
          EXPECT_DOUBLE_EQ(message->effort[7], stream == 0 ? 27.0 : 8.0);
        });
    }
    const auto deadline = Clock::now() + 3s;
    while (Clock::now() < deadline) {
      rclcpp::spin_some(safe_node);
      std::this_thread::sleep_for(1ms);
    }
    for (std::size_t stream = 0; stream < stream_count; ++stream) {
      const auto & times = arrivals[stream];
      ASSERT_GT(times.size(), 600U);
      const double elapsed = std::chrono::duration<double>(times.back() - times.front()).count();
      const double hz = (times.size() - 1) / elapsed;
      EXPECT_GT(hz, 260.0);
      EXPECT_LT(hz, 330.0);
    }
    EXPECT_EQ(fake_sdk::sends, 0);
    EXPECT_EQ(fake_sdk::stages, 0);
    EXPECT_EQ(fake_sdk::idles, 0);
    EXPECT_EQ(fake_sdk::ft_selections, with_ft ? 1 : 0);
    safe_node->timer_->cancel();
    fake_sdk::advance_feedback_frames = false;
  }

  void check_ft_sensor_recording()
  {
    using JS = sensor_msgs::msg::JointState;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("connect_on_start", false),
      rclcpp::Parameter("publish_right_ft_sensor", true),
      rclcpp::Parameter("right_ft_calibration_file", ft_test_calibration()),
      rclcpp::Parameter("right_ft_sensor_frame_id", "right_ft_measurement"),
      rclcpp::Parameter("right_ft_sensor_topic", "/ft_test/sensor"),
      rclcpp::Parameter("state_topic", "/ft_test/joints"),
      rclcpp::Parameter("estimated_joint_disturbance_topic", "/ft_test/disturbance")});
    auto safe_node = std::make_shared<MarvinTeleopBridgeNode>(options);
    safe_node->timer_->cancel();
    ASSERT_TRUE(safe_node->connect_robot());
    EXPECT_FALSE(safe_node->allow_motion_);
    std::vector<JS> ft, measured, estimated;
    auto sub = safe_node->create_subscription<JS>("/ft_test/sensor", 10,
      [&](JS::SharedPtr m) {ft.push_back(*m);});
    auto joint_sub = safe_node->create_subscription<JS>("/ft_test/joints", 10,
      [&](JS::SharedPtr m) {measured.push_back(*m);});
    auto est_sub = safe_node->create_subscription<JS>("/ft_test/disturbance", 10,
      [&](JS::SharedPtr m) {estimated.push_back(*m);});
    for (int i = 0; i < 100 && (safe_node->right_ft_sensor_publisher_->get_subscription_count() == 0 ||
      safe_node->state_publisher_->get_subscription_count() == 0 ||
      safe_node->estimated_joint_disturbance_publisher_->get_subscription_count() == 0); ++i)
    {rclcpp::spin_some(safe_node); std::this_thread::sleep_for(10ms);}
    const auto drain = [&]() {
        for (int i = 0; i < 10; ++i) {rclcpp::spin_some(safe_node); std::this_thread::sleep_for(5ms);}
      };
    auto feedback = fake_sdk::feedback_value;
    const std::array<float, 6> counts{24756, 11805, -1190, 1910, -2126, -41};
    std::copy(counts.begin(), counts.end(), feedback.m_Out[1].m_EST_Joint_Firc_Dot);
    feedback.m_Out[1].m_FB_Joint_SToq[0] = 27.0f;
    feedback.m_Out[1].m_EST_Joint_Force[0] = 8.0f;
    const auto stamp = safe_node->now();
    safe_node->publish_feedback(feedback, stamp);
    drain();
    EXPECT_TRUE(ft.empty());  // Never leak raw or untared values on the existing topic.
    EXPECT_EQ(safe_node->right_ft_status_, "WAITING_FOR_SOFTWARE_TARE");
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    safe_node->on_zero_ft_sensor(response);
    EXPECT_FALSE(response->success);
    const FtCompensation::Wrench session_bias{.2, .1, .8, .003, .005, .016};
    prime_ft_tare(safe_node, session_bias);
    ++feedback.m_Out[1].m_OutFrameSerial;
    safe_node->publish_feedback(feedback, stamp);
    drain();
    ASSERT_EQ(ft.size(), 1U);
    ASSERT_FALSE(measured.empty()); ASSERT_FALSE(estimated.empty());
    EXPECT_EQ(ft[0].name, (std::vector<std::string>{"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"}));
    EXPECT_TRUE(ft[0].position.empty()); EXPECT_TRUE(ft[0].velocity.empty());
    ASSERT_EQ(ft[0].effort.size(), 6U);
    const FtCompensation::Wrench zero_pose_prediction{.4226312057938369,
      .11423782402598909 - .30681170389169404 * 9.81, 1.180125582706692,
      -.0005849172175496492 + .04312979085992821 * .30681170389169404 * 9.81,
      .02454622108848717, .024832731016376147 + .0000457926334648345 * .30681170389169404 * 9.81};
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_NEAR(ft[0].effort[i], counts[i] / 10000.0 - zero_pose_prediction[i] - session_bias[i], 1e-6);
    }
    EXPECT_EQ(ft[0].header.stamp, measured[0].header.stamp);
    EXPECT_EQ(ft[0].header.stamp, estimated[0].header.stamp);
    EXPECT_EQ(ft[0].header.frame_id, "right_ft_measurement");
    EXPECT_DOUBLE_EQ(measured[0].effort[7], 27.0);
    EXPECT_DOUBLE_EQ(estimated[0].effort[7], 8.0);
    safe_node->publish_feedback(feedback, safe_node->now()); // Duplicate SDK frame.
    drain();
    EXPECT_EQ(ft.size(), 1U);
    ++feedback.m_Out[1].m_OutFrameSerial;
    feedback.m_Out[1].m_EST_Joint_Firc[0] = 217;
    safe_node->publish_feedback(feedback, safe_node->now());
    drain();
    EXPECT_EQ(ft.size(), 1U);
    EXPECT_EQ(safe_node->right_ft_status_, "CHANNEL_NOT_216");
    ++feedback.m_Out[1].m_OutFrameSerial;
    feedback.m_Out[1].m_EST_Joint_Firc[0] = 216;
    feedback.m_Out[1].m_EST_Joint_Firc_Dot[5] = std::numeric_limits<float>::infinity();
    safe_node->publish_feedback(feedback, safe_node->now());
    drain();
    EXPECT_EQ(ft.size(), 1U);
    EXPECT_EQ(safe_node->right_ft_status_, "NONFINITE_SENSOR_DATA");
    ++feedback.m_Out[1].m_OutFrameSerial;
    feedback.m_Out[1].m_EST_Joint_Firc_Dot[5] = 0; // Zero is valid.
    safe_node->publish_feedback(feedback, safe_node->now());
    drain();
    ASSERT_EQ(ft.size(), 2U);
    EXPECT_NEAR(ft.back().effort[5], -zero_pose_prediction[5] - session_bias[5], 1e-6);
    EXPECT_EQ(safe_node->right_ft_sensor_diagnostic(std::chrono::steady_clock::now()).message, "OK");
    EXPECT_EQ(safe_node->right_ft_sensor_diagnostic(std::chrono::steady_clock::now() + 1s).message,
      "NO_FRESH_SENSOR_FEEDBACK");
    safe_node->right_ft_sensor_frame_id_.clear();
    EXPECT_EQ(safe_node->right_ft_sensor_diagnostic(std::chrono::steady_clock::now()).message,
      "FRAME_ID_UNSET");
    safe_node->disconnect_robot();
    EXPECT_EQ(safe_node->right_ft_sensor_diagnostic(std::chrono::steady_clock::now()).message, "DISCONNECTED");
    ASSERT_TRUE(safe_node->connect_robot());
    EXPECT_FALSE(safe_node->right_ft_compensation_.active());
    EXPECT_FALSE(safe_node->right_ft_last_frame_);
    EXPECT_FALSE(safe_node->right_ft_last_publish_);
    EXPECT_EQ(fake_sdk::ft_selections, 2);
    EXPECT_EQ(fake_sdk::sends, 0); EXPECT_EQ(fake_sdk::stages, 0); EXPECT_EQ(fake_sdk::idles, 0);
  }

  void check_ft_sensor_setup_and_parameters()
  {
    EXPECT_FALSE(node->publish_right_ft_sensor_);
    EXPECT_FALSE(node->right_ft_sensor_publisher_);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("connect_on_start", false),
      rclcpp::Parameter("publish_right_ft_sensor", true),
      rclcpp::Parameter("right_ft_calibration_file", ft_test_calibration())});
    auto safe_node = std::make_shared<MarvinTeleopBridgeNode>(options);
    safe_node->timer_->cancel();
    EXPECT_FALSE(safe_node->set_parameter(rclcpp::Parameter("publish_right_ft_sensor", false)).successful);
    EXPECT_FALSE(safe_node->set_parameter(rclcpp::Parameter("right_ft_sensor_frame_id", "changed")).successful);
    fake_sdk::ft_select_ok = false;
    ASSERT_TRUE(safe_node->connect_robot()); // FT failure does not discard joint feedback.
    EXPECT_EQ(safe_node->right_ft_status_, "CHANNEL_SETUP_FAILED");
    safe_node->disconnect_robot();
    fake_sdk::ft_select_ok = true;
    fake_sdk::ft_channel_echo = false;
    ASSERT_TRUE(safe_node->connect_robot());
    safe_node->publish_feedback(fake_sdk::feedback_value, safe_node->now());
    EXPECT_EQ(safe_node->right_ft_status_, "CHANNEL_NOT_216");
    EXPECT_FALSE(safe_node->right_ft_last_publish_);
    EXPECT_EQ(fake_sdk::ft_selections, 2);
    EXPECT_EQ(fake_sdk::sends, 0); EXPECT_EQ(fake_sdk::idles, 0);
    for (const auto & topic : {"/io_teleop/joint_states", "/io_teleop/joint_cmd",
      "/io_teleop/joint_states_raw", "/tianji_marvin_teleop_bridge/estimated_joint_disturbance"})
    {
      options.parameter_overrides({rclcpp::Parameter("publish_right_ft_sensor", true),
      rclcpp::Parameter("right_ft_calibration_file", ft_test_calibration()),
        rclcpp::Parameter("right_ft_sensor_topic", topic)});
      EXPECT_THROW(std::make_shared<MarvinTeleopBridgeNode>(options), std::invalid_argument);
    }
  }

  void check_nonfinite_efforts_are_independent(bool bad_measured_torque)
  {
    using JS = sensor_msgs::msg::JointState;
    JS::SharedPtr measured, corrected, raw;
    auto measured_sub = node->create_subscription<JS>(node->state_topic_, 10,
      [&measured](JS::SharedPtr m) {measured = m;});
    auto corrected_sub = node->create_subscription<JS>(node->estimated_joint_disturbance_topic_, 10,
      [&corrected](JS::SharedPtr m) {corrected = m;});
    auto raw_sub = node->create_subscription<JS>(node->raw_state_topic_, 10,
      [&raw](JS::SharedPtr m) {raw = m;});
    DCSS feedback{};
    feedback.m_Out[0].m_FB_Joint_SToq[0] = bad_measured_torque ?
      std::numeric_limits<float>::quiet_NaN() : 3.0f;
    feedback.m_Out[0].m_EST_Joint_Force[0] = bad_measured_torque ?
      4.0f : std::numeric_limits<float>::quiet_NaN();
    const auto stamp = node->now();
    for (int attempt = 0; attempt < 100 && !(measured && corrected && raw); ++attempt) {
      node->publish_feedback(feedback, stamp);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(measured && corrected && raw);
    EXPECT_EQ(measured->header.stamp, corrected->header.stamp);
    EXPECT_EQ(measured->header.stamp, raw->header.stamp);
    EXPECT_EQ(measured->position, corrected->position);
    EXPECT_EQ(measured->name, corrected->name);
    if (bad_measured_torque) {
      EXPECT_TRUE(measured->effort.empty());
      ASSERT_EQ(corrected->effort.size(), kJointCount);
      ASSERT_EQ(raw->effort.size(), kJointCount);
      EXPECT_DOUBLE_EQ(corrected->effort[0], 4.0);
    } else {
      ASSERT_EQ(measured->effort.size(), kJointCount);
      EXPECT_DOUBLE_EQ(measured->effort[0], 3.0);
      EXPECT_TRUE(corrected->effort.empty());
      EXPECT_TRUE(raw->effort.empty());
    }
    EXPECT_EQ(fake_sdk::sends, 0);
  }

  std::shared_ptr<MarvinTeleopBridgeNode> node;
};

TEST_F(TargetSendTest, ImpedanceTeleopForwardsPositionsAndBusyRecoveryUsesLatestTarget) {check_direct_teleoperation(JointControlMode::kJointImpedance);}
TEST_F(TargetSendTest, PositionTeleopForwardsPositionsAndBusyRecoveryUsesLatestTarget) {check_direct_teleoperation(JointControlMode::kPosition);}
TEST_F(TargetSendTest, BackupLimitPolicyDefaultsAreValidatedAndStartupOnly) {check_backup_limit_parameters();}
TEST_F(TargetSendTest, ImpedanceBackupForwardsAllUrdfAndOuterGuardBoundaries) {check_backup_forwards_boundaries(JointControlMode::kJointImpedance, 0.02);}
TEST_F(TargetSendTest, PositionBackupForwardsAllUrdfAndOuterGuardBoundaries) {check_backup_forwards_boundaries(JointControlMode::kPosition, 0.02);}
TEST_F(TargetSendTest, ZeroExtensionBackupStillForwardsUrdfEndpoints) {check_backup_forwards_boundaries(JointControlMode::kJointImpedance, 0.0);}
TEST_F(TargetSendTest, BackupRejectsAllOuterGuardViolationsWithoutSendingOrIdle) {check_backup_rejects_outside_guard();}
TEST_F(TargetSendTest, BackupForwardsBothJoint2RightTargetsThatPreviouslyFrozeTheArm) {check_backup_previously_frozen_targets();}
TEST_F(TargetSendTest, BackupStillRejectsJumpsNonfiniteAndMalformedCommands) {check_backup_keeps_input_validation();}
TEST_F(TargetSendTest, BackupAlignsAndRecoversAtUrdfBoundaryButKeepsAutomaticMoveLimits) {check_backup_aligns_and_recovers_at_urdf_boundary();}
TEST_F(TargetSendTest, PublishedBackupLimitsMatchAcceptanceAndRecoveryForAllJoints) {check_limit_diagnostics_mapping(true);}
TEST_F(TargetSendTest, SoftLimitHoldFreezesOneArmAndForwardsTheOtherDirectly) {check_direct_soft_limit_hold();}
TEST_F(TargetSendTest, InitialPoseSubmitsFinalPositionOnceAndWaitsForMeasuredArrival) {check_initial_pose_direct(0);}
TEST_F(TargetSendTest, InitialPosePointMoveStillTimesOutIfArrivalNeverOccurs) {check_initial_pose_direct(1);}
TEST_F(TargetSendTest, InitialPosePointMoveStopsOnModeLoss) {check_initial_pose_direct(2);}
TEST_F(TargetSendTest, TeleopModeIsValidatedReadOnlyAndDefaultsToImpedance) {check_mode_parameter();}
TEST_F(TargetSendTest, PositionBothArmsPrepareAlignFollowWaitRecoverDisable) {check_position_start_and_recovery(EnabledArms::kBoth);}
TEST_F(TargetSendTest, PositionLeftOnlyNeverWritesRight) {check_position_start_and_recovery(EnabledArms::kLeft);}
TEST_F(TargetSendTest, PositionRightOnlyNeverWritesLeft) {check_position_start_and_recovery(EnabledArms::kRight);}
TEST_F(TargetSendTest, PositionRequiresStationaryBeforeAndAfterPreparation) {check_position_rejections();}
TEST_F(TargetSendTest, PositionSoftLimitRecoversAndBusyRetainsTargetWithoutIdle) {check_position_soft_hold_and_busy();}
TEST_F(TargetSendTest, PositionParameterTimeoutNeverSendsTarget) {check_position_echo_timeout(true);}
TEST_F(TargetSendTest, PositionModeTimeoutNeverSendsTarget) {check_position_echo_timeout(false);}
TEST_F(TargetSendTest, PositionModeReappliesResetSpeedLimitsBeforeAlignment) {check_position_limits_after_mode_switch(true);}
TEST_F(TargetSendTest, PositionLimitEchoTimeoutReportsActualModeAndRatios) {check_position_limits_after_mode_switch(false);}
TEST_F(TargetSendTest, PositionRatioReadbackAcceptsOnlyAtMostOnePointLower) {check_position_ratio_bounds();}
TEST_F(TargetSendTest, PositionModeLossWhileAligningStopsTargets) {check_position_mode_loss(ControlState::kWaitingForAlignment);}
TEST_F(TargetSendTest, PositionModeLossWhileActiveStopsTargets) {check_position_mode_loss(ControlState::kActive);}
TEST_F(TargetSendTest, PositionModeLossWhileSoftHoldStopsTargets) {check_position_mode_loss(ControlState::kSoftLimitHold);}
TEST_F(TargetSendTest, PositionModeLossWhileCommandWaitStopsTargets) {check_position_mode_loss(ControlState::kCommandWait);}
TEST_F(TargetSendTest, InitialPoseStillUsesImpedanceWithPositionTeleopSelected) {check_initial_pose_mode_independent();}
TEST_F(TargetSendTest, InitialPoseRatiosAreValidatedReadOnlyAndDefaultToTen) {check_initial_pose_ratio_parameters();}
TEST_F(TargetSendTest, InitialPoseTenPercentThenRestoresImpedanceTeleopRatios) {check_initial_pose_ratios(JointControlMode::kJointImpedance, false);}
TEST_F(TargetSendTest, InitialPoseCustomRatiosThenRestoresPositionTeleopRatios) {check_initial_pose_ratios(JointControlMode::kPosition, true);}
TEST_F(TargetSendTest, JointZeroCorrectsPublishedRecordingStreamAndWrench) {check_joint_zero_pipeline();}
TEST_F(TargetSendTest, ReadOnlyThreeJointStreamsRunAt300HzWithoutSdkWrites) {check_read_only_300hz_telemetry();}
TEST_F(TargetSendTest, RightFtAndJointStreamsRunAt300HzWithOnlyOneChannelSetup) {check_read_only_300hz_telemetry(true);}
TEST_F(TargetSendTest, RightFtRecordsCompensatedTaredEffortWithSameStampAndRejectsInvalidOrDuplicateFrames) {check_ft_sensor_recording();}
TEST_F(TargetSendTest, RightFtRequiresExplicitCalibrationWithoutHistoricalFallback) {check_ft_requires_explicit_calibration();}
TEST_F(TargetSendTest, RightFtSetupChecksEchoAndIsIndependentOfMotionPermission) {check_ft_sensor_setup_and_parameters();}
TEST_F(TargetSendTest, BadMeasuredTorqueDoesNotEraseDisturbance) {check_nonfinite_efforts_are_independent(true);}
TEST_F(TargetSendTest, BadDisturbanceDoesNotEraseMeasuredTorque) {check_nonfinite_efforts_are_independent(false);}
TEST_F(TargetSendTest, DeadlinePreventsLateRecovery) {check_expired(false);}
TEST_F(TargetSendTest, FeedbackTickEnforcesDeadlineWithoutControlTick) {check_expired(true);}
TEST_F(TargetSendTest, OtherSendErrorsStillFaultImmediately) {check_hard_error();}
TEST_F(TargetSendTest, ContinuousBusyWaitsWithoutIdleRetries) {check_continuous_busy();}
TEST_F(TargetSendTest, StaleCommandWaitsWithoutIdle) {check_stale_command();}
TEST_F(TargetSendTest, FeedbackFailurePausesTeleoperation) {check_feedback_failure();}
TEST_F(TargetSendTest, InvalidTimeoutRejected) {check_invalid_timeout();}
TEST_F(TargetSendTest, PauseOf68msRecoversWithin100msDeadline) {check_68ms_recovery();}
TEST_F(TargetSendTest, SchedulingStartsAfterActualSdkCompletion) {check_pacing_after_sdk_delay();}
TEST_F(TargetSendTest, SlowReadCannotMakeAnOldCommandLookFresh) {check_command_freshness_after_slow_read();}
TEST_F(TargetSendTest, TimeoutPreservesTimingSnapshotWithoutClaimingContinuousBusy) {check_fault_diagnostic_snapshot();}
TEST_F(TargetSendTest, RejectedCommandsRequireFreshRealignmentBeforeDirectRecovery) {check_rejected_then_realigned();}
TEST_F(TargetSendTest, WaitRetainsSubmittedTargetNotSoftLimitStagedTarget) {check_wait_holds_submitted_not_unsent();}
TEST_F(TargetSendTest, OperatorDisableStillExitsFromWait) {check_wait_operator_disable();}
TEST_F(TargetSendTest, RobotFaultStillExitsFromWait) {check_wait_robot_fault();}
TEST_F(TargetSendTest, FeedbackFailureDoesNotIdleInWait) {check_wait_feedback_failure();}
TEST_F(TargetSendTest, LossOfImpedanceModeStillFaultsInWait) {check_wait_mode_loss();}
TEST_F(TargetSendTest, SnapshotExpiryWaitsWithoutSendingOrIdle) {check_snapshot_expiry();}
TEST_F(TargetSendTest, FeedbackOutageRequiresBothFramesThenFreshAlignedCommands) {check_feedback_outage_and_recovery();}
TEST_F(TargetSendTest, PauseCannotCancelManualStop) {check_pause_cannot_override_other_states(ControlState::kHold);}
TEST_F(TargetSendTest, PauseCannotConvertInitialPoseToTeleoperation) {check_pause_cannot_override_other_states(ControlState::kReturningToInitialPose);}
TEST_F(TargetSendTest, ModeConfigurationFailureStillFaults) {check_pause_cannot_override_other_states(ControlState::kConfiguringImpedance);}
TEST_F(TargetSendTest, DisableStillRequestsIdleDuringFeedbackOutage) {check_disable_during_feedback_outage();}
TEST_F(TargetSendTest, NonfiniteFeedbackStillFaults) {check_bad_feedback_still_faults(true);}
TEST_F(TargetSendTest, ControllerErrorStillFaultsOnFeedbackTick) {check_bad_feedback_still_faults(false);}
TEST_F(TargetSendTest, BusyWaitRealignsBeforeSubmittingLatestTargetDirectly) {check_busy_wait_recovery();}
TEST_F(TargetSendTest, MonitorWithMotionLockedRemainsReadOnly) {check_monitor_remains_read_only();}
TEST_F(TargetSendTest, SoftLimitDisplayUsesActualReleaseConditions) {check_soft_limit_diagnostics_use_release_conditions();}
TEST_F(TargetSendTest, PublishedLimitDiagnosticsMatchAll14CanonicalJoints) {check_limit_diagnostics_mapping();}

TEST_F(TargetSendTest, RightRetreatHundredMillimetersSendsAndHolds) {check_retreat(false);}
TEST_F(TargetSendTest, RightRetreatBusyAndFeedbackLossDoNotAdvanceOrIdle) {check_retreat(true);}
TEST_F(TargetSendTest, RightRetreatAllowsCompliantLagWithoutClaimingArrival) {check_retreat(2);}
TEST_F(TargetSendTest, RightRetreatRetainsSharedJointFollowingGuard) {check_retreat(3);}
TEST_F(TargetSendTest, RightRetreatRetainsMeasuredJointLimits) {check_retreat(4);}
TEST_F(TargetSendTest, LeftRetreatInDualProfileControlsOnlyLeftIncludingIdleRetry) {check_retreat(0, 0);}
TEST_F(TargetSendTest, LeftRetreatFeedbackLossHoldsOnlyLeft) {check_retreat(1, 0);}
TEST_F(TargetSendTest, LeftRetreatAllowsCompliantLag) {check_retreat(2, 0);}
TEST_F(TargetSendTest, LeftRetreatRetainsFollowingGuard) {check_retreat(3, 0);}
TEST_F(TargetSendTest, LeftRetreatRetainsMeasuredJointLimits) {check_retreat(4, 0);}
TEST_F(TargetSendTest, LeftRetreatHardFaultIdleRetryRemainsLeftOnly) {check_retreat(5, 0);}
TEST_F(TargetSendTest, RightRetreatHardFaultIdleRetryRemainsRightOnly) {check_retreat(5, 1);}
TEST_F(TargetSendTest, LeftToolPreviewAndUploadVerifyWithoutMotion) {check_tool_config(0, 0);}
TEST_F(TargetSendTest, RightToolPreviewAndUploadVerifyWithoutMotion) {check_tool_config(1, 0);}
TEST_F(TargetSendTest, ToolEchoTimeoutBlocksMotionWithoutIdle) {check_tool_config(0, 1);}
TEST_F(TargetSendTest, ToolSendFailureDoesNotClaimSuccess) {check_tool_config(1, 2);}
TEST_F(TargetSendTest, VerifiedToolDoesNotClearOtherArmsUnconfirmedUpload) {check_tool_config(1, 3);}

}  // namespace tianji_io_teleop_bridge
