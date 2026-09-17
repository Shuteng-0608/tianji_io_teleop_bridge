#include <cstdlib>
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
int clears = 0;
int stages = 0;
int sends = 0;
int idles = 0;
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
  clears = stages = sends = idles = 0;
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
long OnGetSDKVersion() {return 100343013;}
bool OnGetBuf(DCSS *feedback)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(fake_sdk::read_delay_ms));
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
bool OnSetTargetState_A(int state) {++fake_sdk::writes[0]; if (state == ARM_STATE_IDLE) {++fake_sdk::idles;} return true;}
bool OnSetTargetState_B(int state) {++fake_sdk::writes[1]; if (state == ARM_STATE_IDLE) {++fake_sdk::idles;} return true;}
bool OnSetImpType_A(int) {++fake_sdk::writes[0]; return true;}
bool OnSetImpType_B(int) {++fake_sdk::writes[1]; return true;}
bool OnSetJointLmt_A(int, int) {++fake_sdk::writes[0]; return true;}
bool OnSetJointLmt_B(int, int) {++fake_sdk::writes[1]; return true;}
bool OnSetJointKD_A(double *, double *) {++fake_sdk::writes[0]; return true;}
bool OnSetJointKD_B(double *, double *) {++fake_sdk::writes[1]; return true;}
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
    node = std::make_shared<MarvinTeleopBridgeNode>();
    node->timer_->cancel();
    ASSERT_TRUE(node->sdk_.connect({127, 0, 0, 1}));
    node->enabled_arms_ = EnabledArms::kBoth;
    node->allow_motion_ = true;
    node->control_engaged_ = true;
    node->control_gate_.mark_active();
    node->slew_limiter_.reset(JointArray{});
    node->max_velocity_rad_s_.fill(1.0);
    node->max_acceleration_rad_s2_.fill(1.0);
  }
  void TearDown() override {node.reset();}

  void check_recovery()
  {
    const auto t = std::chrono::steady_clock::now();
    node->last_control_update_ = t - 10ms;
    JointArray desired{};
    desired.fill(0.5);
    auto expected_limiter = node->slew_limiter_;
    fake_sdk::busy = true;
    EXPECT_FALSE(node->send_slew_limited_target(desired, t));
    EXPECT_EQ(node->last_control_update_, t - 10ms);
    EXPECT_EQ(node->last_sent_target_rad_, JointArray{});
    EXPECT_EQ(node->control_gate_.state(), ControlState::kActive);
    EXPECT_EQ(fake_sdk::idles, 0);

    // The recovery uses a new opposite target, not the unsent positive target.
    desired.fill(-0.5);
    const auto expected = expected_limiter.update(
      desired, node->max_velocity_rad_s_, node->max_acceleration_rad_s2_, 0.02);
    fake_sdk::busy = false;
    ASSERT_TRUE(node->send_slew_limited_target(desired, t + 10ms));
    EXPECT_EQ(node->last_sent_target_rad_, expected);
    EXPECT_EQ(node->last_control_update_, t + 10ms);
    EXPECT_FALSE(node->target_send_guard_.busy());
    EXPECT_NEAR(fake_sdk::left[0], radians_to_degrees(expected[0]), 1e-12);
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
    node->update_control(DCSS{}, JointArray{}, std::chrono::steady_clock::now());
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
    EXPECT_GT(node->last_sent_target_rad_[6], pose[6]);
    EXPECT_LE(node->last_sent_target_rad_[6] - pose[6], 0.000101);
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
    EXPECT_GT(node->last_sent_target_rad_[6], pose[6]);
    EXPECT_LE(node->last_sent_target_rad_[6] - pose[6], 0.000401);
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

  void check_limit_diagnostics_mapping()
  {
    node->latest_feedback_rad_ = node->initial_pose_rad_;
    node->have_feedback_ = true;
    node->last_feedback_receive_ = std::chrono::steady_clock::now();
    receive(node->initial_pose_rad_);
    node->control_gate_.enter_soft_limit_hold("test");
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
        lower_limits_rad()[i] + node->joint_limit_safety_margin_rad_, 0.00001);
      EXPECT_NEAR(std::stod(value("release_upper_rad")),
        upper_limits_rad()[i] - node->soft_limit_release_margin_rad_, 0.00001);
      EXPECT_NEAR(std::stod(value("safe_lower_rad")),
        lower_limits_rad()[i] + node->soft_limit_release_margin_rad_, 0.00001);
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
    EXPECT_GE(node->retreat_period_sec_, 0.01);
    EXPECT_LT(node->retreat_period_sec_, 0.02); // This commissioning pose fits ~10 s.
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

  std::shared_ptr<MarvinTeleopBridgeNode> node;
};

TEST_F(TargetSendTest, BusyRollsBackLimiterAndRecoveryUsesLatestTarget) {check_recovery();}
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
TEST_F(TargetSendTest, RejectedCommandsRequireFreshRealignmentBeforeSlewLimitedRecovery) {check_rejected_then_realigned();}
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
TEST_F(TargetSendTest, BusyWaitRealignsAndRecoveryIsSlewLimited) {check_busy_wait_recovery();}
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
