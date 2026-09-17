#include "tianji_io_teleop_bridge/marvin_sdk_adapter.hpp"

#include <algorithm>

namespace tianji_io_teleop_bridge
{

bool MarvinSdkAdapter::connect(const std::array<unsigned char, 4> & ip)
{
  last_error_.clear();
  if (!OnLinkTo(ip[0], ip[1], ip[2], ip[3])) {
    return fail("OnLinkTo failed (port busy or robot unreachable)");
  }
  connected_ = true;
  return true;
}

bool MarvinSdkAdapter::disconnect()
{
  if (!connected_) {
    return true;
  }
  if (!OnRelease()) {
    return fail("OnRelease failed");
  }
  connected_ = false;
  return true;
}

bool MarvinSdkAdapter::read_feedback(DCSS & feedback)
{
  if (!connected_) {
    return fail("feedback requested while SDK is disconnected");
  }
  if (!OnGetBuf(&feedback)) {
    return fail("OnGetBuf failed");
  }
  return true;
}

bool MarvinSdkAdapter::configure_joint_impedance_parameters(
  EnabledArms arms,
  const JointImpedanceSettings & settings)
{
  if (!connected_) {
    return fail("impedance configuration requested while SDK is disconnected");
  }
  if (arms == EnabledArms::kNone) {
    return fail("impedance configuration requested with no enabled arms");
  }

  // This packet must settle before enter_joint_impedance_mode() starts another packet.
  if (!OnClearSet()) {
    return fail("OnClearSet failed before impedance parameter packet");
  }
  if (left_enabled(arms)) {
    auto stiffness = settings.left_stiffness;
    auto damping = settings.left_damping;
    if (!OnSetJointLmt_A(settings.velocity_ratio, settings.acceleration_ratio) ||
      !OnSetJointKD_A(stiffness.data(), damping.data()))
    {
      discard_pending_packet();
      return fail("failed to stage left-arm impedance parameters");
    }
  }
  if (right_enabled(arms)) {
    auto stiffness = settings.right_stiffness;
    auto damping = settings.right_damping;
    if (!OnSetJointLmt_B(settings.velocity_ratio, settings.acceleration_ratio) ||
      !OnSetJointKD_B(stiffness.data(), damping.data()))
    {
      discard_pending_packet();
      return fail("failed to stage right-arm impedance parameters");
    }
  }
  if (!OnSetSend()) {
    discard_pending_packet();
    return fail("OnSetSend failed for impedance parameter packet");
  }
  return true;
}

bool MarvinSdkAdapter::enter_joint_impedance_mode(EnabledArms arms)
{
  if (!connected_) {
    return fail("impedance mode requested while SDK is disconnected");
  }
  if (arms == EnabledArms::kNone) {
    return fail("impedance mode requested with no enabled arms");
  }

  // Match Marvin's official impedance examples: this packet changes only the
  // controller state and impedance type. Joint targets are sent separately.
  if (!OnClearSet()) {
    return fail("OnClearSet failed before impedance mode packet");
  }
  if (left_enabled(arms)) {
    if (!OnSetTargetState_A(ARM_STATE_TORQ) || !OnSetImpType_A(ARM_IMP_JOINT))
    {
      discard_pending_packet();
      return fail("failed to stage left-arm joint-impedance mode");
    }
  }
  if (right_enabled(arms)) {
    if (!OnSetTargetState_B(ARM_STATE_TORQ) || !OnSetImpType_B(ARM_IMP_JOINT))
    {
      discard_pending_packet();
      return fail("failed to stage right-arm joint-impedance mode");
    }
  }
  if (!OnSetSend()) {
    discard_pending_packet();
    return fail("OnSetSend failed for impedance mode packet");
  }
  return true;
}

TargetSendResult MarvinSdkAdapter::send_joint_targets(
  EnabledArms arms, const JointArray & positions_deg)
{
  if (!connected_) {
    fail("joint target requested while SDK is disconnected");
    return TargetSendResult::kError;
  }
  if (arms == EnabledArms::kNone) {
    fail("joint target requested with no enabled arms");
    return TargetSendResult::kError;
  }
  if (!OnClearSet()) {
    // For the checksum-pinned SDK this means the previous packet is still busy.
    // No target has been staged: do not discard or overwrite the pending packet.
    fail("OnClearSet busy before joint target packet");
    return TargetSendResult::kBusy;
  }

  if (left_enabled(arms)) {
    std::array<double, kArmDof> left{};
    std::copy_n(positions_deg.begin(), kArmDof, left.begin());
    if (!OnSetJointCmdPos_A(left.data())) {
      discard_pending_packet();
      fail("OnSetJointCmdPos_A failed");
      return TargetSendResult::kError;
    }
  }
  if (right_enabled(arms)) {
    std::array<double, kArmDof> right{};
    std::copy_n(positions_deg.begin() + kArmDof, kArmDof, right.begin());
    if (!OnSetJointCmdPos_B(right.data())) {
      discard_pending_packet();
      fail("OnSetJointCmdPos_B failed");
      return TargetSendResult::kError;
    }
  }
  if (!OnSetSend()) {
    discard_pending_packet();
    fail("OnSetSend failed for joint target packet");
    return TargetSendResult::kError;
  }
  last_error_.clear();
  return TargetSendResult::kSubmitted;
}

bool MarvinSdkAdapter::request_idle(EnabledArms arms)
{
  if (!connected_ || arms == EnabledArms::kNone) {
    return true;
  }
  if (!OnClearSet()) {
    return fail("OnClearSet failed before idle request");
  }
  if (left_enabled(arms) && !OnSetTargetState_A(ARM_STATE_IDLE)) {
    discard_pending_packet();
    return fail("OnSetTargetState_A(IDLE) failed");
  }
  if (right_enabled(arms) && !OnSetTargetState_B(ARM_STATE_IDLE)) {
    discard_pending_packet();
    return fail("OnSetTargetState_B(IDLE) failed");
  }
  if (!OnSetSend()) {
    discard_pending_packet();
    return fail("OnSetSend failed for idle request");
  }
  return true;
}

TargetSendResult MarvinSdkAdapter::configure_tool(std::size_t arm, const ToolConfiguration & config)
{
  if (!connected_ || arm > 1 || !valid_tool_configuration(config)) {
    fail("tool configuration requires connected SDK, valid arm and parameters");
    return TargetSendResult::kError;
  }
  if (!OnClearSet()) {
    fail("OnClearSet busy before tool packet; nothing staged");
    return TargetSendResult::kBusy;
  }
  auto kine = config.kine;
  auto dyn = config.dyn;
  if (!(arm == 0 ? OnSetTool_A(kine.data(), dyn.data()) : OnSetTool_B(kine.data(), dyn.data()))) {
    discard_pending_packet();
    fail("failed to stage tool parameters; controller state not confirmed");
    return TargetSendResult::kError;
  }
  if (!OnSetSend()) {
    discard_pending_packet();
    fail("tool send failed; controller state not confirmed");
    return TargetSendResult::kError;
  }
  last_error_.clear();
  return TargetSendResult::kSubmitted;
}

bool MarvinSdkAdapter::fail(const std::string & message)
{
  last_error_ = message;
  return false;
}

void MarvinSdkAdapter::discard_pending_packet()
{
  (void)OnClearSet();
}

}  // namespace tianji_io_teleop_bridge
