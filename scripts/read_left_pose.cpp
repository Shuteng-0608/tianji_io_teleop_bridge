// One-shot feedback + local FK example. No control/mode/tool-upload API calls.
// Build: see docs/READ_LEFT_POSE.md. SDK q: degrees; FK translation: mm.
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>
#include <Eigen/Geometry>
#include "MarvinSDK.h"
#include "FxRobot.h"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: read_left_pose /absolute/path/ccs_m6_40.MvKDCfg\n"
              << "Reads feedback once from 192.168.1.190; no control commands.\n";
    return 2;
  }
  FX_INT32L types[2]{};
  FX_DOUBLE gravity[2][3]{}, dh[2][8][4]{}, limits[2][7][4]{};
  FX_DOUBLE interference[2][4][3]{}, mass[2][7]{}, com[2][7][3]{}, inertia[2][7][6]{};
  if (!LOADMvCfg(argv[1], types, gravity, dh, limits, interference, mass, com, inertia)) {
    std::cerr << "Model load failed\n";
    return 1;
  }
  FX_LOG_SWITCH(0);
  // These functions configure only the local libKine mathematical model.
  // A fresh process uses the model's default arm-base user frame.
  if (!FX_Robot_Init_Type(0, types[0]) || !FX_Robot_Init_Kine(0, dh[0]) ||
    !FX_Robot_Tool_Rmv(0))
  {
    std::cerr << "Local kinematics initialization failed\n";
    return 1;
  }
  int byte_order = 0;
  if (CheckSDKTypeCompat(&byte_order) < 0) {
    std::cerr << "SDK ABI compatibility check failed\n";
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  if (!OnLinkTo(192, 168, 1, 190)) {
    std::cerr << "Feedback connection failed\n";
    return 1;
  }
  struct ReleaseConnection {
    ~ReleaseConnection() {OnRelease();}
  } connection;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  std::optional<FX_INT32> last_frame;
  int changes = 0;
  DCSS feedback{};
  bool fresh = false;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!OnGetBuf(&feedback)) {continue;}
    const auto frame = feedback.m_Out[0].m_OutFrameSerial;
    if (last_frame && frame != *last_frame) {++changes;}
    last_frame = frame;
    // Require two observed frame updates: never report the initial zero buffer.
    if (changes >= 2) {fresh = true; break;}
  }
  if (!fresh) {
    std::cerr << "No live left-arm feedback: frame counter did not update twice\n";
    return 1;
  }
  const auto captured = std::chrono::system_clock::now();
  Vect7 q{};
  for (int j = 0; j < 7; ++j) {
    q[j] = feedback.m_Out[0].m_FB_Joint_Pos[j];
    if (!std::isfinite(q[j])) {std::cerr << "Invalid joint feedback\n"; return 1;}
  }
  Matrix4 pose{};
  if (!FX_Robot_Kine_FK(0, q, pose)) {std::cerr << "FK failed\n"; return 1;}
  Eigen::Matrix3d rotation;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (!std::isfinite(pose[i][j])) {std::cerr << "Invalid FK output\n"; return 1;}
      if (i < 3 && j < 3) {rotation(i, j) = pose[i][j];}
    }
  }
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
    std::abs(rotation.determinant() - 1.0) > 1e-6)
  {std::cerr << "Invalid rotation matrix\n"; return 1;}
  Eigen::Quaterniond quat(rotation);
  quat.normalize();
  if (quat.w() < 0) {quat.coeffs() *= -1;}
  const double horizontal = std::hypot(rotation(0, 0), rotation(1, 0));
  const double pitch = std::atan2(-rotation(2, 0), horizontal);
  const double roll = horizontal > 1e-8 ? std::atan2(rotation(2, 1), rotation(2, 2)) :
    std::atan2(-rotation(1, 2), rotation(1, 1));
  const double yaw = horizontal > 1e-8 ? std::atan2(rotation(1, 0), rotation(0, 0)) : 0.0;
  const double deg = 180.0 / std::acos(-1.0);
  const auto captured_time = std::chrono::system_clock::to_time_t(captured);
  std::cout << std::fixed << std::setprecision(9)
            << "host_capture_time: " << std::put_time(std::localtime(&captured_time), "%FT%T%z")
            << "\nhost_capture_unix_ms: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(captured.time_since_epoch()).count()
            << "\nleft_feedback_frame: " << *last_frame
            << "\nreference: SDK left arm base; point: flange (identity tool)\n"
            << "joint_degrees: [";
  for (int j = 0; j < 7; ++j) {std::cout << (j ? ", " : "") << q[j];}
  std::cout << "]\nposition_m: [" << pose[0][3] / 1000 << ", " << pose[1][3] / 1000
            << ", " << pose[2][3] / 1000 << "]\nquaternion_xyzw: ["
            << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w()
            << "]\nrpy_degrees (R=Rz(yaw)*Ry(pitch)*Rx(roll)): ["
            << roll * deg << ", " << pitch * deg << ", " << yaw * deg
            << "]\nT_base_flange (translation_mm):\n";
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {std::cout << std::setw(17) << pose[i][j];}
    std::cout << '\n';
  }
  return 0;
}
