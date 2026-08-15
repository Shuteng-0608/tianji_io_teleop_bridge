cmake_minimum_required(VERSION 3.16)
project(tianji_io_teleop_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(
  MARVIN_SDK_ROOT
  "/home/stw/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++"
  CACHE PATH
  "Directory containing the compatible MarvinSDK headers and libMarvinSDK.so"
)
set(MARVIN_SDK_LIBRARY "${MARVIN_SDK_ROOT}/libMarvinSDK.so")
set(MARVIN_SDK_HEADER "${MARVIN_SDK_ROOT}/MarvinSDK.h")
set(MARVIN_KINEMATICS_LIBRARY "${MARVIN_SDK_ROOT}/libKine.so")
set(MARVIN_KINEMATICS_HEADER "${MARVIN_SDK_ROOT}/FxRobot.h")
set(MARVIN_KINEMATICS_CONFIG "${MARVIN_SDK_ROOT}/ccs_m6_40.MvKDCfg")
set(
  MARVIN_SDK_EXPECTED_SHA256
  "47ab2ab6e35cf899eae3978dfb50ee219e05b10f72aedd6bc9ba6a338a8b4011"
  CACHE STRING
  "Expected SHA-256 of the validated Marvin SDK shared library"
)
set(
  MARVIN_KINEMATICS_EXPECTED_SHA256
  "f1512f8e440b53614b8d61adc27b217cab1b97d8bb9a80e32e00c880eee73397"
  CACHE STRING
  "Expected SHA-256 of the validated Marvin kinematics shared library"
)
set(
  MARVIN_KINEMATICS_CONFIG_EXPECTED_SHA256
  "ce20b1a80974c3ed58cdac4973152625096c8ad93a779e5fd4108c26ce19bd52"
  CACHE STRING
  "Expected SHA-256 of the validated Marvin M6 CCS 4.0 kinematics configuration"
)

if(NOT EXISTS "${MARVIN_SDK_LIBRARY}" OR NOT EXISTS "${MARVIN_SDK_HEADER}" OR
  NOT EXISTS "${MARVIN_KINEMATICS_LIBRARY}" OR
  NOT EXISTS "${MARVIN_KINEMATICS_HEADER}" OR
  NOT EXISTS "${MARVIN_KINEMATICS_CONFIG}")
  message(FATAL_ERROR "Compatible Marvin SDK not found under: ${MARVIN_SDK_ROOT}")
endif()

file(SHA256 "${MARVIN_SDK_LIBRARY}" MARVIN_SDK_ACTUAL_SHA256)
if(NOT MARVIN_SDK_ACTUAL_SHA256 STREQUAL MARVIN_SDK_EXPECTED_SHA256)
  message(FATAL_ERROR
    "Marvin SDK checksum mismatch. Expected ${MARVIN_SDK_EXPECTED_SHA256}, "
    "got ${MARVIN_SDK_ACTUAL_SHA256}. Verify the SDK with Tianji before building.")
endif()
file(SHA256 "${MARVIN_KINEMATICS_LIBRARY}" MARVIN_KINEMATICS_ACTUAL_SHA256)
if(NOT MARVIN_KINEMATICS_ACTUAL_SHA256 STREQUAL MARVIN_KINEMATICS_EXPECTED_SHA256)
  message(FATAL_ERROR
    "Marvin kinematics SDK checksum mismatch. Expected "
    "${MARVIN_KINEMATICS_EXPECTED_SHA256}, got ${MARVIN_KINEMATICS_ACTUAL_SHA256}.")
endif()
file(SHA256 "${MARVIN_KINEMATICS_CONFIG}" MARVIN_KINEMATICS_CONFIG_ACTUAL_SHA256)
if(NOT MARVIN_KINEMATICS_CONFIG_ACTUAL_SHA256 STREQUAL
  MARVIN_KINEMATICS_CONFIG_EXPECTED_SHA256)
  message(FATAL_ERROR
    "Marvin kinematics configuration checksum mismatch. Expected "
    "${MARVIN_KINEMATICS_CONFIG_EXPECTED_SHA256}, got "
    "${MARVIN_KINEMATICS_CONFIG_ACTUAL_SHA256}.")
endif()

add_executable(marvin_teleop_bridge_node
  src/marvin_teleop_bridge_node.cpp
  src/marvin_kinematics_adapter.cpp
  src/marvin_sdk_adapter.cpp
)
target_include_directories(marvin_teleop_bridge_node PRIVATE
  include
  "${MARVIN_SDK_ROOT}"
)
target_link_directories(marvin_teleop_bridge_node PRIVATE "${MARVIN_SDK_ROOT}")
target_link_libraries(marvin_teleop_bridge_node MarvinSDK Kine)
ament_target_dependencies(
  marvin_teleop_bridge_node ament_index_cpp diagnostic_msgs geometry_msgs rclcpp sensor_msgs
  std_msgs std_srvs)
target_compile_options(marvin_teleop_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(marvin_teleop_bridge_node PROPERTIES
  BUILD_RPATH "${MARVIN_SDK_ROOT}"
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS marvin_teleop_bridge_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/alignment_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME alignment_dashboard
)
install(FILES "${MARVIN_SDK_LIBRARY}" DESTINATION lib)
install(FILES "${MARVIN_KINEMATICS_LIBRARY}" DESTINATION lib)
install(FILES "${MARVIN_KINEMATICS_CONFIG}"
  DESTINATION share/${PROJECT_NAME}/marvin_kinematics)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_model test/test_joint_model.cpp)
  if(TARGET test_joint_model)
    target_include_directories(test_joint_model PRIVATE include)
    ament_target_dependencies(test_joint_model sensor_msgs)
    target_compile_options(test_joint_model PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_control_gate test/test_control_gate.cpp)
  if(TARGET test_control_gate)
    target_include_directories(test_control_gate PRIVATE include)
    target_compile_options(test_control_gate PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_disturbance_messages test/test_disturbance_messages.cpp)
  if(TARGET test_disturbance_messages)
    target_include_directories(test_disturbance_messages PRIVATE include)
    ament_target_dependencies(test_disturbance_messages geometry_msgs sensor_msgs)
    target_compile_options(test_disturbance_messages PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_marvin_kinematics_adapter
    test/test_marvin_kinematics_adapter.cpp
    src/marvin_kinematics_adapter.cpp)
  if(TARGET test_marvin_kinematics_adapter)
    target_include_directories(test_marvin_kinematics_adapter PRIVATE
      include
      "${MARVIN_SDK_ROOT}")
    target_link_directories(test_marvin_kinematics_adapter PRIVATE "${MARVIN_SDK_ROOT}")
    target_link_libraries(test_marvin_kinematics_adapter Kine)
    ament_target_dependencies(test_marvin_kinematics_adapter geometry_msgs sensor_msgs)
    target_compile_definitions(test_marvin_kinematics_adapter PRIVATE
      TIANJI_TEST_KINEMATICS_CONFIG="${MARVIN_KINEMATICS_CONFIG}")
    target_compile_options(test_marvin_kinematics_adapter PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(test_marvin_kinematics_adapter PROPERTIES
      BUILD_RPATH "${MARVIN_SDK_ROOT}")
  endif()
endif()

ament_package()
