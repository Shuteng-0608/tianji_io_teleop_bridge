#include <limits>
#include "gtest/gtest.h"
#include "tianji_io_teleop_bridge/tool_configuration.hpp"

using namespace tianji_io_teleop_bridge;
TEST(ToolConfiguration, SnapshotUsesCurrentSelectedProfilesNotFirstOrSameSlot)
{
  const std::string root = TOOL_TEST_SOURCE;
  const auto left = load_tool_configuration(root + "/config/tools_cfg.json", 0);
  const auto right = load_tool_configuration(root + "/config/tools_cfg.json", 1);
  EXPECT_EQ(left.profile, "tool-2");
  EXPECT_EQ(right.profile, "tool-1");
  const std::array<double, 10> ld{0.195,-22.133,-42.953,13.967,0.003,0,0,0.008,0,0.008};
  const std::array<double, 10> rd{0.166,-47.89,-3.741,28.536,0.001,0,0,0.016,0,0.004};
  EXPECT_EQ(left.dyn, ld);
  EXPECT_EQ(right.dyn, rd);
  EXPECT_EQ(left.kine, (std::array<double, 6>{}));
  EXPECT_EQ(right.kine, (std::array<double, 6>{}));
}
TEST(ToolConfiguration, InvalidFileSchemaAndArmAreRejectedWithoutFallback)
{
  const std::string root = TOOL_TEST_SOURCE;
  EXPECT_ANY_THROW(load_tool_configuration(root + "/test/fixtures/tools_bad_length.json", 0));
  EXPECT_ANY_THROW(load_tool_configuration(root + "/test/fixtures/tools_duplicate.json", 0));
  EXPECT_ANY_THROW(load_tool_configuration(root + "/test/fixtures/tools_missing.json", 0));
  EXPECT_ANY_THROW(load_tool_configuration(root + "/config/tools_cfg.json", 2));
}
TEST(ToolConfiguration, FiniteParametersAndFloatEchoMustMatchAllFields)
{
  ToolConfiguration config;
  config.dyn[0] = 0.195;
  config.dyn[1] = -22.133;
  float kine[6]{}, dyn[10]{};
  dyn[0] = static_cast<float>(config.dyn[0]);
  dyn[1] = static_cast<float>(config.dyn[1]);
  EXPECT_TRUE(tool_configuration_matches(config, kine, dyn));
  for (int j = 0; j < 6; ++j) {
    kine[j] = 0.01f;
    EXPECT_FALSE(tool_configuration_matches(config, kine, dyn));
    kine[j] = 0;
  }
  dyn[9] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(tool_configuration_matches(config, kine, dyn));
  config.dyn[0] = -1;
  EXPECT_FALSE(valid_tool_configuration(config));
  config.dyn[0] = 1;
  config.kine[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(valid_tool_configuration(config));
}
