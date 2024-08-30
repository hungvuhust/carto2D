

#include <string>
#include <vector>

#include "cartographer_ros/node_options.h"
#include "gtest/gtest.h"
#include "ros/package.h"

namespace cartographer_ros {
namespace {

class ConfigurationFilesTest : public ::testing::TestWithParam<const char*> {};

TEST_P(ConfigurationFilesTest, ValidateNodeOptions) {
  EXPECT_NO_FATAL_FAILURE({
    LoadOptions(
        ::ros::package::getPath("cartographer_ros") + "/configuration_files",
        GetParam());
  });
}

INSTANTIATE_TEST_CASE_P(
    ValidateAllNodeOptions, ConfigurationFilesTest,
    ::testing::Values("backpack_2d.lua", "backpack_2d_localization.lua",
                      "backpack_3d.lua", "backpack_3d_localization.lua",
                      "pr2.lua", "revo_lds.lua", "taurob_tracker.lua"));

}  // namespace
}  // namespace cartographer_ros
