

#include "cartographer_ros/time_conversion.h"

#include <chrono>

#include "cartographer/common/time.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

namespace cartographer_ros {
namespace {

TEST(TimeConversion, testToRos) {
  std::vector<int64_t> values = {0, 1469091375, 1466481821, 1462101382,
                                 1468238899};
  for (int64_t seconds_since_epoch : values) {
    builtin_interfaces::msg::Time ros_now;
    ros_now.fromSec(seconds_since_epoch);
    ::cartographer::common::Time cartographer_now(
        ::cartographer::common::FromSeconds(
            seconds_since_epoch +
            ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds));
    EXPECT_EQ(cartographer_now, ::cartographer_ros::FromRos(ros_now));
    EXPECT_EQ(ros_now, ::cartographer_ros::ToRos(cartographer_now));
  }
}

}  // namespace
}  // namespace cartographer_ros
