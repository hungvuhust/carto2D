

#include "cartographer_ros/time_conversion.h"

#include <builtin_interfaces/msg/time.hpp>

#include "cartographer/common/time.h"

namespace cartographer_ros {

rclcpp::Time ToRos(::cartographer::common::Time time) {
  int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
  int64_t ns_since_unix_epoch =
      (uts_timestamp -
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;
  rclcpp::Time ros_time(ns_since_unix_epoch, rcl_clock_type_t::RCL_ROS_TIME);
  return ros_time;
}

// TODO(pedrofernandez): Write test.
::cartographer::common::Time FromRos(const rclcpp::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll +
      (time.nanoseconds() + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace cartographer_ros
