

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H

#include "cartographer/common/time.h"
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cartographer_ros {

rclcpp::Time ToRos(::cartographer::common::Time time);

::cartographer::common::Time FromRos(const rclcpp::Time &time);

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
