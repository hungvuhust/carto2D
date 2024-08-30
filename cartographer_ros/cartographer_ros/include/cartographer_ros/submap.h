

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SUBMAP_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SUBMAP_H

#include <memory>
#include <string>
#include <vector>

#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include <rclcpp/rclcpp.hpp>

namespace cartographer_ros {

// Fetch 'submap_id' using the 'client' and returning the response or 'nullptr'
// on error.
std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId &submap_id,
    rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr
                                    callback_group_executor,
    const std::chrono::milliseconds timeout);

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SUBMAP_H
