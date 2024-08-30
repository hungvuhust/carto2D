

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H

#include <vector>

#include "cartographer/common/port.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string               &urdf_filename,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H
