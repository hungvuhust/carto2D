

#include "cartographer_ros/tf_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
  tf2::Duration timeout(tf2::durationFromSec(lookup_transform_timeout_sec_));
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
  try {
    const rclcpp::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::rclcpp::Time(0.),
                              timeout)
            .header.stamp;
    const rclcpp::Time requested_time = ToRos(time);

    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = tf2::durationFromSec(0.0);
    }
    return absl::make_unique<::cartographer::transform::Rigid3d>(
        ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                           requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace cartographer_ros
