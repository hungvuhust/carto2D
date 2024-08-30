

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary *parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
public:
  explicit MotionFilter(const proto::MotionFilterOptions &options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const transform::Rigid3d &pose);

private:
  const proto::MotionFilterOptions options_;
  int                              num_total_    = 0;
  int                              num_different_= 0;
  common::Time                     last_time_;
  transform::Rigid3d               last_pose_;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
