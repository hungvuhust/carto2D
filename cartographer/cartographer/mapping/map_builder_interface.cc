

#include "cartographer/mapping/map_builder_interface.h"

#include "cartographer/mapping/pose_graph.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary) {
  proto::MapBuilderOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));
  options.set_collate_by_trajectory(
      parameter_dictionary->GetBool("collate_by_trajectory"));
  *options.mutable_pose_graph_options()= CreatePoseGraphOptions(
      parameter_dictionary->GetDictionary("pose_graph").get());
  return options;
}

} // namespace mapping
} // namespace cartographer
