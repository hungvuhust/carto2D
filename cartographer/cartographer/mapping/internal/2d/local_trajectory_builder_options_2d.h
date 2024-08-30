

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_2D_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"

namespace cartographer {
namespace mapping {

proto::LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
    common::LuaParameterDictionary *parameter_dictionary);

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_OPTIONS_2D_H_
