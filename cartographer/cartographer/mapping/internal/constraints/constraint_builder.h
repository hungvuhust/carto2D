

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/pose_graph/constraint_builder_options.pb.h"

namespace cartographer {
namespace mapping {
namespace constraints {

proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
    common::LuaParameterDictionary *parameter_dictionary);

} // namespace constraints
} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_H_
