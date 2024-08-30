

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_OPTIONS_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_OPTIONS_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/pose_graph/optimization_problem_options.pb.h"

namespace cartographer {
namespace mapping {
namespace optimization {

proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::LuaParameterDictionary *parameter_dictionary);

} // namespace optimization
} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_OPTIONS_H_
