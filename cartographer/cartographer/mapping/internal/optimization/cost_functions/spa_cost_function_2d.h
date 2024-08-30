

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_

#include "cartographer/mapping/pose_graph_interface.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace optimization {

ceres::CostFunction *CreateAutoDiffSpaCostFunction(
    const PoseGraphInterface::Constraint::Pose &pose);

ceres::CostFunction *CreateAnalyticalSpaCostFunction(
    const PoseGraphInterface::Constraint::Pose &pose);

} // namespace optimization
} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
