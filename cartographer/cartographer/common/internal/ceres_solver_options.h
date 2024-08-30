

#ifndef CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
#define CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/proto/ceres_solver_options.pb.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace common {

proto::CeresSolverOptions CreateCeresSolverOptionsProto(
    common::LuaParameterDictionary *parameter_dictionary);

ceres::Solver::Options
    CreateCeresSolverOptions(const proto::CeresSolverOptions &proto);

} // namespace common
} // namespace cartographer

#endif // CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
