

#include "cartographer/mapping/internal/constraints/constraint_builder.h"

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace mapping {
namespace constraints {

proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary) {
  proto::ConstraintBuilderOptions options;
  options.set_sampling_ratio(parameter_dictionary->GetDouble("sampling_ratio"));
  options.set_max_constraint_distance(
      parameter_dictionary->GetDouble("max_constraint_distance"));
  options.set_min_score(parameter_dictionary->GetDouble("min_score"));
  options.set_global_localization_min_score(
      parameter_dictionary->GetDouble("global_localization_min_score"));
  options.set_loop_closure_translation_weight(
      parameter_dictionary->GetDouble("loop_closure_translation_weight"));
  options.set_loop_closure_rotation_weight(
      parameter_dictionary->GetDouble("loop_closure_rotation_weight"));
  options.set_log_matches(parameter_dictionary->GetBool("log_matches"));
  *options.mutable_fast_correlative_scan_matcher_options()=
      scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("fast_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options()=
      scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  return options;
}

} // namespace constraints
} // namespace mapping
} // namespace cartographer
