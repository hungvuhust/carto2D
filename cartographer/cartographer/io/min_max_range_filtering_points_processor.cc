

#include "cartographer/io/min_max_range_filtering_points_processor.h"

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

std::unique_ptr<MinMaxRangeFilteringPointsProcessor>
    MinMaxRangeFilteringPointsProcessor::FromDictionary(
        common::LuaParameterDictionary *const dictionary,
        PointsProcessor *const                next) {
  return absl::make_unique<MinMaxRangeFilteringPointsProcessor>(
      dictionary->GetDouble("min_range"), dictionary->GetDouble("max_range"),
      next);
}

MinMaxRangeFilteringPointsProcessor::MinMaxRangeFilteringPointsProcessor(
    const double min_range, const double max_range, PointsProcessor *next)
    : min_range_squared_(min_range * min_range),
      max_range_squared_(max_range * max_range), next_(next) {}

void MinMaxRangeFilteringPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  absl::flat_hash_set<int> to_remove;
  for (size_t i= 0; i < batch->points.size(); ++i) {
    const float range_squared=
        (batch->points[i].position - batch->origin).squaredNorm();
    if (!(min_range_squared_ <= range_squared &&
          range_squared <= max_range_squared_)) {
      to_remove.insert(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult MinMaxRangeFilteringPointsProcessor::Flush() {
  return next_->Flush();
}

} // namespace io
} // namespace cartographer
