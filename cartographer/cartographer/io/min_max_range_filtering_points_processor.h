

#ifndef CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Filters all points that are farther away from their 'origin' as 'max_range'
// or closer than 'min_range'.
class MinMaxRangeFilteringPointsProcessor : public PointsProcessor {
public:
  constexpr static const char *kConfigurationFileActionName=
      "min_max_range_filter";
  MinMaxRangeFilteringPointsProcessor(
      double min_range, double max_range, PointsProcessor *next);
  static std::unique_ptr<MinMaxRangeFilteringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~MinMaxRangeFilteringPointsProcessor() override {}

  MinMaxRangeFilteringPointsProcessor(
      const MinMaxRangeFilteringPointsProcessor &)= delete;
  MinMaxRangeFilteringPointsProcessor &
      operator=(const MinMaxRangeFilteringPointsProcessor &)= delete;

  void        Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

private:
  const double           min_range_squared_;
  const double           max_range_squared_;
  PointsProcessor *const next_;
};

} // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_MIN_MAX_RANGE_FILTERING_POINTS_PROCESSOR_H_
