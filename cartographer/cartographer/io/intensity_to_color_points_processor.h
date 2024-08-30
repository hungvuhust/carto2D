

#ifndef CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

class IntensityToColorPointsProcessor : public PointsProcessor {
public:
  constexpr static const char *kConfigurationFileActionName=
      "intensity_to_color";

  // Applies ('intensity' - min ) / (max - min) * 255 and color the point grey
  // with this value for each point that comes from the sensor with 'frame_id'.
  // If 'frame_id' is empty, this applies to all points.
  IntensityToColorPointsProcessor(
      float min_intensity, float max_intensity, const std::string &frame_id,
      PointsProcessor *next);

  static std::unique_ptr<IntensityToColorPointsProcessor> FromDictionary(
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~IntensityToColorPointsProcessor() override{};

  IntensityToColorPointsProcessor(const IntensityToColorPointsProcessor &)=
      delete;
  IntensityToColorPointsProcessor &
      operator=(const IntensityToColorPointsProcessor &)= delete;

  void        Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

private:
  const float            min_intensity_;
  const float            max_intensity_;
  const std::string      frame_id_;
  PointsProcessor *const next_;
};

} // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_INTENSITY_TO_COLOR_POINTS_PROCESSOR_H_
