

#ifndef CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Colors points with a fixed color by frame_id.
class ColoringPointsProcessor : public PointsProcessor {
public:
  constexpr static const char *kConfigurationFileActionName= "color_points";

  ColoringPointsProcessor(
      const FloatColor &color, const std::string &frame_id,
      PointsProcessor *next);

  static std::unique_ptr<ColoringPointsProcessor> FromDictionary(
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~ColoringPointsProcessor() override{};

  ColoringPointsProcessor(const ColoringPointsProcessor &)           = delete;
  ColoringPointsProcessor &operator=(const ColoringPointsProcessor &)= delete;

  void        Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

private:
  const FloatColor       color_;
  const std::string      frame_id_;
  PointsProcessor *const next_;
};

} // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_COLORING_POINTS_PROCESSOR_H_
