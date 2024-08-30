

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_WRITING_POINTS_PROCESSOR_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_WRITING_POINTS_PROCESSOR_H

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/proto/probability_grid_range_data_inserter_options_2d.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"

namespace cartographer_ros {

// Very similar to Cartographer's ProbabilityGridPointsProcessor, but writes
// out a PGM and YAML suitable for ROS map server to consume.
class RosMapWritingPointsProcessor
    : public ::cartographer::io::PointsProcessor {
public:
  constexpr static const char *kConfigurationFileActionName= "write_ros_map";
  RosMapWritingPointsProcessor(
      double resolution,
      const ::cartographer::mapping::proto::
          ProbabilityGridRangeDataInserterOptions2D
                                           &range_data_inserter_options,
      ::cartographer::io::FileWriterFactory file_writer_factory,
      const std::string &filestem, PointsProcessor *next);
  RosMapWritingPointsProcessor(const RosMapWritingPointsProcessor &)= delete;
  RosMapWritingPointsProcessor &
      operator=(const RosMapWritingPointsProcessor &)= delete;

  static std::unique_ptr<RosMapWritingPointsProcessor> FromDictionary(
      ::cartographer::io::FileWriterFactory           file_writer_factory,
      ::cartographer::common::LuaParameterDictionary *dictionary,
      PointsProcessor                                *next);

  ~RosMapWritingPointsProcessor() override {}

  void Process(std::unique_ptr<::cartographer::io::PointsBatch> batch) override;
  FlushResult Flush() override;

private:
  const std::string                     filestem_;
  PointsProcessor *const                next_;
  ::cartographer::io::FileWriterFactory file_writer_factory_;
  ::cartographer::mapping::ProbabilityGridRangeDataInserter2D
                                                 range_data_inserter_;
  ::cartographer::mapping::ValueConversionTables conversion_tables_;
  ::cartographer::mapping::ProbabilityGrid       probability_grid_;
};

} // namespace cartographer_ros

#endif // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_WRITING_POINTS_PROCESSOR_H
