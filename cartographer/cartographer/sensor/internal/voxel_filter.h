

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_

#include <bitset>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace sensor {

std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint> &points, const float resolution);
PointCloud VoxelFilter(const PointCloud &point_cloud, const float resolution);
TimedPointCloud VoxelFilter(
    const TimedPointCloud &timed_point_cloud, const float resolution);
std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>
               &range_measurements,
    const float resolution);

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary *const parameter_dictionary);

PointCloud AdaptiveVoxelFilter(
    const PointCloud                        &point_cloud,
    const proto::AdaptiveVoxelFilterOptions &options);

} // namespace sensor
} // namespace cartographer

#endif // CARTOGRAPHER_SENSOR_INTERNAL_VOXEL_FILTER_H_
