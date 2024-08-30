

#ifndef CARTOGRAPHER_MAPPING_DATA_H_
#define CARTOGRAPHER_MAPPING_DATA_H_

#include "absl/memory/memory.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {

namespace mapping {
class TrajectoryBuilderInterface;
}

namespace sensor {

class Data {
public:
  explicit Data(const std::string &sensor_id) : sensor_id_(sensor_id) {}
  virtual ~Data() {}

  virtual common::Time GetTime() const= 0;
  const std::string   &GetSensorId() const { return sensor_id_; }
  virtual void         AddToTrajectoryBuilder(
              mapping::TrajectoryBuilderInterface *trajectory_builder)= 0;

protected:
  const std::string sensor_id_;
};

} // namespace sensor
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_DATA_H_
