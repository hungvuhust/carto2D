

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

template <typename DataType> class Dispatchable : public Data {
public:
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data) {}

  common::Time GetTime() const override { return data_.time; }
  void         AddToTrajectoryBuilder(
              mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }
  const DataType &data() const { return data_; }

private:
  const DataType data_;
};

template <typename DataType>
std::unique_ptr<Dispatchable<DataType>>
    MakeDispatchable(const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

} // namespace sensor
} // namespace cartographer

#endif // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
