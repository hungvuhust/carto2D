

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_

#include <functional>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/data.h"
#include "cartographer/sensor/internal/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

class Collator : public CollatorInterface {
public:
  Collator() {}

  Collator(const Collator &)           = delete;
  Collator &operator=(const Collator &)= delete;

  void AddTrajectory(
      int                                     trajectory_id,
      const absl::flat_hash_set<std::string> &expected_sensor_ids,
      const Callback                         &callback) override;

  void FinishTrajectory(int trajectory_id) override;

  void AddSensorData(int trajectory_id, std::unique_ptr<Data> data) override;

  void Flush() override;

  absl::optional<int> GetBlockingTrajectoryId() const override;

private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  OrderedMultiQueue queue_;

  // Map of trajectory ID to all associated QueueKeys.
  absl::flat_hash_map<int, std::vector<QueueKey>> queue_keys_;
};

} // namespace sensor
} // namespace cartographer

#endif // CARTOGRAPHER_SENSOR_INTERNAL_COLLATOR_H_
