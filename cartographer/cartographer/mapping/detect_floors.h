

#ifndef CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
#define CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"

namespace cartographer {
namespace mapping {

struct Timespan {
  common::Time start;
  common::Time end;
};

struct Floor {
  // The spans of time we spent on this floor. Since we might have walked up and
  // down many times in this place, there can be many spans of time we spent on
  // a particular floor.
  std::vector<Timespan> timespans;

  // The median z-value of this floor.
  double z;
};

// Uses a heuristic which looks at z-values of the poses to detect individual
// floors of a building. This requires that floors are *mostly* on the same
// z-height and that level changes happen *relatively* abrubtly, e.g. by taking
// the stairs.
std::vector<Floor> DetectFloors(const proto::Trajectory &trajectory);

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
