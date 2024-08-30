

#ifndef CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include <string>
#include <vector>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

class Histogram {
public:
  void        Add(float value);
  std::string ToString(int buckets) const;

private:
  std::vector<float> values_;
};

} // namespace common
} // namespace cartographer

#endif // CARTOGRAPHER_COMMON_HISTOGRAM_H_
