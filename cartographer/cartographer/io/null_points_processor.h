

#ifndef CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_

#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// A points processor that just drops all points. The end of a pipeline usually.
class NullPointsProcessor : public PointsProcessor {
public:
  NullPointsProcessor() {}
  ~NullPointsProcessor() override {}

  void        Process(std::unique_ptr<PointsBatch> points_batch) override {}
  FlushResult Flush() override { return FlushResult::kFinished; }
};

} // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_NULL_POINTS_PROCESSOR_H_
