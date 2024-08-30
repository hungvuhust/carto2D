

#ifndef CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/io/points_batch.h"

namespace cartographer {
namespace io {

// A processor in a pipeline. It processes a 'points_batch' and hands it to the
// next processor in the pipeline.
class PointsProcessor {
public:
  enum class FlushResult {
    kRestartStream,
    kFinished,
  };

  PointsProcessor() {}
  virtual ~PointsProcessor() {}

  PointsProcessor(const PointsProcessor &)           = delete;
  PointsProcessor &operator=(const PointsProcessor &)= delete;

  // Receive a 'points_batch', process it and pass it on.
  virtual void Process(std::unique_ptr<PointsBatch> points_batch)= 0;

  // Some implementations will perform expensive computations and others that do
  // multiple passes over the data might ask for restarting the stream.
  virtual FlushResult Flush()= 0;
};

} // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_POINTS_PROCESSOR_H_
