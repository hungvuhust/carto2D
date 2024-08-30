

#include "cartographer/io/counting_points_processor.h"

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

CountingPointsProcessor::CountingPointsProcessor(PointsProcessor *next)
    : num_points_(0), next_(next) {}

std::unique_ptr<CountingPointsProcessor>
    CountingPointsProcessor::FromDictionary(
        common::LuaParameterDictionary *const dictionary,
        PointsProcessor *const                next) {
  return absl::make_unique<CountingPointsProcessor>(next);
}

void CountingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  num_points_+= batch->points.size();
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult CountingPointsProcessor::Flush() {
  switch (next_->Flush()) {
  case FlushResult::kFinished:
    LOG(INFO) << "Processed " << num_points_ << " and finishing.";
    return FlushResult::kFinished;

  case FlushResult::kRestartStream:
    LOG(INFO) << "Processed " << num_points_ << " and restarting stream.";
    num_points_= 0;
    return FlushResult::kRestartStream;
  }
  LOG(FATAL);
}

} // namespace io
} // namespace cartographer
