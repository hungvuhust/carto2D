

#include "cartographer/io/fixed_ratio_sampling_points_processor.h"

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

std::unique_ptr<FixedRatioSamplingPointsProcessor>
    FixedRatioSamplingPointsProcessor::FromDictionary(
        common::LuaParameterDictionary *const dictionary,
        PointsProcessor *const                next) {
  const double sampling_ratio(dictionary->GetDouble("sampling_ratio"));
  CHECK_LT(0., sampling_ratio) << "Sampling ratio <= 0 makes no sense.";
  CHECK_LT(sampling_ratio, 1.) << "Sampling ratio >= 1 makes no sense.";
  return absl::make_unique<FixedRatioSamplingPointsProcessor>(
      sampling_ratio, next);
}

FixedRatioSamplingPointsProcessor::FixedRatioSamplingPointsProcessor(
    const double sampling_ratio, PointsProcessor *next)
    : sampling_ratio_(sampling_ratio), next_(next),
      sampler_(new common::FixedRatioSampler(sampling_ratio_)) {}

void FixedRatioSamplingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> batch) {
  absl::flat_hash_set<int> to_remove;
  for (size_t i= 0; i < batch->points.size(); ++i) {
    if (!sampler_->Pulse()) {
      to_remove.insert(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult FixedRatioSamplingPointsProcessor::Flush() {
  switch (next_->Flush()) {
  case PointsProcessor::FlushResult::kFinished:
    return PointsProcessor::FlushResult::kFinished;

  case PointsProcessor::FlushResult::kRestartStream:
    sampler_= absl::make_unique<common::FixedRatioSampler>(sampling_ratio_);
    return PointsProcessor::FlushResult::kRestartStream;
  }
  LOG(FATAL);
}

} // namespace io
} // namespace cartographer
