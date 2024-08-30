

#include "cartographer/io/pcd_writing_points_processor.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

// Writes the PCD header claiming 'num_points' will follow it into
// 'output_file'.
void WriteBinaryPcdHeader(
    const bool has_color, const int64 num_points,
    FileWriter *const file_writer) {
  std::string color_header_field= !has_color ? "" : " rgb";
  std::string color_header_type = !has_color ? "" : " U";
  std::string color_header_size = !has_color ? "" : " 4";
  std::string color_header_count= !has_color ? "" : " 1";

  std::ostringstream stream;
  stream << "# generated by Cartographer\n"
         << "VERSION .7\n"
         << "FIELDS x y z" << color_header_field << "\n"
         << "SIZE 4 4 4" << color_header_size << "\n"
         << "TYPE F F F" << color_header_type << "\n"
         << "COUNT 1 1 1" << color_header_count << "\n"
         << "WIDTH " << std::setw(15) << std::setfill('0') << num_points << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << std::setw(15) << std::setfill('0') << num_points
         << "\n"
         << "DATA binary\n";
  const std::string out= stream.str();
  file_writer->WriteHeader(out.data(), out.size());
}

void WriteBinaryPcdPointCoordinate(
    const Eigen::Vector3f &point, FileWriter *const file_writer) {
  char buffer[12];
  memcpy(buffer, &point[0], sizeof(float));
  memcpy(buffer + 4, &point[1], sizeof(float));
  memcpy(buffer + 8, &point[2], sizeof(float));
  CHECK(file_writer->Write(buffer, 12));
}

void WriteBinaryPcdPointColor(
    const Uint8Color &color, FileWriter *const file_writer) {
  char buffer[4];
  buffer[0]= color[2];
  buffer[1]= color[1];
  buffer[2]= color[0];
  buffer[3]= 0;
  CHECK(file_writer->Write(buffer, 4));
}

} // namespace

std::unique_ptr<PcdWritingPointsProcessor>
    PcdWritingPointsProcessor::FromDictionary(
        FileWriterFactory                     file_writer_factory,
        common::LuaParameterDictionary *const dictionary,
        PointsProcessor *const                next) {
  return absl::make_unique<PcdWritingPointsProcessor>(
      file_writer_factory(dictionary->GetString("filename")), next);
}

PcdWritingPointsProcessor::PcdWritingPointsProcessor(
    std::unique_ptr<FileWriter> file_writer, PointsProcessor *const next)
    : next_(next), num_points_(0), has_colors_(false),
      file_writer_(std::move(file_writer)) {}

PointsProcessor::FlushResult PcdWritingPointsProcessor::Flush() {
  WriteBinaryPcdHeader(has_colors_, num_points_, file_writer_.get());
  CHECK(file_writer_->Close());

  switch (next_->Flush()) {
  case FlushResult::kFinished: return FlushResult::kFinished;

  case FlushResult::kRestartStream:
    LOG(FATAL) << "PCD generation must be configured to occur after any "
                  "stages that require multiple passes.";
  }
  LOG(FATAL);
}

void PcdWritingPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (batch->points.empty()) {
    next_->Process(std::move(batch));
    return;
  }

  if (num_points_ == 0) {
    has_colors_= !batch->colors.empty();
    WriteBinaryPcdHeader(has_colors_, 0, file_writer_.get());
  }
  for (size_t i= 0; i < batch->points.size(); ++i) {
    WriteBinaryPcdPointCoordinate(
        batch->points[i].position, file_writer_.get());
    if (!batch->colors.empty()) {
      WriteBinaryPcdPointColor(
          ToUint8Color(batch->colors[i]), file_writer_.get());
    }
    ++num_points_;
  }
  next_->Process(std::move(batch));
}

} // namespace io
} // namespace cartographer
