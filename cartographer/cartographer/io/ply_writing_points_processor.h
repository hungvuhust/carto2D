

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Streams a PLY file to disk. The header is written in 'Flush'.
class PlyWritingPointsProcessor : public PointsProcessor {
public:
  constexpr static const char *kConfigurationFileActionName= "write_ply";
  PlyWritingPointsProcessor(
      std::unique_ptr<FileWriter>     file_writer,
      const std::vector<std::string> &comments, PointsProcessor *next);

  static std::unique_ptr<PlyWritingPointsProcessor> FromDictionary(
      const FileWriterFactory        &file_writer_factory,
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~PlyWritingPointsProcessor() override {}

  PlyWritingPointsProcessor(const PlyWritingPointsProcessor &)= delete;
  PlyWritingPointsProcessor &
      operator=(const PlyWritingPointsProcessor &)= delete;

  void        Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

private:
  PointsProcessor *const next_;

  std::vector<std::string>    comments_;
  int64                       num_points_;
  bool                        has_colors_;
  bool                        has_intensities_;
  std::unique_ptr<FileWriter> file_;
};

} // namespace io
} // namespace cartographer
