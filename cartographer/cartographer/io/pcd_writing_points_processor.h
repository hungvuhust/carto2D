

#include <fstream>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

// Streams a PCD file to disk. The header is written in 'Flush'.
class PcdWritingPointsProcessor : public PointsProcessor {
public:
  constexpr static const char *kConfigurationFileActionName= "write_pcd";
  PcdWritingPointsProcessor(
      std::unique_ptr<FileWriter> file_writer, PointsProcessor *next);

  static std::unique_ptr<PcdWritingPointsProcessor> FromDictionary(
      FileWriterFactory               file_writer_factory,
      common::LuaParameterDictionary *dictionary, PointsProcessor *next);

  ~PcdWritingPointsProcessor() override {}

  PcdWritingPointsProcessor(const PcdWritingPointsProcessor &)= delete;
  PcdWritingPointsProcessor &
      operator=(const PcdWritingPointsProcessor &)= delete;

  void        Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

private:
  PointsProcessor *const next_;

  int64                       num_points_;
  bool                        has_colors_;
  std::unique_ptr<FileWriter> file_writer_;
};

} // namespace io
} // namespace cartographer
