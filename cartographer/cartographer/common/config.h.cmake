#ifndef CARTOGRAPHER_COMMON_CONFIG_H_
#define CARTOGRAPHER_COMMON_CONFIG_H_

namespace cartographer {
namespace common {

constexpr char kConfigurationFilesDirectory[] =
    "@CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY@";
constexpr char kSourceDirectory[] = "@PROJECT_SOURCE_DIR@";

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_CONFIG_H_