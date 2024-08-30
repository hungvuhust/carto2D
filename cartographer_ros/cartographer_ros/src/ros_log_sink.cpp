

#include "cartographer_ros/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "glog/log_severity.h"
#include "glog/logging.h"

namespace cartographer_ros {

namespace {

const char* GetBasename(const char* filepath) {
  const char* base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

}  // namespace

ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  (void)base_filename;  // TODO: remove unused arg ?

  // Google glog broke the ToString API, but has no way to tell what version it
  // is using. To support both newer and older glog versions, use a nasty hack
  // were we infer the version based on whether GOOGLE_GLOG_DLL_DECL is defined
#if defined(GOOGLE_GLOG_DLL_DECL)
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
#else
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, ::google::LogMessageTime(*tm_time),
      message, message_len);
#endif
  switch (severity) {
    case ::google::GLOG_INFO:
      RCLCPP_INFO_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_WARNING:
      RCLCPP_WARN_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_ERROR:
      RCLCPP_ERROR_STREAM(logger_, message_string);
      break;

    case ::google::GLOG_FATAL:
      RCLCPP_FATAL_STREAM(logger_, message_string);
      will_die_ = true;
      break;
  }
}

void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_ros
