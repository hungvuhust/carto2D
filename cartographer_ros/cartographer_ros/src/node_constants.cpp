

#include "cartographer_ros/node_constants.h"

#include "glog/logging.h"

namespace cartographer_ros {

std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   const int num_topics) {
  CHECK_GE(num_topics, 0);
  if (num_topics == 1) {
    return {topic};
  }
  std::vector<std::string> topics;
  topics.reserve(num_topics);
  for (int i = 0; i < num_topics; ++i) {
    topics.emplace_back(topic + "_" + std::to_string(i + 1));
  }
  return topics;
}

}  // namespace cartographer_ros
