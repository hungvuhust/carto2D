

#include <regex>
#include <string>

#include "cartographer_ros/assets_writer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read and use the transforms from the bag.");
DEFINE_string(output_file_prefix, "",
              "Will be prefixed to all output file names and can be used to "
              "define the output directory. If empty, the first bag filename "
              "will be used.");

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  FLAGS_alsologtostderr = true;
  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
  CHECK(!FLAGS_pose_graph_filename.empty())
      << "-pose_graph_filename is missing.";

  std::regex regex(",");
  std::vector<std::string> bag_filenames(
      std::sregex_token_iterator(FLAGS_bag_filenames.begin(),
                                 FLAGS_bag_filenames.end(), regex, -1),
      std::sregex_token_iterator());

  ::cartographer_ros::AssetsWriter asset_writer(
      FLAGS_pose_graph_filename, bag_filenames, FLAGS_output_file_prefix);

  asset_writer.Run(FLAGS_configuration_directory, FLAGS_configuration_basename,
                   FLAGS_urdf_filename, FLAGS_use_bag_transforms);
  rclcpp::shutdown();
}
