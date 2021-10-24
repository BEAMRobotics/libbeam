#include <gflags/gflags.h>

#include <beam_mapping/MapBuilder.h>
#include <beam_utils/gflags.h>

#include <iostream>

DEFINE_string(
    config_file, "",
    "Full file path to config file (ex. /path/to/config/file.json (or .ply))");
DEFINE_validator(config_file, &beam::gflags::ValidateFileMustExist);
DEFINE_string(odometry_topic, "", "odometry topic containing poses");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  beam_mapping::MapBuilder map_builder(FLAGS_config_file);
  if (!FLAGS_odometry_topic.empty()) {
    std::string bag_file = map_builder.GetBagFile();
    std::string save_dir = map_builder.GetSaveDir();
    beam_mapping::Poses bag_poses;
    BEAM_INFO("Loading poses from bag: {} and topic: {}", bag_file,
              FLAGS_odometry_topic);
    bag_poses.LoadFromBAG(bag_file, FLAGS_odometry_topic);
    bag_poses.WriteToJSON(save_dir);
    map_builder.OverridePoseFile(save_dir + "_poses.json");
  }
  map_builder.BuildMap();
  return 0;
}
