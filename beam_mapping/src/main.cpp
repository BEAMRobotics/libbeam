#include "beam_mapping/MapBuilder.h"
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "----------------------------------------------------------\n"
              << "**USAGE SUMMARY**\n"
              << "If loading the poses from a pose file: \n"
              << "USAGE: " << argv[0] <<" /path/to/config/file.json (or .ply)\n"
              << "If loading the poses from a bag file: \n"
              << "USAGE: " << argv[0] << " /path/to/config/file.json (or .ply) "
              << "odometry_topic\n"
              << "----------------------------------------------------------\n";
    return 1;
  } else if (argc == 2) {
    std::string config_file;
    config_file = argv[1];
    beam_mapping::MapBuilder map_builder(config_file);
    map_builder.BuildMap();
  } else if (argc == 3) {
    std::string config_file;
    config_file = argv[1];
    std::string odometry_topic;
    odometry_topic = argv[2];
    beam_mapping::MapBuilder map_builder(config_file);
    std::string bag_file = map_builder.GetBagFile();
    std::string save_dir = map_builder.GetSaveDir();
    beam_mapping::Poses bag_poses;
    BEAM_INFO("Loading poses from bag: {} and topic: {}", bag_file.c_str(),
              odometry_topic.c_str());
    bag_poses.LoadFromBAG(bag_file, odometry_topic);
    bag_poses.WriteToJSON(save_dir);
    map_builder.OverridePoseFile(save_dir + "_poses.json");
    map_builder.BuildMap();
  }
  return 0;
}
