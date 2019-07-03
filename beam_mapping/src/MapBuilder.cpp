#include "beam_mapping/MapBuilder.h"

#include "beam_utils/log.hpp"
#include <fstream>

namespace beam_mapping {

MapBuilder::MapBuilder(const std::string& config_file) {
  config_file_ = config_file;
  this->LoadConfigFromJSON(config_file);
}

void MapBuilder::LoadConfigFromJSON(const std::string& config_file) {
  BEAM_INFO("Loading MapBuilder config file: {}", config_file.c_str());
  nlohmann::json J;
  std::ifstream file(config_file);
  file >> J;
  pose_file_path_ = J["pose_file"];
  bag_file_path_ = J["bag_file_path"];
  bag_file_name_ = J["bag_file_name"];
  save_dir_ = J["save_directory"];
}

void MapBuilder::OverrideBagFile(const std::string& bag_file) {
  bag_file_path_ = bag_file;
}

void MapBuilder::OverridePoseFile(const std::string& pose_file) {
  pose_file_path_ = pose_file;
}

void MapBuilder::LoadPosesFromJSON(const std::string& pose_file) {
  poses_.LoadPoseFile(pose_file);

  if (poses_.GetBagName() != bag_file_name_) {
    BEAM_CRITICAL("Bag file name from MapBuilder config file is not the same "
                  "as the name listed in the pose file.");
    throw std::invalid_argument{
        "Bag file name from MapBuilder config file is "
        "not the same as the name listed in the pose file."};
  }

  int num_poses = poses_.time_stamps.size();
  for (int k = 0; k < num_poses; k++) {
    trajectory_.AddTransform(poses_.poses[k], poses_.moving_frame, poses_.fixed_frame,
                             poses_.time_stamps[k]);
  }
}

void MapBuilder::BuildMap() {
  // Load
  this->LoadPosesFromJSON(pose_file_path_);
}

} // namespace beam_mapping
