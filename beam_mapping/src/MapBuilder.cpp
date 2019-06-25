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
  poses_file_path_ = J["poses_file"];
  bag_file_path_ = J["bag_file_path"];
  bag_file_name_ = J["bag_file_name"];
  save_dir_ = J["save_directory"];
}

void MapBuilder::OverrideBagFile(const std::string& bag_file) {
  bag_file_path_ = bag_file;
}

void MapBuilder::OverridePosesFile(const std::string& poses_file) {
  poses_file_path_ = poses_file;
}

void MapBuilder::LoadPosesFromJSON(const std::string& poses_file) {
  BEAM_INFO("Loading MapBuilder config file: {}", poses_file.c_str());
  nlohmann::json J;
  std::ifstream file(poses_file);
  file >> J;
  std::string bag_name_from_poses = J["bag_name"];
  poses_file_date_ = J["poses_file_date"];
  fixed_frame_ = J["fixed_frame"];
  pose_frame_ = J["pose_frame"];

  if (bag_name_from_poses != bag_file_name_) {
    BEAM_CRITICAL("Bag file name from MapBuilder config file is not the same "
                  "as the name listed in the pose file.");
    throw std::invalid_argument {"Bag file name from MapBuilder config file is "
                           "not the same as the name listed in the pose file."};
  }

  poses_file_date_ = J["poses_file_date"];

  for (const auto& pose : J["poses"]){
    double time_stamp_tmp = pose["time_stamp_nsec"];
    double time_stamp_sec = std::floor(time_stamp_tmp / 1000000000);
    double time_stamp_nsec = time_stamp_tmp - time_stamp_sec * 1000000000;
    ros::Time time_stamp_k;
    time_stamp_k.sec = (int) time_stamp_sec;
    time_stamp_k.nsec = (int) time_stamp_nsec;
    beam::Mat4 T;
    int i = 0, j = 0;
    int value_counter = 0;
    for (const auto& value : pose["transform"]) {
      value_counter++;
      T(i, j) = value.get<double>();
      if (j == 3) {
        i++;
        j = 0;
      } else {
        j++;
      }
    }
    if (value_counter != 16) {
      BEAM_CRITICAL("Invalid transform matrix in .json file.");
      throw std::invalid_argument{"Invalid transform matrix in .json file."};
    }

    Eigen::Affine3d TA;
    TA.matrix() = T;
    trajectory_.AddTransform(TA, pose_frame_, fixed_frame_, time_stamp_k);
  }

}

void MapBuilder::BuildMap() {
  // Load
  this->LoadPosesFromJSON(poses_file_path_);
}

} // namespace beam_mapping
