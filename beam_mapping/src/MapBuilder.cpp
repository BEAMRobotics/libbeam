#include "beam_mapping/MapBuilder.h"

#include "beam_utils/log.hpp"
#include <fstream>

namespace beam_mapping {

using filter_params_type = std::pair<std::string, std::vector<double>>;

MapBuilder::MapBuilder(const std::string& config_file) {
  config_file_ = config_file;
  this->LoadConfigFromJSON(config_file);
}

filter_params_type MapBuilder::GetFilterParams(const auto& filter){
  std::string filter_type = filter["filter_type"];
  std::vector<double> params;
  bool success = true;
  if(filter_type == "DROR"){
    double radius_multiplier = filter["radius_multiplier"];
    double azimuth_angle = filter["azimuth_angle"];
    double min_neighbors = filter["min_neighbors"];
    double min_search_radius = filter["min_search_radius"];
    if(!(radius_multiplier >= 1)){
      BEAM_CRITICAL("Invalid radius_multiplier parameter in map builder config");
      success = false;
    }
    if(!(azimuth_angle > 0.01)){
      BEAM_CRITICAL("Invalid azimuth_angle parameter in map builder config");
      success = false;
    }
    if(!(min_neighbors > 1)){
      BEAM_CRITICAL("Invalid min_neighbors parameter in map builder config");
      success = false;
    }
    if(!(min_search_radius > 0)){
      BEAM_CRITICAL("Invalid min_search_radius parameter in map builder config");
      success = false;
    }
    params.push_back(radius_multiplier);
    params.push_back(azimuth_angle);
    params.push_back(min_neighbors);
    params.push_back(min_search_radius);
  } else if (filter_type == "ROR") {
    double search_radius = filter["search_radius"];
    double min_neighbors = filter["min_neighbors"];
    if(!(search_radius > 0)){
      BEAM_CRITICAL("Invalid search_radius parameter in map builder config");
      success = false;
    }
    if(!(min_neighbors > 1)){
      BEAM_CRITICAL("Invalid min_neighbors parameter in map builder config");
      success = false;
    }
    params.push_back(search_radius);
    params.push_back(min_neighbors);
  } else if (filter_type == "VOXEL"){
    double cell_size = filter["cell_size"];
    if(!(cell_size > 0.01)){
      BEAM_CRITICAL("Invalid cell_size parameter in map builder config");
      success = false;
    }
    params.push_back(cell_size);
  } else {
    BEAM_CRITICAL("Invalid filter type in map builder config.");
    success = false;
  }

  if(!success){
    BEAM_CRITICAL("Invalid filter params in map builder config, see bellow for options and requirements:");
    std::cout << "FILTER TYPES SUPPORTED:\n"
              << "----------------------:\n"
              << "Type: DROR\n"
              << "Required inputs:\n"
              << "- radius_multiplier (required >= 1)\n"
              << "- azimuth_angle (required > 0.01)\n"
              << "- min_neighbors (required > 1)\n"
              << "- min_search_radius (required > 0)\n"
              << "Type: ROR\n"
              << "Required inputs:\n"
              << "- search_radius (required > 0)\n"
              << "- min_neighbors (required > 1)\n"
              << "Type: VOXEL\n"
              << "Required inputs:\n"
              << "- cell_size (required > 0.01)\n";
  }
  return  std::make_pair(filter_type, params);
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
  extrinsics_file_ = J["extrinsics_file"];
  intermediary_map_size_ = J["intermediary_map_size"];
  min_translation_ = J["min_translation"];
  min_rotation_deg_ = J["min_rotation_deg"];
  combine_lidar_scans_ = J["combine_lidar_scans"];

  for (const auto& lidar : J["lidars"]){
    lidar_topics_.push_back(lidar["topic"]);
    lidar_frames_.push_back(lidar["frames"]);
    lidar_cropbox_bool_.push_back(lidar["use_cropbox"]);
    lidar_cropbox_min_.push_back(lidar["cropbox_min"]);
    lidar_cropbox_max_.push_back(lidar["cropbox_max"]);
  }
  for (const auto& filter : J["input_filters"]){
    filter_params_type input_filter;
    input_filter = this->GetFilterParams(filter);
    input_filters_.push_back(input_filter);
  }
  for (const auto& filter : J["intermediary_filters"]){
    filter_params_type intermediary_filter;
    intermediary_filter = this->GetFilterParams(filter);
    intermediary_filters_.push_back(intermediary_filter);
  }
  for (const auto& filter : J["output_filters"]){
    filter_params_type output_filter;
    output_filter = this->GetFilterParams(filter);
    output_filters_.push_back(output_filter);
  }
}

void MapBuilder::OverrideBagFile(const std::string& bag_file) {
  bag_file_path_ = bag_file;
}

void MapBuilder::OverridePoseFile(const std::string& pose_file) {
  pose_file_path_ = pose_file;
}

void MapBuilder::LoadTree(const std::string& pose_file) {
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
    tree_.AddTransform(poses_.poses[k], poses_.moving_frame, poses_.fixed_frame,
                             poses_.time_stamps[k]);
  }

  // Add extrinsic calibration to trajectory tree
  beam_calibration::TfTree extrinsics;
  extrinsics.LoadJSON(extrinsics_file_);
  std::unordered_map<std::string, std::vector<std::string>> frames;
  frames = extrinsics.GetAllFrames();
  for(auto& parents : frames){
    std::string parent = parents.first;
    std::vector<std::string> children = parents.second;
    for(std::string& child : children){
      Eigen::Affine3d transform;
      transform = extrinsics.GetTransform(child, parent);
      tree_.AddTransform(transform, child, parent);
    }
  }

}

void MapBuilder::BuildMap() {
  this->LoadTree(pose_file_path_);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> intermediaries;
  pcl::PointCloud<pcl::PointXYZ>::Ptr aggregate;


}

} // namespace beam_mapping
