#include "beam_mapping/MapBuilder.h"

#include "beam_utils/log.hpp"
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace beam_mapping {

using filter_params_type = std::pair<std::string, std::vector<double>>;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

MapBuilder::MapBuilder(const std::string& config_file) {
  config_file_ = config_file;
  this->LoadConfigFromJSON(config_file);
}

filter_params_type MapBuilder::GetFilterParams(const auto& filter) {
  std::string filter_type = filter["filter_type"];
  std::vector<double> params;
  bool success = true;
  if (filter_type == "DROR") {
    double radius_multiplier = filter["radius_multiplier"];
    double azimuth_angle = filter["azimuth_angle"];
    double min_neighbors = filter["min_neighbors"];
    double min_search_radius = filter["min_search_radius"];
    if (!(radius_multiplier >= 1)) {
      BEAM_CRITICAL(
          "Invalid radius_multiplier parameter in map builder config");
      success = false;
    }
    if (!(azimuth_angle > 0.01)) {
      BEAM_CRITICAL("Invalid azimuth_angle parameter in map builder config");
      success = false;
    }
    if (!(min_neighbors > 1)) {
      BEAM_CRITICAL("Invalid min_neighbors parameter in map builder config");
      success = false;
    }
    if (!(min_search_radius > 0)) {
      BEAM_CRITICAL(
          "Invalid min_search_radius parameter in map builder config");
      success = false;
    }
    params.push_back(radius_multiplier);
    params.push_back(azimuth_angle);
    params.push_back(min_neighbors);
    params.push_back(min_search_radius);
  } else if (filter_type == "ROR") {
    double search_radius = filter["search_radius"];
    double min_neighbors = filter["min_neighbors"];
    if (!(search_radius > 0)) {
      BEAM_CRITICAL("Invalid search_radius parameter in map builder config");
      success = false;
    }
    if (!(min_neighbors > 1)) {
      BEAM_CRITICAL("Invalid min_neighbors parameter in map builder config");
      success = false;
    }
    params.push_back(search_radius);
    params.push_back(min_neighbors);
  } else if (filter_type == "VOXEL") {
    double cell_size = filter["cell_size"];
    if (!(cell_size > 0.01)) {
      BEAM_CRITICAL("Invalid cell_size parameter in map builder config");
      success = false;
    }
    params.push_back(cell_size);
  } else {
    BEAM_CRITICAL("Invalid filter type in map builder config.");
    success = false;
  }

  if (!success) {
    BEAM_CRITICAL("Invalid filter params in map builder config, see bellow for "
                  "options and requirements:");
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
  return std::make_pair(filter_type, params);
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

  for (const auto& lidar : J["lidars"]) {
    lidar_topics_.push_back(lidar["topic"]);
    lidar_frames_.push_back(lidar["frames"]);
    lidar_cropbox_bool_.push_back(lidar["use_cropbox"]);
    lidar_cropbox_min_.push_back(lidar["cropbox_min"]);
    lidar_cropbox_max_.push_back(lidar["cropbox_max"]);
  }
  for (const auto& filter : J["input_filters"]) {
    filter_params_type input_filter;
    input_filter = this->GetFilterParams(filter);
    input_filters_.push_back(input_filter);
  }
  for (const auto& filter : J["intermediary_filters"]) {
    filter_params_type intermediary_filter;
    intermediary_filter = this->GetFilterParams(filter);
    intermediary_filters_.push_back(intermediary_filter);
  }
  for (const auto& filter : J["output_filters"]) {
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
  for (auto& parents : frames) {
    std::string parent = parents.first;
    std::vector<std::string> children = parents.second;
    for (std::string& child : children) {
      Eigen::Affine3d transform;
      transform = extrinsics.GetTransformEigen(child, parent);
      tree_.AddTransform(transform, child, parent);
    }
  }
}

bool MapBuilder::CheckPoseChange() {
  // calculate change in pose and check if rot or trans bigger than threshold
  return true;
}

void MapBuilder::ProcessPointCloudMsg(rosbag::View::iterator& iter) {
  bool save_scan = false;
  auto lidar_msg = iter->instantiate<sensor_msgs::PointCloud2>();
  ros::Time scan_time = lidar_msg->header.stamp;
  std::string from_frame = poses_.fixed_frame;
  std::string to_frame = poses_.moving_frame;
  scan_pose_current_ = tree_.GetTransformEigen(to_frame, from_frame, scan_time);
  save_scan = CheckPoseChange();

  if (save_scan) {
    pcl::PCLPointCloud2::Ptr pcl_pc2_tmp =
        boost::make_shared<pcl::PCLPointCloud2>();
    PointCloud::Ptr cloud_tmp = boost::make_shared<PointCloud>();
    pcl_conversions::toPCL(*lidar_msg, *pcl_pc2_tmp);
    pcl::fromPCLPointCloud2(*pcl_pc2_tmp, *cloud_tmp);
    scans_.push_back(cloud_tmp);
    time_stamps_.push_back(scan_time);
    scan_pose_last_ = scan_pose_current_;
  }
}

void MapBuilder::LoadScans(uint8_t lidar_number) {
  // load all params specific to this lidar
  std::string scan_frame = lidar_frames_[lidar_number];
  std::string scan_topic = lidar_topics_[lidar_number];
  std::vector<double> cropbox_min = lidar_cropbox_min_[lidar_number];
  std::vector<double> cropbox_max = lidar_cropbox_max_[lidar_number];
  bool crop_scan = lidar_cropbox_bool_[lidar_number];

  // load rosbag and create view
  rosbag::Bag bag;
  try {
    bag.open(bag_file_path_, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_CRITICAL("Bag exception : {}}", ex.what());
  }
  rosbag::View view(bag, rosbag::TopicQuery(scan_topic), ros::TIME_MIN,
                    ros::TIME_MAX, true);
  int total_messages = view.size();
  int message_counter = 0;
  std::string output_message = "Loading scans for lidar No. ";
  for (auto iter = view.begin(); iter != view.end(); iter++) {
    message_counter++;
    output_message += std::to_string(lidar_number);
    beam::OutputPercentComplete(message_counter, total_messages,
                                output_message);
    if (iter->getTopic() == scan_topic) {
      PointCloud::Ptr new_scan = boost::make_shared<PointCloud>();
      ProcessPointCloudMsg(iter);
    }
  }
}

void MapBuilder::BuildMap() {
  this->LoadTree(pose_file_path_);
  std::vector<PointCloud::Ptr> intermediaries;
  PointCloud::Ptr aggregate = boost::make_shared<PointCloud>();

  // Get the time along the trajectory that we want to aggregate scans
  for (uint8_t i = 0; i < lidar_topics_.size(); i++) {
    scan_pose_last_.matrix().setIdentity();
    scans_.clear();
    time_stamps_.clear();
    this->LoadScans(i);
  }
}

} // namespace beam_mapping
