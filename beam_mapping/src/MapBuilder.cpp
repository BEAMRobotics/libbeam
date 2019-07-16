#include "beam_mapping/MapBuilder.h"

#include "beam_filtering/CropBox.h"
#include "beam_utils/angles.hpp"
#include "beam_utils/log.hpp"
#include "beam_utils/math.hpp"
#include <fstream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
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
  bag_file_name_ = bag_file_path_.substr(bag_file_path_.rfind("/") + 1,
                                         bag_file_path_.rfind(".bag"));
  save_dir_ = J["save_directory"];
  extrinsics_file_ = J["extrinsics_file"];
  intermediary_map_size_ = J["intermediary_map_size"];
  min_translation_ = J["min_translation"];
  min_rotation_deg_ = J["min_rotation_deg"];
  combine_lidar_scans_ = J["combine_lidar_scans"];
  for (const auto& lidar : J["lidars"]) {
    lidar_topics_.push_back(lidar["topic"]);
    lidar_frames_.push_back(lidar["frame"]);
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
  bag_file_name_ = bag_file_path_.substr(bag_file_path_.rfind("/") + 1,
                                         bag_file_path_.rfind(".bag"));
}

void MapBuilder::OverridePoseFile(const std::string& pose_file) {
  pose_file_path_ = pose_file;
}

void MapBuilder::LoadTrajectory(const std::string& pose_file) {
  slam_poses_.LoadPoseFile(pose_file);

  if (slam_poses_.GetBagName() != bag_file_name_) {
    BEAM_CRITICAL("Bag file name from MapBuilder config file is not the same "
                  "as the name listed in the pose file.\nMapBuilderConfig: "
                  "{}\nPoseFile: {}",
                  bag_file_name_.c_str(), slam_poses_.GetBagName().c_str());
    throw std::invalid_argument{
        "Bag file name from MapBuilder config file is "
        "not the same as the name listed in the pose file."};
  }

  extrinsics_.LoadJSON(extrinsics_file_);
  int num_poses = slam_poses_.time_stamps.size();
  for (int k = 0; k < num_poses; k++) {
    trajectory_.AddTransform(slam_poses_.poses[k],
                             slam_poses_.fixed_frame,
                             slam_poses_.moving_frame,
                             slam_poses_.time_stamps[k]);
  }
}

PointCloud::Ptr MapBuilder::CropPointCloud(PointCloud::Ptr cloud,
                                           uint8_t lidar_number) {
  if (!lidar_cropbox_bool_[lidar_number]) { return cloud; }
  PointCloud::Ptr cropped_cloud = boost::make_shared<PointCloud>();
  std::vector<float> min = lidar_cropbox_min_[lidar_number];
  std::vector<float> max = lidar_cropbox_max_[lidar_number];
  Eigen::Vector3f min_vec(min[0], min[1], min[2]);
  Eigen::Vector3f max_vec(max[0], max[1], max[2]);
  beam_filtering::CropBox cropper;
  cropper.SetMinVector(min_vec);
  cropper.SetMaxVector(max_vec);
  cropper.Filter(*cloud, *cropped_cloud);
  // return cropped_cloud;
  return cloud;
}

PointCloud::Ptr MapBuilder::FilterPointCloud(
    PointCloud::Ptr cloud, std::vector<filter_params_type> filter_params) {
  PointCloud::Ptr filtered_cloud = boost::make_shared<PointCloud>();
  // *filtered_cloud = *cloud;
  // return filtered_cloud;
  return cloud;
}

bool MapBuilder::CheckPoseChange() {
  // calculate change in pose and check if rot or trans bigger than threshold

  double l2sqrd = (scan_pose_last_(0, 3) - scan_pose_current_(0, 3)) *
                      (scan_pose_last_(0, 3) - scan_pose_current_(0, 3)) +
                  (scan_pose_last_(1, 3) - scan_pose_current_(1, 3)) *
                      (scan_pose_last_(1, 3) - scan_pose_current_(1, 3)) +
                  (scan_pose_last_(2, 3) - scan_pose_current_(2, 3)) *
                      (scan_pose_last_(2, 3) - scan_pose_current_(2, 3));

  double minRotSq =
      min_rotation_deg_ * DEG_TO_RAD * min_rotation_deg_ * DEG_TO_RAD;
  Eigen::Vector3d eps1, eps2, diffSq;
  eps1 = beam::RToLieAlgebra(scan_pose_last_.rotation());
  eps2 = beam::RToLieAlgebra(scan_pose_current_.rotation());
  diffSq(0, 0) = (eps2(0, 0) - eps1(0, 0)) * (eps2(0, 0) - eps1(0, 0));
  diffSq(1, 0) = (eps2(1, 0) - eps1(1, 0)) * (eps2(1, 0) - eps1(1, 0));
  diffSq(2, 0) = (eps2(2, 0) - eps1(2, 0)) * (eps2(2, 0) - eps1(2, 0));

  // if the norm is greater than the specified minimum sampling distance or
  // if the change in rotation is greater than specified min.
  if (l2sqrd > min_translation_ * min_translation_) {
    return true;
  } else if (diffSq(0, 0) > minRotSq || diffSq(1, 0) > minRotSq ||
             diffSq(2, 0) > minRotSq) {
    return true;
  } else {
    return false;
  }
}

void MapBuilder::ProcessPointCloudMsg(rosbag::View::iterator& iter,
                                      uint8_t lidar_number) {
  bool save_scan = false;
  auto lidar_msg = iter->instantiate<sensor_msgs::PointCloud2>();
  ros::Time scan_time = lidar_msg->header.stamp;

  if ((scan_time < slam_poses_.time_stamps[0]) ||
      (scan_time > slam_poses_.time_stamps.back())) {
    return;
  }
  std::string to_frame = slam_poses_.fixed_frame;
  std::string from_frame = slam_poses_.moving_frame;
  scan_pose_current_ =
      trajectory_.GetTransformEigen(to_frame, from_frame, scan_time);
  save_scan = CheckPoseChange();

  if (save_scan) {
    pcl::PCLPointCloud2::Ptr pcl_pc2_tmp =
        boost::make_shared<pcl::PCLPointCloud2>();
    PointCloud::Ptr cloud_tmp = boost::make_shared<PointCloud>();
    pcl_conversions::toPCL(*lidar_msg, *pcl_pc2_tmp);
    pcl::fromPCLPointCloud2(*pcl_pc2_tmp, *cloud_tmp);
    PointCloud::Ptr cloud_cropped =
        this->CropPointCloud(cloud_tmp, lidar_number);
    PointCloud::Ptr cloud_filtered =
        this->FilterPointCloud(cloud_cropped, input_filters_);
    scans_.push_back(cloud_filtered);
    interpolated_poses_.AddSinglePose(scan_pose_current_);
    interpolated_poses_.AddSingleTimeStamp(scan_time);
    scan_pose_last_ = scan_pose_current_;
  }
}

void MapBuilder::LoadScans(uint8_t lidar_number) {
  // load all params specific to this lidar
  std::string scan_frame = lidar_frames_[lidar_number];
  std::string scan_topic = lidar_topics_[lidar_number];
  std::vector<float> cropbox_min = lidar_cropbox_min_[lidar_number];
  std::vector<float> cropbox_max = lidar_cropbox_max_[lidar_number];

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
      ProcessPointCloudMsg(iter, lidar_number);
    }
  }
}

void MapBuilder::GenerateMap(uint8_t lidar_number) {
  std::string fixed_frame = slam_poses_.fixed_frame;
  std::string moving_frame = slam_poses_.moving_frame;
  std::string lidar_frame = lidar_frames_[lidar_number];
  PointCloud::Ptr scan_aggregate = boost::make_shared<PointCloud>();
  PointCloud::Ptr scan_intermediary = boost::make_shared<PointCloud>();
  Eigen::Affine3d T_MOVING_LIDAR =
      extrinsics_.GetTransformEigen(moving_frame, lidar_frame);
  Eigen::Affine3d T_FIXED_LIDAR;
  // std::cout << "fixed_frame: " << fixed_frame.c_str() << "\n"
  //           << "moving_frame: " << moving_frame.c_str() << "\n"
  //           << "lidar_frame: " << lidar_frame.c_str() << "\n";
  // std::cout << "\nT_" << moving_frame.c_str() << "_" << lidar_frame.c_str() << ": \n"
  //           << T_MOVING_LIDAR.matrix();

  // iterate through all scans
  for (uint32_t k = 0; k < scans_.size(); k++) {
    Eigen::Affine3d T_FIXED_MOVING = interpolated_poses_.poses[k];
    T_FIXED_LIDAR.matrix() = T_FIXED_MOVING.matrix() * T_MOVING_LIDAR.inverse().matrix();
    PointCloud::Ptr scan_transformed = boost::make_shared<PointCloud>();
    pcl::transformPointCloud(*scans_[k], *scan_transformed,
                             T_FIXED_LIDAR);
    *scan_aggregate += *scan_transformed;
    // std::cout << "\nT_" << fixed_frame.c_str() << "_" << moving_frame.c_str() << ": \n"
    //           << T_FIXED_MOVING.matrix();
  }
  maps_.push_back(scan_aggregate);
}

void MapBuilder::SaveMaps() {
  std::string dateandtime =
      beam::convertTimeToDate(std::chrono::system_clock::now());
  boost::filesystem::create_directory(save_dir_ + dateandtime + "/");

  for (uint8_t i = 0; i < maps_.size(); i++) {
    std::string save_path = save_dir_ + dateandtime + "/" + dateandtime + "_" +
                            lidar_frames_[i] + ".pcd";
    BEAM_INFO("Saving map to: {}", save_path);
    pcl::io::savePCDFileBinary(save_path, *maps_[i]);
  }
}

void MapBuilder::BuildMap() {
  this->LoadTrajectory(pose_file_path_);
  for (uint8_t i = 0; i < lidar_topics_.size(); i++) {
    scan_pose_last_.matrix().setIdentity();
    interpolated_poses_.Clear();
    this->scans_.clear();
    this->LoadScans(i);
    this->GenerateMap(i);
  }
  this->SaveMaps();
}

} // namespace beam_mapping
