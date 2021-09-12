#include <beam_mapping/MapBuilder.h>

#include <fstream>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <beam_utils/angles.h>
#include <beam_utils/log.h>
#include <beam_utils/math.h>
#include <beam_utils/pcl_conversions.h>

namespace beam_mapping {

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

MapBuilder::MapBuilder(const std::string& config_file) {
  config_file_ = config_file;
  poses_moving_frame_ = "";
  poses_fixed_frame_ = "";
  this->LoadConfigFromJSON(config_file);
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
  poses_moving_frame_ = J["moving_frame"];
  poses_fixed_frame_ = J["fixed_frame"];
  intermediary_map_size_ = J["intermediary_map_size"];
  min_translation_ = J["min_translation"];
  min_rotation_deg_ = J["min_rotation_deg"];
  combine_lidar_scans_ = J["combine_lidar_scans"];
  for (const auto& lidar : J["lidars"]) {
    MapBuilder::LidarConfig lidar_config;
    lidar_config.topic = lidar["topic"];
    lidar_config.frame = lidar["frame"];
    lidar_config.use_cropbox = lidar["use_cropbox"];
    std::vector<float> min_ = lidar["cropbox_min"];
    std::vector<float> max_ = lidar["cropbox_max"];
    Eigen::Vector3f min(min_[0], min_[1], min_[2]);
    Eigen::Vector3f max(max_[0], max_[1], max_[2]);
    lidar_config.cropbox_min = min;
    lidar_config.cropbox_max = max;
    lidars_.push_back(lidar_config);
  }
  input_filters_ = beam_filtering::LoadFilterParamsVector(J["input_filters"]);
  intermediary_filters_ =
      beam_filtering::LoadFilterParamsVector(J["intermediary_filters"]);
  output_filters_ = beam_filtering::LoadFilterParamsVector(J["output_filters"]);
}

void MapBuilder::OverrideBagFile(const std::string& bag_file) {
  bag_file_path_ = bag_file;
  bag_file_name_ = bag_file_path_.substr(bag_file_path_.rfind("/") + 1,
                                         bag_file_path_.rfind(".bag"));
}

void MapBuilder::OverridePoseFile(const std::string& pose_file) {
  pose_file_path_ = pose_file;
}

void MapBuilder::OverrideExtrinsicsFile(const std::string& extrinsics_file) {
  extrinsics_file_ = extrinsics_file;
}

void MapBuilder::OverrideOutputDir(const std::string& output_dir) {
  save_dir_ = output_dir;
}

void MapBuilder::LoadTrajectory(const std::string& pose_file) {
  std::string pose_type = pose_file_path_.substr(pose_file_path_.rfind("."),
                                                 pose_file_path_.size());
  if (pose_type == ".json") {
    slam_poses_.LoadFromJSON(pose_file);
  } else if (pose_type == ".ply") {
    slam_poses_.LoadFromPLY(pose_file);
  } else if (pose_type == ".pcd") {
    slam_poses_.LoadFromPCD(pose_file);
  } else {
    BEAM_CRITICAL(
        "Invalid pose file type. Valid extensions: .ply, .json, .pcd");
    throw std::invalid_argument{
        "Invalid pose file type. Valid extensions: .ply, .json, .pcd"};
  }

  if (poses_moving_frame_.empty()) {
    poses_moving_frame_ = slam_poses_.moving_frame;
  }

  if (poses_fixed_frame_.empty()) {
    poses_fixed_frame_ = slam_poses_.fixed_frame;
  }

  if (slam_poses_.GetBagName() != bag_file_name_) {
    BEAM_WARN("Bag file name from MapBuilder config file is not the same "
              "as the name listed in the pose file.\nMapBuilderConfig: "
              "{}\nPoseFile: {}",
              bag_file_name_.c_str(), slam_poses_.GetBagName().c_str());
    // throw std::invalid_argument{
    //     "Bag file name from MapBuilder config file is "
    //     "not the same as the name listed in the pose file."};
  }

  extrinsics_.LoadJSON(extrinsics_file_);

  if (poses_moving_frame_ == "") {
    BEAM_CRITICAL("Set moving frame in map builder before building map.");
    throw std::runtime_error{
        "Set moving frame in map builder before building map."};
  } else if (poses_fixed_frame_ == "") {
    BEAM_CRITICAL("Set fixed frame in map builder before building map");
    throw std::runtime_error{
        "Set fixed frame in map builder before building map"};
  }

  int num_poses = slam_poses_.time_stamps.size();
  for (int k = 0; k < num_poses; k++) {
    trajectory_.AddTransform(slam_poses_.poses[k], poses_fixed_frame_,
                             poses_moving_frame_, slam_poses_.time_stamps[k]);
  }
}

PointCloud::Ptr MapBuilder::CropPointCloud(PointCloud::Ptr cloud,
                                           uint8_t lidar_number) {
  if (!lidars_[lidar_number].use_cropbox) { return cloud; }
  beam_filtering::CropBox<PointT> cropper;
  cropper.SetMinVector(lidars_[lidar_number].cropbox_min);
  cropper.SetMaxVector(lidars_[lidar_number].cropbox_max);
  cropper.SetInputCloud(cloud);
  cropper.Filter();
  return std::make_shared<PointCloud>(cropper.GetFilteredCloud());
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
      beam::Deg2Rad(min_rotation_deg_) * beam::Deg2Rad(min_rotation_deg_);
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
  std::string to_frame = poses_fixed_frame_;
  std::string from_frame = poses_moving_frame_;
  scan_pose_current_ =
      trajectory_.GetTransformEigen(to_frame, from_frame, scan_time);
  save_scan = CheckPoseChange();

  if (save_scan) {
    pcl::PCLPointCloud2::Ptr pcl_pc2_tmp =
        std::make_shared<pcl::PCLPointCloud2>();
    PointCloud::Ptr cloud_tmp = std::make_shared<PointCloud>();
    beam::pcl_conversions::toPCL(*lidar_msg, *pcl_pc2_tmp);
    pcl::fromPCLPointCloud2(*pcl_pc2_tmp, *cloud_tmp);
    PointCloud::Ptr cloud_cropped =
        this->CropPointCloud(cloud_tmp, lidar_number);
    PointCloud cloud_filtered =
        beam_filtering::FilterPointCloud(*cloud_cropped, input_filters_);
    scans_.push_back(std::make_shared<PointCloud>(cloud_filtered));
    interpolated_poses_.AddSinglePose(scan_pose_current_);
    interpolated_poses_.AddSingleTimeStamp(scan_time);
    scan_pose_last_ = scan_pose_current_;
  }
}

void MapBuilder::LoadScans(uint8_t lidar_number) {
  // load all params specific to this lidar
  std::string scan_frame = lidars_[lidar_number].frame;
  std::string scan_topic = lidars_[lidar_number].topic;

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
    output_message += std::to_string(lidar_number + 1);
    beam::OutputPercentComplete(message_counter, total_messages,
                                output_message);
    ProcessPointCloudMsg(iter, lidar_number);
  }
}

void MapBuilder::GenerateMap(uint8_t lidar_number) {
  std::string fixed_frame = poses_fixed_frame_;
  std::string moving_frame = poses_moving_frame_;
  std::string lidar_frame = lidars_[lidar_number].frame;
  PointCloud::Ptr scan_aggregate = std::make_shared<PointCloud>();
  PointCloud::Ptr scan_intermediary = std::make_shared<PointCloud>();
  Eigen::Affine3d T_MOVING_LIDAR =
      extrinsics_.GetTransformEigen(moving_frame, lidar_frame);
  Eigen::Affine3d T_FIXED_LIDAR, T_FIXED_MOVING;

  // iterate through all scans
  int intermediary_size = 0;
  Eigen::Affine3d T_FIXED_INT, T_INT_LIDAR;
  T_FIXED_INT.matrix().setIdentity();
  T_INT_LIDAR.matrix().setIdentity();

  for (uint32_t k = 0; k < scans_.size(); k++) {
    intermediary_size++;

    // get the transforms we will need:
    T_FIXED_MOVING = interpolated_poses_.poses[k];
    T_FIXED_LIDAR = T_FIXED_MOVING * T_MOVING_LIDAR;
    if (intermediary_size == 1) { T_FIXED_INT = T_FIXED_LIDAR; }
    T_INT_LIDAR = T_FIXED_INT.inverse() * T_FIXED_LIDAR;

    PointCloud::Ptr scan_intermediate_frame = std::make_shared<PointCloud>();
    PointCloud::Ptr intermediary_transformed = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*scans_[k], *scan_intermediate_frame, T_INT_LIDAR);
    *scan_intermediary += *scan_intermediate_frame;

    if (intermediary_size == this->intermediary_map_size_ ||
        k == scans_.size()) {
      *scan_intermediate_frame = beam_filtering::FilterPointCloud(
          *scan_intermediary, intermediary_filters_);
      pcl::transformPointCloud(*scan_intermediate_frame,
                               *intermediary_transformed, T_FIXED_INT);
      *scan_aggregate += *intermediary_transformed;
      scan_intermediary->clear();
      intermediary_size = 0;
    }
  }
  PointCloud new_map =
      beam_filtering::FilterPointCloud(*scan_aggregate, output_filters_);
  maps_.push_back(std::make_shared<PointCloud>(new_map));
}

void MapBuilder::SaveMaps() {
  std::string dateandtime =
      beam::ConvertTimeToDate(std::chrono::system_clock::now());
  boost::filesystem::create_directory(save_dir_ + dateandtime + "/");

  for (uint8_t i = 0; i < maps_.size(); i++) {
    std::string save_path = save_dir_ + dateandtime + "/" + dateandtime + "_" +
                            lidars_[i].frame + ".pcd";
    BEAM_INFO("Saving map to: {}", save_path);
    std::string error_message{};
    if (!beam::SavePointCloud<pcl::PointXYZI>(
            save_path, *maps_[i], beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
  if (this->combine_lidar_scans_) {
    PointCloud::Ptr combined_map = std::make_shared<PointCloud>();
    for (uint8_t i = 0; i < maps_.size(); i++) { *combined_map += *maps_[i]; }
    std::string save_path =
        save_dir_ + dateandtime + "/" + dateandtime + "_combined.pcd";
    BEAM_INFO("Saving map to: {}", save_path);
    std::string error_message{};
    if (!beam::SavePointCloud<pcl::PointXYZI>(
            save_path, *combined_map, beam::PointCloudFileType::PCDBINARY,
            error_message)) {
      BEAM_ERROR("Unable to save cloud. Reason: {}", error_message);
    }
  }
}

void MapBuilder::SetPosesMovingFrame(std::string& moving_frame) {
  poses_moving_frame_ = moving_frame;
}

std::string MapBuilder::GetPosesMovingFrame() {
  return poses_moving_frame_;
}

void MapBuilder::SetPosesFixedFrame(std::string& fixed_frame) {
  poses_fixed_frame_ = fixed_frame;
}

std::string MapBuilder::GetPosesFixedFrame() {
  return poses_fixed_frame_;
}

void MapBuilder::BuildMap() {
  this->LoadTrajectory(pose_file_path_);
  for (uint8_t i = 0; i < lidars_.size(); i++) {
    scan_pose_last_.matrix().setIdentity();
    interpolated_poses_.Clear();
    this->scans_.clear();
    this->LoadScans(i);
    this->GenerateMap(i);
  }
  this->SaveMaps();
}

} // namespace beam_mapping
