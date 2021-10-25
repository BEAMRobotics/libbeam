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

MapBuilder::MapBuilder(const std::string& config_file) {
  config_file_ = config_file;
  poses_moving_frame_ = "";
  poses_fixed_frame_ = "";
  this->LoadConfigFromJSON(config_file);
}

void MapBuilder::LoadConfigFromJSON(const std::string& config_file) {
  BEAM_INFO("Loading MapBuilder config file: {}", config_file);
  nlohmann::json J;
  if (!beam::ReadJson(config_file, J)) {
    throw std::runtime_error{"Invalid json"};
  }

  try {
    pose_file_path_ = J["pose_file"];
    bag_file_path_ = J["bag_file_path"];
    bag_file_name_ = bag_file_path_.substr(bag_file_path_.rfind("/") + 1,
                                           bag_file_path_.rfind(".bag"));
    save_dir_ = J["save_directory"];
    extrinsics_file_ = J["extrinsics_file"];
    poses_moving_frame_ = J["moving_frame"];
    poses_fixed_frame_ = J["fixed_frame"];
    intermediary_map_size_ = J["intermediary_map_size"];
    min_translation_m_ = J["min_translation_m"];
    min_rotation_deg_ = J["min_rotation_deg"];
    combine_lidar_scans_ = J["combine_lidar_scans"];
    for (const auto& lidar : J["lidars"]) {
      MapBuilder::LidarConfig lidar_config;
      lidar_config.topic = lidar["topic"];
      lidar_config.frame = lidar["frame"];
      lidar_config.use_cropbox = lidar["use_cropbox"];
      lidar_config.remove_outside_points = lidar["remove_outside_points"];
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
    output_filters_ =
        beam_filtering::LoadFilterParamsVector(J["output_filters"]);
  } catch (const nlohmann::json::exception& e) {
    BEAM_CRITICAL("Unable to load json, one or more missing or invalid params. "
                  "Reason: {}",
                  e.what());
    throw std::runtime_error{"Invalid json"};
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
    poses_moving_frame_ = slam_poses_.GetMovingFrame();
  }

  if (poses_fixed_frame_.empty()) {
    poses_fixed_frame_ = slam_poses_.GetFixedFrame();
  }

  if (slam_poses_.GetBagName() != bag_file_name_) {
    BEAM_WARN("Bag file name from MapBuilder config file is not the same "
              "as the name listed in the pose file.\nMapBuilderConfig: "
              "{}\nPoseFile: {}",
              bag_file_name_, slam_poses_.GetBagName());
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

  int num_poses = slam_poses_.GetTimeStamps().size();
  for (int k = 0; k < num_poses; k++) {
    trajectory_.AddTransform(Eigen::Affine3d(slam_poses_.GetPoses()[k]),
                             poses_fixed_frame_, poses_moving_frame_,
                             slam_poses_.GetTimeStamps()[k]);
  }
}

PointCloud MapBuilder::CropLidarRaw(const PointCloud& cloud,
                                    uint8_t lidar_number) {
  if (!lidars_[lidar_number].use_cropbox) { return cloud; }
  beam_filtering::CropBox<pcl::PointXYZ> cropper;
  cropper.SetMinVector(lidars_[lidar_number].cropbox_min);
  cropper.SetMaxVector(lidars_[lidar_number].cropbox_max);
  cropper.SetRemoveOutsidePoints(lidars_[lidar_number].remove_outside_points);
  cropper.SetInputCloud(std::make_shared<PointCloud>(cloud));
  cropper.Filter();
  return cropper.GetFilteredCloud();
}

void MapBuilder::ProcessPointCloudMsg(rosbag::View::iterator& iter,
                                      uint8_t lidar_number) {
  bool save_scan = false;
  auto lidar_msg = iter->instantiate<sensor_msgs::PointCloud2>();
  ros::Time scan_time = lidar_msg->header.stamp;

  if ((scan_time < slam_poses_.GetTimeStamps()[0]) ||
      (scan_time > slam_poses_.GetTimeStamps().back())) {
    return;
  }
  std::string to_frame = poses_fixed_frame_;
  std::string from_frame = poses_moving_frame_;
  Eigen::Matrix4d scan_pose_current =
      trajectory_.GetTransformEigen(to_frame, from_frame, scan_time).matrix();
  save_scan = beam::PassedMotionThreshold(scan_pose_last_, scan_pose_current,
                                          min_rotation_deg_, min_translation_m_,
                                          true, false, false);

  if (save_scan) {
    pcl::PCLPointCloud2::Ptr pcl_pc2_tmp =
        std::make_shared<pcl::PCLPointCloud2>();
    PointCloud cloud_tmp;
    beam::pcl_conversions::toPCL(*lidar_msg, *pcl_pc2_tmp);
    pcl::fromPCLPointCloud2(*pcl_pc2_tmp, cloud_tmp);
    PointCloud cloud_cropped = CropLidarRaw(cloud_tmp, lidar_number);
    PointCloud cloud_filtered = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
        cloud_cropped, input_filters_);
    scans_.push_back(std::make_shared<PointCloud>(cloud_filtered));
    interpolated_poses_.AddSinglePose(scan_pose_current);
    interpolated_poses_.AddSingleTimeStamp(scan_time);
    scan_pose_last_ = scan_pose_current;
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
  Eigen::Matrix4d T_MOVING_LIDAR =
      extrinsics_.GetTransformEigen(moving_frame, lidar_frame).matrix();
  Eigen::Matrix4d T_FIXED_LIDAR;
  Eigen::Matrix4d T_FIXED_MOVING;

  // iterate through all scans
  int intermediary_size = 0;
  Eigen::Matrix4d T_FIXED_INT = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_INT_LIDAR = Eigen::Matrix4d::Identity();

  for (uint32_t k = 0; k < scans_.size(); k++) {
    intermediary_size++;

    // get the transforms we will need:
    T_FIXED_MOVING = interpolated_poses_.GetPoses()[k];
    T_FIXED_LIDAR = T_FIXED_MOVING * T_MOVING_LIDAR;
    if (intermediary_size == 1) { T_FIXED_INT = T_FIXED_LIDAR; }
    T_INT_LIDAR = beam::InvertTransform(T_FIXED_INT) * T_FIXED_LIDAR;

    PointCloud::Ptr scan_intermediate_frame = std::make_shared<PointCloud>();
    PointCloud::Ptr intermediary_transformed = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*scans_[k], *scan_intermediate_frame, T_INT_LIDAR);
    *scan_intermediary += *scan_intermediate_frame;

    if (intermediary_size == this->intermediary_map_size_ ||
        k == scans_.size()) {
      *scan_intermediate_frame =
          beam_filtering::FilterPointCloud<pcl::PointXYZ>(
              *scan_intermediary, intermediary_filters_);
      pcl::transformPointCloud(*scan_intermediate_frame,
                               *intermediary_transformed, T_FIXED_INT);
      *scan_aggregate += *intermediary_transformed;
      scan_intermediary->clear();
      intermediary_size = 0;
    }
  }
  PointCloud new_map = beam_filtering::FilterPointCloud<pcl::PointXYZ>(
      *scan_aggregate, output_filters_);
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
    if (!beam::SavePointCloud<pcl::PointXYZ>(
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
    std::string error_message;
    if (!beam::SavePointCloud<pcl::PointXYZ>(
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
  LoadTrajectory(pose_file_path_);
  for (uint8_t i = 0; i < lidars_.size(); i++) {
    scan_pose_last_ = Eigen::Matrix4d::Identity();
    interpolated_poses_.Clear();
    scans_.clear();
    LoadScans(i);
    GenerateMap(i);
  }
  SaveMaps();
}

} // namespace beam_mapping
