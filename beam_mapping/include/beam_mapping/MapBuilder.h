/** @file
 * @ingroup mapping
 */

#pragma once

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <beam_calibration/TfTree.h>
#include <beam_mapping/Poses.h>
#include <beam_utils/math.h>
#include <beam_filtering/Utils.h>

namespace beam_mapping {
/** @addtogroup mapping
 *  @{ */

/**
 * @brief class for map builder
 */
class MapBuilder {
  struct LidarConfig {
    std::string topic;
    std::string frame;
    bool use_cropbox;
    bool remove_outside_points;
    Eigen::Vector3f cropbox_min;
    Eigen::Vector3f cropbox_max;
  };

public:
  /**
   * @brief constructor which sets some defaults
   * @param config_file full path to configuration file
   */
  MapBuilder(const std::string& config_file);

  /**
   * @brief Default destructor
   */
  ~MapBuilder() = default;

  /**
   * @brief for overriding the bag file specified in the config file. This is
   used when you want to use a different bag file from what is specified in the
   config file. E.g., with testing, you want to use a local reference to the bag
   so we cannot specify the full path in the config file.
   * @param bag_file full path to new bag file
   */
  void OverrideBagFile(const std::string& bag_file);

  /**
   * @brief for overriding the poses file specified in the config file. See
   OverrideBagFile for explanation as to why this might be needed.
   * @param poses_file full path to new poses file
   */
  void OverridePoseFile(const std::string& poses_file);

  /**
   * @brief for overriding the extrinsics file specified in the config file. See
   OverrideBagFile for explanation as to why this might be needed.
   * @param extrinsics_file full path to new extrinsics file
   */
  void OverrideExtrinsicsFile(const std::string& extrinsics_file);

  /**
   * @brief for overriding the output directory. See
   OverrideBagFile for explanation as to why this might be needed.
   * @param output_dir output directory
   */
  void OverrideOutputDir(const std::string& output_dir);

  /**
   * @brief performs the map building
   */
  void BuildMap();

  /**
   * @brief sets moving frame for poses
   * @param moving_frame
   */
  void SetPosesMovingFrame(std::string& moving_frame);

  /**
   * @brief returns moving frame for poses
   * @return poses_moving_frame_
   */
  std::string GetPosesMovingFrame();

  /**
   * @brief sets fixed frame for poses
   * @param fixed_frame
   */
  void SetPosesFixedFrame(std::string& fixed_frame);

  /**
   * @brief returns fixed frame for poses
   * @return poses_fixed_frame_
   */
  std::string GetPosesFixedFrame();

  /**
   * @brief gets the path to the bag file
   * return bag_file_path_
   */
  std::string GetBagFile() { return bag_file_path_; }

  /**
   * @brief gets the path to the save directory
   * return save_dir_
   */
  std::string GetSaveDir() { return save_dir_; }

private:
  /**
   * @brief method to load poses from json and extrinsics
   * @param poses_file full path to poses file
   */
  void LoadTrajectory(const std::string& poses_file);

  /**
   * @brief method for cropping the input point cloud
   * @param cloud point cloud to crop
   * @param lidar_number used for getting crop box parameters
   * @return cropped_cloud
   */
  PointCloud CropLidarRaw(const PointCloud& cloud, uint8_t lidar_number);

  /**
   * @brief method to load configuration from json
   * @param config_file full path to configuration file
   */
  void LoadConfigFromJSON(const std::string& config_file);

  /**
   * @brief processes a point cloud message by first checking if the pose has
   * changed more than the threshold, if so convert it and add to the scans and
   * timestamp vectors
   * @param iter rosbag iterator
   * @param lidar_number
   */
  void ProcessPointCloudMsg(rosbag::View::iterator& iter, uint8_t lidar_number);

  /**
   * @brief loads all the scans from a specific lidar and builds the scans and
   * timestamps vector.
   * @param lidar_number
   */
  void LoadScans(uint8_t lidar_number);

  /**
   * @brief creates an aggregate map for one lidar scan topic
   * @param lidar_number
   */
  void GenerateMap(uint8_t lidar_number);

  /**
   * @brief outputs maps to save directory
   */
  void SaveMaps();

  // From Config file
  std::string pose_file_path_;
  std::string bag_file_path_;
  std::string bag_file_name_;
  std::string save_dir_;
  std::string config_file_;
  std::string extrinsics_file_;
  int intermediary_map_size_;
  double min_translation_m_;
  double min_rotation_deg_;
  bool combine_lidar_scans_;
  std::vector<LidarConfig> lidars_;
  std::vector<beam_filtering::FilterParamsType> input_filters_;
  std::vector<beam_filtering::FilterParamsType> intermediary_filters_;
  std::vector<beam_filtering::FilterParamsType> output_filters_;

  // New objects
  std::string poses_moving_frame_;
  std::string poses_fixed_frame_;
  beam_mapping::Poses slam_poses_;
  beam_mapping::Poses interpolated_poses_;
  beam_calibration::TfTree trajectory_;
  beam_calibration::TfTree extrinsics_;
  PointCloud::Ptr aggregate_;
  std::vector<PointCloud::Ptr> scans_;
  std::vector<PointCloud::Ptr> maps_;
  Eigen::Matrix4d scan_pose_last_;
};

/** @} group mapping */

} // namespace beam_mapping
