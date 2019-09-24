/** @file
 * @ingroup mapping
 */

#pragma once

#include "beam_calibration/TfTree.h"
#include "beam_mapping/Poses.h"
#include "beam_utils/math.hpp"

// PCL specific headers
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS headers
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace beam_mapping {
/** @addtogroup mapping
 *  @{ */

using FilterParamsType = std::pair<std::string, std::vector<double>>;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

/**
 * @brief class for map builder
 */
class MapBuilder {
  struct LidarConfig {
    std::string topic;
    std::string frame;
    bool use_cropbox;
    Eigen::Vector3f cropbox_min;
    Eigen::Vector3f cropbox_max;
  };

  public :
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
   * @brief Method for getting the filter params
   * @param filter
   * @return filter_parameters
   */
  FilterParamsType GetFilterParams(const auto& filter);

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
  PointCloud::Ptr CropPointCloud(PointCloud::Ptr cloud, uint8_t lidar_number);

  /**
   * @brief method for filtering a point cloud based on a list of filters with
   * their associated parameters
   * @param cloud point cloud to filter
   * @param filter_params
   * @return filtered_cloud
   */
  PointCloud::Ptr FilterPointCloud(PointCloud::Ptr cloud,
                                   std::vector<FilterParamsType> filter_params);

  /**
   * @brief method to load configuration from json
   * @param config_file full path to configuration file
   */
  void LoadConfigFromJSON(const std::string& config_file);

  /**
   * @brief checks the rotation and translation change between current pose and
   * last pose and outputs bool of whether it was greater than specified the
   * specified thresholds
   * @return save_scan whether or not to save the current scan
   */
  bool CheckPoseChange();

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
  std::string pose_file_path_, bag_file_path_, bag_file_name_, save_dir_,
      config_file_, extrinsics_file_;
  int intermediary_map_size_;
  double min_translation_, min_rotation_deg_;
  bool combine_lidar_scans_;
  std::vector<LidarConfig> lidars_;
  std::vector<FilterParamsType> input_filters_, intermediary_filters_,
      output_filters_;

  // New objects
  std::string poses_moving_frame_, poses_fixed_frame_;
  beam_mapping::Poses slam_poses_, interpolated_poses_;
  beam_calibration::TfTree trajectory_, extrinsics_;
  PointCloud::Ptr aggregate_;
  std::vector<PointCloud::Ptr> scans_, maps_;
  Eigen::Affine3d scan_pose_last_, scan_pose_current_;
};

/** @} group mapping */

} // namespace beam_mapping
