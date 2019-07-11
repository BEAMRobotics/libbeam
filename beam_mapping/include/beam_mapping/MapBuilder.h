/** @file
 * @ingroup mapping
 */

#include "beam_calibration/TfTree.h"
#include "beam_mapping/Poses.h"

// PCL specific headers
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#pragma once

// beam
#include "beam_utils/math.hpp"

namespace beam_mapping {
/** @addtogroup mapping
 *  @{ */

using filter_params_type = std::pair<std::string, std::vector<double>>;

/**
 * @brief class for map builder
 */
class MapBuilder {
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
   * @brief Method for getting the filter params
   * @param filter
   * @return filter_parameters
   */
  filter_params_type GetFilterParams(const auto& filter);

  /**
   * @brief for overriding the bag file specified in the config file
   * @param bag_file full path to new bag file
   */
  void OverrideBagFile(const std::string& bag_file);

  /**
   * @brief for overriding the poses file specified in the config file
   * @param poses_file full path to new poses file
   */
  void OverridePoseFile(const std::string& poses_file);

  /**
   * @brief performs the map building
   */
  void BuildMap();

private:
  /**
   * @brief method to load poses from json and extrinsics
   * @param poses_file full path to poses file
   */
  void LoadTree(const std::string& poses_file);

  /**
   * @brief method to load configuration from json
   * @param config_file full path to configuration file
   */
  void LoadConfigFromJSON(const std::string& config_file);

  beam_calibration::TfTree tree_;
  beam_mapping::Poses poses_;
  std::string pose_file_path_, bag_file_path_, bag_file_name_, save_dir_,
  config_file_, extrinsics_file_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans_;
  std::vector<ros::Time> time_stamps_;
  int intermediary_map_size_;
  double min_translation_, min_rotation_deg_;
  bool combine_lidar_scans_;
  std::vector<std::string> lidar_topics_, lidar_frames_;
  std::vector<std::vector<double>> lidar_cropbox_min_, lidar_cropbox_max_;
  std::vector<bool> lidar_cropbox_bool_;
  std::vector<filter_params_type> input_filters_, intermediary_filters_,
  output_filters_;
};

/** @} group mapping */

} // namespace beam_mapping
