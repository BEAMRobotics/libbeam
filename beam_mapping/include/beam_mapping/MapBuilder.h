/** @file
 * @ingroup mapping
 */

#include "beam_calibration/TfTree.h"

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
   * @brief for overriding the bag file specified in the config file
   * @param bag_file full path to new bag file
   */
  void OverrideBagFile(const std::string& bag_file);

  /**
   * @brief for overriding the poses file specified in the config file
   * @param poses_file full path to new poses file
   */
  void OverridePosesFile(const std::string& poses_file);

  /**
   * @brief performs the map building
   */
  void BuildMap();

private:
  /**
   * @brief method to load poses from json
   * @param poses_file full path to poses file
   */
  void LoadPosesFromJSON(const std::string& poses_file);

  /**
   * @brief method to load configuration from json
   * @param config_file full path to configuration file
   */
  void LoadConfigFromJSON(const std::string& config_file);

  beam_calibration::TfTree trajectory_;

  std::string poses_file_path_, bag_file_path_, bag_file_name_, save_dir_,
              poses_file_date_, fixed_frame_, pose_frame_, config_file_;
};

/** @} group mapping */

} // namespace beam_mapping
