/** @file
 * @ingroup matching
 *
 * The is an implementation of Lidar Odometry and Mapping (LOAM). See the
 * following papers:
 *
 *    Zhang, J., & Singh, S. (2014). LOAM : Lidar Odometry and Mapping in
 * Real-time. Robotics: Science and Systems.
 * https://doi.org/10.1007/s10514-016-9548-2
 *
 *    Zhang, J., & Singh, S. (2018). Laser–visual–inertial odometry and mapping
 * with high robustness and low drift. Journal of Field Robotics, 35(8),
 * 1242–1264. https://doi.org/10.1002/rob.21809
 *
 * This code was derived from the following repos:
 *
 *    https://github.com/laboshinl/loam_velodyne
 *
 *    https://github.com/libing64/lidar_pose_estimator
 *
 *    https://github.com/TixiaoShan/LIO-SAM
 *
 */

#pragma once

#include <beam_matching/loam/LoamParams.h>
#include <beam_matching/loam/LoamPointCloud.h>
#include <beam_utils/pointclouds.h>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/** @brief A pair describing the start end end index of a range. */
using IndexRange = std::pair<size_t, size_t>;

/**
 * @brief class for extracting loam features from a regular pcl pointcloud
 */
class LoamFeatureExtractor {
public:
  /**
   * @brief constructor that requires a pointer to a loam params object
   */
  LoamFeatureExtractor(const LoamParamsPtr& params);

  /**
   * @brief default destructor
   */
  ~LoamFeatureExtractor() = default;

  /**
   * @brief main function that extracts loam features from a pointcloud and
   * returns a loam pointcloud. This extraction method extracts strong and weak
   * features for surface (or planar) points as well as edge (or sharp)
   * features. Weak sharp points are just less sharp, and weak surface features
   * are just less flat. Weak features can be useful if no correspondence is
   * found for a sharp feature when performing registration.
   * @param pointcloud regular pcl pointcloud in lidar frame. Note this cannot
   * be transformed to world frame since it requires the scan to be separated
   * into scan lines which cannot be done after the point cloud has been
   * transformed to any other coordinate frame.
   * @return loam pointcloud in lidar frame
   */
  LoamPointCloud ExtractFeatures(const PointCloud& cloud);

  /**
   * @brief overload of the function above, where scan lines are separated based
   * on ring label, instead of calculating estimated ring based on beam angle
   * and FOV of sensor.
   * @param pointcloud pcl pointcloud of type PointXYZIRT (defined above)
   * @return loam pointcloud in lidar frame
   */
  LoamPointCloud ExtractFeatures(const pcl::PointCloud<PointXYZIRT>& cloud);

  /**
   * @brief overload of the function above, where scan lines are separated based
   * on ring label, instead of calculating estimated ring based on beam angle
   * and FOV of sensor.
   * @param pointcloud pcl pointcloud of type PointXYZITRRNR
   * @return loam pointcloud in lidar frame
   */
  LoamPointCloud ExtractFeatures(const pcl::PointCloud<PointXYZITRRNR>& cloud);

  /**
   * @brief If this is called, the following will get saved: one point cloud for
   * each extracted scan line and the original scan
   */
  void SaveScanLines(const std::string& debug_output_path);

private:
  LoamPointCloud ExtractFeaturesFromScanLines(
      const std::vector<PointCloudIRT>& scan_lines);

  void Reset();

  std::vector<PointCloudIRT> GetScanLines(const PointCloud& cloud);

  void GetSortedScan(const std::vector<PointCloudIRT>& scan_lines);

  void SetScanBuffersFor(size_t start_idx, size_t end_idx);

  void SetRegionBuffersFor(size_t start_idx, size_t end_idx);

  void MarkAsPicked(size_t cloud_idx, size_t scan_idx);

  /** @brief all parameters needed for feature extraction are stored here */
  LoamParamsPtr params_;

  /** @brief scan sorted based on rings (one ring for each lidar beam) where
   * scan_indices_ stores the start and end of each ring */
  PointCloudIRT sorted_scan_;

  /** @brief start and end indices of the individual scans withing
   * sorted_scan_. Type: vector<pair<size_t, size_t>> */
  std::vector<IndexRange> scan_indices_;

  /** @brief point curvature buffer */
  std::vector<float> region_curvature_;

  /** @brief point label buffer */
  std::vector<PointLabel> region_label_;

  /** @brief sorted region indices based on point curvature */
  std::vector<size_t> region_sort_indices_;

  /** @brief flag if neighboring point was already picked */
  std::vector<int> scan_neighbor_picked_;

  // DEBUG TOOLS
  std::string debug_output_path_;
};

/** @} group matching */
} // namespace beam_matching
