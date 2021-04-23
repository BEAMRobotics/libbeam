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

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/** @brief A pair describing the start end end index of a range. */
using IndexRange = std::pair<size_t, size_t>;

/** @brief Point label options. */
enum PointLabel {
  CORNER_SHARP = 2,      // sharp corner point
  CORNER_LESS_SHARP = 1, // less sharp corner point
  SURFACE_LESS_FLAT = 0, // less flat surface point
  SURFACE_FLAT = -1      // flat surface point
};

class LoamFeatureExtractor {
public:
  /**
   * @brief
   */
  LoamFeatureExtractor(const LoamParamsPtr& params);

  /**
   * @brief
   */
  ~LoamFeatureExtractor() = default;

  LoamPointCloud ExtractFeatures(const PointCloud& cloud);

private:
  void Reset();

  std::vector<PointCloud> GetScanLines(const PointCloud& cloud);

  void GetSortedScan(const std::vector<PointCloud>& scan_lines);

  void SetScanBuffersFor(size_t start_idx, size_t end_idx);

  void SetRegionBuffersFor(size_t start_idx, size_t end_idx);

  void MarkAsPicked(size_t cloud_idx, size_t scan_idx);

  /** @brief all parameters needed for feature extraction are stored here */
  LoamParamsPtr params_;

  /** @brief scan sorted based on rings (one ring for each lidar beam) where
   * scan_indices_ stores the start and end of each ring */
  PointCloud sorted_scan_;

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

  /** @brief sharp corner points cloud */
  PointCloud corner_points_sharp_;

  /** @brief less sharp corner points cloud */
  PointCloud corner_points_less_sharp_;

  /** @brief flat surface points cloud */
  PointCloud surface_points_flat_;

  /** @brief less flat surface points cloud */
  PointCloud surface_points_less_flat_;

  // DEBUG TOOLS
  bool output_scan_lines_{false};
  std::string debug_output_path_{"/home/nick/tmp/loam_tests/"};
};

/** @} group matching */
} // namespace beam_matching
