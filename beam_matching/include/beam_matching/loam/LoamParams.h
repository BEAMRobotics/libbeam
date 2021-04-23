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

#include <boost/shared_ptr.hpp>
#include <vector>

namespace beam_matching {
/** @addtogroup matching
 *  @{ */

/**
 * @brief Struct for storing all LOAM params. This will be used by the
 * LoamPointCloud and LoamMatcher classes
 */
class LoamParams {
public:
  /** @brief To separate points into beams (or rings) we need to separate them
   * into bins based on number of beams and FOV. This can be calculated here
   * once instead of each time we get a new scan*/
  inline std::vector<double> GetBeamAngleBinsDeg() {
    if (beam_angle_bins_.size() != 0) { return beam_angle_bins_; }

    double current_angle = fov_deg / 2 - fov_deg / (number_of_beams - 1) / 2;
    while (current_angle >= -fov_deg / 2) {
      beam_angle_bins_.push_back(current_angle);
      current_angle -= fov_deg / (number_of_beams - 1);
    }
    return beam_angle_bins_;
  }

  /** @brief Used to separate cloud into rings of points. Point smoothness is
   * calculated by searching neighbors on the same ring. If the cloud is
   * aggregated over multiple scans, then multiple rings of points will be
   * used.*/
  int number_of_beams{16};

  /** @brief Used allong with number_of_beams to separate cloud into rings of
   * points*/
  double fov_deg{30};

  /** The time per scan. */
  float scan_period{0.1}; // TODO: what is this used for???

  /** The number of (equally sized) regions used to distribute the feature
   * extraction within a scan. */
  int n_feature_regions{6};

  /** The number of surrounding points (+/- region around a point) used to
   * calculate a point curvature. */
  int curvature_region{5};

  /** The maximum number of sharp corner points per feature region. */
  int max_corner_sharp{2};

  /** The maximum number of less sharp corner points per feature region. */
  int max_corner_less_sharp{10 * 2};

  /** The maximum number of flat surface points per feature region. */
  int max_surface_flat{4};

  /** The voxel size used for down sizing the remaining less flat surface
   * points. */
  float less_flat_filter_size{0.2};

  /** The curvature threshold below / above a point is considered a flat /
   * corner point. */
  float surface_curvature_threshold{0.1};

  /** Vertical axis of the lidar. Used to separate the cloud ino rings. */
  std::string vertical_axis{"Z"};

private:
  std::vector<double> beam_angle_bins_;
};

using LoamParamsPtr = std::shared_ptr<LoamParams>;

/** @} group matching */
} // namespace beam_matching
