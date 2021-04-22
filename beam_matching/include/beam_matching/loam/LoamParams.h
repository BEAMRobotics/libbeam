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

    double current_angle = fov_deg / 2;
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
  double fov_deg{40};

private:
  std::vector<double> beam_angle_bins_;
};

using LoamParamsPtr = boost::shared_ptr<LoamParams>;

/** @} group matching */
} // namespace beam_matching
