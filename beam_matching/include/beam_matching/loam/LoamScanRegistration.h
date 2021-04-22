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

class LoamScanRegistration {
public:
  /**
   * @brief
   */
  LoamScanRegistration(const LoamParamsPtr& params);

  /**
   * @brief
   */
  ~LoamScanRegistration() = default;

private:
  LoamParamsPtr params_;
};

/** @} group matching */
} // namespace beam_matching
