/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <beam_calibration/CameraModel.h>

namespace beam_cv {

/**
 * @brief static class implementing the 8 point algorithm with ransac
 */
class PoseEstimator {
public:
  /**
   * @brief Default constructor
   */
  PoseEstimator() = default;

  /**
   * @brief Default destructor
   */
  ~PoseEstimator() = default;
};

} // namespace beam_cv
