/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>

#include <beam_calibration/CameraModel.h>

template <class T>
using opt = std::optional<T>;

namespace beam_cv {
/**
 * @brief static class implementing various pose estimation algorithms
 */
class AbsolutePoseEstimator {
public:
  /**
   * @brief Computes the absolute pose of camera
   * @param cam camera model for image 
   * @param pixels projected pixel locations of points in cam (min size = 3)
   * @param points 3d feature locations (min size = 3)
   * @returns 4 possible transformation matrices
   */
  static std::vector<Eigen::Matrix4d>
      P3PEstimator(
        // std::shared_ptr<beam_calibration::CameraModel> cam,
                   std::vector<Eigen::Vector2i> pixels,
                   std::vector<Eigen::Vector3d> points);
};

} // namespace beam_cv
