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
   * @brief Computes the essential matrix for 2 cameras given associated pixels
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param pixels_r projected pixel locations of points in camR (min size = 3)
   * @param pixels_c projected pixel locations of points in camC (min size = 3)
   * @param points 3d feature locations (min size = 3)
   */
  static opt<Eigen::Matrix4d>
      P3PEstimator(std::shared_ptr<beam_calibration::CameraModel> camR,
                   std::shared_ptr<beam_calibration::CameraModel> camC,
                   std::vector<Eigen::Vector2i> pixels_r,
                   std::vector<Eigen::Vector2i> pixels_c,
                   std::vector<Eigen::Vector3d> points);

  /**
   * @brief Computes the essential matrix for 2 cameras given associated pixels
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param pixels_r projected pixel locations of points in camR
   * @param pixels_c projected pixel locations of points in camC
   * @param points 3d feature locations
   */
  static opt<Eigen::Matrix4d>
      RANSACP3PEstimator(std::shared_ptr<beam_calibration::CameraModel> camR,
                         std::shared_ptr<beam_calibration::CameraModel> camC,
                         std::vector<Eigen::Vector2i> pixels_r,
                         std::vector<Eigen::Vector2i> pixels_c,
                         std::vector<Eigen::Vector3d> points,
                         int max_iterations, double inlier_threshold);
};

} // namespace beam_cv
