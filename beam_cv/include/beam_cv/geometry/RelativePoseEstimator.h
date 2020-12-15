/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>

#include <beam_calibration/CameraModel.h>

template <class T>
using opt = std::optional<T>;

namespace beam_cv {

enum class EstimatorMethod { EIGHTPOINT = 0, SEVENPOINT, FIVEPOINT };

/**
 * @brief static class implementing various pose estimation algorithms
 */
class RelativePoseEstimator {
public:
  /**
   * @brief Computes the essential matrix for 2 cameras given associated pixels
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param xs corresponding pixels in image 1 (min 8)
   * @param xss corresponding pixels in image 2 (min 8)
   */
  static opt<Eigen::Matrix3d> EssentialMatrix8Point(
      const std::shared_ptr<beam_calibration::CameraModel>& camR,
      const std::shared_ptr<beam_calibration::CameraModel>& camC,
      const std::vector<Eigen::Vector2i>& pr_v,
      const std::vector<Eigen::Vector2i>& pc_v);

  /**
   * @brief Performs RANSAC on the given estimator
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param pr_v corresponding pixels in image 1
   * @param pc_v corresponding pixels in image 2
   * @param method essential matrix estimator method
   * @param seed to seed the random number generator, default value of -1 will
   * use time as seed
   * @param max_iterations number of ransac iterations to perform
   * @param inlier_threshold pixel distance to count an inlier as
   * @return transformation matrix in camR reference frame
   */
  static opt<Eigen::Matrix4d> RANSACEstimator(
      const std::shared_ptr<beam_calibration::CameraModel>& camR,
      const std::shared_ptr<beam_calibration::CameraModel>& camC,
      const std::vector<Eigen::Vector2i>& pr_v,
      const std::vector<Eigen::Vector2i>& pc_v,
      EstimatorMethod method = EstimatorMethod::EIGHTPOINT,
      int max_iterations = 100, double inlier_threshold = 5.0, int seed = -1);

  /**
   * @brief Computes the transformation matrix given an essential matrix
   * @param E essential matrix
   * @param R vector to return possible rotations
   * @param t vector to return possible translations
   */
  static void RtFromE(const Eigen::Matrix3d& E, std::vector<Eigen::Matrix3d>& R,
                      std::vector<Eigen::Vector3d>& t);

  /**
   * @brief Returns only physically possible transformation from 4 possibilities
   * by choosing the one where all triangulated points are in front of both
   * cameras
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param R vector of possible rotations (size = 2)
   * @param t vector of possible translations (size = 2)
   * @return Transformation matrix in camR reference frame
   */
  static opt<Eigen::Matrix4d>
      RecoverPose(const std::shared_ptr<beam_calibration::CameraModel>& camR,
                  const std::shared_ptr<beam_calibration::CameraModel>& camC,
                  const std::vector<Eigen::Vector2i>& pr_v,
                  const std::vector<Eigen::Vector2i>& pc_v,
                  const std::vector<Eigen::Matrix3d>& R,
                  const std::vector<Eigen::Vector3d>& t);
};

} // namespace beam_cv
