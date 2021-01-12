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
   * @param cam1 camera model for image 1
   * @param cam2 camera model for image 2
   * @param p1_v corresponding pixels in image 1 (min 8)
   * @param p2_v corresponding pixels in image 2 (min 8)
   */
  static opt<Eigen::Matrix3d> EssentialMatrix8Point(
      const std::shared_ptr<beam_calibration::CameraModel>& cam1,
      const std::shared_ptr<beam_calibration::CameraModel>& cam2,
      const std::vector<Eigen::Vector2i>& p1_v,
      const std::vector<Eigen::Vector2i>& p2_v);

  /**
   * @brief Computes the essential matrix for 2 cameras given associated pixels
   * @param cam1 camera model for image 1
   * @param cam2 camera model for image 2
   * @param p1_v corresponding pixels in image 1 (min 7)
   * @param p2_v corresponding pixels in image 2 (min 7)
   */
  static opt<std::vector<Eigen::Matrix3d>> EssentialMatrix7Point(
      const std::shared_ptr<beam_calibration::CameraModel>& cam1,
      const std::shared_ptr<beam_calibration::CameraModel>& cam2,
      const std::vector<Eigen::Vector2i>& p1_v,
      const std::vector<Eigen::Vector2i>& p2_v);

  /**
   * @brief Performs RANSAC on the given estimator
   * @param cam1 camera model for image 1
   * @param cam2 camera model for image 2
   * @param p1_v corresponding pixels in image 1
   * @param p2_v corresponding pixels in image 2
   * @param method essential matrix estimator method
   * @param seed to seed the random number generator, default value of -1 will
   * use time as seed
   * @param max_iterations number of ransac iterations to perform
   * @param inlier_threshold pixel distance to count an inlier as
   * @return transformation matrix in cam1 reference frame
   */
  static opt<Eigen::Matrix4d> RANSACEstimator(
      const std::shared_ptr<beam_calibration::CameraModel>& cam1,
      const std::shared_ptr<beam_calibration::CameraModel>& cam2,
      const std::vector<Eigen::Vector2i>& p1_v,
      const std::vector<Eigen::Vector2i>& p2_v,
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
   * @param cam1 camera model for image 1
   * @param cam2 camera model for image 2
   * @param R vector of possible rotations (size = 2)
   * @param t vector of possible translations (size = 2)
   * @param pose to return (if it exists)
   * @return Number of inliers (threshold = 5)
   */
  static int
      RecoverPose(const std::shared_ptr<beam_calibration::CameraModel>& cam1,
                  const std::shared_ptr<beam_calibration::CameraModel>& cam2,
                  const std::vector<Eigen::Vector2i>& p1_v,
                  const std::vector<Eigen::Vector2i>& p2_v,
                  const std::vector<Eigen::Matrix3d>& R,
                  const std::vector<Eigen::Vector3d>& t,
                  opt<Eigen::Matrix4d>& pose);
};

} // namespace beam_cv
