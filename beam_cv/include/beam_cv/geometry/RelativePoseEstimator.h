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
  static opt<Eigen::Matrix3d>
      EssentialMatrix8Point(std::shared_ptr<beam_calibration::CameraModel> camR,
                            std::shared_ptr<beam_calibration::CameraModel> camC,
                            std::vector<Eigen::Vector2i> pr_v,
                            std::vector<Eigen::Vector2i> pc_v);

  /**
   * @brief Performs RANSAC on the given estimator
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param xs corresponding pixels in image 1
   * @param xss corresponding pixels in image 2
   * @param method essential matrix estimator method
   * @param seed to seed the random number generator, default value of -1 will
   * use time as seed
   */
  static opt<Eigen::Matrix4d>
      RANSACEstimator(std::shared_ptr<beam_calibration::CameraModel> camR,
                      std::shared_ptr<beam_calibration::CameraModel> camC,
                      std::vector<Eigen::Vector2i> pr_v,
                      std::vector<Eigen::Vector2i> pc_v, EstimatorMethod method,
                      int max_iterations = 100, double inlier_threshold = 10.0,
                      int seed = -1);

  /**
   * @brief Computes the transformation matrix given essential matrix
   * @param E essential matrix
   * @param R vector to return possible rotations
   * @param t vector to return possible translations
   */
  static void RtFromE(Eigen::Matrix3d E, std::vector<Eigen::Matrix3d>& R,
                      std::vector<Eigen::Vector3d>& t);

  /**
   * @brief Returns only physically possible transformation from 4 possibilities
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param R vector of possible rotations
   * @param t vector of possible translations
   */
  static Eigen::Matrix4d RecoverPose(
      std::shared_ptr<beam_calibration::CameraModel> camR,
      std::shared_ptr<beam_calibration::CameraModel> camC,
      std::vector<Eigen::Vector2i> pr_v, std::vector<Eigen::Vector2i> pc_v,
      std::vector<Eigen::Matrix3d>& R, std::vector<Eigen::Vector3d>& t);
};

} // namespace beam_cv
