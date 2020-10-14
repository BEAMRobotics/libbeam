/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <beam_calibration/CameraModel.h>

namespace beam_cv {

/**
 * @brief static class implementing various pose estimation algorithms
 */
class PoseEstimator {
public:
  /**
   * @brief Computes the essential matrix for 2 cameras given associated pixels
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param xs corresponding pixels in image 1 (min 8)
   * @param xss corresponding pixels in image 2 (min 8)
   */
  static Eigen::Matrix3d
      EssentialMatrix8Point(std::shared_ptr<beam_calibration::CameraModel> camR,
                            std::shared_ptr<beam_calibration::CameraModel> camC,
                            Eigen::MatrixXd xs, Eigen::MatrixXd xss);

  /**
   * @brief Computes the transformation matrix given essential matrix
   * @param E essential matrix
   * @param R vector to return possible rotations
   * @param t vector to return possible translations
   */
  static void RtFromE(Eigen::Matrix3d E, std::vector<Eigen::Matrix3d>& R,
                      std::vector<Eigen::Vector3d>& t);

  /**
   * @brief Returns only physcially possible transformation from 4 possibilities
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param R vector of possible rotations
   * @param t vector of possible translations
   */
  static Eigen::Matrix4d
      RecoverPose(std::shared_ptr<beam_calibration::CameraModel> camR,
                  std::shared_ptr<beam_calibration::CameraModel> camC,
                  Eigen::MatrixXd xs, Eigen::MatrixXd xss,
                  std::vector<Eigen::Matrix3d>& R,
                  std::vector<Eigen::Vector3d>& t);
};

} // namespace beam_cv
