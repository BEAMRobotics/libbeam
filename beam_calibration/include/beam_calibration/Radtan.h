/** @file
 * @ingroup calibration
 */

#pragma once

#include "beam_calibration/CameraModel.h"

namespace beam_calibration {

/**
 * @brief Derived class for pinhole camera model
 */
class Radtan : public CameraModel {
public:
  /**
   * @brief Constructor
   * @param input_file path to input file
   */
  Radtan(const std::string& file_path);

  /**
   * @brief Default destructor
   */
  ~Radtan() override = default;
  /**
   * @brief Method for projecting a point into an image plane
   * @return projected point
   * @param point 3d point to be projected [x,y,z]^T
   */
  opt<Eigen::Vector2i> ProjectPoint(const Eigen::Vector3d& point) override;

  /**
   * @brief Overload projection function for computing jacobian of projection
   * @return projected point
   * @param point 3d point to be projected [x,y,z]^T
   * @param J 2 x 3 projection jacobian.
   * For ProjectPoint: [u,v]^T = [P1(x, y, z), P2(x, y, z)]^T
   *                   J = | dP1/dx , dP1/dy, dP1/dz |
   *                       | dP2/dx , dP2/dy, dP2/dz |
   */
  opt<Eigen::Vector2i> ProjectPoint(const Eigen::Vector3d& point,
                                    Eigen::MatrixXd& J) override;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param point [u,v]
   */
  opt<Eigen::Vector3d> BackProject(const Eigen::Vector2i& pixel) override;

  /**
   * @brief Method for undistorting an image based on camera's distortion
   * @param image_input image to be undistorted
   * @param image_output reference to Mat obejct to store output
   */
  void UndistortImage(const cv::Mat& image_input, cv::Mat& image_output);

protected:
  /**
   * @brief Method to distort point
   * @return Vec2 distorted point
   * @param pixel to undistort
   */
  Eigen::Vector2d DistortPixel(const Eigen::Vector2d& pixel);

  /**
   * @brief Method to undistort point
   * @return Vec2 undistorted point
   * @param pixel to undistort
   */
  Eigen::Vector2d UndistortPixel(const Eigen::Vector2d& pixel);

  /**
   * @brief Method to compute jacobian of the distortion function
   * @return Jacobian
   * @param pixel to compute jacobian around
   */
  Eigen::Matrix2d ComputeDistortionJacobian(const Eigen::Vector2d& pixel);

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double k1_;
  double k2_;
  double p1_;
  double p2_;
};

/** @} group calibration */

} // namespace beam_calibration
