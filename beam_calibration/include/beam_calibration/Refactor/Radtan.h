/** @file
 * @ingroup calibration
 */

#pragma once

#include "beam_calibration/include/Refactor/CameraModel.h"

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
  Radtan();

  /**
   * @brief Default destructor
   */
  ~Radtan() = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @param point 3d point to be projected [x,y,z]^T
   * @param pixel reference to an optional vector with image coordinates after
   * point has been projected into the image plane [u,v]^T
   */
  void ProjectPoint(const Eigen::Vector3d& point,
                    std::optional<Eigen::Vector2i>& pixel) override;

  /**
   * @brief Overload projection function for computing jacobian of projection
   * @param point 3d point to be projected [x,y,z]^T
   * @param pixel reference to an optional vector with image coordinates after
   * point has been projected into the image plane [u,v]^T
   * @param J 2 x 3 projection jacobian.
   * For ProjectPoint: [u,v]^T = [P1(x, y, z), P2(x, y, z)]^T
   *                   J = | dP1/dx , dP1/dy, dP1/dz |
   *                       | dP2/dx , dP2/dy, dP2/dz |
   */
  void ProjectPoint(const Eigen::Vector3d& point,
                    std::optional<Eigen::Vector2i>& pixel,
                    std::optional<Eigen::MatrixXd>& J) override;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param point [u,v]
   */
  void BackProject(const Eigen::Vector2i& pixel,
                   std::optional<Eigen::Vector3d>& ray) override;

  /**
   * @brief Method for validating the inputs. This will be called in the load
   * configuration file step and should validate the intrinsics (i.e. size) and
   * the type
   */
  void ValidateInputs() override;

  /**
   * @brief Method for undistorting an image based on camera's distortion
   * @param image_input image to be undistorted
   * @param image_output reference to Mat obejct to store output
   */
  virtual void UndistortImage(const cv::Mat& image_input,
                              cv::Mat& image_output) = 0;

protected:
  /**
   * @brief Method to distort point
   * @return Vec2 distorted point
   * @param pixel to undistort
   */
  virtual Eigen::Vector2i DistortPixel(const Eigen::Vector2i& pixel) = 0;

  /**
   * @brief Method to undistort point
   * @return Vec2 undistorted point
   * @param pixel to undistort
   */
  virtual Eigen::Vector2i UndistortPixel(const Eigen::Vector2i& pixel) = 0;

  /**
   * @brief Method to compute jacobian of the distortion function
   * @return Jacobian
   * @param pixel to compute jacobian around
   */
  virtual Eigen::Matrix2d
      ComputeDistortionJacobian(const Eigen::Vector2i& pixel) = 0;

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
