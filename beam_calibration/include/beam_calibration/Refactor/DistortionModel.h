/** @file
 * @ingroup calibration
 */

#pragma once
// beam
#include "beam_utils/math.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

namespace beam_calibration {

/**
 * @brief Abstract class for camera models
 */
class DistortionModel {
public:
  /**
   * @brief default constructor
   */
  DistortionModel() = default;

  /**
   * @brief Default destructor
   */
  virtual ~DistortionModel() = default;

  /**
   * @brief Method to distort point
   * @return Vec2 distorted point
   * @param pixel to undistort
   */
  virtual beam::Vec2 DistortPixel(beam::Vec2 pixel) const;

  /**
   * @brief Method to undistort point
   * @return Vec2 undistorted point
   * @param pixel to undistort
   */
  virtual beam::Vec2 UndistortPixel(beam::Vec2 pixel) const;

  /**
   * @brief Method to compute jacobian of the distortion function
   * @return Jacobian
   * @param pixel to compute jacobian around
   */
  virtual beam::Mat2 ComputeJacobian(beam::Vec2 pixel) const;

  /**
   * @brief Method to undistort image
   * @return undistorted image
   * @param Mat3: camera matrix
   * @param cv::Mat image to undistort
   */
  virtual cv::Mat UndistortImage(beam::Mat3, const cv::Mat&) const;

  /**
   * @brief Method to set coefficients
   * @param coefficients vector of distortion coefficients
   */
  virtual void SetDistortionCoefficients(beam::VecX coefficients) const;

protected:
  beam::VecX distortion_coefficients_;
};

} // namespace beam_calibration