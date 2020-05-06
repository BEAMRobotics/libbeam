/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/include/Refactor/DistortionModel.h"

namespace beam_calibration {

/**
 * @brief Derived class for pinhole camera model
 */
class EquidistantDistortion : public DistortionModel {
public:
  /**
   * @brief default constructor
   */
  EquidistantDistortion() = default;

  /**
   * @brief Initialize with coefficients
   */
  EquidistantDistortion(beam::VecX coefficients);

  /**
   * @brief Default destructor
   */
  ~EquidistantDistortion() = default;

  /**
   * @brief Method to distort point
   * @return Vec2 distorted point
   * @param pixel to undistort
   */
  beam::Vec2 DistortPixel(beam::Vec2 pixel) override;

  /**
   * @brief Method to undistort point
   * @return Vec2 undistorted point
   * @param pixel to undistort
   */
  beam::Vec2 UndistortPixel(beam::Vec2 pixel) override;

  /**
   * @brief Method to compute jacobian of the distortion function
   * @return Jacobian
   * @param pixel to compute jacobian around
   */
  beam::Mat2 ComputeJacobian(beam::Vec2 pixel) override;

  /**
   * @brief Method to undistort image
   * @return undistorted image
   * @param Mat3: camera matrix
   * @param cv::Mat image to undistort
   */
  cv::Mat UndistortImage(beam::Mat3 intrinsics, const cv::Mat& image) override;

  /**
   * @brief Method to set coefficients
   * @param coefficients vector of distortion coefficients
   */
  void SetDistortionCoefficients(beam::VecX coefficients) override;
};

/** @} group calibration */

} // namespace beam_calibration