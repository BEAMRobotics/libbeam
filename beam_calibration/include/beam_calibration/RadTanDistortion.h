/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/DistortionModel.h"

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Abstract class for distortion models
 */
class RadTanDistortion : public DistortionModel {
public:
  /**
   * @brief Default destructor
   */
  RadTanDistortion() = default;

  /**
   * @brief Default constructor
   */
  RadTanDistortion(beam::VecX coeffs);

  /**
   * @brief Default destructor
   */
  ~RadTanDistortion() = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  beam::Vec2 Distort(beam::Vec2& point) override;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  beam::Vec2 Undistort(beam::Vec2& point) override;

  /**
   * @brief Method undistorting an image
   * @return Returns undistorted image
   * @param distorted image
   */
  cv::Mat UndistortImage(const cv::Mat& input_image, cv::Mat K) override;
};

/** @} group calibration */

} // namespace beam_calibration