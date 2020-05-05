/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/CameraModel.h"
#include "beam_calibration/DistortionModel.h"

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Derived class for pinhole camera model
 */
class PinholeCamera : public CameraModel {
public:
  /**
   * @brief Default destructor
   */
  PinholeCamera();

  /**
   * @brief Default destructor
   */
  ~PinholeCamera() = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  beam::Vec2 ProjectPoint(beam::Vec3 point) override;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param distorted point
   */
  beam::Vec3 BackProject(beam::Vec2 point) override;

  /**
   * @brief Method for undistorting an image based on camera's distortion
   * @return image
   */
  cv::Mat UndistortImage(cv::Mat image_input) override;

  /**
   * @brief Method for projecting a point into an image plane without distortion
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param point to be projected. Not in homographic form
   */
  beam::Vec2 ProjectUndistortedPoint(beam::Vec3 point);

  /**
   * @brief Sets distortion model for pinhole camera
   * @param model to be set for distortion
   */
  void SetDistortion(std::shared_ptr<DistortionModel> model);

protected:
  std::shared_ptr<DistortionModel> distortion_ = nullptr;
};

/** @} group calibration */

} // namespace beam_calibration