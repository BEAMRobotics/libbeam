/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/CameraModel.h"

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Derived class for pinhole intrinsics
 */
class PinholeCamera : public CameraModel {
public:
  /**
   * @brief Default destructor
   */
  PinholeCamera() = default;

  /**
   * @brief constructor with values
   */
  PinholeCamera(beam::VecX& intrinsics,
                std::shared_ptr<DistortionModel> distortion,
                uint32_t image_width, uint32_t image_height,
                std::string frame_id, std::string date);

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
  beam::Vec2 ProjectPoint(beam::Vec3& point) override;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  beam::Vec2 ProjectPoint(beam::Vec4& point) override;

  /**
   * @brief Method for undistorting an image based on camera's distortion
   * @return image
   */
  cv::Mat UndistortImage(cv::Mat& image_input) override;
};

/** @} group calibration */

} // namespace beam_calibration