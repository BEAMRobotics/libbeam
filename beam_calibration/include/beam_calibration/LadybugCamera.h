/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/CameraModel.h"

//#include <ladybug/ladybug.h>
//#include <ladybug/ladybuggeom.h>
//#include <ladybug/ladybugrenderer.h>

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Derived class for pinhole intrinsics
 */
class LadybugCamera : public CameraModel {
public:
  /**
   * @brief Default destructor
   */
  LadybugCamera() = default;

  /**
   * @brief Default destructor
   */
  ~LadybugCamera() = default;

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

private:
  void ReadCameraCalibration(std::string calibration_path);

  void LadybugCheckError();

  // LadybugContext lb_context_;
  // LadybugError lb_error_;

  unsigned int cam_id_ = 0;
  const unsigned int LB_FULL_WIDTH = 2048;
  const unsigned int LB_FULL_HEIGHT = 2464;
  beam::Vec2 full_img_dims_ = {LB_FULL_WIDTH, LB_FULL_HEIGHT};
};

/** @} group calibration */

} // namespace beam_calibration