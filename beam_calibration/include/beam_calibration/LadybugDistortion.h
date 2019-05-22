/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/DistortionModel.h"

#include <ladybug/ladybug.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Abstract class for distortion models
 */
class LadybugDistortion : public DistortionModel {
public:
  /**
   * @brief Default destructor
   */
  LadybugDistortion() = default;

  /**
   * @brief Construct with camera id
   */
  LadybugDistortion(unsigned int id);

  /**
   * @brief Default destructor
   */
  ~LadybugDistortion() = default;

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
   * @brief Method for creating ladybug context from file
   * @param path to config file
   */
  void LoadConfig(std::string& file);

protected:
  /// Parameter vector for the coefficients.
  beam::VecX coefficients_;

private:
  void LadybugCheckError();

  LadybugContext lb_context_;
  LadybugError lb_error_;

  unsigned int cam_id_ = 0;
  const unsigned int LB_FULL_WIDTH = 2048;
  const unsigned int LB_FULL_HEIGHT = 2464;
  beam::Vec2 img_dims_ = {LB_FULL_WIDTH, LB_FULL_HEIGHT};
};

/** @} group calibration */

} // namespace beam_calibration