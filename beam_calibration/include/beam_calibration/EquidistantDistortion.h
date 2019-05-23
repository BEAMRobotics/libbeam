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
class EquidistantDistortion : public DistortionModel {
public:
  /**
   * @brief Default destructor
   */
  EquidistantDistortion() = default;

  /**
   * @brief Default constructor
   */
  EquidistantDistortion(beam::VecX coeffs,
                        beam_calibration::DistortionType type);

  /**
   * @brief Default destructor
   */
  ~EquidistantDistortion() = default;

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

protected:
  /// Parameter vector for the coefficients.
  beam::VecX coefficients_;
  beam_calibration::DistortionType type_;
  // boolean values to keep track of validity
  bool coefficients_valid_ = false;
};

/** @} group calibration */

} // namespace beam_calibration