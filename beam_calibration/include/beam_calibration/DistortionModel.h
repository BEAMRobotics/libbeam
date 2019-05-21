/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_utils/math.hpp"
#include <fstream>
#include <iostream>

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Enum class for different types of distortion models
 */
enum class DistortionType { RADTAN = 0, EQUIDISTANT };
/**
 * @brief Abstract class for distortion models
 */
class DistortionModel {
public:
  /**
   * @brief Default constructor
   */
  DistortionModel() = default;

  /**
   * @brief Default destructor
   */
  virtual ~DistortionModel() = default;

  /**
   * @brief Factory method for instantiating intrinsics
   * @param type Type of Vehicle to create
   * @return
   */
  static std::unique_ptr<DistortionModel> Create(DistortionType type,
                                                 beam::VecX coeffs);

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 Distort(beam::Vec2& X) = 0;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  virtual beam::Vec2 Undistort(beam::Vec2& X) = 0;

  /**
   * @brief Method for adding coefficients
   * @param coefficient vector
   */
  virtual void SetCoefficients(beam::VecX coeffs);

  /**
   * @brief Method for retrieving coefficients
   * @return coefficient vector
   */
  virtual const beam::VecX GetCoefficients() const;

protected:
  /// Parameter vector for the coefficients.
  beam::VecX coefficients_;
};

/** @} group calibration */

} // namespace beam_calibration
