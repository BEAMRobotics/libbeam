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
enum class DistortionType { NONE = 0, RADTAN, EQUIDISTANT };
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
   * @brief Construct with values
   */
  DistortionModel(beam::VecX coeffs, beam_calibration::DistortionType type);

  /**
   * @brief Default destructor
   */
  virtual ~DistortionModel() = default;

  /**
   * @brief Factory method for instantiating intrinsics
   * @param type Type of Vehicle to create
   * @return
   */
  static std::shared_ptr<DistortionModel> Create(DistortionType type,
                                                 beam::VecX coeffs);

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 Distort(beam::Vec2& point) = 0;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  virtual beam::Vec2 Undistort(beam::Vec2& point) = 0;

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

  /**
   * @brief Method for setting the type of model
   * @param distortion type
   */
  virtual void SetType(beam_calibration::DistortionType type);

  /**
   * @brief Method for retreiving the type of model
   * @return distortion type
   */
  virtual beam_calibration::DistortionType GetType();

protected:
  // Map for keeping required number of coefficient variables based on
  // distortion type
  std::map<beam_calibration::DistortionType, int> get_size_ = {
      {beam_calibration::DistortionType::NONE, 0},
      {beam_calibration::DistortionType::RADTAN, 5},
      {beam_calibration::DistortionType::EQUIDISTANT, 4}};
  // Parameter vector for the coefficients.
  beam::VecX coefficients_;
  beam_calibration::DistortionType type_;
  // boolean values to keep track of validity
  bool coefficients_valid_ = false;
  // constants to hold types
  beam_calibration::DistortionType
      NONE = beam_calibration::DistortionType::NONE,
      RADTAN = beam_calibration::DistortionType::RADTAN,
      EQUIDISTANT = beam_calibration::DistortionType::EQUIDISTANT;
};

/** @} group calibration */

} // namespace beam_calibration