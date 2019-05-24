/** @file
 * @ingroup calibration
 */

#pragma once
// beam
#include "beam_utils/math.hpp"

// std library
#include <fstream>
#include <iostream>

// OpenCV
#include <opencv2/opencv.hpp>

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
  DistortionModel(beam::VecX coeffs, DistortionType type);

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
   * @brief Method undistorting an image
   * @return Returns undistorted image
   * @param distorted image
   */
  virtual cv::Mat UndistortImage(const cv::Mat& input_image,
                                 std::vector<double> intrinsics) = 0;

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
  virtual void SetType(DistortionType type);

  /**
   * @brief Method for retreiving the type of model
   * @return distortion type
   */
  virtual DistortionType GetType();

protected:
  beam::VecX coefficients_;
  DistortionType type_;
  bool coefficients_valid_ = false;
  // Map for keeping required number of coefficient variables based on
  // distortion type
  std::map<DistortionType, int> get_size_ = {{DistortionType::NONE, 0},
                                             {DistortionType::RADTAN, 5},
                                             {DistortionType::EQUIDISTANT, 4}};
};

/** @} group calibration */

} // namespace beam_calibration