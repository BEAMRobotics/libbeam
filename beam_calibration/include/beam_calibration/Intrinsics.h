/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_utils/math.hpp"

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class IntrinsicsType { PINHOLE = 0, FISHEYE, LADYBUG };

/**
 * @brief Abstract class for calibrations
 */
class Intrinsics {
public:
  /**
   * @brief Default constructor
   */
  Intrinsics() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Intrinsics() = default;

  /**
   * @brief Pure virtual method for returning type of intrinsics
   * @return type of intrinsics object
   */
  virtual IntrinsicsType GetType() const = 0;


  /**
   * @brief Factory method for instantiating intrinsics
   * @param type Type of Vehicle to create
   * @return
   */
  static std::unique_ptr<Intrinsics> Create(IntrinsicsType type);


  /**
   * @brief Pure virtual method for loading a pinhole calibration from a .json
   * @param file_location absolute path to json file
   */
  virtual void LoadJSON(std::string &file_location) = 0;

  /**
   * @brief Pure virtual method for returning the frame id of an intrinsics
   * calibration object
   * @return Returns frame id
   */
  virtual std::string GetFrameId() = 0;

  /**
   * @brief Pure virtual method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  virtual void SetFrameId(std::string &frame_id) = 0;

  /**
   * @brief Pure virtual method for getting the image dimensions
   * @return img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  virtual beam::Vec2 GetImgDims() = 0;

  /**
   * @brief Pure virtual method for adding K matrix
   * @param K intrinsics matrix
   */
  virtual void SetK(beam::Mat3 K) = 0;

  /**
   * @brief Pure virtual method for returning K matrix
   * @return intrinsics matrix K_
   */
  virtual beam::Mat3 GetK() = 0;

  /**
   * @brief Pure virtual method for adding the image dimensions
   * @param img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  virtual void SetImgDims(beam::Vec2 &img_dims) = 0;

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec3 &X) = 0;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec4 &X) = 0;

  /**
   * @brief Method for projecting a point into an image plane where the image is
   * distorted.
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 ProjectDistortedPoint(beam::Vec3 &X) = 0;
};

/** @} group calibration */

} // namespace beam_calibration
