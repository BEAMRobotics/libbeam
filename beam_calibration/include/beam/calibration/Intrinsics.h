#pragma once
#include "beam/utils/math.hpp"

namespace beam_calibration {

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
   * @brief Pure virtual method for returning the frame id of an intrinsics
   * calibration object
   * @return Returns frame id
   */
  virtual std::string GetFrameId() = 0;

  /**
   * @brief Pure virtual method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  virtual void AddFrameId(std::string frame_id) = 0;

  /**
   * @brief Pure virtual method for getting the image dimensions
   * @return img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  virtual beam::Vec2 GetImgDims() = 0;

  /**
   * @brief Pure virtual method for adding the image dimensions
   * @param img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  virtual void AddImgDims(beam::Vec2 img_dims) = 0;


};

} // namespace beam_calibration
