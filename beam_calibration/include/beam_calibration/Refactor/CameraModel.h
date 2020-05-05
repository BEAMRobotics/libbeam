/** @file
 * @ingroup calibration
 */

#pragma once
// beam
#include "beam_utils/math.hpp"

// format libraries
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

// OpenCV
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace beam_calibration {
/** @addtogroup calibration
 *  @{ */

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class CameraType { PINHOLE = 0, LADYBUG };

/**
 * @brief Abstract class for camera models
 */
class CameraModel {
public:
  /**
   * @brief Default constructor
   */
  CameraModel() = default;

  /**
   * @brief Factory constructor
   */
  CameraModel(std::string& file_location);

  /**
   * @brief Default destructor
   */
  virtual ~CameraModel() = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param point to be projected
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec3 point) = 0;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param distorted point [u,v]
   */
  virtual beam::Vec3 BackProject(beam::Vec2 point) = 0;

  /**
   * @brief Method for undistorting an image based on camera's distortion
   * @return image
   */
  virtual cv::Mat UndistortImage(cv::Mat image_input) = 0;

  /**
   * @brief Method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  virtual void SetFrameID(std::string id);

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  virtual void SetCalibrationDate(std::string id);

  /**
   * @brief Method for adding the image dimensions
   * @param height and width of the image plane
   */
  virtual void SetImageDims(uint32_t height, uint32_t width);

  /**
   * @brief Method for adding intrinsic values
   * Pinhole: [fx,fy,cx,cy]
   * Ladybug: [fx,fy,cy,cx]
   * @param intrinsics of the camera
   */
  virtual void SetIntrinsics(beam::VecX instrinsics);

  /**
   * @brief Method for returning the frame id of an intrinsics
   * calibration object
   * @return Returns frame id
   */
  virtual const std::string GetFrameID() const;

  /**
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  virtual const std::string GetCalibrationDate() const;

  /**
   * @brief Method for getting the image height
   * @return image height
   */
  virtual uint32_t GetHeight() const;

  /**
   * @brief Method for getting the image width
   * @return image width
   */
  virtual uint32_t GetWidth() const;

  /**
   * @brief Method for retrieving the intrinsic values of the model
   * @return intrinsics of the camera
   */
  virtual const beam::VecX GetIntrinsics() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  virtual CameraType GetType() const;

  /**
   * @brief Method for retrieving camera matrix
   * @return K
   */
  virtual beam::Mat3 GetCameraMatrix() const;

  /**
   * @brief Method for checking if pixel is in image
   * @return Returns boolean
   * @param pixel
   */
  bool PixelInImage(beam::Vec2 pixel_in);

protected:
  CameraType type_;
  std::string frame_id_, calibration_date_;
  uint32_t image_height_, image_width_;
  beam::VecX intrinsics_;
  // Boolean values to keep track of validity
  bool intrinsics_valid_ = false, calibration_date_set_ = false;
  // Map for keeping required number of values in distortion vector
  std::map<CameraType, int> intrinsics_size_ = {{CameraType::LADYBUG, 4},
                                                {CameraType::PINHOLE, 4}};
};

} // namespace beam_calibration