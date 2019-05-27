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
enum class CameraType { NONE = 0, PINHOLE, LADYBUG, EQUIDISTANT };
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
   * @brief Default destructor
   */
  virtual ~CameraModel() = default;

  /**
   * @brief method to instantiate camera model from json file
   * @param file_location absolute path to json file
   */
  static std::shared_ptr<CameraModel> LoadJSON(std::string& file_location);

  /**
   * @brief Factory method for camera models
   * @param type Type of model to create
   * @return
   */
  static std::shared_ptr<CameraModel>
      Create(CameraType camera_type, beam::VecX intrinsics,
             beam::VecX distortion, uint32_t image_width, uint32_t image_height,
             std::string frame_id, std::string date);

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec3& point) = 0;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec4& point) = 0;

  /**
   * @brief Method distorting a point
   * @return Returns distorted point
   * @param undistorted point
   */
  virtual beam::Vec2 DistortPoint(beam::Vec2& point) = 0;

  /**
   * @brief Method undistorting a point
   * @return Returns undistorted point
   * @param distorted point
   */
  virtual beam::Vec2 UndistortPoint(beam::Vec2& point) = 0;

  /**
   * @brief Method for undistorting an image based on camera's distortion
   * @return image
   */
  virtual cv::Mat UndistortImage(cv::Mat& image_input) = 0;

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
  virtual void SetIntrinsics(beam::VecX& instrinsics);

  /**
   * @brief Method for adding the distortion model
   * @param distortion model
   */
  virtual void SetDistortionCoefficients(beam::VecX& coeffs);

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
   * @brief Method for getting the image dimensions
   * @return img_dims dimensions of the images taken by the camera associated
   * with this intrinsics object: [height, width]^T
   */
  virtual beam::Vec2 GetImageDims() const;

  /**
   * @brief Method for retrieving the intrinsic values of the model
   * @return intrinsics of the camera
   */
  virtual const beam::VecX& GetIntrinsics() const;

  /**
   * @brief Method for retrieving the distortion model
   * @return distortion model
   */
  virtual const beam::VecX& GetDistortionCoefficients() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  virtual CameraType GetType();

  /**
   * @brief Method for retrieving fx
   * @return fx
   */
  virtual double GetFx();

  /**
   * @brief Method for retrieving fy
   * @return fy
   */
  virtual double GetFy();

  /**
   * @brief Method for retrieving cx
   * @return cx
   */
  virtual double GetCx();

  /**
   * @brief Method for retrieving cy
   * @return cy
   */
  virtual double GetCy();

  /**
   * @brief Method for retrieving camera matrix
   * @return K
   */
  virtual beam::Mat3 GetCameraMatrix();

protected:
  CameraType type_;
  std::string frame_id_, calibration_date_;
  uint32_t image_height_, image_width_, required_size_;
  beam::VecX intrinsics_, distortion_coefficients_;
  // Boolean values to keep track of validity
  bool intrinsics_valid_ = false, distortion_set_ = false,
       calibration_date_set_ = false;
  // Map for keeping required number of values in distortion vector
  std::map<CameraType, int> get_coeffs_size_ = {{CameraType::LADYBUG, 0},
                                                {CameraType::PINHOLE, 5},
                                                {CameraType::EQUIDISTANT, 4}};
};

/** @} group calibration */

} // namespace beam_calibration