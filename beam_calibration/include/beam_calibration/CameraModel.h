/** @file
 * @ingroup calibration
 */

#pragma once
#include "beam_calibration/DistortionModel.h"
#include "beam_utils/math.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

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
   * @brief Construct with values
   */
  CameraModel(beam_calibration::CameraType camera_type, beam::VecX& intrinsics,
              std::unique_ptr<DistortionModel> distortion, uint32_t image_width,
              uint32_t image_height, std::string frame_id, std::string date);

  /**
   * @brief Default destructor
   */
  virtual ~CameraModel() = default;

  /**
   * @brief method to instantiate camera model from json file
   * @param file_location absolute path to json file
   */
  static std::unique_ptr<beam_calibration::CameraModel>
      LoadJSON(std::string& file_location);

  /**
   * @brief Factory method for camera models
   * @param type Type of Vehicle to create
   * @return
   */
  static std::unique_ptr<CameraModel>
      Create(beam_calibration::CameraType camera_type, beam::VecX intrinsics,
             std::unique_ptr<DistortionModel> distortion, uint32_t image_width,
             uint32_t image_height, std::string frame_id, std::string date);

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec3& X) = 0;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec4& X) = 0;

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
   * @param intrinsics of the camera
   */
  virtual void SetIntrinsics(beam::VecX instrinsics);

  /**
   * @brief Method for adding the distortion model
   * @param distortion model
   */
  virtual void SetDistortion(
      std::unique_ptr<beam_calibration::DistortionModel> distortion);

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  virtual void SetType(beam_calibration::CameraType type);

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
  virtual const beam::VecX GetIntrinsics() const;

  /**
   * @brief Method for retrieving the distortion model
   * @return distortion model
   */
  virtual const beam_calibration::DistortionModel& GetDistortion() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  virtual beam_calibration::CameraType GetType();

protected:
  beam_calibration::CameraType type_;
  std::string frame_id_, calibration_date_;
  uint32_t image_height_, image_width_;
  // Parameter vector for the intrinsic parameters of the model.
  beam::VecX intrinsics_;
  // The distortion for this camera.
  std::unique_ptr<beam_calibration::DistortionModel> distortion_;
};

/** @} group calibration */

} // namespace beam_calibration
