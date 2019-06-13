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
 * @brief Enum class for different types of dsitortion models
 */
enum class DistortionType { NONE = 0, RADTAN, EQUIDISTANT };

/**
 * @brief Struct to perform distortion functions
 */
struct Distortion {
  DistortionType type_;
  /*
   * @brief Initialize with type
   */
  Distortion() = default;
  /*
   * @brief Default destructor
   */
  virtual ~Distortion() = default;
  /*
   * @brief Method to return type of distortion
   * @return DistortionType
   */
  DistortionType GetType();
  /*
   * @brief Method to distort point
   * @return Vec2 distorted point
   * @param VecX: intrinsics
   * @param Vec2: point
   */
  virtual beam::Vec2 DistortPixel(beam::VecX, beam::Vec2) const = 0;

  /*
   * @brief Method to undistort point
   * @return Vec2 undistorted point
   * @param Vec2: point
   */
  virtual beam::Vec2 UndistortPixel(beam::VecX, beam::Vec2) const = 0;

  /*
   * @brief Method to compute jacobian of the distortion function
   * @return Mat2: jacobian
   * @param VecX: coefficients
   * @param Vec2: point
   */
  virtual beam::Mat2 ComputeJacobian(beam::VecX, beam::Vec2) const = 0;
  /*
   * @brief Method to undistort image
   * @return undistorted image
   * @param Mat3: camera matrix
   * @param VecX dsitortion coefficients
   * @param cv::Mat image to undistort
   * @param uint32_t height and width
   */
  virtual cv::Mat UndistortImage(beam::Mat3, beam::VecX, const cv::Mat&,
                                 uint32_t, uint32_t) const = 0;
};

/*
 *@brief Struct to perform distortion functions for radial tangential model
 */
struct Radtan : Distortion {
  Radtan();
  ~Radtan() = default;
  beam::Vec2 DistortPixel(beam::VecX, beam::Vec2) const override;
  beam::Vec2 UndistortPixel(beam::VecX, beam::Vec2) const override;
  beam::Mat2 ComputeJacobian(beam::VecX, beam::Vec2) const override;
  cv::Mat UndistortImage(beam::Mat3, beam::VecX, const cv::Mat&, uint32_t,
                         uint32_t) const override;
};

/*
 *@brief Struct to perform distortion functions for equidistant model
 */
struct Equidistant : Distortion {
  Equidistant();
  ~Equidistant() = default;
  beam::Vec2 DistortPixel(beam::VecX, beam::Vec2) const override;
  beam::Vec2 UndistortPixel(beam::VecX, beam::Vec2) const override;
  beam::Mat2 ComputeJacobian(beam::VecX, beam::Vec2) const override;
  cv::Mat UndistortImage(beam::Mat3, beam::VecX, const cv::Mat&, uint32_t,
                         uint32_t) const override;
};

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
      Create(CameraType camera_type, DistortionType dist_type,
             beam::VecX intrinsics, beam::VecX distortion,
             uint32_t image_height, uint32_t image_width, std::string frame_id,
             std::string date);

  /**
   * @brief Method for projecting a point into an image plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. Not in homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec3 point) = 0;

  /**
   * @brief Method for projecting a point in homographic form into an image
   * plane
   * @return Returns image coordinates after point has been projected into image
   * plane.
   * @param X point to be projected. In homographic form
   */
  virtual beam::Vec2 ProjectPoint(beam::Vec4 point) = 0;

  /**
   * @brief Method distorting a point
   * @return Returns distorted point
   * @param undistorted point
   */
  virtual beam::Vec2 DistortPoint(beam::Vec2 point) = 0;

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
   * @brief Method for adding the distortion model
   * @param distortion model
   */
  virtual void SetDistortionCoefficients(beam::VecX coeffs);

  /**
   * @brief Method for setting distortion type
   * @param distortion model
   */
  virtual void SetDistortionType(DistortionType dist);

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
   * @brief Method for retrieving the distortion model
   * @return distortion model
   */
  virtual const beam::VecX GetDistortionCoefficients() const;

  /**
   * @brief Method for setting distortion type
   * @param distortion model
   */
  virtual DistortionType GetDistortionType() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  virtual CameraType GetType() const;

  /**
   * @brief Method for retrieving fx
   * @return fx
   */
  virtual double GetFx() const;

  /**
   * @brief Method for retrieving fy
   * @return fy
   */
  virtual double GetFy() const;

  /**
   * @brief Method for retrieving cx
   * @return cx
   */
  virtual double GetCx() const;

  /**
   * @brief Method for retrieving cy
   * @return cy
   */
  virtual double GetCy() const;

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
  beam::VecX intrinsics_, distortion_coefficients_;
  std::unique_ptr<Distortion> distortion_ = nullptr;
  // Boolean values to keep track of validity
  bool intrinsics_valid_ = false, distortion_coeffs_set_ = false,
       calibration_date_set_ = false, distortion_set_ = false;
  // Map for keeping required number of values in distortion vector
  std::map<CameraType, int> intrinsics_size_ = {{CameraType::LADYBUG, 4},
                                                {CameraType::PINHOLE, 4}};

  std::map<DistortionType, int> distortion_size_ = {
      {DistortionType::NONE, 0},
      {DistortionType::RADTAN, 5},
      {DistortionType::EQUIDISTANT, 4}};
};

/** @} group calibration */

} // namespace beam_calibration