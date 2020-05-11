/** @file
 * @ingroup calibration
 */

#pragma once

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace beam_calibration {

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class CameraType {
  PINHOLE_RADTAN = 0,
  PINHOLE_EQUI,
  DOUBLESPHERE,
  LADYBUG
};

/**
 * @brief Abstract class for camera models
 */
class CameraModel {
public:
  /**
   * @brief Constructor
   * @param input_file path to input file
   */
  virtual CameraModel(const std::string& file_path) = 0;

  /**
   * @brief Default destructor
   */
  virtual ~CameraModel() = default;

  /**
   * @brief Method for projecting a point into an image plane
   * @param point 3d point to be projected [x,y,z]^T
   * @param pixel reference to an optional vector with image coordinates after
   * point has been projected into the image plane [u,v]^T
   */
  virtual void ProjectPoint(const Eigen::Vector3d& point,
                            std::optional<Eigen::Vector2d>& pixel) = 0;

  /**
   * @brief Method back projecting
   * @return Returns bearing vector
   * @param point [u,v]
   */
  virtual void BackProject(const Eigen::Vector2d& pixel,
                           std::optional<Eigen::Vector3d>& ray) = 0;

  /**
   * @brief Method for adding the frame id
   * @param frame_id frame associated with the intrinsics calibration object
   */
  void SetFrameID(const std::string& id);

  /**
   * @brief Method for setting the date that the calibration was done
   * @param Calibration date
   */
  void SetCalibrationDate(const std::string& date);

  /**
   * @brief Method for adding the image dimensions
   * @param height and width of the image plane
   */
  void SetImageDims(const uint32_t height, const uint32_t width);

  /**
   * @brief Method for adding intrinsic values
   * @param intrinsics of the camera
   */
  void SetIntrinsics(const Eigen::VectorXd& instrinsics);

  /**
   * @brief Method for returning the frame id of a camera object
   * @return Returns frame id
   */
  const std::string GetFrameID() const;

  /**
   * @brief Method for retrieving the date that the calibration was done
   * @return Return calibration date
   */
  const std::string GetCalibrationDate() const;

  /**
   * @brief Method for getting the image height
   * @return image height
   */
  const uint32_t GetHeight() const;

  /**
   * @brief Method for getting the image width
   * @return image width
   */
  const uint32_t GetWidth() const;

  /**
   * @brief Method for retrieving the intrinsic values of the model
   * @return intrinsics of the camera
   */
  const Eigen::VectorXd GetIntrinsics() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  CameraType GetType() const;

  /**
   * @brief Method for checking if pixel is in image
   * @return Returns boolean
   * @param pixel
   */
  bool PixelInImage(const Eigen::Vector2d& pixel);

protected:

  CameraType type_;
  std::string frame_id_{""};
  std::string calibration_date_{""};
  uint32_t image_height_{0};
  uint32_t image_width_{0};
  Eigen::VectorXd intrinsics_;
  // Map for keeping required number of values in distortion vector
  std::map<CameraType, int> intrinsics_size_ = {{CameraType::LADYBUG, 4},
                                                {CameraType::PINHOLE_RADTAN, 8},
                                                {CameraType::PINHOLE_EQUI, 8},
                                                {CameraType::DOUBLESPHERE, 6}};
};

} // namespace beam_calibration
