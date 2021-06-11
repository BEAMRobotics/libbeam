/** @file
 * @ingroup calibration
 */

#pragma once

#include <beam_utils/utils.h>

#include <beam_utils/optional.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

static bool default_bool = false;

namespace beam_calibration {

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class CameraType {
  RADTAN = 0,
  KANNALABRANDT,
  DOUBLESPHERE,
  LADYBUG,
  CATADITROPIC
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
   * @brief Factory method to create camera models at runtime
   */
  static std::shared_ptr<CameraModel> Create(std::string& file_location);

  /**
   * @brief Method to perform a deep copying of this object
   */
  virtual std::shared_ptr<CameraModel> Clone() = 0;

  /**
   * @brief Method for projecting a point into an image plane, if the input
   * point is outside of the valid projection domain, then the point will not be
   * projected and will retain whatever value the input parameter has
   * @param[in] in_point 3d point to be projected [x,y,z]^T
   * @param[out] out_pixel pixel the point projects to
   * @param[in] J optional param to compute the jacobian
   * @param[out] in_image_plane true if the pixel is outside of the image plane
   * @return whether the input point is in the domain of the function
   */
  virtual bool ProjectPoint(const Eigen::Vector3d& in_point,
                            Eigen::Vector2d& out_pixel,
                            bool& in_image_plane = default_bool,
                            std::shared_ptr<Eigen::MatrixXd> J = nullptr) = 0;

  /**
   * @brief Method back projecting, if the input pixel is outside of back
   * projection domain then it will not compute the back projection
   * @param[in] in_pixel pixel to back project
   * @param[out] out_point ray towards the input pixel
   * @return return whether the input pixel is in the domain of the function
   */
  virtual bool BackProject(const Eigen::Vector2i& in_pixel,
                           Eigen::Vector3d& out_point) = 0;

  /**
   * @brief Method for checking if a 3d point is projectable
   * @return Returns boolean
   * @param point
   */
  virtual bool InProjectionDomain(const Eigen::Vector3d& point) = 0;

  /**
   * @brief Method for setting the LadyBug camera ID
   * @param id of the camera to use
   */
  virtual void SetCameraID(const unsigned int id);

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
  uint32_t GetHeight() const;

  /**
   * @brief Method for getting the image width
   * @return image width
   */
  uint32_t GetWidth() const;

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
  bool PixelInImage(const Eigen::Vector2i& pixel);

  /**
   * @brief Method for checking if pixel is in image
   * @return Returns boolean
   * @param pixel
   */
  bool PixelInImage(const Eigen::Vector2d& pixel);

  /**
   * @brief Method for writing camera model to a json file
   * @param file_path full path to json
   */
  void WriteJSON(const std::string& file_path,
                 const std::string& method = std::string());

protected:
  /**
   * @brief Method for loading calibration information from a json.
   * @param file_path full path to json
   */
  void LoadJSON(const std::string& file_path);

  /**
   * @brief Method for outputting all camera model types from intrinsics_types_
   */
  void OutputCameraTypes();

  CameraType type_; // THIS SHOULD BE SET IN EACH DERIVED CLASS CONSTRUCTOR
  std::string frame_id_{""};
  std::string calibration_date_{""};
  uint32_t image_height_{0};
  uint32_t image_width_{0};
  Eigen::VectorXd intrinsics_;

  // static bool outside_domain_default_ = false;

  unsigned int cam_id_ = 0;

  // Map for keeping required number of values in distortion vector
  std::map<CameraType, int> intrinsics_size_ = {{CameraType::LADYBUG, 4},
                                                {CameraType::RADTAN, 8},
                                                {CameraType::KANNALABRANDT, 8},
                                                {CameraType::DOUBLESPHERE, 6},
                                                {CameraType::CATADITROPIC, 9}};
  // Map for storing string input
  std::map<std::string, CameraType> intrinsics_types_ = {
      {"LADYBUG", CameraType::LADYBUG},
      {"RADTAN", CameraType::RADTAN},
      {"KANNALABRANDT", CameraType::KANNALABRANDT},
      {"DOUBLESPHERE", CameraType::DOUBLESPHERE},
      {"CATADITROPIC", CameraType::CATADITROPIC}};
};

} // namespace beam_calibration
