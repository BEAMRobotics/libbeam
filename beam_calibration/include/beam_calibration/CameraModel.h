/** @file
 * @ingroup calibration
 */

#pragma once

#include <fstream>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>

#include <beam_utils/log.h>
#include <beam_utils/optional.h>

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
   * @param[out] out_pixel pixel the point projects to [col, row]
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
   * @param[in] in_pixel pixel to back project [col, row]
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
   * @brief Create the undistortion map
   */
  void InitUndistortMap();

  /**
   * @brief undistort a pixel
   * @param in_pixel in distorted image
   * @param out_pixel undistorted pixel
   * @return whether its valid or not
   */
  bool UndistortPixel(const Eigen::Vector2i& in_pixel,
                      Eigen::Vector2i& out_pixel);

  /**
   * @brief Returns a rectified camera model
   */
  std::shared_ptr<CameraModel> GetRectifiedModel();

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
   * @brief Method for adding the safe projection radius. This is a radius from
   * the center of the image where projections are valid. Of course, points also
   * need to project to the image plane. By default, this is set to zero, if set
   * to zero then it won't be applied. If this value is not in the config json,
   * then it will also be set to zero
   * @param safe_projection_radius distance in pixels from the center of the
   * image to the edge of the projection circle. This may fall outside the image
   * plane for some parts of the image.
   */
  void SetSafeProjectionRadius(uint32_t safe_projection_radius);

  /**
   * @brief Method for adding intrinsic values
   * @param intrinsics of the camera. We expect all intrinsics to start with fx,
   * fy, cx, cy
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
   * @brief Method for getting the safe projection radius. See definition in
   * SetSafeProjectionRadius()
   * @return safe projection radius, in pixels from center
   */

  uint32_t GetSafeProjectionRadius();

  /**
   * @brief Method for retrieving the intrinsic values of the model
   * @return intrinsics of the camera
   */
  const Eigen::VectorXd& GetIntrinsics() const;

  /**
   * @brief Method for retrieving the camera type
   * @return camera type
   */
  CameraType GetType() const;

  /**
   * @brief Method for checking if pixel is in image, and in the safe projection
   * radius
   * @return Returns boolean
   * @param pixel
   */
  bool PixelInImage(const Eigen::Vector2i& pixel);

  /**
   * @brief Method for checking if pixel is in image, and in the safe projection
   * radius
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

  /**
   * @brief Create a Camera Frustum object for visualization, in the camera
   * coordinate frame
   * @param t timestamp to add to the points
   * @param increment distance between points
   * @param length of each ray of the frustum
   * @param RGB color of frustum
   * @return pcl::PointCloud<pcl::PointXYZRGBL>
   */
  pcl::PointCloud<pcl::PointXYZRGBL> CreateCameraFrustum(
      const ros::Time& t = ros::Time(0), double increment = 0.01,
      double length = 0.3,
      const Eigen::Vector3i& RGB = Eigen::Vector3i(255, 0, 0));

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

  std::shared_ptr<cv::Mat> pixel_map_;
  std::shared_ptr<CameraModel> rectified_model_;

  CameraType type_; // THIS SHOULD BE SET IN EACH DERIVED CLASS CONSTRUCTOR
  std::string frame_id_{""};
  std::string calibration_date_{""};
  uint32_t image_height_{0};
  uint32_t image_width_{0};
  uint32_t safe_projection_radius_{0};
  Eigen::VectorXd intrinsics_;

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

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace beam_calibration
