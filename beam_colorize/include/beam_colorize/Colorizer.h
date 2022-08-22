/** @file
 * @ingroup colorizer
 * Includes all colorization classes / functions
 *
 * @defgroup colorizer
 * Colorization classes / functions
 */

#pragma once

// Gen
#include <boost/make_shared.hpp>

// libbeam
#include "beam_calibration/CameraModel.h"
#include "beam_containers/PointBridge.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/core.hpp>

// ROS
#include <sensor_msgs/Image.h>

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class ColorizerType { PROJECTION = 0, RAY_TRACE = 1 };

/**
 * @brief Pixel type for iterating through the image
 */
using Pixel = cv::Point3_<uchar>;

/**
 * @brief Abstract class which different colorization methods can implement
 */
class Colorizer {
public:
  Colorizer();

  virtual ~Colorizer() = default;

  static std::unique_ptr<Colorizer> Create(ColorizerType type);
  /**
   * @brief Method for adding a point cloud of point type XYZ. This is required.
   * @param cloud_input Input point cloud in lidar frame (see SetTransform to
   * update)
   */
  void SetPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

  /**
   * @brief Method for adding a point cloud of point type XYZRGB. This is
   * required.
   * @param cloud_input Input point cloud
   */
  void SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input);

  /**
   * @brief Method for adding an image of type scv::Mat. This is required.
   * @param image_input Input image
   */
  void SetImage(const cv::Mat& image_input);

  /**
   * @brief Method for adding an image of type sensor_msgs::Image. This is
   * required.
   * @param image_input Input image
   */
  void SetImage(const sensor_msgs::Image& image_input);

  /**
   * @brief Method for adding the intrinsics object. This is required.
   * @param intrinsics pointer to intrinsics abstract object
   */
  void SetIntrinsics(std::shared_ptr<beam_calibration::CameraModel> intrinsics);

  /**
   * @brief Method for adding a transformation between the image and point
   * cloud frames.
   * @param T_C_L transform from lidar frame to camera frame. Default is
   * the identity (no transformation)
   */
  void SetTransform(const Eigen::Matrix4d& T_C_L);

  /**
   * @brief Method for adding a the distortion condition of the image
   * @param image_distored set to true if the image is distorted, or false if
   * the image has already been undistorted. Default = true;
   */
  void SetDistortion(const bool& image_distored);

  /**
   * @brief Pure virtual method for colorizing a point cloud
   * @param return_in_cam_frame set to false to return the cloud in its original
   * frame. Internally, when a cloud is added it's converted to the camera
   * frame, so here when we return the pointcloud it can be in either frame.
   * @return Colored point cloud pointer
   */
  virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      ColorizePointCloud(bool return_in_cam_frame = false) const = 0;

  /**
   * @brief Pure virtual method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  virtual pcl::PointCloud<beam_containers::PointBridge>::Ptr
      ColorizeMask(bool return_in_cam_frame = false) const = 0;

protected:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloudInLidarFrame(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_colored) const;

  pcl::PointCloud<beam_containers::PointBridge>::Ptr GetCloudInLidarFrame(
      const pcl::PointCloud<beam_containers::PointBridge>::Ptr&
          cloud_pointbridge) const;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_lidar_frame_{
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>()};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_camera_frame_{
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>()};
  std::shared_ptr<cv::Mat> image_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_distorted_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_undistorted_;
  bool image_distorted_{true};
  bool image_initialized_{false};
  Eigen::Matrix4d T_camera_lidar_{Eigen::Matrix4d::Identity()};
};

/** @} group colorizer */

} // namespace beam_colorize
