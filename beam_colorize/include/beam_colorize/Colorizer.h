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
#include "beam_calibration/Intrinsics.h"
#include <beam_utils/log.hpp>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
 * @brief Abstract class which different colorization methods can implement
 */
class Colorizer {
public:
  Colorizer() = default;

  virtual ~Colorizer() = default;

  static std::unique_ptr<Colorizer> Create(ColorizerType type);
  /**
   * @brief Method for adding a point cloud of point type XYZ. This is required.
   * @param cloud_input Input point cloud
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
  void SetIntrinsics(beam_calibration::Intrinsics* intrinsics);

  /**
   * @brief Method for adding a transformation between the image and point
   * cloud frames.
   * @param T_C_L transfrom from lidar frame to camera frame. Default is
   * the identity (no transformation)
   */
  void SetTransform(const Eigen::Affine3d& T_C_L);

  /**
   * @brief Method for adding a the distortion condition of the image
   * @param image_distored set to true if the image is distorted, or false if
   * the image has already been undistorted. Default = true;
   */
  void SetDistortion(const bool& image_distored);

  /**
   * @brief Pure virtual method for colorizing a point cloud
   * @return Colored point cloud pointer
   */
  virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud() const = 0;

protected:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud_;
  std::shared_ptr<cv::Mat> image_;
  beam_calibration::Intrinsics* intrinsics_;
  Eigen::Affine3d T_C_L_;
  bool image_distored_, image_initialized_, point_cloud_initialized_,
      intrinsics_initialized_, transform_set_;
};

/** @} group colorizer */

} // namespace beam_colorize
