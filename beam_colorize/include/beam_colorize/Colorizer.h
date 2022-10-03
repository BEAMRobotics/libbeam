/** @file
 * @ingroup colorizer
 * Includes all colorization classes / functions
 *
 * @defgroup colorizer
 * Colorization classes / functions
 */

#pragma once

#include <boost/make_shared.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>

#include <beam_calibration/CameraModel.h>
#include <beam_containers/PointBridge.h>
#include <beam_utils/pointclouds.h>

namespace beam_colorize {
/** @addtogroup colorizer
 *  @{ */

/**
 * @brief Enum class for different types of intrinsic calibrations
 */
enum class ColorizerType {
  PROJECTION = 0,
  PROJECTION_OCCLUSION_SAFE,
  RAY_TRACE
};

/**
 * @brief Pixel type for iterating through the image
 */
using Pixel = cv::Point3_<uchar>;

/**
 * @brief alias for defect clouds
 */
typedef pcl::PointCloud<beam_containers::PointBridge> DefectCloud;

struct ProjectedPointMeta {
  ProjectedPointMeta(uint64_t _id, double _depth) : id(_id), depth(_depth) {}

  uint64_t id;  // id in original cloud
  double depth; // depth from camera in m
};

using UMapType = std::unordered_map<uint64_t, ProjectedPointMeta>;

/**
 * @brief class for storing a point projection map. This is stored as a 2D (or
 * two level nested) hash map so we can lookup point IDs associated with image
 * pixel coordinates (u,v). Note that since we only want to colorize closest
 * points to the image, we only store the closest points to each individual
 * pixel
 *
 */
class ProjectionMap {
public:
  void Add(uint64_t u, uint64_t v, uint64_t point_id, double depth = 0);

  bool Get(uint64_t u, uint64_t v, ProjectedPointMeta& point_meta);

  void Erase(uint64_t u, uint64_t v);

  int Size();

  std::unordered_map<uint64_t, UMapType>::iterator VBegin();

  std::unordered_map<uint64_t, UMapType>::iterator VEnd();

private:
  // map: v -> {map: u -> closest point ID}
  std::unordered_map<uint64_t, UMapType> map_;

  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
};

/**
 * @brief Abstract class which different colorization methods can implement
 */
class Colorizer {
public:
  Colorizer();

  virtual ~Colorizer() = default;

  static std::unique_ptr<Colorizer> Create(ColorizerType type);

  /**
   * @brief Pure virtual method for generating a projection map given a colored
   * pointcloud
   */
  virtual ProjectionMap CreateProjectionMap(
      const PointCloudCol::Ptr& cloud_in_camera_frame) const = 0;

  /**
   * @brief Pure virtual method for generating a projection map given a defect
   * pointcloud
   */
  virtual ProjectionMap CreateProjectionMap(
      const DefectCloud::Ptr& cloud_in_camera_frame) const = 0;

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
   * @brief Method for adding a the distortion condition of the image
   * @param image_distored set to true if the image is distorted, or false if
   * the image has already been undistorted. Default = true;
   */
  void SetDistortion(const bool& image_distored);

  /**
   * @brief colors map with RGB information from the stored image given a
   * reference to a defect cloud
   */
  void ColorizePointCloud(DefectCloud::Ptr& cloud_in_camera_frame) const;

  /**
   * @brief colors map with RGB information from the stored image given a const
   * pointer to an XYZ cloud
   * @return colored cloud in camera frame
   */
  PointCloudCol::Ptr
      ColorizePointCloud(const PointCloud::Ptr& cloud_in_camera_frame) const;

  /**
   * @brief colors map with RGB information from the stored image given a const
   * pointer to a colored cloud
   * @return colored cloud in camera frame 
   */
  PointCloudCol::Ptr
      ColorizePointCloud(const PointCloudCol::Ptr& cloud_in_camera_frame) const;

  /**
   * @brief labels map with defect information from the stored image (it must be
   * a mask) given a reference to a defect cloud cloud
   */
  void ColorizeMask(DefectCloud::Ptr& cloud_in_camera_frame) const;

  /**
   * @brief labels map with defect information from the stored image (it must be
   * a mask) given a const pointer to an XYZ cloud
   * @return defect cloud in camera frame
   */
  DefectCloud::Ptr
      ColorizeMask(const PointCloud::Ptr& cloud_in_camera_frame) const;

  /**
   * @brief labels map with defect information from the stored image (it must be
   * a mask) given a const pointer to a colored cloud.
   * @return defect cloud in camera frame
   */
  DefectCloud::Ptr
      ColorizeMask(const PointCloudCol::Ptr& cloud_in_camera_frame) const;

protected:
  std::shared_ptr<cv::Mat> image_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_distorted_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_undistorted_;
  bool image_distorted_{true};
  bool image_initialized_{false};
};

/** @} group colorizer */

} // namespace beam_colorize
