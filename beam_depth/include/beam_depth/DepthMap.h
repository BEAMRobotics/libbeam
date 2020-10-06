/** @file
 * @ingroup depth
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <beam_calibration/CameraModel.h>

namespace beam_depth {

class DepthMap {
public:
  /**
   * @brief Default constructor
   */
  DepthMap() = default;

  /**
   * @brief Custom constructor
   */
  DepthMap(std::shared_ptr<beam_calibration::CameraModel> model,
           const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

  /**
   * @brief Default destructor
   */
  ~DepthMap() = default;

  /**
   * @brief Gets point cloud member attribute
   * @return Returns point to XYZ point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
  /**
   * @brief Sets cloud point cloud attribute
   * @param input_cloud point cloud to set
   */
  void SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

  /**
   * @brief Gets depth image attribe
   * @return Returns cv::mat representing depth image
   */
  cv::Mat GetDepthImage();
  /**
   * @brief Sets depth image attribute
   * @param input depth image to set
   */
  void SetDepthImage(cv::Mat input);
  /**
   * @brief Gets camera model
   * @return Returns a pointer to a camera model
   */
  std::shared_ptr<beam_calibration::CameraModel> GetCameraModel();
  /**
   * @brief Sets camera model
   * @param Camera model to set
   */
  void SetCameraModel(
      std::shared_ptr<beam_calibration::CameraModel> input_model);

  /**
   * @brief Computes the depth image based on the given point cloud and image
   * @param threshold threshold value to be used to determine hit detection of
   * ray cast
   * @param mask_size used as input for beam_cv::CreateHitMask
   * @return number of points extracted
   */
  int ExtractDepthMap(float threshold, int mask_size);

  /**
   * @brief Computes the depth image based on the given point cloud and image
   * using projection over ray casting
   * @param thresh depth threshold to limit points being projected
   * @return number of points extracted
   */
  int ExtractDepthMapProjection(float thresh);

  /*
   * @brief Creates point cloud from interpolated depth image
   * @return point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractPointCloud();

  /**
   * @brief Checks if variables (point_cloud_initialized_,
   * depth_image_extracted_ model_initialized_) are properly set
   * @return bool
   */
  bool CheckState();

  /**
   * @brief Returns XYZ coordinates of a pixel in the depth map
   * @param pixel u,v pixel of image
   * @return x,y,z coordinates
   */
  Eigen::Vector3d GetXYZ(const Eigen::Vector2i& pixel);

  /**
   * @brief returns distance in world between two pixels in depth map
   * @param p1 pixel input one
   * @param p2 pixel input two
   * @return Vec2
   */
  float GetDistance(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2);

  /**
   * @brief returns area of a pixel in world scale
   * @param pixel
   * @return float
   */
  float GetPixelScale(const Eigen::Vector2i& pixel);

  /**
   * @brief returns area of a pixel in world scale
   * @param pixel
   * @return float
   */
  void Subsample(const float percentage_drop);

protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::shared_ptr<cv::Mat> depth_image_;
  std::shared_ptr<beam_calibration::CameraModel> model_;
  float min_depth_, max_depth_;
  bool point_cloud_initialized_ = false, model_initialized_ = false,
       depth_image_extracted_ = false;
};
} // namespace beam_depth
