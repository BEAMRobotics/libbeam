/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_calibration/CameraModel.h"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_cv {

enum class Direction { UP = 0, DOWN, LEFT, RIGHT };

/*
 * @brief behaviour for raytrace
 */
void HitBehaviour(std::shared_ptr<cv::Mat> image,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  const int* position, int index);

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

  /***********************Getters/Setters**********************/
  /**
   * @brief Gets cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
  /**
   * @brief Sets cloud
   * @param cloud to set
   */
  void SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
  /**
   * @brief Gets depth image
   */
  std::shared_ptr<cv::Mat> GetDepthImage();
  /**
   * @brief Gets depth image
   */
  void SetDepthImage(cv::Mat input);
  /**
   * @brief Gets camera model
   */
  std::shared_ptr<beam_calibration::CameraModel> GetModel();
  /**
   * @brief Sets camera model
   * @param Camera model to set
   */
  void SetModel(std::shared_ptr<beam_calibration::CameraModel> input_model);

  /***********************Computation methods**********************/
  /**
   * @brief Computes the depth image based on the given point cloud and image
   * @return cv::Mat
   */
  void ExtractDepthMap(double threshold, int mask_size);

  /**
   * @brief Performs depth completion on sparse depth image
   * @return cv::Mat
   */
  void DepthCompletion(cv::Mat kernel);

  /**
   * @brief Performs interpolation to densify depth map
   * @return cv::Mat
   */
  void DepthInterpolation(int window_width, int window_height, float threshold,
                          int iterations);

  /*
   * @brief Creates point cloud form interpolated depth image
   * @return point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractPointCloud();

  /*
   * @brief Performs meshing but on depth image
   */
  void DepthMeshing();

  cv::Mat KMeansCompletion(int K, cv::Mat img);

  /***********************Helper Functions**********************/

  /**
   * @brief Checks if variables are properly set
   * @return bool
   */
  bool CheckState();

  /**
   * @brief Returns nearest pixel to input point with non-zero value in
   * direction
   * @return Vec2
   */
  beam::Vec2 FindClosest(beam::Vec2 search_pixel);

  /**
   * @brief Returns nearest pixel to input point with non-zero value in
   * direction
   * @return Vec2
   */
  beam::Vec2 FindClosestLeft(beam::Vec2 search_pixel, cv::Mat mesh);
  beam::Vec2 FindClosestRight(beam::Vec2 search_pixel, cv::Mat mesh);
  beam::Vec2 FindClosestUp(beam::Vec2 search_pixel, cv::Mat mesh);
  beam::Vec2 FindClosestDown(beam::Vec2 search_pixel, cv::Mat mesh);

  /**
   * @brief Returns XYZ coordinates of a pixel in the depth map
   * @return Vec3
   */
  beam::Vec3 GetXYZ(beam::Vec2 pixel);

  /**
   * @brief returns distance in world between two pixels in depth map
   * @return Vec2
   */
  float GetDistance(beam::Vec2 p1, beam::Vec2 p2);

  /***********************Member variables**********************/
protected:
  // input point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  // pointer to hold depth image
  std::shared_ptr<cv::Mat> depth_image_;
  // camera model used
  std::shared_ptr<beam_calibration::CameraModel> model_;
  // stores the min and max depth in the depth map
  double min_depth_, max_depth_;
  // verification variables
  bool point_cloud_initialized_ = false, model_initialized_ = false,
       depth_image_extracted_ = false;
  // kernels used in various operations
  std::map<int, cv::Mat> FULL_KERNEL_ = {
      {3, beam::GetFullKernel(3)},   {5, beam::GetFullKernel(5)},
      {7, beam::GetFullKernel(7)},   {9, beam::GetFullKernel(9)},
      {11, beam::GetFullKernel(11)}, {15, beam::GetFullKernel(15)},
      {21, beam::GetFullKernel(21)}, {31, beam::GetFullKernel(31)},
  };
};
} // namespace beam_cv