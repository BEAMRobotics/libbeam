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
  cv::Mat GetDepthImage();
  /**
   * @brief Gets depth image
   */
  void SetDepthImage(cv::Mat1d input);
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
   * @return number of points extracted
   */
  int ExtractDepthMap(double threshold, int mask_size);

  /**
   * @brief Performs interpolation to densify depth map
   * @return number of points interpolated
   */
  int DepthInterpolation(int window_width, int window_height, float threshold,
                         int iterations);

  /*
   * @brief Performs Completiion using k means
   * @param K: number of segments, img: color image
   */
  void KMeansCompletion(int K, cv::Mat img);

  /*
   * @brief Creates point cloud form interpolated depth image
   * @return point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractPointCloud();

  /***********************Helper Functions**********************/

  /**
   * @brief Checks if variables are properly set
   * @return bool
   */
  bool CheckState();

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
};
} // namespace beam_cv