/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_calibration/CameraModel.h"
#include "beam_cv/Morphology.h"
#include "beam_utils/math.hpp"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_cv {
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
           const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
           const cv::Mat& image_input);

  /**
   * @brief Default destructor
   */
  virtual ~DepthMap() = default;

  /***********************Getters/Setters**********************/
  /**
   * @brief Gets cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
  /**
   * @brief Sets cloud
   */
  void SetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
  /**
   * @brief Gets cloud
   */
  std::shared_ptr<cv::Mat> GetDepthImage();
  /**
   * @brief Sets cloud
   */
  void SetImage(const cv::Mat& image_input);
  /**
   * @brief Sets cloud
   */
  std::shared_ptr<cv::Mat> GetImage();
  /**
   * @brief Gets cloud
   */
  std::shared_ptr<beam_calibration::CameraModel> GetModel();
  /**
   * @brief Sets cloud
   */
  void SetModel(std::shared_ptr<beam_calibration::CameraModel> input_model);

  /***********************Computation methods**********************/
  /**
   * @brief Computes the depth image based on the given point cloud and image
   * @return cv::Mat
   */
  void ExtractDepthMap(double threshold, int dilate, int mask_size);

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
                          int dilate, int iterations);

  /*
   * @brief Creates point cloud form interpolated depth image
   * @return point cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractPointCloud();

  /***********************Helper Functions**********************/

  /*
   * @brief Creates hit mask to speed up ray casting
   * @return cv::Mat
   */
  cv::Mat CreateHitMask(int mask_size);

  /**
   * @brief Normalizes depth image and returns in COLORMAP_JET
   * @return cv::Mat
   */
  cv::Mat VisualizeDepthImage();

  /**
   * @brief Checks if variables are properly set
   * @return bool
   */
  bool CheckState();

  /***********************Member variables**********************/
protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::shared_ptr<cv::Mat> image_;
  std::shared_ptr<cv::Mat> depth_image_;
  std::shared_ptr<beam_calibration::CameraModel> model_;
  double min_depth_, max_depth_;
  bool point_cloud_initialized_ = false, image_initialized_ = false,
       model_initialized_ = false, depth_image_extracted_ = false;
  cv::Mat full_k_5 = beam::GetFullKernel(5), full_k_9 = beam::GetFullKernel(9),
          full_k_15 = beam::GetFullKernel(15),
          full_k_21 = beam::GetFullKernel(21);
};
} // namespace beam_cv