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
  DepthMap(std::shared_ptr<beam_calibration::CameraModel> model,
           const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
           const cv::Mat& image_input);

  virtual ~DepthMap() = default;
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
                          int dilate);

  /**
   * @brief Normalizes depth image and returns in COLORMAP_JET
   * @return cv::Mat
   */
  cv::Mat VisualizeDepthImage();

protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::shared_ptr<cv::Mat> image_;
  std::shared_ptr<cv::Mat> depth_image_;
  std::shared_ptr<beam_calibration::CameraModel> model_;
  bool point_cloud_initialized_ = false, image_initialized_ = false,
       model_initialized_ = false, depth_image_extracted_ = false;
};
} // namespace beam_cv