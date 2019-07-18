/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_calibration/CameraModel.h"
#include "beam_utils/math.hpp"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_cv {

class DepthMap {
public:
  /**
   * @brief Cosntructor to intialize member variables
   */
  DepthMap(cv::Mat input_image, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
           std::shared_ptr<beam_calibration::CameraModel> input_model);

  /**
   * @brief Default destructor
   */
  virtual ~DepthMap() = default;

  /**
   * @brief Computes the depth image based on the given point cloud and image
   * @return cv::Mat of 3 channels (x,y,z)
   */
  cv::Mat ExtractDepthMap();

protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
  std::shared_ptr<cv::Mat> image_;
  std::shared_ptr<cv::Mat> depth_image_;
  std::shared_ptr<beam_calibration::CameraModel> cam_model_;
};

} // namespace beam_cv