/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_calibration/CameraModel.h"
#include "beam_utils/math.hpp"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace beam_cv {
/**
 * @brief Computes the depth image based on the given point cloud and image
 * @return cv::Mat
 */
cv::Mat
    ExtractDepthMap(const cv::Mat& input_image,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                    std::shared_ptr<beam_calibration::CameraModel> input_model);

/**
 * @brief Normalizes depth image and returns in COLORMAP_JET
 * @return cv::Mat
 */
cv::Mat VisualizeDepthImage(cv::Mat depth_image);

/**
 * @brief Performs depth completion on sparse depth image
 * @return cv::Mat
 */
cv::Mat DepthCompletion(cv::Mat depth_image, cv::Mat kernel);

} // namespace beam_cv