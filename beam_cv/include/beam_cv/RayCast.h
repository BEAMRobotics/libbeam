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

/**
 * @brief Performs ray casting with custom behaviour on hit
 */
void RayCastXYZ(std::shared_ptr<cv::Mat> image,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat hit_mask,
                float threshold,
                std::shared_ptr<beam_calibration::CameraModel> model,
                void (*f)(std::shared_ptr<cv::Mat> image_i,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i,
                          const int* position, int index));
/**
 * @brief Performs ray casting with custom behaviour on hit
 */
void RayCastXYZRGB(std::shared_ptr<cv::Mat> image,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   cv::Mat hit_mask, float threshold,
                   std::shared_ptr<beam_calibration::CameraModel> model,
                   void (*f)(std::shared_ptr<cv::Mat> image_i,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_i,
                             const int* position, int index));
} // namespace beam_cv