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
 * @param image image for casting into cloud
 * @param cloud cloud for casting
 * @param hit_mask hit mask to speed up ray tracing algorithm
 * @param threshold threshold to determine ray contact
 * @param model camera model
 * @param function that determines the hit behaviour
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
 * @param image image for casting into cloud
 * @param cloud cloud for casting
 * @param hit_mask hit mask to speed up ray tracing algorithm
 * @param threshold threshold to determine ray contact
 * @param model camera model
 * @param function that determines the hit behaviour
 */
void RayCastXYZRGB(std::shared_ptr<cv::Mat> image,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   cv::Mat hit_mask, float threshold,
                   std::shared_ptr<beam_calibration::CameraModel> model,
                   void (*f)(std::shared_ptr<cv::Mat> image_i,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_i,
                             const int* position, int index));

/**
 * @brief Creates hit mask to speed up ray casting
 * @param mask_size size of mask used for dilating hit mask
 * @param model camera model used for projecting points
 * @param cloud cloud to use for hit detection
 * @return cv::Mat
 */
cv::Mat CreateHitMask(int mask_size,
                      std::shared_ptr<beam_calibration::CameraModel> model,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
} // namespace beam_cv