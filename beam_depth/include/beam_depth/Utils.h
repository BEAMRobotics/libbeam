/** @file
 * @ingroup cv
 */

#pragma once
#include <opencv2/opencv.hpp>

#include <beam_utils/math.h>

namespace beam_depth {

/**
 * @brief Method for visualizing depth image
 * @return image with colormap jet
 * @param input image
 */
cv::Mat VisualizeDepthImage(const cv::Mat&);

/**
 * @brief returns closest non zero pixel in depth image
 * @param search_pixel u,v search pixel
 * @param depth_image depth image to search
 * @return Eigen::Vector2i
 */
Eigen::Vector2i FindClosest(const Eigen::Vector2i& search_pixel,
                            const cv::Mat& depth_image);

/**
 * @brief Computes distance between two pixels
 * @param p1 pixel one
 * @param p2 pixel two
 * @return double
 */
double PixelDistance(cv::Point2i p1, cv::Point2i p2);

/**
 * @brief returns closest non zero pixel
 * @param search_pixel u,v search pixel
 * @param depth_image depth image to search
 * @return Eigen::Vector2i
 */
Eigen::Vector2i FindClosest(const Eigen::Vector2i& search_pixel,
                            const cv::Mat& depth_image);

/**
 * @brief returns vector of mats where each mat represents a certain depth range
 * of the input depth image
 * @param depth_image depth image to segment
 * @return Vector of mats
 */
std::vector<cv::Mat> SegmentMultiscale(const cv::Mat& depth_image);

/**
 * @brief saves a depth image as grayscale to path
 * @param depth_image depth image to segment
 * @param path to save image at
 */
void SaveDepthImageBW(const cv::Mat& depth_image, const std::string& path);

} // namespace beam_depth
