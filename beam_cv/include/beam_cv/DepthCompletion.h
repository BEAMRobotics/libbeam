/** @file
 * @ingroup cv
 */

#pragma once
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

/**
 * @brief Performs an interpolation to introduce more points in the image
 */
cv::Mat1f DepthInterpolation(int window_width, int window_height,
                             float threshold, cv::Mat1f depth_image);

/**
 * @brief Performs a depth completion using K-means and simple averaging
 */
cv::Mat1f KMeansCompletion(int K, cv::Mat rgb_image, cv::Mat1f depth_image);

/**
 * @brief Performs depth completion using superpixels and multiple regression
 */
cv::Mat1f SuperpixelCompletion(cv::Mat rgb_image, cv::Mat1f depth_image);
} // namespace beam_cv