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
 * @param window_width size of window going from the pixel out to left/right
 * @param window_height size of window going from pixel to up/down
 * @param threshold how similar two points may be in value for them to be
 * interpolated
 * @param depth_image depth map to perform on
 * @return completed depth map
 */
cv::Mat1f DepthInterpolation(int window_width, int window_height,
                             float threshold, cv::Mat1f depth_image);

/**
 * @brief Performs a depth completion using K-means and simple averaging
 * @param K number of components to segment
 * @param rgb_image color image associated to the depth image
 * @param depth_image depth map to perform on
 * @return completed depth map
 */
cv::Mat1f KMeansCompletion(int K, cv::Mat rgb_image, cv::Mat1f depth_image);

/**
 * @brief Performs ip basic fast completion
 * @param depth_img depth map to perform on
 * @return completed depth map
 */
cv::Mat1f IPBasic(cv::Mat1f depth_img);

/**
 * @brief Performs idw interpolation to complete depth image
 * @param depth_img depth map to perform on
 * @param window_size size of window (square) to search for interpolation
 * @return completed depth map
 */
cv::Mat1f IDWInterpolation(cv::Mat1f depth_img, int window_size);

/**
 * @brief Performs multiscale interpolation and ip_basic
 * @param depth_img depth map to perform on
 * @return completed depth map
 */
cv::Mat1f MultiscaleInterpolation(cv::Mat1f depth_img);

} // namespace beam_cv