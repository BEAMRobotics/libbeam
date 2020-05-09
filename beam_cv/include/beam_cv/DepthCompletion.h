/** @file
 * @ingroup cv
 */

#pragma once
#include <opencv2/opencv.hpp>

namespace beam_cv {

/**
 * @brief Performs an interpolation to introduce more points in the image by
 * linear interpolating a point and a found point in the window, introducing a
 * point halfway between them
 * @param window_width size of window going from the pixel out to left/right
 * @param window_height size of window going from pixel to up/down
 * @param threshold how similar two points may be in value for them to be
 * interpolated
 * @param depth_image depth map to perform on
 */
void DepthInterpolation(int window_width, int window_height, float threshold,
                        cv::Mat& depth_image);
/**
 * @brief Performs ip basic fast completion:
 * https://arxiv.org/pdf/1802.00036.pdf
 * @param depth_image depth map to perform on
 */
void IPBasic(cv::Mat& depth_image);

/**
 * @brief Performs windowed inverse distance weighting interpolation to complete
 * depth image: : https://en.wikipedia.org/wiki/Inverse_distance_weighting
 * @param depth_image depth map to perform on
 * @param window_size size of window (square) to search for interpolation
 */
void IDWInterpolation(cv::Mat& depth_image, int window_size);

/**
 * @brief Performs multiscale interpolation and ip_basic
 * @param depth_image depth map to perform on
 */
void MultiscaleInterpolation(cv::Mat& depth_image);

} // namespace beam_cv