/** @file
 * @ingroup depth
 */

#pragma once
#include <opencv2/opencv.hpp>

namespace beam_depth {

/**
 * @brief Applies a cross shaped window to each valid depth pixel. Searches for
 * the closest point in each "prong" of the window and introduces a point
 * halfway between the two points (found point and point in the center of the
 * window)
 *
 *    |---------------------------------|
 *    |                                 |
 *    x              z               o  |
 *    |                                 |
 *    |---------------------------------|
 * x is the current pixel, o is the closest pixel in the window prong, z is
 * where the interpolated point would be placed
 * - represents 1 pixel of the windows width, | represents 1 pixel of the
 * windows height
 * this window will be applied to the left, right, up and down of
 * the pixel x
 *
 * @param window_width size of window going from the pixel out to left/right
 * @param window_height size of window going from pixel to up/down
 * metres)
 * @param depth_image depth map to perform on
 */
void DepthInterpolation(int window_width, int window_height, float threshold,
                        cv::Mat& depth_image);
/**
 * @brief Performs ip basic fast completion, for algorithm details see:
 * https://arxiv.org/pdf/1802.00036.pdf
 * @param depth_image depth map to perform on
 */
void IPBasic(cv::Mat& depth_image);

/**
 * @brief Performs windowed inverse distance weighting interpolation to complete
 * depth image, see https://en.wikipedia.org/wiki/Inverse_distance_weighting for
 * more information
 * @param depth_image depth map to perform on
 * @param window_size size of window (square) to search for interpolation
 */
void IDWInterpolation(cv::Mat& depth_image, int window_size);

/**
 * @brief Segments the depth image into 3 "ranges" of depth and applies IPBasic
 * to each
 * @param depth_image depth map to perform on
 */
void MultiscaleInterpolation(cv::Mat& depth_image);

} // namespace beam_depth