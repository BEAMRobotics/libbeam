/** @file
 * @ingroup cv
 */

#pragma once
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

cv::Mat1f DepthInterpolation(int window_width, int window_height,
                             float threshold, int iterations,
                             cv::Mat1f depth_image);

cv::Mat1f KMeansCompletion(int K, cv::Mat rgb_image, cv::Mat1f depth_image);

} // namespace beam_cv