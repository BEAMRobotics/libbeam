/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_utils/math.hpp"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

/**
 * @brief Method for extracting skeleton of objects in scene
 * @return binary image of 1 pixel wide skeleton
 * @param input image of detected objects
 */
cv::Mat ExtractSkeleton(const cv::Mat& input_image);

/**
 * @brief Method for removing unconnected skeleton under certain pixel size
 * threshold
 * @return binary image of reduced noise skeleton image
 * @param input image and pixel threshold
 */
cv::Mat RemoveClusters(const cv::Mat& input_image, int threshold);

/**
 * @brief Method for removing holes in objects but keep the boundaries the
 * same
 * @return binary image with no holes
 * @param input image
 */
cv::Mat CloseObjects(const cv::Mat& input_image);

} // namespace beam_cv