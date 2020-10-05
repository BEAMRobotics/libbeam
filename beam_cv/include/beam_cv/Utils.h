/** @file
 * @ingroup cv
 */

#pragma once
#include <opencv2/opencv.hpp>

#include <beam_utils/math.hpp>

namespace beam_cv {

/**
 * @brief Method to perform histogram equalization
 * @param input image
 * @return histogram equalized image
 */
cv::Mat AdaptiveHistogram(const cv::Mat&);

/**
 * @brief Method for performing k means on image
 * @return k mean image
 * @param input image
 * @param K number of clusters
 */
cv::Mat KMeans(const cv::Mat&, int);

/**
 * @brief Method for extracting skeleton of objects in scene
 * @return binary image of 1 pixel wide skeleton
 * @param input_image binary image of detected objects (e.g. crack mask)
 */
cv::Mat ExtractSkeleton(const cv::Mat& input_image);

/**
 * @brief Method for removing unconnected components below certain threshold
 * size threshold
 * @return binary image of reduced noise skeleton image
 * @param input_image binary image
 * @param threshold size threshold for removing pixel clusters
 */
cv::Mat RemoveClusters(const cv::Mat& input_image, int threshold);

/**
 * @brief Method for segmenting connected components into seperate mat objects
 * @return vector of mat objects, each represeting a different connected
 *         component
 * @param image a binary image of conncected components (e.g. cracks)
 */
std::vector<cv::Mat> SegmentComponents(const cv::Mat& image);

/**
 * @brief Finds connected components for grayscale image of arbitrary color
 * depth
 * @return vector of sets
 * @param image
 */
std::map<int, std::vector<cv::Point2i>>
    ConnectedComponents(const cv::Mat& image);

/**
 * @brief returns closest non zero pixel
 * @param search_pixel u,v search pixel
 * @param depth_image depth image to search
 * @return Eigen::Vector2i
 */
Eigen::Vector2i FindClosest(const Eigen::Vector2i& search_pixel,
                            const cv::Mat& depth_image);

} // namespace beam_cv
