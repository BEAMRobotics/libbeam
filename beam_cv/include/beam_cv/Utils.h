/** @file
 * @ingroup cv
 */

#pragma once
#include <opencv2/opencv.hpp>

#include "beam_utils/math.hpp"

namespace beam_cv {

/**
 * @brief Method to perform histogram equalization
 * @param input image
 * @return histogram equalized image
 */
cv::Mat AdaptiveHistogram(const cv::Mat&);

/**
 * @brief Method for visualizing depth image
 * @return image with colormap jet
 * @param input image
 */
cv::Mat VisualizeDepthImage(const cv::Mat&);

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
 * @return Vec2
 */
beam::Vec2 FindClosest(beam::Vec2 search_pixel, const cv::Mat& depth_image);

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
void SaveDepthImageBW(const cv::Mat& depth_image, std::string path);

} // namespace beam_cv