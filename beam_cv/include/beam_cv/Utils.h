/** @file
 * @ingroup cv
 */

#pragma once
// OpenCV
#include "beam_utils/math.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

/**
 * @brief Method for perform histogram equalization
 * @return equalized image
 * @param input image
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
 */
cv::Mat KMeans(const cv::Mat&, int);

/**
 * @brief Method for performing k means on image
 * @return k mean image
 * @param input image
 */
cv::Mat HierarchicalCluster(const cv::Mat& input);

/**
 * @brief Method for extracting skeleton of objects in scene
 * @return binary image of 1 pixel wide skeleton
 * @param input image of detected objects
 */
cv::Mat ExtractSkeleton(const cv::Mat& input_image);

/**
 * @brief Method for removing unconnected components
 * threshold
 * @return binary image of reduced noise skeleton image
 * @param input image and pixel threshold
 */
cv::Mat RemoveClusters(const cv::Mat& input_image, int threshold);

/**
 * @brief Method for segmenting connected components into seperate mats
 * @return vector of cracks
 * @param skeleton image of cracks
 * @param original binary image of all cracks
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
 * @brief Fits plane to set of points
 * @return centroid:normal
 * @param vector of points
 */
std::pair<beam::Vec3, beam::Vec3> FitPlane(const std::vector<beam::Vec3>& c);

/**
 * @brief Computes intersection point of line and plane
 * @return {x,y,z}
 */
beam::Vec3 IntersectPoint(beam::Vec3 ray_vector, beam::Vec3 ray_point,
                          beam::Vec3 plane_normal, beam::Vec3 plane_point);

/**
 * @brief Computes distance between two pixels
 * @return double
 */
double PixelDistance(cv::Point2i p1, cv::Point2i p2);
} // namespace beam_cv