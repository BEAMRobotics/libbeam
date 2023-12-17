#pragma once

#include <opencv2/opencv.hpp>

#include <boost/make_shared.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace beam_cv { namespace OpenCVConversions {

int DepthStrToInt(const std::string depth);

int GetCvType(const std::string& encoding);

/** @brief maps image encoding string found in ros Image message, to it's
 * debayered counterpart */
static std::unordered_map<std::string, std::string> bayer_decoding_map{
    {"bayer_bggr16", "bgr16"}, {"bayer_bggr8", "bgr8"},
    {"bayer_gbrg16", "bgr16"}, {"bayer_gbrg8", "bgr8"},
    {"bayer_grbg16", "bgr16"}, {"bayer_grbg8", "bgr8"},
    {"bayer_rggb16", "bgr16"}, {"bayer_rggb8", "bgr8"}};

/**
 * @brief Converts a ROS Image to a cv::Mat by sharing the data or changing
 * its endianness if needed. This will also debayer the image if necessary
 * @param source ros image message
 * @return cv::Mat of image
 */
cv::Mat RosImgToMat(const sensor_msgs::Image& source);

/**
 * @brief Converts a cv::Mat to a ROS Image
 * @param source cv mat image to convert
 * @param header ros header for the new image
 * @param encoding Image encoding ("mono8", "bgr8", etc.) See:
 * http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
 * @return ros image
 */
sensor_msgs::Image MatToRosImg(const cv::Mat source,
                               const std_msgs::Header& header,
                               const std::string& encoding);
/**
 * @brief Debayers an image according to its encoding
 * @param source cv mat image to debayer
 * @param encoding Image encoding ("mono8", "bgr8", etc.) See:
 * http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
 * @return cv mat
 */
cv::Mat Debayer(const cv::Mat& source, const std::string& encoding);

}} // namespace beam_cv::OpenCVConversions