#include <opencv2/opencv.hpp>

#include <boost/make_shared.hpp>
#include <sensor_msgs/Image.h>

namespace beam_cv { namespace OpenCVConversions {

int DepthStrToInt(const std::string depth);

int GetCvType(const std::string& encoding);

/**
 * @brief Converts a ROS Image to a cv::Mat by sharing the data or changing
 * its endianness if needed
 * @param source ros image message
 * @return cv::Mat of image
 */
cv::Mat RosImgToMat(const sensor_msgs::Image& source);

/**
 * @brief Converts a cv::Mat to a ROS Image
 * @param source cv mat image to convert
 * @param header ros header for the new image
 * @param encoding Image encoding ("mono8", "bgr8", etc.)
 * @return ros image
 */
sensor_msgs::Image MatToRosImg(const cv::Mat source,
                               const std_msgs::Header& header,
                               const std::string& encoding);

}} // namespace beam_cv::OpenCVConversions