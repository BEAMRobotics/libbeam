#include <opencv2/opencv.hpp>

#include <boost/make_shared.hpp>
#include <boost/regex.hpp>
#include <sensor_msgs/Image.h>

namespace beam_cv {

class OpenCVConversions {
public:
  // ImgToMat helper
  static int DepthStrToInt(const std::string depth);

  // ImgToMat helper
  static int GetCvType(const std::string& encoding);

  /**
   * @brief Converts a ROS Image to a cv::Mat by sharing the data or changing
   * its endianness if needed
   * @param source ros image message
   * @return cv::Mat of image
   */
  static cv::Mat ImgToMat(const sensor_msgs::Image& source);
};
} // namespace beam_cv