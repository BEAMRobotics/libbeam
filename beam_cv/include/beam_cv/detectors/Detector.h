/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

/** Representation of a generic keypoint detector
 */
class Detector {
public:
  /**
   * @brief Default constructor
   */
  Detector() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Detector() = default;

  /**
   * @brief Detects features in an image. Calls a different detector depending
   * on the derived class.
   * @param image the image to detect features in.
   * @return a vector of the detected keypoints.
   */
  virtual std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image) = 0;
};

} // namespace beam_cv
