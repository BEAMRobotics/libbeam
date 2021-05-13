/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

/**
 * @brief Enum class for different types of detectors
 */
enum class DetectorType { ORB = 0, SIFT, FAST };

  // Map for storing string input
std::map<std::string, DetectorType> DetectorTypeStringMap = {
    {"ORB", DetectorType::ORB},
    {"SIFT", DetectorType::SIFT},
    {"FAST", DetectorType::FAST}};

// Map for storing int input
std::map<uint8_t, DetectorType> DetectorTypeIntMap = {
    {0, DetectorType::ORB},
    {1, DetectorType::SIFT},
    {2, DetectorType::FAST}};

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

  /** Detects features in an image. Calls a different detector depending on
   *  the derived class.
   *  @param image the image to detect features in.
   *  @return a vector of the detected keypoints.
   */
  virtual std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image) = 0;
};

} // namespace beam_cv
