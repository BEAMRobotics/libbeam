/** @file
 * @ingroup cv
 */

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <beam_utils/log.h>

namespace beam_cv {

/**
 * @brief Enum class for different types of detectors
 */
enum class DetectorType { ORB = 0, SIFT, FAST, GFTT };

// Map for storing string input
static std::map<std::string, DetectorType> DetectorTypeStringMap = {
    {"ORB", DetectorType::ORB},
    {"SIFT", DetectorType::SIFT},
    {"FAST", DetectorType::FAST},
    {"GFTT", DetectorType::GFTT}};

// Map for storing int input
static std::map<uint8_t, DetectorType> DetectorTypeIntMap = {
    {0, DetectorType::ORB},
    {1, DetectorType::SIFT},
    {2, DetectorType::FAST},
    {3, DetectorType::GFTT}};

// function for listing types of detectors available
inline std::string GetDetectorTypes() {
  std::string types;
  for (auto it = DetectorTypeStringMap.begin();
       it != DetectorTypeStringMap.end(); it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

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

  /** @brief Factory method to create detector at runtime */
  static std::shared_ptr<Detector> Create(DetectorType type,
                                          const std::string& file_path = "");

  /** Detects features in an image. Calls a different detector depending on
   *  the derived class.
   *  @param image the image to detect features in.
   *  @return a vector of the detected keypoints.
   */
  virtual std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image) = 0;
};

} // namespace beam_cv
