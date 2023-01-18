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
enum class DetectorType { ORB = 0, SIFT, FAST, FASTSSC, GFTT };

namespace internal {
// Map for storing string input
static std::map<std::string, DetectorType> DetectorStringTypeMap = {
    {"ORB", DetectorType::ORB},
    {"SIFT", DetectorType::SIFT},
    {"FAST", DetectorType::FAST},
    {"FASTSSC", DetectorType::FASTSSC},
    {"GFTT", DetectorType::GFTT}};

const std::map<DetectorType, std::string> DetectorTypeStringMap = {
    {DetectorType::ORB, "ORB"},
    {DetectorType::SIFT, "SIFT"},
    {DetectorType::FAST, "FAST"},
    {DetectorType::FASTSSC, "FASTSSC"},
    {DetectorType::GFTT, "GFTT"}};

} // namespace internal

// function for listing types of detectors available
inline std::string GetDetectorTypes() {
  std::string types;
  for (auto it = internal::DetectorTypeStringMap.begin();
       it != internal::DetectorTypeStringMap.end(); it++) {
    types += it->second;
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
  Detector(int grid_cols, int grid_rows);

  /**
   * @brief Default destructor
   */
  virtual ~Detector() = default;

  /** @brief Factory method to create detector at runtime */
  static std::shared_ptr<Detector> Create(DetectorType type,
                                          const std::string& file_path = "");

  /** @brief Gridded feature keypoint detection. Calls DetectLocalFeatures
   * defined in each derived class.  Returns a max of num_features_ keypoints.
   *  @param image the image to detect features in.
   *  @return a vector of the detected keypoints.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

  /** @brief Gets the string representation of the type of descriptor
   *  @return string of descriptor type
   */
  virtual std::string GetTypeString() const = 0;

  /** @brief Gets the type of descriptor
   *  @return descriptor type
   */
  virtual DetectorType GetType() const = 0;

private:
  /** @brief Detects keypoints in an image/grid space. Calls a different
   * detector depending on the derived class.  Returns a max of num_features
   * divided by grid spaces keypoints
   *  @param image the image to detect features in.
   *  @return a vector of the detected keypoints.
   */
  virtual std::vector<cv::KeyPoint>
      DetectLocalFeatures(const cv::Mat& image) = 0;

  int grid_cols_ = 3;
  int grid_rows_ = 2;
};

} // namespace beam_cv
