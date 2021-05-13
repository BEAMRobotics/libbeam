/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <beam_utils/utils.h>

namespace beam_cv {

/**
 * @brief Enum class for different types descriptors
 */
enum class DescriptorType { ORB = 0, SIFT, BRISK };

// Map for storing string input
static std::map<std::string, DescriptorType> DescriptorTypeStringMap = {
    {"ORB", DescriptorType::ORB},
    {"SIFT", DescriptorType::SIFT},
    {"BRISK", DescriptorType::BRISK}};

// Map for storing int input
static std::map<uint8_t, DescriptorType> DescriptorTypeIntMap = {
    {0, DescriptorType::ORB},
    {1, DescriptorType::SIFT},
    {2, DescriptorType::BRISK}};

/** Representation of a generic keypoint descriptor extractor.
 */
class Descriptor {
public:
  /**
   * @brief Default constructor
   */
  Descriptor() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Descriptor() = default;

  /** Extracts keypoint descriptors from an image. Calls a different extractor
   *  depending on the derived class.
   *  @param image the image to extract keypoints from.
   *  @param keypoints the keypoints detected in the image.
   *  @return descriptors, the computed keypoint descriptors.
   */
  virtual cv::Mat ExtractDescriptors(const cv::Mat& image,
                                     std::vector<cv::KeyPoint>& keypoints) = 0;
};

} // namespace beam_cv
