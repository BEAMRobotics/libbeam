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

// Map for storing binary descriptor types
static std::vector<DescriptorType> BinaryDescriptorTypes = {
    DescriptorType::ORB, DescriptorType::BRISK};
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

  /** @brief Extracts keypoint descriptors from an image. Calls a different
   * extractor depending on the derived class.
   *  @param image the image to extract keypoints from.
   *  @param keypoints the keypoints detected in the image.
   *  @return descriptors, the computed keypoint descriptors.
   */
  virtual cv::Mat ExtractDescriptors(const cv::Mat& image,
                                     std::vector<cv::KeyPoint>& keypoints) = 0;

  /** @brief Creates a single descriptor from a vector of floats in the desired
   * descriptor types encoding
   *  @param data raw descriptor data stored as floats
   *  @param type desired descriptor type
   *  @return cv mat of the descriptor in its associated encoding
   */
  static cv::Mat CreateDescriptor(std::vector<float> data,
                                  DescriptorType type) {
    cv::Mat descriptor(1, data.size(), CV_32FC1, data.data());
    // if the type is a binary type then convert to uint8
    if (std::find(BinaryDescriptorTypes.begin(), BinaryDescriptorTypes.end(),
                  type) != BinaryDescriptorTypes.end()) {
      descriptor.convertTo(descriptor, CV_8U);
    }
    return descriptor;
  }

};

} // namespace beam_cv
