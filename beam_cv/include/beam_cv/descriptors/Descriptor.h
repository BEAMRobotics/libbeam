/** @file
 * @ingroup cv
 */

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <beam_utils/utils.h>
#include <beam_utils/log.h>

namespace beam_cv {

/**
 * @brief Enum class for different types descriptors
 */
enum class DescriptorType { ORB = 0, SIFT, BRISK, BEBLID };

// vector for storing binary descriptor types
static std::vector<DescriptorType> BinaryDescriptorTypes = {
    DescriptorType::ORB, DescriptorType::BRISK, DescriptorType::BEBLID};

// Map for storing string input
static std::map<std::string, DescriptorType> DescriptorTypeStringMap = {
    {"ORB", DescriptorType::ORB},
    {"SIFT", DescriptorType::SIFT},
    {"BRISK", DescriptorType::BRISK},
    {"BEBLID", DescriptorType::BEBLID}};

// Map for storing int input
static std::map<uint8_t, DescriptorType> DescriptorTypeIntMap = {
    {0, DescriptorType::ORB},
    {1, DescriptorType::SIFT},
    {2, DescriptorType::BRISK},
    {3, DescriptorType::BEBLID}};

// function for listing types of Descriptor available
inline std::string GetDescriptorTypes() {
  std::string types;
  for (auto it = DescriptorTypeStringMap.begin();
       it != DescriptorTypeStringMap.end(); it++) {
    types += it->first;
    types += ", ";
  }
  types.erase(types.end() - 2, types.end());
  return types;
}

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

  /** @brief Factory method to create Descriptor at runtime */
  static std::shared_ptr<Descriptor> Create(DescriptorType type,
                                            const std::string& file_path = "");

  /** @brief Extracts keypoint descriptors from an image. Calls a different
   * extractor depending on the derived class.
   *  @param image the image to extract keypoints from.
   *  @param keypoints the keypoints detected in the image. NOTE: if the
   * descriptor has some criteria to validate the descriptors (e.g., ORB), it
   * may only keep the keypoints that have valid descriptors. Therefore the
   * keypoints vector size may change.
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
    } else {
      BEAM_ERROR("cannot create descriptor which is not of binary type.");
    }
    return descriptor;
  }
};

} // namespace beam_cv
