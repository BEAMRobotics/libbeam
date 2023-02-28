/** @file
 * @ingroup cv
 */

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <beam_utils/log.h>
#include <beam_utils/optional.h>
#include <beam_utils/time.h>

namespace beam_cv {

/**
 * @brief Enum class for different types descriptors
 */
enum class DescriptorType { ORB = 0, SIFT, BRISK, BEBLID };

namespace internal {
// Map for storing string input
const std::map<std::string, DescriptorType> DescriptorStringTypeMap = {
    {"ORB", DescriptorType::ORB},
    {"SIFT", DescriptorType::SIFT},
    {"BRISK", DescriptorType::BRISK},
    {"BEBLID", DescriptorType::BEBLID}};

const std::map<DescriptorType, std::string> DescriptorTypeStringMap = {
    {DescriptorType::ORB, "ORB"},
    {DescriptorType::SIFT, "SIFT"},
    {DescriptorType::BRISK, "BRISK"},
    {DescriptorType::BEBLID, "BEBLID"}};

// vector for storing binary descriptor types
const std::vector<DescriptorType> BinaryDescriptorTypes = {
    DescriptorType::ORB, DescriptorType::BRISK, DescriptorType::BEBLID};

} // namespace internal

// function for listing types of Descriptor available
inline std::string GetDescriptorTypes() {
  std::string types;
  for (const auto& [string, type] : internal::DescriptorStringTypeMap) {
    types += string + ", ";
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
  virtual cv::Mat
      ExtractDescriptors(const cv::Mat& image,
                         std::vector<cv::KeyPoint>& keypoints) const = 0;

  /** @brief Gets the string representation of the type of descriptor
   *  @return string of descriptor type
   */
  virtual std::string GetTypeString() const = 0;

  /** @brief Gets the type of descriptor
   *  @return descriptor type
   */
  virtual DescriptorType GetType() const = 0;

  /** @brief Converts a string to the corresponding descritpor type if it exist
   *  @param type desired descriptor type as a string
   *  @return descriptor type
   */
  static beam::opt<DescriptorType>
      StringToDescriptorType(const std::string& type_str) {
    if (internal::DescriptorStringTypeMap.find(type_str) !=
        internal::DescriptorStringTypeMap.end()) {
      return internal::DescriptorStringTypeMap.at(type_str);
    } else {
      BEAM_ERROR("Invalid descriptor type, valid types: \n{}",
                 GetDescriptorTypes());
    }
    return {};
  }

  /** @brief Creates a single descriptor from a vector of floats in the
   * desired descriptor types encoding
   *  @param data raw descriptor data stored as floats
   *  @param type desired descriptor type
   *  @return cv mat of the descriptor in its associated encoding
   */
  static cv::Mat VectorDescriptorToCvMat(std::vector<float> data,
                                         const DescriptorType type) {
    cv::Mat descriptor(1, data.size(), CV_32FC1, data.data());
    // if the type is a binary type then convert to uint8
    if (std::find(internal::BinaryDescriptorTypes.begin(),
                  internal::BinaryDescriptorTypes.end(),
                  type) != internal::BinaryDescriptorTypes.end()) {
      descriptor.convertTo(descriptor, CV_8U);
    }
    return descriptor;
  }

  /** @brief Creates a single descriptor from a vector of floats in the
   * desired descriptor types encoding (binary or float)
   *  @param data raw descriptor data stored as floats
   *  @param type desired descriptor type in string format
   *  @return cv mat of the descriptor in its associated encoding
   */
  static cv::Mat VectorDescriptorToCvMat(std::vector<float> data,
                                         const std::string& type_str) {
    const auto type = StringToDescriptorType(type_str);
    if (type.has_value()) {
      return VectorDescriptorToCvMat(data, type.value());
    }
    return {};
  }

  /** @brief Converts a descriptor into a float vector using the specified tytpe
   *  @param descriptor opencv mat representing the descriptor
   *  @param type desired descriptor type
   *  @return cv mat of the descriptor in its associated encoding
   */
  static std::vector<float> CvMatDescriptorToVector(cv::Mat descriptor,
                                                    const DescriptorType type) {
    // if the type is a binary type then convert it to a float
    if (std::find(internal::BinaryDescriptorTypes.begin(),
                  internal::BinaryDescriptorTypes.end(),
                  type) == internal::BinaryDescriptorTypes.end()) {
      descriptor.convertTo(descriptor, CV_32FC1);
    }

    // convert mat to std vector
    std::vector<float> descriptor_v;
    if (descriptor.isContinuous()) {
      descriptor_v.assign(descriptor.data,
                          descriptor.data +
                              descriptor.total() * descriptor.channels());
    } else {
      BEAM_ERROR("Invalid descriptor, must be a continuous vector.");
      throw std::logic_error{
          "Invalid descriptor, must be a continuous vector."};
    }
    return descriptor_v;
  }

  /** @brief Converts a descriptor into a float vector using the specified
  type
   *  @param descriptor opencv mat representing the descriptor
   *  @param type desired descriptor type
   *  @return cv mat of the descriptor in its associated encoding
   */
  static std::vector<float>
      CvMatDescriptorToVector(cv::Mat descriptor, const std::string& type_str) {
    const auto type = StringToDescriptorType(type_str);
    if (type.has_value()) {
      return CvMatDescriptorToVector(descriptor, type.value());
    }
    return {};
  }
};

} // namespace beam_cv
