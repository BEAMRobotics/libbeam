/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/models/SuperPointModel.h>

namespace beam_cv {

/**
 * Add description
 */
class SuperPointDescriptor : public Descriptor {
public:
  /**
   * @brief Constructor
   * @param model pointer to the superpoint model
   */
  SuperPointDescriptor(const std::shared_ptr<SuperPointModel>& model);

  /**
   * @brief Default destructor
   */
  ~SuperPointDescriptor() override = default;

  /**
   * @brief Extracts descriptors from the keypoints in an image, using the
   * SuperPoint descriptor extractor.
   * @param image the image to detect features in.
   * @param keypoints the keypoints from the detected image
   * @return an array containing the computed descriptors.
   */
  cv::Mat ExtractDescriptors(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints);

private:
  std::shared_ptr<SuperPointModel> model_;
};
} // namespace beam_cv
