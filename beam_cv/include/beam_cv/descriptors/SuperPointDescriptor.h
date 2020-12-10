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
 * Wrapper around SuperPointModel which allows the user to only extract
 * discriptors. For information on how it works and a description of all
 * parameter descriptions, see SuperPointModel.h
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
   * SuperPoint descriptor extractor. If first checks that the model has been
   * initialized, and if it hasn't, it'll initialize it. It then calls
   * SuperPointModel::ComputeDescriptors.
   * Note: It is expected that SuperPointDetector::DetectFeatures was first
   * called on the underlying model (this initalizes the model by calling
   * SuperPointModel::Detect), but if it hasn't, then this function will be
   * initalize the model by calling SuperPointModel::Detect itself. In our
   * traditional pipeline, the detector is always called first, however this is
   * not necessary for SuperPoint. The only issue with not calling
   * SuperPointDetector is that you cannot set the keypoint extraction
   * parameters from SuperPointDescriptor.
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
