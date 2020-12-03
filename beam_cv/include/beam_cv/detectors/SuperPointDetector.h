/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/detectors/Detector.h>
#include <beam_cv/models/SuperPointModel.h>

namespace beam_cv {

/**
 * Add descriptions
 */
class SuperPointDetector : public Detector {
public:
  /**
   * @brief Constructor
   * @param model pointer to the superpoint model
   * @param threshold Lower the threshold, more features are extracted. SP-SLAM
   * uses between 7 and 20
   * @param nms Non-Maximum Suppression. This helps ensure images are evenly
   * distributed in the image.
   */
  SuperPointDetector(const std::shared_ptr<SuperPointModel>& model,
                     float threshold = 0.1, bool nms = true,
                     bool use_cuda = true);

  /**
   * @brief Default destructor
   */
  ~SuperPointDetector() override = default;

  /**
   * @brief Detects features in an image.
   * @param image the image to detect features in.
   * @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

private:
  std::shared_ptr<SuperPointModel> model_;
  float threshold_;
  bool nms_;
  bool use_cuda_;
};
} // namespace beam_cv
