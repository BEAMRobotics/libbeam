/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <beam_utils/utils.hpp> 

namespace beam_cv {

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
