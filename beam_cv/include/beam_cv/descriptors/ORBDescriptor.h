/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/descriptors/Descriptor.h>

namespace beam_cv {

/** Representation of a descriptor extractor using the ORB algorithm.
 * Internally, this class is wrapping OpenCV's ORB descriptor module. More info
 * can be found here: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html
 */
class ORBDescriptor : public Descriptor {
public:
  /**
   * @brief Constructor
   * @param tuple_size The number of points that compose the ORB descriptor. A
   * value of 2 constructs the descriptor by comparing the brightnesses of a
   * random pair of points. Other values are 3 or 4; 3 will use 3 random points,
   * and 4 similarly will use 4 random points. For these last two options, the
   *  output result will be 2 bits, as the result is the index of the
   *  brightest point (0, 1, 2, 3). Thus, the distance method used for the
   *  BruteForceMatcher or FLANNMatcher will have to be cv::NORM_HAMMING_2, a
   *  variation to the Hamming distance, when tuple_size = 3 or 4. Options: 2,
   * 3, or 4.
   * @param patch_size The size of the square patch used in the random point
   * sampling to construct the descriptor. This patch is smoothed using an
   * integral image. Default: 31 (31 x 31 pixels). Must be greater than 0.
   */
  ORBDescriptor(const int tuple_size = 2, const int patch_size = 31);

  /**
   * @brief Default destructor
   */
  ~ORBDescriptor() override = default;

  /** Extracts descriptors from the keypoints in an image, using the ORB
   *  descriptor extractor.
   *  @param image the image to detect features in.
   *  @param keypoints the keypoints from the detected image
   *  @return an array containing the computed descriptors.
   */
  cv::Mat ExtractDescriptors(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints);

private:
  int tuple_size_;
  int patch_size_;
  /** The pointer to the wrapped cv::ORB object. */
  cv::Ptr<cv::ORB> orb_descriptor_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
