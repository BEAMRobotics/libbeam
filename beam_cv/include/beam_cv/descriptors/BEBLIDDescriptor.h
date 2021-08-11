/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/xfeatures2d.hpp>

#include <beam_cv/descriptors/Descriptor.h>

namespace beam_cv {

/** Representation of a descriptor extractor using the ORB algorithm.
 * Internally, this class is wrapping OpenCV's ORB descriptor module. More info
 * can be found here: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html
 */
class BEBLIDDescriptor : public Descriptor {
public:
  struct Params {
    /** Adjust the sampling window around detected keypoints:
     *    1.00f should be the scale for ORB keypoints
     *    6.75f should be the scale for SIFT detected keypoints
     *    6.25f is default and fits for KAZE, SURF detected keypoints
     *    5.00f should be the scale for AKAZE, MSD, GFTT, AGAST, FAST, BRISK keypoints
     */
    float scale_factor = 1.00;

    /** Determines the number of bits in the descriptor. Should be
     * either 0 (BEBLID::SIZE_512_BITS) or 1 (BEBLID::SIZE_256_BITS)
     */
    int n_bits = cv::xfeatures2d::BEBLID::SIZE_512_BITS;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  BEBLIDDescriptor(const Params& params);

  /**
   * @brief Constructor
   * @param scale_factor adjust the sampling window around detected keypoints:
   *    1.00f should be the scale for ORB keypoints
   *    6.75f should be the scale for SIFT detected keypoints
   *    6.25f is default and fits for KAZE, SURF detected keypoints
   *    5.00f should be the scale for AKAZE, MSD, GFTT, AGAST, FAST, BRISK keypoints
   * @param n_bits Determine the number of bits in the descriptor. Should be
   * either 0 (BEBLID::SIZE_512_BITS) or 1 (BEBLID::SIZE_256_BITS)
   */
  BEBLIDDescriptor(float scale_factor = 1.00,
                   int n_bits = cv::xfeatures2d::BEBLID::SIZE_512_BITS);

  /**
   * @brief Default destructor
   */
  ~BEBLIDDescriptor() override = default;

  /** Extracts descriptors from the keypoints in an image, using the BEBLID
   *  descriptor extractor.
   *  @param image the image to detect features in.
   *  @param keypoints the keypoints from the detected image
   *  @return an array containing the computed descriptors.
   */
  cv::Mat ExtractDescriptors(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints);

private:
  // this gets called in each constructor
  void Setup();

  Params params_;

  /** The pointer to the wrapped cv::BEBLID object. */
  cv::Ptr<cv::xfeatures2d::BEBLID> beblid_descriptor_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
