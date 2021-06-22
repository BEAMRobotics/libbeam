/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/descriptors/Descriptor.h>

namespace beam_cv {

/** Representation of a descriptor extractor using the BRISK algorithm.
 *  Internally, this class is wrapping OpenCV's BRISK descriptor module.
 *  More info can be found here:
 *  http://docs.opencv.org/trunk/de/dbf/classcv_1_1BRISK.html
 */
class BRISKDescriptor : public Descriptor {
public:
  struct Params {
    /** defines the radius of each subsequent circle (in pixels).
     *  All numbers must be positive. Cannot be empty. Recommended: radius_list
     * = {0.0, 2.47, 4.17, 6.29, 9.18} */
    std::vector<float> rlist = {0.0, 2.47, 4.17, 6.29, 9.18};

    /** number_list defines the number of points in each circle. Must
     * be the same size as radiusList. All numbers must be positive. Cannot be
     * empty.  Recommended: number_list = {1, 10, 14, 15, 20}; */
    std::vector<int> nlist = {1, 10, 14, 15, 20};

    /** d_max and d_min are threshold distances to classify a pair of
     * points as a \a long pair or a \a short pair. Short pairs are not used in
     * the brightness comparison, due to balancing effects of local gradients.
     * The long pairs are not used in the assembly of the bit vector descriptor.
     *  d_max specifies the maximum threshold for short pairs. The value of
     *  d_max must be less than that of d_min. Recommended: d_max = 5.85 */
    float d_max = 5.85;

    /** d_min specifies the minimum threshold for long pairs.
     *  The value of d_min must be more than that of d_max.
     *  Recommended:  d_min = 8.2 */
    float d_min = 8.2;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  BRISKDescriptor(const Params& params);

  /**
   * @brief Custom constructor requiring individual params
   * @param rlist defines the radius of each subsequent circle (in pixels).
   *  All numbers must be positive. Cannot be empty. Recommended: radius_list =
   * {0.0, 2.47, 4.17, 6.29, 9.18}
   * @param nlist number_list defines the number of points in each circle. Must
   * be the same size as radiusList. All numbers must be positive. Cannot be
   * empty.  Recommended: number_list = {1, 10, 14, 15, 20};
   * @param d_max d_max and d_min are threshold distances to classify a pair of
   * points as a \a long pair or a \a short pair. Short pairs are not used in
   * the brightness comparison, due to balancing effects of local gradients. The
   *  long pairs are not used in the assembly of the bit vector descriptor.
   *  d_max specifies the maximum threshold for short pairs. The value of
   *  d_max must be less than that of d_min. Recommended: d_max = 5.85
   * @param d_min d_min specifies the minimum threshold for long pairs.
   *  The value of d_min must be more than that of d_max.
   *  Recommended:  d_min = 8.2
   */
  BRISKDescriptor(const std::vector<float>& rlist = {0.0, 2.47, 4.17, 6.29,
                                                     9.18},
                  const std::vector<int>& nlist = {1, 10, 14, 15, 20},
                  float d_max = 5.85, float d_min = 8.2);

  /**
   * @brief Default destructor
   */
  ~BRISKDescriptor() override = default;

  /** Extracts descriptors from the keypoints in an image, using the ORB
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

  /** The pointer to the wrapped cv::BRISK object. */
  cv::Ptr<cv::BRISK> brisk_descriptor_;

  /** Checks whether the desired configuration is valid. */
  void CheckConfig();
};
} // namespace beam_cv
