/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <beam_cv/descriptors/Descriptor.h>

namespace beam_cv {

/** Representation of a descriptor extractor using the SIFT algorithm.
 *
 * Internally, this class is wrapping OpenCV's SIFT descriptor module. More info
 * can be found here: http://docs.opencv.org/trunk/db/d95/classcv_1_1SIFT.html
 */
class SIFTDescriptor : public Descriptor {
public:
  /**
   * @brief Constructor
   * @param num_features The number of best features to retain
   * @param num_octave_layers The number of layers in each octave
   * @param contrast_threshold The contrast threshold used to filter out weak features in semi-uniform
   *  (low-contrast) regions. The larger the threshold, the less features are
   *  produced by the detector.
   * @param edge_threshold The threshold used to filter out edge-like features. Note that the its
   *  meaning is different from the contrastThreshold, i.e. the larger the
   *  edgeThreshold, the less features are filtered out (more features are
   *  retained).
   * @param sigma The sigma of the Gaussian applied to the input image at the octave #0. If
   *  your image is captured with a weak camera with soft lenses, you might want
   *  to reduce the number.
   */
  SIFTDescriptor(const int num_features = 0, const int n_octave_layers = 3,
                 const double contrast_threshold = 0.04,
                 const double edge_threshold = 10, const double sigma = 1.6);

  /**
   * @brief Default destructor
   */
  ~SIFTDescriptor() override = default;

  /** Extracts descriptors from the keypoints in an image, using the SIFT
   *  descriptor extractor.
   *  @param image the image to detect features in.
   *  @param keypoints the keypoints from the detected image
   *  @return an array containing the computed descriptors.
   */
  cv::Mat ExtractDescriptors(const cv::Mat& image,
                             std::vector<cv::KeyPoint>& keypoints);

private:
  int num_features_;
  int num_octave_layers_;
  double contrast_threshold_;
  double edge_threshold_;
  double sigma_;
  /** The pointer to the wrapped cv::SIFT object. */
  cv::Ptr<cv::xfeatures2d::SIFT> sift_descriptor_;
  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
