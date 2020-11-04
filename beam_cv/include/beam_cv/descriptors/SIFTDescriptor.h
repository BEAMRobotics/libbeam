/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>

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
   * @brief Default cosntructor
   */
  SIFTDescriptor() = default;

  /**
   * @brief Constructor
   * @param tuple_size The number of points that compose the SIFT descriptor.
   * @param patch_size The size of the square patch used in the random point
   * sampling to construct the descriptor
   */
  SIFTDescriptor(const int nfeatures, const int nOctaveLayers,
                 const double contrastThreshold, const double edgeThreshold,
                 const double sigma);

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
  /*	The number of best features to retain. The features are ranked by their
   * scores (measured in SIFT algorithm as the local contrast)*/
  int nfeatures_ = 500;
  /*	The number of layers in each octave. 3 is the value used in D. Lowe paper.
   * The number of octaves is computed automatically from the image
   * resolution.*/
  int nOctaveLayers_ = 3;
  /*The contrast threshold used to filter out weak features in semi-uniform
   * (low-contrast) regions. The larger the threshold, the less features are
   * produced by the detector.*/
  double contrastThreshold_ = 0.04;
  /*	The threshold used to filter out edge-like features. Note that the its
   * meaning is different from the contrastThreshold, i.e. the larger the
   * edgeThreshold, the less features are filtered out (more features are
   * retained).*/
  double edgeThreshold_ = 10;
  /*The sigma of the Gaussian applied to the input image at the octave #0. If
   * your image is captured with a weak camera with soft lenses, you might want
   * to reduce the number. */
  double sigma_ = 1.6;

  cv::Ptr<cv::xfeatures2d::SIFT> sift_descriptor_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
