/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <beam_cv/detectors/Detector.h>

namespace beam_cv {

/** Representation of a keypoint detector using the SIFT algorithm.
 *
 * Internally, this class is wrapping OpenCV's SIFT descriptor module. More info
 * can be found here: http://docs.opencv.org/trunk/db/d95/classcv_1_1SIFT.html
 */
class SIFTDetector : public Detector {
public:
  /**
   * @brief Constructor
   * @param nfeatures
   * @param nOctaveLayers
   * @param contrastThreshold
   * @param edgeThreshold
   * @param sigma
   */
  SIFTDetector(const int nfeatures = 0, const int nOctaveLayers = 3,
               const double contrastThreshold = 0.04,
               const double edgeThreshold = 10, const double sigma = 1.6);

  /**
   * @brief Default destructor
   */
  ~SIFTDetector() override = default;

  /** Detects features in an image.
   *  @param image the image to detect features in.
   *  @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

private:
  /**	The number of best features to retain. The features are ranked by their
   *  scores (measured in SIFT algorithm as the local contrast)
   */
  int nfeatures_ = 0;
  /** The number of layers in each octave. 3 is the value used in D. Lowe paper.
   *  The number of octaves is computed automatically from the image
   *  resolution.
   */
  int nOctaveLayers_ = 3;
  /** The contrast threshold used to filter out weak features in semi-uniform
   *  (low-contrast) regions. The larger the threshold, the less features are
   *  produced by the detector.
   */
  double contrastThreshold_ = 0.04;
  /**	The threshold used to filter out edge-like features. Note that the its
   *  meaning is different from the contrastThreshold, i.e. the larger the
   *  edgeThreshold, the less features are filtered out (more features are
   *  retained).
   */
  double edgeThreshold_ = 10;
  /** The sigma of the Gaussian applied to the input image at the octave #0. If
   *  your image is captured with a weak camera with soft lenses, you might want
   *  to reduce the number.
   */
  double sigma_ = 1.6;
  /** The pointer to the wrapped cv::SIFT object. */
  cv::Ptr<cv::xfeatures2d::SIFT> sift_detector_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
