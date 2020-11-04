/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/detectors/Detector.h>

namespace beam_cv {

/** Representation of a feature detector using the FAST algorithm.
 *  Internally, this class is wrapping OpenCV's FastFeatureDetector module.
 *  Further reference on the FastFeatureDetector can be found here:
 *  http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
class FASTDetector : public Detector {
public:

  /**
   * @brief Constructor
   */
  FASTDetector(const int threshold = 10, const bool nonmax_suppression = true,
               const int type = cv::FastFeatureDetector::TYPE_9_16, const int num_features = 0);

  /**
   * @brief Default destructor
   */
  ~FASTDetector() override = default;

  /** Detects features in an image.
   *  @param image the image to detect features in.
   *  @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

private:
  /** Threshold on difference between intensity of the central pixel, and
   *  pixels in a circle (Bresenham radius 3) around this pixel.
   *  Recommended: 10. Must be greater than zero.
   */
  int threshold_ = 10;
  /** Removes keypoints in adjacent locations.
   *  Recommended: true
   */
  bool nonmax_suppression_ = true;
  /** Neighbourhood, as defined in the paper by Rosten. TYPE_N_M refers to
   *  the pixel circumference (M) and number of consecutive pixels (N) that
   *  must be brighter or darker than the center pixel for the algorithm to
   *  deem the point as a corner.
   *  Options:
   *  cv::FastFeatureDetector::TYPE_5_8
   *  cv::FastFeatureDetector::TYPE_7_12
   *  cv::FastFeatureDetector::TYPE_9_16 (recommended)
   */
  int type_ = cv::FastFeatureDetector::TYPE_9_16;
  /** The number of features to keep from detection.
   *  If set to 0, the detector will keep all features detected. Else, it will
   *  retain the best keypoints up to the number specified.
   *  Default: 0.
   */
  int num_features_ = 0;

  /** The pointer to the wrapped cv::FastFeatureDetector object. */
  cv::Ptr<cv::FastFeatureDetector> fast_detector_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
