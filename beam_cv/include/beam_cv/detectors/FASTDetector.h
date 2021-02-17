/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/detectors/Detector.h>

namespace beam_cv {

/**
 * Representation of a feature detector using the FAST algorithm.
 * Internally, this class is wrapping OpenCV's FastFeatureDetector module.
 * Further reference on the FastFeatureDetector can be found here:
 * http://docs.opencv.org/trunk/df/d74/classcv_1_1FastFeatureDetector.html
 */
class FASTDetector : public Detector {
public:
  /**
   * @brief Constructor
   * @param num_features number of features to retain, 0 will keep all.
   * @param threshold Threshold on difference between intensity of the central
   * pixel, and pixels in a circle (Bresenham radius 3) around this pixel.
   *  Recommended: 10. Must be greater than zero.
   * @param nonmax_suppression Removes keypoints in adjacent locations.
   *  Recommended: true
   * @param type Threshold on difference between intensity of the central pixel,
   * and pixels in a circle (Bresenham radius 3) around this pixel.
   *  Recommended: 10. Must be greater than zero.
   */
  FASTDetector(int num_features = 0, int threshold = 10,
               bool nonmax_suppression = true,
               int type = cv::FastFeatureDetector::TYPE_9_16);

  /**
   * @brief Default destructor
   */
  ~FASTDetector() override = default;

  /**
   * @brief Detects features in an image.
   * @param image the image to detect features in.
   * @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image) override;

private:
  int threshold_;
  bool nonmax_suppression_;
  int type_;
  int num_features_;

  /** The pointer to the wrapped cv::FastFeatureDetector object. */
  cv::Ptr<cv::FastFeatureDetector> fast_detector_;

  /**
   * @brief Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
