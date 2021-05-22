/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/detectors/Detector.h>

namespace beam_cv {

/** Representation of a feature detector using the GFTT algorithm.
 *  Internally, this class is wrapping OpenCV's GFTTDetector module.
 *  Further reference on the GFTTDetector can be found here:
 */
class GFTTDetector : public Detector {
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
  GFTTDetector(int maxCorners = 1000, double qualityLevel = 0.01,
               double minDistance = 1, int blockSize = 3,
               bool useHarrisDetector = false, double k = 0.04);

  /**
   * @brief Default destructor
   */
  ~GFTTDetector() override = default;

  /** Detects features in an image.
   *  @param image the image to detect features in.
   *  @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

private:
  int max_corners_{};
  double quality_level_{};
  double min_distance_{};
  int block_size_{};
  bool use_harris_detector_{};
  double k_{};

  /** The pointer to the wrapped cv::GFTTDetector object. */
  cv::Ptr<cv::GFTTDetector> GFTT_detector_;

};
} // namespace beam_cv
