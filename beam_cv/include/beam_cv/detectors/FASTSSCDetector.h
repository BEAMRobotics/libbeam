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
class FASTSSCDetector : public Detector {
public:
  struct Params {
    // number of features to retain, 0 will keep all.
    int num_features = 0;

    //  Threshold on difference between intensity of the central pixel, and
    //  pixels in a circle (Bresenham radius 3) around this pixel.
    //  Recommended: 10. Must be greater than zero.
    int threshold = 10;

    // Removes keypoints in adjacent locations. Recommended: true
    bool nonmax_suppression = true;

    // tolerance for ssc algorithm, more info can be found here:
    // https://www.researchgate.net/publication/323388062_Efficient_adaptive_non-maximal_suppression_algorithms_for_homogeneous_spatial_keypoint_distribution
    float ssc_tolerance = 0.1;

    // Options: TYPE_9_16, TYPE_7_12, TYPE_5_8. See opencv docs for more details
    cv::FastFeatureDetector::DetectorType type =
        cv::FastFeatureDetector::TYPE_9_16;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  FASTSSCDetector(const Params& params);

  /**
   * @brief Constructor that specifies each param individually
   * @param num_features number of features to retain, 0 will keep all.
   * @param threshold Threshold on difference between intensity of the
   * central pixel, and pixels in a circle (Bresenham radius 3) around this
   * pixel. Recommended: 10. Must be greater than zero.
   * @param nonmax_suppression Removes keypoints in adjacent locations.
   *  Recommended: true
   * @param ssc_tolerance tolerance for ssc algorithm
   *  Recommended: 0.1
   * @param type Threshold on difference between intensity of the central pixel,
   * and pixels in a circle (Bresenham radius 3) around this pixel.
   *  Recommended: 10. Must be greater than zero.
   */
  FASTSSCDetector(int num_features = 0, int threshold = 10,
                  bool nonmax_suppression = true, float ssc_tolerance = 0.1,
                  cv::FastFeatureDetector::DetectorType type =
                      cv::FastFeatureDetector::TYPE_9_16);

  /**
   * @brief Default destructor
   */
  ~FASTSSCDetector() override = default;

  /** @brief Detects features in one image grid space.
   *  @param image the image to detect features in.
   *  @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectLocalFeatures(const cv::Mat& image);

  /** @brief Performs non max suppression using the ssc algorithm introduced in:
   * https://www.researchgate.net/publication/323388062_Efficient_adaptive_non-maximal_suppression_algorithms_for_homogeneous_spatial_keypoint_distribution
   *  @param keypoints existing keypoints
   *  @param num_features number of features to retain
   *  @param cols in given image
   *  @param rows in given image
   *  @return a vector of keypoints in the image
   */
  std::vector<cv::KeyPoint>
      SSCNonmaxSuppression(const std::vector<cv::KeyPoint>& keypoints,
                           const int num_features, const int cols,
                           const int rows) const;

  /** @brief Gets the string representation of the type of detector
   *  @return string of detector type
   */
  std::string GetTypeString() const { return "FASTSSC"; }

  /** @brief Gets the type of detector
   *  @return detector type
   */
  DetectorType GetType() const { return DetectorType::FASTSSC; }

private:
  // this gets called in each constructor
  void Setup();

  Params params_;

  /** The pointer to the wrapped cv::FastFeatureDetector object. */
  cv::Ptr<cv::FastFeatureDetector> fast_detector_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
