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
  struct Params {
    // number of features to retain, 0 will keep all.
    int num_features = 0;

    //  Threshold on difference between intensity of the central pixel, and
    //  pixels in a circle (Bresenham radius 3) around this pixel.
    //  Recommended: 10. Must be greater than zero.
    int threshold = 10;

    // Removes keypoints in adjacent locations. Recommended: true
    bool nonmax_suppression = true;

    // Options: TYPE_9_16, TYPE_7_12, TYPE_5_8. See opencv docs for more details
    int type = cv::FastFeatureDetector::TYPE_9_16;

    // number of columns in image grid
    int grid_rows = 3;
    // number of rows in image grid
    int grid_cols = 2;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  FASTDetector(const Params& params);

  /**
   * @brief Constructor that specifies each param individually
   * @param num_features number of features to retain, 0 will keep all.
   * @param threshold Threshold on difference between intensity of the central
   * pixel, and pixels in a circle (Bresenham radius 3) around this pixel.
   *  Recommended: 10. Must be greater than zero.
   * @param nonmax_suppression Removes keypoints in adjacent locations.
   *  Recommended: true
   * @param type Threshold on difference between intensity of the central pixel,
   * and pixels in a circle (Bresenham radius 3) around this pixel.
   *  Recommended: 10. Must be greater than zero.
   * @param grid_cols // number of columns in image grid
   * @param grid_rows // number of rows in image grid
   */
  FASTDetector(int num_features = 0, int threshold = 10,
               bool nonmax_suppression = true,
               int type = cv::FastFeatureDetector::TYPE_9_16, int grid_cols = 3,
               int grid_rows = 2);

  /**
   * @brief Default destructor
   */
  ~FASTDetector() override = default;

  /** @brief Detects features in one image grid space.
   *  @param image the image to detect features in.
   *  @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectLocalFeatures(const cv::Mat& image);

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
