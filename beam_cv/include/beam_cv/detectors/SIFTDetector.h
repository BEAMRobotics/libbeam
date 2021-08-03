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
  struct Params {
    // The number of best features to retain. The features are ranked by their
    // scores (measured in SIFT algorithm as the local contrast)
    int num_features = 0;

    // The number of layers in each octave. 3 is the value used in D. Lowe
    // paper. The number of octaves is computed automatically from the image
    // resolution.
    int n_octave_layers = 3;

    // The contrast threshold used to filter out weak features in semi-uniform
    // (low-contrast) regions. The larger the threshold, the less features are
    // produced by the detector.
    double contrast_threshold = 0.04;

    // The threshold used to filter out edge-like features. Note that the its
    // meaning is different from the contrastThreshold, i.e. the larger the
    // edgeThreshold, the less features are filtered out (more features are
    // retained).
    double edge_threshold = 10;

    // The sigma of the Gaussian applied to the input image at the octave #0. If
    // your image is captured with a weak camera with soft lenses, you might
    // want to reduce the number.
    double sigma = 1.6;

    // number of columns in image grid
    int grid_cols = 3;
    // number of rows in image grid
    int grid_rows = 2;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  SIFTDetector(const Params& params);

  /**
   * @brief Constructor that takes the individual parameters
   * @param num_features The number of best features to retain. The features are
   * ranked by their scores (measured in SIFT algorithm as the local contrast)
   * @param n_octave_layers The number of layers in each octave. 3 is the
   * value used in D. Lowe paper. The number of octaves is computed
   * automatically from the image resolution.
   * @param contrast_threshold The contrast threshold used to filter out weak
   * features in semi-uniform (low-contrast) regions. The larger the threshold,
   * the less features are produced by the detector.
   * @param edge_threshold The threshold used to filter out edge-like features.
   * Note that the its meaning is different from the contrastThreshold, i.e. the
   * larger the edgeThreshold, the less features are filtered out (more features
   * are retained).
   * @param sigma The sigma of the Gaussian applied to the input image at the
   * octave #0. If your image is captured with a weak camera with soft lenses,
   * you might want to reduce the number.
   * @param grid_cols // number of columns in image grid
   * @param grid_rows // number of rows in image grid
   */
  SIFTDetector(int num_features = 0, int n_octave_layers = 3,
               double contrast_threshold = 0.04, double edge_threshold = 10,
               double sigma = 1.6, int grid_cols = 3, int grid_rows = 2);

  /**
   * @brief Default destructor
   */
  ~SIFTDetector() override = default;

  /**
   * @brief Detects features in one image grid space.
   * @param image the image to detect features in.
   * @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectLocalFeatures(const cv::Mat& image);

private:
  // this gets called in each constructor
  void Setup();

  Params params_;

  /** The pointer to the wrapped cv::SIFT object. */
  //cv::Ptr<cv::xfeatures2d::SIFT> sift_detector_;
  cv::Ptr<cv::SIFT> sift_detector_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
