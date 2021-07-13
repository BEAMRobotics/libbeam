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
  struct Params {
    // number of features to retain, 0 will keep all.
    int num_features = 1000;

    // for docs, see opencv
    double quality_level = 0.01;
    double min_distance = 1;
    int block_size = 3;
    bool use_harris_detector = false;
    double k = 0.04;

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
  GFTTDetector(const Params& params);

  /**
   * @brief Constructor that sets each param individually
   * @param num_features number of features to retain, 0 will keep all.
   * @param quality_level
   * @param min_distance
   * @param block_size
   * @param use_harris_detector
   * @param k
   * @param grid_cols // number of columns in image grid
   * @param grid_rows // number of rows in image grid
   */
  GFTTDetector(int num_features = 1000, double quality_level = 0.01,
               double min_distance = 1, int block_size = 3,
               bool use_harris_detector = false, double k = 0.04,
               int grid_cols = 3, int grid_rows = 2);

  /**
   * @brief Default destructor
   */
  ~GFTTDetector() override = default;

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

  /** The pointer to the wrapped cv::GFTTDetector object. */
  cv::Ptr<cv::GFTTDetector> GFTT_detector_;
};
} // namespace beam_cv
