/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/detectors/Detector.h>

namespace beam_cv {

/** Representation of a keypoint detector using the ORB algorithm.
 * Internally, this class is wrapping OpenCV's ORB descriptor module. More info
 * can be found here: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html
 */
class ORBDetector : public Detector {
public:
  struct Params {
    // number of features to retain, 0 will keep all.
    int num_features = 500;

    // for docs, see opencv
    float scale_factor = 1.2;
    int num_levels = 8;
    int edge_threshold = 31;
    cv::ORB::ScoreType score_type = cv::ORB::HARRIS_SCORE;
    int fast_threshold = 20;

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
  ORBDetector(const Params& params);

  /**
   * @brief Constructor
   * @param num_features number of features to retain, 0 will keep all.
   * @param scale_factor
   * @param num_levels
   * @param edge_threshold
   * @param score_type
   * @param fast_threshold
   * @param grid_cols // number of columns in image grid
   * @param grid_rows // number of rows in image grid
   */
  ORBDetector(int num_features = 500, float scale_factor = 1.2,
              int num_levels = 8, int edge_threshold = 31,
              cv::ORB::ScoreType score_type = cv::ORB::HARRIS_SCORE,
              int fast_threshold = 20, int grid_cols = 3, int grid_rows = 2);

  /**
   * @brief Default destructor
   */
  ~ORBDetector() override = default;

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

  /** The pointer to the wrapped cv::ORB object. */
  cv::Ptr<cv::ORB> orb_detector_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
