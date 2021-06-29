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
    int num_features = 500;
    float scale_factor = 1.2;
    int num_levels = 8;
    int edge_threshold = 31;
    int score_type = cv::ORB::HARRIS_SCORE;
    int fast_threshold = 20;

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
   * @param num_features
   * @param scale_factor
   * @param num_levels
   * @param edge_threshold
   * @param score_type
   * @param fast_threshold
   */
  ORBDetector(int num_features = 500, float scale_factor = 1.2,
              int num_levels = 8, int edge_threshold = 31,
              int score_type = cv::ORB::HARRIS_SCORE, int fast_threshold = 20);

  /**
   * @brief Default destructor
   */
  ~ORBDetector() override = default;

  /** Detects features in an image.
   *  @param image the image to detect features in.
   *  @return a vector containing all of the keypoints found within the image.
   */
  std::vector<cv::KeyPoint> DetectFeatures(const cv::Mat& image);

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
