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
    int max_corners = 1000;
    double quality_level = 0.01;
    double min_distance = 1;
    int block_size = 3;
    bool use_harris_detector = false;
    double k = 0.04;

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
   * @param max_corners
   * @param quality_level
   * @param min_distance
   * @param block_size
   * @param use_harris_detector
   * @param k
   */
  GFTTDetector(int max_corners = 1000, double quality_level = 0.01,
               double min_distance = 1, int block_size = 3,
               bool use_harris_detector = false, double k = 0.04);

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
  // this gets called in each constructor
  void Setup();

  Params params_;

  /** The pointer to the wrapped cv::GFTTDetector object. */
  cv::Ptr<cv::GFTTDetector> GFTT_detector_;
};
} // namespace beam_cv
