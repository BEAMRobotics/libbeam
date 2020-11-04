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
  /**
   * @brief Default cosntructor
   */
  ORBDetector() = default;
  /**
   * @brief Constructor
   * @param tuple_size The number of points that compose the ORB descriptor.
   * @param patch_size The size of the square patch used in the random point
   * sampling to construct the descriptor
   */
  ORBDetector(const int num_features = 500, const float scale_factor = 1.2f,
              const int num_levels = 8, const int edge_threshold = 31,
              const int score_type = cv::ORB::HARRIS_SCORE,
              const int fast_threshold = 20);

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
  int num_features_ = 500;
  float scale_factor_ = 1.2;
  int num_levels_ = 8;
  int edge_threshold_ = 31;
  int score_type_ = cv::ORB::HARRIS_SCORE;
  int fast_threshold_ = 20;
  /** The pointer to the wrapped cv::ORB object. */
  cv::Ptr<cv::ORB> orb_detector_;

  /** Checks whether the desired configuration is valid.
   */
  void CheckConfig();
};
} // namespace beam_cv
