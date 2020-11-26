#include <beam_cv/detectors/SuperPointDetector.h>

namespace beam_cv {

SuperPointDetector::SuperPointDetector() {
  //
}

std::vector<cv::KeyPoint>
    SuperPointDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  return keypoints;
}

} // namespace beam_cv
