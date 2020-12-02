#include <beam_cv/detectors/SuperPointDetector.h>

namespace beam_cv {

SuperPointDetector::SuperPointDetector(
    const std::shared_ptr<SuperPointModel>& model, float threshold, int border_width, bool nms)
    : model_(model), threshold_(threshold), border_width_(border_width), nms_(nms) {}

std::vector<cv::KeyPoint>
    SuperPointDetector::DetectFeatures(const cv::Mat& image) {
  // calculate start and end pixel coordinates
  int iniX = border_width_;
  int iniY = border_width_;
  int maxX = image.cols - border_width_;
  int maxY = image.rows - border_width_;

  std::vector<cv::KeyPoint> keypoints;
  model_->GetKeyPoints(threshold_, iniX, maxX,
                       iniY, maxY, keypoints, nms_);
  return keypoints;
}

} // namespace beam_cv
