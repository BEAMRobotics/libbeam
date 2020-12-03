#include <beam_cv/detectors/SuperPointDetector.h>

namespace beam_cv {

SuperPointDetector::SuperPointDetector(
    const std::shared_ptr<SuperPointModel>& model, float threshold, bool nms,
    bool use_cuda)
    : model_(model), threshold_(threshold), nms_(nms), use_cuda_(use_cuda) {}

std::vector<cv::KeyPoint>
    SuperPointDetector::DetectFeatures(const cv::Mat& image) {
  model_->Detect(image, use_cuda_);

  std::vector<cv::KeyPoint> keypoints;
  model_->GetKeyPoints(threshold_, keypoints, nms_);
  return keypoints;
}

} // namespace beam_cv
