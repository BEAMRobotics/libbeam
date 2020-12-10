#include <beam_cv/detectors/SuperPointDetector.h>

namespace beam_cv {

SuperPointDetector::SuperPointDetector(
    const std::shared_ptr<SuperPointModel>& model, float conf_threshold,
    int border, int nms_dist_threshold, int max_features, int grid_size, bool use_cuda)
    : model_(model),
      conf_threshold_(conf_threshold),
      border_(border),
      nms_dist_threshold_(nms_dist_threshold),
      max_features_(max_features),
      grid_size_(grid_size),
      use_cuda_(use_cuda) {}

std::vector<cv::KeyPoint>
    SuperPointDetector::DetectFeatures(const cv::Mat& image) {
  model_->Detect(image, use_cuda_);

  std::vector<cv::KeyPoint> keypoints;
  model_->GetKeyPoints(keypoints, conf_threshold_, border_, nms_dist_threshold_,
                       max_features_, grid_size_);
  return keypoints;
}

} // namespace beam_cv
