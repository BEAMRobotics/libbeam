#include <beam_cv/detectors/SuperPointDetector.h>

namespace beam_cv {

SuperPointDetector::SuperPointDetector(
    const std::shared_ptr<SuperPointModel>& model, int max_features,
    float conf_threshold, int border, int nms_dist_threshold, int grid_size,
    bool use_cuda) {
  params_.conf_threshold = conf_threshold;
  params_.border = border;
  params_.nms_dist_threshold = nms_dist_threshold;
  params_.max_features = max_features;
  params_.grid_size = grid_size;
  params_.use_cuda = use_cuda;
  model_ = model;
}

SuperPointDetector::SuperPointDetector(
    const std::shared_ptr<SuperPointModel>& model,
    const SuperPointDetector::Params& params)
    : params_(params), model_(model) {}

std::vector<cv::KeyPoint>
    SuperPointDetector::DetectFeatures(const cv::Mat& image) {
  model_->Detect(image, params_.use_cuda);
  std::vector<cv::KeyPoint> keypoints;
  model_->GetKeyPoints(keypoints, params_.conf_threshold, params_.border,
                       params_.nms_dist_threshold, params_.max_features,
                       params_.grid_size);
  return keypoints;
}

} // namespace beam_cv
