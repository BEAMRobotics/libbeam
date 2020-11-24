#include <beam_cv/detectors/FASTDetector.h>

namespace beam_cv {

FASTDetector::FASTDetector(const int threshold, const bool nonmax_suppression,
                           const int type, const int num_features) {
  this->threshold_ = threshold;
  this->nonmax_suppression_ = nonmax_suppression;
  this->type_ = type;
  this->num_features_ = num_features;
  // Ensure parameters are valid
  this->CheckConfig();
  this->fast_detector_ = cv::FastFeatureDetector::create(
      this->threshold_, this->nonmax_suppression_, this->type_);
}

std::vector<cv::KeyPoint> FASTDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  // Detect features in image and return keypoints.
  this->fast_detector_->detect(image, keypoints);
  // Retain best keypoints, if specified.
  if (this->num_features_ != 0) {
    cv::KeyPointsFilter::retainBest(keypoints, this->num_features_);
  }
  return keypoints;
}

void FASTDetector::CheckConfig() {
    // Check parameters. If invalid, throw an exception.
    if (this->threshold_ <= 0) {
        throw std::invalid_argument("threshold must be greater than 0!");
    } else if (this->type_ < 0 || this->type_ > 3) {
        throw std::invalid_argument("Invalid type for FASTDetector!");
    } else if (this->num_features_ < 0) {
        throw std::invalid_argument(
          "num_features must be greater than/equal to 0!");
    }
}

} // namespace beam_cv
