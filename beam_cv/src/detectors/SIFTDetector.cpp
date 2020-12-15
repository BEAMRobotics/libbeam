#include <beam_cv/detectors/SIFTDetector.h>

namespace beam_cv {

SIFTDetector::SIFTDetector(int num_features, int num_octave_layers,
                           double contrast_threshold, double edge_threshold,
                           double sigma) {
  this->num_features_ = num_features;
  this->num_octave_layers_ = num_octave_layers;
  this->contrast_threshold_ = contrast_threshold;
  this->edge_threshold_ = edge_threshold;
  this->sigma_ = sigma;
  // Ensure parameters are valid
  this->CheckConfig();

  this->sift_detector_ = cv::xfeatures2d::SIFT::create(
      this->num_features_, this->num_octave_layers_, this->contrast_threshold_,
      this->edge_threshold_, this->sigma_);
}

std::vector<cv::KeyPoint> SIFTDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  // Detect features in image and return keypoints.
  this->sift_detector_->detect(image, keypoints);
  return keypoints;
}

void SIFTDetector::CheckConfig() {
  return;
}

} // namespace beam_cv
