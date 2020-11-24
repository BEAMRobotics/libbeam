#include <beam_cv/detectors/SIFTDetector.h>

namespace beam_cv {

SIFTDetector::SIFTDetector(const int nfeatures, const int nOctaveLayers,
                           const double contrastThreshold,
                           const double edgeThreshold, const double sigma) {
  this->nfeatures_ = nfeatures;
  this->nOctaveLayers_ = nOctaveLayers;
  this->contrastThreshold_ = contrastThreshold;
  this->edgeThreshold_ = edgeThreshold;
  this->sigma_ = sigma;
  // Ensure parameters are valid
  this->CheckConfig();

  this->sift_detector_ = cv::xfeatures2d::SIFT::create(
      this->nfeatures_, this->nOctaveLayers_, this->contrastThreshold_,
      this->edgeThreshold_, this->sigma_);
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
