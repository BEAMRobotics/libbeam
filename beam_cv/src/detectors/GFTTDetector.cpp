#include <beam_cv/detectors/GFTTDetector.h>

namespace beam_cv {

GFTTDetector::GFTTDetector(int maxCorners, double qualityLevel,
                           double minDistance, int blockSize,
                           bool useHarrisDetector, double k) {
  this->max_corners_ = maxCorners;
  this->quality_level_ = qualityLevel;
  this->min_distance_ = minDistance;
  this->block_size_ = blockSize;
  this->use_harris_detector_ = useHarrisDetector;
  this->k_ = k;
  this->GFTT_detector_ = cv::GFTTDetector::create(
      this->max_corners_, this->quality_level_, this->min_distance_,
      this->block_size_, this->use_harris_detector_, this->k_);
}

std::vector<cv::KeyPoint> GFTTDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  // Detect features in image and return keypoints.
  this->GFTT_detector_->detect(image, keypoints);
  // Retain best keypoints, if specified.
  if (this->max_corners_ != 0) {
    cv::KeyPointsFilter::retainBest(keypoints, this->max_corners_);
  }
  return keypoints;
}

} // namespace beam_cv
