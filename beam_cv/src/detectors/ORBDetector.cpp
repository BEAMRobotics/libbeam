#include <beam_cv/detectors/ORBDetector.h>

namespace beam_cv {

ORBDetector::ORBDetector(int num_features, float scale_factor, int num_levels,
                         int edge_threshold, int score_type,
                         int fast_threshold) {
  this->num_features_ = num_features;
  this->scale_factor_ = scale_factor;
  this->num_levels_ = num_levels;
  this->edge_threshold_ = edge_threshold;
  this->score_type_ = score_type;
  this->fast_threshold_ = fast_threshold;
  // Ensure parameters are valid
  this->CheckConfig();

  // Default parameters that should not be modified, and are not associated
  // with the descriptor. These values are the defaults recommended by OpenCV.
  //-------------------------------------------------------------------------
  int first_level = 0; // As per OpenCV docs, first_level must be zero.
  int tuple_size = 2;
  int patch_size = 31;

  this->orb_detector_ = cv::ORB::create(
      this->num_features_, this->scale_factor_, this->num_levels_,
      this->edge_threshold_, first_level, tuple_size, this->score_type_,
      patch_size, this->fast_threshold_);
}

std::vector<cv::KeyPoint> ORBDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  // Detect features in image and return keypoints.
  this->orb_detector_->detect(image, keypoints);
  return keypoints;
}

void ORBDetector::CheckConfig() {
  // Check parameters. If invalid, throw an exception.
  if (this->num_features_ < 0) {
    throw std::invalid_argument("num_features must be greater than/equal to 0");
  } else if (this->scale_factor_ < 1.0) {
    throw std::invalid_argument(
        "scale_factor must be greater than/equal to 1.0!");
  } else if (this->num_levels_ <= 0) {
    throw std::invalid_argument("num_levels must be greater than 0");
  } else if (this->edge_threshold_ < 0) {
    throw std::invalid_argument(
        "edge_threshold must be greater than/equal to 0");
  } else if (this->score_type_ < cv::ORB::HARRIS_SCORE ||
             this->score_type_ > cv::ORB::FAST_SCORE) {
    throw std::invalid_argument("Invalid score_type for ORBDetector!");
  } else if (this->fast_threshold_ <= 0) {
    throw std::invalid_argument("fast_threshold must be greater than 0");
  }
}

} // namespace beam_cv
