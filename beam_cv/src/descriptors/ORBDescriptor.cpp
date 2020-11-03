#include <beam_cv/descriptors/ORBDescriptor.h>

namespace beam_cv {

// Default constructor. Struct may be default or user defined.
ORBDescriptor::ORBDescriptor(const int tuple_size, const int patch_size) {
  this->tuple_size_ = tuple_size;
  this->patch_size_ = patch_size;
  // Ensure parameters are valid
  this->CheckConfig();

  // Default parameters that should not be modified, and are not associated
  // with the descriptor. These values are the defaults recommended by OpenCV.
  //-------------------------------------------------------------------------
  int num_features = 500;
  float scale_factor = 1.2f;
  int num_levels = 8;
  int edge_threshold = 31;
  int first_level = 0; // As per OpenCV docs, first_level must be zero.
  int score_type = cv::ORB::HARRIS_SCORE;
  int fast_threshold = 20;

  // Create cv::ORB object with the desired parameters
  this->orb_descriptor_ = cv::ORB::create(
      num_features, scale_factor, num_levels, edge_threshold, first_level,
      this->tuple_size_, score_type, this->patch_size_, fast_threshold);
}

void ORBDescriptor::CheckConfig() {
  // Check that the value of tuple_size is between 2 and 4, and that
  // patch_size is greater than zero.
  if (this->tuple_size_ < 2 || this->tuple_size_ > 4) {
    BEAM_CRITICAL("tuple_size_ is not an acceptable value!");
    throw std::invalid_argument("tuple_size is not an acceptable value!");
  } else if (this->patch_size_ <= 0) {
    BEAM_CRITICAL("patch_size_ is less than/ equal to zero!");
    throw std::invalid_argument("patch_size is less than/ equal to zero!");
  }
}

cv::Mat
    ORBDescriptor::ExtractDescriptors(const cv::Mat& image,
                                      std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  this->orb_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
