#include <beam_cv/descriptors/SIFTDescriptor.h>

namespace beam_cv {

// Default constructor. Struct may be default or user defined.
SIFTDescriptor::SIFTDescriptor(int num_features, int num_octave_layers,
                               double contrast_threshold, double edge_threshold,
                               double sigma) {
  this->num_features_ = num_features;
  this->num_octave_layers_ = num_octave_layers;
  this->contrast_threshold_ = contrast_threshold;
  this->edge_threshold_ = edge_threshold;
  this->sigma_ = sigma;
  // Ensure parameters are valid
  this->CheckConfig();

  this->sift_descriptor_ = cv::xfeatures2d::SIFT::create(
      this->num_features_, this->num_octave_layers_, this->contrast_threshold_,
      this->edge_threshold_, this->sigma_);
}

void SIFTDescriptor::CheckConfig() {
  return;
}

cv::Mat
    SIFTDescriptor::ExtractDescriptors(const cv::Mat& image,
                                       std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  this->sift_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
