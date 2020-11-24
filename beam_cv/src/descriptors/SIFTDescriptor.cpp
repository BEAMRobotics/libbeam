#include <beam_cv/descriptors/SIFTDescriptor.h>

namespace beam_cv {

// Default constructor. Struct may be default or user defined.
SIFTDescriptor::SIFTDescriptor(const int nfeatures, const int nOctaveLayers,
                               const double contrastThreshold,
                               const double edgeThreshold, const double sigma) {
  this->nfeatures_ = nfeatures;
  this->nOctaveLayers_ = nOctaveLayers;
  this->contrastThreshold_ = contrastThreshold;
  this->edgeThreshold_ = edgeThreshold;
  this->sigma_ = sigma;
  // Ensure parameters are valid
  this->CheckConfig();

  // Create cv::SIFT object with the desired parameters
  this->sift_descriptor_ = cv::xfeatures2d::SIFT::create(
      this->nfeatures_, this->nOctaveLayers_, this->contrastThreshold_,
      this->edgeThreshold_, this->sigma_);
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
