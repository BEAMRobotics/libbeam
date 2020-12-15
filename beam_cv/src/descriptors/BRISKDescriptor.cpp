#include <beam_cv/descriptors/BRISKDescriptor.h>

namespace beam_cv {

// Default constructor. Struct may be default or user defined.
BRISKDescriptor::BRISKDescriptor(const std::vector<float>& rlist,
                                 const std::vector<int>& nlist, float d_max,
                                 float d_min) {
  this->radius_list_ = rlist;
  this->number_list_ = nlist;
  this->d_max_ = d_max;
  this->d_min_ = d_min;
  // Ensure parameters are valid
  this->CheckConfig();
  // OpenCV refers to this as a parameter for "index remapping of the bits."
  // Kaehler and Bradski's book, "Learning OpenCV3: Computer Vision in C++
  // with the OpenCV Library" states this parameter is unused, and should be
  // omitted.
  std::vector<int> index_change;

  // Create cv::BRISK object with the desired parameters
  this->brisk_descriptor_ =
      cv::BRISK::create(this->radius_list_, this->number_list_, this->d_max_,
                        this->d_min_, index_change);
}

void BRISKDescriptor::CheckConfig() {
  // Check that the size of radiusList and numberList are equal and positive
  if (this->radius_list_.size() == 0) {
    throw std::invalid_argument("No parameters in radius_list!");
  } else if (this->number_list_.size() == 0) {
    throw std::invalid_argument("No parameters in number_list!");
  } else if (this->radius_list_.size() != this->number_list_.size()) {
    throw std::invalid_argument(
        "radius_list and number_list are of unequal size!");
  }
  // Ensure all values of radiusList are positive
  for (const auto& radius : this->radius_list_) {
    if (radius < 0) {
      throw std::invalid_argument("radius_list has a negative parameter!");
    }
  }
  // Ensure all values of numberList are positive
  for (auto& num_points : this->number_list_) {
    if (num_points < 0) {
      throw std::invalid_argument("number_list has a negative parameter!");
    }
  }
  // Ensure dMax and dMin are both positive, and check dMax is less than dMin
  if (this->d_max_ < 0) {
    throw std::invalid_argument("d_max is a negative value!");
  } else if (this->d_min_ < 0) {
    throw std::invalid_argument("d_min is a negative value!");
  } else if (this->d_max_ > this->d_min_) {
    throw std::invalid_argument("d_max is greater than d_min!");
  }
}

cv::Mat
    BRISKDescriptor::ExtractDescriptors(const cv::Mat& image,
                                        std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  this->brisk_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
