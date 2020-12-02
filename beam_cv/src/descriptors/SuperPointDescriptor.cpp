#include <beam_cv/descriptors/SuperPointDescriptor.h>

namespace beam_cv {

SuperPointDescriptor::SuperPointDescriptor(
    const std::shared_ptr<SuperPointModel>& model)
    : model_(model) {}

cv::Mat SuperPointDescriptor::ExtractDescriptors(
    const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
  
  if(!model_->has_been_initialized){
    model_->Detect(image, true);
  }

  cv::Mat descriptors;
  model_->ComputeDescriptors(keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
