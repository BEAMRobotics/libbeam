#include <beam_cv/descriptors/SuperPointDescriptor.h>

namespace beam_cv {

SuperPointDescriptor::SuperPointDescriptor() {
  //
}

cv::Mat SuperPointDescriptor::ExtractDescriptors(
    const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  return descriptors;
}
} // namespace beam_cv
