#include <beam_cv/descriptors/SIFTDescriptor.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void SIFTDescriptor::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for SIFT descriptor params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  num_features = J["num_features"];
  num_octave_layers = J["num_octave_layers"];
  contrast_threshold = J["contrast_threshold"];
  edge_threshold = J["edge_threshold"];
  sigma = J["sigma"];
}

SIFTDescriptor::SIFTDescriptor(const Params& params) : params_(params) {
  Setup();
};

// Default constructor. Struct may be default or user defined.
SIFTDescriptor::SIFTDescriptor(int num_features, int num_octave_layers,
                               double contrast_threshold, double edge_threshold,
                               double sigma) {
  params_.num_features = num_features;
  params_.num_octave_layers = num_octave_layers;
  params_.contrast_threshold = contrast_threshold;
  params_.edge_threshold = edge_threshold;
  params_.sigma = sigma;
  Setup();
}

void SIFTDescriptor::Setup() {
  // Ensure parameters are valid
  CheckConfig();

  sift_descriptor_ = cv::SIFT::create(
      params_.num_features, params_.num_octave_layers,
      params_.contrast_threshold, params_.edge_threshold, params_.sigma);
}

void SIFTDescriptor::CheckConfig() {
  return;
}

cv::Mat SIFTDescriptor::ExtractDescriptors(
    const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) const {
  cv::Mat descriptors;
  sift_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
