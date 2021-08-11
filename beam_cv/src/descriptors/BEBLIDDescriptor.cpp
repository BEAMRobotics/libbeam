#include <beam_cv/descriptors/BEBLIDDescriptor.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void BEBLIDDescriptor::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for BEBLID descriptor params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  scale_factor = J["scale_factor"];
  n_bits = J["n_bits"];
}

BEBLIDDescriptor::BEBLIDDescriptor(const Params& params) : params_(params) {
  Setup();
};

// Default constructor. Struct may be default or user defined.
BEBLIDDescriptor::BEBLIDDescriptor(float scale_factor, int n_bits) {
  params_.scale_factor = scale_factor;
  params_.n_bits = n_bits;
  Setup();
}

void BEBLIDDescriptor::Setup() {
  // Ensure parameters are valid
  CheckConfig();

  // Create cv::ORB object with the desired parameters
  beblid_descriptor_ =
      cv::xfeatures2d::BEBLID::create(params_.scale_factor, params_.n_bits);
}

void BEBLIDDescriptor::CheckConfig() {}

cv::Mat
    BEBLIDDescriptor::ExtractDescriptors(const cv::Mat& image,
                                         std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  beblid_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
