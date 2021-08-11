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

void BEBLIDDescriptor::CheckConfig() {
  if (params_.n_bits != cv::xfeatures2d::BEBLID::SIZE_512_BITS &&
      params_.n_bits != cv::xfeatures2d::BEBLID::SIZE_256_BITS) {
    BEAM_ERROR("n_bits is not an acceptable value!");
    throw std::invalid_argument("tuple_size is not an acceptable value!");
  } else if (params_.scale_factor < 1.0 || params_.scale_factor > 7.0) {
    BEAM_ERROR("scale_factor is not within acceptable range: [1.0, 7.0]");
    throw std::invalid_argument(
        "scale_factor is not within acceptable range: [1.0, 7.0]");
  }
}

cv::Mat
    BEBLIDDescriptor::ExtractDescriptors(const cv::Mat& image,
                                         std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  beblid_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
