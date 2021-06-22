#include <beam_cv/descriptors/ORBDescriptor.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void ORBDescriptor::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for ORB descriptor params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  tuple_size = J["tuple_size"];
  patch_size = J["patch_size"];
}

ORBDescriptor::ORBDescriptor(const Params& params) : params_(params) {Setup();};

// Default constructor. Struct may be default or user defined.
ORBDescriptor::ORBDescriptor(int tuple_size, int patch_size) {
  params_.tuple_size = tuple_size;
  params_.patch_size = patch_size;
  Setup();
}

void ORBDescriptor::Setup(){
  // Ensure parameters are valid
  CheckConfig();

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
  orb_descriptor_ = cv::ORB::create(
      num_features, scale_factor, num_levels, edge_threshold, first_level,
      params_.tuple_size, score_type, params_.patch_size, fast_threshold);
}

void ORBDescriptor::CheckConfig() {
  // Check that the value of tuple_size is between 2 and 4, and that
  // patch_size is greater than zero.
  if (params_.tuple_size < 2 || params_.tuple_size > 4) {
    BEAM_CRITICAL("tuple_size_ is not an acceptable value!");
    throw std::invalid_argument("tuple_size is not an acceptable value!");
  } else if (params_.patch_size <= 0) {
    BEAM_CRITICAL("patch_size_ is less than/ equal to zero!");
    throw std::invalid_argument("patch_size is less than/ equal to zero!");
  }
}

cv::Mat
    ORBDescriptor::ExtractDescriptors(const cv::Mat& image,
                                      std::vector<cv::KeyPoint>& keypoints) {
  cv::Mat descriptors;
  orb_descriptor_->compute(image, keypoints, descriptors);
  return descriptors;
}
} // namespace beam_cv
