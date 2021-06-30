#include <beam_cv/detectors/SIFTDetector.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void SIFTDetector::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for SIFT detector params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  num_features = J["num_features"];
  n_octave_layers = J["n_octave_layers"];
  contrast_threshold = J["contrast_threshold"];
  edge_threshold = J["edge_threshold"];
  sigma = J["sigma"];
}

SIFTDetector::SIFTDetector(const Params& params) : params_(params) {Setup();};

SIFTDetector::SIFTDetector(int num_features, int n_octave_layers,
                           double contrast_threshold, double edge_threshold,
                           double sigma) {
  params_.num_features = num_features;
  params_.n_octave_layers = n_octave_layers;
  params_.contrast_threshold = contrast_threshold;
  params_.edge_threshold = edge_threshold;
  params_.sigma = sigma;
  Setup();
}

void SIFTDetector::Setup(){
  // Ensure parameters are valid
  CheckConfig();

  sift_detector_ = cv::SIFT::create(
      params_.num_features, params_.n_octave_layers,
      params_.contrast_threshold, params_.edge_threshold, params_.sigma);
}

std::vector<cv::KeyPoint> SIFTDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  // Detect features in image and return keypoints.
  sift_detector_->detect(image, keypoints);
  return keypoints;
}

void SIFTDetector::CheckConfig() {
  return;
}

} // namespace beam_cv
