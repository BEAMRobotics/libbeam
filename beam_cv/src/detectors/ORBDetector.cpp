#include <beam_cv/detectors/ORBDetector.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void ORBDetector::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for ORB detector params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  num_features = J["num_features"];
  scale_factor = J["scale_factor"];
  num_levels = J["num_levels"];
  edge_threshold = J["edge_threshold"];
  std::string score_type_str = J["score_type"];
  if (score_type_str == "HARRIS_SCORE") {
    score_type = cv::ORB::HARRIS_SCORE;
  } else if (score_type_str == "FAST_SCORE") {
    score_type = cv::ORB::FAST_SCORE;
  } else {
    BEAM_ERROR("Invalid score type param given to ORBDetector. Using default: "
               "HARRIS_SCORE");
    score_type = cv::ORB::FAST_SCORE;
  }
  std::string fast_threshold = J["fast_threshold"];
}

ORBDetector::ORBDetector(const Params& params) : params_(params) {Setup();};

ORBDetector::ORBDetector(int num_features, float scale_factor, int num_levels,
                         int edge_threshold, cv::ORB::ScoreType score_type,
                         int fast_threshold) {
  params_.num_features = num_features;
  params_.scale_factor = scale_factor;
  params_.num_levels = num_levels;
  params_.edge_threshold = edge_threshold;
  params_.score_type = score_type;
  params_.fast_threshold = fast_threshold;
  Setup();
}

void ORBDetector::Setup(){
  // Ensure parameters are valid
  CheckConfig();

  // Default parameters that should not be modified, and are not associated
  // with the descriptor. These values are the defaults recommended by OpenCV.
  //-------------------------------------------------------------------------
  int first_level = 0; // As per OpenCV docs, first_level must be zero.
  int tuple_size = 2;
  int patch_size = 31;

  orb_detector_ = cv::ORB::create(params_.num_features, params_.scale_factor,
                                  params_.num_levels, params_.edge_threshold,
                                  first_level, tuple_size, params_.score_type,
                                  patch_size, params_.fast_threshold);
}

std::vector<cv::KeyPoint> ORBDetector::DetectFeatures(const cv::Mat& image) {
  std::vector<cv::KeyPoint> keypoints;
  // Detect features in image and return keypoints.
  orb_detector_->detect(image, keypoints);
  return keypoints;
}

void ORBDetector::CheckConfig() {
  // Check parameters. If invalid, throw an exception.
  if (params_.num_features < 0) {
    throw std::invalid_argument("num_features must be greater than/equal to 0");
  } else if (params_.scale_factor < 1.0) {
    throw std::invalid_argument(
        "scale_factor must be greater than/equal to 1.0!");
  } else if (params_.num_levels <= 0) {
    throw std::invalid_argument("num_levels must be greater than 0");
  } else if (params_.edge_threshold < 0) {
    throw std::invalid_argument(
        "edge_threshold must be greater than/equal to 0");
  } else if (params_.score_type < cv::ORB::HARRIS_SCORE ||
             params_.score_type > cv::ORB::FAST_SCORE) {
    throw std::invalid_argument("Invalid score_type for ORBDetector!");
  } else if (params_.fast_threshold <= 0) {
    throw std::invalid_argument("fast_threshold must be greater than 0");
  }
}

} // namespace beam_cv
