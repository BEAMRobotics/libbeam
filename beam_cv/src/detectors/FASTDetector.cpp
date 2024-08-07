#include <beam_cv/detectors/FASTDetector.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void FASTDetector::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for FAST detector params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  num_features = J["num_features"];
  threshold = J["threshold"];
  nonmax_suppression = J["nonmax_suppression"];
  std::string type_str = J["type"];
  if (type_str == "TYPE_9_16") {
    type = cv::FastFeatureDetector::TYPE_9_16;
  } else if (type_str == "TYPE_7_12") {
    type = cv::FastFeatureDetector::TYPE_7_12;
  } else if (type_str == "TYPE_5_8") {
    type = cv::FastFeatureDetector::TYPE_5_8;
  } else {
    BEAM_ERROR(
        "Invalid type param given to FastDetector. Using default: TYPE_9_16");
    type = cv::FastFeatureDetector::TYPE_9_16;
  }
  grid_cols = J["grid_cols"];
  grid_rows = J["grid_rows"];
}

FASTDetector::FASTDetector(const Params& params)
    : Detector(params.grid_cols, params.grid_rows), params_(params) {
  Setup();
};

FASTDetector::FASTDetector(int num_features, int threshold,
                           bool nonmax_suppression,
                           cv::FastFeatureDetector::DetectorType type,
                           int grid_cols, int grid_rows)
    : Detector(grid_cols, grid_rows) {
  params_.threshold = threshold;
  params_.nonmax_suppression = nonmax_suppression;
  params_.type = type;
  params_.num_features = num_features;
  params_.grid_cols = grid_cols;
  params_.grid_rows = grid_rows;
  Setup();
}

void FASTDetector::Setup() {
  // Ensure parameters are valid
  CheckConfig();

  fast_detector_ = cv::FastFeatureDetector::create(
      params_.threshold, params_.nonmax_suppression, params_.type);
}

std::vector<cv::KeyPoint>
    FASTDetector::DetectLocalFeatures(const cv::Mat& image) {
  // Detect features in image and return keypoints.
  std::vector<cv::KeyPoint> keypoints;
  fast_detector_->detect(image, keypoints);

  // Retain best features since opencv fast does not have num features control.
  if (params_.num_features != 0) {
    int num_features_per_grid =
        params_.num_features / (params_.grid_cols * params_.grid_rows);
    cv::KeyPointsFilter::retainBest(keypoints, num_features_per_grid);
    // tied scores result in overages
    keypoints.resize(num_features_per_grid);
  }

  return keypoints;
}

void FASTDetector::CheckConfig() {
  // Check parameters. If invalid, throw an exception.
  if (params_.threshold <= 0) {
    throw std::invalid_argument("threshold must be greater than 0!");
  } else if (params_.type < 0 || params_.type > 3) {
    throw std::invalid_argument("Invalid type for FASTDetector!");
  } else if (params_.num_features < 0) {
    throw std::invalid_argument(
        "num_features must be greater than/equal to 0!");
  }
}

} // namespace beam_cv
