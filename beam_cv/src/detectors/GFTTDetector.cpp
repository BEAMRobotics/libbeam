#include <beam_cv/detectors/GFTTDetector.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void GFTTDetector::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for GFTT detector params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;
  num_features = J["num_features"];
  quality_level = J["quality_level"];
  min_distance = J["min_distance"];
  block_size = J["block_size"];
  use_harris_detector = J["use_harris_detector"];
  k = J["k"];
  grid_cols = J["grid_cols"];
  grid_rows = J["grid_rows"];
}

GFTTDetector::GFTTDetector(const Params& params)
    : Detector(params.grid_cols, params.grid_rows), params_(params) {
  Setup();
};

GFTTDetector::GFTTDetector(int num_features, double quality_level,
                           double min_distance, int block_size,
                           bool use_harris_detector, double k, int grid_cols,
                           int grid_rows) : Detector(grid_cols, grid_rows) {
  params_.num_features = num_features;
  params_.quality_level = quality_level;
  params_.min_distance = min_distance;
  params_.block_size = block_size;
  params_.use_harris_detector = use_harris_detector;
  params_.k = k;
  params_.grid_cols = grid_cols;
  params_.grid_rows = grid_rows;
  Setup();
}

void GFTTDetector::Setup() {
  int num_features_per_grid =
      params_.num_features / (params_.grid_cols * params_.grid_rows);

  GFTT_detector_ = cv::GFTTDetector::create(
      num_features_per_grid, params_.quality_level, params_.min_distance,
      params_.block_size, params_.use_harris_detector, params_.k);
}

std::vector<cv::KeyPoint>
    GFTTDetector::DetectLocalFeatures(const cv::Mat& image) {
  // Detect features in image and return keypoints.
  std::vector<cv::KeyPoint> keypoints;
  GFTT_detector_->detect(image, keypoints);
  return keypoints;
}

} // namespace beam_cv
