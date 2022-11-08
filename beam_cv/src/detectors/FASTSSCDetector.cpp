#include <beam_cv/detectors/FASTSSCDetector.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void FASTSSCDetector::Params::LoadFromJson(const std::string& config_path) {
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
    BEAM_ERROR("Invalid type param given to FASTSSCDetector. Using default: "
               "TYPE_9_16");
    type = cv::FastFeatureDetector::TYPE_9_16;
  }
}

FASTSSCDetector::FASTSSCDetector(const Params& params)
    : Detector(0, 0), params_(params) {
  Setup();
};

FASTSSCDetector::FASTSSCDetector(int num_features, int threshold,
                                 bool nonmax_suppression,
                                 cv::FastFeatureDetector::DetectorType type)
    : Detector(0, 0) {
  params_.threshold = threshold;
  params_.nonmax_suppression = nonmax_suppression;
  params_.type = type;
  params_.num_features = num_features;
  Setup();
}

void FASTSSCDetector::Setup() {
  // Ensure parameters are valid
  CheckConfig();

  fast_detector_ =
      cv::FastFeatureDetector::create(params_.threshold, false, params_.type);
}

std::vector<cv::KeyPoint>
    FASTSSCDetector::DetectLocalFeatures(const cv::Mat& image) {
  // Detect features in image and return keypoints.
  std::vector<cv::KeyPoint> keypoints;
  fast_detector_->detect(image, keypoints);

  if (params_.nonmax_suppression) {
    keypoints = SSCNonmaxSuppression(keypoints, params_.num_features,
                                     image.cols, image.rows);
  } else if (params_.num_features != 0) {
    cv::KeyPointsFilter::retainBest(keypoints, params_.num_features);
    keypoints.resize(params_.num_features);
  }

  return keypoints;
}

void FASTSSCDetector::CheckConfig() {
  // Check parameters. If invalid, throw an exception.
  if (params_.threshold <= 0) {
    throw std::invalid_argument("threshold must be greater than 0!");
  } else if (params_.type < 0 || params_.type > 3) {
    throw std::invalid_argument("Invalid type for FASTSSCDetector!");
  } else if (params_.num_features < 0) {
    throw std::invalid_argument(
        "num_features must be greater than/equal to 0!");
  }
}

std::vector<cv::KeyPoint> FASTSSCDetector::SSCNonmaxSuppression(
    const std::vector<cv::KeyPoint>& keypoints, const int num_features,
    const int cols, const int rows) const {
  // several temp expression variables to simplify solution equation
  const int exp1 = rows + cols + 2 * num_features;
  const long long exp2 =
      ((long long)4 * cols + (long long)4 * num_features +
       (long long)4 * rows * num_features + (long long)rows * rows +
       (long long)cols * cols - (long long)2 * rows * cols +
       (long long)4 * rows * cols * num_features);
  const double exp3 = sqrt(exp2);
  const double exp4 = num_features - 1;

  const double sol1 = -round((exp1 + exp3) / exp4); // first solution
  const double sol2 = -round((exp1 - exp3) / exp4); // second solution

  // binary search range initialization with positive solution
  int high = (sol1 > sol2) ? sol1 : sol2;
  int low = std::floor(sqrt((double)keypoints.size() / num_features));
  low = std::max(1, low);

  int width;
  int prev_width = -1;

  std::vector<int> result_vec;
  bool complete = false;
  const unsigned int k_min =
      std::round(num_features - (num_features * params_.ssc_tolerance));
  const unsigned int k_max =
      std::round(num_features + (num_features * params_.ssc_tolerance));

  std::vector<int> result;
  result.reserve(keypoints.size());
  while (!complete) {
    width = low + (high - low) / 2;
    if (width == prev_width || low > high) {
      result_vec = result; // return the keypoints from the previous iteration
      break;
    }
    result.clear();
    int cur_kp_idx = 0;
    const double c = (double)width / 2.0; // initializing Grid
    const int num_cell_cols = std::floor(cols / c);
    const int num_cell_rows = std::floor(rows / c);
    std::vector<std::vector<bool>> covered_vec(
        num_cell_rows + 1, std::vector<bool>(num_cell_cols + 1, false));

    auto process_cell_nms = [&](const auto& kp) {
      cur_kp_idx++;
      const int row = std::floor(kp.pt.y / c);
      const int col = std::floor(kp.pt.x / c);
      if (covered_vec[row][col]) { return; }
      result.push_back(cur_kp_idx - 1);
      const int width_over_c = std::floor(width / c);
      // get range which current radius is covering
      const int row_min =
          ((row - width_over_c) >= 0) ? (row - width_over_c) : 0;
      const int row_max = ((row + width_over_c) <= num_cell_rows)
                              ? (row + width_over_c)
                              : num_cell_rows;
      const int col_min =
          ((col - width_over_c) >= 0) ? (col - width_over_c) : 0;
      const int col_max = ((col + width_over_c) <= num_cell_cols)
                              ? (col + width_over_c)
                              : num_cell_cols;
      for (int row_to_cov = row_min; row_to_cov <= row_max; ++row_to_cov) {
        for (int col_to_cov = col_min; col_to_cov <= col_max; ++col_to_cov) {
          if (!covered_vec[row_to_cov][col_to_cov]) {
            // cover cells within the square bounding box with width w
            covered_vec[row_to_cov][col_to_cov] = true;
          }
        }
      }
    };
    std::for_each(keypoints.begin(), keypoints.end(), process_cell_nms);

    if (result.size() >= k_min && result.size() <= k_max) { // solution found
      result_vec = result;
      complete = true;
    } else if (result.size() < k_min) {
      high = width - 1; // update binary search range
    } else {
      low = width + 1;
    }
    prev_width = width;
  }

  // retrieve final keypoints
  std::vector<cv::KeyPoint> kp;
  for (unsigned int i = 0; i < result_vec.size(); i++)
    kp.push_back(keypoints[result_vec[i]]);

  return kp;
}

} // namespace beam_cv
