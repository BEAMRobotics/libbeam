#include <beam_cv/matchers/BFMatcher.h>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void BFMatcher::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for BF matcher params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;

  std::string norm_type_str = J["norm_type"];
  if (norm_type_str == "NORM_L2") {
    norm_type = cv::NORM_L2;
  } else if (norm_type_str == "NORM_INF") {
    norm_type = cv::NORM_INF;
  } else if (norm_type_str == "NORM_L1") {
    norm_type = cv::NORM_L1;
  } else if (norm_type_str == "NORM_L2SQR") {
    norm_type = cv::NORM_L2SQR;
  } else if (norm_type_str == "NORM_HAMMING") {
    norm_type = cv::NORM_HAMMING;
  } else if (norm_type_str == "NORM_HAMMING2") {
    norm_type = cv::NORM_HAMMING2;
  } else if (norm_type_str == "NORM_TYPE_MASK") {
    norm_type = cv::NORM_TYPE_MASK;
  } else if (norm_type_str == "NORM_RELATIVE") {
    norm_type = cv::NORM_RELATIVE;
  } else if (norm_type_str == "NORM_MINMAX") {
    norm_type = cv::NORM_MINMAX;
  } else {
    BEAM_ERROR(
        "Invalid norm_type param given to BFMatcher. Using default: NORM_L2");
    norm_type = cv::NORM_L2;
  }

  cross_check = J["cross_check"];
  auto_remove_outliers = J["auto_remove_outliers"];
  ratio_threshold = J["ratio_threshold"];
  distance_threshold = J["distance_threshold"];
  use_knn = J["use_knn"];
}

BFMatcher::BFMatcher(const Params& params) : params_(params) {
  Setup();
};

BFMatcher::BFMatcher(int norm_type, bool cross_check, bool auto_remove_outliers,
                     double ratio_threshold, int distance_threshold,
                     bool use_knn) {
  params_.norm_type = norm_type;
  params_.auto_remove_outliers = auto_remove_outliers;
  params_.cross_check = cross_check;
  params_.ratio_threshold = ratio_threshold;
  params_.distance_threshold = distance_threshold;
  params_.use_knn = use_knn;
  Setup();
}

void BFMatcher::Setup() {
  bf_matcher_ = cv::BFMatcher::create(params_.norm_type, params_.cross_check);
}

std::vector<cv::DMatch>
    BFMatcher::MatchDescriptors(cv::Mat& descriptors_1, cv::Mat& descriptors_2,
                                const std::vector<cv::KeyPoint>& keypoints_1,
                                const std::vector<cv::KeyPoint>& keypoints_2,
                                cv::InputArray mask) {
  std::vector<cv::DMatch> filtered_matches;
  if(keypoints_1.empty() || keypoints_2.empty()){
    BEAM_WARN("Empty keypoints vector provided, not matching descriptors.");
    return filtered_matches;
  }

  if (params_.use_knn) {
    std::vector<std::vector<cv::DMatch>> raw_matches;
    // Number of neighbours for the k-nearest neighbour search. Only used
    // for the ratio test, therefore only want 2.
    int k = 2;
    bf_matcher_->knnMatch(descriptors_1, descriptors_2, raw_matches, k, mask,
                          false);
                          
    filtered_matches = FilterMatches(raw_matches);
  } else {
    // Determine matches between sets of descriptors
    std::vector<cv::DMatch> raw_matches;
    bf_matcher_->match(descriptors_1, descriptors_2, raw_matches, mask);
    filtered_matches = FilterMatches(raw_matches);
  }

  if (params_.auto_remove_outliers) {
    std::vector<cv::DMatch> good_matches =
        RemoveOutliers(filtered_matches, keypoints_1, keypoints_2);
    return good_matches;
  }
  return filtered_matches;
}

std::vector<cv::DMatch> BFMatcher::RemoveOutliers(
    const std::vector<cv::DMatch>& matches,
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2) const {
  std::vector<cv::DMatch> good_matches;
  std::vector<cv::Point2f> fp1, fp2;
  // Take all good keypoints from matches, convert to cv::Point2f
  for (auto& match : matches) {
    fp1.push_back(keypoints_1.at((size_t)match.queryIdx).pt);
    fp2.push_back(keypoints_2.at((size_t)match.trainIdx).pt);
  }
  // Find fundamental matrix
  std::vector<uchar> mask;
  cv::Mat fundamental_matrix;
  // Maximum distance from a point to an epipolar line in pixels. Any points
  // further are considered outliers. Only used for RANSAC.
  double fm_param_1 = 3.0;
  // Desired confidence interval of the estimated fundamental matrix. Only
  // used for RANSAC or LMedS methods.
  double fm_param_2 = 0.99;
  fundamental_matrix = cv::findFundamentalMat(fp1, fp2, cv::FM_RANSAC,
                                              fm_param_1, fm_param_2, mask);
  // Only retain the inliers matches
  for (size_t i = 0; i < mask.size(); i++) {
    if (mask.at(i) != 0) { good_matches.push_back(matches.at(i)); }
  }
  return good_matches;
}

std::vector<cv::DMatch>
    BFMatcher::FilterMatches(const std::vector<cv::DMatch>& matches) const {
  std::vector<cv::DMatch> filtered_matches;
  // Determine closest match
  auto closest_match = std::min_element(matches.begin(), matches.end());
  auto min_distance = closest_match->distance;
  // Keep any match that is less than the rejection heuristic times minimum
  // distance
  for (auto& match : matches) {
    if (match.distance <= params_.distance_threshold * min_distance) {
      filtered_matches.push_back(match);
    }
  }
  return filtered_matches;
}

std::vector<cv::DMatch> BFMatcher::FilterMatches(
    const std::vector<std::vector<cv::DMatch>>& matches) const {
  std::vector<cv::DMatch> filtered_matches;
  for (auto& match : matches) {
    // Calculate ratio between two best matches. Accept if less than
    // ratio heuristic
    float ratio = match[0].distance / match[1].distance;
    if (ratio <= params_.ratio_threshold) {
      filtered_matches.push_back(match[0]);
    }
  }
  return filtered_matches;
}
}; // namespace beam_cv