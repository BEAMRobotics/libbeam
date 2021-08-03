#include <beam_cv/matchers/FLANNMatcher.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace beam_cv {

void FLANNMatcher::Params::LoadFromJson(const std::string& config_path) {
  if (config_path.empty()) { return; }

  if (!boost::filesystem::exists(config_path)) {
    BEAM_ERROR("Invalid file path for FLANN matcher params, using default "
               "params. Input: {}",
               config_path);
    return;
  }

  nlohmann::json J;
  std::ifstream file(config_path);
  file >> J;

  ratio_threshold = J["ratio_threshold"];
  auto_remove_outliers = J["auto_remove_outliers"];
  use_knn = J["use_knn"];

  std::string fm_method_str = J["fm_method"];
  if (fm_method_str == "FM_RANSAC") {
    fm_method = cv::FM_RANSAC;
  } else if (fm_method_str == "FM_7POINT") {
    fm_method = cv::FM_7POINT;
  } else if (fm_method_str == "FM_8POINT") {
    fm_method = cv::FM_8POINT;
  } else if (fm_method_str == "FM_LMEDS") {
    fm_method = cv::FM_LMEDS;
  } else {
    BEAM_ERROR("Invalid fm_method param given to FLANNMatcher. Using default: "
               "FM_RANSAC");
    fm_method = cv::FM_RANSAC;
  }

  distance_threshold = J["distance_threshold"];
}

FLANNMatcher::FLANNMatcher(const Params& params) : params_(params) {
  Setup();
};

FLANNMatcher::FLANNMatcher(int flann_method, double ratio_threshold,
                           bool auto_remove_outliers, bool use_knn,
                           int fm_method, int distance_threshold) {
  params_.flann_method = flann_method;
  params_.use_knn = use_knn;
  params_.ratio_threshold = ratio_threshold;
  params_.distance_threshold = distance_threshold;
  params_.auto_remove_outliers = auto_remove_outliers;
  params_.fm_method = fm_method;
  Setup();
}

void FLANNMatcher::Setup() {
  // Check flann_method and create the appropriate parameters struct.
  CheckConfig();

  // Depending on the FLANN method, different parameters are required.
  if (params_.flann_method == FLANN::KDTree) {
    // Create FLANN matcher with default KDTree and Search params.
    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KDTreeIndexParams>(),
                                  cv::makePtr<cv::flann::SearchParams>());
    flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  } else if (params_.flann_method == FLANN::KMeans) {
    // Create FLANN matcher with default KMeans and Search params
    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KMeansIndexParams>(),
                                  cv::makePtr<cv::flann::SearchParams>());
    flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  } else if (params_.flann_method == FLANN::Composite) {
    // Create FLANN matcher with default Composite and Search params
    cv::FlannBasedMatcher matcher(
        cv::makePtr<cv::flann::CompositeIndexParams>(),
        cv::makePtr<cv::flann::SearchParams>());
    flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  } else if (params_.flann_method == FLANN::LSH) {
    // Create LSH params with default values. These are values recommended
    // by Kaehler and Bradski - the LSH struct in OpenCV does not have
    // default values unlike the others.
    unsigned int num_tables = 20; // Typically between 10-30
    unsigned int key_size = 15;   // Typically between 10-20
    unsigned int multi_probe_level = 2;
    // Create FLANN matcher with default LSH and Search params
    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(
                                      num_tables, key_size, multi_probe_level),
                                  cv::makePtr<cv::flann::SearchParams>());
    flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  }
}

std::vector<cv::DMatch> FLANNMatcher::MatchDescriptors(
    cv::Mat& descriptors_1, cv::Mat& descriptors_2,
    const std::vector<cv::KeyPoint>& keypoints_1,
    const std::vector<cv::KeyPoint>& keypoints_2, cv::InputArray mask) {
  std::vector<cv::DMatch> filtered_matches;
  // The FLANN matcher (except for the LSH method) requires the descriptors
  // to be of type CV_32F (float, from 0-1.0). Some descriptors
  // (ex. ORB, BRISK) provide descriptors in the form of CV_8U (unsigned int).
  // To use the other methods, the descriptor must be converted before
  // matching.
  if (params_.flann_method != FLANN::LSH && descriptors_1.type() != CV_32F) {
    descriptors_1.convertTo(descriptors_1, CV_32F);
  }
  if (params_.flann_method != FLANN::LSH && descriptors_2.type() != CV_32F) {
    descriptors_2.convertTo(descriptors_2, CV_32F);
  }
  if (params_.use_knn) {
    std::vector<std::vector<cv::DMatch>> raw_matches;
    // Number of neighbours for the k-nearest neighbour search. Only used
    // for the ratio test, therefore only want 2.
    int k = 2;
    flann_matcher_->knnMatch(descriptors_1, descriptors_2, raw_matches, k, mask,
                             false);
    filtered_matches = FilterMatches(raw_matches);
  } else {
    std::vector<cv::DMatch> raw_matches;
    // Determine matches between sets of descriptors
    flann_matcher_->match(descriptors_1, descriptors_2, raw_matches, mask);
    filtered_matches = FilterMatches(raw_matches);
  }
  if (params_.auto_remove_outliers) {
    std::vector<cv::DMatch> good_matches =
        RemoveOutliers(filtered_matches, keypoints_1, keypoints_2);
    return good_matches;
  }
  return filtered_matches;
}

std::vector<cv::DMatch> FLANNMatcher::RemoveOutliers(
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
  fundamental_matrix = cv::findFundamentalMat(fp1, fp2, params_.fm_method,
                                              fm_param_1, fm_param_2, mask);

  // Only retain the inliers matches
  for (size_t i = 0; i < mask.size(); i++) {
    if (mask.at(i) != 0) { good_matches.push_back(matches.at(i)); }
  }
  return good_matches;
}

void FLANNMatcher::CheckConfig() {
  // Check that the value of flann_method is one of the valid values.
  if (params_.flann_method < FLANN::KDTree ||
      params_.flann_method > FLANN::LSH) {
    throw std::invalid_argument("Flann method selected does not exist!");
  }
  // Check the value of the ratio_test heuristic
  if (params_.ratio_threshold < 0.0 || params_.ratio_threshold > 1.0) {
    throw std::invalid_argument("ratio_threshold is not an appropriate value!");
  }
  // Check the value of the threshold distance heuristic
  if (params_.distance_threshold < 0) {
    throw std::invalid_argument("distance_threshold is a negative value!");
  }
  // Only acceptable values are 1, 2, 4, and 8
  if (params_.fm_method != cv::FM_7POINT &&
      params_.fm_method != cv::FM_8POINT && params_.fm_method != cv::FM_LMEDS &&
      params_.fm_method != cv::FM_RANSAC) {
    throw std::invalid_argument("fm_method is not an acceptable value!");
  }
}

std::vector<cv::DMatch>
    FLANNMatcher::FilterMatches(const std::vector<cv::DMatch>& matches) const {
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

std::vector<cv::DMatch> FLANNMatcher::FilterMatches(
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