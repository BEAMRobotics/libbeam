#include <beam_cv/matchers/BFMatcher.h>

namespace beam_cv {

BFMatcher::BFMatcher(int norm_type, bool cross_check, bool auto_remove_outliers,
                     double ratio_threshold, int distance_threshold,
                     bool use_knn) {
  this->norm_type_ = norm_type;
  this->auto_remove_outliers_ = auto_remove_outliers;
  this->cross_check_ = cross_check;
  this->ratio_threshold_ = ratio_threshold;
  this->distance_threshold_ = distance_threshold;
  this->use_knn_ = use_knn;
  this->bf_matcher_ =
      cv::BFMatcher::create(this->norm_type_, this->cross_check_);
}

std::vector<cv::DMatch>
    BFMatcher::MatchDescriptors(cv::Mat& descriptors_1, cv::Mat& descriptors_2,
                                const std::vector<cv::KeyPoint>& keypoints_1,
                                const std::vector<cv::KeyPoint>& keypoints_2,
                                cv::InputArray mask) {
  std::vector<cv::DMatch> filtered_matches;

  if (this->use_knn_) {
    std::vector<std::vector<cv::DMatch>> raw_matches;
    // Number of neighbours for the k-nearest neighbour search. Only used
    // for the ratio test, therefore only want 2.
    int k = 2;
    this->bf_matcher_->knnMatch(descriptors_1, descriptors_2, raw_matches, k,
                                   mask, false);
    filtered_matches = this->FilterMatches(raw_matches);
  } else {
    std::vector<cv::DMatch> raw_matches;
    // Determine matches between sets of descriptors
    this->bf_matcher_->match(descriptors_1, descriptors_2, raw_matches,
                                mask);
    filtered_matches = this->FilterMatches(raw_matches);
  }

  if (this->auto_remove_outliers_) {
    std::vector<cv::DMatch> good_matches =
        this->RemoveOutliers(filtered_matches, keypoints_1, keypoints_2);
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
    if (match.distance <= this->distance_threshold_ * min_distance) {
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
    if (ratio <= this->ratio_threshold_) {
      filtered_matches.push_back(match[0]);
    }
  }
  return filtered_matches;
}
}; // namespace beam_cv