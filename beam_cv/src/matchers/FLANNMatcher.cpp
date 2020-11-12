#include <beam_cv/matchers/FLANNMatcher.h>

FLANNMatcher::FLANNMatcher(const int flann_method, const double ratio_threshold,
                           const bool auto_remove_outliers, const bool use_knn,
                           const int fm_method, const int distance_threshold) {
  this->flann_method_ = flann_method;
  this->use_knn_ = use_knn;
  this->ratio_threshold_ = ratio_threshold;
  this->distance_threshold_ = distance_threshold;
  this->auto_remove_outliers_ = auto_remove_outliers;
  this->fm_method_ = fm_method;
  // Check flann_method and create the appropriate parameters struct.
  this->CheckConfig();

  // Depending on the FLANN method, different parameters are required.
  if (this->flann_method_ == FLANN::KDTree) {
    // Create FLANN matcher with default KDTree and Search params.
    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KDTreeIndexParams>(),
                                  cv::makePtr<cv::flann::SearchParams>());
    this->flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  } else if (this->flann_method_ == FLANN::KMeans) {
    // Create FLANN matcher with default KMeans and Search params
    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::KMeansIndexParams>(),
                                  cv::makePtr<cv::flann::SearchParams>());
    this->flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  } else if (this->flann_method_ == FLANN::Composite) {
    // Create FLANN matcher with default Composite and Search params
    cv::FlannBasedMatcher matcher(
        cv::makePtr<cv::flann::CompositeIndexParams>(),
        cv::makePtr<cv::flann::SearchParams>());
    this->flann_matcher = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  } else if (this->flann_method_ == FLANN::LSH) {
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
    this->flann_matcher_ = cv::makePtr<cv::FlannBasedMatcher>(matcher);
  }
}

std::vector<cv::DMatch>
    FLANNMatcher::MatchDescriptors(cv::Mat& descriptors_1,
                                   cv::Mat& descriptors_2,
                                   const std::vector<cv::KeyPoint>& keypoints_1,
                                   const std::vector<cv::KeyPoint>& keypoints_2,
                                   cv::InputArray mask = cv::noArray()) {
  std::vector<cv::DMatch> filtered_matches;
  // The FLANN matcher (except for the LSH method) requires the descriptors
  // to be of type CV_32F (float, from 0-1.0). Some descriptors
  // (ex. ORB, BRISK) provide descriptors in the form of CV_8U (unsigned int).
  // To use the other methods, the descriptor must be converted before
  // matching.
  if (this->flann_method_ != FLANN::LSH && descriptors_1.type() != CV_32F) {
    descriptors_1.convertTo(descriptors_1, CV_32F);
  }
  if (this->flann_method_ != FLANN::LSH && descriptors_2.type() != CV_32F) {
    descriptors_2.convertTo(descriptors_2, CV_32F);
  }
  if (this->use_knn_) {
    std::vector<std::vector<cv::DMatch>> raw_matches;
    // Number of neighbours for the k-nearest neighbour search. Only used
    // for the ratio test, therefore only want 2.
    int k = 2;
    this->flann_matcher_->knnMatch(descriptors_1, descriptors_2, raw_matches, k,
                                   mask, false);
    filtered_matches = this->FilterMatches(raw_matches);
  } else {
    std::vector<cv::DMatch> raw_matches;
    // Determine matches between sets of descriptors
    this->flann_matcher_->match(descriptors_1, descriptors_2, raw_matches,
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
  fundamental_matrix = cv::findFundamentalMat(fp1, fp2, this->fm_method_,
                                              fm_param_1, fm_param_2, mask);
  // Only retain the inliers matches
  for (size_t i = 0; i < mask.size(); i++) {
    if (mask.at(i) != 0) { good_matches.push_back(matches.at(i)); }
  }
  return good_matches;
}

void FLANNMatcher::CheckConfig() {
  // Check that the value of flann_method is one of the valid values.
  if (this->flann_method_ < FLANN::KDTree || this->flann_method_ > FLANN::LSH) {
    throw std::invalid_argument("Flann method selected does not exist!");
  }
  // Check the value of the ratio_test heuristic
  if (this->ratio_threshold_ < 0.0 || this->ratio_threshold_ > 1.0) {
    throw std::invalid_argument("ratio_threshold is not an appropriate value!");
  }
  // Check the value of the threshold distance heuristic
  if (this->distance_threshold_ < 0) {
    throw std::invalid_argument("distance_threshold is a negative value!");
  }
  // Only acceptable values are 1, 2, 4, and 8
  if (this->fm_method_ != cv::FM_7POINT && this->fm_method_ != cv::FM_8POINT &&
      this->fm_method_ != cv::FM_LMEDS && this->fm_method_ != cv::FM_RANSAC) {
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
    if (match.distance <=
        this->current_config.distance_threshold * min_distance) {
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
    if (ratio <= this->current_config.ratio_threshold) {
      filtered_matches.push_back(match[0]);
    }
  }
  return filtered_matches;
}
