/** @file
 * @ingroup cv
 */

#pragma once

#include <beam_cv/matchers/Matcher.h>

namespace beam_cv {

/** brute force matcher */
class BFMatcher : public Matcher {
public:
  struct Params {
    int norm_type = cv::NORM_L2;
    bool cross_check = false;
    bool auto_remove_outliers = true;
    double ratio_threshold = 0.8;
    int distance_threshold = 5;
    bool use_knn = true;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  BFMatcher(const Params& params);

  /**
   * @brief Custom constructor. The user can also specify their own params or
   * use default.
   */
  BFMatcher(int norm_type = cv::NORM_L2, bool cross_check = false,
            bool auto_remove_outliers = true, double ratio_threshold = 0.8,
            int distance_threshold = 5, bool use_knn = true);

  /**
   * @brief Default destructor
   */
  ~BFMatcher() override = default;

  /** Remove outliers between matches using epipolar constraints
   *
   * @param matches the unfiltered matches computed from two images
   * @param keypoints_1 the keypoints from the first image
   * @param keypoints_2 the keypoints from the second image
   *
   * @return the filtered matches
   */
  std::vector<cv::DMatch> RemoveOutliers(
      const std::vector<cv::DMatch>& matches,
      const std::vector<cv::KeyPoint>& keypoints_1,
      const std::vector<cv::KeyPoint>& keypoints_2) const override;

  /** Matches keypoints descriptors between two images using the
   *  derived matcher.
   *  @param descriptors_1 the descriptors extracted from the first image.
   *  @param descriptors_2 the descriptors extracted from the second image.
   *  @param keypoints_1 the keypoints detected in the first image
   *  @param keypoints_2 the keypoints detected in the second image
   *  @param mask
   *  \parblock indicates which descriptors can be matched between the two
   *  sets. As per OpenCV docs "queryDescriptors[i] can be matched with
   *  trainDescriptors[j] only if masks.at<uchar>(i,j) is non-zero. In the
   *  libwave wrapper, queryDescriptors are descriptors_1, and
   *  trainDescriptors are descriptors_2. Default is cv::noArray().
   *  \endparblock
   *  @return vector containing the best matches.
   */
  std::vector<cv::DMatch>
      MatchDescriptors(cv::Mat& descriptors_1, cv::Mat& descriptors_2,
                       const std::vector<cv::KeyPoint>& keypoints_1,
                       const std::vector<cv::KeyPoint>& keypoints_2,
                       cv::InputArray mask = cv::noArray()) override;

private:
  Params params_;
  cv::Ptr<cv::BFMatcher> bf_matcher_;

  void Setup();

  /** @brief Remove outliers between matches. Uses a heuristic based approach as
   * a first pass to determine good matches.
   *  @param matches the unfiltered matches computed from two images.
   *  @return the filtered matches.
   */
  std::vector<cv::DMatch>
      FilterMatches(const std::vector<cv::DMatch>& matches) const override;

  /** @brief First pass to filter bad matches. Takes in a vector of matches and
   * uses the ratio test to filter the matches.
   *  @param matches the unfiltered matches computed from two images.
   *  @return the filtered matches.
   */
  std::vector<cv::DMatch> FilterMatches(
      const std::vector<std::vector<cv::DMatch>>& matches) const override;
};

} // namespace beam_cv
