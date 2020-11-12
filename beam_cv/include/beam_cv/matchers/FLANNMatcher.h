/** @file
 * @ingroup cv
 */

#pragma once

#include <string>
#include <vector>

#include <beam_cv/matchers/Matcher.h>

namespace beam_cv {

/**
 * @brief Enum class for different FLANN types
 */
enum class FLANN { KDTree = 1, KMeans = 2, Composite = 3, LSH = 4 };

/** Representation of a descriptor matcher using the FLANN algorithm.
 *
 *  Internally, this class is wrapping OpenCV's FLANNBasedMatcher module.
 *  Further reference on the BFMatcher can be found here:
 *  http://docs.opencv.org/trunk/dc/de2/classcv_1_1FlannBasedMatcher.html
 */
class FLANNMatcher : public Matcher {
public:
  /** Default constructor. The user can also specify their own struct with
   *  desired values. If no struct is provided, default values are used.
   *
   */
  FLANNMatcher(const int flann_method = FLANN::KDTree,
               const double ratio_threshold = 0.8,
               const bool auto_remove_outliers = true,
               const bool use_knn = true, const int fm_method = cv::FM_RANSAC,
               const int distance_threshold = 5);

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
   *  FLANNMatcher.
   *
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
   *
   *  @return vector containing the best matches.
   */
  std::vector<cv::DMatch>
      MatchDescriptors(cv::Mat& descriptors_1, cv::Mat& descriptors_2,
                       const std::vector<cv::KeyPoint>& keypoints_1,
                       const std::vector<cv::KeyPoint>& keypoints_2,
                       cv::InputArray mask = cv::noArray()) override;

private:
  /** The pointer to the wrapped cv::FlannBasedMatcher object */
  cv::Ptr<cv::FlannBasedMatcher> flann_matcher_;

  /** Checks whether the desired configuration is valid
   *
   *  @param check_config containing the desired configuration values.
   */
  void CheckConfig();

  /** Remove outliers between matches. Uses a heuristic based approach as a
   *  first pass to determine good matches.
   *
   *  @param matches the unfiltered matches computed from two images.
   *
   *  @return the filtered matches.
   */
  std::vector<cv::DMatch>
      FilterMatches(const std::vector<cv::DMatch>& matches) const override;

  /** First pass to filter bad matches. Takes in a vector of matches and uses
   *  the ratio test to filter the matches.
   *
   *  @param matches the unfiltered matches computed from two images.
   *
   *  @return the filtered matches.
   */
  std::vector<cv::DMatch> FilterMatches(
      const std::vector<std::vector<cv::DMatch>>& matches) const override;

  /** The FLANN method to use (described in the FLANN namespace). As a note,
   *  currently selecting a method will set up the FLANN matcher with
   *  default parameters.
   *
   *  Options:
   *  FLANN::KDTree: kd-tree separation
   *  FLANN::KMeans: k-means clustering.
   *  FLANN::Composite: Combines the above methods.
   *  FLANN::LSH: Locality-sensitive hash table separation.
   *
   *  Recommended: FLANN::KDTree
   */
  int flann_method_ = FLANN::KDTree;

  /** Determines whether to use a k-nearest-neighbours match.
   *
   *  Matcher can conduct a knn match with the best 2 matches for each
   *  descriptor. This uses the ratio test (@param ratio_threshold)
   *  to discard outliers.
   *
   *  If false, the matcher uses a distance heuristic
   *  (@param distance_threshold) to discard poor matches. This also
   *  incorporates cross checking between matches.
   *
   *  Recommended: true.
   */
  bool use_knn_ = true;

  /** Specifies heuristic for the ratio test, illustrated by Dr. David G. Lowe
   *  in his paper _Distinctive Image Features from Scale-Invariant Keypoints_
   *  (2004). The test takes the ratio of the closest keypoint distance
   *  to that of the second closest neighbour. If the ratio is less than
   *  the heuristic, it is discarded.
   *
   *  A value of 0.8 was shown by Dr. Lowe to reject 90% of the false matches,
   *  and discard only 5% of the correct matches.
   *
   *  Recommended: 0.8. Must be between 0 and 1.
   */
  double ratio_threshold_ = 0.8;

  /** Specifies the distance threshold for good matches.
   *
   *  Matches will only be kept if the descriptor distance is less than or
   *  equal to the product of the distance threshold and the _minimum_ of all
   *  descriptor distances. The greater the value, the more matches will
   *  be kept.
   *
   *  Recommended: 5. Must be greater than or equal to zero.
   */
  int distance_threshold_ = 5;

  /** Determines whether to automatically remove outliers using the method
   *  described in fm_method.
   *
   *  If true, the wave::BruteForceMatcher::matchDescriptors method will
   *  automatically call the wave::BruteForceMatcher::removeOutliers method,
   *  which uses the method in wave::BFMatcherParams::fm_method to remove
   *  matched outliers.
   *
   *  If false, the matches returned will only have been passed through the
   *  distance threshold or ratio tests described above.
   *
   *  Recommended: True
   */
  bool auto_remove_outliers_ = true;

  /** Method to find the fundamental matrix and remove outliers.
   *
   *  Options:
   *  cv::FM_7POINT: 7-point algorithm
   *  cv::FM_8POINT: 8-point algorithm
   *  cv::FM_LMEDS : least-median algorithm
   *  cv::FM_RANSAC: RANSAC algorithm
   *
   *  Recommended: cv::FM_RANSAC.
   */
  int fm_method_ = cv::FM_RANSAC;
};

} // namespace beam_cv
