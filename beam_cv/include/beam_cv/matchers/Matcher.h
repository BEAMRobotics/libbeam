/** @file
 * @ingroup cv
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace beam_cv {

/** Representation of a generic keypoint detector
 */
class Matcher {
public:
  /**
   * @brief Default constructor
   */
  Matcher() = default;

  /**
   * @brief Default destructor
   */
  virtual ~Matcher() = default;

  /** Remove outliers between matches using epipolar constraints
   *
   * @param matches the unfiltered matches computed from two images
   * @param keypoints_1 the keypoints from the first image
   * @param keypoints_2 the keypoints from the second image
   *
   * @return the filtered matches
   */
  virtual std::vector<cv::DMatch>
      RemoveOutliers(const std::vector<cv::DMatch>& matches,
                     const std::vector<cv::KeyPoint>& keypoints_1,
                     const std::vector<cv::KeyPoint>& keypoints_2) const = 0;

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
  virtual std::vector<cv::DMatch>
      MatchDescriptors(cv::Mat& descriptors_1, cv::Mat& descriptors_2,
                       const std::vector<cv::KeyPoint>& keypoints_1,
                       const std::vector<cv::KeyPoint>& keypoints_2,
                       cv::InputArray mask = cv::noArray()) = 0;

protected:
  /** Filter matches using a heuristic based method.
   *
   *  If the distance between matches is less than the defined heuristic, it
   *  is rejected.
   *
   *  @param matches the unfiltered matches computed from two images.
   *
   *  @return the matches with outliers removed.
   */
  virtual std::vector<cv::DMatch>
      FilterMatches(const std::vector<cv::DMatch>& matches) const = 0;

  /** Overloaded method, which takes in a vector of a vector of matches. This
   *  is designed to be used with the knnMatchDescriptors method, and uses the
   *  ratio test to filter the matches.
   *
   *  @param matches the unfiltered matches computed from two images.
   *
   *  @return the filtered matches.
   */
  virtual std::vector<cv::DMatch> FilterMatches(
      const std::vector<std::vector<cv::DMatch>>& matches) const = 0;
};

} // namespace beam_cv