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
namespace FLANN {
enum { KDTree = 1, KMeans = 2, Composite = 3, LSH = 4 };
};

/** Representation of a descriptor matcher using the FLANN algorithm.
 *  Internally, this class is wrapping OpenCV's FLANNBasedMatcher module.
 *  Further reference on the BFMatcher can be found here:
 *  http://docs.opencv.org/trunk/dc/de2/classcv_1_1FlannBasedMatcher.html
 */
class FLANNMatcher : public Matcher {
public:
  struct Params {
    /** The FLANN method to use (described in the FLANN
     * namespace). As a note, currently selecting a method will set up the FLANN
     * matcher with default parameters.  Recommended: FLANN::KDTree.
     * NOTE: This cannot be loaded from a json*/
    int flann_method = FLANN::KDTree;

    /** Specifies heuristic for the ratio test, illustrated
     * by Dr. David G. Lowe in his paper _Distinctive Image Features from
     * Scale-Invariant Keypoints_ (2004). The test takes the ratio of the
     * closest keypoint distance to that of the second closest neighbour. If the
     * ratio is less than the heuristic, it is discarded. Reccomended: 0.8 */
    double ratio_threshold = 0.8;

    /** Determines whether to automatically remove
     * outliers using the method described in fm_method. */
    bool auto_remove_outliers = true;

    /** use k nearest neighbours */
    bool use_knn = true;

    /**  Method to find the fundamental matrix and remove outliers. */
    int fm_method = cv::FM_RANSAC;

    /** Specifies the distance threshold for good
     * matches. */
    int distance_threshold = 5;

    // load params from json. If empty, it will use default params
    void LoadFromJson(const std::string& config_path);
  };

  /**
   * @brief Constructor that requires a params object
   * @param params see struct above
   */
  FLANNMatcher(const Params& params);

  /** @brief Custom constructor. The user can also specify their own params or
   * use default.
   * @param flann_method The FLANN method to use (described in the FLANN
   * namespace). As a note, currently selecting a method will set up the FLANN
   * matcher with default parameters.  Recommended: FLANN::KDTree
   * @param ratio_threshold Specifies heuristic for the ratio test, illustrated
   * by Dr. David G. Lowe in his paper _Distinctive Image Features from
   * Scale-Invariant Keypoints_ (2004). The test takes the ratio of the closest
   * keypoint distance to that of the second closest neighbour. If the ratio is
   * less than the heuristic, it is discarded. Reccomended: 0.8
   * @param auto_remove_outliers Determines whether to automatically remove
   * outliers using the method described in fm_method.
   * @param use_knn
   * @param fm_method Method to find the fundamental matrix and remove outliers.
   * @param distance_threshold Specifies the distance threshold for good
   * matches.
   */
  FLANNMatcher(int flann_method = FLANN::KDTree, double ratio_threshold = 0.8,
               bool auto_remove_outliers = true, bool use_knn = true,
               int fm_method = cv::FM_RANSAC, int distance_threshold = 5);

  /**
   * @brief Default destructor
   */
  ~FLANNMatcher() override = default;

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
  void Setup();

  /** @brief Checks whether the desired configuration is valid
   *  @param check_config containing the desired configuration values.
   */
  void CheckConfig();

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

  Params params_;

  /** The pointer to the wrapped cv::FlannBasedMatcher object */
  cv::Ptr<cv::FlannBasedMatcher> flann_matcher_;
};

} // namespace beam_cv
