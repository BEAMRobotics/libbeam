/** @file
 * @ingroup cv
 */

#pragma once
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/descriptors/Descriptor.h>
#include <beam_cv/detectors/Detector.h>
#include <beam_cv/matchers/Matcher.h>
#include <beam_utils/math.h>
#include <beam_utils/optional.h>

namespace beam_cv {

void GetScreenResolution(int& width, int& height);

/**
 * @brief Method to perform histogram equalization
 * @param input image
 * @return histogram equalized image
 */
cv::Mat AdaptiveHistogram(const cv::Mat&);

/**
 * @brief Method for performing k means on image
 * @return k mean image
 * @param input image
 * @param K number of clusters
 */
cv::Mat KMeans(const cv::Mat&, int);

/**
 * @brief Method for extracting skeleton of objects in scene
 * @return binary image of 1 pixel wide skeleton
 * @param input_image binary image of detected objects (e.g. crack mask)
 */
cv::Mat ExtractSkeleton(const cv::Mat& input_image);

/**
 * @brief Method for removing unconnected components below certain threshold
 * size threshold
 * @return binary image of reduced noise skeleton image
 * @param input_image binary image
 * @param threshold size threshold for removing pixel clusters
 */
cv::Mat RemoveClusters(const cv::Mat& input_image, int threshold);

/**
 * @brief Method for segmenting connected components into seperate mat objects
 * @return vector of mat objects, each represeting a different connected
 *         component
 * @param image a binary image of conncected components (e.g. cracks)
 */
std::vector<cv::Mat> SegmentComponents(const cv::Mat& image);

/**
 * @brief Finds connected components for grayscale image of arbitrary color
 * depth
 * @return vector of sets
 * @param image
 */
std::map<int, std::vector<cv::Point2i>>
    ConnectedComponents(const cv::Mat& image);

/** @brief Convert a single cv::KeyPoint to Eigen::Vector2d
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
Eigen::Vector2d ConvertKeypoint(const cv::KeyPoint& keypoint);

/** @brief Convert a single cv::Point2f to Eigen::Vector2d
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
Eigen::Vector2d ConvertKeypoint(const cv::Point2f& keypoint);

/** @brief Convert a single Eigen::Vector2d to cv::Point2f
 * @param keypoint input keypoint
 * @param vec_keypoints converted keypoint
 */
cv::Point2f ConvertKeypoint(const Eigen::Vector2d& keypoint);

/** @brief Convert a vector of cv::KeyPoint to a vector of Eigen::Vector2d
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<Eigen::Vector2d, beam::AlignVec2d>
    ConvertKeypoints(const std::vector<cv::KeyPoint>& keypoints);

/** @brief Convert a vector of cv::Point2f to a vector of Eigen::Vector2d
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<Eigen::Vector2d, beam::AlignVec2d>
    ConvertKeypoints(const std::vector<cv::Point2f>& keypoints);

/** @brief Convert a vector of Eigen::Vector2d to a vector of cv::Point2f
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<cv::Point2f> ConvertKeypoints(
    const std::vector<Eigen::Vector2d, beam::AlignVec2d>& keypoints);

/**
 * @brief computes number of inliers projections
 *
 * Note: Using this for relative poses, assume T_cam1_world is identity, and
 * T_cam2_world is the transform from cam1 to cam2
 * @param cam1 camera model for image 1
 * @param cam2 camera model for image 2
 * @param p1_v corresponding pixels in image 1
 * @param p2_v corresponding pixels in image 2
 * @param T_cam1_world transform to camera 1
 * @param T_cam2_world transform to camera 2
 * @param inlier_threshold
 */
int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam1,
                 std::shared_ptr<beam_calibration::CameraModel> cam2,
                 const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p1_v,
                 const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p2_v,
                 const Eigen::Matrix4d& T_cam1_world,
                 const Eigen::Matrix4d& T_cam2_world, double inlier_threshold);

/**
 * @brief computes number of inliers projections
 *
 * Note: Using this for relative poses, assume T_cam1_world is identity, and
 * T_cam2_world is the transform from cam1 to cam2. In this case points will be
 * in cam1's frame of reference
 * @param cam1 camera model for image 1
 * @param cam2 camera model for image 2
 * @param p1_v corresponding pixels in image 1
 * @param p2_v corresponding pixels in image 2
 * @param points triangulated pixels in world frame
 * @param T_cam1_world transform to camera 1
 * @param T_cam2_world transform to camera 2
 * @param inlier_threshold
 */
int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam1,
                 std::shared_ptr<beam_calibration::CameraModel> cam2,
                 const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p1_v,
                 const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p2_v,
                 const std::vector<Eigen::Vector3d, beam::AlignVec3d>& points,
                 const Eigen::Matrix4d& T_cam1_world,
                 const Eigen::Matrix4d& T_cam2_world, double inlier_threshold);

/**
 * @brief computes number of inliers projections
 * @param cam camera model for image
 * @param points 3d points
 * @param pixels associated pixels
 * @param T_cam_world transform to camera
 */
int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam,
                 const std::vector<Eigen::Vector3d, beam::AlignVec3d>& points,
                 const std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
                 const Eigen::Matrix4d& T_cam_world, double inlier_threshold);

/**
 * @brief performs entire matching pipeline
 * @param imL left image
 * @param imR right image
 * @param descriptor ref to descriptor to use
 * @param detector ref to detector to use
 * @param matcher ref to matcher to use
 * @param pL_v vector to fill matches with in left image
 * @param pR_v vector to fill matches with in right image
 */
void DetectComputeAndMatch(
    cv::Mat imL, cv::Mat imR,
    const std::shared_ptr<beam_cv::Descriptor>& descriptor,
    const std::shared_ptr<beam_cv::Detector>& detector,
    const std::shared_ptr<beam_cv::Matcher>& matcher,
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pL_v,
    std::vector<Eigen::Vector2i, beam::AlignVec2i>& pR_v);

/**
 * @brief Detects features and computes descriptors using the
 * detector and descriptor.
 * @param image input image
 * @param descriptor ref to descriptor to use
 * @param detector ref to detector to use
 * @param keypoints ref to keypoints to fill in
 * @param descriptors ref to descriptors to fill in
 */
void DetectAndCompute(const cv::Mat& image,
                      const std::shared_ptr<beam_cv::Descriptor>& descriptor,
                      const std::shared_ptr<beam_cv::Detector>& detector,
                      std::vector<cv::KeyPoint>& keypoints,
                      cv::Mat& descriptors);

/**
 * @brief computes median distance between matches
 * @param matches vector of matches
 * @param keypoints_1 corresponding keypoints in left image
 * @param keypoints_2 corresponding keypoints in right image
 */
double ComputeMedianMatchDistance(std::vector<cv::DMatch> matches,
                                  const std::vector<cv::KeyPoint>& keypoints_1,
                                  const std::vector<cv::KeyPoint>& keypoints_2);

/**
 * @brief Gets a list of pixels forming a circle around the given one with a
 * specified radius
 * @param center of circle
 * @param r radius of the circle
 */
std::vector<Eigen::Vector2i> GetCircle(Eigen::Vector2i center, uint32_t r,
                                       float threshold = 0.5);

/**
 * @brief Fits an ellipse to a set of 2d points
 * @param points input points
 */
Eigen::Matrix2d FitEllipse(std::vector<Eigen::Vector2d> points);

/**
 * @brief This class provides a simple yet efficient Union-Find data structure
 * which is helpful in finding disjoint sets in various datasets:
 * https://en.wikipedia.org/wiki/Disjoint-set_data_structure
 * Used in finding connected components in single channel images since opencv's
 * connected components algorithm is only implemented for binary images
 */
class UnionFind {
public:
  /**
   * @brief Creates UnionFind structure with n items
   */
  void Initialize(int n) {
    for (int i = 0; i < n; i++) {
      id_.push_back(i);
      rank_.push_back(0);
    }
  }

  /**
   * @brief Returns parent of set containing p
   */
  int FindSet(int p) {
    if (p != id_[p]) { id_[p] = this->FindSet(id_[p]); }
    return id_[p];
  }

  /**
   * @brief Performs set union on sets containing p and q
   */
  void UnionSets(int p, int q) {
    int i = this->FindSet(p);
    int j = this->FindSet(q);
    if (i != j) {
      if (rank_[i] < rank_[j]) {
        id_[i] = j;
      } else if (rank_[i] > rank_[j]) {
        id_[j] = i;
      } else {
        id_[j] = i;
        rank_[i] += 1;
      }
    }
  }

protected:
  std::vector<int> id_;
  std::vector<int> rank_;
};

} // namespace beam_cv
