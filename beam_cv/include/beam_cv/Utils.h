/** @file
 * @ingroup cv
 */

#pragma once
#include <opencv2/opencv.hpp>

#include <cstdlib>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/math.hpp>

namespace beam_cv {

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
std::vector<Eigen::Vector2d>
    ConvertKeypoints(const std::vector<cv::KeyPoint>& keypoints);

/** @brief Convert a vector of cv::Point2f to a vector of Eigen::Vector2d
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<Eigen::Vector2d>
    ConvertKeypoints(const std::vector<cv::Point2f>& keypoints);

/** @brief Convert a vector of Eigen::Vector2d to a vector of cv::Point2f
 * @param keypoints input keypoints
 * @param vec_keypoints converted keypoints
 */
std::vector<cv::Point2f>
    ConvertKeypoints(const std::vector<Eigen::Vector2d>& keypoints);

/**
 * @brief computes number of inliers projections
 * @param camR camera model for image 1
 * @param camC camera model for image 2
 * @param xs corresponding pixels in image 1
 * @param xss corresponding pixels in image 2
 * @param T_camR_world transform to camera R
 * @param T_camC_world transform to camera C
 */
int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> camR,
                 std::shared_ptr<beam_calibration::CameraModel> camC,
                 std::vector<Eigen::Vector2i> pr_v,
                 std::vector<Eigen::Vector2i> pc_v,
                 Eigen::Matrix4d T_camR_world, Eigen::Matrix4d T_camC_world,
                 double inlier_threshold);

/**
 * @brief computes number of inliers projections
 * @param cam camera model for image
 * @param points 3d points
 * @param pixels associated pixels
 * @param T_cam_world transform to camera
 */
int CheckInliers(std::shared_ptr<beam_calibration::CameraModel> cam,
                 std::vector<Eigen::Vector3d> points,
                 std::vector<Eigen::Vector2i> pixels,
                 Eigen::Matrix4d T_cam_world, double inlier_threshold);

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
