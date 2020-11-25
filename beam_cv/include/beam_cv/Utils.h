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

} // namespace beam_cv
