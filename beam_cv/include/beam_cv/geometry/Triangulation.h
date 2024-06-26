/** @file
 * @ingroup cv
 */

#pragma once

#include <optional>

#include <Eigen/Dense>

#include <beam_calibration/CameraModel.h>
#include <beam_cv/Utils.h>
#include <beam_utils/optional.h>

namespace beam_cv {

/**
 * @brief static class implementing the point triangulation for any camera model
 */
class Triangulation {
public:
  /**
   * @brief Triangulates single point given two camera models and corresponding
   * pixel locations
   * @param cam1 camera model for image 1
   * @param cam2 camera model for image 2
   * @param T_cam1_world transformation matrix from world to image 1 frame
   * @param T_cam2_world transformation matrix from world to image 2 frame
   * @param p1 pixel in image 1 to triangulate [col, row]
   * @param p2 pixel in image 2 to triangulate [col, row]
   * @param max_dist maximum distance from the camera to accept as a valid
   * solution
   */
  static beam::opt<Eigen::Vector3d> TriangulatePoint(
      const std::shared_ptr<beam_calibration::CameraModel>& cam1,
      const std::shared_ptr<beam_calibration::CameraModel>& cam2,
      const Eigen::Matrix4d& T_cam1_world, const Eigen::Matrix4d& T_cam2_world,
      const Eigen::Vector2i& p1, const Eigen::Vector2i& p2,
      const double max_dist = 100, const double reprojection_threshold = -1);

  /**
   * @brief Triangulates single point given a single camera model and multiple
   * measurements
   * @param cam camera model
   * @param T_cam_world list of transforms form world to camera
   * @param pixels list of pixel locations in each camera image
   * @param max_dist maximum distance from the camera to accept as a valid
   * solution
   * @param reprojection_threshold pixel reprojection trheshold to accept a
   * solution (negative for no outlier checking)
   */
  static beam::opt<Eigen::Vector3d> TriangulatePoint(
      const std::shared_ptr<beam_calibration::CameraModel>& cam,
      const std::vector<Eigen::Matrix4d, beam::AlignMat4d>& T_cam_world,
      const std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
      const double max_dist = 100, const double reprojection_threshold = -1);

  /**
   * @brief Triangulates a list of points given two camera models and
   * corresponding pixel locations
   * @param cam1 camera model for image 1
   * @param cam2 camera model for image 2
   * @param T_cam1_world transformation matrix from world to image 1 frame
   * @param T_cam2_world transformation matrix from world to image 2 frame
   * @param p1_v list of correspondences to triangulate (image 1)
   * @param p2_v list of correspondences to triangulate (image 2)
   */
  static std::vector<beam::opt<Eigen::Vector3d>> TriangulatePoints(
      const std::shared_ptr<beam_calibration::CameraModel>& cam1,
      const std::shared_ptr<beam_calibration::CameraModel>& cam2,
      const Eigen::Matrix4d& T_cam1_world, const Eigen::Matrix4d& T_cam2_world,
      const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p1_v,
      const std::vector<Eigen::Vector2i, beam::AlignVec2i>& p2_v,
      const double max_dist = 100, const double reprojection_threshold = -1);
};

} // namespace beam_cv
