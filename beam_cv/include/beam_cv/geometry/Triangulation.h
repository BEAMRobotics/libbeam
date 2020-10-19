/** @file
 * @ingroup cv
 */

#pragma once

#include <optional>

#include <Eigen/Dense>

#include <beam_calibration/CameraModel.h>

template <class T>
using opt = std::optional<T>;

namespace beam_cv {

/**
 * @brief static class implementing the point triangulation for any camera model
 */
class Triangulation {
public:
  /**
   * @brief Triangulates single point given two camera models and corresponding
   * pixel locations
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param Pr transformation matrix from world to image 1 frame
   * @param Pc transformation matrix from world to image 2 frame
   * @param pr pixel in image 1 to triangulate
   * @param pr pixel in image 2 to triangulate
   */
  static opt<Eigen::Vector3d>
      TriangulatePoint(std::shared_ptr<beam_calibration::CameraModel> camR,
                       std::shared_ptr<beam_calibration::CameraModel> camC,
                       Eigen::Matrix4d Pr, Eigen::Matrix4d Pc,
                       Eigen::Vector2i pr, Eigen::Vector2i pc);

  /**
   * @brief Triangulates a list of points given two camera models and
   * corresponding pixel locations
   * @param camR camera model for image 1
   * @param camC camera model for image 2
   * @param Pr transformation matrix from world to image 1 frame
   * @param Pc transformation matrix from world to image 2 frame
   * @param points list of correspondences to triangulate
   */
  static std::vector<opt<Eigen::Vector3d>>
      TriangulatePoints(std::shared_ptr<beam_calibration::CameraModel> camR,
                        std::shared_ptr<beam_calibration::CameraModel> camC,
                        Eigen::Matrix4d Pr, Eigen::Matrix4d Pc,
                        std::vector<Eigen::Vector2i> pr_v,
                        std::vector<Eigen::Vector2i> pc_v);
};

} // namespace beam_cv
