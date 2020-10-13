/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <beam_calibration/CameraModel.h>

#include <optional>

template <class T>
using opt = std::optional<T>;

namespace beam_cv {

/**
 * @brief static class implementing the 8 point algorithm with ransac
 */
class Triangulation {
public:
  /**
   * @brief Default constructor
   */
  Triangulation() = default;

  /**
   * @brief Default destructor
   */
  ~Triangulation() = default;

  /**
   * @brief Triangulates single point given two camera models and corresponding
   * pixel locations
   */
  static opt<Eigen::Vector3d>
      TriangulatePoint(std::shared_ptr<beam_calibration::CameraModel> camR,
                       std::shared_ptr<beam_calibration::CameraModel> camC,
                       Eigen::Matrix4d Pr, Eigen::Matrix4d Pc,
                       Eigen::Vector2i pr, Eigen::Vector2i pc);

  /**
   * @brief Triangulates list of corresponding points in each camera model
   */
  static opt<std::vector<Eigen::Vector3d>> TriangulatePoints(
      std::shared_ptr<beam_calibration::CameraModel> camR,
      std::shared_ptr<beam_calibration::CameraModel> camC, Eigen::Affine3d Pr,
      Eigen::Affine3d Pc,
      std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i>> points);
};

} // namespace beam_cv
