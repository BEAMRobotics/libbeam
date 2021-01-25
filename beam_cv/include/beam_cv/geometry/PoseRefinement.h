/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include <beam_calibration/CameraModel.h>

template <class T>
using opt = std::optional<T>;

using AlignVec2d = Eigen::aligned_allocator<Eigen::Vector2d>;
using AlignVec4d = Eigen::aligned_allocator<Eigen::Vector4d>;

namespace beam_cv {
/**
 * @brief static class implementing pose refinement.
 */
class PoseRefinement {
public:
  /**
   * @brief Refines an estimated transformation matrix to minimize reprojection
   * error for a set of points/pixel correspondences.
   * @param estimate Initial transformation matrix estimate
   * @param cam camera model for image
   * @param pixels projected pixel locations of feature points in cam
   * @param points 3d locations of features
   * @param report string to store ceres report
   * @returns Refined transformation matrix
   */
  Eigen::Matrix4d
      RefinePose(const Eigen::Matrix4d& estimate,
                 const std::shared_ptr<beam_calibration::CameraModel>& cam,
                 const std::vector<Eigen::Vector2i>& pixels,
                 const std::vector<Eigen::Vector3d>& points,
                 std::string& report);

private:
  /**
   * @brief Initializes a ceres problem object for the pose refinement.
   */
  std::shared_ptr<ceres::Problem> SetupCeresProblem();

  ceres::Solver::Options ceres_solver_options_;
  std::unique_ptr<ceres::LossFunction> loss_function_;
  std::unique_ptr<ceres::LocalParameterization> parameterization_;
};

} // namespace beam_cv
