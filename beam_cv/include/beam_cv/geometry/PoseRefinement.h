/** @file
 * @ingroup cv
 */

#pragma once

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <beam_cv/Utils.h>
#include <beam_calibration/CameraModel.h>
#include <string>

static std::string default_string = std::string();

namespace beam_cv {

/**
 * @brief static class implementing pose refinement.
 */

class PoseRefinement {
public:
  /**
   * @brief Default constructor - ceres solver options are set to defaults.
   */
  PoseRefinement();

  /**
   * @brief Constructor for custom ceres solver options.
   * @param options Client's ceres solver options.
   */
  PoseRefinement(const ceres::Solver::Options options);

  /**
   * @brief Refines an estimated transformation matrix to minimize
   * reprojection error for a set of points/pixel correspondences.
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
                 const std::vector<Eigen::Vector2i, beam_cv::AlignVec2i>& pixels,
                 const std::vector<Eigen::Vector3d, beam_cv::AlignVec3d>& points,
                 std::string& report = default_string, bool remove_points_outside_domain = true);

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
