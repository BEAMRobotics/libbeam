#include "beam_cv/geometry/PoseRefinement.h"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/loss_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <beam_optimization/CamPoseReprojectionCost.h>

namespace beam_cv {

PoseRefinement::PoseRefinement() {
  // set ceres solver params
  ceres_solver_options_.minimizer_progress_to_stdout = false;
  ceres_solver_options_.max_num_iterations = 100;
  ceres_solver_options_.max_solver_time_in_seconds = 1e6;
  ceres_solver_options_.function_tolerance = 1e-8;
  ceres_solver_options_.gradient_tolerance = 1e-10;
  ceres_solver_options_.parameter_tolerance = 1e-8;
  ceres_solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;
}

PoseRefinement::PoseRefinement(const ceres::Solver::Options options) {
  ceres_solver_options_ = options;
}

Eigen::Matrix4d PoseRefinement::RefinePose(
    const Eigen::Matrix4d& estimate,
    const std::shared_ptr<beam_calibration::CameraModel>& cam,
    const std::vector<Eigen::Vector2i>& pixels,
    const std::vector<Eigen::Vector3d>& points, std::string& report) {
  // vector to store optimized pose quaternion and translation
  Eigen::Matrix3d estimate_r = estimate.block(0, 0, 3, 3);
  Eigen::Quaterniond estimate_q(estimate_r);
  std::vector<double> results{estimate_q.w(), estimate_q.x(), estimate_q.y(),
                              estimate_q.z(), estimate(0, 3), estimate(1, 3),
                              estimate(2, 3)};

  // use member function to instantiate solver
  std::shared_ptr<ceres::Problem> problem = SetupCeresProblem();

  // add parameter block for pose
  problem->AddParameterBlock(&(results[0]), 7, parameterization_.get());

  // add residual block for each correspondence
  for (size_t i = 0; i < points.size(); i++) {
    std::unique_ptr<ceres::CostFunction> cost_function(
        CeresReprojectionCostFunction::Create(pixels[i].cast<double>(),
                                              points[i], cam));

    problem->AddResidualBlock(cost_function.release(), loss_function_.get(),
                              &(results[0]));
  }

  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_solver_options_, problem.get(), &ceres_summary);
  report = ceres_summary.FullReport();

  // recover pose from optimization results
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q{results[0], results[1], results[2], results[3]};
  pose.block(0, 0, 3, 3) = q.toRotationMatrix();
  pose(0, 3) = results[4];
  pose(1, 3) = results[5];
  pose(2, 3) = results[6];
  return pose;
}

std::shared_ptr<ceres::Problem> PoseRefinement::SetupCeresProblem() {
  // set ceres problem options
  ceres::Problem::Options ceres_problem_options;

  // if we want to manage our own data for these, we can set these flags:
  ceres_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;

  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_problem_options);

  loss_function_ =
      std::unique_ptr<ceres::LossFunction>(new ceres::HuberLoss(1.0));

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));
  parameterization_ = std::unique_ptr<ceres::LocalParameterization>(
      new ceres::ProductParameterization(quat_parameterization.release(),
                                         identity_parameterization.release()));

  return problem;
}

} // namespace beam_cv