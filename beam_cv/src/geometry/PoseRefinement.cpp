#include "beam_cv/geometry/PoseRefinement.h"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/covariance.h>
#include <ceres/loss_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <beam_optimization/CamPoseReprojectionCost.h>
#include <beam_optimization/CamPoseUnitSphereCost.h>
#include <beam_optimization/PosePriorCost.h>

class WelschLoss : public ceres::LossFunction {
public:
  explicit WelschLoss(const double a) : b_(a * a), c_(-1.0 / b_) {}

  void Evaluate(double s, double rho[3]) const override {
    const double exp = std::exp(s * c_);

    rho[0] = b_ * (1 - exp);
    rho[1] = exp;
    rho[2] = c_ * exp;
  }

private:
  const double b_;
  const double c_;
};

namespace beam_cv {

PoseRefinement::PoseRefinement() {
  // set ceres solver params
  ceres_solver_options_.minimizer_progress_to_stdout = true;
  ceres_solver_options_.max_num_iterations = 100;
  ceres_solver_options_.max_solver_time_in_seconds = 1e6;
  ceres_solver_options_.function_tolerance = 1e-8;
  ceres_solver_options_.gradient_tolerance = 1e-10;
  ceres_solver_options_.parameter_tolerance = 1e-8;
  ceres_solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;
}

PoseRefinement::PoseRefinement(double time_limit, bool is_silent,
                               double reprojection_weight, bool use_unit_sphere)
    : reprojection_weight_(reprojection_weight),
      use_unit_sphere_(use_unit_sphere) {
  // set ceres solver params
  ceres_solver_options_.minimizer_progress_to_stdout = !is_silent;
  ceres_solver_options_.max_num_iterations = 100;
  if (is_silent) { ceres_solver_options_.logging_type = ceres::SILENT; }
  ceres_solver_options_.max_solver_time_in_seconds = time_limit;
  ceres_solver_options_.function_tolerance = 1e-8;
  ceres_solver_options_.gradient_tolerance = 1e-10;
  ceres_solver_options_.parameter_tolerance = 1e-8;
  ceres_solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;
}

PoseRefinement::PoseRefinement(const ceres::Solver::Options options,
                               double reprojection_weight, bool use_unit_sphere)
    : reprojection_weight_(reprojection_weight),
      use_unit_sphere_(use_unit_sphere) {
  ceres_solver_options_ = options;
}

Eigen::Matrix4d PoseRefinement::RefinePose(
    const Eigen::Matrix4d& estimate,
    const std::shared_ptr<beam_calibration::CameraModel>& cam,
    const std::vector<Eigen::Vector2i, beam::AlignVec2i>& pixels,
    const std::vector<Eigen::Vector3d, beam::AlignVec3d>& points,
    std::shared_ptr<Eigen::Matrix<double, 6, 6>> A_in,
    std::shared_ptr<Eigen::Matrix<double, 6, 6>> A_out, std::string& report,
    bool remove_points_outside_domain) {
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
    // check if point is in the function domain
    if (remove_points_outside_domain) {
      Eigen::Vector4d point_h;
      point_h << points[i][0], points[i][1], points[i][2], 1;
      Eigen::Vector4d point_transformed = estimate * point_h;
      if (!cam->InProjectionDomain(point_transformed.hnormalized())) {
        continue;
      }
    }
    if (use_unit_sphere_) {
      std::unique_ptr<ceres::CostFunction> reproj_cost(
          beam_optimization::CeresUnitSphereCostFunction::Create(
              pixels[i].cast<double>(), points[i], cam));
      problem->AddResidualBlock(reproj_cost.release(), loss_function_.get(),
                                &(results[0]));
    } else {
      std::unique_ptr<ceres::CostFunction> reproj_cost(
          beam_optimization::CeresReprojectionCostFunction::Create(
              pixels[i].cast<double>(), points[i], cam, reprojection_weight_));
      problem->AddResidualBlock(reproj_cost.release(), loss_function_.get(),
                                &(results[0]));
    }
  }

  // add a prior to the pose if a weighting matrix is passed in
  if (A_in) {
    std::unique_ptr<ceres::CostFunction> prior_pose_cost(
        CeresPosePriorCostFunction::Create(estimate, *A_in));
    problem->AddResidualBlock(prior_pose_cost.release(), loss_function_.get(),
                              &(results[0]));
  }

  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_solver_options_, problem.get(), &ceres_summary);
  report = ceres_summary.FullReport();

  if (A_out) {
    // compute covariance
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    std::vector<const double*> covariance_block;
    covariance_block.push_back(&(results[0]));
    covariance.Compute(covariance_block, problem.get());

    // setup covariance to return as eigen matrix
    double covariance_arr[7 * 7];
    covariance.GetCovarianceBlock(&(results[0]), &(results[0]), covariance_arr);
    Eigen::Matrix<double, 7, 7> covariance_eig(covariance_arr);
    A_out->block<6, 6>(0, 0) = covariance_eig.block<6, 6>(1, 1);
  }

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

  // loss_function_ =
  //     std::unique_ptr<ceres::LossFunction>(new ceres::HuberLoss(1.0));

  loss_function_ =
      std::unique_ptr<ceres::LossFunction>(new WelschLoss(1.0));

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