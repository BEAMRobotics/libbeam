#define CATCH_CONFIG_MAIN

#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <vicon_calibration/Utils.h>
#include <vicon_calibration/optimization/CeresLidarCostFunction.h>

using namespace vicon_calibration;

class Data {
public:
  Data() {
    // create poses
    srand(time(NULL));
    double max_pert_rot{10};
    double max_pert_trans{0.05};
    T_WORLD_CLOUD1 = Eigen::Matrix4d::Identity();
    Eigen::VectorXd perturb(6);
    perturb << beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_rot, -max_pert_rot),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans),
        beam::randf(max_pert_trans, -max_pert_trans);
    T_WORLD_CLOUD2 = beam::PerturbTransformDegM(T_WORLD_CLOUD1, perturb);
  }

  ceres::Solver::Options solver_options;
  std::unique_ptr<ceres::LossFunction> loss_function;
  std::unique_ptr<ceres::LocalParameterization> se3_parameterization;
  bool output_results_{false};
  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;

};

Data data_;

std::shared_ptr<ceres::Problem> SetupCeresProblem() {
  // set ceres solver params
  data_.solver_options.minimizer_progress_to_stdout = false;
  data_.solver_options.max_num_iterations = 50;
  data_.solver_options.max_solver_time_in_seconds = 1e6;
  data_.solver_options.function_tolerance = 1e-8;
  data_.solver_options.gradient_tolerance = 1e-10;
  data_.solver_options.parameter_tolerance = 1e-8;
  data_.solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  data_.solver_options.preconditioner_type = ceres::SCHUR_JACOBI;

  // set ceres problem options
  ceres::Problem::Options ceres_problem_options;

  // if we want to manage our own data for these, we can set these flags:
  ceres_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;

  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_problem_options);

  data_.loss_function =
      std::unique_ptr<ceres::LossFunction>(new ceres::HuberLoss(1.0));

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));
  data_.se3_parameterization_ = std::unique_ptr<ceres::LocalParameterization>(
      new ceres::ProductParameterization(quat_parameterization.release(),
                                         identity_parameterization.release()));

  return problem;
}

void SolveProblem(const std::shared_ptr<ceres::Problem>& problem,
                  bool output_results) {
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(data_.solver_options, problem.get(), &ceres_summary);
  if (output_results) {
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = ceres_summary.FullReport();
    std::cout << report << "\n";
  }
}

TEST_CASE("Test point to line with perturbed pose") {
  // create points: take 3 points along each of the principle axes
  Eigen::Vector3d PX1(1,0,0);
  
}

