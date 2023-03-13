#define CATCH_CONFIG_MAIN

#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <beam_optimization/PointToLineCost.h>

namespace beam_optimization {

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
  bool output_results{false};
  Eigen::Matrix4d T_WORLD_CLOUD1;
  Eigen::Matrix4d T_WORLD_CLOUD2;
};

Data data_;

std::vector<double> MatrixToPoseVector(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);
  Eigen::Quaternion<double> q = Eigen::Quaternion<double>(R);
  return std::vector<double>{q.w(),   q.x(),   q.y(),  q.z(),
                             T(0, 3), T(1, 3), T(2, 3)};
}

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
  data_.se3_parameterization = std::unique_ptr<ceres::LocalParameterization>(
      new ceres::ProductParameterization(quat_parameterization.release(),
                                         identity_parameterization.release()));

  return problem;
}

void SolveProblem(const std::shared_ptr<ceres::Problem>& problem) {
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(data_.solver_options, problem.get(), &ceres_summary);
  if (data_.output_results) {
    LOG_INFO("Done.");
    LOG_INFO("Outputting ceres summary:");
    std::string report = ceres_summary.FullReport();
    std::cout << report << "\n";
  }
}

TEST_CASE("Test point to line with perturbed pose") {
  // create reference points: take 2 points along each of the principle axes
  Eigen::Vector3d PX1(0, 0, 0);
  Eigen::Vector3d PX2(1, 0, 0);
  Eigen::Vector3d PY1(0, 0, 0);
  Eigen::Vector3d PY2(0, 1, 0);
  Eigen::Vector3d PZ1(0, 0, 0);
  Eigen::Vector3d PZ2(0, 0, 1);

  // create target points: take point in between reference points
  Eigen::Vector3d PX(0.5, 0, 0);
  Eigen::Vector3d PY(0, 0.5, 0);
  Eigen::Vector3d PZ(0, 0, 0.5);

  // build problem and add params
  auto results = MatrixToPoseVector(data_.T_WORLD_CLOUD2);
  auto ground_truth = MatrixToPoseVector(data_.T_WORLD_CLOUD1);

  std::shared_ptr<ceres::Problem> problem = SetupCeresProblem();
  problem->AddParameterBlock(&(results[0]), 7,
                             data_.se3_parameterization.get());

  std::unique_ptr<ceres::CostFunction> cost_function1(
      CeresPointToLineCostFunction::Create(PX, PX1, PX2));
  problem->AddResidualBlock(cost_function1.release(), data_.loss_function.get(),
                            &(results[0]));

  std::unique_ptr<ceres::CostFunction> cost_function2(
      CeresPointToLineCostFunction::Create(PY, PY1, PY2));
  problem->AddResidualBlock(cost_function2.release(), data_.loss_function.get(),
                            &(results[0]));

  std::unique_ptr<ceres::CostFunction> cost_function3(
      CeresPointToLineCostFunction::Create(PZ, PZ1, PZ2));
  problem->AddResidualBlock(cost_function3.release(), data_.loss_function.get(),
                            &(results[0]));

  SolveProblem(problem);

  for (int i = 0; i < 7; i++){
    REQUIRE(std::abs(results[i] - ground_truth[i]) < 0.001);
  }
}

} // namespace beam_optimization