#define CATCH_CONFIG_MAIN

#include <Eigen/Geometry>
#include <catch2/catch.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <beam_utils/math.h>

#include <beam_optimization/PosePriorCost.h>

TEST_CASE("Test prior pose cost function with identity covariance") {
  ceres::Problem::Options ceres_problem_options;
  ceres_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres_problem_options.local_parameterization_ownership =
      ceres::DO_NOT_TAKE_OWNERSHIP;

  std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
      new ceres::QuaternionParameterization());
  std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
      new ceres::IdentityParameterization(3));
  std::unique_ptr<ceres::LocalParameterization> parameterization_ =
      std::unique_ptr<ceres::LocalParameterization>(
          new ceres::ProductParameterization(
              quat_parameterization.release(),
              identity_parameterization.release()));
  std::unique_ptr<ceres::LossFunction> loss_function_ =
      std::unique_ptr<ceres::LossFunction>(new ceres::TrivialLoss());

  ceres::Solver::Options ceres_solver_options;
  ceres_solver_options.minimizer_progress_to_stdout = true;
  ceres_solver_options.max_num_iterations = 100;
  ceres_solver_options.max_solver_time_in_seconds = 1e6;
  ceres_solver_options.function_tolerance = 1e-8;
  ceres_solver_options.gradient_tolerance = 1e-10;
  ceres_solver_options.parameter_tolerance = 1e-8;
  ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  // initial estimate
  Eigen::Matrix4d T_P = Eigen::Matrix4d::Identity();
  T_P(2, 3) = 5;
  Eigen::Quaterniond tp_q;
  Eigen::Vector3d tp_p;
  beam::TransformMatrixToQuaternionAndTranslation(T_P, tp_q, tp_p);
  // create perturbed initial
  Eigen::VectorXd perturbation(6, 1);
  perturbation << 10, -10, 10, 1, -1, 1;
  Eigen::Matrix4d T_P_pert = beam::PerturbTransformDegM(T_P, perturbation);
  Eigen::Quaterniond tpert_q;
  Eigen::Vector3d tpert_p;
  beam::TransformMatrixToQuaternionAndTranslation(T_P_pert, tpert_q, tpert_p);

  // result
  std::vector<double> results{tp_q.w(), tp_q.x(), tp_q.y(), tp_q.z(),
                              tp_p[0],  tp_p[1],  tp_p[2]};

  // identity covariance (absolute certainty)
  Eigen::Matrix<double, 6, 6> covariance = Eigen::MatrixXd::Identity(6, 6);

  // setup ceres problem
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_problem_options);

  problem->AddParameterBlock(&(results[0]), 7, parameterization_.get());
  std::unique_ptr<ceres::CostFunction> cost_function(
      CeresPosePriorCostFunction::Create(T_P_pert, covariance));
  problem->AddResidualBlock(cost_function.release(), loss_function_.get(),
                            &(results[0]));

  // solve problem
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_solver_options, problem.get(), &ceres_summary);
  std::string report = ceres_summary.FullReport();

  // get pose from result
  std::cout << "INITIAL" << std::endl;
  std::cout << tp_q.w() << "," << tp_q.x() << "," << tp_q.y() << "," << tp_q.z()
            << "," << tp_p[0] << "," << tp_p[1] << "," << tp_p[2] << std::endl;
  std::cout << "RESULT" << std::endl;
  for (auto& i : results) { std::cout << i << ", "; }
  std::cout << std::endl;
  std::cout << "EXPECTED" << std::endl;
  std::cout << tpert_q.w() << "," << tpert_q.x() << "," << tpert_q.y() << ","
            << tpert_q.z() << "," << tpert_p[0] << "," << tpert_p[1] << ","
            << tpert_p[2] << std::endl;

  // REQUIRE(beam::RoundMatrix(T_P_pert, 5) == beam::RoundMatrix(pose, 5));
}