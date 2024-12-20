#define CATCH_CONFIG_MAIN

#include <Eigen/Geometry>
#include <catch2/catch.hpp>

#include <beam_utils/filesystem.h>
#include <beam_utils/math.h>
#include <beam_utils/se3.h>

#include <beam_optimization/CeresParams.h>
#include <beam_optimization/PosePriorCost.h>

using namespace beam_optimization;

std::string GetFilepathConfig(std::string filename) {
  return beam::LibbeamRoot() + "beam_optimization/config/" + filename;
}

TEST_CASE("Test prior pose cost function with identity covariance") {
  std::string ceres_config = GetFilepathConfig("CeresParamsDefault.json");
  CeresParams ceres_params(ceres_config);

  // initial estimate
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(2, 3) = 5;
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
  beam::TransformMatrixToQuaternionAndTranslation(T, q, t);

  // create perturbed initial
  Eigen::VectorXd pert(6, 1);
  pert << 10, -10, 10, 0.1, -0.1, 0.1;
  Eigen::Matrix4d T_pert = beam::PerturbTransformDegM(T, pert);
  Eigen::Quaterniond q_pert;
  Eigen::Vector3d t_pert;
  beam::TransformMatrixToQuaternionAndTranslation(T_pert, q_pert, t_pert);

  // result
  std::vector<double> results{q_pert.w(), q_pert.x(), q_pert.y(), q_pert.z(),
                              t_pert[0],  t_pert[1],  t_pert[2]};

  // set low covariance
  Eigen::Matrix<double, 6, 6> covariance =
      Eigen::MatrixXd::Identity(6, 6) * 1e-8;

  // setup ceres problem
  std::shared_ptr<ceres::Problem> problem =
      std::make_shared<ceres::Problem>(ceres_params.ProblemOptions());

  std::unique_ptr<ceres::LocalParameterization> parameterization =
      ceres_params.SE3QuatTransLocalParametrization();

  std::unique_ptr<ceres::LossFunction> loss_function =
      ceres_params.LossFunction();

  problem->AddParameterBlock(&(results[0]), 7, parameterization.get());

  std::unique_ptr<ceres::CostFunction> cost_function(
      CeresPosePriorCostFunction::Create(T, covariance));

  problem->AddResidualBlock(cost_function.release(), loss_function.get(),
                            &(results[0]));

  // solve problem
  ceres::Solver::Summary ceres_summary;
  ceres::Solve(ceres_params.SolverOptions(), problem.get(), &ceres_summary);
  std::string report = ceres_summary.FullReport();

  Eigen::Quaterniond result_q{results[0], results[1], results[2], results[3]};
  Eigen::Vector3d result_p{results[4], results[5], results[6]};
  Eigen::Matrix4d result_T;

  beam::QuaternionAndTranslationToTransformMatrix(result_q, result_p, result_T);
  REQUIRE(beam::RoundMatrix(T_pert, 5) == beam::RoundMatrix(result_T, 5));
}