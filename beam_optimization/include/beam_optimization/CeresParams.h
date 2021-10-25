#pragma once

#include <cstdio>
#include <fstream>
#include <map>

#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <nlohmann/json.hpp>

#include <beam_utils/filesystem.h>
#include <beam_utils/log.h>

namespace beam_optimization {

/**
 * @brief This class serves as a helper to setting up ceres optimization
 * problems. A lot of the repetitive code ( e.g., creating parameterizations and
 * setting loss functions) can be factored out and put in this class. This class
 * also sets default parameters that are often used, and allows you to read
 * parameters from a json.
 */
class CeresParams {
public:
  /**
   * @brief Constructor. This will use the default params set in this
   * class, plus the input ownership variables.
   */
  CeresParams(bool cost_function_take_ownership = false,
              bool loss_function_take_ownership = false,
              bool parameterization_take_ownership = false) {
    LoadDefaultParams(cost_function_take_ownership,
                      loss_function_take_ownership,
                      parameterization_take_ownership);
  }

  /**
   * @brief Constructor that takes in a path to a json config file with all
   * relevant parameters, plus the ownership variables. These ownership
   * variables cannot be set using a config file because they are dependent on
   * how the code implements the ceres problem.
   * @param config_path full path to config file. For example, see
   * beam_optimization/config/CeresParamsDefault.json
   *
   */
  CeresParams(const std::string& config_path,
              bool cost_function_take_ownership = false,
              bool loss_function_take_ownership = false,
              bool parameterization_take_ownership = false) {
    LoadDefaultParams(cost_function_take_ownership,
                      loss_function_take_ownership,
                      parameterization_take_ownership);

    if (config_path.empty()) { return; }

    BEAM_INFO("Loading ceres config file: {}", config_path);
    nlohmann::json J;
    if (!beam::ReadJson(config_path, J)) {
      BEAM_ERROR("Using default ceres params.");
    }

    try {
      nlohmann::json J_solver_options = J["solver_options"];
      solver_options_.minimizer_progress_to_stdout =
          J_solver_options["minimizer_progress_to_stdout"];
      solver_options_.max_num_iterations =
          J_solver_options["max_num_iterations"];
      solver_options_.max_solver_time_in_seconds =
          J_solver_options["max_solver_time_in_seconds"];
      solver_options_.function_tolerance =
          J_solver_options["function_tolerance"];
      solver_options_.gradient_tolerance =
          J_solver_options["gradient_tolerance"];
      solver_options_.parameter_tolerance =
          J_solver_options["parameter_tolerance"];

      std::string linear_solver_type = J_solver_options["linear_solver_type"];
      if (linear_solver_map_.find(linear_solver_type) ==
          linear_solver_map_.end()) {
        BEAM_ERROR(
            "Invalid linear_solver_type param. Using default (SPARSE_SCHUR). "
            "Options: DENSE_NORMAL_CHOLESKY, DENSE_QR, SPARSE_NORMAL_CHOLESKY, "
            "DENSE_SCHUR, SPARSE_SCHUR, ITERATIVE_SCHUR, CGNR");
        solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
      } else {
        solver_options_.linear_solver_type =
            linear_solver_map_[linear_solver_type];
      }

      std::string preconditioner_type = J_solver_options["preconditioner_type"];
      if (preconditioner_type_map_.find(preconditioner_type) ==
          preconditioner_type_map_.end()) {
        BEAM_ERROR(
            "Invalid preconditioner_type param. Using default (SCHUR_JACOBI). "
            "Options: IDENTITY, JACOBI, SCHUR_JACOBI");
        solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;
      } else {
        solver_options_.preconditioner_type =
            preconditioner_type_map_[preconditioner_type];
      }

      nlohmann::json J_loss_function = J["loss_function"];
      loss_function_type_ = J_loss_function["type"];
      loss_function_scaling_ = J_loss_function["scaling"];
      if (loss_function_types_.find(loss_function_type_) ==
          loss_function_types_.end()) {
        LOG_ERROR("Invalid loss function type, Options: HUBER, CAUCHY, NULL. "
                  "Using default: HUBER");
        loss_function_type_ = "HUBER";
      }
    } catch (const nlohmann::json::exception& e) {
      BEAM_ERROR(
          "Cannot load json config file for CeresParams, one or more "
          "params are missing or invalid. Using default parameters. Reason: {}",
          e.what());
    }
  }

  /**
   * @brief default destructor
   */
  ~CeresParams() = default;

  /**
   * @brief get solver options
   */
  ceres::Solver::Options SolverOptions() { return solver_options_; }

  /**
   * @brief get problem options
   */
  ceres::Problem::Options ProblemOptions() { return problem_options_; }

  /**
   * @brief get loss function type as a string
   */
  std::string LossFunctionType() { return loss_function_type_; }

  /**
   * @brief get loss function scaling
   */
  double LossFunctionScaling() { return loss_function_scaling_; }

  /**
   * @brief get a unique pointer to a loss function based on the type specified
   * herein
   *
   * IMPORTANT NOTE: When using this, you
   * must store the unique pointer in your class before calling ptr.get() when
   * adding to ceres. Otherwise, you will get a segmentation fault
   */
  std::unique_ptr<ceres::LossFunction> LossFunction() {
    std::unique_ptr<ceres::LossFunction> loss_function;
    if (loss_function_type_ == "HUBER") {
      loss_function = std::unique_ptr<ceres::LossFunction>(
          new ceres::HuberLoss(loss_function_scaling_));
    } else if (loss_function_type_ == "CAUCHY") {
      loss_function = std::unique_ptr<ceres::LossFunction>(
          new ceres::CauchyLoss(loss_function_scaling_));
    } else if (loss_function_type_ == "NULL") {
      loss_function = std::unique_ptr<ceres::LossFunction>(nullptr);
    }
    return std::move(loss_function);
  }

  /**
   * @brief get a unique pointer to an SE3 parameterization, parameterized based
   * on the combination of a quaternion (w x y z) parameterization plus an
   * identity (x y z) parameterization.
   *
   * IMPORTANT NOTE: When using this, you
   * must store the unique pointer in your class before calling ptr.get() when
   * adding to ceres. Otherwise, you will get a segmentation fault
   */
  std::unique_ptr<ceres::LocalParameterization>
      SE3QuatTransLocalParametrization() {
    std::unique_ptr<ceres::LocalParameterization> quat_parameterization(
        new ceres::QuaternionParameterization());
    std::unique_ptr<ceres::LocalParameterization> identity_parameterization(
        new ceres::IdentityParameterization(3));
    std::unique_ptr<ceres::LocalParameterization> se3_parameterization(
        new ceres::ProductParameterization(
            quat_parameterization.release(),
            identity_parameterization.release()));
    return std::move(se3_parameterization);
  }

private:
  void LoadDefaultParams(bool cost_function_take_ownership = false,
                         bool loss_function_take_ownership = false,
                         bool parameterization_take_ownership = false) {
    solver_options_.minimizer_progress_to_stdout = false;
    solver_options_.max_num_iterations = 50;
    solver_options_.max_solver_time_in_seconds = 1e6;
    solver_options_.function_tolerance = 1e-8;
    solver_options_.gradient_tolerance = 1e-10;
    solver_options_.parameter_tolerance = 1e-8;
    solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;

    // set these here, these cannot be overridden because of the way this class
    // returns loss functino and parameterization
    problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.local_parameterization_ownership =
        ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    if (cost_function_take_ownership) {
      problem_options_.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    }
    if (loss_function_take_ownership) {
      problem_options_.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    }
    if (parameterization_take_ownership) {
      problem_options_.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;
    }
  }

  std::map<std::string, ceres::LinearSolverType> linear_solver_map_{
      {"DENSE_NORMAL_CHOLESKY", ceres::DENSE_NORMAL_CHOLESKY},
      {"DENSE_QR", ceres::DENSE_QR},
      {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY},
      {"DENSE_SCHUR", ceres::DENSE_SCHUR},
      {"SPARSE_SCHUR", ceres::SPARSE_SCHUR},
      {"ITERATIVE_SCHUR", ceres::ITERATIVE_SCHUR},
      {"CGNR", ceres::CGNR},
  };

  std::map<std::string, ceres::PreconditionerType> preconditioner_type_map_{
      {"IDENTITY", ceres::IDENTITY},
      {"JACOBI", ceres::JACOBI},
      {"SCHUR_JACOBI", ceres::SCHUR_JACOBI},
  };

  std::set<std::string> loss_function_types_{
      {"HUBER"},
      {"CAUCHY"},
      {"NULL"},
  };

  ceres::Problem::Options problem_options_;
  ceres::Solver::Options solver_options_;
  std::string loss_function_type_{"HUBER"};
  double loss_function_scaling_{1.0};
};

} // namespace beam_optimization
