#include "beam_cv/LinearRegression.h"
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <unsupported/Eigen/MatrixFunctions>

namespace beam_cv {

LinearRegression::LinearRegression(const Eigen::MatrixXd X,
                                   const Eigen::RowVectorXd y,
                                   const bool intercept) {
  if (X.rows() != y.cols()) {
    BEAM_CRITICAL("Unequal number of predictors to responses.");
    throw std::runtime_error{"Unequal number of predictors to responses."};
  }
}

double LinearRegression::ComputeError(const Eigen::RowVectorXd predictors,
                                      const Eigen::RowVectorXd responses) {}

Eigen::RowVectorXd LinearRegression::Predict(const Eigen::MatrixXd X) {}

double LinearRegression::Train(const Eigen::MatrixXd X,
                               const Eigen::RowVectorXd y,
                               const bool intercept) {}

} // namespace beam_cv