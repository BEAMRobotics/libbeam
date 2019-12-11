/** @file
 * @ingroup cv
 */

#pragma once
// beam
#include "beam_utils/math.hpp"
#include <Eigen/Dense>

namespace beam_cv {
class LinearRegression {
public:
  /**
   * @brief Default constructor
   */
  LinearRegression() = default;

  /**
   * @brief Custom constructor
   * @param X matrix of predictors (features)
   * @param y vector of responses
   * @param intercept bool of whether to include an intercept or not
   */
  LinearRegression(const Eigen::MatrixXd X, const Eigen::RowVectorXd y,
                   const bool intercept = false);

  /**
   * @brief Default destructor
   */
  ~LinearRegression() = default;

  /**
   * @brief Computes the L2 squared error
   * @param predicted matrix of predictors (features)
   * @param responses vector of responses
   */
  double ComputeError(const Eigen::RowVectorXd predictors,
                      const Eigen::RowVectorXd responses);

  /**
   * @brief Returns the parameters from the trained model
   */
  Eigen::RowVectorXd GetParameters() { return *B_; }

  /**
   * @brief Calculate the predicted value for the given points in X
   * @param X matrix of predictors (features)
   * @return matrix of predicted values
   */
  Eigen::RowVectorXd Predict(const Eigen::MatrixXd X);

  /**
   * @brief Train the linear regression model on the given data
   * @param X matrix of predictors (features)
   * @param y vector of responses
   * @param intercept bool of whether to include an intercept or not
   */
  double Train(const Eigen::MatrixXd X, const Eigen::RowVectorXd y,
               const bool intercept);

protected:
  std::shared_ptr<Eigen::RowVectorXd> B_;
};

} // namespace beam_cv