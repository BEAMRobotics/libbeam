/** @file
 * @ingroup cv
 */

#pragma once
#include "beam_utils/math.hpp"

namespace beam_cv {

class Dataset {
public:
  Eigen::MatrixXf X;
  Eigen::RowVectorXf y;
  int number_predictor;
  int length;

  /**
   * @brief Default constructor
   */
  Dataset() = default;

  /**
   * @brief Constructor to initlialize data
   * @param X_train X matrix of predictors
   * @param y_train y vector of responses
   * @param length_train number of epochs to train
   * @param number_predictor_train number of training variables
   */
  Dataset(Eigen::MatrixXf X_train, Eigen::RowVectorXf y_train, int length_train,
          int number_predictor_train);

  /**
   * @brief Default destructor
   */
  ~Dataset() = default;

  /**
   * @brief copy constructor
   */
  void copy(const Dataset& data);
};

class Weights {
public:
  Eigen::RowVectorXf values;
  int number_weights;

  /**
   * @brief Default constructor
   */
  Weights() = default;

  /**
   * @brief Default destructor
   */
  ~Weights() = default;

  /**
   * @brief Initializes dataset with desired number of predictor vairables
   * @param number_predictor number of predictor variables
   * @param random_init flag if random initialization is desired
   */
  void init(int number_predictor, int random_init);

  /**
   * @brief Updates weights given prediccted y values and learning rate
   * @param data to be trained on
   * @param y_pred predicted y values
   * @param learning_rate
   */
  void update(Dataset data, Eigen::RowVectorXf y_pred, double learning_rate);

protected:
  int MAX_WEIGHTS_;
};

class LinearRegression {
public:
  /**
   * @brief Default constructor
   */
  LinearRegression() = default;

  /**
   * @brief Constructor to initlialize regression
   * @param data_train training dataset
   */
  LinearRegression(const Dataset& data_train);

  /**
   * @brief Default destructor
   */
  ~LinearRegression() = default;

  /**
   * @brief Print the current weights
   */
  void PrintWeights();

  /**
   * @brief Perform training on data
   * @param max_iteration maximum number of iterations to perform
   * @param learning_rate the speed of which to descend during optimization
   */
  void Train(int max_iteration, double learning_rate);

  /**
   * @brief Predict response for given input
   * @param x observation
   */
  double Predict(Eigen::VectorXf x);

  /**
   * @brief Fill a vector of responses to the predictors for the current weights
   * @param y_pred vector to fill
   */
  void Fit(Eigen::RowVectorXf& y_pred);

protected:
  Dataset data_;
  Weights weights_;
};

/**
 * Useful helper math functions
 */
double mean(Eigen::RowVectorXf y, int length);
double sum_of_square(Eigen::RowVectorXf y, int length);
double sum_residual(Dataset data, Eigen::RowVectorXf y_pred,
                    int current_predictor);
double residual_sum_of_square(Eigen::RowVectorXf y_pred,
                              Eigen::RowVectorXf y_true, int length);
int calculate_r2(Eigen::RowVectorXf y_pred, Eigen::RowVectorXf y_true,
                 int length);
double mean_squared_error(Eigen::RowVectorXf y_pred, Eigen::RowVectorXf y_true,
                          int length);
double intercept_sum(Eigen::RowVectorXf y_pred, Eigen::RowVectorXf y_true,
                     int length);
double slope_sum(Eigen::RowVectorXf x, Eigen::RowVectorXf y_pred,
                 Eigen::RowVectorXf y_true, int length);

} // namespace beam_cv