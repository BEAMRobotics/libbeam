#include "beam_cv/LinearRegression.h"
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
// std lib

namespace beam_cv {

/**********************Regression implementation********************/
LinearRegression::LinearRegression(const Dataset& data_train) {
  data.copy(data_train);
  weights.init(data.number_predictor, 0);
}

void LinearRegression::PrintWeights() {
  std::cout << "Weights: " << std::endl;
  std::cout << "y = ";
  for (int i = 0; i < weights.number_weights; i++) {
    std::cout << weights.values[i] << " * x" << i;
    if (i < weights.number_weights - 1) {
      std::cout << " + ";
    } else {
      std::cout << std::endl;
    }
  }
}

void LinearRegression::Train(int max_iteration, double learning_rate) {
  // Mallocating some space for prediction
  Eigen::RowVectorXf y_pred(data.length);

  while (max_iteration > 0) {
    this->Fit(y_pred);
    weights.update(data, y_pred, learning_rate);

    double mse = mean_squared_error(y_pred, data.y, data.length);

    /*if (max_iteration % 1000 == 0) {
      std::cout << "Iteration left: " << max_iteration << "; MSE = " << mse
                << "\n";
    }*/
    max_iteration--;
  }
}

double LinearRegression::Predict(Eigen::VectorXf x) {
  double prediction = 0;
  for (int i = 0; i < weights.number_weights; i++) {
    prediction = prediction + weights.values(i) * x(i);
  }
  return prediction;
}

void LinearRegression::Fit(Eigen::RowVectorXf& y_pred) {
  for (int i = 0; i < data.length; i++) { y_pred(i) = Predict(data.X.row(i)); }
}

/**********************Weights implementation********************/
void Weights::init(int number_predictor, int random_init) {
  // Random Init Variables
  MAX_WEIGHTS = 100;
  srand(time(0)); // random number generator

  number_weights = number_predictor;
  values.resize(number_weights);
  for (int i = 0; i < number_weights; i++) {
    if (random_init == 1) {
      values(i) = (rand() % MAX_WEIGHTS);
    } else {
      values(i) = 0;
    }
  }
}

void Weights::update(Dataset data, Eigen::RowVectorXf y_pred,
                     double learning_rate) {
  double multiplier = learning_rate / data.length;
  // Update each weights
  for (int i = 0; i < number_weights; i++) {
    double sum = (sum_residual(data, y_pred, i));
    values(i) = values(i) - multiplier * sum;
  }
}

/**********************Dataset implementation********************/
Dataset::Dataset(Eigen::MatrixXf X_train, Eigen::RowVectorXf y_train,
                 int length_train, int number_predictor_train) {
  X = X_train;
  y = y_train;

  length = length_train;
  number_predictor = number_predictor_train;
}

void Dataset::copy(const Dataset& data) {
  X = data.X;
  y = data.y;

  length = data.length;
  number_predictor = data.number_predictor;
}

/**********************Helper function implementation********************/

double mean(Eigen::RowVectorXf y, int length) {
  double total = 0;
  for (int i = 0; i < length; i++) { total = total + y(i); }
  return (total / length);
}

double sum_residual(Dataset data, Eigen::RowVectorXf y_pred,
                    int current_predictor) {
  double total = 0;
  double residual;
  for (int i = 0; i < data.length; i++) {
    residual = (y_pred(i) - data.y(i));
    total = total + residual * data.X(i, current_predictor);
  }
  return total;
}

double sum_of_square(Eigen::RowVectorXf y, int length) {
  // Not the most efficient way of calculating variance, see :
  // https://www.sciencebuddies.org/science-fair-projects/science-fair/variance-and-standard-deviation
  double total = 0;
  double residual;
  double y_mean = mean(y, length);
  for (int i = 0; i < length; i++) {
    residual = (y(i) - y_mean);
    total = total + (residual * residual);
  }
  return total;
}

double residual_sum_of_square(Eigen::RowVectorXf y_pred,
                              Eigen::RowVectorXf y_true, int length) {
  double total = 0;
  double residual;
  for (int i = 0; i < length; i++) {
    residual = (y_true(i) - y_pred(i));
    total = total + (residual * residual);
  }
  return total;
}

int calculate_r2(Eigen::RowVectorXf y_pred, Eigen::RowVectorXf y_true,
                 int length) {
  // Taken from: https://en.wikipedia.org/wiki/Coefficient_of_determination
  double sum_squared_residual = residual_sum_of_square(y_pred, y_true, length);
  double sum_squared_total = sum_of_square(y_true, length);
  return (1 - ((sum_squared_residual / sum_squared_total)));
}

double mean_squared_error(Eigen::RowVectorXf y_pred, Eigen::RowVectorXf y_true,
                          int length) {
  return residual_sum_of_square(y_pred, y_true, length) / length;
}

double intercept_sum(Eigen::RowVectorXf y_pred, Eigen::RowVectorXf y_true,
                     int length) {
  double total = 0;
  double residual;
  for (int i = 0; i < length; i++) {
    residual = (y_pred(i) - y_true(i));
    total = total + residual;
  }
  return total;
}

double slope_sum(Eigen::RowVectorXf x, Eigen::RowVectorXf y_pred,
                 Eigen::RowVectorXf y_true, int length) {
  double total = 0;
  double residual;
  for (int i = 0; i < length; i++) {
    residual = (y_pred(i) - y_true(i));
    total = total + residual * x(i);
  }
  return total;
}

} // namespace beam_cv