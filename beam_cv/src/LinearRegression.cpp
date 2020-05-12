#include "beam_cv/LinearRegression.h"

namespace beam_cv {

LinearRegression::LinearRegression(std::shared_ptr<Dataset> data_train) {
  this->data_ = data_train;
  this->weights_ = std::make_shared<Weights>(data_->number_predictor);
}

void LinearRegression::PrintWeights() {
  std::cout << "Weights: " << std::endl;
  std::cout << "y = ";
  for (int i = 0; i < weights_->number_weights; i++) {
    std::cout << weights_->values[i] << " * x" << i;
    if (i < weights_->number_weights - 1) {
      std::cout << " + ";
    } else {
      std::cout << std::endl;
    }
  }
}

void LinearRegression::Train(int max_iteration, double learning_rate) {
  // Mallocating some space for prediction
  Eigen::RowVectorXf y_pred(data_->length);

  while (max_iteration > 0) {
    this->Fit(y_pred);
    weights_->Update(*data_, y_pred, learning_rate);

    double mse = mean_squared_error(y_pred, data_->y, data_->length);

    if (max_iteration % 1000 == 0) {
      BEAM_DEBUG("Iteration left: {}; MSE = {}", max_iteration, mse);
    }
    max_iteration--;
  }
}

double LinearRegression::Predict(Eigen::VectorXf x) {
  double prediction = 0;
  for (int i = 0; i < weights_->number_weights; i++) {
    prediction = prediction + weights_->values(i) * x(i);
  }
  return prediction;
}

void LinearRegression::Fit(Eigen::RowVectorXf& y_pred) {
  for (int i = 0; i < data_->length; i++) {
    y_pred(i) = Predict(data_->X.row(i));
  }
}

Weights::Weights(int number_predictor) {
  // Random Init Variables
  MAX_WEIGHTS_ = 100;
  srand(time(0)); // random number generator

  number_weights = number_predictor;
  values.resize(number_weights);
  for (int i = 0; i < number_weights; i++) {
    values(i) = (rand() % MAX_WEIGHTS_);
  }
}

void Weights::Update(Dataset data_, Eigen::RowVectorXf y_pred,
                     double learning_rate) {
  double multiplier = learning_rate / data_.length;
  // Update each weights
  for (int i = 0; i < number_weights; i++) {
    double sum = (sum_residual(data_, y_pred, i));
    values(i) = values(i) - multiplier * sum;
  }
}

Dataset::Dataset(Eigen::MatrixXf X_train, Eigen::RowVectorXf y_train,
                 int length_train, int number_predictor_train) {
  X = X_train;
  y = y_train;

  length = length_train;
  number_predictor = number_predictor_train;
}

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