#pragma once

#include <beam_utils/math.h>
#include <cstdio>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/rotation.h>

/**
 * @brief Ceres cost functor for a prior pose with a covariance
 */
struct CeresPosePriorCostFunction {
  /**
   * @brief Constructor
   * @param T_P Prior pose estimate
   * @param A residual weighting matrix on the pose estimate (6x6)
   */
  CeresPosePriorCostFunction(const Eigen::Matrix4d& T_P,
                             const Eigen::Matrix<double, 6, 6>& A)
      : A_(A) {
    beam::TransformMatrixToQuaternionAndTranslation(T_P, q_, p_);
  }

  template <typename T>
  bool operator()(const T* const T_CR, T* residuals) const {
    // Compute the delta quaternion
    T orientation[4] = {T_CR[0], T_CR[1], T_CR[2], T_CR[3]};
    T observation_inverse[4] = {T(q_.w()), T(-q_.x()), T(-q_.y()), T(-q_.z())};
    T difference[4];
    ceres::QuaternionProduct(observation_inverse, orientation, difference);
    ceres::QuaternionToAngleAxis(difference, residuals);

    // Compute the position error
    residuals[3] = T_CR[4] - T(p_(0));
    residuals[4] = T_CR[5] - T(p_(1));
    residuals[5] = T_CR[6] - T(p_(2));

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residual_map(residuals);
    residual_map.applyOnTheLeft(A_.template cast<T>());

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Matrix4d T_P,
                                     const Eigen::MatrixXd A) {
    return (new ceres::AutoDiffCostFunction<CeresPosePriorCostFunction, 1, 7>(
        new CeresPosePriorCostFunction(T_P, A)));
  }

  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;
  Eigen::Matrix<double, 6, 6> A_;
};
