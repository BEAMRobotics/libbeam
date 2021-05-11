#pragma once

#include <cstdio>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/rotation.h>

/**
 * @brief Ceres cost functor for a point to line error where the line is defined
 * by two 3D points
 */
struct CeresPointToLineCostFunction {
  /**
   * @brief Constructor
   * @param P_TGT point in question
   * @param P_REF1 reference line point 1
   * @param P_REF2 reference line point 2
   */
  CeresPointToLineCostFunction(const Eigen::Vector3d P_TGT,
                               const Eigen::Vector3d P_REF1,
                               const Eigen::Vector3d P_REF2)
      : P_TGT_(P_TGT), P_REF1_(P_REF1), P_REF2_(P_REF2) {}

  // T_REF_TGT is [qw, qx, qy, qz, tx, ty, tz]
  template <typename T>
  bool operator()(const T* const T_REF_TGT, T* residuals) const {
    // cast member variables
    Eigen::Matrix<T,3,1> _P_TGT = P_TGT_.cast<T>();
    Eigen::Matrix<T,3,1> _P_REF1 = P_REF1_.cast<T>();
    Eigen::Matrix<T,3,1> _P_REF2 = P_REF2_.cast<T>();

    T P_TGT[3];
    P_TGT[0] = _P_TGT[0];
    P_TGT[1] = _P_TGT[1];
    P_TGT[2] = _P_TGT[2];

    // rotate and translate point
    T P_REF[3];
    ceres::QuaternionRotatePoint(T_REF_TGT, P_TGT, P_REF);
    P_REF[0] += T_REF_TGT[4];
    P_REF[1] += T_REF_TGT[5];
    P_REF[2] += T_REF_TGT[6];

    /*
     * e = distance from point to line
     *   = | (P_REF - P_REF1) x (P_REF - P_REF1) |
     *     ---------------------------------------
     *               | P_REF1 - P_REF2 |
     *
     *   = | d1 x d2 |
     *     -----------
     *       | d12 |
     *
     *   Where P_REF = T_REF_TGT * P_TGT
     *
     */
    T d1[3], d2[3], d12[3];
    d1[0] = P_REF[0] - _P_REF1[0];
    d1[1] = P_REF[1] - _P_REF1[1];
    d1[2] = P_REF[2] - _P_REF1[2];

    d2[0] = P_REF[0] - _P_REF2[0];
    d2[1] = P_REF[1] - _P_REF2[1];
    d2[2] = P_REF[2] - _P_REF2[2];

    d12[0] = _P_REF1[0] - _P_REF2[0];
    d12[1] = _P_REF1[1] - _P_REF2[1];
    d12[2] = _P_REF1[2] - _P_REF2[2];

    T cross[3];
    ceres::CrossProduct(d1, d2, cross);

    T norm =
        sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
    T norm12 = sqrt(d12[0] * d12[0] + d12[1] * d12[1] + d12[2] * d12[2]);

    residuals[0] = norm / norm12;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Vector3d P_TGT,
                                     const Eigen::Vector3d P_REF1,
                                     const Eigen::Vector3d P_REF2) {
    return (new ceres::AutoDiffCostFunction<CeresPointToLineCostFunction, 1, 7>(
        new CeresPointToLineCostFunction(P_TGT, P_REF1, P_REF2)));
  }

  Eigen::Vector3d P_TGT_;
  Eigen::Vector3d P_REF1_;
  Eigen::Vector3d P_REF2_;
};
