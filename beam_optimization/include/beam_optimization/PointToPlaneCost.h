#pragma once

#include <cstdio>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/rotation.h>

/**
 * @brief Ceres cost functor for a point to plane error where the plane is
 * defined by three 3D points
 */
struct CeresPointToPlaneCostFunction {
  /**
   * @brief Constructor
   * @param P_TGT point in question
   * @param P_REF1 reference surface point 1
   * @param P_REF2 reference surface point 2
   * @param P_REF3 reference surface point 2
   */
  CeresPointToPlaneCostFunction(const Eigen::Vector3d& P_TGT,
                                const Eigen::Vector3d& P_REF1,
                                const Eigen::Vector3d& P_REF2,
                                const Eigen::Vector3d& P_REF3)
      : P_TGT_(P_TGT), P_REF1_(P_REF1), P_REF2_(P_REF2), P_REF3_(P_REF3) {}

  // T_REF_TGT is [qw, qx, qy, qz, tx, ty, tz]
  template <typename T>
  bool operator()(const T* const T_REF_TGT, T* residuals) const {
    // get pointer of type T to current point
    T P_TGT[3];
    P_TGT[0] = static_cast<T>(P_TGT_[0]);
    P_TGT[1] = static_cast<T>(P_TGT_[1]);
    P_TGT[2] = static_cast<T>(P_TGT_[2]);

    // rotate and translate point
    T P_REF[3];
    ceres::QuaternionRotatePoint(T_REF_TGT, P_TGT, P_REF);
    P_REF[0] += T_REF_TGT[4];
    P_REF[1] += T_REF_TGT[5];
    P_REF[2] += T_REF_TGT[6];

    /*
     * e = distance from point to line
     *   = | (P_REF - P_REF1) (P_REF1 - P_REF2) x (P_REF1 - P_REF3) |
     *     ----------------------------------------------------------
     *             | (P_REF1 - P_REF2) x (P_REF1 - P_REF3) |
     *
     *   = | dR1 (d12 x d12) |
     *     -------------------
     *       | d12 x d13 |
     *
     *   Where P_REF = T_REF_TGT * P_TGT
     *
     */
    T d12[3], d13[3], dR1[3];
    d12[0] = static_cast<T>(P_REF1_[0]) - static_cast<T>(P_REF2_[0]);
    d12[1] = static_cast<T>(P_REF1_[1]) - static_cast<T>(P_REF2_[1]);
    d12[2] = static_cast<T>(P_REF1_[2]) - static_cast<T>(P_REF2_[2]);

    d13[0] = static_cast<T>(P_REF1_[0]) - static_cast<T>(P_REF3_[0]);
    d13[1] = static_cast<T>(P_REF1_[1]) - static_cast<T>(P_REF3_[1]);
    d13[2] = static_cast<T>(P_REF1_[2]) - static_cast<T>(P_REF3_[2]);

    dR1[0] = P_REF[0] - static_cast<T>(P_REF1_[0]);
    dR1[1] = P_REF[1] - static_cast<T>(P_REF1_[1]);
    dR1[2] = P_REF[2] - static_cast<T>(P_REF1_[2]);

    T cross[3];
    ceres::CrossProduct(d12, d13, cross);
    T norm =
        sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);

    residuals[0] = ceres::DotProduct(dR1, cross) / norm;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Eigen::Vector3d& P_TGT,
                                     const Eigen::Vector3d& P_REF1,
                                     const Eigen::Vector3d& P_REF2,
                                     const Eigen::Vector3d& P_REF3) {
    return (
        new ceres::AutoDiffCostFunction<CeresPointToPlaneCostFunction, 1, 7>(
            new CeresPointToPlaneCostFunction(P_TGT, P_REF1, P_REF2, P_REF3)));
  }

  Eigen::Vector3d P_TGT_;
  Eigen::Vector3d P_REF1_;
  Eigen::Vector3d P_REF2_;
  Eigen::Vector3d P_REF3_;
};
