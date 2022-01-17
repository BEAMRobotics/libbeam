#pragma once

#include <cstdio>

#include <ceres/autodiff_cost_function.h>
#include <ceres/cost_function_to_functor.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/rotation.h>

#include <beam_calibration/CameraModel.h>
#include <beam_utils/optional.h>

template <class T>
using opt = beam::optional<T>;

namespace beam_optimization {
/**
 * @brief Ceres reprojection cost function
 * sets the residual for the projection of a 3D point as determined by the
 * preceding functor and its corresponding detected pixel
 * if the projection of the 3D point is outside the domain of the camera model,
 * the cost function will return false
 * this will fail the ceres solution immediately on the first iteration or
 * after two consecutive false returns at any other point during the solution
 */
struct CeresUnitSphereCostFunction {
  CeresUnitSphereCostFunction(
      Eigen::Vector2d pixel_detected, Eigen::Vector3d P_STRUCT,
      std::shared_ptr<beam_calibration::CameraModel> camera_model)
      : pixel_detected_(pixel_detected),
        P_STRUCT_(P_STRUCT),
        camera_model_(camera_model) {
    // get pixel in the unit
    Eigen::Vector2i pixel_i = pixel_detected.cast<int>();
    in_domain_ = camera_model_->BackProject(pixel_i, unit_sphere_pixel_);
    unit_sphere_pixel_ = unit_sphere_pixel_.normalized();

    // compute tanget base of the measurement
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = unit_sphere_pixel_;
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp) tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base_.block<1, 3>(0, 0) = b1.transpose();
    tangent_base_.block<1, 3>(1, 0) = b2.transpose();

    // compute sqrt information matrix
    sqrt_info_ = Eigen::Matrix2d::Identity();
    sqrt_info_(0, 0) = camera_model_->GetIntrinsics()[0] / 1.5;
    sqrt_info_(1, 1) = camera_model_->GetIntrinsics()[1] / 1.5;
  }

  template <typename T>
  bool operator()(const T* const T_CR, T* residual) const {
    if (!in_domain_) { return false; }

    T P_STRUCT[3];
    P_STRUCT[0] = P_STRUCT_.cast<T>()[0];
    P_STRUCT[1] = P_STRUCT_.cast<T>()[1];
    P_STRUCT[2] = P_STRUCT_.cast<T>()[2];

    // rotate and translate point
    T P_CAMERA[3];
    ceres::QuaternionRotatePoint(T_CR, P_STRUCT, P_CAMERA);
    P_CAMERA[0] += T_CR[4];
    P_CAMERA[1] += T_CR[5];
    P_CAMERA[2] += T_CR[6];

    Eigen::Matrix<T, 3, 1> P_CAM;
    P_CAM[0] = P_CAMERA[0];
    P_CAM[1] = P_CAMERA[1];
    P_CAM[2] = P_CAMERA[2];

    // compute the residual on the unit sphere
    Eigen::Matrix<T, 2, 1> result =
        tangent_base_.cast<T>() *
        (P_CAM.normalized() - unit_sphere_pixel_.cast<T>());

    // apply sqrt information matrix
    result = sqrt_info_.cast<T>() * result;

    // fill residual
    residual[0] = result[0];
    residual[1] = result[1];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction*
      Create(const Eigen::Vector2d pixel_detected,
             const Eigen::Vector3d P_STRUCT,
             const std::shared_ptr<beam_calibration::CameraModel> camera_model) {
    return (new ceres::AutoDiffCostFunction<CeresUnitSphereCostFunction, 2, 7>(
        new CeresUnitSphereCostFunction(pixel_detected, P_STRUCT,
                                        camera_model)));
  }

  Eigen::Vector3d unit_sphere_pixel_; // back projected pixel
  Eigen::Vector2d pixel_detected_;    // pixel that was detected
  Eigen::Vector3d P_STRUCT_; // location of 3d point corresponding to pixel
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  Eigen::Matrix<double, 2, 3> tangent_base_;
  Eigen::Matrix2d sqrt_info_;
  bool in_domain_{true};
};

} // namespace beam_optimization