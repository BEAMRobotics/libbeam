#ifndef CAMCAD_CCCF_HPP
#define CAMCAD_CCCF_HPP

#include <ceres/numeric_diff_cost_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/rotation.h>
#include <ceres/cost_function_to_functor.h>
#include <optional>
#include <stdio.h>

#include <beam_calibration/CameraModel.h>

struct CameraProjectionFunctor {
  CameraProjectionFunctor(
      const std::shared_ptr<beam_calibration::CameraModel>& camera_model)
      : camera_model_(camera_model) {}

  bool operator()(const double* P, double* pixel) const {
    Eigen::Vector3d P_CAMERA_eig{P[0], P[1], P[2]};
    std::optional<Eigen::Vector2d> pixel_projected =
        camera_model_->ProjectPointPrecise(P_CAMERA_eig);
    //get image dims to set residual to large value proportionate to image size
    uint16_t height = camera_model_->GetHeight() != 0 ? camera_model_->GetHeight() : 5000; 
    uint16_t width = camera_model_->GetWidth() != 0 ? camera_model_->GetWidth() : 5000;
    if (!pixel_projected.has_value()) {
      pixel[0] = 2*height;  // set the residual to a large value if the projection fails 
      pixel[1] = 2*width;  
      return false; 
    }
    pixel[0] = pixel_projected.value()[0];
    pixel[1] = pixel_projected.value()[1];
    return true;
  }

  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
};

struct CeresReprojectionCostFunction {
  CeresReprojectionCostFunction(
      Eigen::Vector2d pixel_detected, Eigen::Vector3d P_STRUCT,
      std::shared_ptr<beam_calibration::CameraModel> camera_model)
      : pixel_detected_(pixel_detected),
        P_STRUCT_(P_STRUCT),
        camera_model_(camera_model) {
        compute_projection.reset(new ceres::CostFunctionToFunctor<2, 3>(
          new ceres::NumericDiffCostFunction<CameraProjectionFunctor,
                                           ceres::CENTRAL, 2, 3>(
            new CameraProjectionFunctor(camera_model_))));
  }

  template <typename T>
  bool operator()(const T* const T_CR, T* residuals) const {
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

    const T* P_CAMERA_const = &(P_CAMERA[0]);

    T pixel_projected[2];
    (*compute_projection)(P_CAMERA_const, &(pixel_projected[0]));

    residuals[0] = pixel_detected_.cast<T>()[0] - pixel_projected[0];
    residuals[1] = pixel_detected_.cast<T>()[1] - pixel_projected[1];

    return true;
    
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(
      const Eigen::Vector2d pixel_detected, const Eigen::Vector3d P_STRUCT,
      const std::shared_ptr<beam_calibration::CameraModel> camera_model) {
    return (new ceres::AutoDiffCostFunction<CeresReprojectionCostFunction, 2, 7>(
        new CeresReprojectionCostFunction(pixel_detected, P_STRUCT,
                                    camera_model)));
  }

  Eigen::Vector2d pixel_detected_;
  Eigen::Vector3d P_STRUCT_;
  std::shared_ptr<beam_calibration::CameraModel> camera_model_;
  std::unique_ptr<ceres::CostFunctionToFunctor<2, 3>> compute_projection;
};

#endif