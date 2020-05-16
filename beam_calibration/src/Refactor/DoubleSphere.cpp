#include "beam_calibration/include/Refactor/DoubleSphere.h"

#include <math.h>

namespace beam_calibration {

DoubleSphere::DoubleSphere(const std::string& file_path) {
  LoadJSON(file_path);
  fx_ = &intrinsics_[0];
  fy_ = &intrinsics_[1];
  cx_ = &intrinsics_[2];
  cy_ = &intrinsics_[3];
  eps_ = &intrinsics_[4];
  alpha_ = &intrinsics_[5];
}

void DoubleSphere::ProjectPoint(const Eigen::Vector3d& point,
                                std::optional<Eigen::Vector2i>& pixel) {
  double w1;
  if (alpha_ > 0.5) {
    w1 = (1 - alpha_) / alpha_;
  } else {
    w1 = alpha_ / (1 - alpha_);
  }
  double w2 = (w1 + eps_) / sqrt(2 * w1 * eps_ + eps_ * eps_ + 1);
  double d1 =
      sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);

  // check pixels are valid for projection
  if (z <= -w2 * d1) {
    pixel = std::nullopt;
    return;
  }
  double d2 = sqrt(point[0] * point[0] + point[1] * point[1] +
                   (eps_ * d1 + point[2]) * (eps_ * d1 + point[2]));
  Eigen::Vector2i point_projected;
  point_projected[0] =
      fx_ * point[0] /
      (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + point[2]) + cx_);
  point_projected[1] =
      fy_ * point[1] /
      (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + point[2]) + cy_);
  if (PixelInImage(point_projected)) {
    pixel = std::optional<Eigen::Vector2i>(point_projected);
  } else {
    pixel = std::nullopt;
  }
  return;
}

void DoubleSphere::BackProject(const Eigen::Vector2i& pixel,
                               std::optional<Eigen::Vector3d>& ray) {
  double mx = (pixel[0] - cx_) / fx_;
  double my = (pixel[1] - cy_) / fy_;
  double r2 = mx * mx + my * my;

  // check pixels are valid for back projection
  if(alpha_ > 0.5 && r2 > 1 / (2 * alpha_ - 1)){
    ray = std::nullopt;
    return;
  }

  double mz = (1 - alpha_ * alpha_ * r2) /
              (alpha_ * sqrt(1 - (2 * alpha_ - 1) * r2) + 1 - alpha_);
  double A =
      (mz * eps_ + sqrt(mz * mz + (1 - eps_ * eps_) * r2)) / (mz * mz + r2);
  ray = std::optional<Eigen::Vector3d>(
      Eigen::Vectotr3d(A * mx, A * my, A * mz - eps_));
  return;
}

void DoubleSphere::ValidateInputs(){
  if(type_ != CameraType::DOUBLESPHERE){
    BEAM_WARN("Invalid camera model type read. Read {}, changing to DOUBLESPHERE");
    type_ = CameraType::DOUBLESPHERE;
  }

  if (intrinsics_.size() != intrinsics_size_[type_]) {
    BEAM_CRITICAL("Invalid number of intrinsics read. read: {}, required: {}",
                  intrinsics.size(), intrinsics_size_[type_]);
    throw std::invalid_argument{"Invalid number of instrinsics read."};
  }
}

} // namespace beam_calibration
