#include "beam_calibration/DoubleSphere.h"

namespace beam_calibration {

DoubleSphere::DoubleSphere(const std::string& file_path) {
  type_ = CameraType::DOUBLESPHERE;
  BEAM_INFO("Loading file: {}", file_path);
  LoadJSON(file_path);
  fx_ = intrinsics_[0];
  fy_ = intrinsics_[1];
  cx_ = intrinsics_[2];
  cy_ = intrinsics_[3];
  eps_ = intrinsics_[4];
  alpha_ = intrinsics_[5];
}

opt<Eigen::Vector2d>
    DoubleSphere::ProjectPointPrecise(const Eigen::Vector3d& point) {
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
  if (point[2] <= -w2 * d1) { return {}; }
  double d2 = sqrt(point[0] * point[0] + point[1] * point[1] +
                   (eps_ * d1 + point[2]) * (eps_ * d1 + point[2]));
  Eigen::Vector2d point_projected;
  point_projected[0] =
      fx_ * point[0] /
      (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + point[2]) + cx_);
  point_projected[1] =
      fy_ * point[1] /
      (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + point[2]) + cy_);
  if (PixelInImage(point_projected)) { return point_projected; }
  return {};
}

opt<Eigen::Vector2i> DoubleSphere::ProjectPoint(const Eigen::Vector3d& point) {
  opt<Eigen::Vector2d> pixel = ProjectPointPrecise(point);
  if (pixel.has_value()) {
    Eigen::Vector2i pixel_rounded;
    pixel_rounded << std::round(pixel.value()[0]), std::round(pixel.value()[1]);
    return pixel_rounded;
  }
  return {};
}

// todo
opt<Eigen::Vector2i> DoubleSphere::ProjectPoint(const Eigen::Vector3d& point,
                                                Eigen::MatrixXd& J) {}

opt<Eigen::Vector3d> DoubleSphere::BackProject(const Eigen::Vector2i& pixel) {
  double mx = (pixel[0] - cx_) / fx_;
  double my = (pixel[1] - cy_) / fy_;
  double r2 = mx * mx + my * my;

  // check pixels are valid for back projection
  if (alpha_ > 0.5 && r2 > 1 / (2 * alpha_ - 1)) { return {}; }

  double mz = (1 - alpha_ * alpha_ * r2) /
              (alpha_ * sqrt(1 - (2 * alpha_ - 1) * r2) + 1 - alpha_);
  double A =
      (mz * eps_ + sqrt(mz * mz + (1 - eps_ * eps_) * r2)) / (mz * mz + r2);
  return Eigen::Vector3d(A * mx, A * my, A * mz - eps_);
}

} // namespace beam_calibration
