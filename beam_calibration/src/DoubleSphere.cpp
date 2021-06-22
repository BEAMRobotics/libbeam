#include "beam_calibration/DoubleSphere.h"

namespace beam_calibration {

DoubleSphere::DoubleSphere(const std::string& file_path) {
  type_ = CameraType::DOUBLESPHERE;
  LoadJSON(file_path);
  fx_ = intrinsics_[0];
  fy_ = intrinsics_[1];
  cx_ = intrinsics_[2];
  cy_ = intrinsics_[3];
  eps_ = intrinsics_[4];
  alpha_ = intrinsics_[5];
}

std::shared_ptr<CameraModel> DoubleSphere::Clone() {
  std::shared_ptr<DoubleSphere> clone = std::make_shared<DoubleSphere>();
  clone->type_ = CameraType::DOUBLESPHERE;
  clone->SetIntrinsics(this->GetIntrinsics());
  clone->SetImageDims(this->GetHeight(), this->GetWidth());
  clone->SetFrameID(this->GetFrameID());
  clone->SetCalibrationDate(this->GetCalibrationDate());
  return clone;
}

bool DoubleSphere::ProjectPoint(const Eigen::Vector3d& in_point,
                                Eigen::Vector2d& out_pixel,
                                bool& in_image_plane,
                                std::shared_ptr<Eigen::MatrixXd> J) {
  if (!this->InProjectionDomain(in_point)) { return false; }
  double Px = in_point[0];
  double Py = in_point[1];
  double Pz = in_point[2];
  double d1 = sqrt(Px * Px + Py * Py + Pz * Pz);
  double d2 = sqrt(Px * Px + Py * Py + (eps_ * d1 + Pz) * (eps_ * d1 + Pz));

  // project point
  out_pixel[0] =
      fx_ * Px / (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + Pz)) + cx_;
  out_pixel[1] =
      fy_ * Py / (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + Pz)) + cy_;

  if (PixelInImage(out_pixel)) {
    in_image_plane = true;
  } else {
    in_image_plane = false;
  }

  if (J) {
    double H = 1 / (alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + Pz));

    Eigen::MatrixXd dPxdP(1, 3);
    Eigen::MatrixXd dPydP(1, 3);
    Eigen::MatrixXd dPzdP(1, 3);
    dPxdP << 1, 0, 0;
    dPydP << 0, 1, 0;
    dPzdP << 0, 0, 1;

    Eigen::MatrixXd dd1dP(1, 3);
    dd1dP << Px / d1, Py / d1, Pz / d1;

    double tmp = (eps_ * eps_ + 1 + eps_ * Pz / d1) / d2;
    double dd2dPz =
        ((eps_ * eps_ + 1) * Pz + eps_ * d1 + eps_ * Pz * Pz / d1) / d2;
    Eigen::MatrixXd dd2dP(1, 3);
    dd2dP << Px * tmp, Py * tmp, dd2dPz;

    Eigen::MatrixXd dldP(1, 3);
    dldP = alpha_ * dd2dP + eps_ * (1 - alpha_) * dd1dP + (1 - alpha_) * dPzdP;

    Eigen::MatrixXd dHdP(1, 3);
    double tmp2 = alpha_ * d2 + (1 - alpha_) * (eps_ * d1 + Pz);
    dHdP = -1 / (tmp2 * tmp2) * dldP;

    J->block(0, 0, 1, 3) = fx_ * (dPxdP * H + Px * dHdP);
    J->block(1, 0, 1, 3) = fy_ * (dPydP * H + Py * dHdP);
  }
  return true;
}

bool DoubleSphere::BackProject(const Eigen::Vector2i& in_pixel,
                               Eigen::Vector3d& out_point) {

  double mx = (in_pixel[0] - cx_) / fx_;
  double my = (in_pixel[1] - cy_) / fy_;
  double r2 = mx * mx + my * my;

  // check pixels are valid for back projection
  if (alpha_ > 0.5 && r2 > 1 / (2 * alpha_ - 1)) { return false; }

  double mz = (1 - alpha_ * alpha_ * r2) /
              (alpha_ * sqrt(1 - (2 * alpha_ - 1) * r2) + 1 - alpha_);
  double A =
      (mz * eps_ + sqrt(mz * mz + (1 - eps_ * eps_) * r2)) / (mz * mz + r2);
  out_point = Eigen::Vector3d(A * mx, A * my, A * mz - eps_);
  return true;
}

bool DoubleSphere::InProjectionDomain(const Eigen::Vector3d& point) {
  double w1;
  if (alpha_ > 0.5) {
    w1 = (1 - alpha_) / alpha_;
  } else {
    w1 = alpha_ / (1 - alpha_);
  }
  double w2 = (w1 + eps_) / sqrt(2 * w1 * eps_ + eps_ * eps_ + 1);

  double Px = point[0];
  double Py = point[1];
  double Pz = point[2];
  double d1 = sqrt(Px * Px + Py * Py + Pz * Pz);

  // check pixels are valid for projection
  if (point[2] <= -w2 * d1) { return false; }
  return true;
}

} // namespace beam_calibration
