#include "beam_calibration/Cataditropic.h"

namespace beam_calibration {

Cataditropic::Cataditropic(const std::string& file_path) {
  type_ = CameraType::CATADITROPIC;
  LoadJSON(file_path);
  fx_ = intrinsics_[0];
  fy_ = intrinsics_[1];
  cx_ = intrinsics_[2];
  cy_ = intrinsics_[3];
  xi_ = intrinsics_[4];
  k1_ = intrinsics_[5];
  k2_ = intrinsics_[6];
  p1_ = intrinsics_[7];
  p2_ = intrinsics_[8];

  m_inv_K11 = 1.0 / fx_;
  m_inv_K13 = -cx_ / fx_;
  m_inv_K22 = 1.0 / fy_;
  m_inv_K23 = -cy_ / fy_;
}

std::shared_ptr<CameraModel> Cataditropic::Clone() {
  std::shared_ptr<Cataditropic> clone = std::make_shared<Cataditropic>();
  clone->type_ = CameraType::CATADITROPIC;
  clone->SetIntrinsics(this->GetIntrinsics());
  clone->SetImageDims(this->GetHeight(), this->GetWidth());
  clone->SetFrameID(this->GetFrameID());
  clone->SetCalibrationDate(this->GetCalibrationDate());
  return clone;
}

bool Cataditropic::ProjectPoint(const Eigen::Vector3d& in_point,
                                Eigen::Vector2d& out_pixel,
                                bool& in_image_plane, std::shared_ptr<Eigen::MatrixXd> J) {
  if (in_point(2) < 0) { return false; }
  Eigen::Vector2d p_u, p_d;
  // Project points to the normalised plane
  double z = in_point(2) + xi_ * in_point.norm();
  p_u << in_point(0) / z, in_point(1) / z;

  // Apply distortion
  Eigen::Vector2d d_u;
  this->Distortion(p_u, d_u);
  p_d = p_u + d_u;

  // Apply generalised projection matrix
  out_pixel << fx_ * p_d(0) + cx_, fy_ * p_d(1) + cy_;
  if (PixelInImage(out_pixel)) {
    in_image_plane = true;
  } else {
    in_image_plane = false;
  }
  if (J) { BEAM_WARN("Jacobian not yet implemented."); }
  return true;
}

bool Cataditropic::BackProject(const Eigen::Vector2i& in_pixel,
                               Eigen::Vector3d& out_point) {
  if (!PixelInImage(in_pixel)) { return false; }
  double mx_d, my_d,  mx_u, my_u, rho2_d;
  // double lambda;

  // Lift points to normalised plane
  mx_d = m_inv_K11 * in_pixel(0) + m_inv_K13;
  my_d = m_inv_K22 * in_pixel(1) + m_inv_K23;

  // Recursive distortion model
  int n = 8;
  Eigen::Vector2d d_u;
  this->Distortion(Eigen::Vector2d(mx_d, my_d), d_u);
  // Approximate value
  mx_u = mx_d - d_u(0);
  my_u = my_d - d_u(1);

  for (int i = 1; i < n; ++i) {
    this->Distortion(Eigen::Vector2d(mx_u, my_u), d_u);
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);
  }

  // Obtain a projective ray
  if (xi_ == 1.0) {
    out_point << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
  } else {
    // Reuse variable
    rho2_d = mx_u * mx_u + my_u * my_u;
    out_point << mx_u, my_u,
        1.0 - xi_ * (rho2_d + 1.0) /
                  (xi_ + sqrt(1.0 + (1.0 - xi_ * xi_) * rho2_d));
  }
  return true;
}

void Cataditropic::Distortion(const Eigen::Vector2d& p_u,
                              Eigen::Vector2d& d_u) const {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);
}

} // namespace beam_calibration
