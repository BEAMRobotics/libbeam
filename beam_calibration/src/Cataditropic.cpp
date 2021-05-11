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

beam::opt<Eigen::Vector2d>
    Cataditropic::ProjectPointPrecise(const Eigen::Vector3d& point,
                                      bool& outside_domain) {
  if (point(2) <= 0) {
    outside_domain = true;
    return {};
  }

  Eigen::Vector2d p_u, p_d;
  Eigen::Vector2d out_point;
  // Project points to the normalised plane
  double z = point(2) + xi_ * point.norm();
  p_u << point(0) / z, point(1) / z;

  // Apply distortion
  Eigen::Vector2d d_u;
  this->Distortion(p_u, d_u);
  p_d = p_u + d_u;

  // Apply generalised projection matrix
  out_point << fx_ * p_d(0) + cx_, fy_ * p_d(1) + cy_;

  if (PixelInImage(out_point)) { return out_point; }
  return {};
}

beam::opt<Eigen::Vector2i>
    Cataditropic::ProjectPoint(const Eigen::Vector3d& point,
                               bool& outside_domain) {
  beam::opt<Eigen::Vector2d> pixel = ProjectPointPrecise(point, outside_domain);
  if (pixel.has_value()) {
    Eigen::Vector2i pixel_rounded;
    pixel_rounded << std::round(pixel.value()[0]), std::round(pixel.value()[1]);
    return pixel_rounded;
  }
  return {};
}

beam::opt<Eigen::Vector2i>
    Cataditropic::ProjectPoint(const Eigen::Vector3d& point, Eigen::MatrixXd& J,
                               bool& outside_domain) {
  double norm, inv_denom;
  double dxdmx, dydmx, dxdmy, dydmy;
  norm = point.norm();
  // Project points to the normalised plane
  inv_denom = 1.0 / (point(2) + xi_ * norm);

  // Calculate jacobian
  inv_denom = inv_denom * inv_denom / norm;
  double dudx = inv_denom * (norm * point(2) +
                             xi_ * (point(1) * point(1) + point(2) * point(2)));
  double dvdx = -inv_denom * xi_ * point(0) * point(1);
  double dudy = dvdx;
  double dvdy = inv_denom * (norm * point(2) +
                             xi_ * (point(0) * point(0) + point(2) * point(2)));
  inv_denom = inv_denom * (-xi_ * point(2) - norm); // reuse variable
  double dudz = point(0) * inv_denom;
  double dvdz = point(1) * inv_denom;

  // Make the product of the jacobians
  // and add projection matrix jacobian
  inv_denom = fx_ * (dudx * dxdmx + dvdx * dxdmy); // reuse
  dvdx = fy_ * (dudx * dydmx + dvdx * dydmy);
  dudx = inv_denom;

  inv_denom = fx_ * (dudy * dxdmx + dvdy * dxdmy); // reuse
  dvdy = fy_ * (dudy * dydmx + dvdy * dydmy);
  dudy = inv_denom;

  inv_denom = fx_ * (dudz * dxdmx + dvdz * dxdmy); // reuse
  dvdz = fy_ * (dudz * dydmx + dvdz * dydmy);
  dudz = inv_denom;

  J << dudx, dudy, dudz, dvdx, dvdy, dvdz;
  return ProjectPoint(point, outside_domain);
}

beam::opt<Eigen::Vector3d>
    Cataditropic::BackProject(const Eigen::Vector2i& pixel) {
  Eigen::Vector3d P;
  double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
  double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
  // double lambda;

  // Lift points to normalised plane
  mx_d = m_inv_K11 * pixel(0) + m_inv_K13;
  my_d = m_inv_K22 * pixel(1) + m_inv_K23;

  if (0) {
    // Apply inverse distortion model
    // proposed by Heikkila
    mx2_d = mx_d * mx_d;
    my2_d = my_d * my_d;
    mxy_d = mx_d * my_d;
    rho2_d = mx2_d + my2_d;
    rho4_d = rho2_d * rho2_d;
    radDist_d = k1_ * rho2_d + k2_ * rho4_d;
    Dx_d = mx_d * radDist_d + p2_ * (rho2_d + 2 * mx2_d) + 2 * p1_ * mxy_d;
    Dy_d = my_d * radDist_d + p1_ * (rho2_d + 2 * my2_d) + 2 * p2_ * mxy_d;
    inv_denom_d = 1 / (1 + 4 * k1_ * rho2_d + 6 * k2_ * rho4_d +
                       8 * p1_ * my_d + 8 * p2_ * mx_d);

    mx_u = mx_d - inv_denom_d * Dx_d;
    my_u = my_d - inv_denom_d * Dy_d;
  } else {
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
  }

  // Obtain a projective ray
  if (xi_ == 1.0) {
    P << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
  } else {
    // Reuse variable
    rho2_d = mx_u * mx_u + my_u * my_u;
    P << mx_u, my_u,
        1.0 - xi_ * (rho2_d + 1.0) /
                  (xi_ + sqrt(1.0 + (1.0 - xi_ * xi_) * rho2_d));
  }
  return P;
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

void Cataditropic::Distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u,
                              Eigen::Matrix2d& J) const {
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);

  double dxdmx = 1.0 + rad_dist_u + k1_ * 2.0 * mx2_u +
                 k2_ * rho2_u * 4.0 * mx2_u + 2.0 * p1_ * p_u(1) +
                 6.0 * p2_ * p_u(0);
  double dydmx = k1_ * 2.0 * p_u(0) * p_u(1) +
                 k2_ * 4.0 * rho2_u * p_u(0) * p_u(1) + p1_ * 2.0 * p_u(0) +
                 2.0 * p2_ * p_u(1);
  double dxdmy = dydmx;
  double dydmy = 1.0 + rad_dist_u + k1_ * 2.0 * my2_u +
                 k2_ * rho2_u * 4.0 * my2_u + 6.0 * p1_ * p_u(1) +
                 2.0 * p2_ * p_u(0);

  J << dxdmx, dxdmy, dydmx, dydmy;
}

} // namespace beam_calibration
