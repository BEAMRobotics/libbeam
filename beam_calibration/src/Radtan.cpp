#include "beam_calibration/Radtan.h"

namespace beam_calibration {

Radtan::Radtan(const std::string& file_path) {
  type_ = CameraType::RADTAN;
  BEAM_INFO("Loading file: {}", file_path);
  LoadJSON(file_path);
  fx_ = intrinsics_[0];
  fy_ = intrinsics_[1];
  cx_ = intrinsics_[2];
  cy_ = intrinsics_[3];
  k1_ = intrinsics_[4];
  k2_ = intrinsics_[5];
  p1_ = intrinsics_[6];
  p2_ = intrinsics_[7];
}

opt<Eigen::Vector2i> Radtan::ProjectPoint(const Eigen::Vector3d& point) {
  // check if point is behind image plane
  if (point[2] < 0) { return {}; }

  Eigen::Vector2d out_point;
  // Project point
  const double x = point[0], y = point[1], z = point[2];
  const double rz = 1.0 / z;
  out_point << (x * rz), (y * rz);
  // Distort point using radtan distortion model
  out_point = this->DistortPixel(out_point);
  double xx = out_point[0], yy = out_point[1];
  out_point[0] = (fx_ * xx + cx_);
  out_point[1] = (fy_ * yy + cy_);

  Eigen::Vector2i coords;
  coords << std::round(out_point[0]), std::round(out_point[1]);
  if (PixelInImage(coords)) { return coords; }
  return {};
}

opt<Eigen::Vector2i> Radtan::ProjectPoint(const Eigen::Vector3d& point,
                                          Eigen::MatrixXd& J) {
  Eigen::Vector2d tmp;
  const double x = point[0], y = point[1], z = point[2];
  const double rz = 1.0 / z;
  tmp << (x * rz), (y * rz);
  dHdF = this->ComputeDistortionJacobian(tmp);

  const double rz = 1 / z;
  const double rz2 = rz * rz;
  const double duf_dx = fx_ * dHdF(0, 0) * rz;
  const double duf_dy = fx_ * dHdF(0, 1) * rz;
  const double duf_dz = -fx_ * (x * dHdF(0, 0) + y * dHdF(0, 1)) * rz2;
  const double dvf_dx = fy_ * dHdF(1, 0) * rz;
  const double dvf_dy = fy_ * dHdF(1, 1) * rz;
  const double dvf_dz = -fy_ * (x * dHdF(1, 0) + y * dHdF(1, 1)) * rz2;
  J << duf_dx, duf_dy, duf_dz, dvf_dx, dvf_dy, dvf_dz;

  return ProjectPoint(point);
}

opt<Eigen::Vector3d> Radtan::BackProject(const Eigen::Vector2i& pixel) {
  Eigen::Vector3d out_point;
  Eigen::Vector2d kp;
  kp[0] = (pixel[0] - cx_) / fx_;
  kp[1] = (pixel[1] - cy_) / fy_;
  Eigen::Vector2d undistorted = this->UndistortPixel(kp);
  out_point << undistorted[0], (undistorted[1]), 1;
  out_point.normalize();
  return out_point;
}

void Radtan::UndistortImage(const cv::Mat& image_input, cv::Mat& image_output) {
  uint32_t height = image_input.rows, width = image_input.cols;
  Eigen::Matrix3d camera_matrix;
  camera_matrix << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(camera_matrix, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // convert eigen to cv mat
  beam::VecX new_coeffs(5);
  new_coeffs << k1_, k2_, 0, p1_, p2_;
  cv::Mat D(1, 5, CV_8UC1);
  cv::eigen2cv(new_coeffs, D);
  // Undistort image
  cv::Mat map1, map2;
  cv::Size img_size = cv::Size(width, height);
  cv::initUndistortRectifyMap(K, D, R, K, img_size, CV_32FC1, map1, map2);
  cv::remap(image_input, image_output, map1, map2, 1);
}

Eigen::Vector2d Radtan::DistortPixel(const Eigen::Vector2d& pixel) {
  Eigen::Vector2d coords;
  double x = pixel[0], y = pixel[1];
  double xx, yy;
  double mx2_u = x * x;
  double my2_u = y * y;
  double mxy_u = x * y;
  double rho2_u = mx2_u + my2_u;
  double rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
  xx = x + (x * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u));
  yy = y + (y * rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u));
  coords << xx, yy;
  return coords;
}

Eigen::Vector2d Radtan::UndistortPixel(const Eigen::Vector2d& pixel) {
  constexpr int n = 200; // Max. number of iterations
  Eigen::Vector2d y = pixel;
  Eigen::Vector2d ybar = y;
  Eigen::Matrix2d F;
  Eigen::Vector2d y_tmp;
  // Handle special case around image center.
  if (y.squaredNorm() < 1e-6) return y; // Point remains unchanged.
  for (int i = 0; i < n; ++i) {
    y_tmp = this->DistortPixel(ybar);
    F = this->ComputeDistortionJacobian(ybar);
    Eigen::Vector2d e(y - y_tmp);
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;
    ybar += du;
  }
  y = ybar;
  return y;
}

Eigen::Matrix2d
    Radtan::ComputeDistortionJacobian(const Eigen::Vector2d& pixel) {
  Eigen::Matrix2d out_jacobian;
  double x = pixel[0];
  double y = pixel[1];
  double x2 = x * x;
  double y2 = y * y;
  double xy = x * y;
  double r2 = x2 + y2;
  double rad_dist_u = k1_ * r2 + k2_ * r2 * r2;
  const double duf_du = 1.0 + rad_dist_u + 2.0 * k1_ * x2 +
                        4.0 * k2_ * r2 * x2 + 2.0 * p1_ * y + 6.0 * p2_ * x;
  const double duf_dv =
      2.0 * k1_ * xy + 4.0 * k2_ * r2 * xy + 2.0 * p1_ * x + 2.0 * p2_ * y;
  const double dvf_du = duf_dv;
  const double dvf_dv = 1.0 + rad_dist_u + 2.0 * k1_ * y2 +
                        4.0 * k2_ * r2 * y2 + 2.0 * p2_ * x + 6.0 * p1_ * y;
  out_jacobian << duf_du, duf_dv, dvf_du, dvf_dv;
  return out_jacobian;
}

} // namespace beam_calibration
