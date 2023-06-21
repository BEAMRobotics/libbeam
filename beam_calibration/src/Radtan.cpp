#include "beam_calibration/Radtan.h"

namespace beam_calibration {

Radtan::Radtan(const std::string& file_path) {
  type_ = CameraType::RADTAN;
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

Radtan::Radtan(uint32_t image_height, uint32_t image_width,
               const Eigen::Matrix<double, 8, 1>& intrinsics) {
  type_ = CameraType::RADTAN;
  image_height_ = image_height;
  image_width_ = image_width;
  intrinsics_ = intrinsics;
  fx_ = intrinsics_[0];
  fy_ = intrinsics_[1];
  cx_ = intrinsics_[2];
  cy_ = intrinsics_[3];
  k1_ = intrinsics_[4];
  k2_ = intrinsics_[5];
  p1_ = intrinsics_[6];
  p2_ = intrinsics_[7];
}

std::shared_ptr<CameraModel> Radtan::Clone() {
  std::shared_ptr<Radtan> clone = std::make_shared<Radtan>();
  clone->type_ = CameraType::RADTAN;
  clone->SetIntrinsics(this->GetIntrinsics());
  clone->SetImageDims(this->GetHeight(), this->GetWidth());
  clone->SetFrameID(this->GetFrameID());
  clone->SetCalibrationDate(this->GetCalibrationDate());
  return clone;
}

bool Radtan::ProjectPoint(const Eigen::Vector3d& in_point,
                          Eigen::Vector2d& out_pixel, bool& in_image_plane,
                          std::shared_ptr<Eigen::MatrixXd> J) {
  // check domain of point
  if (!this->InProjectionDomain(in_point)) { return false; }

  // Project point
  const double x = in_point[0], y = in_point[1], z = in_point[2];
  const double rz = 1.0 / z;
  out_pixel << (x * rz), (y * rz);
  Eigen::Vector2d tmp;
  tmp << (x * rz), (y * rz);
  // Distort point using radtan distortion model
  if (k1_ > 0 && k2_ > 0 && p1_ > 0 && p2_ > 0) {
    out_pixel = DistortPixel(out_pixel);
  }
  double xx = out_pixel[0], yy = out_pixel[1];
  out_pixel[0] = (fx_ * xx + cx_);
  out_pixel[1] = (fy_ * yy + cy_);

  if (PixelInImage(out_pixel)) {
    in_image_plane = true;
  } else {
    in_image_plane = false;
  }

  if (J) {
    Eigen::MatrixXd dGdH = Eigen::MatrixXd(2, 2);
    Eigen::MatrixXd dHdF = Eigen::MatrixXd(2, 2);
    Eigen::MatrixXd dFdP = Eigen::MatrixXd(2, 3);
    dGdH(0, 0) = fx_;
    dGdH(1, 0) = 0;
    dGdH(0, 1) = 0;
    dGdH(1, 1) = fy_;
    dHdF = ComputeDistortionJacobian(tmp);
    dFdP(0, 0) = 1 / z;
    dFdP(1, 0) = 0;
    dFdP(0, 1) = 0;
    dFdP(1, 1) = 1 / z;
    dFdP(0, 2) = -x / (z * z);
    dFdP(1, 2) = -y / (z * z);
    Eigen::MatrixXd result = dGdH * dHdF * dFdP;
    *J = result;
  }
  return true;
}

bool Radtan::BackProject(const Eigen::Vector2i& in_pixel,
                         Eigen::Vector3d& out_point) {
  cv::Point2f p(in_pixel[0], in_pixel[1]);
  std::vector<cv::Point2f> src = {p};
  std::vector<cv::Point2f> dst;

  Eigen::Matrix3d camera_matrix;
  camera_matrix << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(camera_matrix, K);

  if (k1_ > 0 && k2_ > 0 && p1_ > 0 && p2_ > 0) {
    std::vector<double> dist_coeffs = {k1_, k2_, p1_, p2_};
    // undistort points automatically normalizes the output
    cv::undistortPoints(src, dst, K, dist_coeffs);
    out_point << dst[0].x, dst[0].y, 1;
  } else {
    out_point << (p.x - cx_) / fx_, (p.y - cy_) / fy_, 1;
  }

  return true;
}

bool Radtan::InProjectionDomain(const Eigen::Vector3d& point) {
  // check pixels are valid for projection
  if (point[2] <= 0) { return false; }
  return true;
}

void Radtan::UndistortImage(const cv::Mat& image_input, cv::Mat& image_output) {
  uint32_t height = image_input.rows, width = image_input.cols;
  Eigen::Matrix3d camera_matrix;
  camera_matrix << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
  cv::Mat K(3, 3, CV_32F);
  cv::eigen2cv(camera_matrix, K);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  // convert eigen to cv mat
  Eigen::VectorXd new_coeffs(5);
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